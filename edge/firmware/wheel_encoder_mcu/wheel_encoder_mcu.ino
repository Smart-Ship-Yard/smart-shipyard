/*
  wheel_encoder_mcu.ino  (Arduino MEGA + Cytron MDD10A 버전)
  --------------------------------------------------------------
  [2026 업데이트] 동료가 실제 배선/테스트한 핀 구성을 그대로 반영함.
  모터드라이버는 Cytron MDD10A (PWM+DIR 모드 사용, MODE 핀이 PWM+DIR
  모드로 점퍼/배선되어 있어야 함 - 하드웨어 확인 필요, 코드와 무관).

  MDD10A 핀 구성 참고:
    VM, GND    : 모터 전원 (배터리 직결, Arduino와 무관)
    M1A/M1B    : 모터1 출력, M2A/M2B: 모터2 출력
    DIR1/PWM1  : 모터1 제어 입력 (아래 D2/D3)
    DIR2/PWM2  : 모터2 제어 입력 (아래 D4/D5)
    5V         : 로직 전원 입력 - Arduino 5V에 연결 (활성화 핀 아님, 필수 전원)
    MODE       : 제어 모드 선택 점퍼, PWM+DIR 모드로 설정되어 있어야 함

  ---- 핀 배치 (동료 테스트 코드 기준, 검증된 배선 그대로 사용) ----
  D2  - 모터1 PWM (속도, MDD10A PWM1)
  D3  - 모터1 DIR (방향, MDD10A DIR1)
  D4  - 모터2 PWM (속도, MDD10A PWM2)
  D5  - 모터2 DIR (방향, MDD10A DIR2)
  D18 - 모터1 엔코더 A상 (인터럽트)
  D19 - 모터1 엔코더 B상 (방향 판별, 일반 읽기)
  D20 - 모터2 엔코더 A상 (인터럽트)
  D21 - 모터2 엔코더 B상 (방향 판별, 일반 읽기)

  모터1 = 왼쪽, 모터2 = 오른쪽으로 가정 (반대라면 track_width 부호가 아니라
  ROS2 쪽 wheel_odom_node의 좌우 매핑만 바꾸면 됨 - 이 펌웨어는 그대로 둘 것).

  ---- 시리얼 프로토콜 (115200bps, 줄바꿈 구분) ----
  Jetson -> Arduino:  "V,<left_ticks_per_sec>,<right_ticks_per_sec>\n"
  Arduino -> Jetson:  "E,<left_delta_ticks>,<right_delta_ticks>,<dt_ms>\n"
  (wheel_odom_node.py와 반드시 일치해야 하는 부분, 임의로 바꾸지 말 것)
*/

#include <Arduino.h>

// ---------------- 핀 정의 (동료 테스트 배선 그대로) ----------------
const uint8_t PIN_L_PWM = 2;
const uint8_t PIN_L_DIR = 3;
const uint8_t PIN_R_PWM = 4;
const uint8_t PIN_R_DIR = 5;

const uint8_t PIN_ENC_L_A = 18;
const uint8_t PIN_ENC_L_B = 19;
const uint8_t PIN_ENC_R_A = 20;
const uint8_t PIN_ENC_R_B = 21;

// ---------------- 타이밍 ----------------
const unsigned long REPORT_INTERVAL_MS = 20;   // 엔코더 보고 주기 (50Hz)
const unsigned long CMD_TIMEOUT_MS = 500;      // 이 시간 동안 명령 없으면 정지 (안전 워치독)

// ---------------- PID 게인 (튜닝 필요, 시작값) ----------------
// ★ 좌우 모터가 가속/감속 순간(과도응답)에 서로 다르게 반응해서
//   출발 직후와 정지 직전에 짧게 휘는 현상이 실측으로 확인됨.
//   trim(정상상태 배율)으로는 해결이 안 되는 문제라 좌우 게인을 분리함.
//   시작은 동일값으로 두고, 실측하며 KP_R을 올리거나 KP_L을 낮추는 식으로 튜닝.
// ★ 2026-08-18 x4 쿼드러처 전환에 맞춰 게인을 1/4 로 낮춤.
//   틱 수가 4배가 되면 같은 속도 오차가 4배 큰 숫자로 들어온다.
//   게인을 그대로 두면 실효 게인이 4배가 되어 훨씬 공격적으로 변한다.
//   1/4 을 곱해 이전과 같은 응답을 유지한다.
//   (적분 제한 ±2000 은 그대로 두었다. ki 가 1/4 이 되었으므로 적분항이
//    낼 수 있는 최대 출력이 PWM 200 -> 50 으로 줄어, 와인드업 위험도 함께 낮아진다.)
float KP_L = 0.1;
float KI_L = 0.025;
float KD_L = 0.0;

float KP_R = 0.1125;
float KI_R = 0.03;
float KD_R = 0.0;
const int PWM_MAX = 255;

// ---------------- 엔코더 카운트 (인터럽트에서만 갱신) ----------------
volatile long enc_l_count = 0;
volatile long enc_r_count = 0;

// ---------------- 속도 제어 상태 ----------------
long target_l_tps = 0;

// ★ 기동 감쇠(startup damping): 정지 상태에서 새로 출발하는 순간,
//   오른쪽 모터가 왼쪽보다 먼저/세게 반응하는 과도응답 차이가 실측으로
//   확인됨 (teleop 출발 직후 왼쪽으로 꺾이는 원인). 출발 직후 짧은 시간
//   동안만 오른쪽 PWM 출력에 계수를 곱해 그 차이를 미리 상쇄한다.
unsigned long startup_time_ms = 0;      // 목표가 0에서 0이 아닌 값으로 바뀐 시각
bool was_stopped = true;                // 직전까지 정지 상태였는지
const unsigned long STARTUP_DAMP_DURATION_MS = 400;  // 이 시간 동안만 감쇠 적용
const float STARTUP_DAMP_FACTOR_R = 0.85f;           // 오른쪽 출력을 85%로 제한
long target_r_tps = 0;
float integral_l = 0.0f, integral_r = 0.0f;
long prev_err_l = 0, prev_err_r = 0;
unsigned long last_cmd_time = 0;

// ★ 하드 스톱 (2026-08-17). true 면 PID 를 건너뛰고 PWM 을 직접 0 으로 쓴다.
//   엔코더가 거짓말을 해도 모터를 확실히 멈추기 위한 마지막 방어선.
//   켜지는 경우 두 가지:
//     ① 워치독 (CMD_TIMEOUT_MS 동안 명령 없음)
//     ② 상위(ROS)에서 "H,1" 명령을 받았을 때 — 폭주 감지기가 쓴다
//   해제는 "H,0" 또는 정상 속도명령("V,...") 수신이다.
bool hard_stop = false;

// ---------------- 보고 상태 ----------------
long last_report_enc_l = 0;
long last_report_enc_r = 0;
unsigned long last_report_time = 0;

// ---------------- 시리얼 수신 버퍼 ----------------
String rx_line = "";

// ==========================================================
// ★★ 2026-08-18 — 반쪽 디코딩 -> 정식 x4 쿼드러처 상태머신 ★★
//
// 이전 방식: A상 상승엣지에서만 인터럽트를 걸고, 그 순간 B상을 한 번 읽어
//            방향을 결정했다. 간단하지만 **노이즈 한 번에 방향이 통째로 뒤집힌다.**
//            A상에 가짜 엣지가 뜨면 그때 읽은 B값이 곧 방향이 되기 때문이다.
//            그 결과 PID 부호가 역전되어 모터가 자가 가속하는 폭주가
//            2026-08-17~18 에 다섯 번 발생했다 (특히 회전 시 = 전류/노이즈가 큰 순간).
//
// 새 방식: A·B 양쪽의 모든 엣지(CHANGE)에서 인터럽트를 걸고, 직전 상태와
//          현재 상태의 쌍으로 방향을 판정한다.
//            - 정상 전이면 +1 또는 -1
//            - **있을 수 없는 전이(두 비트가 동시에 바뀜)는 0** -> 노이즈를 버린다
//          가짜 엣지가 하나 껴도 다음 엣지에서 되돌아와 +1/-1 로 상쇄되므로,
//          부호가 통째로 뒤집히지 않는다. 이것이 폭주의 뿌리를 끊는다.
//
// 덤: 한 바퀴에 세는 엣지가 4배가 되어 해상도가 4배 올라간다.
//     ticks_per_rev 330 -> 1320 (스펙값과 일치. 설치가이드 4장의
//     "왜 CPR 이 1/4 인가" 수수께끼가 바로 이 반쪽 디코딩 때문이었다).
//
// ★ 좌우 부호가 다른 것은 정상이다 — 두 모터가 좌우 대칭으로 마주보게
//   달려 있어 전진 시 물리적 회전 방향이 반대이기 때문. 아래 상수로 분리해
//   두었으니, 부호가 반대로 나오면 해당 상수만 뒤집으면 된다.
// ★ 2026-08-18 실측으로 확정: 손으로 각 바퀴를 "전진 방향"으로 돌려
//   /wheel/raw_ticks 의 부호를 보고 정한 값이다 (양쪽 다 뒤집혀 있었다 —
//   x4 전이표의 방향 규약이 이전 반쪽 디코딩과 달라서 생긴 일이며 정상이다).
//   부호가 반대로 나오면 여기만 뒤집으면 된다. 다른 곳은 손댈 필요 없다.
const int8_t ENC_L_SIGN = +1;
const int8_t ENC_R_SIGN = -1;

// 쿼드러처 전이표. 인덱스 = (직전상태 << 2) | 현재상태,  상태 = (A << 1) | B
// 0 인 칸이 "있을 수 없는 전이" = 노이즈로 보고 버리는 자리다.
const int8_t QEM[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

volatile uint8_t enc_l_state = 0;
volatile uint8_t enc_r_state = 0;

void isr_enc_l() {
  uint8_t s = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
  enc_l_count += ENC_L_SIGN * QEM[(enc_l_state << 2) | s];
  enc_l_state = s;
}

void isr_enc_r() {
  uint8_t s = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);
  enc_r_count += ENC_R_SIGN * QEM[(enc_r_state << 2) | s];
  enc_r_state = s;
}

// ==========================================================
// PWM+DIR 방식 (MDD10A 검증 배선): pwm이 음수면 DIR을 반대로, 크기만 analogWrite
void setMotorPWM(int pwm_pin, int dir_pin, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(dir_pin, HIGH);
    analogWrite(pwm_pin, pwm);
  } else {
    digitalWrite(dir_pin, LOW);
    analogWrite(pwm_pin, -pwm);
  }
}

void stopAllMotors() {
  setMotorPWM(PIN_L_PWM, PIN_L_DIR, 0);
  setMotorPWM(PIN_R_PWM, PIN_R_DIR, 0);
  integral_l = integral_r = 0.0f;
  prev_err_l = prev_err_r = 0;
}

// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  pinMode(PIN_L_PWM, OUTPUT);
  pinMode(PIN_L_DIR, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);
  pinMode(PIN_R_DIR, OUTPUT);

  stopAllMotors();

  // ★ 초기 상태를 읽어 둔다. 안 하면 첫 인터럽트에서 엉뚱한 전이로 잡힌다.
  enc_l_state = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
  enc_r_state = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);

  // ★ A·B 양쪽, 상승·하강 모두(CHANGE) — x4 쿼드러처.
  //   Mega 의 18~21 번은 넷 다 인터럽트 가능 핀이라 그대로 쓸 수 있다.
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isr_enc_l, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isr_enc_l, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isr_enc_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isr_enc_r, CHANGE);

  last_report_time = millis();
  last_cmd_time = millis();
}

// ==========================================================
void handleLine(const String& line) {
  if (!line.startsWith("V,")) return;

  int comma1 = line.indexOf(',', 2);
  if (comma1 < 0) return;

  long l = line.substring(2, comma1).toInt();
  long r = line.substring(comma1 + 1).toInt();

  bool is_now_moving = (l != 0 || r != 0);
  if (was_stopped && is_now_moving) {
    startup_time_ms = millis();   // 정지->출발 전환 시점 기록
  }
  was_stopped = !is_now_moving;

  target_l_tps = l;
  target_r_tps = r;
  last_cmd_time = millis();

  // ★ hard_stop 해제는 "0이 아닌 속도 명령"이 왔을 때만 (2026-08-17).
  //   V,0,0 으로도 풀리게 하면, 상위가 0을 계속 재전송하는 동안 하드스톱이
  //   즉시 풀려 PID 가 되살아난다. 그러면 엔코더가 거짓말하는 상황에서
  //   또 폭주한다 — 하드스톱을 넣은 의미가 없어진다.
  if (is_now_moving) {
    hard_stop = false;
  }

  // ★ 목표 속도가 0(정지 명령)이면, 이전에 쌓인 PID 적분값을 즉시 비운다.
  //   안 그러면 오래 돈 뒤 갑자기 멈추라는 명령이 와도, 남아있던 integral
  //   때문에 몇 초간 계속 낮은 속도로 밀리는 현상이 생긴다
  //   (관성이 아니라 PID 잔류 출력임 - 바퀴를 띄워도 계속 도는 것으로 확인됨).
  if (target_l_tps == 0 && target_r_tps == 0) {
    integral_l = 0.0f;
    integral_r = 0.0f;
    prev_err_l = 0;
    prev_err_r = 0;
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      rx_line.trim();
      if (rx_line.length() > 0) handleLine(rx_line);
      rx_line = "";
    } else if (c != '\r') {
      rx_line += c;
      if (rx_line.length() > 64) rx_line = "";
    }
  }
}

// ==========================================================
int pidUpdate(long target_tps, long measured_tps, float& integral, long& prev_err,
              float dt_s, float kp, float ki, float kd) {
  long err = target_tps - measured_tps;
  integral += err * dt_s;
  integral = constrain(integral, -2000.0f, 2000.0f);
  float deriv = (dt_s > 0.0f) ? (err - prev_err) / dt_s : 0.0f;
  prev_err = err;

  float output = kp * err + ki * integral + kd * deriv;
  return (int)constrain(output, -255.0f, 255.0f);
}

void controlLoop(float dt_s, long measured_l_tps, long measured_r_tps) {
  // ★★ 2026-08-17 안전 수정 — PID 를 우회하는 하드 스톱 ★★
  //
  // 왜 필요한가 (실제로 두 번 겪은 사고):
  //   왼쪽 엔코더가 배선 문제로 방향을 거꾸로 보고하면(바퀴는 앞으로 도는데
  //   -1250 tps 로 보고), PID 는 이렇게 반응한다.
  //       목표 0, 측정 -1250 -> 오차 +1250 -> 비례항만으로 0.4*1250=500
  //       -> PWM 255 (최대). 바퀴는 더 빨리 돌고 측정은 더 큰 음수가 된다.
  //   **정지 명령(목표 0)으로도, 워치독으로도 멈출 수 없다.** 워치독은
  //   target 을 0 으로 바꿀 뿐이고 PID 는 계속 돌기 때문이다. 실제로 사람이
  //   배터리를 뽑기 전까지 멈추지 않았다.
  //
  // 그래서 hard_stop 이 걸리면 PID 계산 자체를 건너뛰고 PWM 을 0으로 직접 쓴다.
  // 센서가 거짓말을 해도 모터는 확실히 멈춘다 — 이것이 마지막 방어선이다.
  if (hard_stop) {
    integral_l = 0.0f;  integral_r = 0.0f;
    prev_err_l = 0;     prev_err_r = 0;
    setMotorPWM(PIN_L_PWM, PIN_L_DIR, 0);
    setMotorPWM(PIN_R_PWM, PIN_R_DIR, 0);
    return;
  }

  int pwm_l = pidUpdate(target_l_tps, measured_l_tps, integral_l, prev_err_l,
                        dt_s, KP_L, KI_L, KD_L);
  int pwm_r = pidUpdate(target_r_tps, measured_r_tps, integral_r, prev_err_r,
                        dt_s, KP_R, KI_R, KD_R);

  // ★ 기동 감쇠 적용: 출발 후 STARTUP_DAMP_DURATION_MS 이내라면
  //   오른쪽 출력만 일시적으로 낮춘다 (좌회전 방향 편향 상쇄).
  if (millis() - startup_time_ms < STARTUP_DAMP_DURATION_MS) {
    pwm_r = (int)(pwm_r * STARTUP_DAMP_FACTOR_R);
  }

  setMotorPWM(PIN_L_PWM, PIN_L_DIR, pwm_l);
  setMotorPWM(PIN_R_PWM, PIN_R_DIR, pwm_r);
}

// ==========================================================
void loop() {
  readSerial();

  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    // ★ 워치독 발동 시에도 handleLine과 동일하게 적분값을 반드시 리셋해야 한다.
    //   ROS2가 꺼져 있으면 handleLine 자체가 호출되지 않으므로(새 명령이 안 옴),
    //   여기서 리셋 안 하면 들었다 놓는 등의 충격으로 생긴 잔여 오차가
    //   PID 적분에 쌓인 채 수 초간 스스로 진정될 때까지 계속 미세하게 떨린다.
    target_l_tps = 0;
    target_r_tps = 0;
    integral_l = 0.0f;
    integral_r = 0.0f;
    prev_err_l = 0;
    prev_err_r = 0;
    // ★ 목표를 0 으로 두는 것만으로는 부족하다. 엔코더가 거짓 값을 주면
    //   PID 가 그 오차를 없애려고 계속 구동한다. PWM 을 직접 끊는다.
    hard_stop = true;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - last_report_time;

  if (elapsed >= REPORT_INTERVAL_MS) {
    noInterrupts();
    long cur_l = enc_l_count;
    long cur_r = enc_r_count;
    interrupts();

    long delta_l = cur_l - last_report_enc_l;
    long delta_r = cur_r - last_report_enc_r;
    float dt_s = elapsed / 1000.0f;

    long measured_l_tps = (long)(delta_l / dt_s);
    long measured_r_tps = (long)(delta_r / dt_s);
    controlLoop(dt_s, measured_l_tps, measured_r_tps);

    Serial.print("E,");
    Serial.print(delta_l);
    Serial.print(",");
    Serial.print(delta_r);
    Serial.print(",");
    Serial.println(elapsed);

    last_report_enc_l = cur_l;
    last_report_enc_r = cur_r;
    last_report_time = now;
  }
}
