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
float KP_L = 0.4;
float KI_L = 0.1;
float KD_L = 0.0;

float KP_R = 0.45;
float KI_R = 0.12;
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
void isr_enc_l() {
  // ★ 왼쪽 채널만 모터 극성과 엔코더 부호가 반대로 배선되어 있어(실측 확인됨),
  //   PID가 자기 자신을 가속시키는 양성 피드백 폭주를 일으켰음.
  //   물리적 재배선 대신 소프트웨어에서 부호만 반전해서 보정.
  bool b = digitalRead(PIN_ENC_L_B);
  if (b) enc_l_count--;
  else enc_l_count++;
}

void isr_enc_r() {
  bool b = digitalRead(PIN_ENC_R_B);
  if (b) enc_r_count++;
  else enc_r_count--;
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

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isr_enc_l, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isr_enc_r, RISING);

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
