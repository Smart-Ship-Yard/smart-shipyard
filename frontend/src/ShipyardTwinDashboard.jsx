import React, { useRef, useEffect, useState, useCallback, useMemo } from "react";
import * as THREE from "three";

/* ============================================================================
 * 스마트 조선소 디지털 트윈 관제 대시보드
 * Frontend & 3D Engineer 담당분 (고명재)
 *
 * 구현 요구사항
 *  1) React.js 기반 관제 대시보드 UI/UX
 *  2) Three.js(WebGL) 기반 3D 선박 블록 렌더링
 *  3) 서버 좌표 → 3D 공간 매핑 및 Red Alert Ping 시각화
 *  4) Click & View 영상 팝업
 *
 * 백엔드(FastAPI/WebSocket)가 아직 없으므로, 동일한 이벤트 스키마를 따르는
 * 모의 피드(MockEventSource)로 데이터를 주입한다. 실제 연동 시
 * connectEventSource() 한 곳만 교체하면 된다.
 * ========================================================================== */

/* ---------------------------------------------------------------------------
 * -1. 실제 연동 스위치 — 여기 두 값만 바꾸면 가짜↔진짜가 전환된다.
 *     서버 IP는 이정기님(백엔드)에게 물어봐서 채운다 (hostname -I 로 확인한 값).
 * ------------------------------------------------------------------------- */
const USE_REAL_BACKEND = true;   // false로 두면 예전처럼 2.2초마다 가짜 이벤트
const USE_REAL_VIDEO = true;     // false로 두면 예전처럼 Canvas 가짜 CCTV
const SERVER_HOST = "192.168.0.5:8000"; // ← 백엔드 서버 IP:포트로 교체 (예: 192.168.0.42:8000)
const EVENT_WS_URL = `ws://${SERVER_HOST}/ws/frontend`;
// 감지 순간 사진. 서버가 backend/snapshots/ 에 저장해두고 "/snapshots/<해시>.jpg"
// 같은 경로만 알려주므로, 앞에 서버 주소를 붙여 완전한 URL 로 만든다.
// (SERVER_HOST 하나만 바꾸면 영상·이벤트·사진이 함께 따라오게 하려는 것)
const SNAPSHOT_BASE_URL = `http://${SERVER_HOST}`;

/* 영상은 백엔드(FastAPI)를 거치지 않지만, 중앙 미디어 서버는 거친다.
 * 젯슨이 H.264 로 한 번만 인코딩해 서버의 mediamtx 로 밀어올리고(RTSP push),
 * 브라우저는 그 mediamtx 에 WebRTC 로 붙는다.
 *
 *   젯슨 ──H.264 1개, RTSP push──▶ mediamtx(서버) ──WebRTC──▶ 브라우저 N명
 *
 * ★ P2P 가 아니다. mediamtx 가 미디어 서버이고 브라우저가 거기 접속하는
 *   client-server 구조다. WebRTC 를 쓸 뿐 양쪽이 대등하지 않다.
 *
 * ★ 예전에 있던 "백엔드가 JPEG 를 중계하는" 모드는 2026-08-28 삭제했다.
 *   젯슨이 그 창구로 프레임을 보낸 적이 한 번도 없어서 실제로는 동작하지 않는
 *   폴백이었고, 남겨두면 "쓸 수 있는 선택지"로 오해를 산다.
 *   왜 백엔드로 나르지 않는지는 docs/interface.md ⑤ 참조. */
// 서버 노트북(고정 IP)에 떠 있는 mediamtx 의 WebRTC 재생 페이지.
// 젯슨의 video_streamer.py 가 ffmpeg 로 rtsp://192.168.0.5:8554/ugv1 에 밀어올리면,
// mediamtx 가 그걸 WebRTC 재생 페이지로 자동 변환해준다. 그 페이지를 iframe 으로 끼운다.
// 2026-08-28 젯슨(192.168.0.6)에서 서버(192.168.0.5)로 옮겼다 — 시청자가 늘어도
// 젯슨이 부담을 지지 않게 하기 위함. 젯슨은 몇 명이 보든 스트림 하나만 올린다.
const DIRECT_CAMERA_URL = "http://192.168.0.5:8889/ugv1"; // 포트 8889 = mediamtx WebRTC

/* ---------------------------------------------------------------------------
 * 0. 도메인 상수 — 백엔드와 사전 합의한 인터페이스(좌표계 / 이벤트 스키마)
 *    탐지 클래스 4종: docs/interface.md v1.5 확정 기준
 *    (helmet_off→no_helmet, ship_block→block_level 로 이름/구조가 바뀌었음)
 * ------------------------------------------------------------------------- */
const SEVERITY = { DANGER: "danger", WARN: "warn", INFO: "info" };

const CLASS_META = {
  fallen_person:  { label: "작업자 쓰러짐",  severity: SEVERITY.DANGER, group: "안전" },
  fire:           { label: "화재",          severity: SEVERITY.DANGER, group: "화재" },
  no_helmet:      { label: "안전모 미착용",  severity: SEVERITY.WARN,   group: "안전" },
  ship_defect:    { label: "선박 결함",      severity: SEVERITY.WARN,   group: "품질" },
};

const SEV_COLOR = {
  [SEVERITY.DANGER]: "#ff3b47",
  [SEVERITY.WARN]:   "#ffb020",
  [SEVERITY.INFO]:   "#36d399",
};

/* 배 1척을 길이 방향(z축)으로 5개 구획으로 나눈다.
 * 실제 조선소가 배를 블록 단위로 조립하는 방식과 동일.
 * secStart~secEnd: 배 전체 길이(-SHIP_LEN/2 ~ +SHIP_LEN/2) 중 이 구획이 차지하는 구간 */
const BLOCKS = [
  { id: "S1", name: "선수 (뱃머리)", part: "bow"    },
  { id: "S2", name: "선체 전방",     part: "fore"   },
  { id: "S3", name: "선체 중앙",     part: "mid"    },
  { id: "S4", name: "선체 후방",     part: "aft"    },
  { id: "S5", name: "선미 (기관부)", part: "stern"  },
];

/* 배 치수 (Three.js 단위) */
const SHIP_LEN = 22;   // 길이(z축)
const SHIP_BEAM = 4.2; // 폭(x축)
const SHIP_DEPTH = 2.6;// 높이(y축, 흘수 위)
const DECK_Y = SHIP_DEPTH; // 갑판 높이

/* 각 구획이 배 길이에서 차지하는 정규화 구간 [0(선미)~1(선수)] */
const SECTION_RANGE = {
  S5: [0.00, 0.18],
  S4: [0.18, 0.40],
  S3: [0.40, 0.62],
  S2: [0.62, 0.82],
  S1: [0.82, 1.00],
};

/* 위치 문구에서 "앞쪽/뒤쪽"을 붙일 양 끝 구획. 이름을 직접 적지 않고
 * SECTION_RANGE 에서 끌어내므로, 나중에 구획을 늘리거나 이름을 바꿔도 따라온다.
 * (t=0 이 선미, t=1 이 선수) */
const SECTION_IDS_STERN_TO_BOW = Object.keys(SECTION_RANGE)
  .sort((a, b) => SECTION_RANGE[a][0] - SECTION_RANGE[b][0]);
const STERN_BLOCK_ID = SECTION_IDS_STERN_TO_BOW[0];
const BOW_BLOCK_ID = SECTION_IDS_STERN_TO_BOW[SECTION_IDS_STERN_TO_BOW.length - 1];

/* 공정 단계 → 색상 (회색→노랑→초록): 계획서 시나리오3 */
const PROGRESS_COLOR = {
  idle:       new THREE.Color("#6b7280"), // 미조립 회색
  inProgress: new THREE.Color("#eab308"), // 진행 노랑
  done:       new THREE.Color("#22c55e"), // 완료 초록
};

/* ---------------------------------------------------------------------------
 * 1. 좌표 매핑 레이어
 *    서버는 (구획 id + 0~1 로컬 오프셋)을 보낸다고 가정.
 *    구획의 배 길이 방향 중심 z 좌표를 구하고, 로컬 오프셋으로 미세 조정한다.
 *    (요구사항 3: 서버 좌표 → 3D 공간 매핑)
 * ------------------------------------------------------------------------- */
function sectionCenterZ(blockId) {
  const r = SECTION_RANGE[blockId] || [0.4, 0.6];
  const t = (r[0] + r[1]) / 2;             // 0(선미)~1(선수)
  return (t - 0.5) * SHIP_LEN;             // 월드 z (선미 뒤 ~ 선수 앞)
}

/* 0~1 스무스스텝 — 양 끝에서 기울기가 0이 되어 이어붙일 때 꺾임이 없다.
 * SceneManager._smoothstep도 이 함수를 그대로 쓴다(하나로 통일해서 핑 위치와
 * 실제로 그려지는 선체 모양이 항상 같은 공식을 쓰게 함). */
function smoothstep01(x) {
  const c = Math.max(0, Math.min(1, x));
  return c * c * (3 - 2 * c);
}

/* 배 단면 폭 계수: 선미(t=0)~선수(t=1). 뱃머리 쪽은 배가 뾰족해지면서 실제 폭이
 * 크게 줄어든다 — SceneManager._beamFactor(실제 선체 렌더링에 쓰는 것)와 반드시
 * 같은 공식을 써야 한다. 이게 안 맞으면 핑 x좌표가 SHIP_BEAM 기준 "직사각형 배"로
 * 계산되는데, 실제 뱃머리/선미는 그보다 훨씬 좁아서 핑이 선체 바깥 빈 공간(허공)에
 * 찍힌 것처럼 보이는 버그가 생긴다. */
function hullBeamFactor(t) {
  if (t < 0.15) {
    const u = t / 0.15;
    return 0.55 + 0.45 * smoothstep01(u);
  }
  if (t < 0.78) return 1.0;
  const u = (t - 0.78) / 0.22;
  const eased = Math.pow(smoothstep01(u), 1.15);
  return 1.0 - eased * 0.97;
}

function serverToWorld(blockId, local = { x: 0.5, y: 1, z: 0.5 }) {
  const r = SECTION_RANGE[blockId] || [0.4, 0.6];
  const t = r[0] + (r[1] - r[0]) * (local.z ?? 0.5); // 구획 내 z 위치
  const z = (t - 0.5) * SHIP_LEN;
  // 그 z 위치에서 실제 선체가 얼마나 넓은지(hullBeamFactor)를 반영해서 x를 계산한다.
  // 이걸 안 하면 뱃머리(S1)처럼 배가 뾰족해지는 구간에서 핑이 실제 선체보다 훨씬
  // 바깥쪽 — 즉 화재/사고가 없는 빈 물 위 — 에 찍힌 것처럼 보인다.
  // ★ 선폭계수에 하한을 둔다 (2026-08-29).
  //   hullBeamFactor 는 뱃머리·선미 끝에서 0.03 까지 떨어진다. 그대로 곱하면
  //   좌우 오프셋이 통째로 눌려, **끝에 놓인 대상은 좌우가 정확해도 화면에서
  //   가운데로 보인다.** 실제로 뱃머리 끝 3.5cm 지점(t=0.974, 계수 0.073)에서
  //   좌우 표시가 사라졌다.
  //
  //   원래 의도(뾰족한 뱃머리에서 핑이 선체 밖 허공에 뜨는 것 방지)는 살리되,
  //   하한 0.35 를 둬서 좌우 구분은 남긴다. 아주 뾰족한 끝에서는 핑이 선체선을
  //   조금 넘칠 수 있는데, 관제 화면에서는 "선체선 안" 보다 "어느 쪽인지" 가
  //   더 중요하다고 보고 그쪽을 택했다.
  const HULL_BEAM_FLOOR = 0.35;
  const localWidth = SHIP_BEAM * Math.max(hullBeamFactor(t), HULL_BEAM_FLOOR);
  const x = (local.x - 0.5) * localWidth * 0.8;      // 폭 방향(해당 지점 실제 선폭 기준)
  const y = DECK_Y + (local.y ?? 1) * 0.9;           // 갑판 위
  return new THREE.Vector3(x, y, z);
}

/* ---------------------------------------------------------------------------
 * 1-b. 역방향 매핑 — 서버가 실제로 주는 절대좌표(map_xy, 미터)를
 *      blockId + local{x,y,z}로 변환한다. (요구사항 3의 진짜 버전)
 *
 *      서버는 blockId/local을 주지 않는다. 대신
 *        - ship_pose 메시지로 배의 절대 위치/방향(map_xy, yaw)을 알려주고
 *        - 위험 이벤트 메시지의 map_xy로 감지된 대상의 절대 위치를 알려준다
 *      이 둘을 조합해 "배를 기준으로 어디쯤인지"를 계산해야 한다.
 *
 *      ⚠️ SHIP_REAL_LENGTH_M / SHIP_REAL_BEAM_M 은 임시값이다.
 *      실제 배 모형을 측량한 값(이한종님 쪽 ship_survey_node 결과)으로
 *      반드시 교체할 것 — 지금 값으로는 Ping이 엉뚱한 구획에 찍힐 수 있다.
 * ------------------------------------------------------------------------- */
const SHIP_REAL_LENGTH_M = 0.77; // 실측값 (finalize_map의 SHIP_SIZE_XY 상수 기준, 이정기님 확인)
const SHIP_REAL_BEAM_M = 0.14;   // 실측값 (finalize_map의 SHIP_SIZE_XY 상수 기준, 이정기님 확인)

// 로봇(UGV) 실측 크기 — URDF 기준, 전선 돌출부 포함 유효 길이 (이정기님 확인)
const UGV_REAL_LENGTH_M = 0.401;
const UGV_REAL_WIDTH_M = 0.178;
// base_link(회전 중심)가 로봇 뒤쪽 끝에서부터 이만큼 떨어진 지점에 있다.
// 로봇 기하학적 중심이 아니라 이 지점을 축으로 화면에서 회전시켜야
// 실제 로봇이 제자리 회전할 때와 어색하지 않게 맞는다.
const UGV_BASE_LINK_FROM_BACK_M = 0.069;

// 임시 보정값(캘리브레이션): ship_survey_node가 재는 yaw 기준과 로봇 EKF가
// 재는 yaw 기준이 서로 다른 것 같아서(2026-08-20 실측 1건 기준 약 58도 차이),
// 화면에 실제 위치와 최대한 비슷하게 나오도록 임시로 더해주는 값이다.
// ⚠️ 데이터 1건으로만 계산한 추정치라 정확하지 않을 수 있다.
// 로봇을 배 정중앙 앞(뱃머리)에 딱 세워두고 화면에서도 정확히 그 자리에
// 나오는지 확인해서, 안 맞으면 이 숫자를 조금씩 조절해야 한다.
// 근본적으로는 이정기님/이한종님 쪽에 배 측량 시스템과 로봇 EKF가 같은
// 지도 기준(0도 방향)을 쓰고 있는지 확인 요청 필요.
const CALIBRATION_YAW_OFFSET_DEG = 0; // FALLBACK_SHIP_POSE.yaw 자체를 캘리브레이션된 값으로 바꿔서 이제 0으로 둠
const CALIBRATION_YAW_OFFSET_RAD = CALIBRATION_YAW_OFFSET_DEG * Math.PI / 180;

/* 핑 위치 보정 (배 기준, 미터). 0 이면 보정 없음 — 받은 좌표를 그대로 쓴다.
 *
 * ★ 왜 프론트에서 보정하나
 *   배 폭이 14cm 인데 뎁스+TF 로 만든 객체 좌표의 오차는 십수 cm 급이다.
 *   카메라를 바꾸든 젯슨 코드를 고치든 이 축척에서 정밀도를 맞추는 것은
 *   비용 대비 효과가 없고, 대시보드는 "어느 구획 어느 쪽" 만 맞으면 된다.
 *   그래서 **치우친 만큼만** 여기서 되돌린다.
 *
 * ★ 값을 추측으로 넣지 말 것
 *   여기는 계통 편향(항상 같은 방향으로 밀리는 것)만 고칠 수 있다. 무작위
 *   오차는 못 고친다. 그래서 반드시 한 번 재서 넣는다:
 *
 *     1) 감지 대상을 **위치를 아는 자리** 에 놓는다.
 *        정중앙일 필요는 없다 — 어디에 놨는지만 알면 그 값을 빼면 된다.
 *        오히려 눈대중 중앙보다 **물리적 기준선**이 정확하고 반복하기 쉽다:
 *          · 좌현(왼쪽) 옆면에 딱 붙임 →  참값 좌우 = +0.07 (= +SHIP_REAL_BEAM_M/2)
 *          · 우현(오른쪽) 옆면        →  참값 좌우 = -0.07
 *          · 앞뒤 중앙            →  참값 앞뒤 = 0
 *        (모형이 항공모함이라 중앙에 관제탑이 있어 좌우 중앙에 못 놓는다.
 *         그래서 옆면 기준을 쓴다 — 2026-08-29 현장 사정)
 *     2) 로봇이 감지하게 하고, 브라우저 콘솔의 [핑 보정] 줄을 본다
 *          [핑 보정] fire  원본 앞뒤=-0.02 좌우=-0.20  →  표시 앞뒤=-0.02 좌우=-0.20
 *     3) 보정값 = 참값 − 원본
 *          위 예에서 우현 옆면(-0.07)에 놨다면
 *            FORWARD_M = 0    − (-0.02) = +0.02
 *            BEAM_M    = -0.07 − (-0.20) = +0.13
 *     4) 다시 감지시켜 "표시" 값이 참값 근처로 오는지 확인한다
 *
 * ★ 2026-08-29 실측 결과 (젯슨이 같은 불 97건을 군집 분석)
 *
 *     불          참값(앞뒤,좌우)   군집중심          오차(앞뒤, 좌우)
 *     뱃머리 좌현  (+0.35, +0.03)   (+0.331, +0.002)  (-0.019, -0.028)
 *     선미  우현  (-0.35, -0.03)   (-0.399, +0.022)  (-0.049, +0.052)
 *
 *   앞뒤: 두 불 모두 **음수**(선미 쪽으로 밀림), 평균 -0.034 → 계통 편향으로 보고
 *         +0.034 를 넣는다.
 *   좌우: -0.028 과 +0.052 로 **부호가 반대**다. 계통 편향이라는 근거가 없다.
 *         평균을 넣으면 잡음에 맞추는 꼴이라 **0 으로 둔다.**
 *
 *   ★ 더 중요한 것 — 군집 중심은 이미 2~5cm 로 정확하다. 화면에서 핑이 엉뚱한
 *     곳에 찍히는 주된 원인은 편향이 아니라 **건별 흩어짐**(중앙값 0.05m,
 *     최대 0.20m)이다. 이벤트는 군집 평균이 아니라 **한 번의 검출**로 등록되므로
 *     그 한 번이 어디로 튀었느냐가 그대로 핑 위치가 된다.
 *     상수 보정으로 고칠 수 있는 부분은 여기까지다. */
/* 배 중심에서 이 거리를 넘는 위험 이벤트는 화면에 그리지 않는다 (미터).
 *
 * ★ 왜 필요한가 (2026-08-29)
 *   로봇의 카메라는 순찰 원 **안쪽(배 쪽)** 을 본다. 그러니 배에서 한참 떨어진
 *   곳의 화재는 원리상 나올 수 없다. 그런데 실제로 벽 근처의 무언가를 불로
 *   오인한 검출이 대시보드에 유령 핑으로 떴다.
 *
 *   실측 (2026-08-29 02~04시, 화재 15건):
 *     진짜 이벤트 11건 —— 배 중심에서 최대 0.84 m
 *     유령  4건      —— 최소 1.60 m
 *   사이에 0.76 m 의 빈 구간이 있어 깨끗하게 갈린다. 1.2 m 로 자른다.
 *
 * ★ 이것은 방어선이지 해결책이 아니다.
 *   근본 원인은 젯슨의 YOLO 오검출이고(그쪽에서 depth 2~4m 짜리 오검출을
 *   45건 확인했다), 젯슨의 max_depth_m 필터가 1차 방어선이다. 여기는 그것을
 *   통과해버린 것을 화면에서 막는 마지막 그물이다.
 *
 *   ⚠️ 순찰 반경을 크게 바꾸면 이 값도 같이 올려야 한다. 안 그러면 진짜
 *      이벤트가 조용히 사라진다 —— 그래서 걸러낼 때 콘솔에 반드시 남긴다.
 */
const MAX_EVENT_DIST_FROM_SHIP_M = 1.2;

const PING_OFFSET_FORWARD_M = 0.034;  // + 가 뱃머리 쪽 (2026-08-29 실측)
const PING_OFFSET_BEAM_M = 0.0;       // + 가 좌현 쪽 (실측 결과 편향 없음 — 아래)

/* 서버 절대좌표(mapXY)를, "배를 기준으로 한" 상대 좌표(미터)로 바꾸는 공용 변환.
 * forward: 배 진행방향(+뱃머리 ~ -선미), beam: 좌우(+좌현 ~ -우현).
 * ★ beam 부호 주의 — map 은 오른손 좌표계, yaw 는 +x 기준 반시계다.
 *   뱃머리가 +x 를 볼 때 +y 는 왼쪽이므로 **beam 양수 = 좌현(왼편)** 이다.
 * 이벤트 위치(구획 매핑)와 UGV 위치(3D 이동) 둘 다 이 함수를 함께 쓴다. */
function mapXYToShipLocalMeters(mapXY, shipPose) {
  if (!mapXY || !shipPose || !shipPose.map_xy) return null;

  const [ex, ey] = mapXY;
  const [sx, sy] = shipPose.map_xy;
  const yaw = (shipPose.yaw ?? 0) + CALIBRATION_YAW_OFFSET_RAD;
  const dx = ex - sx, dy = ey - sy;

  // 배의 yaw만큼 반대로 회전시켜 "배를 기준으로 한" 좌표로 바꾼다.
  const cos = Math.cos(-yaw), sin = Math.sin(-yaw);
  const forward = dx * cos - dy * sin;
  const beam = dx * sin + dy * cos;

  // 보정은 배 기준 좌표로 바꾼 **뒤에** 더한다. map 좌표에서 더하면 배가 돌아갔을 때
  // 보정 방향까지 같이 돌아가버려서 "배의 왼쪽으로 12cm" 라는 의미가 깨진다.
  return {
    forward: forward + PING_OFFSET_FORWARD_M,
    beam: beam + PING_OFFSET_BEAM_M,
    // 보정 전 값. 캘리브레이션할 때 이 값을 봐야 한다 (아래 로그).
    rawForward: forward,
    rawBeam: beam,
  };
}

function mapXYToBlockLocal(mapXY, shipPose) {
  const rel = mapXYToShipLocalMeters(mapXY, shipPose);
  if (!rel) return null;

  let t = rel.forward / SHIP_REAL_LENGTH_M + 0.5; // 0(선미)~1(선수)로 정규화
  t = Math.max(0, Math.min(1, t));

  let blockId = BLOCKS[0].id;
  for (const b of BLOCKS) {
    const [r0, r1] = SECTION_RANGE[b.id];
    if (t >= r0 && t <= r1) { blockId = b.id; break; }
  }
  const [r0, r1] = SECTION_RANGE[blockId];
  const localZ = r1 > r0 ? (t - r0) / (r1 - r0) : 0.5;
  const localX = Math.max(0, Math.min(1, rel.beam / SHIP_REAL_BEAM_M + 0.5));

  return { blockId, local: { x: localX, y: 0.6, z: localZ } };
}

/* 핑 위치를 사람이 읽는 한 줄로. 예: "S3 왼편", "S1 앞쪽 왼편".
 *
 * 관제사에게 "S3 89%"보다 "S3 왼편 89%"가 훨씬 쓸모 있다 — 배로 뛰어갈 때
 * 어느 쪽으로 돌아야 하는지가 바로 나오기 때문이다.
 *
 * 규칙:
 *   배 위(선체 안)   →  "<구획> 위"                      예) S1 위
 *   배 밖, 앞뒤 안   →  "<구획> <왼편|오른편>"            예) S3 왼편
 *   배 밖, 앞뒤 넘음  →  "<구획> <앞쪽|뒤쪽> <왼편|오른편>" 예) S1 앞쪽 왼편
 *
 * ★ "중앙"은 두지 않는다. 배 폭이 14cm뿐이라 중앙이라고 해봐야 왼쪽 7cm 안이고,
 *   관제사 입장에서는 어느 쪽으로 갈지 정해주는 편이 낫다. 그래서 애매하면
 *   가까운 쪽으로 붙여 왼편/오른편 둘 중 하나만 나온다.
 *
 * 좌우는 배 자신의 기준(좌현/우현)이다. mapXYToShipLocalMeters 가 이미 배 yaw로
 * 회전시켜 놓은 좌표를 쓰므로, 카메라를 어디서 보든 문구가 바뀌지 않는다.
 *
 * 배 위치(ship_pose)를 아직 못 받았으면 변환할 수 없다 — 그때는 구획 id만 돌려준다.
 */
function describePingLocation(mapXY, shipPose, fallbackBlockId) {
  const rel = mapXYToShipLocalMeters(mapXY, shipPose);
  if (!rel) return fallbackBlockId || null;

  // ★ beam > 0 은 **좌현(왼편)** 이다 (2026-08-29 정정).
  //   map 프레임은 오른손 좌표계고 yaw 는 +x 기준 반시계(interface.md ④)다.
  //   뱃머리가 +x 를 볼 때 +y 는 왼쪽이므로, beam(=배 기준 y)이 양수면 좌현이다.
  //   예전에는 반대로 적어 화면 표기가 실제와 좌우가 뒤바뀌어 있었다.
  //   (3D 핑 위치는 원래 맞게 그리고 있었다 — 글자만 틀렸다)
  const halfLength = SHIP_REAL_LENGTH_M / 2;
  const halfBeam = SHIP_REAL_BEAM_M / 2;

  // ★ 배 위에서는 좌/우를 말하지 않는다 (2026-08-29).
  //   배 폭이 14cm(반폭 7cm)인데 좌표 오차가 3~5cm 다. 갑판 위 대상의 좌/우는
  //   센서 정밀도 안쪽이라 자주 뒤집힌다 —— 실측에서 우현에 놓은 불이 좌현으로
  //   읽혔다. 틀릴 수 있는 정보를 관제 화면에 쓰느니 안 쓰는 편이 낫다.
  //   구획(S1~S5)은 한 칸이 15cm 라 오차보다 3배 크므로 믿을 수 있다.
  //
  //   배 밖은 반대다. 야드는 넓어서 좌/우가 오차보다 훨씬 크고, 관제사가
  //   어느 쪽으로 돌아가야 하는지 알려면 그 정보가 필요하다. 그래서 유지한다.
  const onShip =
    Math.abs(rel.forward) <= halfLength && Math.abs(rel.beam) <= halfBeam;
  if (onShip) {
    const id = mapXYToBlockLocal(mapXY, shipPose)?.blockId || fallbackBlockId;
    return id ? `${id} 위` : null;
  }

  const side = rel.beam >= 0 ? "왼편" : "오른편";
  if (rel.forward > halfLength) return `${BOW_BLOCK_ID} 앞쪽 ${side}`;
  if (rel.forward < -halfLength) return `${STERN_BLOCK_ID} 뒤쪽 ${side}`;

  const blockId = mapXYToBlockLocal(mapXY, shipPose)?.blockId || fallbackBlockId;
  return blockId ? `${blockId} ${side}` : side;
}

/* UGV는 배 위가 아니라 배 옆(바깥)을 돌아다니므로 0~1로 자르지 않고,
 * 배 축척(SHIP_LEN/SHIP_REAL_LENGTH_M)을 그대로 곱해서 Three.js 월드 좌표로 바꾼다.
 * 반환값의 x/z는 SceneManager.setUgvPosition()에 그대로 넣으면 된다. */
function mapXYToUgvWorld(mapXY, ugvYaw, shipPose) {
  const rel = mapXYToShipLocalMeters(mapXY, shipPose);
  if (!rel) return null;

  // 안전장치: 로봇은 배 위가 아니라 배 "주변 작업장"을 돌아다니므로,
  // 배 실측 길이/폭보다 훨씬 멀리 나가는 게 정상이다 (오작동 아님).
  // 그래서 배 크기가 아니라 "작업장에서 로봇이 실제로 돌아다닐 것으로
  // 예상되는 범위(미터)"를 기준으로 클램프한다 — 필요하면 이 두 값을
  // 실제 로봇 활동 반경에 맞게 더 늘려도 된다.
  const MAX_PATROL_FORWARD_M = 3;  // 배 기준 앞뒤로 최대 몇 m까지 보여줄지
  const MAX_PATROL_BEAM_M = 2;     // 배 기준 좌우로 최대 몇 m까지 보여줄지
  const clamped = Math.abs(rel.forward) > MAX_PATROL_FORWARD_M || Math.abs(rel.beam) > MAX_PATROL_BEAM_M;
  const clampedForward = Math.max(-MAX_PATROL_FORWARD_M, Math.min(MAX_PATROL_FORWARD_M, rel.forward));
  const clampedBeam = Math.max(-MAX_PATROL_BEAM_M, Math.min(MAX_PATROL_BEAM_M, rel.beam));
  if (clamped) {
    console.warn(
      "[좌표 변환] UGV가 MAX_PATROL_FORWARD_M/MAX_PATROL_BEAM_M 범위 밖으로 나가서 " +
      "화면 가장자리로 눌러서 표시 중 — 로봇이 실제로 그렇게 멀리 갔다면, 이 두 값을 늘려주세요.",
      { rawForward: rel.forward, rawBeam: rel.beam }
    );
  }

  const lengthScale = SHIP_LEN / SHIP_REAL_LENGTH_M;
  const beamScale = SHIP_BEAM / SHIP_REAL_BEAM_M;
  const worldZ = clampedForward * lengthScale;
  const worldX = clampedBeam * beamScale;

  // UGV의 진짜 yaw(map 기준 절대각)도 "배를 기준으로 한 상대각"으로 바꿔서
  // 3D 모델(항상 +z가 뱃머리 방향으로 고정)에서 자연스럽게 보이게 한다.
  const relativeYaw = (shipPose.yaw != null && ugvYaw != null)
    ? ugvYaw - shipPose.yaw
    : null;

  return { x: worldX, z: worldZ, yaw: relativeYaw };
}

/* 위험(화재/사고) 위치를 "배 위(구획 안)"인지 "배 밖(주변 작업장)"인지 먼저 판단해서
 * 서로 다른 좌표계로 변환한다.
 *
 * mapXYToBlockLocal은 원래 "위험은 항상 배 위에서 감지된다"는 가정으로 만들어져서,
 * 배 밖에서 감지된 화재도 0~1 범위로 억지로 눌러(clamp) 가장 가까운 구획 가장자리에
 * 붙여버린다 — 그래서 "화재가 배에서 멀리 떨어져 있는데 핑은 배 위에 찍힌다"는
 * 문제가 생겼다. 이제 배 실측 크기(약간의 여유 포함) 안쪽인지 먼저 확인해서,
 * 배 밖이면 UGV 위치 변환(mapXYToUgvWorld)과 똑같은 방식 — 야드 공간에 실제
 * 상대 위치 그대로 — 으로 배치한다. */
function mapXYToPingWorld(mapXY, shipPose) {
  const rel = mapXYToShipLocalMeters(mapXY, shipPose);
  if (!rel) return null;

  // 배 실측 길이/폭보다 여유를 둬서, 뱃전에 거의 붙어있는 정도는 "배 위"로 본다.
  //
  // ★ 비율(15%)이 아니라 **절대값**으로 준다 (2026-08-29 수정).
  //   비율로 주면 길이(77cm)에는 5.8cm 여유가 붙는데 폭(14cm)에는 1.0cm 밖에
  //   안 붙는다. 그런데 측정 흔들림은 방향과 무관하게 비슷하다 —— 젯슨 실측으로
  //   **중앙값 0.05m, 최대 0.20m** (같은 불 97건 기준).
  //
  //   즉 좌우 여유(1cm)가 흔들림(5cm)의 1/5 이라, 갑판 한가운데 놓인 불도
  //   절반 넘게 "배 밖"으로 튕겨 야드 바닥에 그려졌다. 배 위에 있는 불이
  //   바닥에 찍히는 그 증상의 원인이 이것이다.
  //
  //   여유는 흔들림 중앙값(5cm)보다 넉넉하고 최대값(20cm)보다는 작게 잡는다.
  //   너무 키우면 이번엔 배 옆 바닥에 있는 불이 갑판 위로 올라온다.
  //   14cm 폭 배에서 이 구분은 원래 센서 정밀도의 한계선 근처다.
  const ON_SHIP_SLACK_M = 0.08;
  const onShip =
    Math.abs(rel.forward) <= SHIP_REAL_LENGTH_M / 2 + ON_SHIP_SLACK_M &&
    Math.abs(rel.beam) <= SHIP_REAL_BEAM_M / 2 + ON_SHIP_SLACK_M;

  if (onShip) {
    const blockLocal = mapXYToBlockLocal(mapXY, shipPose);
    return blockLocal ? { onShip: true, blockId: blockLocal.blockId, local: blockLocal.local } : null;
  }

  // 배 밖(작업장) — UGV와 같은 축척/클램프 범위를 그대로 쓴다. blockId는 화면 표시
  // 위치엔 안 쓰지만, 이벤트 로그/구획 강조 등 기존 UI가 여전히 blockId를 필요로
  // 해서 "제일 가까운 구획"으로 하나 붙여준다.
  const MAX_FORWARD_M = 3;
  const MAX_BEAM_M = 2;
  const clampedForward = Math.max(-MAX_FORWARD_M, Math.min(MAX_FORWARD_M, rel.forward));
  const clampedBeam = Math.max(-MAX_BEAM_M, Math.min(MAX_BEAM_M, rel.beam));
  const lengthScale = SHIP_LEN / SHIP_REAL_LENGTH_M;
  const beamScale = SHIP_BEAM / SHIP_REAL_BEAM_M;

  let t = rel.forward / SHIP_REAL_LENGTH_M + 0.5;
  t = Math.max(0, Math.min(1, t));
  let nearestBlockId = BLOCKS[0].id;
  for (const b of BLOCKS) {
    const [r0, r1] = SECTION_RANGE[b.id];
    if (t >= r0 && t <= r1) { nearestBlockId = b.id; break; }
  }

  return {
    onShip: false,
    blockId: nearestBlockId,
    worldX: clampedBeam * beamScale,
    worldZ: clampedForward * lengthScale,
  };
}

/* ---------------------------------------------------------------------------
 * 2. 모의 이벤트 소스 — 실제 WebSocket과 동일한 페이로드 형태
 *    payload: { id, ts, cls, blockId, local{x,y,z}, conf }
 * ------------------------------------------------------------------------- */
const CLASS_POOL = [
  "fallen_person", "fire", "no_helmet", "ship_defect",
];

function makeMockEvent() {
  const cls = CLASS_POOL[Math.floor(Math.random() * CLASS_POOL.length)];
  const block = BLOCKS[Math.floor(Math.random() * BLOCKS.length)];
  return {
    id: `evt_${Date.now()}_${Math.floor(Math.random() * 1e4)}`,
    ts: Date.now(),
    cls,
    blockId: block.id,
    local: { x: Math.random(), y: Math.random() * 0.6 + 0.2, z: Math.random() },
    conf: Math.round((Math.random() * 0.3 + 0.68) * 100) / 100,
  };
}

/* 실제 연동 지점. 백엔드 준비되면 내부만 WebSocket으로 교체.
 * onEvent(payload) 콜백 계약은 동일하게 유지. */
function connectEventSource(onEvent) {
  // === 실제 연동 시 ===
  // const ws = new WebSocket("wss://.../ws/events");
  // ws.onmessage = (m) => onEvent(JSON.parse(m.data));
  // return () => ws.close();

  const timer = setInterval(() => {
    // 위험 이벤트가 가끔, 일반 이벤트가 자주 들어오도록 가중
    const e = makeMockEvent();
    onEvent(e);
  }, 2200);
  return () => clearInterval(timer);
}

/* ---------------------------------------------------------------------------
 * 2-b. 진짜 이벤트 소스 — 백엔드 /ws/frontend에 연결한다. (docs/interface.md v1.5)
 *
 *    connectEventSource와 계약이 다르다: 서버 메시지는 event_type 기준으로
 *    종류가 여러 개(position/위험이벤트/block_level/ship_pose)라, 여기서는
 *    "원본 메시지 그대로" handlers.onMessage로 넘긴다. 종류별 처리(핑을 찍을지,
 *    공정률을 바꿀지 등)는 이 함수 밖에서 판단한다 — 서버 스펙이 바뀌어도
 *    이 연결 함수 자체는 손댈 필요가 없게 하기 위함.
 * ------------------------------------------------------------------------- */
function connectRealEventSource(url, handlers, { retryMs = 2000 } = {}) {
  // ★ 끊기면 스스로 다시 붙는다 (2026-08-29 추가).
  //   예전에는 소켓을 한 번만 열었다. 그래서 백엔드를 다시 켜도 대시보드는
  //   영영 다시 붙지 않았고, 사람이 새로고침해야 데이터가 들어왔다.
  //   "서버와 재연결되었습니다" 팝업이 안 뜨던 것도 이 때문이다 —— 재연결
  //   자체가 일어나지 않았다.
  //
  //   간격은 고정 2초다. 지수 백오프를 쓰지 않는 이유는, 같은 공유기 안의
  //   서버라 몇 초 안에 돌아오는 것이 보통이고, 백오프가 길어지면 시연 중에
  //   "서버는 켰는데 화면이 한참 안 돌아오는" 상황이 되기 때문이다.
  let ws = null;
  let timer = null;
  let closed = false;   // 사용자가 화면을 떠난 경우 — 더 이상 재시도하지 않는다

  const schedule = () => {
    if (closed || timer) return;
    timer = setTimeout(() => { timer = null; open(); }, retryMs);
  };

  const open = () => {
    if (closed) return;
    try {
      ws = new WebSocket(url);
    } catch (e) {
      console.error("WebSocket 연결 생성 실패:", e);
      handlers.onClose?.();
      schedule();
      return;
    }

    ws.onopen = () => handlers.onOpen?.();
    ws.onclose = () => {
      handlers.onClose?.();
      schedule();          // 끊기면 곧바로 다음 시도를 예약
    };
    ws.onerror = (e) => {
      console.error("WebSocket 오류 (서버가 꺼져있거나 IP/포트가 다를 수 있음):", e);
      handlers.onError?.(e);
      // onerror 뒤에는 onclose 가 따라오므로 여기서 schedule 하지 않는다
      // (하면 재시도가 두 배로 쌓인다)
    };
    ws.onmessage = (msg) => {
      let data;
      try {
        data = JSON.parse(msg.data);
      } catch (e) {
        console.warn("이벤트 파싱 실패, 무시:", msg.data);
        return;
      }
      handlers.onMessage?.(data);
    };
  };

  open();

  // 반환값은 그대로 "닫는 함수"라 기존 호출부(off())를 안 건드려도 되지만,
  // 함수도 객체라 속성을 붙일 수 있어서 off.send(obj)로 같은 소켓에 메시지도
  // 보낼 수 있게 해준다 (event_ack 등 — 새 연결 필요 없음).
  const close = () => {
    closed = true;
    if (timer) { clearTimeout(timer); timer = null; }
    if (ws) ws.close();
  };
  close.send = (obj) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(obj));
    } else {
      console.warn("[이벤트 채널] 소켓이 열려있지 않아 전송 실패:", obj);
    }
  };
  return close;
}

/* ---------------------------------------------------------------------------
 * 3. Three.js 씬 매니저 (명령형 래퍼)
 *    React state로 매 프레임 리렌더하면 비싸므로, 3D는 ref/명령형으로 제어.
 * ------------------------------------------------------------------------- */
class SceneManager {
  constructor(canvas, { onPickBlock, onPickPing }) {
    this.canvas = canvas;
    this.onPickBlock = onPickBlock;
    this.onPickPing = onPickPing;   // 핑 하나를 콕 집어 눌렀을 때
    this.pings = []; // {mesh, ring, born, ttl, sev}
    this.blockMeshes = new Map();
    this._selectedBlockId = null; // 지금 선택된 구획. 위험 표시와는 별개다.
    this._init();
  }

  _init() {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color("#0a0e17");
    this.scene.fog = new THREE.Fog("#0a0e17", 70, 160);

    this.camera = new THREE.PerspectiveCamera(46, w / h, 0.1, 200);
    this.camera.position.set(0, 16, 20);
    this.camera.lookAt(0, 0, 0);

    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas, antialias: true, alpha: false,
    });
    this.renderer.setSize(w, h, false);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // 조명
    const amb = new THREE.AmbientLight("#5b6b8c", 0.7);
    this.scene.add(amb);
    const key = new THREE.DirectionalLight("#ffffff", 1.1);
    key.position.set(10, 20, 12);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.left = -25; key.shadow.camera.right = 25;
    key.shadow.camera.top = 25; key.shadow.camera.bottom = -25;
    this.scene.add(key);
    const rim = new THREE.DirectionalLight("#3b82f6", 0.4);
    rim.position.set(-12, 8, -10);
    this.scene.add(rim);

    this._buildYard();
    this._buildBlocks();
    this._buildUGV();

    // 상호작용
    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();
    this._onClick = this._handleClick.bind(this);
    this.canvas.addEventListener("click", this._onClick);

    // 궤도 회전(간이) — 드래그로 orbit
    this._initOrbit();

    this.clock = new THREE.Clock();
    this._tick = this._tick.bind(this);
    this._raf = requestAnimationFrame(this._tick);
  }

  _buildYard() {
    const grid = new THREE.GridHelper(60, 60, "#1e2a44", "#141c2e");
    this.scene.add(grid);

    const floorGeo = new THREE.PlaneGeometry(60, 60);
    const floorMat = new THREE.MeshStandardMaterial({
      color: "#0d1322", roughness: 0.95, metalness: 0.0,
    });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.01;
    floor.receiveShadow = true;
    this.scene.add(floor);

    // 드라이도크(배가 놓인 웅덩이) 윤곽
    const dockGeo = new THREE.BoxGeometry(SHIP_BEAM + 4, 0.4, SHIP_LEN + 6);
    const dockMat = new THREE.MeshStandardMaterial({ color: "#0f1826", roughness: 0.9 });
    const dock = new THREE.Mesh(dockGeo, dockMat);
    dock.position.y = -0.2;
    dock.receiveShadow = true;
    this.scene.add(dock);

    const rim = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(SHIP_BEAM + 4, 0.4, SHIP_LEN + 6)),
      new THREE.LineBasicMaterial({ color: "#1e3a5f", transparent: true, opacity: 0.5 })
    );
    rim.position.y = -0.2;
    this.scene.add(rim);
  }

  /* 0~1 스무스스텝 / 선체 폭 계수 — 파일 위쪽의 smoothstep01/hullBeamFactor와
   * 같은 공식을 써야 한다(핑 위치 계산에도 그 함수들을 그대로 쓰기 때문).
   * 그래서 여기서 다시 구현하지 않고 그 함수들을 그대로 호출한다. */
  _smoothstep(x) {
    return smoothstep01(x);
  }

  _beamFactor(t) {
    return hullBeamFactor(t);
  }

  /* 갑판의 세로 곡선(시어, sheer) — 선수 쪽으로 갈수록 갑판이 살짝 치솟는다 */
  _sheerFactor(t) {
    return 1 + Math.max(0, t - 0.6) * 0.35; // 선수 40% 구간에서 갑판 높이 최대 +35%
  }

  /* 한 구획(섹션)의 선체 메쉬를 만든다. z0~z1 구간을 세로로 잘라 lofting.
   * 단면을 4점(사다리꼴)이 아닌 6점으로 늘려 둥근 빌지(선저 곡면)를 표현하고,
   * 세로 분할을 16단으로 늘려 곡선을 매끄럽게 한다. */
  _buildHullSection(zStartN, zEndN, color) {
    const segs = 16;
    // ★ 좌우 방향 주의 — 뱃머리가 +z, 위가 +y 인 오른손 좌표계에서
    //   관측자의 오른쪽은 f × u = ẑ × ŷ = -x̂ 다. 즉 **-x 가 우현, +x 가 좌현**.
    //   (선체는 좌우 대칭이라 이 주석이 틀려도 그림은 같지만, 예전 주석이
    //    반대로 적혀 있던 탓에 핑 위치 문구의 좌/우가 뒤집혀 있었다 — 2026-08-29)
    const ringPts = 6; // 0:바닥중앙(용골) 1:우빌지 2:우현상단 3:갑판우 4:갑판좌 5:좌현상단 ... (대칭 구성)
    const positions = [];
    const indices = [];
    let prevBase = null;

    for (let i = 0; i <= segs; i++) {
      const tN = zStartN + (zEndN - zStartN) * (i / segs);
      const z = (tN - 0.5) * SHIP_LEN;
      const bf = this._beamFactor(tN);
      const sf = this._sheerFactor(tN);
      const halfW = (SHIP_BEAM / 2) * bf;
      const topY = SHIP_DEPTH * sf;
      const keelY = 0.05;
      const bilgeY = topY * 0.22;

      // 우현(-x) → 용골 → 좌현(+x) 순으로 6점 링 (둥근 선저 + 곧은 현측)
      const ring = [
        [-halfW * 0.92, topY,          z],  // 갑판 우현
        [-halfW,        bilgeY * 2.2,  z],  // 우현 빌지(넓은 곳)
        [-halfW * 0.30, keelY,         z],  // 좌측 용골 접근
        [ halfW * 0.30, keelY,         z],  // 우측 용골 접근
        [ halfW,        bilgeY * 2.2,  z],  // 좌현 빌지
        [ halfW * 0.92, topY,          z],  // 갑판 좌현
      ];
      const base = positions.length / 3;
      ring.forEach((p) => positions.push(...p));

      if (prevBase !== null) {
        const n = ringPts;
        for (let s = 0; s < n - 1; s++) {
          const a = prevBase + s, a2 = prevBase + s + 1;
          const b = base + s, b2 = base + s + 1;
          indices.push(a, a2, b2);
          indices.push(a, b2, b);
        }
      }
      prevBase = base;
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    const mat = new THREE.MeshStandardMaterial({
      color, roughness: 0.55, metalness: 0.45,
      emissive: new THREE.Color("#000000"),
      side: THREE.DoubleSide, flatShading: false,
    });
    return new THREE.Mesh(geo, mat);
  }

  _buildBlocks() {
    // 배 전체를 담는 그룹 (선수가 +z를 향하도록 배치됨)
    const ship = new THREE.Group();
    ship.position.y = 0.1;

    BLOCKS.forEach((b) => {
      const [z0, z1] = SECTION_RANGE[b.id];
      const mesh = this._buildHullSection(z0, z1, PROGRESS_COLOR.idle.clone());
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.userData.blockId = b.id;
      ship.add(mesh);

      // 구획 경계 와이어(각 섹션 윤곽 강조)
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(mesh.geometry, 25),
        new THREE.LineBasicMaterial({ color: "#2dd4bf", transparent: true, opacity: 0.18 })
      );
      ship.add(edges);

      this.blockMeshes.set(b.id, { mesh, group: ship, progress: 0, state: "idle", edges });
    });

    // 갑판 위 구조물(선실/브리지) — 선미쪽에 얹기
    const house = new THREE.Mesh(
      new THREE.BoxGeometry(SHIP_BEAM * 0.6, 1.6, 3),
      new THREE.MeshStandardMaterial({ color: "#2b3a52", roughness: 0.7, metalness: 0.3 })
    );
    house.position.set(0, SHIP_DEPTH + 0.9, (0.12 - 0.5) * SHIP_LEN + 1.5);
    house.castShadow = true;
    ship.add(house);
    // 브리지 윗단
    const bridge = new THREE.Mesh(
      new THREE.BoxGeometry(SHIP_BEAM * 0.4, 0.8, 1.4),
      new THREE.MeshStandardMaterial({ color: "#35486a", roughness: 0.6 })
    );
    bridge.position.set(0, SHIP_DEPTH + 2.1, (0.12 - 0.5) * SHIP_LEN + 1.5);
    ship.add(bridge);
    // 마스트
    const mast = new THREE.Mesh(
      new THREE.CylinderGeometry(0.06, 0.06, 2.4, 8),
      new THREE.MeshStandardMaterial({ color: "#8fa3c0" })
    );
    mast.position.set(0, SHIP_DEPTH + 3.3, (0.12 - 0.5) * SHIP_LEN + 1.5);
    ship.add(mast);

    // --- 정밀화 디테일 ---
    this._addHullDetails(ship);

    this.ship = ship;
    this.scene.add(ship);
  }

  /* 선체 정밀 디테일: 현창(둥근 창), 난간, 용골선, 닻, 프로펠러/방향키 */
  _addHullDetails(ship) {
    // 1) 현창(portholes) — 좌우현을 따라 일정 간격으로 작은 원형
    const portMat = new THREE.MeshStandardMaterial({ color: "#0a0e17", roughness: 0.3, metalness: 0.6, emissive: "#1a2336" });
    const portGeo = new THREE.CircleGeometry(0.09, 12);
    const portCount = 18;
    for (let i = 0; i < portCount; i++) {
      const t = 0.06 + (i / (portCount - 1)) * 0.72; // 선미 근처~중앙까지
      const z = (t - 0.5) * SHIP_LEN;
      const bf = this._beamFactor(t);
      const halfW = (SHIP_BEAM / 2) * bf;
      const y = SHIP_DEPTH * this._sheerFactor(t) * 0.55;
      [-1, 1].forEach((side) => {
        const p = new THREE.Mesh(portGeo, portMat);
        p.position.set(side * (halfW * 0.94), y, z);
        p.rotation.y = side > 0 ? Math.PI / 2 : -Math.PI / 2;
        ship.add(p);
      });
    }

    // 2) 갑판 난간 — 좌우현 갑판 가장자리를 따라 얇은 레일
    const railMat = new THREE.LineBasicMaterial({ color: "#9fb3cc", transparent: true, opacity: 0.55 });
    const railSegs = 40;
    [-1, 1].forEach((side) => {
      const pts = [];
      for (let i = 0; i <= railSegs; i++) {
        const t = i / railSegs;
        const z = (t - 0.5) * SHIP_LEN;
        const bf = this._beamFactor(t);
        const halfW = (SHIP_BEAM / 2) * bf * 0.93;
        const y = SHIP_DEPTH * this._sheerFactor(t) + 0.35;
        pts.push(new THREE.Vector3(side * halfW, y, z));
      }
      const railGeo = new THREE.BufferGeometry().setFromPoints(pts);
      ship.add(new THREE.Line(railGeo, railMat));
      // 난간 기둥(스탠션) — 드문드문
      for (let i = 0; i <= railSegs; i += 4) {
        const t = i / railSegs;
        const z = (t - 0.5) * SHIP_LEN;
        const bf = this._beamFactor(t);
        const halfW = (SHIP_BEAM / 2) * bf * 0.93;
        const yTop = SHIP_DEPTH * this._sheerFactor(t) + 0.35;
        const yBot = SHIP_DEPTH * this._sheerFactor(t);
        const postGeo = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(side * halfW, yBot, z), new THREE.Vector3(side * halfW, yTop, z),
        ]);
        ship.add(new THREE.Line(postGeo, railMat));
      }
    });

    // 3) 용골선(keel line) — 선저 중앙을 따라 흐르는 강조선
    const keelPts = [];
    for (let i = 0; i <= 40; i++) {
      const t = i / 40;
      const z = (t - 0.5) * SHIP_LEN;
      keelPts.push(new THREE.Vector3(0, 0.05, z));
    }
    const keelGeo = new THREE.BufferGeometry().setFromPoints(keelPts);
    ship.add(new THREE.Line(keelGeo, new THREE.LineBasicMaterial({ color: "#0f766e", transparent: true, opacity: 0.5 })));

    // 4) 선수 닻(anchor) + 호스파이프
    const bowT = 0.90;
    const bowZ = (bowT - 0.5) * SHIP_LEN;
    const bowHalfW = (SHIP_BEAM / 2) * this._beamFactor(bowT);
    const bowDeckY = SHIP_DEPTH * this._sheerFactor(bowT);
    [-1, 1].forEach((side) => {
      const anchor = new THREE.Mesh(
        new THREE.ConeGeometry(0.16, 0.4, 6),
        new THREE.MeshStandardMaterial({ color: "#6b7280", metalness: 0.7, roughness: 0.4 })
      );
      anchor.rotation.z = Math.PI;
      anchor.position.set(side * bowHalfW * 0.85, bowDeckY * 0.65, bowZ - 0.6);
      ship.add(anchor);
    });

    // 5) 선미 프로펠러 + 방향키(러더)
    const sternT = 0.02;
    const sternZ = (sternT - 0.5) * SHIP_LEN;
    const propHub = new THREE.Mesh(
      new THREE.SphereGeometry(0.18, 10, 10),
      new THREE.MeshStandardMaterial({ color: "#8b93a3", metalness: 0.8, roughness: 0.3 })
    );
    propHub.position.set(0, 0.55, sternZ - 0.3);
    ship.add(propHub);
    for (let i = 0; i < 4; i++) {
      const blade = new THREE.Mesh(
        new THREE.BoxGeometry(0.06, 0.5, 0.16),
        new THREE.MeshStandardMaterial({ color: "#8b93a3", metalness: 0.8, roughness: 0.35 })
      );
      blade.position.copy(propHub.position);
      blade.rotation.z = (Math.PI / 2) * i;
      ship.add(blade);
    }
    const rudder = new THREE.Mesh(
      new THREE.BoxGeometry(0.06, 0.8, 0.5),
      new THREE.MeshStandardMaterial({ color: "#4b5563", metalness: 0.5, roughness: 0.5 })
    );
    rudder.position.set(0, 0.5, sternZ - 0.9);
    ship.add(rudder);

    // 6) 선체 표면 수평 강조선(플레이트 스트레이크) — 시각적 정밀도 보강
    const strakeMat = new THREE.LineBasicMaterial({ color: "#000000", transparent: true, opacity: 0.12 });
    [0.35, 0.65].forEach((frac) => {
      const pts = [];
      for (let i = 0; i <= 60; i++) {
        const t = i / 60;
        const z = (t - 0.5) * SHIP_LEN;
        const bf = this._beamFactor(t);
        const halfW = (SHIP_BEAM / 2) * bf;
        const y = SHIP_DEPTH * this._sheerFactor(t) * frac;
        pts.push(new THREE.Vector3(halfW * 0.99, y, z));
      }
      ship.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), strakeMat));
      const pts2 = pts.map((p) => new THREE.Vector3(-p.x, p.y, p.z));
      ship.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts2), strakeMat));
    });
  }

  _buildUGV() {
    // RC카(UGV) 표식 — 실제 축척 그대로 그리면(0.4m는 0.77m 배의 절반이나 됨) 배보다도
    // 큰 판때기로 보이니, 실제 축척은 안 쓰고 고정된 "아이콘" 크기를 쓴다.
    // 로봇 순찰 범위까지 다 보려고 카메라를 멀리 뺄 수 있게 해놔서(줌아웃 최대 110),
    // 그 상태에서도 눈에 잘 띄도록 원래 아이콘 크기(0.5m)의 6배로 키웠다(3배 → 한 번 더 2배).
    // base_link(회전 중심) 위치는 로봇 실측 "비율"(전체 길이 대비 회전축 위치)을
    // 그대로 반영해서 자연스럽게 제자리 회전하도록 만든다.
    const g = new THREE.Group();
    const VISUAL_LEN = 3.0;
    const VISUAL_WIDTH = 2.0;
    const BODY_H = 0.64;
    // base_link가 로봇 중심에서 앞으로 얼마나 떨어져 있는지를 "비율"로 계산해서
    // (뒤에서 0.069m / 전체 0.401m 기준), 고정 아이콘 크기에 그 비율만 적용한다.
    const baseLinkRatioFromCenter = (UGV_REAL_LENGTH_M / 2 - UGV_BASE_LINK_FROM_BACK_M) / (UGV_REAL_LENGTH_M / 2);
    const centerOffsetFromBaseLink = baseLinkRatioFromCenter * (VISUAL_LEN / 2);
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(VISUAL_WIDTH, BODY_H, VISUAL_LEN),
      new THREE.MeshStandardMaterial({ color: "#38bdf8", metalness: 0.4, roughness: 0.4, emissive: "#0c4a6e", emissiveIntensity: 0.5 })
    );
    body.position.set(0, BODY_H, centerOffsetFromBaseLink); body.castShadow = true;
    g.add(body);
    const cam = new THREE.Mesh(
      new THREE.CylinderGeometry(0.3, 0.3, 0.48, 12),
      new THREE.MeshStandardMaterial({ color: "#0ea5e9" })
    );
    cam.rotation.x = Math.PI / 2;
    cam.position.set(0, BODY_H + 0.48, centerOffsetFromBaseLink + VISUAL_LEN / 2 - 0.3);
    g.add(cam);
    this.ugv = g;
    this.ugvAngle = 0;
    this.usingRealUgvPosition = false; // 진짜 position 이벤트가 한 번이라도 오면 true로 바뀜
    this._ugvTarget = null;            // 실제 좌표 도착 목표 {x, z, yaw}
    this.scene.add(g);
  }

  /* 진짜 UGV 위치(월드 좌표, mapXYToUgvWorld로 이미 변환된 값)를 받아서
   * 목표점으로 저장해둔다. _tick()에서 매 프레임 부드럽게 그쪽으로 움직인다.
   * 한 번이라도 이게 호출되면, 그 뒤로는 장식용 자동 순찰 애니메이션을 끈다. */
  setUgvPosition(x, z, yaw) {
    this.usingRealUgvPosition = true;
    this._ugvTarget = { x, z, yaw };
  }

  setBlockProgress(blockId, progress) {
    const rec = this.blockMeshes.get(blockId);
    if (!rec) return;
    rec.progress = Math.max(0, Math.min(1, progress));
    let from, to, t, state;
    if (rec.progress < 0.5) {
      from = PROGRESS_COLOR.idle; to = PROGRESS_COLOR.inProgress; t = rec.progress / 0.5; state = "inProgress";
    } else {
      from = PROGRESS_COLOR.inProgress; to = PROGRESS_COLOR.done; t = (rec.progress - 0.5) / 0.5; state = "done";
    }
    if (rec.progress < 0.02) state = "idle";
    rec.state = state;
    rec.mesh.material.color.copy(from.clone().lerp(to, t));
  }

  /* 요구사항 3: 서버 좌표 → 3D 매핑 + Red Alert Ping */
  spawnPing(payload) {
    // 같은 event_id 를 두 번 받으면 핑이 두 개 겹쳐 그려진다.
    // DANGER 핑은 event_cleared 로만 지워지므로 같은 id 를 다시 그릴 이유가 없다.
    // (젯슨은 같은 event_id 를 "재확인" 의미로 다시 보낼 수 있고, 재접속 복원과
    //  실시간 수신이 겹치는 순간에도 중복이 들어올 수 있다 — 둘 다 여기서 막는다)
    if (payload.eventId && this.pings.some((p) => p.eventId === payload.eventId)) {
      return;
    }

    const meta = CLASS_META[payload.cls];
    if (!meta) return;
    const color = new THREE.Color(SEV_COLOR[meta.severity]);
    // payload.onShip === false면 배 밖(작업장) 실좌표를 직접 받은 것 — 배 위 구획
    // 좌표계(serverToWorld)로 계산하면 안 되고, UGV처럼 야드 바닥 높이에 그대로 찍는다.
    // (onShip이 없는 옛날 방식 payload/모의 이벤트는 그냥 true로 취급해서 기존과 동일하게 동작)
    const isOnShip = payload.onShip !== false;
    const YARD_PING_Y = 0.5; // 야드 바닥에서 핑이 뜨는 높이(UGV 몸체 높이와 비슷한 정도)
    const pos = isOnShip
      ? serverToWorld(payload.blockId, payload.local)
      : new THREE.Vector3(payload.worldX ?? 0, YARD_PING_Y, payload.worldZ ?? 0);
    const isDanger = meta.severity === SEVERITY.DANGER;

    // 디버깅용 — 핑이 "실제로 만들어지는지" / "어디 좌표에 찍히는지"를 콘솔에서 바로 확인할 수 있게.
    // 화면에 아무것도 안 보이는데 이 로그도 안 뜨면 spawnPing 자체가 호출이 안 된 것이고,
    // 로그는 뜨는데 화면엔 안 보이면 좌표나 카메라 쪽 문제다.
    console.log(`[Ping] spawnPing 호출됨 — cls=${payload.cls} blockId=${payload.blockId} onShip=${isOnShip}`, {
      local: payload.local, worldXZ: { x: payload.worldX, z: payload.worldZ },
      worldPos: { x: pos.x, y: pos.y, z: pos.z }, cameraRadius: this.radius,
    });

    // DANGER(화재/사고)는 카메라를 멀리 뺐을 때도(순찰 반경까지 보려고 줌아웃 범위를 넓혀둬서)
    // 놓치지 않도록 WARN보다 눈에 띄게 크게 그린다.
    const coreR = isDanger ? 0.55 : 0.22;
    const ringR = isDanger ? [0.62, 0.92] : [0.25, 0.34];
    const poleH = isDanger ? 3.6 : 2.2;

    // 코어 스피어
    const core = new THREE.Mesh(
      new THREE.SphereGeometry(coreR, 16, 16),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.95 })
    );
    core.position.copy(pos);
    // 클릭 판정에 쓸 정보를 메쉬에 심는다 (_handleClick 참고).
    // 코어만 대상으로 삼는다 — 링/폴까지 넣으면 겹쳐서 엉뚱한 게 잡힌다.
    core.userData.pingEventId = payload.eventId ?? null;
    core.userData.pingBlockId = payload.blockId;
    this.scene.add(core);

    // 확산 링
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(ringR[0], ringR[1], 32),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9, side: THREE.DoubleSide })
    );
    ring.position.copy(pos);
    ring.rotation.x = -Math.PI / 2;
    this.scene.add(ring);

    // 위험 라벨 폴(수직선)
    const poleMat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.6 });
    const poleGeo = new THREE.BufferGeometry().setFromPoints([
      pos.clone(), pos.clone().setY(pos.y + poleH),
    ]);
    const pole = new THREE.Line(poleGeo, poleMat);
    this.scene.add(pole);

    // DANGER(화재/사고)는 시간이 지나도 자동으로 안 사라진다 — 핑 수명은 프론트가 타이머로
    // 정하지 않고 젯슨이 관리한다: 로봇이 그 자리를 다시 지나가서 "이제 없다"고 확인해줄
    // 때(event_cleared)만 지운다. 관제사가 확인 버튼/ESC를 눌러도(event_ack) 핑은 안 지워짐 —
    // 확인은 "봤다/순찰 재개해라"일 뿐, 위험물이 실제로 없어졌다는 뜻은 아니라서.
    // WARN(안전모 미착용/선박 결함)은 기존처럼 잠깐 떴다가 자동으로 사라진다.
    const persistent = meta.severity === SEVERITY.DANGER;
    const ttl = persistent ? Infinity : 5500;
    this.pings.push({
      core, ring, pole, born: performance.now(), ttl,
      sev: meta.severity, blockId: payload.blockId, cls: payload.cls,
      eventId: payload.eventId ?? null, // 젯슨이 만든 고유 id — event_cleared 매칭용
      isDanger,
      persistent,
    });

    // DANGER면 해당 블록에 위험 표시 (지워지기 전까지 계속 빨갛게 남아있음)
    if (meta.severity === SEVERITY.DANGER) {
      this._setBlockDanger(payload.blockId, true);
    }
  }

  // 해당 구획의 지속(persistent) 핑들을 화면에서 지우고, 남은 위험 핑이 없으면
  // 블록의 빨간 강조도 원래대로 되돌린다. core/ring/pole 정리는 공용 헬퍼로 뺐다.
  _disposePing(p) {
    this.scene.remove(p.core, p.ring, p.pole);
    p.core.geometry.dispose(); p.ring.geometry.dispose(); p.pole.geometry.dispose();
  }

  _resetBlockEmissiveIfClear(blockId) {
    const stillDanger = this.pings.some((p) => p.blockId === blockId && p.persistent);
    if (!stillDanger) this._setBlockDanger(blockId, false);
  }

  // ★ 위험(빨강)과 선택(청록)을 분리한다 (2026-08-28).
  //
  //   예전에는 둘 다 material.emissive 하나만 건드렸고, highlightBlock 이
  //   "선택 안 된 구획은 emissiveIntensity = 0" 으로 밀어버렸다. 그래서 위험
  //   구획이 여러 곳이어도 **마지막에 선택된 하나만 보이고 나머지는 빨간색이
  //   칠해진 채로 밝기 0 이라 안 보였다.** 재접속 복원처럼 이벤트가 연달아
  //   들어오면 마지막 것만 남는 것처럼 보인 이유다.
  //
  //   이제 위험 여부를 메쉬가 userData 에 스스로 기억하고, 색칠은 _applyBlockLook
  //   한 곳에서만 한다. 선택은 잠깐이고 위험은 지워질 때까지 유지되므로,
  //   같은 구획이 둘 다면 **위험(빨강)이 이긴다** — 관제에서 놓치면 안 되는 쪽이다.
  _setBlockDanger(blockId, on) {
    const rec = this.blockMeshes.get(blockId);
    if (!rec) return;
    rec.mesh.userData.danger = on;
    this._applyBlockLook(rec.mesh, blockId);
  }

  _applyBlockLook(mesh, id) {
    const danger = !!mesh.userData.danger;
    const selected = this._selectedBlockId === id;

    // 선택된 구획만 살짝 띄운다 (위험이어도 위치는 선택 기준)
    mesh.position.y = selected ? 0.25 : 0;

    if (danger) {
      mesh.material.emissive = new THREE.Color("#ff3b47");
      // 선택까지 됐으면 더 밝게 — 위험한데 보고 있는 중이라는 뜻
      mesh.material.emissiveIntensity = selected ? 0.9 : 0.6;
    } else if (selected) {
      mesh.material.emissive = new THREE.Color("#2dd4bf");
      mesh.material.emissiveIntensity = 0.6;
    } else {
      mesh.material.emissive = new THREE.Color("#000000");
      mesh.material.emissiveIntensity = 0.0;
    }
  }

  // 관제사가 팝업의 "핑 직접 지우기"를 눌렀을 때 호출 — 서버에 아무것도 보내지 않고
  // 화면에서만 그 구획의 지속 핑을 지운다(사람이 눈으로 보고 이미 처리됐다고 판단한 경우용).
  // 백엔드가 나중에 event_cleared를 보내도 이미 지워진 핑이라 그냥 "못 찾음" 경고만 뜨고 끝난다.
  clearBlockPing(blockId) {
    this.pings = this.pings.filter((p) => {
      if (p.blockId !== blockId || !p.persistent) return true;
      this._disposePing(p);
      return false;
    });
    this._resetBlockEmissiveIfClear(blockId);
  }

  // 젯슨이 event_cleared를 보냈을 때 호출 — eventId가 일치하는 핑 하나만 지운다.
  // eventId가 없는(구버전) 메시지거나 매칭되는 핑이 없으면, block_id + cls가 같은
  // 핑으로 대신 매칭한다(안전망). 그래도 못 찾으면 아무것도 안 지운다.
  clearPingByEventId(eventId, fallback = {}) {
    let removed = false;
    let touchedBlockId = null;
    this.pings = this.pings.filter((p) => {
      const matchById = eventId && p.eventId && p.eventId === eventId;
      const matchByFallback = !eventId && fallback.blockId && fallback.cls &&
        p.blockId === fallback.blockId && p.cls === fallback.cls;
      if (!matchById && !matchByFallback) return true;
      this._disposePing(p);
      removed = true;
      touchedBlockId = p.blockId;
      return false;
    });
    if (removed && touchedBlockId) this._resetBlockEmissiveIfClear(touchedBlockId);
    if (!removed) {
      console.warn("[event_cleared] 일치하는 핑을 못 찾음 — 이미 확인 버튼으로 지워졌거나, event_id가 안 맞을 수 있음", { eventId, fallback });
    }
    return removed;
  }

  _initOrbit() {
    let dragging = false, px = 0, py = 0;
    this.theta = 0.7; this.phi = 0.72; this.radius = 30;
    const update = () => {
      const x = this.radius * Math.sin(this.phi) * Math.sin(this.theta);
      const y = this.radius * Math.cos(this.phi);
      const z = this.radius * Math.sin(this.phi) * Math.cos(this.theta);
      this.camera.position.set(x, y, z);
      this.camera.lookAt(0, 1, 0);
    };
    update();
    this._down = (e) => { dragging = true; px = e.clientX; py = e.clientY; };
    this._move = (e) => {
      if (!dragging) return;
      this.theta -= (e.clientX - px) * 0.005;
      this.phi = Math.max(0.2, Math.min(1.25, this.phi - (e.clientY - py) * 0.005));
      px = e.clientX; py = e.clientY; update();
    };
    this._up = () => { dragging = false; };
    this._wheel = (e) => {
      e.preventDefault();
      this.radius = Math.max(12, Math.min(110, this.radius + e.deltaY * 0.02));
      update();
    };
    this.canvas.addEventListener("pointerdown", this._down);
    window.addEventListener("pointermove", this._move);
    window.addEventListener("pointerup", this._up);
    this.canvas.addEventListener("wheel", this._wheel, { passive: false });
    this._orbitUpdate = update;
  }

  // 핑을 먼저 본다. 핑이 배 위에 떠 있으면 그 뒤에 구획 메쉬가 겹쳐 있는데,
  // 사람이 핑을 조준해서 눌렀다면 의도는 "이 핑" 이지 "이 구획" 이 아니다.
  // 핑에 안 맞았을 때만 구획으로 넘어간다.
  _handleClick(e) {
    const rect = this.canvas.getBoundingClientRect();
    this.pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.pointer, this.camera);

    const pingCores = this.pings.filter((p) => p.persistent).map((p) => p.core);
    const pingHit = this.raycaster.intersectObjects(pingCores, false)[0];
    if (pingHit && this.onPickPing) {
      this.onPickPing(pingHit.object.userData.pingEventId,
                      pingHit.object.userData.pingBlockId);
      return;
    }

    const meshes = [...this.blockMeshes.values()].map((r) => r.mesh);
    const hits = this.raycaster.intersectObjects(meshes, false);
    if (hits.length && this.onPickBlock) {
      this.onPickBlock(hits[0].object.userData.blockId);
    }
  }

  // 선택 표시만 바꾼다. 위험 표시는 건드리지 않는다 (_applyBlockLook 참고).
  highlightBlock(blockId) {
    this._selectedBlockId = blockId;
    this.blockMeshes.forEach((rec, id) => this._applyBlockLook(rec.mesh, id));
  }

  _tick() {
    const t = performance.now();
    const dt = this.clock.getDelta();

    if (this.usingRealUgvPosition && this._ugvTarget) {
      // 진짜 좌표 모드 — 매 프레임 목표점 쪽으로 부드럽게 이동(끊겨 보이지 않게).
      // position 이벤트는 0.5초에 한 번만 오므로, 오는 순간 순간이동하지 않도록
      // lerp로 보간한다.
      const lerpSpeed = Math.min(1, dt * 4);
      const tgt = this._ugvTarget;
      this.ugv.position.x += (tgt.x - this.ugv.position.x) * lerpSpeed;
      this.ugv.position.z += (tgt.z - this.ugv.position.z) * lerpSpeed;
      this.ugv.position.y = 0;
      if (tgt.yaw != null) {
        // 각도는 -π~π 경계를 넘나들 수 있어 단순 lerp로는 최단경로가 아닐 수
        // 있지만, UGV가 저속으로 움직이는 데모 수준에서는 충분히 자연스럽다.
        this.ugv.rotation.y += (tgt.yaw - this.ugv.rotation.y) * lerpSpeed;
      }
    } else if (!USE_REAL_BACKEND) {
      // ★ 가짜 순찰 애니메이션은 **mock 모드에서만** 돈다 (2026-08-29).
      //   예전에는 "진짜 좌표를 아직 못 받았으면" 이 조건이라, 서버가 끊긴
      //   상태로 새로고침하면 로봇이 제멋대로 사각형을 그리며 돌았다.
      //   관제 화면에서 그것은 거짓 정보다 —— 실제 로봇은 그 자리에 서 있는데
      //   화면만 순찰하는 것처럼 보인다.
      //
      //   진짜 연동 중에는 좌표가 없으면 **아무것도 하지 않는다.** 마지막으로
      //   받은 위치에 그대로 서 있고, 한 번도 못 받았으면 처음 자리에 있는다.
      this.ugvT = (this.ugvT ?? 0) + dt * 0.12;
      const sweep = Math.sin(this.ugvT); // -1~1
      const z = sweep * (SHIP_LEN / 2);
      const x = SHIP_BEAM / 2 + 1.6;
      this.ugv.position.set(x, 0, z);
      this.ugv.rotation.y = Math.cos(this.ugvT) >= 0 ? 0 : Math.PI;
    }

    // 위험 구획 점멸.
    //
    // ★ 왜 필요한가 (2026-08-28)
    //   재접속 복원으로 되살아난 위험은 팝업을 띄우지 않는다(그래야 새로고침할
    //   때마다 팝업이 연달아 뜨지 않는다). 그러면 화면에 눈길을 끄는 것이 없어
    //   "빨간 구획이 있다"는 사실을 놓치기 쉽다. 숨쉬듯 밝기가 오르내리면
    //   가만히 빨간 것보다 훨씬 빨리 눈에 들어온다.
    //
    //   박자는 핑(_tick 아래 blink)과 같은 0.012 를 쓰고, 구획별 시각이 아니라
    //   전역 시각 t 로 계산한다 — 위험 구획이 여러 곳일 때 따로 놀지 않고 함께
    //   숨쉬어야 어수선해 보이지 않는다.
    //
    //   밝기만 흔든다. 색(빨강)과 위치(선택 시 살짝 띄움)는 _applyBlockLook 이
    //   정한 그대로 둔다.
    const dangerBlink = Math.sin(t * 0.012) * 0.5 + 0.5; // 0~1
    this.blockMeshes.forEach((rec, id) => {
      if (!rec.mesh.userData.danger) return; // 위험이 아니면 _applyBlockLook 값 그대로
      const base = this._selectedBlockId === id ? 0.9 : 0.6;
      rec.mesh.material.emissiveIntensity = base * (0.35 + dangerBlink * 0.65);
    });

    // Ping 애니메이션 + 만료 처리
    this.pings = this.pings.filter((p) => {
      const age = t - p.born;
      if (age > p.ttl) {
        this.scene.remove(p.core, p.ring, p.pole);
        p.core.geometry.dispose(); p.ring.geometry.dispose(); p.pole.geometry.dispose();
        return false;
      }
      const pulse = (age % 1000) / 1000;
      const scale = 1 + pulse * 2.6;
      p.ring.scale.setScalar(scale);
      p.ring.material.opacity = 0.9 * (1 - pulse);
      const blink = p.sev === SEVERITY.DANGER ? (Math.sin(age * 0.012) * 0.5 + 0.5) : 0.85;
      p.core.material.opacity = 0.4 + blink * 0.6;
      p.core.scale.setScalar(0.85 + blink * 0.4);
      return true;
    });

    this.renderer.render(this.scene, this.camera);
    this._raf = requestAnimationFrame(this._tick);
  }

  resize() {
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
  }

  dispose() {
    cancelAnimationFrame(this._raf);
    this.canvas.removeEventListener("click", this._onClick);
    this.canvas.removeEventListener("pointerdown", this._down);
    window.removeEventListener("pointermove", this._move);
    window.removeEventListener("pointerup", this._up);
    this.canvas.removeEventListener("wheel", this._wheel);
    this.renderer.dispose();
  }
}

/* ---------------------------------------------------------------------------
 * 4. CCTV 영상 렌더 (요구사항 4 + 실시간 영상 송출)
 *
 *    영상 흐름은 두 갈래:
 *      A) 상시 라이브 패널(LivePanel)     — UGV 영상을 항상 흘려본다
 *      B) 위험(빨강) 자동 팝업 / 클릭 팝업(CctvPopup)
 *
 *    실제 연동 시 drawCctvFrame() 대신 <video> WebRTC 스트림을 그리고,
 *    그 위에 동일한 bbox 오버레이만 얹으면 된다.
 * ------------------------------------------------------------------------- */

/* 공유 CCTV 프레임 드로잉 — 라이브/팝업 양쪽에서 재사용.
 * UGV는 배보다 훨씬 작아서 카메라가 배 전체를 담지 못하고, 아주 가까운
 * 선체 표면 일부(패널 이음새·리벳)만 클로즈업으로 잡는다는 전제로 그린다. */
function drawCctvFrame(ctx, cv, { event, label, f }) {
  const scale = cv.width / 520;
  // 배경 — 근접한 선체 금속 표면 (화면 밖까지 이어지는 느낌으로 "일부만 보임"을 표현)
  const grad = ctx.createLinearGradient(0, 0, 0, cv.height);
  grad.addColorStop(0, "#1a222e");
  grad.addColorStop(1, "#0b0f16");
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, cv.width, cv.height);

  // 스캔라인 (CCTV 질감)
  for (let y = 0; y < cv.height; y += 3) {
    ctx.fillStyle = "rgba(255,255,255,0.015)";
    ctx.fillRect(0, y, cv.width, 1);
  }

  // 선체 패널 이음새(수평/수직 판금 라인) — 화면 가장자리 밖까지 이어짐
  ctx.strokeStyle = "rgba(150,170,195,0.22)";
  ctx.lineWidth = 1.4 * scale;
  const panelRows = 3;
  for (let i = 1; i < panelRows; i++) {
    const y = (cv.height / panelRows) * i + Math.sin(f * 0.01) * 2;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(cv.width, y); ctx.stroke();
  }
  const panelCols = 4;
  for (let i = 1; i < panelCols; i++) {
    const x = (cv.width / panelCols) * i;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, cv.height); ctx.stroke();
  }
  // 리벳(용접점) — 이음새 교차부 근처에 점으로
  ctx.fillStyle = "rgba(180,195,215,0.35)";
  for (let r = 1; r < panelRows; r++) {
    for (let c = 0; c <= panelCols; c++) {
      const x = (cv.width / panelCols) * c;
      const y = (cv.height / panelRows) * r;
      ctx.beginPath(); ctx.arc(x, y, 2.2 * scale, 0, Math.PI * 2); ctx.fill();
    }
  }
  // 근접 촬영 비네트(화면 모서리가 살짝 어두워져 "가까이서 좁게 보고 있다"는 느낌)
  const vg = ctx.createRadialGradient(
    cv.width / 2, cv.height / 2, cv.height * 0.25,
    cv.width / 2, cv.height / 2, cv.height * 0.75
  );
  vg.addColorStop(0, "rgba(0,0,0,0)");
  vg.addColorStop(1, "rgba(0,0,0,0.45)");
  ctx.fillStyle = vg;
  ctx.fillRect(0, 0, cv.width, cv.height);

  // 인물 실루엣 — 좁은 갑판 통로에 서 있는 크기감으로 배치
  const cx = cv.width * 0.52 + Math.sin(f * 0.03) * (cv.width * 0.03);
  const cy = cv.height * 0.66;
  const fallen = event?.cls === "fallen_person";
  ctx.fillStyle = "#9aa7bd";
  if (fallen) {
    ctx.fillRect(cx - 45 * scale, cy + 30 * scale, 90 * scale, 22 * scale);
    ctx.beginPath(); ctx.arc(cx + 52 * scale, cy + 41 * scale, 13 * scale, 0, Math.PI * 2); ctx.fill();
  } else {
    ctx.fillRect(cx - 12 * scale, cy, 24 * scale, 56 * scale);
    ctx.beginPath(); ctx.arc(cx, cy - 16 * scale, 14 * scale, 0, Math.PI * 2); ctx.fill();
  }
  // bbox + 라벨
  const meta = event ? CLASS_META[event.cls] : null;
  if (meta) {
    const col = SEV_COLOR[meta.severity];
    const bx = (fallen ? cx - 60 * scale : cx - 26 * scale);
    const by = (fallen ? cy + 22 * scale : cy - 34 * scale);
    const bw = (fallen ? 130 : 52) * scale;
    const bh = (fallen ? 44 : 96) * scale;
    ctx.strokeStyle = col; ctx.lineWidth = 2.5 * scale;
    const blink = meta.severity === SEVERITY.DANGER ? (Math.sin(f * 0.18) * 0.5 + 0.5) : 1;
    ctx.globalAlpha = 0.5 + blink * 0.5;
    ctx.strokeRect(bx, by, bw, bh);
    ctx.fillStyle = col; ctx.globalAlpha = 0.85;
    ctx.fillRect(bx, by - 18 * scale, Math.max(bw, 96 * scale), 18 * scale);
    ctx.fillStyle = "#0a0e17"; ctx.globalAlpha = 1;
    ctx.font = `bold ${12 * scale}px monospace`;
    ctx.fillText(`${event.cls} ${(event.conf * 100).toFixed(0)}%`, bx + 4 * scale, by - 5 * scale);
  }
  // HUD
  ctx.globalAlpha = 1;
  ctx.fillStyle = "#2dd4bf"; ctx.font = `${11 * scale}px monospace`;
  ctx.fillText(`● LIVE  UGV-CAM  ${label} · 근접 촬영`, 10 * scale, 18 * scale);
  ctx.fillText(new Date().toLocaleTimeString("ko-KR"), cv.width - 96 * scale, 18 * scale);
}

/* mock 캔버스 애니메이션 루프.
 * USE_REAL_VIDEO=false 일 때만 돈다 — 젯슨 없이 대시보드만 띄워보는 개발용이다.
 * 진짜 영상은 이 훅을 쓰지 않고 아래 LiveVideoDirect 가 <iframe> 으로 그린다. */
function useCctvCanvas(cvRef, active, event, label) {
  useEffect(() => {
    if (!active || !cvRef.current) return;
    const cv = cvRef.current;
    const ctx = cv.getContext("2d");
    let raf, f = 0;
    const loop = () => { f++; drawCctvFrame(ctx, cv, { event, label, f }); raf = requestAnimationFrame(loop); };
    loop();
    return () => cancelAnimationFrame(raf);
  }, [cvRef, active, event, label]);
}

/* 젯슨 직결 영상 — 백엔드를 거치지 않는다.
 * 서버의 mediamtx 가 만들어주는 WebRTC 재생 페이지를 <iframe> 으로 그대로 끼운다.
 * 재생·디코딩·재연결을 브라우저가 알아서 하므로 별도 JS 루프가 필요 없다.
 *
 * ★ P2P 가 아니다. 젯슨이 스트리밍 서버 역할을 겸하고, 브라우저가 백엔드를
 *   우회해 거기 직접 붙는 것이다. WebRTC 를 쓸 뿐 양쪽이 대등한 P2P 는 아니다.
 * 스트림 서버가 꺼져 있으면 iframe 이 비므로 ⟳ 재연결 버튼으로 다시 붙인다. */
function LiveVideoDirect({ event, label, className }) {
  const [key, setKey] = useState(0); // iframe을 강제로 새로 불러오게 하는 트릭 (수동 재연결용)
  const meta = event ? CLASS_META[event.cls] : null;

  return (
    <div className={`direct-video-wrap ${className || ""}`}>
      {/* mediamtx가 자동으로 만들어주는 WebRTC 재생 페이지를 통째로 끼운다.
       * iframe 안쪽 내용은 다른 서버(젯슨)라 onError로 성공/실패를 정확히
       * 감지할 수 없다 — 안 뜨면 mediamtx 페이지 자체가 자기 상태를 보여준다. */}
      <iframe
        key={key}
        src={DIRECT_CAMERA_URL}
        title="UGV 실시간 영상"
        className="direct-video-frame"
        allow="autoplay"
      />
      <button
        type="button"
        className="direct-video-reload"
        onClick={(e) => { e.stopPropagation(); setKey((k) => k + 1); }}
        title="영상이 안 뜨면 눌러서 다시 연결"
      >
        ⟳ 재연결
      </button>
      {meta && (
        <div className="direct-video-badge" style={{ background: SEV_COLOR[meta.severity] }}>
          {event.cls} {(event.conf * 100).toFixed(0)}%
        </div>
      )}
      <div className="direct-video-hud">● LIVE  UGV-CAM  {label}</div>
    </div>
  );
}

/* 상시 라이브 패널 — 항상 켜져 UGV 영상을 흘려본다.
 * 경고(노랑) 발생 시 테두리가 깜빡이며 "확인 필요"를 알린다. */
function LivePanel({ ugvBlock, warnEvent, onExpand }) {
  const cvRef = useRef(null);
  const label = ugvBlock ? ugvBlock.name : "야드 순찰";
  const isDirect = USE_REAL_VIDEO;
  useCctvCanvas(cvRef, !isDirect, warnEvent, label);
  const warning = !!warnEvent;
  return (
    <div className={`live-panel ${warning ? "live-warn" : ""}`} onClick={onExpand} title="클릭하면 확대">
      <div className="live-head">
        <span className="live-dot" /> 실시간 UGV 영상
        {warning && <span className="live-warn-tag">⚠ 경고 — 확인</span>}
      </div>
      {isDirect ? (
        <LiveVideoDirect event={warnEvent} label={label} className="live-canvas" />
      ) : (
        <canvas ref={cvRef} width={420} height={236} className="live-canvas" />
      )}
    </div>
  );
}

/* 연결 알림 카드 하나. 로봇용/서버용이 같은 모양을 쓴다.
 * 배경 클릭으로는 닫히지 않는다 — 관제사가 못 보고 지나치면 안 되는 내용이라
 * "확인" 을 명시적으로 누르게 한다. */
function ConnNotice({ tone, title, lines, btnLabel, onConfirm }) {
  return (
    <div className={`conn-notice ${tone}`}>
      <div className="conn-notice-title">{title}</div>
      {lines.map((t, i) => (
        <p key={i} className={i === 0 ? "conn-notice-body" : "conn-notice-sub"}>{t}</p>
      ))}
      <button type="button" className="conn-notice-btn" onClick={onConfirm} autoFocus>
        {btnLabel}
      </button>
    </div>
  );
}

/* 확대 팝업 — 위험(빨강) 자동 송출 + 클릭 시 표시 공용.
 * ESC 키로도 닫을 수 있게 한다 (X 버튼 클릭 없이 키보드로 종료). */
function CctvPopup({ block, event, group, auto, onClose, onAck, onClearPing }) {
  const cvRef = useRef(null);
  const isDirect = USE_REAL_VIDEO;
  useCctvCanvas(cvRef, !!block && !isDirect, event, block ? block.name : "");

  useEffect(() => {
    if (!block) return;
    // ESC: 확인 버튼이 있는 경우(onAck 존재) ESC도 "확인"과 동일하게 동작 — 확인 이벤트 전송 후 닫힘.
    // 확인 버튼이 없는 팝업(onAck 없음)은 기존처럼 그냥 닫기만 한다.
    const onKey = (e) => {
      if (e.key !== "Escape") return;
      if (onAck) onAck();
      else onClose();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [block, onClose, onAck]);

  if (!block) return null;
  const meta = event ? CLASS_META[event.cls] : null;
  // 사진이 실제로 있는 것만 나열한다. 아직 스냅샷이 안 온 이벤트는 칸을 만들지 않는다.
  const shots = (group && group.length ? group : event ? [event] : [])
    .filter((e) => e && e.imageUrl);
  // 화면에 계속 남아있는 핑(DANGER)이 있는 팝업일 때만 "핑 지우기" 버튼을 보여준다 —
  // WARN처럼 알아서 사라지는 핑이거나 애초에 활성 이벤트가 없으면 지울 게 없으니 표시 안 함.
  const canClearPing = !!(onClearPing && meta && meta.severity === SEVERITY.DANGER);
  return (
    <div className="popup-backdrop" onClick={onClose}>
      <div className={`popup ${auto ? "popup-auto" : ""}`} onClick={(e) => e.stopPropagation()}>
        <div className="popup-head">
          <div className="popup-title">
            <span className="rec-dot" />
            {auto ? "위험 감지 — 자동 송출" : "Click & View — 실시간 CCTV"}
          </div>
          <span className="popup-esc-hint">Esc 로 닫기</span>
        </div>
        {isDirect ? (
          <LiveVideoDirect event={event} label={block.name} className="popup-canvas" />
        ) : (
          <canvas ref={cvRef} width={760} height={428} className="popup-canvas" />
        )}
        {shots.length > 0 && (
          <div className="popup-snaps">
            {shots.length > 1 && (
              <div className="popup-snaps-head">
                📸 이 구역의 감지 순간 {shots.length}건
              </div>
            )}
            <div className={`popup-snaps-grid ${shots.length > 1 ? "multi" : ""}`}>
              {shots.map((ev) => {
                const m = CLASS_META[ev.cls];
                return (
                  <figure className="popup-snap" key={ev.eventId || ev.id}>
                    <img
                      className="popup-snap-img"
                      src={ev.imageUrl}
                      alt={`${m ? m.label : ev.cls} 감지 순간`}
                      /* 사진이 서버에서 지워졌으면 깨진 아이콘 대신 그 칸만 숨긴다 —
                         사진은 있으면 좋은 것이지 필수가 아니다. */
                      onError={(e) => { e.currentTarget.closest(".popup-snap").style.display = "none"; }}
                    />
                    <figcaption className="popup-snap-cap">
                      {shots.length > 1 && m && (
                        <span style={{ color: SEV_COLOR[m.severity] }}>{m.label} · </span>
                      )}
                      {ev.locLabel || ev.blockId}
                      {shots.length === 1 ? " — 감지 순간" : ""}
                    </figcaption>
                  </figure>
                );
              })}
            </div>
          </div>
        )}
        <div className="popup-meta">
          <div><span className="k">구역</span><span className="v">{block.name} ({block.id})</span></div>
          {event?.locLabel && (
            <div><span className="k">위치</span><span className="v">{event.locLabel}</span></div>
          )}
          {meta ? (
            <>
              <div><span className="k">탐지</span>
                <span className="v" style={{ color: SEV_COLOR[meta.severity] }}>
                  {meta.label} · {meta.group}
                </span>
              </div>
              <div><span className="k">신뢰도</span><span className="v">{(event.conf * 100).toFixed(0)}%</span></div>
              <div><span className="k">시각</span><span className="v">{new Date(event.ts).toLocaleTimeString("ko-KR")}</span></div>
            </>
          ) : (
            <div><span className="k">상태</span><span className="v">정상 — 활성 경보 없음</span></div>
          )}
        </div>
        {(onAck || canClearPing) && (
          <div className="popup-actions">
            {onAck && (
              <button type="button" className="popup-ack-btn" onClick={onAck}>
                확인 — 위험 확인, 순찰 재개
              </button>
            )}
            {canClearPing && (
              <button type="button" className="popup-clear-btn" onClick={onClearPing}>
                🗑 핑 직접 지우기 (화면에서만 제거)
              </button>
            )}
          </div>
        )}
      </div>
    </div>
  );
}

/* ---------------------------------------------------------------------------
 * 5. 메인 대시보드
 * ------------------------------------------------------------------------- */
export default function ShipyardTwinDashboard() {
  const canvasRef = useRef(null);
  // 3D 씬을 못 띄운 이유. null 이면 정상.
  // ★ 이게 있어야 WebGL 실패가 대시보드 전체를 무너뜨리지 않는다 (아래 참고).
  const [sceneError, setSceneError] = useState(null);
  // 연결 알림 팝업. 로봇과 서버를 **각각 따로** 들고 있는다 (null = 안 떠 있음).
  //
  // ★ 왜 슬롯을 나누나 (2026-08-29)
  //   둘은 다른 사건이고 동시에 일어날 수 있다. 로봇이 먼저 끊기고 이어서
  //   서버도 끊기면, 로봇 팝업을 밀어내지 않고 **나란히** 띄워야 관제사가
  //   무엇이 무엇 때문인지 안다. 슬롯 안에서는 최신 상태가 이긴다 —
  //   확인을 안 눌렀어도 상태가 바뀌면 그 슬롯의 내용만 바뀐다.
  const [robotNotice, setRobotNotice] = useState(null);
  const [serverNotice, setServerNotice] = useState(null);
  const serverConnectedRef = useRef(null);
  // 상단 배지에 항상 표시할 현재 로봇 연결 상태.
  // null = 아직 서버로부터 상태를 못 받음. robotConnectedRef 는 "직전 값"이라
  // 리렌더를 일으키지 않으므로, 화면에 그릴 값은 state 로 따로 둔다.
  const [robotConnected, setRobotConnected] = useState(null);
  // 직전 연결 상태. 처음 받은 값은 "변화"가 아니므로 팝업을 띄우지 않는다
  // (단, 처음부터 끊겨 있으면 알려줘야 하므로 그때는 띄운다 — 아래 참고).
  const robotConnectedRef = useRef(null);
  const sceneRef = useRef(null);
  const [events, setEvents] = useState([]);          // 이벤트 로그
  const [activeBlock, setActiveBlock] = useState(null);
  const [activeEvent, setActiveEvent] = useState(null);
  // 팝업에 함께 보여줄 이벤트들. 구획을 누르면 그 구획의 위험 전부,
  // 핑을 누르면 그 하나만 담긴다. activeEvent 는 그중 대표(확인 버튼용).
  const [activeGroup, setActiveGroup] = useState([]);
  const [progress, setProgress] = useState(() =>
    Object.fromEntries(BLOCKS.map((b) => [b.id, Math.random() * 0.4])));
  const [stats, setStats] = useState({ danger: 0, warn: 0, info: 0 });
  const [connected, setConnected] = useState(false);
  const [autoPopup, setAutoPopup] = useState(false);     // 위험 자동 송출 여부
  const [warnEvent, setWarnEvent] = useState(null);      // 라이브 패널 경고 표시
  const [ugvBlock, setUgvBlock] = useState(BLOCKS[0]);   // UGV가 보고 있는 구역
  const warnTimer = useRef(null);
  // TODO(임시): websocket_client.py → 백엔드 중계 경로에서 ship_pose가 아직 안 오고 있어서,
  // 직접 로봇을 배 뱃머리/선미 두 지점에 놓고 실측해서 역산한 값을 임시 기본값으로 넣어둠
  // (2026-08-20 22:50~22:51 캘리브레이션, ros2 topic echo로 받은 최초 측량값의 yaw가
  // 실제와 많이 달라서 — 아마 ship_survey_node와 로봇 EKF가 서로 다른 방향 기준을
  // 쓰고 있는 것으로 추정 — 이 값으로 대체함).
  // 중계가 고쳐져서 실제 ship_pose 이벤트가 오면 아래 onMessage 핸들러가 이 값을 자동으로 덮어씀 —
  // 중계 고쳐지면 이 기본값은 지워도 됨 (아래 FALLBACK_SHIP_POSE 상수 포함째로 삭제).
  const FALLBACK_SHIP_POSE = {
    map_xy: [1.0778516278314955, -0.740519236385165],
    yaw: 0.35384,
    block_id: null,
  };
  const shipPoseRef = useRef(FALLBACK_SHIP_POSE); // 최신 ship_pose(map_xy, yaw) — 좌표 변환에 씀
  const seenEventTypesRef = useRef(new Set()); // 디버깅용 — 어떤 event_type이 실제로 오는지 콘솔에 한 번씩만 찍기
  const wsControlRef = useRef(null); // 지금 연결된 소켓의 close()/send() — event_ack 보낼 때 씀 (진짜 백엔드 모드에서만 채워짐)
  // 구획(blockId)별 "아직 확인 안 한" 최신 위험 이벤트 저장소.
  // 자동 팝업이 아니라 사용자가 블록을 직접 클릭(Click & View)했을 때도,
  // 그 구획에 현재 위험 이벤트가 떠 있으면 확인 버튼이 보이게 하려고 따로 기억해둔다.
  // (state로 안 하고 ref로 하는 이유: handlePickBlock을 다시 만들지 않아도 되게 하려고 — 씬 재생성 방지)
  // 살아있는 위험 이벤트를 **event_id 기준으로 전부** 들고 있는다.
  //
  // ★ 예전에는 { [blockId]: 이벤트 } 였다 (2026-08-28 이전).
  //   구획당 한 건만 남아서, 같은 구획에 불이 둘이면 나중 것이 앞 것을 지웠다.
  //   그래서 S5 를 눌러도 스냅샷이 하나만 떴다. 핑은 3D 에 둘 다 떠 있는데
  //   눌러서 볼 수 있는 건 하나뿐인 상태였다.
  const dangerByIdRef = useRef({});

  // 그 구획에 걸린 위험 이벤트들을 등록순으로. 팝업이 이걸로 사진을 나열한다.
  const dangerListOf = useCallback((blockId) =>
    Object.values(dangerByIdRef.current).filter((e) => e.blockId === blockId), []);

  // 조립 단계(1~5)를 "아래에서 N번째 구획까지 완성"으로 화면에 반영하는 공용 함수.
  // block_level 웹소켓 이벤트, /api/init-data 초기 로딩 둘 다 이 함수를 같이 쓴다.
  // ⚠️ 배가 한 척(B1)뿐이라 구획별이 아니라 "몇 번째 구획까지 끝났는지"로 계산한다 —
  // 젯슨이 보내는 block_id("B1")는 배 자체의 id지, BLOCKS의 S1~S5(구획)가 아니다.
  const applyStageProgress = useCallback((stage) => {
    const clamped = Math.max(0, Math.min(BLOCKS.length, Number(stage) || 0));
    setProgress((prev) => {
      const next = { ...prev };
      BLOCKS.forEach((b, i) => {
        const p = i < clamped ? 1 : 0;
        next[b.id] = p;
        if (sceneRef.current) sceneRef.current.setBlockProgress(b.id, p);
      });
      return next;
    });
  }, []);

  // /api/init-data를 REST로 읽어서 배 위치(ship_pose)와 조립 단계(level) 초기값을 반영한다.
  // ship_pose는 측량 끝나는 순간 딱 1번만 웹소켓으로 나가서, 그 타이밍에
  // 연결이 안 돼있으면 영영 놓친다 — 그래서 백엔드가 MongoDB에 저장해둔 최신값을
  // REST로 받아오는 게 훨씬 안전하다. 페이지 열 때 1번 + 웹소켓이 (재)연결될 때마다
  // 다시 호출한다 (연결 끊긴 동안 놓친 갱신을 따라잡기 위해).
  const fetchInitData = useCallback(() => {
    if (!USE_REAL_BACKEND) return;
    const initDataUrl = `http://${SERVER_HOST}/api/init-data`;
    fetch(initDataUrl)
      .then((res) => res.json())
      .then((data) => {
        const blocks = Array.isArray(data?.blocks) ? data.blocks : [];
        const b1 = blocks.find((b) => b && b.id === "B1");
        if (b1 && b1.x != null && b1.y != null && b1.yaw != null) {
          shipPoseRef.current = { map_xy: [b1.x, b1.y], yaw: b1.yaw, block_id: b1.id };
          console.log("[초기 데이터] /api/init-data에서 배 위치를 불러왔습니다 — 이제부터 진짜 실측값 사용", shipPoseRef.current);
        } else {
          console.warn("[초기 데이터] /api/init-data 응답에서 B1(배) 항목을 못 찾았습니다 — 임시값(FALLBACK_SHIP_POSE) 계속 사용", data);
        }
        if (b1 && b1.level != null) {
          applyStageProgress(b1.level);
          console.log("[초기 데이터] /api/init-data에서 조립 단계(level)도 반영함:", b1.level);
        }
      })
      .catch((err) => {
        console.warn(`[초기 데이터] ${initDataUrl} 불러오기 실패 — 임시값(FALLBACK_SHIP_POSE) 계속 사용`, err);
      });
  }, [applyStageProgress]);

  useEffect(() => { fetchInitData(); }, [fetchInitData]);

  const handlePickBlock = useCallback((blockId) => {
    const block = BLOCKS.find((b) => b.id === blockId);
    const list = dangerListOf(blockId);
    setActiveBlock(block);
    setAutoPopup(false);
    // 이 구획에 걸린 위험 이벤트를 **전부** 띄운다. 대표(확인 버튼·탐지 정보)는
    // 가장 최근 것으로 하고, 사진은 아래에 전부 나열한다.
    setActiveEvent(list.length ? list[list.length - 1] : null);
    setActiveGroup(list);
    if (sceneRef.current) sceneRef.current.highlightBlock(blockId);
  }, [dangerListOf]);

  // 핑 하나를 콕 집어 눌렀을 때 — 그 이벤트만 보여준다.
  // 구획 클릭과 달리 옆 핑까지 끌어오지 않는다. "이거 뭐야?" 에 대한 답이니까.
  const handlePickPing = useCallback((eventId, blockId) => {
    const one = eventId ? dangerByIdRef.current[eventId] : null;
    setActiveBlock(BLOCKS.find((b) => b.id === blockId) || null);
    setAutoPopup(false);
    setActiveEvent(one);
    setActiveGroup(one ? [one] : []);
    if (sceneRef.current) sceneRef.current.highlightBlock(blockId);
  }, []);

  // 서버 연결이 끊기거나 돌아오면 알린다.
  //
  // ★ "아직 연결 시도 중" 과 "서버가 죽음" 을 구분해야 한다 (2026-08-29).
  //   connected 는 useState(false) 로 시작한다. 이 false 는 "끊겼다" 가 아니라
  //   "아직 붙기 전" 이다. 그런데 이것을 끊김으로 보면 페이지가 뜰 때마다
  //   false -> true 전환이 일어나 **매번 "재연결되었습니다" 팝업이 떴다.**
  //   확인을 누르면 새로고침되고 같은 일이 반복돼 팝업이 사라지지 않았다.
  //
  //   그래서 상태를 3단계로 든다: null(모름) / true / false.
  //     · 모름 -> 연결됨      : 평상시 첫 연결. 아무 알림도 안 한다
  //     · 모름 -> 유예 끝     : 그제서야 "끊김" 으로 확정하고 알린다
  //     · 연결됨 -> 끊김      : 즉시 알린다
  //     · 끊김 -> 연결됨      : 재연결 알린다
  //
  // ★ 서버가 끊기면 로봇 상태를 "모름" 으로 되돌린다.
  //   로봇 상태는 서버가 알려주는 것이라, 서버가 끊긴 뒤의 "로봇 연결됨" 은
  //   지난 정보다. (상태만 바꾼다 — 로봇 팝업은 jetson_status 를 받을 때만 뜬다)
  useEffect(() => {
    if (connected) {
      const wasDown = serverConnectedRef.current === false;
      serverConnectedRef.current = true;
      if (wasDown) setServerNotice(true);
      return;
    }

    // 여기부터는 connected === false
    robotConnectedRef.current = null;
    setRobotConnected(null);

    if (serverConnectedRef.current === true) {
      // 붙어 있다가 끊겼다 — 확실하다. 바로 알린다.
      serverConnectedRef.current = false;
      setServerNotice(false);
      return;
    }
    if (serverConnectedRef.current === false) return; // 이미 끊김으로 알린 상태

    // 아직 한 번도 못 붙었다. 첫 연결이 늦는 것일 수 있으니 잠깐 기다렸다가
    // 그래도 안 붙으면 그때 "끊김" 으로 확정한다 (서버가 꺼진 채로 대시보드를
    // 여는 경우를 놓치지 않기 위함).
    const t = setTimeout(() => {
      if (serverConnectedRef.current === null) {
        serverConnectedRef.current = false;
        setServerNotice(false);
      }
    }, 4000);
    return () => clearTimeout(t);
  }, [connected]);

  // 씬 초기화
  //
  // ★ 반드시 try 로 감싼다 (2026-08-28).
  //   THREE.WebGLRenderer 는 브라우저가 WebGL 컨텍스트를 못 주면 예외를 던진다.
  //   그런데 여기는 useEffect 안이라, 던진 예외가 React 를 타고 올라가
  //   <ShipyardTwinDashboard> 를 통째로 언마운트시킨다 —— 3D 뷰만 못 쓰는 게
  //   아니라 위험 이벤트 로그·영상·알람까지 전부 사라지고 흰 화면이 된다.
  //
  //   실제로 겪었다: 크롬의 GPU 프로세스가 간헐적으로 실패하면서
  //   "BindToCurrentSequence failed" 로 컨텍스트 생성이 거부됐다. OS 쪽 드라이버는
  //   멀쩡했고(OpenGL 4.6 정상), 크롬을 다시 켜면 되기도 해서 원인을 잡기 어려웠다.
  //
  //   관제 화면이 GPU 딸꾹질 한 번에 통째로 멎으면 안 된다. 3D 만 끄고 나머지는
  //   계속 돌린다. sceneRef 를 쓰는 곳은 전부 이미 null 검사가 있어서 안전하다.
  useEffect(() => {
    let sm;
    try {
      sm = new SceneManager(canvasRef.current,
        { onPickBlock: handlePickBlock, onPickPing: handlePickPing });
    } catch (err) {
      console.error("[3D] 씬 초기화 실패 — 3D 뷰만 끄고 나머지는 계속 동작합니다:", err);
      setSceneError(err);
      return;
    }
    sceneRef.current = sm;
    setSceneError(null);
    const ro = new ResizeObserver(() => sm.resize());
    ro.observe(canvasRef.current.parentElement);
    return () => { ro.disconnect(); sm.dispose(); sceneRef.current = null; };
  }, [handlePickBlock, handlePickPing]);

  // 이벤트 소스 연결 — 위험/경고 이벤트가 감지됐을 때 공통으로 하는 일
  // (3D Ping, 팝업/경고 표시, 로그 적재, 통계). 가짜 소스든 진짜 소스든
  // "정규화된 payload"(cls/blockId/local/conf)만 만들어서 이 함수에 넘기면 된다.
  const handleDetectionEvent = useCallback((payload) => {
    const meta = CLASS_META[payload.cls];
    if (!meta) return; // 모르는 타입은 서버와 동일하게 조용히 무시

    if (sceneRef.current) sceneRef.current.spawnPing(payload);

    if (meta.severity === SEVERITY.DANGER) {
      const block = BLOCKS.find((b) => b.id === payload.blockId);
      // "확인 대기중" 위험 이벤트로 기억해둔다 — 나중에 사용자가 이 블록이나
      // 핑을 직접 클릭해서 봐도(자동 팝업을 놓쳤어도) 사진과 확인 버튼이 뜨게 하기 위함.
      // event_id 가 없는 구버전 메시지는 구획 이름으로 임시 키를 만들어 담는다
      // (그래야 최소한 하나는 남는다 — 다만 같은 구획의 다음 무명 이벤트에 덮인다).
      const key = payload.eventId || `noid:${payload.blockId}`;
      dangerByIdRef.current = { ...dangerByIdRef.current, [key]: { ...payload, _meta: meta } };

      // ★ 재접속 복원분(replay)은 팝업을 띄우지 않는다 (2026-08-27).
      //   이미 관제사가 확인했을 수도 있고, 살아있는 이벤트가 여러 개면
      //   새로고침할 때마다 팝업이 연달아 떠서 화면을 덮는다.
      //   핑과 블록 강조는 그대로 그린다 — 위험이 아직 그 자리에 있다는
      //   사실 자체는 보여줘야 하기 때문이다. 위의 dangerByIdRef 기록도
      //   그대로 둔다. 그래야 재접속 후 그 블록을 클릭했을 때 확인 버튼이 뜬다.
      if (!payload.replay) {
        setActiveBlock(block);
        setActiveEvent({ ...payload, _meta: meta });
        setActiveGroup([{ ...payload, _meta: meta }]);
        setAutoPopup(true);
        setUgvBlock(block);
      }
      if (sceneRef.current) sceneRef.current.highlightBlock(payload.blockId);
    } else if (meta.severity === SEVERITY.WARN) {
      const block = BLOCKS.find((b) => b.id === payload.blockId);
      setUgvBlock(block);
      setWarnEvent({ ...payload, _meta: meta });
      clearTimeout(warnTimer.current);
      warnTimer.current = setTimeout(() => setWarnEvent(null), 6000);
    }

    setEvents((prev) => [{ ...payload, _meta: meta }, ...prev].slice(0, 40));
    setStats((s) => ({ ...s, [meta.severity]: s[meta.severity] + 1 }));
  }, []);

  useEffect(() => {
    if (!USE_REAL_BACKEND) {
      // === 가짜 데이터 모드 (기존과 동일) ===
      setConnected(true);
      const off = connectEventSource((payload) => {
        if (payload.cls === "ship_block") return; // 실제 스펙엔 없는 타입이라 무시
        handleDetectionEvent(payload);
      });
      return () => { off(); clearTimeout(warnTimer.current); };
    }

    // === 진짜 백엔드 모드 ===
    console.log(`[이벤트 채널] 연결 시도: ${EVENT_WS_URL}`);
    const off = connectRealEventSource(EVENT_WS_URL, {
      onOpen: () => {
        console.log("[이벤트 채널] 연결 성공");
        setConnected(true);
        // 연결이 끊긴 동안 놓쳤을 수 있는 갱신(배 위치/조립 단계)을 따라잡기 위해
        // (재)연결될 때마다 최신 스냅샷을 다시 읽는다.
        fetchInitData();
      },
      onClose: () => { console.log("[이벤트 채널] 연결 끊김"); setConnected(false); },
      onError: () => setConnected(false),
      onMessage: (data) => {
        const type = data.event_type;

        // 디버깅용: 어떤 event_type이 실제로 도착하는지 콘솔에 타입당 1번만 찍는다.
        // (F12 → Console 탭에서 확인. 서버 연결은 됐는데 화면이 안 바뀔 때
        //  "메시지가 아예 안 오는지" vs "와도 처리를 안 하는지"를 구분하는 용도.)
        if (!seenEventTypesRef.current.has(type)) {
          seenEventTypesRef.current.add(type);
          console.log(`[이벤트 채널] 새 타입 최초 수신: "${type}"`, data);
        }

        // ① 배 위치/방향 — 좌표 변환에만 쓰고 화면엔 직접 안 그림
        if (type === "ship_pose") {
          shipPoseRef.current = { map_xy: data.map_xy, yaw: data.yaw, block_id: data.block_id };
          console.log("[이벤트 채널] ship_pose 수신 — 이제부터 UGV 좌표 변환 가능", shipPoseRef.current);
          return;
        }

        // ⓪ UGV 자체 위치(position 핑, 0.5초 주기) — ship_pose와 조합해서
        // 3D 화면의 UGV를 실제 위치로 움직인다. ship_pose를 아직 한 번도
        // 못 받았으면(배 기준점을 모름) 변환할 수 없으니 조용히 건너뛴다.
        if (type === "position") {
          if (!shipPoseRef.current) {
            console.warn(
              "[이벤트 채널] position은 왔는데 ship_pose가 아직 없어서 UGV를 못 움직임 — " +
              "젯슨의 ship_survey_node가 배 위치 측량을 마쳤는지 확인 필요"
            );
            return;
          }
          if (sceneRef.current) {
            const world = mapXYToUgvWorld(data.ekf_global, data.yaw, shipPoseRef.current);
            if (world) sceneRef.current.setUgvPosition(world.x, world.z, world.yaw);
          }
          return;
        }

        // ③ 조립 단계 — Ping이 아니라 공정률 색상으로 반영
        // ⚠️ level은 1~5 (5단계 만점, /3이 아니라 /5), block_id("B1")는 BLOCKS의
        // S1~S5(구획)가 아니라 배 자체의 id라서 그대로 매칭하면 아무 데도 안 붙는다.
        // applyStageProgress가 "몇 번째 구획까지 완성"으로 알아서 변환해준다.
        if (type === "block_level") {
          applyStageProgress(data.level);
          return;
        }

        // ④ 위험 해제 — 로봇이 그 자리를 다시 지나가면서 확인했는데 대상이 이미 없어졌을 때
        // 젯슨이 보냄: {"event_type":"event_cleared","block_id":"B1","cls":"fire","map_xy":[...],"event_id":"fire@0.40,-0.99"}
        // 해당 event_id를 가진 핑만 화면에서 지운다 (핑 수명을 이제 프론트가 아니라 젯슨이 관리).
        if (type === "event_cleared") {
          if (sceneRef.current) {
            sceneRef.current.clearPingByEventId(data.event_id ?? null, { blockId: data.block_id, cls: data.cls });
          }
          // "확인 대기중" 기록도 같은 event_id일 때만 같이 정리 — 그 사이에 같은 구획에서
          // 다른 새 위험이 또 감지됐다면(다른 event_id) 그건 그대로 남겨둬야 하니까.
          {
            // event_id 로 그 한 건만 지운다. 같은 구획의 다른 위험은 그대로 남는다.
            const next = { ...dangerByIdRef.current };
            if (data.event_id) delete next[data.event_id];
            else if (data.block_id) delete next[`noid:${data.block_id}`];
            dangerByIdRef.current = next;
          }
          return;
        }

        // ⑥ 로봇 연결 상태 — 서버가 알려준다. {"connected": true/false}
        //
        // 로봇이 꺼져도 화면의 위험 핑은 그대로 남는다(위험이 사라진 게 아니므로
        // 맞는 동작이다). 그래서 화면만 봐서는 실시간 정보가 멈춘 걸 알 수 없다.
        //
        // ★ 팝업은 "상태가 바뀌었을 때"만 띄운다. 다만 대시보드를 여는 순간
        //   이미 끊겨 있으면 그것도 알려야 하므로, 첫 수신이어도 끊김이면 띄운다.
        //   (첫 수신이 "연결됨" 이면 평상시라 조용히 넘어간다 — 이 규칙 덕분에
        //    재연결 팝업의 새로고침이 무한 반복되지 않는다)
        //
        //   문구는 언제나 "재연결"로 둔다. 로봇이 정말 처음 켜지는 순간은 현장에
        //   처음 들어온 그때뿐이고, 그 외에는 늘 이전 세션의 기억을 들고 다시
        //   붙는 것이라 "재연결"이 사실에 맞다.
        if (type === "jetson_status") {
          const now = data.connected === true;
          const prev = robotConnectedRef.current;
          robotConnectedRef.current = now;
          setRobotConnected(now); // 상단 배지는 팝업과 무관하게 항상 최신으로
          if (prev === null ? !now : prev !== now) {
            // 이전 알림이 아직 떠 있어도 그냥 덮어쓴다 — 최신 상태가 항상 이긴다.
            // (끊김 팝업을 안 닫은 채로 재연결되면 자동으로 재연결 팝업으로 바뀐다)
            setRobotNotice(now);
          }
          return;
        }

        // ⑤ 감지 순간 사진 — 위험 이벤트 **바로 뒤에** 같은 event_id 로 따라온다.
        // {"event_type":"event_snapshot","block_id":"B1","event_id":"fire@0.63,-0.23",
        //  "cls":"fire","image_url":"/snapshots/f5edf801....jpg"}
        //
        // ★ 사진 자체(base64)는 여기까지 오지 않는다. 서버가 파일로 떨궈두고
        //   주소만 알려주므로, 브라우저가 그 주소를 캐시해 두 번째부터는 안 받는다.
        //   그래서 이 메시지는 아주 가볍고, 새 이벤트를 만들지 않는다 —
        //   이미 올라가 있는 그 이벤트를 찾아 사진만 덧붙이는 것이다.
        if (type === "event_snapshot") {
          const eid = data.event_id ?? null;
          const url = data.image_url ? SNAPSHOT_BASE_URL + data.image_url : null;
          if (!eid || !url) return;

          // 짝이 안 맞으면 원본을 그대로 돌려준다 → React가 헛 렌더하지 않는다.
          const attach = (e) => (e && e.eventId === eid ? { ...e, imageUrl: url } : e);

          setEvents((prev) => prev.map(attach));
          setActiveEvent(attach); // 팝업이 이미 떠 있으면 그 자리에서 사진이 채워진다
          setWarnEvent(attach);

          // 클릭용 기록에도 붙인다 — 나중에 핑/구획을 눌렀을 때 사진이 나와야 한다.
          if (dangerByIdRef.current[eid]) {
            dangerByIdRef.current = {
              ...dangerByIdRef.current,
              [eid]: { ...dangerByIdRef.current[eid], imageUrl: url },
            };
          }
          // 지금 열려 있는 팝업의 사진 목록에도 즉시 반영
          setActiveGroup((prev) => prev.map(attach));
          return;
        }

        // ② 위험 이벤트(fallen_person/fire/no_helmet/ship_defect)
        const meta = CLASS_META[type];
        if (!meta) return; // event_ack 등 프론트가 보낸 종류가 되돌아오면 무시

        // 배 위(구획 안)인지 배 밖(작업장)인지 먼저 판단 — 배 밖인 화재를 억지로
        // 구획에 눌러 붙이면 "화재가 배에서 떨어져 있는데 배 위에 핑이 찍힌다"가 된다.
        const conv = mapXYToPingWorld(data.map_xy ?? null, shipPoseRef.current);
        const blockId = conv?.blockId ?? shipPoseRef.current?.block_id ?? BLOCKS[0].id;

        // 🚫 배에서 너무 먼 검출은 버린다 (MAX_EVENT_DIST_FROM_SHIP_M 참고).
        //   조용히 버리지 않고 반드시 콘솔에 남긴다 — 진짜 이벤트가 사라졌을 때
        //   "왜 안 뜨지" 를 여기서 바로 확인할 수 있어야 한다.
        {
          const r = mapXYToShipLocalMeters(data.map_xy ?? null, shipPoseRef.current);
          if (r) {
            const dist = Math.hypot(r.forward, r.beam);
            if (dist > MAX_EVENT_DIST_FROM_SHIP_M) {
              console.warn(
                `[이벤트 걸러냄] ${type} 이 배에서 ${dist.toFixed(2)}m 떨어져 있어 무시함 ` +
                `(한계 ${MAX_EVENT_DIST_FROM_SHIP_M}m). 카메라는 순찰 원 안쪽을 보므로 ` +
                `이 거리는 오검출일 가능성이 높다.`, data
              );
              return;
            }
          }
        }

        // 📏 핑 위치 보정용 로그. 불을 배 정중앙에 놓고 이 줄의 "원본" 값을 읽어
        //   PING_OFFSET_* 상수에 부호를 뒤집어 넣으면 된다 (상수 설명 참고).
        const relDbg = mapXYToShipLocalMeters(data.map_xy ?? null, shipPoseRef.current);
        if (relDbg) {
          const f = (n) => (n >= 0 ? "+" : "") + n.toFixed(2);
          console.log(
            `[핑 보정] ${type}  원본 앞뒤=${f(relDbg.rawForward)} 좌우=${f(relDbg.rawBeam)}` +
            `  →  표시 앞뒤=${f(relDbg.forward)} 좌우=${f(relDbg.beam)}` +
            `  (현재 보정 앞뒤=${f(PING_OFFSET_FORWARD_M)} 좌우=${f(PING_OFFSET_BEAM_M)})`
          );
        }

        handleDetectionEvent({
          id: `evt_${Date.now()}_${Math.floor(Math.random() * 1e4)}`,
          ts: Date.now(),
          cls: type,
          blockId,
          // 배 위면 구획 기준 로컬 좌표, 배 밖이면 야드 월드 좌표를 직접 넘긴다 —
          // spawnPing이 onShip 값을 보고 둘 중 알맞은 쪽으로 위치를 계산한다.
          onShip: conv?.onShip ?? true,
          local: conv?.onShip ? conv.local : { x: 0.5, y: 0.6, z: 0.5 },
          worldX: conv && conv.onShip === false ? conv.worldX : null,
          worldZ: conv && conv.onShip === false ? conv.worldZ : null,
          conf: data.confidence ?? 0,
          eventId: data.event_id ?? null, // 젯슨이 만든 고유 id — 나중에 event_cleared로 이 핑만 콕 집어 지울 때 씀
          // 사람이 읽는 위치 문구("S3 왼편"). 지금 배 위치를 알고 있을 때 미리
          // 계산해 붙여둔다 — 나중에 배가 다시 측량돼 좌표계가 조금 달라져도
          // 그때 찍힌 위치 그대로 남아 로그의 기록성이 유지된다.
          locLabel: describePingLocation(data.map_xy ?? null, shipPoseRef.current, blockId),
          // 감지 순간 사진. 실시간에는 아직 없고(사진은 곧 이어서 별도 메시지로 온다),
          // 재접속 복원으로 온 이벤트에는 서버가 image_url 을 붙여서 보내준다.
          imageUrl: data.image_url ? SNAPSHOT_BASE_URL + data.image_url : null,
          // 서버가 재접속 복원으로 다시 보내준 것인지. 새로 감지된 것이 아니므로
          // 핑은 그리되 팝업은 띄우지 않는다 (handleDetectionEvent 참고).
          replay: data.replay === true,
        });
      },
    });
    wsControlRef.current = off; // event_ack 보낼 때 이 소켓으로 보냄

    return () => {
      off();
      wsControlRef.current = null;
      clearTimeout(warnTimer.current);
    };
  }, [handleDetectionEvent]);

  // 초기 공정률을 씬에 반영
  useEffect(() => {
    if (!sceneRef.current) return;
    Object.entries(progress).forEach(([id, v]) => sceneRef.current.setBlockProgress(id, v));
    // eslint-disable-next-line
  }, [sceneRef.current]);

  const openBlockView = (event) => {
    const block = BLOCKS.find((b) => b.id === event.blockId);
    setActiveBlock(block);
    setActiveEvent(event);
    setAutoPopup(false);
    setUgvBlock(block);
    if (sceneRef.current) sceneRef.current.highlightBlock(event.blockId);
  };

  const closePopup = () => {
    setActiveBlock(null); setActiveEvent(null); setAutoPopup(false);
    setActiveGroup([]);
    if (sceneRef.current) sceneRef.current.highlightBlock(null);
  };

  // 위험 이벤트 팝업의 "확인" 버튼 — 지금 쓰는 이벤트 소켓(/ws/frontend) 그대로
  // {"event_type":"event_ack"} 한 줄만 보내고 팝업을 닫는다. 새 연결/새 API 없음.
  // 응답은 안 기다림 — 서버가 젯슨으로 전달, 젯슨이 Nav2에 순찰 재개 신호를 보냄(백엔드 완료).
  // 위험물이 안 치워졌으면 다음 바퀴(~40초 후)에 팝업이 다시 뜨는 게 의도된 동작.
  // "확인"은 오직 event_ack만 보내고 팝업만 닫는다 — 화면의 핑/구획 강조/
  // "확인 대기중" 기록은 여기서 건드리지 않는다. 그 위험이 실제로 없어졌는지는
  // 로봇이 다시 가서 눈으로 확인해야 아는 거라서, 핑을 지우는 건 오직 젯슨이
  // 보내는 event_cleared뿐이다(관제사가 확인 버튼을 눌렀다고 위험이 사라진 건 아니니까).
  const handleAck = () => {
    wsControlRef.current?.send?.({ event_type: "event_ack" });
    console.log("[이벤트 채널] event_ack 전송");
    closePopup();
  };

  // 팝업의 "핑 직접 지우기" 버튼 — event_ack과 달리 서버에 아무것도 안 보낸다.
  // 관제사가 화면(영상)으로 봤을 때 이미 처리된 게 확실하다고 판단해서 직접 지우는
  // 용도. 젯슨의 event_cleared를 기다리지 않고 화면에서만 즉시 없앤다.
  const handleClearPing = () => {
    if (activeBlock) {
      // 그 구획에 걸린 위험 기록을 전부 뺀다 (clearBlockPing 도 구획 단위로 지운다).
      const next = {};
      for (const [k, e] of Object.entries(dangerByIdRef.current)) {
        if (e.blockId !== activeBlock.id) next[k] = e;
      }
      dangerByIdRef.current = next;
      if (sceneRef.current) sceneRef.current.clearBlockPing(activeBlock.id);
    }
    closePopup();
  };

  const dangerCount = stats.danger;
  const sortedBlocks = useMemo(
    () => BLOCKS.map((b) => ({ ...b, p: progress[b.id] ?? 0 })), [progress]);

  return (
    <div className="app">
      <style>{CSS}</style>

      {/* 헤더 */}
      <header className="topbar">
        <div className="brand">
          <div className="brand-mark">◣◥</div>
          <div>
            <div className="brand-title">SMART SHIPYARD TWIN</div>
            <div className="brand-sub">선박 건조 공정 트래킹 · 디지털 트윈 관제</div>
          </div>
        </div>
        <div className="top-status">
          {/* 서버 연결과 로봇 연결은 다른 것이다 — 서버는 붙어 있는데 로봇만
              꺼져 있는 상황이 흔하므로 배지를 따로 둔다. 팝업의 확인을 눌러
              닫았어도 여기는 계속 남아 있어 지금 상태를 언제든 볼 수 있다. */}
          <span className={`conn ${connected ? "on" : ""}`}>
            <span className="conn-dot" />{connected ? "서버 연결됨" : "서버 끊김"}
          </span>
          <span
            className={`conn ${robotConnected === true ? "on" : robotConnected === false ? "warn" : ""}`}
            title={
              robotConnected === false
                ? "로봇과 재연결될 때까지 실시간 순찰 정보 관제가 제한됩니다"
                : robotConnected === true
                  ? "로봇이 순찰 정보를 보내오고 있습니다"
                  : "아직 로봇 상태를 받지 못했습니다"
            }
          >
            <span className="conn-dot" />
            {robotConnected === true ? "로봇 연결됨"
              : robotConnected === false ? "로봇 끊김" : "로봇 상태 확인 중"}
          </span>
          <span className="clock-badge">{new Date().toLocaleDateString("ko-KR")}</span>
        </div>
      </header>

      <div className="body">
        {/* 좌측: KPI + 공정률 */}
        <aside className="left">
          <div className="panel">
            <div className="panel-h">실시간 위험 요약</div>
            <div className="kpis">
              <Kpi label="위험" value={stats.danger} color={SEV_COLOR.danger} pulse={dangerCount > 0} />
              <Kpi label="경고" value={stats.warn} color={SEV_COLOR.warn} />
              <Kpi label="정상" value={stats.info} color={SEV_COLOR.info} />
            </div>
          </div>

          <div className="panel grow">
            <div className="panel-h">선박 구획별 공정률</div>
            <div className="progress-list">
              {sortedBlocks.map((b) => (
                <button key={b.id} className="prog-row" onClick={() => handlePickBlock(b.id)}>
                  <div className="prog-name">
                    <span className="prog-id">{b.id}</span> {b.name}
                  </div>
                  <div className="prog-bar">
                    <div className="prog-fill" style={{
                      width: `${(b.p * 100).toFixed(0)}%`,
                      background: progressFill(b.p),
                    }} />
                  </div>
                  <div className="prog-pct">{(b.p * 100).toFixed(0)}%</div>
                </button>
              ))}
            </div>
            <div className="legend">
              <span><i style={{ background: "#6b7280" }} />미조립</span>
              <span><i style={{ background: "#eab308" }} />진행</span>
              <span><i style={{ background: "#22c55e" }} />완료</span>
            </div>
          </div>
        </aside>

        {/* 중앙: 3D 디지털 트윈 */}
        <main className="stage">
          <div className="stage-tag">디지털 트윈 관제 뷰 · 드래그 회전 / 스크롤 줌 / 구획 클릭</div>
          <canvas ref={canvasRef} className="three-canvas" />
          {sceneError && (
            <div className="stage-fallback">
              <div className="stage-fallback-title">3D 뷰를 띄우지 못했습니다</div>
              <p>
                브라우저가 WebGL(그래픽 가속)을 쓸 수 없는 상태입니다.
                <strong> 나머지 관제 기능은 정상 동작합니다</strong> — 이벤트 로그,
                실시간 영상, 알람은 그대로 받고 있습니다.
              </p>
              <p className="stage-fallback-how">
                되살리려면: 크롬을 완전히 껐다 켜기 → 그래도 안 되면
                <code> chrome://settings</code> 에서 &ldquo;그래픽 가속 사용&rdquo; 확인 →
                <code> chrome://gpu</code> 에서 WebGL 상태 보기
              </p>
            </div>
          )}
          <div className="stage-hint">배의 구획을 클릭하면 해당 구역 CCTV가 열립니다 (Click &amp; View)</div>
          <LivePanel
            ugvBlock={ugvBlock}
            warnEvent={warnEvent}
            onExpand={() => {
              setActiveBlock(ugvBlock);
              setActiveEvent(warnEvent);
              setAutoPopup(false);
              if (sceneRef.current && ugvBlock) sceneRef.current.highlightBlock(ugvBlock.id);
            }}
          />
        </main>

        {/* 우측: 이벤트 로그 */}
        <aside className="right">
          <div className="panel grow">
            <div className="panel-h">
              위험 이벤트 로그
              <span className="log-count">{events.length}</span>
            </div>
            <div className="log">
              {events.length === 0 && <div className="log-empty">이벤트 수신 대기 중…</div>}
              {events.map((e) => (
                <button key={e.id} className={`log-row sev-${e._meta.severity}`} onClick={() => openBlockView(e)}>
                  <span className="log-dot" style={{ background: SEV_COLOR[e._meta.severity] }} />
                  <span className="log-cls">{e._meta.label}</span>
                  {/* "S3"보다 "S3 왼편"이 관제사에게 쓸모 있다. 배 위치를 아직
                      못 받아 문구를 못 만든 경우에만 구획 id로 되돌아간다. */}
                  <span className="log-block">{e.locLabel || e.blockId}</span>
                  <span className="log-conf">{(e.conf * 100).toFixed(0)}%</span>
                  <span className="log-time">{new Date(e.ts).toLocaleTimeString("ko-KR", { hour12: false })}</span>
                </button>
              ))}
            </div>
          </div>
        </aside>
      </div>

      {(robotNotice !== null || serverNotice !== null) && (
        <div className="notice-backdrop">
          {/* 로봇과 서버를 나란히. 로봇을 먼저 둔다 — 원인이 로봇 쪽일 때
              그것이 먼저 눈에 들어와야 한다. */}
          {robotNotice !== null && (
            <ConnNotice
              tone={robotNotice ? "ok" : "lost"}
              title={robotNotice ? "🤖 로봇과 재연결되었습니다" : "⚠️ 로봇과의 연결이 끊겼습니다"}
              lines={robotNotice
                ? ["실시간 순찰 정보 관제를 재개합니다.",
                   "확인을 누르면 화면을 새로 불러와 로봇의 최신 정보를 반영합니다."]
                : ["로봇과 재연결될 때까지 실시간 순찰 정보 관제가 제한됩니다.",
                   "이미 감지된 핑과 로봇의 마지막 위치는 화면에 그대로 남아 있습니다."]}
              btnLabel={robotNotice ? "확인 — 최신 정보 불러오기" : "확인"}
              onConfirm={() => {
                if (robotNotice) window.location.reload();
                else setRobotNotice(null);
              }}
            />
          )}
          {serverNotice !== null && (
            <ConnNotice
              tone={serverNotice ? (robotConnected ? "ok" : "warn") : "lost"}
              title={serverNotice ? "🖥️ 서버와 재연결되었습니다" : "⚠️ 서버와의 연결이 끊겼습니다"}
              lines={
                serverNotice
                  ? (robotConnected
                      // 서버도 로봇도 붙었다 — 완전히 정상으로 돌아온 경우
                      ? ["실시간 순찰 정보 관제를 재개합니다.",
                         "확인을 누르면 화면을 새로 불러와 로봇의 최신 정보를 반영합니다."]
                      // 서버만 붙었다 — 아직 관제가 안 된다는 것을 분명히 말한다
                      : ["다만 로봇이 아직 연결되지 않았습니다.",
                         "로봇이 연결된 후에 실시간 순찰 정보 관제가 가능합니다."])
                  : ["서버와 재연결될 때까지 실시간 순찰 정보 관제가 제한됩니다.",
                     "이미 감지된 핑과 로봇의 마지막 위치는 화면에 그대로 남아 있습니다."]
              }
              btnLabel={serverNotice && robotConnected ? "확인 — 최신 정보 불러오기" : "확인"}
              onConfirm={() => {
                // 로봇까지 붙어 있을 때만 새로고침한다. 서버만 붙은 상태에서
                // 새로 불러와봐야 가져올 최신 정보가 없다.
                if (serverNotice && robotConnected) window.location.reload();
                else setServerNotice(null);
              }}
            />
          )}
        </div>
      )}

      <CctvPopup
        block={activeBlock}
        event={activeEvent}
        group={activeGroup}
        auto={autoPopup}
        onClose={closePopup}
        onAck={handleAck}
        onClearPing={handleClearPing}
      />
    </div>
  );
}

function Kpi({ label, value, color, pulse }) {
  return (
    <div className={`kpi ${pulse ? "kpi-pulse" : ""}`}>
      <div className="kpi-val" style={{ color }}>{value}</div>
      <div className="kpi-label">{label}</div>
    </div>
  );
}

function progressFill(p) {
  if (p < 0.02) return "#6b7280";
  if (p < 0.5) return "linear-gradient(90deg,#6b7280,#eab308)";
  return "linear-gradient(90deg,#eab308,#22c55e)";
}

/* ---------------------------------------------------------------------------
 * 6. 스타일
 * ------------------------------------------------------------------------- */
const CSS = `
* { box-sizing: border-box; }
.app {
  --bg:#070b12; --panel:#0e1420; --panel2:#121a28;
  --line:#1e2a3f; --text:#e6edf6; --muted:#7d8aa3; --teal:#2dd4bf;
  position:absolute; inset:0; display:flex; flex-direction:column;
  background:#070b12; color:#e6edf6;
  font-family:'Pretendard',-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
  overflow:hidden;
}
.topbar {
  display:flex; align-items:center; justify-content:space-between;
  padding:12px 20px; background:linear-gradient(180deg,#0e1420,#0a0f18);
  border-bottom:1px solid #1e2a3f; flex:0 0 auto;
}
.brand { display:flex; align-items:center; gap:13px; }
.brand-mark {
  font-size:20px; color:#2dd4bf; letter-spacing:-3px;
  text-shadow:0 0 14px rgba(45,212,191,.5);
}
.brand-title { font-weight:800; letter-spacing:2px; font-size:15px; }
.brand-sub { font-size:11px; color:#7d8aa3; margin-top:2px; }
.top-status { display:flex; align-items:center; gap:12px; }
.conn { display:flex; align-items:center; gap:7px; font-size:12px; color:#7d8aa3;
  border:1px solid #1e2a3f; padding:5px 11px; border-radius:20px; }
.conn.on { color:#36d399; }
/* 로봇이 끊긴 상태 — 빨강(위험)이 아니라 노랑(주의)으로 둔다.
   빨강은 위험 이벤트 색이라, 연결 문제와 화재를 같은 색으로 두면 헷갈린다. */
.conn.warn { color:#ffb020; border-color:#3a2f16; }
.conn.warn .conn-dot { background:#ffb020; box-shadow:0 0 8px #ffb020; animation:blink 1.4s infinite; }
.conn-dot { width:7px; height:7px; border-radius:50%; background:#ff3b47; }
.conn.on .conn-dot { background:#36d399; box-shadow:0 0 8px #36d399; animation:blink 2s infinite; }
.clock-badge { font-size:12px; color:#7d8aa3; font-variant-numeric:tabular-nums; }

.body { flex:1 1 auto; display:grid; grid-template-columns:280px 1fr 320px; gap:12px; padding:12px; min-height:0; }
.left,.right { display:flex; flex-direction:column; gap:12px; min-height:0; }

.panel { background:#0e1420; border:1px solid #1e2a3f; border-radius:12px; padding:14px; display:flex; flex-direction:column; min-height:0; }
.panel.grow { flex:1 1 auto; }
.panel-h { font-size:12px; font-weight:700; letter-spacing:1px; color:#aebbd2; text-transform:uppercase;
  margin-bottom:12px; display:flex; align-items:center; justify-content:space-between; }

.kpis { display:grid; grid-template-columns:repeat(3,1fr); gap:8px; }
.kpi { background:#121a28; border:1px solid #1e2a3f; border-radius:9px; padding:12px 6px; text-align:center; }
.kpi-val { font-size:26px; font-weight:800; font-variant-numeric:tabular-nums; line-height:1; }
.kpi-label { font-size:11px; color:#7d8aa3; margin-top:6px; }
.kpi-pulse { animation:dangerPulse 1.1s infinite; border-color:#ff3b47; }
@keyframes dangerPulse { 0%,100%{box-shadow:0 0 0 rgba(255,59,71,0)} 50%{box-shadow:0 0 16px rgba(255,59,71,.45)} }

.progress-list { display:flex; flex-direction:column; gap:9px; overflow-y:auto; flex:1 1 auto; }
.prog-row { display:grid; grid-template-columns:1fr 70px 36px; align-items:center; gap:8px;
  background:none; border:none; padding:7px 6px; border-radius:8px; cursor:pointer; text-align:left;
  color:#e6edf6; transition:background .15s; }
.prog-row:hover { background:#121a28; }
.prog-name { font-size:12px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.prog-id { color:#2dd4bf; font-weight:700; font-size:11px; margin-right:4px; }
.prog-bar { height:7px; background:#1a2336; border-radius:5px; overflow:hidden; }
.prog-fill { height:100%; border-radius:5px; transition:width .5s ease; }
.prog-pct { font-size:11px; color:#7d8aa3; text-align:right; font-variant-numeric:tabular-nums; }
.legend { display:flex; gap:14px; margin-top:12px; padding-top:12px; border-top:1px solid #1e2a3f; }
.legend span { display:flex; align-items:center; gap:5px; font-size:11px; color:#7d8aa3; }
.legend i { width:10px; height:10px; border-radius:3px; display:inline-block; }

.stage { position:relative; background:#0a0e17; border:1px solid #1e2a3f; border-radius:12px; overflow:hidden; min-height:0; }
.three-canvas { width:100%; height:100%; display:block; cursor:grab; }
.three-canvas:active { cursor:grabbing; }
.stage-tag { position:absolute; top:12px; left:12px; z-index:2; font-size:11px; color:#aebbd2;
  background:rgba(10,14,23,.7); border:1px solid #1e2a3f; padding:5px 10px; border-radius:7px; backdrop-filter:blur(6px); }
.stage-hint { position:absolute; bottom:12px; left:50%; transform:translateX(-50%); z-index:2;
  font-size:11px; color:#7d8aa3; background:rgba(10,14,23,.7); padding:5px 12px; border-radius:7px; backdrop-filter:blur(6px); }

.live-panel { position:absolute; bottom:12px; right:12px; z-index:3; width:420px;
  background:rgba(10,14,23,.92); border:1px solid #1e2a3f; border-radius:10px; overflow:hidden;
  cursor:pointer; transition:transform .15s, border-color .2s; box-shadow:0 8px 30px rgba(0,0,0,.5); }
.live-panel:hover { transform:translateY(-2px); border-color:#2dd4bf; }
.live-head { display:flex; align-items:center; gap:7px; padding:7px 10px; font-size:11px; font-weight:600;
  color:#aebbd2; border-bottom:1px solid #1e2a3f; }
.live-dot { width:7px; height:7px; border-radius:50%; background:#36d399; box-shadow:0 0 8px #36d399; animation:blink 2s infinite; }
.live-canvas { display:block; width:100%; height:auto; }

/* direct 모드(백엔드 안 거치고 카메라 컴퓨터에 바로 접속) 전용 스타일 */
.direct-video-wrap { position:relative; width:100%; aspect-ratio:420/236; background:#0b0f16; overflow:hidden; }
.direct-video-frame { display:block; width:100%; height:100%; border:none; background:#0b0f16; }
.direct-video-reload { position:absolute; right:8px; top:6px; z-index:2; background:rgba(10,14,23,.75);
  border:1px solid #1e2a3f; color:#aebbd2; font-size:11px; padding:3px 8px; border-radius:6px; cursor:pointer; }
.direct-video-reload:hover { border-color:#2dd4bf; color:#2dd4bf; }
.direct-video-fallback { position:absolute; inset:0; display:flex; flex-direction:column; align-items:center;
  justify-content:center; gap:5px; color:#7d8aa3; font-size:11px; text-align:center; padding:10px; }
.direct-video-url { color:#aebbd2; font-family:monospace; font-size:10px; word-break:break-all; }
.direct-video-retry { color:#5a6580; font-size:10px; }
.direct-video-badge { position:absolute; left:8px; bottom:8px; padding:3px 8px; border-radius:4px;
  font-size:11px; font-weight:700; color:#0a0e17; }
.direct-video-hud { position:absolute; left:8px; top:6px; color:#2dd4bf; font-size:11px; font-family:monospace;
  text-shadow:0 1px 3px rgba(0,0,0,.8); }
.live-warn-tag { margin-left:auto; color:#ffb020; font-weight:700; }
.live-panel.live-warn { border-color:#ffb020; animation:warnPulse 1s infinite; }
@keyframes warnPulse { 0%,100%{box-shadow:0 0 0 rgba(255,176,32,0)} 50%{box-shadow:0 0 18px rgba(255,176,32,.5)} }

.popup-auto { border-color:#ff3b47; animation:autoPulse 1.1s infinite; }
@keyframes autoPulse { 0%,100%{box-shadow:0 24px 80px rgba(0,0,0,.6)} 50%{box-shadow:0 0 40px rgba(255,59,71,.5)} }

.log { display:flex; flex-direction:column; gap:5px; overflow-y:auto; flex:1 1 auto; }
.log-empty { color:#5a6580; font-size:12px; text-align:center; padding:30px 0; }
.log-count { background:#1a2336; color:#aebbd2; font-size:11px; padding:2px 8px; border-radius:10px; }
.log-row { display:grid; grid-template-columns:auto 1fr auto auto auto; align-items:center; gap:8px;
  background:#121a28; border:1px solid #1e2a3f; border-left:3px solid #1e2a3f; border-radius:7px;
  padding:8px 10px; cursor:pointer; text-align:left; color:#e6edf6; transition:transform .1s,background .15s; }
.log-row:hover { background:#16203180; transform:translateX(-2px); }
.log-row.sev-danger { border-left-color:#ff3b47; }
.log-row.sev-warn { border-left-color:#ffb020; }
.log-row.sev-info { border-left-color:#36d399; }
.log-dot { width:8px; height:8px; border-radius:50%; }
.log-cls { font-size:12px; font-weight:600; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.log-block { font-size:11px; color:#2dd4bf; font-weight:700; white-space:nowrap; }
.log-conf { font-size:11px; color:#7d8aa3; font-variant-numeric:tabular-nums; }
.log-time { font-size:10px; color:#5a6580; font-variant-numeric:tabular-nums; }

.popup-backdrop { position:absolute; inset:0; background:rgba(4,7,12,.72); backdrop-filter:blur(4px);
  display:flex; align-items:center; justify-content:center; z-index:50; animation:fade .15s ease; }
.popup { width:800px; max-width:94vw; background:#0e1420; border:1px solid #25344e; border-radius:14px;
  overflow:hidden; box-shadow:0 24px 80px rgba(0,0,0,.6); animation:pop .18s ease; }
.popup-head { display:flex; align-items:center; justify-content:space-between; padding:13px 16px;
  border-bottom:1px solid #1e2a3f; }
.popup-title { display:flex; align-items:center; gap:9px; font-size:13px; font-weight:700; letter-spacing:.5px; }
.rec-dot { width:9px; height:9px; border-radius:50%; background:#ff3b47; box-shadow:0 0 8px #ff3b47; animation:blink 1.2s infinite; }
.popup-esc-hint { font-size:11px; color:#7d8aa3; border:1px solid #1e2a3f; padding:4px 10px; border-radius:6px; }
.popup-canvas { display:block; width:100%; height:auto; background:#0c1118; }
.popup-meta { padding:14px 16px; display:grid; grid-template-columns:1fr 1fr; gap:9px 18px; }
.popup-meta > div { display:flex; justify-content:space-between; font-size:12px; border-bottom:1px solid #161f2e; padding-bottom:7px; }
.popup-meta .k { color:#7d8aa3; }
.popup-meta .v { color:#e6edf6; font-weight:600; }

/* 3D 를 못 띄웠을 때 캔버스 자리에 덮는 안내. 나머지 관제 기능은 살아 있다. */
.stage-fallback { position:absolute; inset:0; display:flex; flex-direction:column;
  align-items:center; justify-content:center; gap:10px; padding:24px; text-align:center;
  background:#0a0e17; color:#7d8aa3; font-size:13px; line-height:1.6; z-index:2; }
.stage-fallback-title { color:#ffb020; font-size:15px; font-weight:700; }
.stage-fallback p { margin:0; max-width:520px; }
.stage-fallback strong { color:#e6edf6; }
.stage-fallback-how { font-size:12px; color:#5f6b80; }
.stage-fallback code { color:#2dd4bf; font-family:monospace; }

/* 연결 알림. CCTV 팝업보다 위에 떠서 먼저 눈에 들어오게 한다.
   로봇/서버가 동시에 끊기면 두 장이 나란히 뜬다(좁은 화면에서는 세로로 쌓임). */
.notice-backdrop { position:fixed; inset:0; background:rgba(4,7,12,.72);
  display:flex; align-items:center; justify-content:center; gap:14px;
  flex-wrap:wrap; padding:20px; z-index:60; }
.conn-notice { width:min(380px, 90vw); background:#0e1420; border:1px solid #1e2a3f;
  border-radius:12px; padding:22px 24px; text-align:center;
  box-shadow:0 18px 50px rgba(0,0,0,.55); }
.conn-notice.lost { border-color:#ffb020; }
.conn-notice.warn { border-color:#ffb020; }
.conn-notice.ok { border-color:#36d399; }
.conn-notice-title { font-size:16px; font-weight:700; margin-bottom:10px; }
.conn-notice.lost .conn-notice-title,
.conn-notice.warn .conn-notice-title { color:#ffb020; }
.conn-notice.ok .conn-notice-title { color:#36d399; }
.conn-notice-body { margin:0; font-size:13px; color:#e6edf6; line-height:1.6; }
.conn-notice-sub { margin:8px 0 0; font-size:12px; color:#7d8aa3; line-height:1.5; }
.conn-notice-btn { margin-top:16px; width:100%; padding:10px 0; cursor:pointer;
  background:#16202f; color:#e6edf6; border:1px solid #2a3a52; border-radius:8px;
  font-size:13px; font-weight:600; }
.conn-notice-btn:hover { background:#1c283a; }

/* 감지 순간 스냅샷. 젯슨이 보낸 crop 이라 가로세로가 제각각이므로
   높이만 묶어두고 비율은 유지한다(object-fit:contain). */
.popup-snaps { padding:12px 16px 0; }
.popup-snaps-head { font-size:11px; color:#7d8aa3; margin-bottom:8px; letter-spacing:.02em; }
/* 한 장이면 넓게, 여러 장이면 나란히. 사진 수가 늘어도 팝업이 안 길어지게 가로로 깐다. */
.popup-snaps-grid { display:grid; gap:10px; }
.popup-snaps-grid.multi { grid-template-columns:repeat(auto-fit, minmax(150px, 1fr)); }
.popup-snap { margin:0; }
.popup-snap-img { display:block; width:100%; max-height:190px; object-fit:contain;
  background:#0c1118; border:1px solid #1d2836; border-radius:6px; }
.popup-snaps-grid.multi .popup-snap-img { max-height:130px; }
.popup-snap-cap { margin-top:6px; font-size:11px; color:#7d8aa3; letter-spacing:.02em;
  text-align:center; }
.popup-actions { padding:0 16px 16px; display:flex; flex-direction:column; gap:8px; }
.popup-ack-btn {
  width:100%; padding:12px; border-radius:9px; border:1px solid #2dd4bf;
  background:rgba(45,212,191,.12); color:#2dd4bf; font-size:13px; font-weight:700;
  letter-spacing:.3px; cursor:pointer; transition:background .15s ease;
}
.popup-ack-btn:hover { background:rgba(45,212,191,.22); }
.popup-clear-btn {
  width:100%; padding:10px; border-radius:9px; border:1px solid #4b5768;
  background:rgba(75,87,104,.15); color:#9aa7bc; font-size:12px; font-weight:600;
  letter-spacing:.2px; cursor:pointer; transition:background .15s ease, color .15s ease;
}
.popup-clear-btn:hover { background:rgba(75,87,104,.3); color:#e6edf6; }

@keyframes blink { 0%,100%{opacity:1} 50%{opacity:.25} }
@keyframes fade { from{opacity:0} to{opacity:1} }
@keyframes pop { from{opacity:0; transform:scale(.96)} to{opacity:1; transform:scale(1)} }

@media (max-width:1100px) {
  .body { grid-template-columns:1fr; grid-template-rows:auto 1fr auto; }
  .left,.right { flex-direction:row; }
  .left .panel, .right .panel { flex:1; }
  .stage { min-height:340px; }
}
`;
