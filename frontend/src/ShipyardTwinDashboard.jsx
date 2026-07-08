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
 * 0. 도메인 상수 — 백엔드와 사전 합의한 인터페이스(좌표계 / 이벤트 스키마)
 *    탐지 클래스 8종: 계획서 9p 기준
 * ------------------------------------------------------------------------- */
const SEVERITY = { DANGER: "danger", WARN: "warn", INFO: "info" };

const CLASS_META = {
  fallen_person:  { label: "작업자 쓰러짐",  severity: SEVERITY.DANGER, group: "안전" },
  fire:           { label: "화재",          severity: SEVERITY.DANGER, group: "화재" },
  helmet_off:     { label: "안전모 미착용",  severity: SEVERITY.WARN,   group: "안전" },
  helmet_on:      { label: "안전모 착용",    severity: SEVERITY.INFO,   group: "안전" },
  ship_block:     { label: "블록 탐지",      severity: SEVERITY.INFO,   group: "공정" },
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

function serverToWorld(blockId, local = { x: 0.5, y: 1, z: 0.5 }) {
  const r = SECTION_RANGE[blockId] || [0.4, 0.6];
  const t = r[0] + (r[1] - r[0]) * (local.z ?? 0.5); // 구획 내 z 위치
  const z = (t - 0.5) * SHIP_LEN;
  const x = (local.x - 0.5) * SHIP_BEAM * 0.8;       // 폭 방향
  const y = DECK_Y + (local.y ?? 1) * 0.9;           // 갑판 위
  return new THREE.Vector3(x, y, z);
}

/* ---------------------------------------------------------------------------
 * 2. 모의 이벤트 소스 — 실제 WebSocket과 동일한 페이로드 형태
 *    payload: { id, ts, cls, blockId, local{x,y,z}, conf }
 * ------------------------------------------------------------------------- */
const CLASS_POOL = [
  "fallen_person", "fire", "helmet_off", "helmet_on", "ship_block",
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
 * 3. Three.js 씬 매니저 (명령형 래퍼)
 *    React state로 매 프레임 리렌더하면 비싸므로, 3D는 ref/명령형으로 제어.
 * ------------------------------------------------------------------------- */
class SceneManager {
  constructor(canvas, { onPickBlock }) {
    this.canvas = canvas;
    this.onPickBlock = onPickBlock;
    this.pings = []; // {mesh, ring, born, ttl, sev}
    this.blockMeshes = new Map();
    this._init();
  }

  _init() {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color("#0a0e17");
    this.scene.fog = new THREE.Fog("#0a0e17", 28, 60);

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

  /* 0~1 스무스스텝 — 양 끝에서 기울기가 0이 되어 이어붙일 때 꺾임이 없다 */
  _smoothstep(x) {
    const c = Math.max(0, Math.min(1, x));
    return c * c * (3 - 2 * c);
  }

  /* 배 단면 폭 계수: 선미(t=0)~선수(t=1). 중앙은 평행한 최대폭 구간(패럴렐 미드바디),
   * 선미는 완만하게, 선수는 뾰족하게 — 스무스스텝으로 이어 붙여 꺾임 없이 매끄럽다. */
  _beamFactor(t) {
    if (t < 0.15) {
      const u = t / 0.15;
      return 0.55 + 0.45 * this._smoothstep(u); // 선미: 0.55 → 1.0
    }
    if (t < 0.78) return 1.0; // 평행 중앙부(최대폭 유지)
    const u = (t - 0.78) / 0.22;
    const eased = Math.pow(this._smoothstep(u), 1.15); // 선수: 1.0 → 거의 0 (뾰족)
    return 1.0 - eased * 0.97;
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
    const ringPts = 6; // 0:바닥중앙(용골) 1:좌빌지 2:좌현상단 3:갑판좌 4:갑판우 5:우현상단 ... (대칭 구성)
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

      // 좌현 → 용골 → 우현 순으로 6점 링 (둥근 선저 + 곧은 현측)
      const ring = [
        [-halfW * 0.92, topY,          z],  // 갑판 좌현
        [-halfW,        bilgeY * 2.2,  z],  // 좌현 빌지(넓은 곳)
        [-halfW * 0.30, keelY,         z],  // 좌측 용골 접근
        [ halfW * 0.30, keelY,         z],  // 우측 용골 접근
        [ halfW,        bilgeY * 2.2,  z],  // 우현 빌지
        [ halfW * 0.92, topY,          z],  // 갑판 우현
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
    // RC카(UGV) 표식 — 배 크기 대비 실제 축척에 맞게 작게 만든다 (RC카는 배보다 훨씬 작음)
    const g = new THREE.Group();
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.34, 0.14, 0.5),
      new THREE.MeshStandardMaterial({ color: "#38bdf8", metalness: 0.4, roughness: 0.4, emissive: "#0c4a6e", emissiveIntensity: 0.4 })
    );
    body.position.y = 0.14; body.castShadow = true;
    g.add(body);
    const cam = new THREE.Mesh(
      new THREE.CylinderGeometry(0.05, 0.05, 0.08, 12),
      new THREE.MeshStandardMaterial({ color: "#0ea5e9" })
    );
    cam.rotation.x = Math.PI / 2; cam.position.set(0, 0.22, 0.2);
    g.add(cam);
    this.ugv = g;
    this.ugvAngle = 0;
    this.scene.add(g);
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
    const meta = CLASS_META[payload.cls];
    if (!meta) return;
    const color = new THREE.Color(SEV_COLOR[meta.severity]);
    const pos = serverToWorld(payload.blockId, payload.local);

    // 코어 스피어
    const core = new THREE.Mesh(
      new THREE.SphereGeometry(0.22, 16, 16),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.95 })
    );
    core.position.copy(pos);
    this.scene.add(core);

    // 확산 링
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.25, 0.34, 32),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9, side: THREE.DoubleSide })
    );
    ring.position.copy(pos);
    ring.rotation.x = -Math.PI / 2;
    this.scene.add(ring);

    // 위험 라벨 폴(수직선)
    const poleMat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.6 });
    const poleGeo = new THREE.BufferGeometry().setFromPoints([
      pos.clone(), pos.clone().setY(pos.y + 2.2),
    ]);
    const pole = new THREE.Line(poleGeo, poleMat);
    this.scene.add(pole);

    const ttl = meta.severity === SEVERITY.DANGER ? 9000 : 5500;
    this.pings.push({ core, ring, pole, born: performance.now(), ttl, sev: meta.severity, blockId: payload.blockId });

    // DANGER면 해당 블록을 잠시 강조
    if (meta.severity === SEVERITY.DANGER) {
      const rec = this.blockMeshes.get(payload.blockId);
      if (rec) rec.mesh.material.emissive = new THREE.Color("#ff3b47");
    }
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
      this.radius = Math.max(12, Math.min(46, this.radius + e.deltaY * 0.02));
      update();
    };
    this.canvas.addEventListener("pointerdown", this._down);
    window.addEventListener("pointermove", this._move);
    window.addEventListener("pointerup", this._up);
    this.canvas.addEventListener("wheel", this._wheel, { passive: false });
    this._orbitUpdate = update;
  }

  _handleClick(e) {
    const rect = this.canvas.getBoundingClientRect();
    this.pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const meshes = [...this.blockMeshes.values()].map((r) => r.mesh);
    const hits = this.raycaster.intersectObjects(meshes, false);
    if (hits.length && this.onPickBlock) {
      this.onPickBlock(hits[0].object.userData.blockId);
    }
  }

  highlightBlock(blockId) {
    this.blockMeshes.forEach((rec, id) => {
      // 배 전체가 아니라 해당 구획 메쉬만 살짝 띄운다
      rec.mesh.position.y = id === blockId ? 0.25 : 0;
      rec.mesh.material.emissiveIntensity = id === blockId ? 0.6 : 0.0;

      if (id === blockId) rec.mesh.material.emissive = new THREE.Color("#2dd4bf");
      else if (rec.mesh.material.emissive.getHexString() !== "ff3b47")
        rec.mesh.material.emissive = new THREE.Color("#000000");
    });
  }

  _tick() {
    const t = performance.now();
    const dt = this.clock.getDelta();

    // UGV 순찰 — 배 우현을 따라 앞뒤로 왕복
    this.ugvT = (this.ugvT ?? 0) + dt * 0.12;
    const sweep = Math.sin(this.ugvT); // -1~1
    const z = sweep * (SHIP_LEN / 2);
    const x = SHIP_BEAM / 2 + 1.6;
    this.ugv.position.set(x, 0, z);
    this.ugv.rotation.y = Math.cos(this.ugvT) >= 0 ? 0 : Math.PI;

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

/* 캔버스 애니메이션 루프 공유 훅 */
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

/* 상시 라이브 패널 — 항상 켜져 UGV 영상을 흘려본다.
 * 경고(노랑) 발생 시 테두리가 깜빡이며 "확인 필요"를 알린다. */
function LivePanel({ ugvBlock, warnEvent, onExpand }) {
  const cvRef = useRef(null);
  useCctvCanvas(cvRef, true, warnEvent, ugvBlock ? ugvBlock.name : "야드 순찰");
  const warning = !!warnEvent;
  return (
    <div className={`live-panel ${warning ? "live-warn" : ""}`} onClick={onExpand} title="클릭하면 확대">
      <div className="live-head">
        <span className="live-dot" /> 실시간 UGV 영상
        {warning && <span className="live-warn-tag">⚠ 경고 — 확인</span>}
      </div>
      <canvas ref={cvRef} width={420} height={236} className="live-canvas" />
    </div>
  );
}

/* 확대 팝업 — 위험(빨강) 자동 송출 + 클릭 시 표시 공용.
 * ESC 키로도 닫을 수 있게 한다 (X 버튼 클릭 없이 키보드로 종료). */
function CctvPopup({ block, event, auto, onClose }) {
  const cvRef = useRef(null);
  useCctvCanvas(cvRef, !!block, event, block ? block.name : "");

  useEffect(() => {
    if (!block) return;
    const onKey = (e) => { if (e.key === "Escape") onClose(); };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [block, onClose]);

  if (!block) return null;
  const meta = event ? CLASS_META[event.cls] : null;
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
        <canvas ref={cvRef} width={760} height={428} className="popup-canvas" />
        <div className="popup-meta">
          <div><span className="k">구역</span><span className="v">{block.name} ({block.id})</span></div>
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
      </div>
    </div>
  );
}

/* ---------------------------------------------------------------------------
 * 5. 메인 대시보드
 * ------------------------------------------------------------------------- */
export default function ShipyardTwinDashboard() {
  const canvasRef = useRef(null);
  const sceneRef = useRef(null);
  const [events, setEvents] = useState([]);          // 이벤트 로그
  const [activeBlock, setActiveBlock] = useState(null);
  const [activeEvent, setActiveEvent] = useState(null);
  const [progress, setProgress] = useState(() =>
    Object.fromEntries(BLOCKS.map((b) => [b.id, Math.random() * 0.4])));
  const [stats, setStats] = useState({ danger: 0, warn: 0, info: 0 });
  const [connected, setConnected] = useState(false);
  const [autoPopup, setAutoPopup] = useState(false);     // 위험 자동 송출 여부
  const [warnEvent, setWarnEvent] = useState(null);      // 라이브 패널 경고 표시
  const [ugvBlock, setUgvBlock] = useState(BLOCKS[0]);   // UGV가 보고 있는 구역
  const warnTimer = useRef(null);

  const handlePickBlock = useCallback((blockId) => {
    const block = BLOCKS.find((b) => b.id === blockId);
    setActiveBlock(block);
    setAutoPopup(false);
    setActiveEvent((prev) => prev && prev.blockId === blockId ? prev : null);
    if (sceneRef.current) sceneRef.current.highlightBlock(blockId);
  }, []);

  // 씬 초기화
  useEffect(() => {
    const sm = new SceneManager(canvasRef.current, { onPickBlock: handlePickBlock });
    sceneRef.current = sm;
    const ro = new ResizeObserver(() => sm.resize());
    ro.observe(canvasRef.current.parentElement);
    return () => { ro.disconnect(); sm.dispose(); };
  }, [handlePickBlock]);

  // 이벤트 소스 연결
  useEffect(() => {
    setConnected(true);
    const off = connectEventSource((payload) => {
      const meta = CLASS_META[payload.cls];
      // 3D Ping (위험/경고만 시각화, info는 로그만)
      if (meta.severity !== SEVERITY.INFO && sceneRef.current) {
        sceneRef.current.spawnPing(payload);
      }
      // ship_block info → 공정률 진행
      if (payload.cls === "ship_block") {
        setProgress((p) => {
          const next = Math.min(1, (p[payload.blockId] ?? 0) + 0.08);
          if (sceneRef.current) sceneRef.current.setBlockProgress(payload.blockId, next);
          return { ...p, [payload.blockId]: next };
        });
      }

      // === 영상 송출 정책 ===
      // 위험(빨강): CCTV 큰 팝업 자동 송출 (클릭 안 해도 뜸)
      if (meta.severity === SEVERITY.DANGER) {
        const block = BLOCKS.find((b) => b.id === payload.blockId);
        setActiveBlock(block);
        setActiveEvent({ ...payload, _meta: meta });
        setAutoPopup(true);
        setUgvBlock(block);
        if (sceneRef.current) sceneRef.current.highlightBlock(payload.blockId);
      }
      // 경고(노랑): 라이브 패널에 경고 표시(테두리 점멸) + 영상 전환, 팝업은 안 띄움
      else if (meta.severity === SEVERITY.WARN) {
        const block = BLOCKS.find((b) => b.id === payload.blockId);
        setUgvBlock(block);
        setWarnEvent({ ...payload, _meta: meta });
        clearTimeout(warnTimer.current);
        warnTimer.current = setTimeout(() => setWarnEvent(null), 6000);
      }

      // 로그 적재 (최근 40개)
      setEvents((prev) => [{ ...payload, _meta: meta }, ...prev].slice(0, 40));
      // 통계
      setStats((s) => ({ ...s, [meta.severity]: s[meta.severity] + 1 }));
    });
    return () => { off(); clearTimeout(warnTimer.current); };
  }, []);

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
    if (sceneRef.current) sceneRef.current.highlightBlock(null);
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
          <span className={`conn ${connected ? "on" : ""}`}>
            <span className="conn-dot" />{connected ? "WebSocket 연결됨" : "연결 끊김"}
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
                  <span className="log-block">{e.blockId}</span>
                  <span className="log-conf">{(e.conf * 100).toFixed(0)}%</span>
                  <span className="log-time">{new Date(e.ts).toLocaleTimeString("ko-KR", { hour12: false })}</span>
                </button>
              ))}
            </div>
          </div>
        </aside>
      </div>

      <CctvPopup
        block={activeBlock}
        event={activeEvent}
        auto={autoPopup}
        onClose={closePopup}
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
.log-block { font-size:11px; color:#2dd4bf; font-weight:700; }
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
