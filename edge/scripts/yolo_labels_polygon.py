#!/usr/bin/env python3
"""
yolo_labels_polygon.py
----------------------
YOLO 라벨(.txt)을 검사하고, 사각형(bbox) 행을 4점 폴리곤 행으로 변환한다.

[왜 필요한가]
ultralytics는 라벨 파일 하나 안에 6토큰 초과 행이 "하나라도" 있으면 그 파일의
"모든" 행을 폴리곤으로 간주한다 (ultralytics/data/utils.py:verify_image_label).
따라서 level5만 폴리곤으로 찍고 나머지를 사각형으로 두면, 같은 이미지에 들어있는
사각형 행 "cls cx cy w h"의 4개 값이 (cx,cy),(w,h) 두 점으로 해석되어
min/max 박스로 조용히 망가진다. 에러도 경고도 나지 않는다.
  실측: "0 0.5 0.5 0.2 0.2"  ->  "0 0.35 0.35 0.3 0.3"

=> 데이터셋 전체를 폴리곤 형식으로 통일하는 것이 유일하게 안전한 길이다.
   사각형을 4점 폴리곤으로 바꿔도 detect 학습 결과는 완전히 동일하고
   (내부에서 다시 bbox로 환원됨), 나중에 -seg 모델로 갈아탈 수도 있게 된다.

[사용법]
  python3 yolo_labels_polygon.py <labels_dir>          # 검사만 (기본, 파일 안 건드림)
  python3 yolo_labels_polygon.py <labels_dir> --apply  # bbox 행 -> 4점 폴리곤 변환
  python3 yolo_labels_polygon.py --selftest            # 자체 검증

<labels_dir>는 하위 폴더까지 재귀 검색하므로 데이터셋 루트를 그냥 넘겨도 된다.
(예: dataset/  ->  train/labels, valid/labels, test/labels 전부 처리)
"""

import argparse
import sys
from pathlib import Path


def bbox_to_polygon(parts):
    """cls cx cy w h  ->  cls x1 y1 x2 y2 x3 y3 x4 y4 (좌상->우상->우하->좌하)."""
    cls = parts[0]
    cx, cy, w, h = (float(v) for v in parts[1:5])
    x1, y1 = cx - w / 2, cy - h / 2
    x2, y2 = cx + w / 2, cy + h / 2
    # 정규화 범위를 벗어나면 ultralytics가 assert로 학습을 멈추므로 여기서 잘라둔다.
    # (라벨링 툴이 이미지 경계 밖으로 살짝 넘겨 내보내는 경우가 실제로 있음)
    c = lambda v: min(1.0, max(0.0, v))
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    return " ".join([cls] + [f"{c(v):.6f}" for p in pts for v in p])


def convert_lines(lines):
    """라벨 파일 한 개의 행들을 (변환된 행들, bbox행수, 폴리곤행수)로 돌려준다."""
    out, n_box, n_poly = [], 0, 0
    for line in lines:
        parts = line.split()
        if not parts:
            continue
        if len(parts) == 5:            # cls + cx cy w h
            n_box += 1
            out.append(bbox_to_polygon(parts))
        elif len(parts) >= 7 and len(parts) % 2 == 1:   # cls + 3점 이상
            n_poly += 1
            out.append(" ".join(parts))
        else:
            raise ValueError(f"토큰 수가 이상한 행({len(parts)}개): {line!r}")
    return out, n_box, n_poly


def main(argv=None):
    ap = argparse.ArgumentParser(description="YOLO 라벨 bbox -> 폴리곤 통일")
    ap.add_argument("labels_dir", nargs="?", help="라벨 .txt가 들어있는 폴더 (재귀 검색)")
    ap.add_argument("--apply", action="store_true", help="실제로 파일을 덮어쓴다")
    ap.add_argument("--selftest", action="store_true", help="자체 검증만 수행")
    args = ap.parse_args(argv)

    if args.selftest:
        selftest()
        return 0
    if not args.labels_dir:
        ap.error("labels_dir 가 필요합니다 (또는 --selftest)")

    root = Path(args.labels_dir)
    files = sorted(root.rglob("*.txt"))
    # classes.txt / notes 같은 부속 파일은 라벨이 아니므로 제외
    files = [f for f in files if f.name not in ("classes.txt", "notes.json")]
    if not files:
        print(f"라벨 .txt를 찾지 못했습니다: {root}", file=sys.stderr)
        return 1

    mixed, box_only, poly_only, empty, changed, bad = [], [], [], [], 0, []
    for f in files:
        try:
            out, n_box, n_poly = convert_lines(f.read_text(encoding="utf-8").splitlines())
        except ValueError as e:
            bad.append((f, e))
            continue

        if n_box and n_poly:
            mixed.append(f)        # ★ 학습이 조용히 망가지는 케이스
        elif n_box:
            box_only.append(f)
        elif n_poly:
            poly_only.append(f)
        else:
            empty.append(f)        # 배경 이미지(빈 라벨)는 정상이므로 그대로 둔다

        if args.apply and n_box:
            f.write_text("\n".join(out) + "\n", encoding="utf-8")
            changed += 1

    print(f"검사 대상       : {len(files)}개")
    print(f"  bbox만        : {len(box_only)}개")
    print(f"  폴리곤만      : {len(poly_only)}개")
    print(f"  ★섞임(위험)  : {len(mixed)}개")
    print(f"  빈 라벨(배경) : {len(empty)}개")
    for f, e in bad:
        print(f"  [형식오류] {f}: {e}")
    for f in mixed[:10]:
        print(f"  [섞임] {f}")
    if len(mixed) > 10:
        print(f"  ... 외 {len(mixed) - 10}개")

    if args.apply:
        print(f"\n변환 완료: {changed}개 파일의 bbox 행을 4점 폴리곤으로 바꿨습니다.")
        print("labels.cache 파일이 있으면 지우고 다시 학습하세요.")
    elif box_only or mixed:
        print("\n--apply 를 붙이면 실제로 변환합니다. (원본 백업 먼저 권장)")
    return 2 if bad else 0


def selftest():
    """변환이 정보를 잃지 않는지 + 섞임을 잡아내는지 확인."""
    # 1) bbox -> 폴리곤 -> (ultralytics가 하는 것과 동일한) min/max 환원이 원본과 같아야 한다
    line = "3 0.5 0.4 0.2 0.1"
    poly = bbox_to_polygon(line.split())
    xs = [float(v) for v in poly.split()[1::2]]
    ys = [float(v) for v in poly.split()[2::2]]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    w, h = max(xs) - min(xs), max(ys) - min(ys)
    assert poly.split()[0] == "3"
    assert (round(cx, 5), round(cy, 5), round(w, 5), round(h, 5)) == (0.5, 0.4, 0.2, 0.1), poly

    # 2) 경계를 넘는 좌표는 0~1로 잘려야 한다 (ultralytics assert 회피)
    clamped = [float(v) for v in bbox_to_polygon("0 0.05 0.5 0.4 0.2".split()).split()[1:]]
    assert min(clamped) >= 0.0 and max(clamped) <= 1.0, clamped

    # 3) bbox/폴리곤 섞인 파일을 섞임으로 판정해야 한다
    out, n_box, n_poly = convert_lines(
        ["0 0.5 0.5 0.2 0.2", "4 0.1 0.1 0.9 0.1 0.9 0.9 0.1 0.9"]
    )
    assert (n_box, n_poly) == (1, 1)
    assert all(len(o.split()) >= 9 for o in out), out   # 변환 후엔 전부 폴리곤

    # 4) 이미 폴리곤인 행은 손대지 않아야 한다 (반복 실행해도 안전)
    poly_line = "4 0.1 0.1 0.9 0.1 0.9 0.9 0.1 0.9"
    assert convert_lines([poly_line])[0] == [poly_line]

    print("selftest OK")


if __name__ == "__main__":
    sys.exit(main())
