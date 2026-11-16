"""YOLOv8 세그멘테이션 학습 스크립트

사용법:
    python3 train_yolov8_seg.py                    # 학습
    python3 train_yolov8_seg.py --check             # 데이터셋 점검만
    python3 train_yolov8_seg.py --epochs 150
    python3 train_yolov8_seg.py --base yolov8n-seg.pt  # 더 작은 베이스로 바꾸고 싶을 때

학습이 끝나면 가중치가 여기에 생긴다:
    runs/segment/<NAME>/weights/best.pt   <- 이걸 쓴다
    runs/segment/<NAME>/weights/last.pt
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from prepare_dataset import OUT_ROOT  # 경로 하드코딩 없이 prepare_dataset.py와 동일한 위치 사용

# ---------------------------------------------------------------------------
DATA_YAML = os.path.join(OUT_ROOT, 'data.yaml')

BASE_MODEL = 'yolov8s-seg.pt'  # COCO 사전학습, 매번 여기서 새로 시작

RUN_NAME = 'limo_seg'

REQUIRED_CLASSES = [
    'centerline', 'lane1', 'lane2', 'green_light', 'obstacle_vehicle',
    'red_light', 'obstacle_red_box',
]


def check_dataset(data_yaml):
    import yaml

    if not os.path.exists(data_yaml):
        print(f"[에러] data.yaml 을 찾을 수 없습니다: {data_yaml}")
        print("       먼저 prepare_dataset.py 를 실행해서 데이터셋을 만드세요.")
        return False

    with open(data_yaml, 'r') as f:
        cfg = yaml.safe_load(f)

    names = cfg.get('names', [])
    if isinstance(names, dict):
        names = [names[k] for k in sorted(names)]

    print(f"data.yaml   : {data_yaml}")
    print(f"클래스 수    : {len(names)}")
    print(f"클래스 이름  : {names}")

    ok = True
    for required in REQUIRED_CLASSES:
        if required in names:
            print(f"  [OK]   {required}")
        else:
            print(f"  [없음] {required}")
            ok = False

    root = os.path.dirname(os.path.abspath(data_yaml))
    for split in ('train', 'val'):
        img_dir = os.path.join(root, split, 'images')
        lbl_dir = os.path.join(root, split, 'labels')
        if not os.path.isdir(img_dir):
            continue
        n_img = len([f for f in os.listdir(img_dir)
                     if f.lower().endswith(('.jpg', '.jpeg', '.png'))])
        n_lbl = len([f for f in os.listdir(lbl_dir)
                     if f.endswith('.txt')]) if os.path.isdir(lbl_dir) else 0
        print(f"{split:6s}: 이미지 {n_img}장, 라벨 {n_lbl}개")
        if split == 'train' and n_img < 200:
            print(f"        [참고] 학습 이미지가 {n_img}장뿐입니다 - 과적합 위험 큼. "
                  "가능하면 더 찍어서 늘리는 걸 권장.")

    return ok


def train(args):
    from ultralytics import YOLO

    print(f"베이스 모델: {args.base}")
    model = YOLO(args.base)

    model.train(
        data=DATA_YAML,
        name=RUN_NAME,
        epochs=args.epochs,
        imgsz=640,
        batch=args.batch,
        device=0,
        workers=8,
        patience=30,
        seed=0,

        # 차선 좌/우 의미가 있는 도메인이라 기본 증강을 그대로 쓰면 안 된다.
        fliplr=0.0,
        flipud=0.0,
        mosaic=0.0,
        degrees=0.0,
        shear=0.0,
        perspective=0.0,
        translate=0.05,
        scale=0.2,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
    )

    best = os.path.join('runs', 'segment', RUN_NAME, 'weights', 'best.pt')
    print()
    print("=" * 70)
    print(f"학습 완료 -> {best}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--base', default=BASE_MODEL)
    args = parser.parse_args()

    ok = check_dataset(DATA_YAML)
    if args.check:
        return
    if not ok:
        print("\n[중단] 데이터셋 점검 실패. 위 경고를 먼저 해결하세요.")
        return

    train(args)


if __name__ == '__main__':
    main()
