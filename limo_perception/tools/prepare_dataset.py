"""../data/ 안의 데이터셋 폴더 전부(협업 했을 경우 파일이 여러개가 나오기 때문)를
하나로 합쳐 학습용 데이터셋을 만든다.


이 스크립트가 하는 일:
  1. ../data/ 밑에 있는 각 LIMO 데이터셋 폴더(Roboflow YOLOv8-seg export 등)를
     자동으로 찾는다 (data.yaml 이 폴더 바로 밑에 있는 것 전부).
  2. 클래스 번호를 '이름' 기준으로 UNIFIED 번호에 재매핑한다 (Roboflow에서 내려받을
     때마다 클래스 번호 순서가 달라질 수 있어서 이름 매칭이 필수).
  3. 폴리곤이 아닌 라벨(bbox)이나 UNIFIED에 없는 클래스명은 버린다.
  4. valid 또는 test 폴더가 있으면(Roboflow 3-way split) 재분할 없이 그대로 val로
     편입한다. train만 있는 데이터셋은 80/20으로 직접 나눈다.
  5. 결과를 OUT_ROOT 에 쓴다.

라벨링 시 반드시 지킬 것:
  - 클래스 이름을 UNIFIED 리스트와 정확히 동일하게 쓸 것 (오타/대소문자 다르면 그
    라벨은 조용히 버려진다 - 학습 후에야 "왜 안 됐지"로 발견되니 --check 로 먼저 확인).
  - 폴리곤(세그멘테이션) 라벨로 export 할 것 (bbox 라벨은 버려짐).

사용법:
    python3 prepare_dataset.py            # 병합 실행
    python3 prepare_dataset.py --check    # 데이터셋 점검만
"""

import argparse
import glob
import os
import random
import shutil

# ---------------------------------------------------------------------------
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
LOCAL_DATA_ROOT = os.path.join(TOOLS_DIR, '..', 'data')

# 저장소 어디에 clone 되든(사용자/워크스페이스 이름과 무관하게) 동작하도록 절대경로
# 대신 이 파일 위치 기준 상대경로로 계산한다. 결과물은 용량이 커서 git에는 안 올림
# (레포의 .gitignore 에 /datasets 가 등록되어 있음).
OUT_ROOT = os.path.join(TOOLS_DIR, '..', '..', 'datasets', 'limo_dataset')

UNIFIED = [
    'centerline', 'lane1', 'lane2', 'green_light', 'obstacle_vehicle',
    'red_light', 'obstacle_red_box',
]
NAME_TO_ID = {n: i for i, n in enumerate(UNIFIED)}

VAL_RATIO = 0.2
SEED = 0
IMG_EXT = ('.jpg', '.jpeg', '.png')


def load_names(ds_dir):
    import yaml
    cfg = yaml.safe_load(open(os.path.join(ds_dir, 'data.yaml')))
    names = cfg['names']
    if isinstance(names, dict):
        names = [names[k] for k in sorted(names)]
    return names


def discover_local_datasets():
    """../data/ 밑에서 data.yaml 을 가진 폴더를 전부 찾는다."""
    found = []
    if not os.path.isdir(LOCAL_DATA_ROOT):
        return found
    for entry in sorted(os.listdir(LOCAL_DATA_ROOT)):
        ds_dir = os.path.join(LOCAL_DATA_ROOT, entry)
        if os.path.isfile(os.path.join(ds_dir, 'data.yaml')):
            found.append((entry, ds_dir))
    return found


def remap_label(text, names):
    """라벨 텍스트의 클래스 번호를 UNIFIED 번호로 바꾼다. 버릴 줄은 제외."""
    out = []
    dropped_junk = 0
    dropped_bbox = 0
    for line in text.splitlines():
        parts = line.split()
        if not parts:
            continue
        cid = int(parts[0])
        coords = parts[1:]
        if len(coords) < 6 or len(coords) % 2 != 0:
            dropped_bbox += 1
            continue
        name = names[cid] if 0 <= cid < len(names) else None
        if name not in NAME_TO_ID:
            dropped_junk += 1
            continue
        out.append(f"{NAME_TO_ID[name]} " + " ".join(coords))
    return out, dropped_junk, dropped_bbox


def check():
    local = discover_local_datasets()
    if not local:
        print(f"[정보] {LOCAL_DATA_ROOT} 밑에 아직 LIMO 데이터셋이 없습니다.")
        print("       capture_frames.py 로 프레임을 찍고 Roboflow에서 라벨링/export 한 뒤")
        print("       그 폴더를 data/ 밑에 넣으세요.")
        return
    for tag, ds_dir in local:
        names = load_names(ds_dir)
        splits_present = [s for s in ('train', 'valid', 'test')
                           if os.path.isdir(os.path.join(ds_dir, s, 'labels'))]
        n_labels = sum(len(glob.glob(os.path.join(ds_dir, s, 'labels', '*.txt')))
                       for s in splits_present)
        unknown = [n for n in names if n not in NAME_TO_ID]
        print(f"[{tag}] splits={splits_present} 라벨 {n_labels}개, 클래스={names}")
        if unknown:
            print(f"  [경고] UNIFIED에 없는 클래스명: {unknown} (해당 라벨은 병합 시 버려짐)")


def _copy_one(ds_dir, src_split, names, tag, out_split, counts, junk_counter, bbox_counter, noimg_counter):
    lbl_files = sorted(glob.glob(os.path.join(ds_dir, src_split, 'labels', '*.txt')))
    for lf in lbl_files:
        stem = os.path.splitext(os.path.basename(lf))[0]
        img_src = None
        for ext in IMG_EXT:
            cand = os.path.join(ds_dir, src_split, 'images', stem + ext)
            if os.path.exists(cand):
                img_src = cand
                break
        if img_src is None:
            noimg_counter[0] += 1
            continue

        lines, dj, db = remap_label(open(lf).read(), names)
        junk_counter[0] += dj
        bbox_counter[0] += db

        new_stem = f"{tag}_{stem}"
        shutil.copy(img_src, os.path.join(
            OUT_ROOT, out_split, 'images', new_stem + os.path.splitext(img_src)[1]))
        with open(os.path.join(OUT_ROOT, out_split, 'labels', new_stem + '.txt'), 'w') as f:
            f.write("\n".join(lines))
        counts[out_split] += 1


def merge_local_datasets():
    random.seed(SEED)
    counts = {'train': 0, 'val': 0}
    junk = [0]
    bbox = [0]
    noimg = [0]

    for tag, ds_dir in discover_local_datasets():
        names = load_names(ds_dir)
        has_valid = os.path.isdir(os.path.join(ds_dir, 'valid', 'labels'))
        has_test = os.path.isdir(os.path.join(ds_dir, 'test', 'labels'))
        has_train = os.path.isdir(os.path.join(ds_dir, 'train', 'labels'))

        if has_valid or has_test:
            # Roboflow가 이미 나눠준 것 - 우리가 다시 섞지 않고 그대로 편입한다.
            print(f"[{tag}] Roboflow 자체 split 사용 (train/valid/test 그대로)")
            if has_train:
                _copy_one(ds_dir, 'train', names, tag, 'train', counts, junk, bbox, noimg)
            if has_valid:
                _copy_one(ds_dir, 'valid', names, tag, 'val', counts, junk, bbox, noimg)
            if has_test:
                _copy_one(ds_dir, 'test', names, tag, 'val', counts, junk, bbox, noimg)
            continue

        if not has_train:
            print(f"[{tag}] [경고] train/labels 도 없어서 건너뜀")
            continue

        # train 폴더 하나뿐인 경우 - 80/20으로 직접 나눈다.
        lbl_files = sorted(glob.glob(os.path.join(ds_dir, 'train', 'labels', '*.txt')))
        random.shuffle(lbl_files)
        n_val = int(len(lbl_files) * VAL_RATIO)
        print(f"[{tag}] 자체 80/20 분할 ({len(lbl_files)}개 라벨), 클래스={names}")
        for i, lf in enumerate(lbl_files):
            stem = os.path.splitext(os.path.basename(lf))[0]
            img_src = None
            for ext in IMG_EXT:
                cand = os.path.join(ds_dir, 'train', 'images', stem + ext)
                if os.path.exists(cand):
                    img_src = cand
                    break
            if img_src is None:
                noimg[0] += 1
                continue
            lines, dj, db = remap_label(open(lf).read(), names)
            junk[0] += dj
            bbox[0] += db
            split = 'val' if i < n_val else 'train'
            new_stem = f"{tag}_{stem}"
            shutil.copy(img_src, os.path.join(
                OUT_ROOT, split, 'images', new_stem + os.path.splitext(img_src)[1]))
            with open(os.path.join(OUT_ROOT, split, 'labels', new_stem + '.txt'), 'w') as f:
                f.write("\n".join(lines))
            counts[split] += 1

    if junk[0] or bbox[0] or noimg[0]:
        print(f"버려진 라벨 - 이름불일치: {junk[0]}, bbox형: {bbox[0]}, 이미지없음: {noimg[0]}")
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--check', action='store_true', help='병합 없이 data/ 폴더들만 점검')
    args = parser.parse_args()

    if args.check:
        check()
        return

    for split in ('train', 'val'):
        for sub in ('images', 'labels'):
            os.makedirs(os.path.join(OUT_ROOT, split, sub), exist_ok=True)

    counts = merge_local_datasets()

    yaml_path = os.path.join(OUT_ROOT, 'data.yaml')
    with open(yaml_path, 'w') as f:
        f.write(f"path: {OUT_ROOT}\n")
        f.write("train: train/images\n")
        f.write("val: val/images\n\n")
        f.write(f"nc: {len(UNIFIED)}\n")
        f.write(f"names: {UNIFIED}\n")

    print()
    print(f"합계: train {counts['train']}장, val {counts['val']}장")
    print(f"-> {yaml_path}")


if __name__ == '__main__':
    main()
