# limo_perception — LIMO 카메라용 차선/신호등/장애물 인식 데이터·학습

`data/` 밑의 데이터셋 5개(H-Mobility 원본 4개 + LIMO에서 직접 찍어 라벨링한 것)를
전부 합쳐서, COCO 사전학습 가중치부터 새로 학습한다. H-Mobility 쪽에서 이미 만든
`hmobility_merged`/`best.pt`는 쓰지 않는다 — 그건 H-Mobility 원본 카메라(실물 크기
차량, 다른 높이/화각)로만 학습된 것이라 LIMO 시뮬레이터 카메라에서 `centerline`
위치가 자주 틀리는 sim-to-sim 갭이 있었다. 그래서 원본 데이터셋들을 직접 다시
합쳐서 하나의 모델을 처음부터 학습하는 쪽으로 파이프라인을 짰다.

## 폴더 구조

```
limo_perception/
├── data/           # 병합 대상 데이터셋들 (Roboflow YOLOv8-seg export 그대로)
│                   # data.yaml 이 폴더 바로 밑에 있어야 prepare_dataset.py가 인식한다.
├── models/         # 학습 완료된 .pt 파일을 두는 곳 (아래 "학습된 모델 보관" 참고)
└── tools/
    ├── capture_frames.py    # 시뮬(또는 실물) 카메라에서 라벨링용 프레임 캡처
    ├── prepare_dataset.py   # data/ 밑 데이터셋들을 전부 하나로 병합
    ├── train_yolov8_seg.py  # 병합 데이터셋으로 COCO 사전학습 가중치부터 새로 학습
    └── debug_dashboard.py   # 학습된 모델의 실시간 예측을 보는 디버깅 뷰어
```

> **이 레포(Gazebo Harmonic 이식판)에서 달라진 점**
>
> - `data/`와 `models/`는 비어 있다. 원본 레포(`limo_ros2`, `humble_low_Mesh`)의 데이터셋
>   `LIMO with track map.yolov8`과 `models/best.pt`는 Gazebo Classic의 옛 트랙 맵에서 찍고
>   학습한 것이라 옮기지 않았다. 맵을 새로 만들면 아래 순서대로 다시 찍고 학습한다.
> - 아래 본문의 "데이터셋 5개", H-Mobility 관련 설명은 원본 레포 기준의 기록이다.
>   도구(`tools/`)의 동작은 그대로다.
> - `capture_frames.py`와 `debug_dashboard.py`는 `cv_bridge` 없이 이미지를 직접 변환한다
>   (ultralytics가 쓰는 NumPy 2.x와 `cv_bridge`가 같이 동작하지 않아서).

원본 레포의 `data/`에 들어 있던 5개:

| 폴더 | 출처 | 클래스 |
|---|---|---|
| `dataset1`, `dataset3`, `h mobility.yolov8`, `My First Project.yolov8` | H-Mobility 원본 (raw) | `centerline`/`lane1`/`lane2`/`green_light`/`obstacle_vehicle` 일부 |
| `LIMO with track map.yolov8` | LIMO 시뮬레이터(Classic)에서 직접 캡처+라벨링 | `centerline`/`lane1`/`lane2`/`red_light`/`obstacle_red_box` |

## 클래스 (7개, UNIFIED)

```
centerline, lane1, lane2, green_light, obstacle_vehicle, red_light, obstacle_red_box
```

`red_light`/`obstacle_red_box`인 이유: LIMO 트랙 신호등 기본 상태가 빨간불이고,
장애물이 실제 차량이 아니라 빨간 박스라서다 (H-Mobility의 `green_light`/
`obstacle_vehicle`과는 다른 대상 — 하나로 합치면 의미가 반대/틀리게 됨).

## 사용 순서

### 1. LIMO 카메라로 사진 찍기 (데이터를 더 늘리고 싶을 때)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash                 # 워크스페이스 폴더에서
ros2 launch limo_gz sim.launch.py         # 다른 터미널
```

- 대시보드 "Camera Capture" 패널 (`P` 키 수동 촬영 또는 Interval/Max frames 자동 촬영). `ros2 launch limo_gz sim.launch.py`를 하면 같이 뜬다.
- 또는 스크립트로 직접: `python3 tools/capture_frames.py data/limo_track_raw --every 15 --max 200`

### 2. 라벨링

Roboflow에 업로드해서 폴리곤 세그멘테이션 라벨링. 클래스 이름은 위 7개 중
해당하는 것만, 철자까지 정확히 (`prepare_dataset.py`의 `UNIFIED`와 동일해야
함 — 다르면 그 라벨은 조용히 버려짐, `--check`로 미리 확인).

YOLOv8 포맷(폴리곤, bbox 아님)으로 export 후 압축을 풀어서 `data/` 밑에 새
폴더로 추가한다. Roboflow가 train/valid/test로 나눠준 경우 그대로 두면 된다 —
`prepare_dataset.py`가 valid/test를 재분할 없이 그대로 val로 편입한다. `train`
폴더만 있는(2-way) 데이터셋은 80/20으로 자동 분할된다.

### 3. 데이터셋 점검 + 병합

```bash
cd tools
python3 prepare_dataset.py --check   # data/ 밑 폴더별 라벨 개수/클래스명 확인
python3 prepare_dataset.py           # 전부 병합
```

결과는 레포 최상위의 `datasets/limo_dataset/` 에 생성된다 (경로는 절대경로 하드코딩이
아니라 `prepare_dataset.py` 위치 기준 상대경로로 계산됨 — 어느 컴퓨터에 clone해도
동일하게 동작. 용량이 커서 `.gitignore`에 `/datasets`로 등록해뒀다).

### 4. 학습

```bash
python3 train_yolov8_seg.py --check   # 데이터셋 클래스/장수 확인
python3 train_yolov8_seg.py           # yolov8s-seg.pt(COCO)부터 새로 학습
```

끝나면 `tools/runs/segment/limo_seg/weights/best.pt` 가 새 모델이다.

### 5. 학습된 모델 보관

`tools/runs/`는 `.gitignore`에 등록된 **임시 산출물 폴더**라 학습을 다시 돌리면
덮어써지거나 폴더명이 바뀔 수 있다(`limo_seg2` 등). 그래서 실제로 쓸 모델은
`best.pt`를 꺼내서 `limo_perception/models/`에 옮겨 보관한다:

```bash
cp tools/runs/segment/limo_seg/weights/best.pt models/limo_seg_best.pt
```

**주의**: 지금은 이 경로를 참조하는 launch 파일이 없다 — LIMO는 아직 인식 결과를
조향으로 연결하는 자율주행 파이프라인(H-Mobility의 `yolov8_node` →
`lane_info_extractor_node` → `decision_making_pkg` 체인에 해당하는 것) 자체가
없어서, `models/`에 넣어둔다고 자동으로 쓰이지는 않는다. 나중에 그 파이프라인을
만들 때 이 폴더의 파일을 `model` 파라미터로 가리키면 된다.

### 6. 모델 디버깅 뷰어

학습이 실제로 잘 됐는지 카메라 화면에서 바로 확인하고 싶을 때 쓴다. RViz 안에
넣지 않고 별도 OpenCV 창으로 띄운다 — RViz에서 세그멘테이션 마스크+클래스별
confidence를 같이 보려면 커스텀 디스플레이 플러그인이 필요해서 번거롭다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash                 # 워크스페이스 폴더에서
ros2 launch limo_gz sim.launch.py         # 다른 터미널

cd tools
python3 debug_dashboard.py                  # 기본: ../models/best.pt 로드
python3 debug_dashboard.py --model ../models/limo_seg_best.pt
python3 debug_dashboard.py --conf 0.4 --device cuda:0
```

창 상단 트랙바로 confidence threshold를 실시간 조절할 수 있고, 하단에
추론 시간(ms)/FPS, 프레임별 클래스 검출 요약(`centerlinex4(0.88)` 형태)이
같이 표시된다. `q` 또는 `ESC`로 종료.
