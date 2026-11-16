"""YOLO 예측 결과를 실시간으로 보는 디버깅용 뷰어.
"""

import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(TOOLS_DIR, '..', 'models', 'best.pt')

WINDOW = 'LIMO YOLO Debug'
TRACKBAR = 'conf %'


def image_to_bgr(msg):
    """sensor_msgs/Image (rgb8 또는 bgr8) -> OpenCV BGR 배열.

    [GZ] cv_bridge 대신 직접 변환한다. cv_bridge 의 컴파일된 부분은 NumPy 1.x 용이라,
    ultralytics 와 함께 pip 로 깔리는 NumPy 2.x 환경에서는 임포트 단계에서 죽는다.
    """
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'지원하지 않는 이미지 인코딩: {msg.encoding}')
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()


class DebugViewer(Node):

    def __init__(self, topic):
        super().__init__('yolo_debug_dashboard')
        self.latest_msg = None
        self.sub = self.create_subscription(Image, topic, self._image_cb, 10)

    def _image_cb(self, msg):
        self.latest_msg = msg


def summary_text(result, names):
    """클래스별 검출 개수 + 최고 confidence 한 줄 요약."""
    if result.boxes is None or len(result.boxes) == 0:
        return "no detections"
    counts = {}
    best_conf = {}
    for cls_id, conf in zip(result.boxes.cls.tolist(), result.boxes.conf.tolist()):
        name = names[int(cls_id)]
        counts[name] = counts.get(name, 0) + 1
        best_conf[name] = max(best_conf.get(name, 0.0), conf)
    parts = [f"{name}x{n}({best_conf[name]:.2f})" for name, n in counts.items()]
    return "  ".join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', default=DEFAULT_MODEL)
    parser.add_argument('--topic', default='/rgb/image_raw')
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f"[에러] 모델 파일이 없습니다: {args.model}")
        print("       train_yolov8_seg.py로 학습하고 best.pt를 models/에 옮겨두세요.")
        return

    from ultralytics import YOLO
    print(f"모델 로딩: {args.model} (device={args.device})")
    model = YOLO(args.model)

    rclpy.init()
    node = DebugViewer(args.topic)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    conf_state = {'value': args.conf}
    cv2.createTrackbar(TRACKBAR, WINDOW, int(args.conf * 100), 100,
                        lambda v: conf_state.__setitem__('value', max(v, 1) / 100.0))

    print(f"'{args.topic}' 구독 시작. 창에 포커스 두고 q 또는 ESC로 종료.")

    last_t = time.time()
    fps = 0.0
    frame_count = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            if node.latest_msg is not None:
                msg = node.latest_msg
                node.latest_msg = None

                frame = image_to_bgr(msg)

                t0 = time.time()
                results = model.predict(source=frame, conf=conf_state['value'],
                                         device=args.device, verbose=False)
                infer_ms = (time.time() - t0) * 1000

                now = time.time()
                fps = 0.9 * fps + 0.1 * (1.0 / max(now - last_t, 1e-6))
                last_t = now

                annotated = results[0].plot()

                bar = np.zeros((40, annotated.shape[1], 3), dtype=np.uint8)
                cv2.putText(bar, f"conf>={conf_state['value']:.2f}  infer {infer_ms:.1f}ms  {fps:.1f} FPS",
                            (8, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
                canvas = np.vstack([annotated, bar])

                summary = summary_text(results[0], model.names)
                sum_bar = np.zeros((24, canvas.shape[1], 3), dtype=np.uint8)
                cv2.putText(sum_bar, summary, (8, 17), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1, cv2.LINE_AA)
                canvas = np.vstack([canvas, sum_bar])

                cv2.imshow(WINDOW, canvas)

                frame_count += 1
                if frame_count % 20 == 0:
                    print(f"[{frame_count}프레임 처리됨] {summary}")

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
