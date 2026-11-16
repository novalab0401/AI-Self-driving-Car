"""LIMO 시뮬레이터(또는 실물) 카메라에서 라벨링용 프레임을 뽑는다.

사용법:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash                 # 워크스페이스 폴더에서
    # 다른 터미널에서 ros2 launch limo_gz sim.launch.py 실행 중이어야 함

    python3 capture_frames.py ../data/limo_track_raw --every 15 --max 200
"""

import argparse
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


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


class FrameGrabber(Node):

    def __init__(self, out_dir, topic, every, max_frames, prefix):
        super().__init__('frame_grabber')
        self.out_dir = out_dir
        self.every = every
        self.max_frames = max_frames
        self.prefix = prefix
        self.seen = 0
        self.saved = 0
        os.makedirs(out_dir, exist_ok=True)
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.get_logger().info(f"'{topic}' 구독 시작 -> {out_dir}")

    def cb(self, msg):
        self.seen += 1
        if self.seen % self.every != 0:
            return

        img = image_to_bgr(msg)
        name = f"{self.prefix}_{self.saved:05d}.png"
        cv2.imwrite(os.path.join(self.out_dir, name), img)
        self.saved += 1
        self.get_logger().info(f"[{self.saved}/{self.max_frames}] {name}")

        if self.saved >= self.max_frames:
            self.get_logger().info("목표 장수 도달, 종료")
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('out_dir', help='이미지를 저장할 폴더')
    parser.add_argument('--topic', default='/rgb/image_raw')
    parser.add_argument('--every', type=int, default=15,
                         help='N 프레임마다 1장 저장 (너무 촘촘히 찍으면 비슷한 사진만 쌓임)')
    parser.add_argument('--max', type=int, default=200, dest='max_frames',
                         help='최대 저장 장수')
    parser.add_argument('--prefix', default='limo',
                         help='파일명 접두어 (트랙/조명 조건별로 바꿔서 구분 추천)')
    args = parser.parse_args()

    rclpy.init()
    node = FrameGrabber(args.out_dir, args.topic, args.every, args.max_frames, args.prefix)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
