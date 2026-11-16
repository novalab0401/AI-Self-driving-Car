"""가장 간단한 LiDAR 예제: /scan 토픽을 구독해서 값을 출력만 하는 코드.

sensor_msgs/LaserScan 메시지 구조 (알아야 할 필드는 이 4개뿐):
    ranges          - 각도별 거리값 리스트 (미터). 인덱스 0번이 angle_min 방향.
    angle_min       - ranges[0]이 가리키는 각도 (라디안). LIMO는 정면 기준 -약 120도.
    angle_max       - ranges[-1]이 가리키는 각도.
    angle_increment - ranges 인덱스 하나가 몇 라디안 차이인지.

"정면 거리가 몇 미터냐"를 알고 싶으면, 각도 0(라디안)이 ranges의 몇 번 인덱스인지부터
계산해야 한다 (front_index 계산 부분 참고). 이 계산 패턴은 "특정 방향의 거리"가
필요할 때마다 그대로 재사용할 수 있다 (각도만 바꾸면 됨).

실행:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash                 # 워크스페이스 폴더에서
    ros2 launch limo_gz sim.launch.py         # 다른 터미널
    ros2 run limo_examples scan_subscriber_example

확장 아이디어:
    - front_index 대신 좌/우 90도 방향의 거리도 같이 출력해보기
      (각도를 +-math.pi/2로 바꿔서 front_index와 같은 방식으로 인덱스 계산)
    - 매 프레임 출력 대신, 거리가 급격히 줄어들 때만 경고 출력해보기
    - obstacle_stop_example.py를 보면 이 값으로 실제로 로봇을 멈추는 예제가 있다
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class ScanSubscriberExample(Node):

    def __init__(self):
        super().__init__('scan_subscriber_example')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan):
        # 각도 0(정면)이 ranges의 몇 번째 인덱스인지 계산.
        # (원하는 각도 - angle_min) / angle_increment = 그 각도의 인덱스
        front_index = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        front_index = max(0, min(front_index, len(msg.ranges) - 1))  # 범위 밖 방지
        front_distance = msg.ranges[front_index]

        # 가장 가까운 장애물까지의 거리. LaserScan은 측정 실패 시 0.0이나 inf/nan을
        # 채워 넣으므로, 유효한 값(0보다 크고 유한한 값)만 걸러서 최솟값을 구한다.
        valid_ranges = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        closest = min(valid_ranges) if valid_ranges else float('inf')

        self.get_logger().info(
            f'정면 거리: {front_distance:.2f}m   가장 가까운 장애물: {closest:.2f}m')


def main():
    # [GZ] 다른 예제와 같은 설정 (cmd_vel_publisher_example.py 의 main 참고)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ScanSubscriberExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
