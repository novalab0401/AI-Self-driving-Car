"""cmd_vel_publisher_example.py + scan_subscriber_example.py를 합친 예제:
장애물 감지 시 정지.

동작 원리:
    - 평소에는 전진한다
    - 정면 장애물이 STOP_DISTANCE보다 가까워지면 정지한다
    - 장애물이 다시 멀어지면 자동으로 전진을 재개한다

두 예제 파일과 다른 점 하나: 여기서는 "타이머로 계속 발행" 대신 "/scan이 들어올
때마다 그 자리에서 판단해서 발행"하는 구조다. LiDAR가 오는 주기(보통 8~10Hz)가
로봇을 제어하기에 충분히 빠르기 때문에 별도 타이머가 필요 없다 - 실제로 인식
결과에 반응해서 움직이는 노드를 만들 때 자주 쓰는 패턴이다.

이 코드는 "회피"가 아니라 "정지"만 한다. 여기서 확장하는 방법 예시:
    - 정지 대신 반대쪽으로 회전해서 피해가기 (msg.angular.z 추가)
    - 정면뿐 아니라 좌/우 LiDAR 값도 비교해서 어느 쪽이 더 트여있는지 보고
      그쪽으로 회전하기 (scan_subscriber_example.py의 "확장 아이디어" 참고)
    - 정지 후 N초 지나면 후진하고 다시 시도하기

실행:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash                 # 워크스페이스 폴더에서
    ros2 launch limo_gz sim.launch.py         # 다른 터미널
    ros2 run limo_examples obstacle_stop_example
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

FORWARD_SPEED = 0.15   # m/s
STOP_DISTANCE = 0.4    # m - 정면 장애물이 이 거리보다 가까우면 정지


class ObstacleStopExample(Node):

    def __init__(self):
        super().__init__('obstacle_stop_example')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan):
        front_index = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        front_index = max(0, min(front_index, len(msg.ranges) - 1))
        distance = msg.ranges[front_index]
        front_distance = distance if math.isfinite(distance) else float('inf')

        cmd = Twist()
        if front_distance > STOP_DISTANCE:
            cmd.linear.x = FORWARD_SPEED
            self.get_logger().info(f'전진 (정면 {front_distance:.2f}m)')
        else:
            cmd.linear.x = 0.0
            self.get_logger().info(
                f'정지! (정면 {front_distance:.2f}m < {STOP_DISTANCE}m)')
        self.cmd_pub.publish(cmd)


def main():
    # [GZ] Jazzy 에서는 기본 설정이면 Ctrl+C 를 누를 때 rclpy 가 먼저 종료되어, 아래
    # finally 에서 정지 명령을 보낼 수 없다(Humble 과 다른 점). 신호 처리를 꺼 두면
    # Ctrl+C 가 KeyboardInterrupt 로만 들어와서, 정지 명령을 보낸 뒤 직접 종료할 수 있다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ObstacleStopExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # 종료 시 정지 명령
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
