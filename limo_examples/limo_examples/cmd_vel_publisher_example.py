"""가장 간단한 예제: LIMO를 전진시키는 코드.

ROS2 노드가 실제로 하는 일은 아래 5줄이 전부입니다 (publish_forward 안):

    msg = Twist()              # 1. 메시지 객체를 만든다
    msg.linear.x = FORWARD_SPEED  # 2. 전진 속도를 채운다
    self.publisher.publish(msg)   # 3. 발행한다

나머지 코드(__init__, main)는 "이 3줄을 ROS2 안에서 반복 실행시키기 위한 뼈대"입니다.
한 번 실행하고 끝나는 게 아니라, Twist 메시지를 계속(0.1초마다) 다시 보냅니다.
실물 LIMO는 명령이 끊기면 곧 멈추기 때문에, 계속 움직이려면 이렇게 반복해서 보내야
합니다. ([GZ] Gazebo Harmonic 시뮬레이터는 반대로 마지막 명령을 계속 따릅니다. 그래서
끝낼 때 정지 명령을 꼭 보내야 합니다 - 아래 main 의 finally 참고.)

실행:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash                 # 워크스페이스 폴더에서
    ros2 launch limo_gz sim.launch.py         # 다른 터미널
    ros2 run limo_examples cmd_vel_publisher_example

확장 아이디어:
    - FORWARD_SPEED를 음수로 바꿔서 후진시켜보기
    - publish_forward 안에서 시간(self.get_clock().now())에 따라 속도를 바꿔보기
      (예: 처음 3초는 전진, 그 다음 3초는 정지 - 반복)
    - Twist에는 msg.angular.z(회전 속도)도 있다 - 같이 넣어서 곡선으로 움직여보기
"""

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

FORWARD_SPEED = 0.15  # m/s - 너무 빠르면 커브에서 벗어날 수 있음
PUBLISH_HZ = 10.0     # 이 주기로 계속 다시 발행해야 로봇이 멈추지 않는다


class CmdVelPublisherExample(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher_example')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # create_timer: PUBLISH_HZ 주기로 publish_forward를 반복 호출하도록 등록.
        # ROS2에서 "계속 뭔가를 반복한다"는 대부분 이 패턴(타이머 콜백)으로 만든다.
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_forward)

    def publish_forward(self):
        msg = Twist()
        msg.linear.x = FORWARD_SPEED
        self.publisher.publish(msg)


def main():
    # [GZ] Jazzy 에서는 기본 설정이면 Ctrl+C 를 누를 때 rclpy 가 먼저 종료되어, 아래
    # finally 에서 정지 명령을 보낼 수 없다(Humble 과 다른 점). 신호 처리를 꺼 두면
    # Ctrl+C 가 KeyboardInterrupt 로만 들어와서, 정지 명령을 보낸 뒤 직접 종료할 수 있다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CmdVelPublisherExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C로 끌 때 정지 명령을 한 번 보내고 종료 - 안 하면 로봇이 마지막
        # 속도로 계속 움직이다가 벽/장애물에 부딪힐 수 있다.
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
