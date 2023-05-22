import rclpy
from rclpy.node import Node
import subprocess
from rclpy.qos import QoSProfile
from multiprocessing import Process, Value

from aedbot_interfaces.msg import FallDetectionToNav2, Bridge


URL = "http://130.162.152.119/HRI"


def run(state):
    """
    multi-processing을 위한 함수
    get_face()에서 while문 안으로 들어가기 때문에, topic sub이 되지 않는다.
    따라서, multi-processing을 통해 topic sub을 실행시킨다.

    state: multiprocessing.Value (type: bool)
    위의 공유변수를 통해 프로세스끼리 통신 가능.
    """

    def arrive_sub(sub_node):
        """
        ros2 topic subcriber를 생성하는 함수
        """
        sub = sub_node.create_subscription(
            Bridge,
            "arrive_dest",
            sub_callback_done,
            10,
        )

    def sub_callback_done(msg):
        """
        topic을 받으면, 공유변수를 True로 만듦.
        """
        state.value = True

    """
    멀티프로세싱에서도 ros2를 사용하기 위해, 동일하게 init()을 해준다.
    """
    rclpy.init()
    sub_node = Node("listener_node")
    arrive_sub(sub_node)
    rclpy.spin(sub_node)
    sub_node.destroy_node()
    rclpy.shutdown()


def get_face(self):
    # launch firefox in a subprocess
    p = subprocess.Popen(["firefox", URL, "--kiosk"])

    while True:
        if self.state.value:
            break

    p.terminate()
    return None


class Sub(Node):
    def __init__(self):
        super().__init__("sub")
        qos_profile = QoSProfile(depth=10)

        self.subscription_start = self.create_subscription(
            FallDetectionToNav2,
            "dest_val",
            self.listener_callback_get_dest,
            qos_profile,
        )

        self.callback_count = False  # callback이 두번 호출되는 것을 방지하기 위함

    def listener_callback_get_dest(self, msg):
        if self.callback_count:
            """
            callback이 두번 호출되는 것을 방지하기 위함
            """
            return None

        self.callback_count = True
        get_face(self)


def main():
    state = Value("B", False)

    rclpy.init()
    node = Sub()
    node.state = state  # 공유변수를 node에 추가

    p = Process(target=run, args=(state,))
    p.start()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
