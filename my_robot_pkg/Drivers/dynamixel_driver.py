import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from .dynamixel_client import Dynamixel


class DynamixelDriver(Node):
    # インスタンス時に実行される関数
    def __init__(self):
        super().__init__("dynamixel_driver")

        self.subscription = self.create_subscription(
            Int32, "motor_goal", self.listenerCallback, 10
        )

        self.declare_parameter("port", "/dev/ttyUSB0")
        portName = self.get_parameter("port").get_parameter_value().string_value

        try:
            # dynamixelの接続と初期化
            self.dxl = Dynamixel(portName, 115200)
            self.ID = 0
            self.dxl.enable_torque(self.ID)
            self.get_logger().info("Dynamixel Connected")
        except Exception:
            self.get_logger().error(f"{Exception}のエラーが発生")

    # データを受信した瞬間に実行する関数
    def listenerCallback(self, msg):
        targetPos = msg.data
        try:
            self.dxl.write_position(self.ID, targetPos)
        except Exception:
            self.get_logger().error(f"{Exception}のエラーが発生")


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelDriver()
    rclpy.spin(node)

    # 終了したときの処理
    node.dxl.disable_torque(node.ID)
    node.dxl.close_port()
    node.destroy_node()
    rclpy.shutdown()
