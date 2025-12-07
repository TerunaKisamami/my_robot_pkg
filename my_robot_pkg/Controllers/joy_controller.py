from rclpy.qos_event import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pygame


class JoyController(Node):
    def __init__(self):
        super().__init__("joy_contoroller")
        self.publisher_ = self.create_publisher(Int32, "motor_goal", 10)

        # pygameとかの初期化
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.minPos = 0
        self.maxPos = 4095
        self.currentGoal = 2048

        # main_loopを0.02秒ごとに廻す
        self.timer = self.create_timer(0.02, self.main_loop)

        # get_logger : ログ表示機能(printの代わり)
        self.get_logger().info("Joystick Controller Started")

    def main_loop(self):
        pygame.event.pump()
        value = self.joystick.get_axis(0)

        if abs(value) > 0.1:
            delta = int(-50 * value)
            self.currentGoal += delta
            self.currentGoal = max(self.minPos, min(self.currentGoal, self.maxPos))

            # ｑうえうえに送信
            msg = Int32()
            msg.data = int(self.currentGoal)
            self.publisher_.publish(msg)

            # コンソールにログ表示
            self.get_logger().info(f"Publishing: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
