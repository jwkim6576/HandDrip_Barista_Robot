import rclpy
from rclpy.node import Node
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class GripperNode(Node):
    def __init__(self):
        super().__init__("gripper_node", namespace=ROBOT_ID)

        # DSR base node 등록
        DR_init.__dsr__node = self

        self.get_logger().info("Loading DSR API...")

        # ⚠️ DSR_ROBOT2 import는 여기에서만 해야 한다 (Node 생성 후!)
        from DSR_ROBOT2 import (
            set_digital_outputs,
            get_digital_inputs,
            wait_digital_input,
            tp_log,
            ON,
        )

        # 함수 바인딩
        self.set_digital_outputs = set_digital_outputs
        self.get_digital_inputs = get_digital_inputs
        self.wait_digital_input = wait_digital_input
        self.tp_log = tp_log
        self.ON = ON

        self.get_logger().info("DSR API loaded successfully.")

    def grip_close(self):
        self.get_logger().info("Closing gripper...")
        self.set_digital_outputs([1, -2])
        ret = self.wait_digital_input(1, self.ON, 3)
        self.get_logger().info(f"Close status: {ret == 0}")

    def grip_open(self):
        self.get_logger().info("Opening gripper...")
        self.set_digital_outputs([-1, 2])
        ret = self.wait_digital_input(2, self.ON, 3)
        self.get_logger().info(f"Open status: {ret == 0}")


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()

    try:
        node.grip_close()
        node.grip_open()
    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
