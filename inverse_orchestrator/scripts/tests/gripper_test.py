import rclpy
from rclpy.node import Node

from scripts.ur_gripper_controller import URGripper
import argparse

class GripperTestNode(Node):

    def __init__(self):
        super().__init__("gripper_test")
        self._gripper = URGripper(node=self)
        self.get_logger().info("GripperTestNode initialized, ready to send commands to the gripper.")

    def test_open(self) -> None:
        self.get_logger().info("Opening gripper...")
        future = self._gripper.open()
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Open result: {future.result()}")

    def test_close(self) -> None:
        self.get_logger().info("Closing gripper...")
        future = self._gripper.close()
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Close result: {future.result()}")

    def test_command(self, position: float = 0.5, max_effort: float = 50.0) -> None:
        self.get_logger().info(f"Sending gripper command: position={position}, max_effort={max_effort}...")
        future = self._gripper.command(position=position, max_effort=max_effort)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Command result: {future.result()}")



def main(args=None):
    rclpy.init(args=args)
    
    # Crea il nodo
    node = GripperTestNode()

    # Usa argparse per gestire gli argomenti
    parser = argparse.ArgumentParser(description="Test UR gripper commands.")
    parser.add_argument('--open', action="store_true", help="Test opening the gripper")
    parser.add_argument('--close', action="store_true", help="Test closing the gripper")
    
    args = parser.parse_args(args)

    # Se entrambi gli argomenti sono specificati, stampa un messaggio di errore
    if args.open and args.close:
        print("Please specify either --open or --close, not both.")
        return

    # Esegui i comandi di apertura o chiusura del gripper
    if args.open:
        node.test_open()
    elif args.close:
        node.test_close()

    # Distruggi il nodo e termina
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
