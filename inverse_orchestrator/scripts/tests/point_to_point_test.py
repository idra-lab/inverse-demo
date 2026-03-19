import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from inverse_msgs.srv import PointToPointMotion
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion, Point

class Orchestrator(Node):
    
    def __init__(self):
        super().__init__("orchestrator")

        self.get_logger().info("Orchestrator initialized and ready.")
        self.get_logger().info("Starting orchestrator...")


    def test_point_to_point(self) -> None:
        
        self.get_logger().info("Testing PointToPointMotion service call...")

        client = self.create_client(PointToPointMotion, "/planner/execute_ptp_motion")

        while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("Waiting for PointToPointMotion service...")
        
        request = PointToPointMotion.Request()

        test_intial_pose = PoseStamped()
        test_intial_pose.header = Header()
        test_intial_pose.header.frame_id = "world"
        test_intial_pose.pose = Pose(
            position=Point(x=0.5, y=-0.5, z=0.5),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

        test_target_pose = PoseStamped()
        test_target_pose.header = Header()
        test_target_pose.header.frame_id = "world"
        test_target_pose.pose = Pose(
            position=Point(x=0.5, y=0.0, z=0.5),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

        request.y0 = test_intial_pose
        request.g = test_target_pose
        request.max_vel = 0.15
        request.plan_y0_motion = True

        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"PointToPointMotion response: {future.result()}")
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")

def main():
    rclpy.init()
    orchestrator = Orchestrator()
    orchestrator.test_point_to_point()
    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
