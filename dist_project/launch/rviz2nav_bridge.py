import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__('goal_pose_bridge')
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self._action_client = ActionClient(self, NavigateToPose, '/shelfino1/navigate_to_pose')

    def goal_callback(self, msg: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        self.get_logger().info('Sending goal to nav2...')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
