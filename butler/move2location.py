import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import math

class ButlerRobotNavigator(Node):
    def __init__(self):
        super().__init__('butler_robot_navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # To store coordinates for each location
        self.locations = {
            'home': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'kitchen': {'x': 10.55, 'y': 0.05, 'yaw': 0.0},
            'table1': {'x': 11.25, 'y': 3.88, 'yaw': 0.0},
            'table2': {'x': 6.25, 'y': 3.77, 'yaw': 0.0},
            'table3': {'x': 2.19, 'y': 4.11, 'yaw': 0.0}
        }
    
    def send_goal(self, location):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = location['x']
        pose.pose.position.y = location['y']
        yaw = location['yaw']
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
     
        goal_msg.pose = pose
        self.action_client.wait_for_server()
        self.get_logger().info(f"Navigating to location: {location}")

        return self.action_client.send_goal_async(goal_msg)
    
    def navigate(self):
        location_name = input("Enter destination (home, kitchen, table1, table2, table3): ").strip()
        if location_name in self.locations:
            location = self.locations[location_name]
            future = self.send_goal(location)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info("Navigation complete")
        else:
            self.get_logger().error("Invalid location. Please enter a valid location name.")

def main(args=None):
    rclpy.init(args=args)
    navigator = ButlerRobotNavigator()
    
    try:
        navigator.navigate()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
