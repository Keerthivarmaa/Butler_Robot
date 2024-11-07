import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import math
import time

class ButlerRobot(Node):
    def __init__(self):
        super().__init__('butler_robot')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Define coordinates for each location
        self.locations = {
            'home': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'kitchen': {'x': 10.35, 'y': 0.11, 'yaw': 90.0},
            'table1': {'x': 11.25, 'y': 3.88, 'yaw': 0.0},
            'table2': {'x': 6.25, 'y': 3.77, 'yaw': 0.0},
            'table3': {'x': 2.19, 'y': 4.11, 'yaw': 0.0}
        }
        
        # Order queue
        self.orders = []
        self.current_order = None
        self.task_status = "Idle"
    
    def add_order(self, table, requires_confirmation=False, cancelable=False):
        order = {
            'table': table,
            'requires_confirmation': requires_confirmation,
            'cancelable': cancelable,
            'status': 'new'
        }
        self.orders.append(order)
        self.get_logger().info(f"Order added for Table {table}")

    def send_goal(self, location):
        goal_msg = NavigateToPose.Goal()
        # Populate the goal with the coordinates
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = location['x']
        pose.pose.position.y = location['y']
        
        # Convert yaw to quaternion
        yaw = location['yaw']
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        goal_msg.pose = pose
        self.action_client.wait_for_server()
        
        # Send the goal and wait for result
        return self.action_client.send_goal_async(goal_msg)

    def execute_order(self):
        if not self.orders:
            self.get_logger().info("No orders to process.")
            return

        self.current_order = self.orders.pop(0)
        table = self.current_order['table']
        self.get_logger().info(f"Starting delivery to Table {table}")
        
        self.task_status = "Moving to Kitchen"
        self.send_goal(self.locations['kitchen']).result()
        
        if self.current_order['requires_confirmation']:
            self.get_logger().info("Waiting for confirmation at the kitchen...")
            if not self.wait_for_confirmation(timeout=10):
                self.get_logger().info("Timeout at kitchen. Returning home.")
                self.return_home()
                return

        self.task_status = "Moving to Table"
        self.send_goal(self.locations[table]).result()

        if self.current_order['requires_confirmation']:
            self.get_logger().info(f"Waiting for confirmation at Table {table}...")
            if not self.wait_for_confirmation(timeout=10):
                self.get_logger().info(f"Timeout at Table {table}. Returning to kitchen.")
                self.return_to_kitchen()
                self.return_home()
                return

        self.task_status = "Returning Home"
        self.return_home()

    def wait_for_confirmation(self, timeout):
        start_time = time.time()
        while time.time() - start_time < timeout:
            # Simulate a confirmation (replace with real input handling or sensor data)
            user_input = input("Confirm task completion? (yes/no): ").strip()
            if user_input.lower() == 'yes':
                return True
            elif user_input.lower() == 'no':
                return False
        return False

    def return_home(self):
        self.task_status = "Returning Home"
        self.get_logger().info("Returning to home position.")
        self.send_goal(self.locations['home']).result()
        self.task_status = "Idle"

    def return_to_kitchen(self):
        self.task_status = "Returning to Kitchen"
        self.get_logger().info("Returning to kitchen.")
        self.send_goal(self.locations['kitchen']).result()

def main(args=None):
    rclpy.init(args=args)
    robot = ButlerRobot()
    
    # Add some sample orders
    robot.add_order('table1', requires_confirmation=True)
    robot.add_order('table2', requires_confirmation=True, cancelable=True)
    robot.add_order('table3')
    
    # Execute orders
    while robot.orders:
        robot.execute_order()
        
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
