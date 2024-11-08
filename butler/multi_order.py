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
        
        self.locations = {
            'home': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'kitchen': {'x': 10.35, 'y': 0.20, 'yaw': 0.0},
            'table1': {'x': 11.25, 'y': 3.88, 'yaw': 0.0},
            'table2': {'x': 6.25, 'y': 3.77, 'yaw': 0.0},
            'table3': {'x': 2.19, 'y': 4.11, 'yaw': 0.0}
        }
        

        self.orders = []
        self.task_status = "Idle"
        self.any_order_canceled = False  # Flag to check if any order was canceled

    def add_order(self, table):
        order = {
            'table': table,
            'requires_confirmation': True,
            'status': 'new'
        }
        self.orders.append(order)
        self.get_logger().info(f"Order added for Table {table}")

    def send_goal(self, location):
        goal_msg = NavigateToPose.Goal()
        
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
        
        return self.action_client.send_goal_async(goal_msg)

    def wait_for_confirmation(self, prompt, timeout):
        start_time = time.time()
        while time.time() - start_time < timeout:
            user_input = input(prompt).strip().lower()
            if user_input == 'yes':
                return True
            elif user_input == 'cancel':
                self.get_logger().info("Operation canceled.")
                return False
        return False

    def execute_orders(self):
        if not self.orders:
            self.get_logger().info("No orders to process.")
            return

        self.get_logger().info("Moving to kitchen to collect orders.")
        self.send_goal(self.locations['kitchen']).result()

        if not self.wait_for_confirmation("Confirm kitchen handover (yes/cancel): ", timeout=10):
            self.get_logger().info("Order collection canceled.")
            return  

        # Process each order
        for order in self.orders:
            table = order['table']
            
            # Move to the table to deliver the order
            self.get_logger().info(f"Moving to Table {table} for delivery.")
            self.send_goal(self.locations[table]).result()

            # Wait for confirmation at the table
            if order['requires_confirmation']:
                self.get_logger().info(f"Waiting for confirmation at Table {table}...")
                if not self.wait_for_confirmation(f"Confirm delivery at Table {table} (yes/cancel): ", timeout=10):
                    self.get_logger().info(f"Order at Table {table} was canceled.")
                    self.any_order_canceled = True  # Mark if any order was canceled
                    continue  

        # Check if any order was canceled
        if self.any_order_canceled:
            self.return_to_kitchen()  # Return to the kitchen if an order was canceled
        else:
            self.return_home()  # If no cancellations, go directly to home

    def return_to_kitchen(self):
        # Return to the kitchen with canceled orders
        self.get_logger().info("Returning to kitchen with canceled orders.")
        self.send_goal(self.locations['kitchen']).result()

        # Wait for confirmation from the kitchen to accept the canceled orders
        if self.wait_for_confirmation("Confirm return of canceled items to kitchen (yes/cancel): ", timeout=10):
            self.return_home()  # After confirmation, go to home
        else:
            self.get_logger().info("Return to kitchen canceled.")
            # Optionally handle if the confirmation is canceled (e.g., retry)

    def return_home(self):
        self.task_status = "Returning Home"
        self.get_logger().info("Returning to home position.")
        self.send_goal(self.locations['home']).result()
        self.task_status = "Idle"

def main(args=None):
    rclpy.init(args=args)
    robot = ButlerRobot()
    
    # Take user input for order details
    while True:
        table = input("Enter table number (table1, table2, table3) or 'exit' to stop: ").strip()
        if table == 'exit':
            break
        if table not in robot.locations:
            print("Invalid table number. Please try again.")
            continue
        robot.add_order(table)
    
    # Execute all orders
    robot.execute_orders()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
