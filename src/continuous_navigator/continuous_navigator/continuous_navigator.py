#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations
import random
import time

WORLD_X_MIN = -0.2
WORLD_X_MAX = 4.2
WORLD_Y_MIN = -1.7
WORLD_Y_MAX = 2.7

class ContinuousNavigatorNode(Node):
    def __init__(self):
        super().__init__('continuous_navigator_node')
        self.navigator = BasicNavigator()

        self.get_logger().info('Navigation2 is active. Initializing navigation timer.')
        self.navigation_timer = self.create_timer(1.0, self.run_navigation_once)
    
    def generate_random_goal(self): # Generating ramdom goal pose inside of the WORLD area
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        goal_pose.pose.position.x = random.uniform(WORLD_X_MIN, WORLD_X_MAX)
        goal_pose.pose.position.y = random.uniform(WORLD_Y_MIN, WORLD_Y_MAX)

        random_yaw = random.uniform(-3.14159, 3.14159)
        q = tf_transformations.quaternion_from_euler(0, 0, random_yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.get_logger().info(
            f"Generated new goal: x={goal_pose.pose.position.x:.2f}, "
            f"y={goal_pose.pose.position.y:.2f}, "
            f"yaw={random_yaw:.2f}"
        )
        return goal_pose

    def run_navigation_once(self): # attempt to reach the goal pose
        if self.navigation_timer:
            self.navigation_timer.cancel()

        goal_pose = self.generate_random_goal()
        self.navigator.goToPose(goal_pose)

        while rclpy.ok() and not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                pass
            time.sleep(0.5) # cycle time of 0.5s (2 Hz)

        if not rclpy.ok():
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Goal failed!')
        else:
            self.get_logger().info(f'Goal has an invalid return status: {result}')

        if rclpy.ok():
            self.navigation_timer = self.create_timer(2.0, self.run_navigation_once)


    def on_shutdown(self):
        self.get_logger().info("Shutting down continuous navigator node.")
        if self.navigation_timer and self.navigation_timer.is_active():
            self.navigation_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousNavigatorNode()
    if not rclpy.ok():
        node.get_logger().warn("RCLPY was shut down during node initialization.")
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception in spin: {str(e)}")
        else:
            print(f"Unhandled exception in spin (node not initialized): {str(e)}")
    finally:
        if node:
            node.on_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()