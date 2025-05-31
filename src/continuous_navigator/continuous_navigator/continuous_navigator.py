#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations
import random
import time

WORLD_X_MIN = -2.2
WORLD_X_MAX = 2.2
WORLD_Y_MIN = -2.2
WORLD_Y_MAX = 2.2

class ContinuousNavigatorNode(Node):
    def __init__(self):
        super().__init__('continuous_navigator_node')
        self.navigator = BasicNavigator()

        self.get_logger().info('Navigation2 is active. Initializing navigation timer.')
        self.navigation_timer = self.create_timer(1.0, self.run_navigation_once)
    
    def generate_random_goal(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map' # Nav2は通常 'map' フレームでゴールを受け取る
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

    def run_navigation_once(self):
        # タイマーを一度キャンセルして、処理が完了するまで再実行されないようにする
        if self.navigation_timer:
            self.navigation_timer.cancel()

        goal_pose = self.generate_random_goal()
        self.navigator.goToPose(goal_pose)

        # ゴール達成またはタイムアウトまで待機
        while rclpy.ok() and not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # feedback.navigation_time や feedback.estimated_time_remaining などを利用可能
                # self.get_logger().info(f'Time elapsed: {feedback.navigation_time.sec}s, Est. time remaining: {feedback.estimated_time_remaining.sec}s')
                pass
            time.sleep(0.5) # CPU負荷軽減のため少し待つ

        if not rclpy.ok(): # シャットダウン要求があった場合
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

        # 次のゴール設定のために再度タイマーを設定 (例: 2秒後)
        if rclpy.ok():
            self.navigation_timer = self.create_timer(2.0, self.run_navigation_once)


    def on_shutdown(self):
        self.get_logger().info("Shutting down continuous navigator node.")
        if self.navigation_timer and self.navigation_timer.is_active():
            self.navigation_timer.cancel()
        # Nav2のクリーンアップ（もしBasicNavigatorが明示的なシャットダウンを要求する場合）
        # self.navigator.lifecycleShutdown() # BasicNavigatorは通常自動でクリーンアップされる

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousNavigatorNode()
    if not rclpy.ok(): # Check if node initialization caused shutdown
        node.get_logger().warn("RCLPY was shut down during node initialization.")
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node: # Check if node object exists
            node.get_logger().fatal(f"Unhandled exception in spin: {str(e)}")
        else:
            print(f"Unhandled exception in spin (node not initialized): {str(e)}")
    finally:
        if node: # Check if node object exists
            node.on_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()