import imp
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BackUp, Spin


class Recovery(Node):

    def __init__(self):
        super().__init__(node_name='recovery')
        self.client = ActionClient(self, Spin, '/spin')

    
    def spin(self):

        self.info_msg('Start recovery')
        request = Spin.Goal()
        request.target_yaw = 6.28
        future = self.client.send_goal_async(request)
        try:
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            self.info_msg(f'Result: {result}')
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))
        self.info_msg('Finish recovery')


    def info_msg(self, msg: str):

        self.get_logger().info(msg)


    def error_msg(self, msg: str):

        self.get_logger().error(msg)


def main():

    rclpy.init()

    recovery = Recovery()

    recovery.spin()

    rclpy.spin(recovery)



if __name__ == '__main__':
    main()