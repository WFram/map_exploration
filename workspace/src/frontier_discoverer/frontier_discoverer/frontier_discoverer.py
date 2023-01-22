from queue import Empty
import sys
from tabnanny import verbose
import time
import numpy as np
import math

from enum import Enum

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import Costmap
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy.qos

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_simple_commander.robot_navigator import BackUp, Spin


"""
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
"""
def quaternion_from_euler(roll, pitch, yaw):

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


class OccupancyGrid2d:

    class CostValues(Enum):
        FreeSpace = 0
        LethalObstacle = 100
        NoInformation = -1

    # When the object created, set the map attribute. The attribute is a map msg
    def __init__(self, map):

        self.map = map

    # Cost means occupancy probability (http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)
    def getCost(self, mx, my):

        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):

        return self.map.info.width, self.map.info.height

    def getSizeX(self):

        return self.map.info.width

    def getSizeY(self):

        return self.map.info.height

    def mapToWorld(self, mx, my):

        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return wx, wy

    # Convert odom pose in [m] to odom pose in [cells] (map frame)
    def worldToMap(self, wx, wy):

        # Compare to the real-world pose of the cell (0,0) in the map
        if wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y:
            raise Exception("World coordinates out of bounds")

        # Get current pose in map frame, but measured in map cells
        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        
        if my > self.map.info.height or mx > self.map.info.width:
            raise Exception("Out of bounds")

        return mx, my

    def __getIndex(self, mx, my):

        # y * width + x, where width is Map Width in cells (see format)
        return my * self.map.info.width + mx


class FrontierDiscoverer(Node):

    def __init__(self):
    
        super().__init__(node_name='nav2_frontier_discoverer', namespace='')
        
        self.waypoints = None
        self.goal = None
        self.currentPose = None
        self.lastWaypoint = None

        self.mx = None
        self.my = None
        self.count = 0
        self.extra_count = 0
        
        self.bad_cells = []
        self.min_frontiers = 10
        
        self.obstacle_tol = False
        self.num_recoveries = 0

        self.initial_pose_received = False

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.model_pose_sub = self.create_subscription(Odometry,
                                                       '/odom', self.poseCallback, qos_policy)
        
        self.finish_sub = self.create_subscription(Empty, '/finish_discovering', self.finish_cbk, qos_policy)
        
        self.navigator = BasicNavigator()
        
        self.recovery_action_client = ActionClient(self, Spin, '/spin')
        
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map_test', 10)

        self.costmapSub = self.create_subscription(OccupancyGrid, '/map', self.occupancyGridCallback, qos_policy)
        self.costmap = None

        self.get_logger().info('Running Frontier Discoverer')
        
        
    def finish_cbk(self):
        
        rclpy.shutdown()


    def recovery_spin(self, target_yaw):

        self.warn_msg('Start recovering')
        request = Spin.Goal()
        request.target_yaw = target_yaw
        goal_future = self.recovery_action_client.send_goal_async(request)
        try:
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))    
            
        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
            status = result_future.result().status
            result = result_future.result().result 
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))
        
        self.warn_msg('Finish recovering')
        self.num_recoveries += 1


    def is_bad(self, frontier, window_neighbour, check_unknowns=False, verbose=False):
        
        x = frontier[0]
        y = frontier[1]
        costs = []
        unknowns_cnt = 0
        for xx in range(x - window_neighbour, x + window_neighbour + 1):
            if xx <= 0 or xx >= self.costmap.getSizeX():
                # unknown_cnt += 1
                continue
            for yy in range(y - window_neighbour, y + window_neighbour + 1):
                if yy <= 0 or yy >= self.costmap.getSizeY():
                    # unknown_cnt += 1
                    continue
                cost = self.costmap.getCost(xx, yy)
                costs.append(cost)
                if check_unknowns and cost == OccupancyGrid2d.CostValues.NoInformation.value:
                    unknowns_cnt += 1
                
        # TODO: if too much recoveries (or trials) don't take into account obstacles at all
        #       return back the global flag after finding  
        if verbose:        
            self.info_msg(f'Costs for the frontier: {costs}')
        if (OccupancyGrid2d.CostValues.FreeSpace.value not in costs) or (OccupancyGrid2d.CostValues.LethalObstacle.value in costs):
            return True
        elif unknowns_cnt >= 6:
            return True
        else:
            return False


    def search_in_window(self, mx, my, costmap, shift_x, shift_y, window_size):

        self.info_msg('Searching in window')
        
        # Cell coordinates
        frontiers = []
        bad_frontier = False
        unknown_cnt = 0
        for x in range(mx + shift_x - window_size, mx + shift_x + window_size + 1):
            if x <= 0 or x >= costmap.getSizeX():
                continue
            for y in range(my + shift_y - window_size, my + shift_y + window_size + 1):
                if y <= 0 or y >= costmap.getSizeY():
                    continue

                # If the cell is NOT out of bounds
                if costmap.getCost(x, y) == OccupancyGrid2d.CostValues.NoInformation.value:
                    # TODO: experiment with it
                    
                    frontier = [x, y]
                    window_neighbour = 2
                    bad_frontier = self.is_bad(frontier, window_neighbour)

                    # TODO: develop a criteria, when a frontier is located over a wall
                    #       judging by the number of free cells around
                    if not bad_frontier:
                        frontiers.append([x, y])
                        
        self.info_msg(f'Number of frontiers: {len(frontiers)}')

        return frontiers


    def findFrontiers(self, pose, costmap):

        # Get a current pose (robot pose) in map cells (map frame)
        self.mx, self.my = costmap.worldToMap(pose.position.x, pose.position.y)

        shift_x = 0
        shift_y = 0
        window_size = 10
        frontiers = []
        if self.count < 1:
            frontiers = self.search_in_window(self.mx, self.my, costmap, shift_x, shift_y, window_size)

        while len(frontiers) < self.min_frontiers:
            self.info_msg(f'Count {self.count}')
            mul = self.count // 8 + 1
            
            if self.count % 8 == 0:
                shift_x = mul * window_size
                shift_y = 0
            if self.count % 8 == 1:
                shift_x = 0
                shift_y = mul * window_size
            if self.count % 8 == 2:
                shift_x = -mul * window_size
                shift_y = 0
            if self.count % 8 == 3:
                shift_x = 0
                shift_y = -mul * window_size
            if self.count % 8 == 4:
                shift_x = mul * window_size
                shift_y = mul * window_size
            if self.count % 8 == 5:
                shift_x = -mul * window_size
                shift_y = mul * window_size
            if self.count % 8 == 6:
                shift_x = -mul * window_size
                shift_y = -mul * window_size
            if self.count % 8 == 7:
                shift_x = mul * window_size
                shift_y = -mul * window_size

            if self.count > 500:
                self.recovery_spin(target_yaw=6.28)
                self.count = 0
                if self.num_recoveries > 1:
                    self.min_frontiers = 3
                    self.extra_count += 1
                    self.count = self.extra_count
                    self.warn_msg(f'Extra counts: {self.count}')
                    # TODO: check unknowns additionally
                    self.num_recoveries = 0
                continue
            frontiers = self.search_in_window(self.mx, self.my, costmap, shift_x, shift_y, window_size)
            self.count += 1

        return frontiers

    
    def setWaypoints(self, waypoints):

        self.waypoints = []
        for wp in waypoints:

            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.navigator.get_clock().now().to_msg()
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            theta = math.atan2(wp[1] - self.currentPose.position.y, wp[0] - self.currentPose.position.x)
            self.info_msg(f'Goal: \n \t {wp[0]}, {wp[1]}, {theta}')
            q = quaternion_from_euler(0, 0, theta)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]

            self.waypoints.append(msg)


    def setGoal(self, goal):

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.navigator.get_clock().now().to_msg()
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]

        theta = math.atan2(goal[1] - self.currentPose.position.y, goal[0] - self.currentPose.position.x)
        self.info_msg(f'Goal: \n \t {goal[0]}, {goal[1]}, {theta}')
        q = quaternion_from_euler(0, 0, theta)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.goal = msg
    
    
    def find_best_frontier(self, frontiers, dist_th):
        
        best_dist = 10000
        best_frontier = None
        for frontier in frontiers:
            diff_x = self.mx - frontier[0]
            diff_y = self.my - frontier[1] 
            dist = math.sqrt(diff_x**2 + diff_y**2)
            if dist > dist_th and dist <= best_dist and not self.is_bad(frontier, window_neighbour=1, check_unknowns=True) and not self.is_bad_cell(frontier):
                best_dist = dist
                best_frontier = frontier
                
        return best_frontier
    

    def set_bad_cells(self, cell):
        
        window_size = 1
        for x in range(cell[0] - window_size, cell[0] + window_size + 1):
            for y in range(cell[1] - window_size, cell[1] + window_size + 1):
                self.bad_cells.append([x, y])
                
    
    def is_bad_cell(self, cell):
        
        if cell in self.bad_cells:
            return True
        else:
            return False


    def moveToFrontiers(self):

        initial_dist_th = 4.0
        dist_th = initial_dist_th
        best_frontier = None
        i = 0
        while best_frontier == None:
            
            self.info_msg('Moving to frontiers')

            if abs(initial_dist_th - dist_th) < 1e-9:
                self.info_msg(f"Searching for frontiers with {self.min_frontiers} as minimal")
                frontiers = self.findFrontiers(self.currentPose, self.costmap)
                lengh_frontiers = len(frontiers)
                print(f'Found {lengh_frontiers} frontiers')
            
            best_frontier = self.find_best_frontier(frontiers, dist_th)
            
            if best_frontier != None:
                break
            
            dist_th -= 1.0
            if dist_th < 1.0:
                dist_th = initial_dist_th
                self.warn_msg(f'No frontiers with count {self.count}. Inreasing')
                self.count += 1
                
        self.min_frontiers = 10
        self.extra_count = 0
        frontiers.clear()
            
        # TODO: check best frontier here to know its neighbours cost value
        # TODO: Subscribe to current map here and spin. Update a current frontier value. If it's occupied or free, change a goal 
        self.is_bad(best_frontier, window_neighbour=1, verbose=True)
                
        print(f'X: {best_frontier[0]}')
        print(f'Y: {best_frontier[1]}')
        frontier_coords = self.costmap.mapToWorld(best_frontier[0], best_frontier[1])
        location = [frontier_coords]
        self.count = 0

        self.info_msg(f'World points: {location}')
        self.setGoal(frontier_coords)

        nav_start = self.navigator.get_clock().now()
        self.info_msg('Start moving to a goal')
        self.navigator.goToPose(self.goal)
        
        result = self.navigator.getResult()

        i = 0
        while not self.navigator.isTaskComplete():

            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                self.info_msg(f'Feedback: {feedback}')
            now = self.navigator.get_clock().now()

            # TODO: implement here switching a goal if gets occupied or free
            if now - nav_start > Duration(seconds=100000000.0):
                self.navigator.cancelTask()
                
            self.costmap = None
            while self.costmap == None:
                rclpy.spin_once(self, timeout_sec=1.0)
            x = best_frontier[0]
            y = best_frontier[1]
            
            if self.costmap.getCost(x, y) != OccupancyGrid2d.CostValues.NoInformation.value:
                self.navigator.cancelTask()
                
            if feedback.number_of_recoveries > 4:
                self.warn_msg('Too many recoveries. Setting bad cells')
                self.set_bad_cells(best_frontier)
                self.navigator.cancelTask()

        result = self.navigator.getResult()
        now = self.navigator.get_clock().now()
        self.info_msg(f'Time between goal reaching: {now - nav_start}')
        if result == TaskResult.SUCCEEDED:
            self.info_msg('Goal succeeded!')
            if now - nav_start < Duration(seconds=1.0):
                self.warn_msg('Goals being achieved too fast. Setting bad cells')
                self.set_bad_cells(best_frontier)
        elif result == TaskResult.CANCELED:
          self.info_msg('Goal was canceled!')
        elif result == TaskResult.FAILED:
          self.info_msg('Goal failed!')
        else:
          self.info_msg('Goal has an invalid return status!')

        self.currentPose = None
        self.costmap = None

        while self.costmap == None or self.currentPose == None:
            rclpy.spin_once(self, timeout_sec=1.0)

        self.moveToFrontiers()


    def setInitialPose(self, pose):
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = pose[0]
        self.init_pose.pose.pose.position.y = pose[1]
        self.init_pose.header.frame_id = 'map'
        self.currentPose = self.init_pose.pose.pose
        time.sleep(1)

    
    def poseCallback(self, msg):

        self.currentPose = msg.pose.pose
        self.initial_pose_received = True


    def occupancyGridCallback(self, msg):

        self.costmap = OccupancyGrid2d(msg)

    
    def info_msg(self, msg: str):

        self.get_logger().info(msg)


    def warn_msg(self, msg: str):
        
        self.get_logger().warn(msg)


    def error_msg(self, msg: str):

        self.get_logger().error(msg)


def main():

    rclpy.init()

    frontier_discoverer = FrontierDiscoverer()

    while not frontier_discoverer.initial_pose_received:
        frontier_discoverer.info_msg('Getting initial pose')
        rclpy.spin_once(frontier_discoverer, timeout_sec=1.0)

    while frontier_discoverer.costmap == None:
        frontier_discoverer.info_msg('Getting initial map')
        rclpy.spin_once(frontier_discoverer, timeout_sec=1.0)

    frontier_discoverer.moveToFrontiers()

    rclpy.spin(frontier_discoverer)
    
    frontier_discoverer.destroy_node()


if __name__ == '__main__':
    main()
