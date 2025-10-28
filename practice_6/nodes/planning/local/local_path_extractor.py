#!/usr/bin/env python3

import rospy
import threading
import traceback
import shapely
import math
import numpy as np
from scipy.interpolate import interp1d
from autoware_mini.msg import Path, Waypoint
from geometry_msgs.msg import PoseStamped

class LocalPathExtractor:

    def __init__(self):

        # parameters
        self.publish_rate = rospy.get_param("~publish_rate")
        self.local_path_length = rospy.get_param("local_path_length")

        # variables
        self.current_pose = None
        self.global_path_xyz = None
        self.global_path_linestring = None
        self.global_path_velocities = None

        # Lock for thread safety
        self.lock = threading.Lock()

        # timer
        rospy.Timer(rospy.Duration(1 / self.publish_rate), self.extract_local_path)

        # publishers
        self.local_path_pub = rospy.Publisher('extracted_local_path', Path, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1,
                         tcp_nodelay=True)
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)

    def current_pose_callback(self, msg):
        self.current_pose = msg

    def global_path_callback(self, msg):
        if len(msg.waypoints) == 0:
            with self.lock:
                self.global_path_xyz = None
                self.global_path_linestring = None
                self.global_path_velocities = None
            rospy.loginfo("%s - Empty global path received", rospy.get_name())
        else:
            try:
                with self.lock:
                    self.global_path_xyz = np.array([(waypoint.position.x, waypoint.position.y, waypoint.position.z) for waypoint in msg.waypoints])
                    self.global_path_linestring = shapely.LineString(self.global_path_xyz)
                    self.global_path_velocities = np.array([waypoint.speed for waypoint in msg.waypoints])
                rospy.loginfo("%s - Global path received with %i waypoints", rospy.get_name(),
                              len(self.global_path_xyz))
            except Exception as e:
                self.global_path_xyz = None
                self.global_path_linestring = None
                self.global_path_velocities = None
                rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(),
                                      traceback.format_exc())


    def extract_local_path(self, _):
        try:
            with self.lock:
                current_pose = self.current_pose
                global_path_xyz = self.global_path_xyz
                global_path_linestring = self.global_path_linestring
                global_path_velocities = self.global_path_velocities

            if current_pose is None:
                return

            local_path = Path()
            local_path.header = current_pose.header

            if global_path_xyz is None:
                self.local_path_pub.publish(local_path)
                return

            current_position = None

            ego_distance_from_global_path_start = None

            global_path_distances = None

            global_path_velocities_interpolator = None

            current_position = shapely.Point(current_pose.pose.position.x, current_pose.pose.position.y,
                                             current_pose.pose.position.z)

            ego_distance_from_global_path_start = global_path_linestring.project(current_position)

            global_path_distances = np.cumsum(np.sqrt(np.sum(np.diff(global_path_xyz[:,:2], axis=0)**2, axis=1)))
            global_path_distances = np.insert(global_path_distances, 0, 0)

            global_path_velocities_interpolator = interp1d(global_path_distances, global_path_velocities, kind='linear', fill_value=0.0, bounds_error=False)

            # extract local path using distances and velocities interpolator
            local_path_waypoints = self.extract_waypoints(global_path_linestring, global_path_distances, ego_distance_from_global_path_start, self.local_path_length, global_path_velocities_interpolator)

            local_path = Path()
            local_path.header = current_pose.header
            local_path.waypoints = local_path_waypoints

            self.local_path_pub.publish(local_path)
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def extract_waypoints(self, global_path_linestring, global_path_distances, d_ego_from_path_start, local_path_length, global_path_velocities_interpolator):

        # current position is projected at the end of the global path - goal reached
        if math.isclose(d_ego_from_path_start, global_path_linestring.length):
            return None

        d_to_local_path_end = d_ego_from_path_start + local_path_length

        # find index where distances are higher than ego_d_on_global_path
        index_start = np.argmax(global_path_distances >= d_ego_from_path_start)
        index_end = np.argmax(global_path_distances >= d_to_local_path_end)

        # if end point of local_path is past the end of the global path (returns 0) then take index of last point
        if index_end == 0:
            index_end = len(global_path_linestring.coords) - 1

        # create local path from global path add interpolated points at start and end, use sliced point coordinates in between
        start_point = global_path_linestring.interpolate(d_ego_from_path_start)
        end_point = global_path_linestring.interpolate(d_to_local_path_end)
        local_path_xyz = start_point.coords[:] + list(global_path_linestring.coords[index_start:index_end]) + end_point.coords[:]
        local_path_distances = [d_ego_from_path_start] + list(global_path_distances[index_start:index_end]) + [d_to_local_path_end]

        local_path_waypoints = []
        for i in range(len(local_path_xyz)):
            waypoint = Waypoint()
            waypoint.position.x = local_path_xyz[i][0]
            waypoint.position.y = local_path_xyz[i][1]
            waypoint.position.z = local_path_xyz[i][2]
            waypoint.speed = global_path_velocities_interpolator(local_path_distances[i])
            local_path_waypoints.append(waypoint)

        return local_path_waypoints

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('local_path_extractor')
    node = LocalPathExtractor()
    node.run()
