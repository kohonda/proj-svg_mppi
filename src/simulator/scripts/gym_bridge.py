#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped

from std_msgs.msg import Bool, Int32, Float32, std_msgs
from visualization_msgs.msg import MarkerArray, Marker

from f1tenth_gym_ros.msg import RaceInfo
from tf2_ros import transform_broadcaster
from tf.transformations import quaternion_from_euler
import numpy as np
from agents import PurePursuitAgent, StaticAgent

from f110_gym.envs.base_classes import Integrator

import gym


class GymBridge(object):
    def __init__(self):
        # get params
        self.ego_scan_topic = rospy.get_param("ego_scan_topic")
        self.ego_odom_topic = rospy.get_param("ego_odom_topic")
        self.opp_odom_topic = rospy.get_param("opp_odom_topic")
        self.ego_drive_topic = rospy.get_param("ego_drive_topic")
        self.race_info_topic = rospy.get_param("race_info_topic")
        self.reset_gym_env_topic = rospy.get_param("gym_reset_topic")

        self.scan_distance_to_base_link = rospy.get_param("scan_distance_to_base_link")

        self.map_path = rospy.get_param("map_path")
        self.map_img_ext = rospy.get_param("map_img_ext")
        print(self.map_path, self.map_img_ext)

        scan_fov = rospy.get_param("scan_fov")
        scan_beams = rospy.get_param("scan_beams")
        self.angle_min = -scan_fov / 2.0
        self.angle_max = scan_fov / 2.0
        self.angle_inc = scan_fov / scan_beams

        self.csv_path = rospy.get_param("waypoints_path")

        self.wheelbase = 0.3302
        # mass = 3.47
        # l_r = 0.17145
        # I_z = 0.04712
        # mu = 0.523
        # h_cg = 0.074
        # cs_f = 4.718
        # cs_r = 5.4562
        # init gym backend
        self._lf = 0.15875
        self._lr = 0.17145
        self._length = 0.58
        self._width = 0.31
        num_ego_vehicle = 1
        self.num_opponent_vehicle = 1
        # get param as private param
        self.num_static_obstacles = rospy.get_param("~num_static_obstacles")

        # remove extension
        map_wo_ext = self.map_path.split(".")[0]
        self.racecar_env = gym.make(
            "f110_gym:f110-v0",
            map=map_wo_ext,
            map_ext=self.map_img_ext,
            num_agents=num_ego_vehicle
            + self.num_opponent_vehicle
            + self.num_static_obstacles,
            timestep=0.01,
            integrator=Integrator.RK4,
        )

        # init opponent agent
        self.opp_agent = PurePursuitAgent(self.csv_path, self.wheelbase)

        # init static obstacles list, num is num_static_obstacles
        self.static_obstacles = []
        self.seed_rank = rospy.get_param("seed")
        for i in range(self.num_static_obstacles):
            self.static_obstacles.append(
                StaticAgent(csv_path=self.csv_path, seed=i + self.seed_rank)
            )

        self.reset_env(
            num_static_obs=self.num_static_obstacles,
            num_opponent_vehicle=self.num_opponent_vehicle,
        )

        # keep track of latest sim state
        self.ego_scan = list(self.obs["scans"][0])

        # keep track of collision
        self.ego_collision = False

        self.static_obs_collisions = [False for _ in range(self.num_static_obstacles)]

        # transform broadcaster
        self.br = transform_broadcaster.TransformBroadcaster()

        # pubs
        self.ego_scan_pub = rospy.Publisher(
            self.ego_scan_topic, LaserScan, queue_size=1
        )
        self.ego_odom_pub = rospy.Publisher(self.ego_odom_topic, Odometry, queue_size=1)
        self.opp_odom_pub = rospy.Publisher(self.opp_odom_topic, Odometry, queue_size=1)
        self.info_pub = rospy.Publisher(self.race_info_topic, RaceInfo, queue_size=1)
        self.static_obstacles_markers_pub = rospy.Publisher(
            "/static_obstacles", MarkerArray, queue_size=1
        )
        self.collision_check_pub = rospy.Publisher(
            "/collision_check", Marker, queue_size=1
        )

        # subs
        self.drive_sub = rospy.Subscriber(
            self.ego_drive_topic,
            AckermannDriveStamped,
            self.drive_callback,
            queue_size=1,
        )
        self.resetenv_sub = rospy.Subscriber(
            self.reset_gym_env_topic, Bool, self.reset_gym_env_callback, queue_size=1
        )  # signal to reset gym env

        # Timer
        self.timer = rospy.Timer(rospy.Duration(0.004), self.timer_callback)

    def reset_env(self, num_static_obs, num_opponent_vehicle):
        # # init opponent agent
        # self.opp_agent = PurePursuitAgent(self.csv_path, self.wheelbase)

        # load ego initial states
        ego_initial_x = rospy.get_param("ego_initial_x")
        ego_initial_y = rospy.get_param("ego_initial_y")
        ego_initial_theta = rospy.get_param("ego_initial_theta")

        ego_initial_state = np.array(
            [[ego_initial_x, ego_initial_y, ego_initial_theta]]
        )

        # load opp initial states
        opp_initial_x = rospy.get_param("opp_initial_x")
        opp_initial_y = rospy.get_param("opp_initial_y")
        opp_initial_theta = rospy.get_param("opp_initial_theta")

        opp_initial_state = np.array(
            [[opp_initial_x, opp_initial_y, opp_initial_theta]]
        )

        self.static_obstacles = []
        self.seed_rank = rospy.get_param("seed")
        for i in range(self.num_static_obstacles):
            self.static_obstacles.append(
                StaticAgent(csv_path=self.csv_path, seed=i + self.seed_rank)
            )

        random_static_obs_state = []
        radius = 0.1
        for i in range(num_static_obs):
            random_static_obs_state.append(
                self.static_obstacles[i].get_random_pos(radius=radius)
            )

        # reset gym environment and initialize
        initial_state = np.concatenate((ego_initial_state, opp_initial_state), axis=0)
        for obs_state in random_static_obs_state:
            initial_state = np.concatenate((initial_state, obs_state), axis=0)
        self.obs, reward, self.done, info = self.racecar_env.reset(initial_state)

        self.ego_pose = [ego_initial_x, ego_initial_y, ego_initial_theta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_steer = 0.0

        self.opp_pose = [ego_initial_x, ego_initial_y, ego_initial_theta]
        self.opp_speed = [0.0, 0.0, 0.0]
        self.opp_steer = 0.0

        self.static_obstacles_poses = []
        for pos in random_static_obs_state:
            self.static_obstacles_poses.append([pos[0][0], pos[0][1], pos[0][2]])

    def update_sim_state(self):
        self.ego_scan = list(self.obs["scans"][0])

        self.ego_pose[0] = self.obs["poses_x"][0]
        self.ego_pose[1] = self.obs["poses_y"][0]
        self.ego_pose[2] = self.obs["poses_theta"][0]
        self.ego_speed[0] = self.obs["linear_vels_x"][0]
        self.ego_speed[1] = self.obs["linear_vels_y"][0]
        self.ego_speed[2] = self.obs["ang_vels_z"][0]

        self.opp_pose[0] = self.obs["poses_x"][1]
        self.opp_pose[1] = self.obs["poses_y"][1]
        self.opp_pose[2] = self.obs["poses_theta"][1]
        self.opp_speed[0] = self.obs["linear_vels_x"][1]
        self.opp_speed[1] = self.obs["linear_vels_y"][1]
        self.opp_speed[2] = self.obs["ang_vels_z"][1]

    def drive_callback(self, drive_msg):
        # print('in drive callback')
        # TODO: trigger opp agent plan, step env, update pose and steer and vel
        ego_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        ego_control_vec = np.array([[self.ego_steer, ego_speed]])

        # activate opp_car path-tracking
        opp_speed, self.opp_steer = self.opp_agent.plan(self.obs)
        opp_control_vec = np.array([[self.opp_steer, opp_speed]])

        action_vec = np.concatenate((ego_control_vec, opp_control_vec), axis=0)
        for static_obs in self.static_obstacles_poses:
            action_vec = np.concatenate((action_vec, np.array([[0.0, 0.0]])), axis=0)
        self.obs, step_reward, self.done, info = self.racecar_env.step(action_vec)

        if self.obs["collisions"][0]:
            self.ego_collision = True
        else:
            self.ego_collision = False

        for i in range(self.num_static_obstacles):
            self.static_obs_collisions[i] = self.obs["collisions"][i + 2]

        self.update_sim_state()

    def reset_gym_env_callback(self, reset_gym_env_msg):
        if reset_gym_env_msg.data == True:
            print("reset gym environment")
            self.reset_env(
                num_opponent_vehicle=self.num_opponent_vehicle,
                num_static_obs=self.num_static_obstacles,
            )

            self.ego_collision = False
            self.static_obs_collisions = [
                False for _ in range(self.num_static_obstacles)
            ]

    def timer_callback(self, timer):
        ts = rospy.Time.now()

        # pub scan
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = "ego_racecar/laser"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.0
        scan.range_max = 30.0
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        # pub tf
        self.publish_odom(ts)
        self.publish_transforms(ts)
        self.publish_laser_transforms(ts)
        self.publish_wheel_transforms(ts)

        # publish static obstacles marker
        self.publish_static_obstacle_markers(ts)

        # pub race info
        self.publish_race_info(ts)

        self.publish_collision_check(ts)

    def publish_race_info(self, ts):
        info = RaceInfo()
        info.header.stamp = ts
        info.header.frame_id = ""
        info.ego_collision = bool(self.ego_collision)
        info.ego_elapsed_time = float(self.obs["lap_times"][0])
        info.ego_lap_count = int(self.obs["lap_counts"][0])
        info.num_obstacles = int(self.num_static_obstacles)
        for i in range(self.num_static_obstacles):
            info.static_obstacle_collisions.append(bool(self.static_obs_collisions[i]))
        self.info_pub.publish(info)

    def publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = "/map"
        ego_odom.child_frame_id = "ego_racecar/base_link"
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = quaternion_from_euler(0.0, 0.0, self.ego_pose[2])
        ego_odom.pose.pose.orientation.x = ego_quat[0]
        ego_odom.pose.pose.orientation.y = ego_quat[1]
        ego_odom.pose.pose.orientation.z = ego_quat[2]
        ego_odom.pose.pose.orientation.w = ego_quat[3]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        opp_odom = Odometry()
        opp_odom.header.stamp = ts
        opp_odom.header.frame_id = "/map"
        opp_odom.child_frame_id = "opp_racecar/base_link"
        opp_odom.pose.pose.position.x = self.opp_pose[0]
        opp_odom.pose.pose.position.y = self.opp_pose[1]
        opp_quat = quaternion_from_euler(0.0, 0.0, self.opp_pose[2])
        opp_odom.pose.pose.orientation.x = opp_quat[0]
        opp_odom.pose.pose.orientation.y = opp_quat[1]
        opp_odom.pose.pose.orientation.z = opp_quat[2]
        opp_odom.pose.pose.orientation.w = opp_quat[3]
        opp_odom.twist.twist.linear.x = self.opp_speed[0]
        opp_odom.twist.twist.linear.y = self.opp_speed[1]
        opp_odom.twist.twist.angular.z = self.opp_speed[2]
        self.opp_odom_pub.publish(opp_odom)

    def publish_transforms(self, ts):
        ego_t = Transform()
        # center to baselink
        ego_t.translation.x = self.ego_pose[0] - self._lr * np.cos(self.ego_pose[2])
        ego_t.translation.y = self.ego_pose[1] - self._lr * np.sin(self.ego_pose[2])
        ego_t.translation.z = 0.0
        ego_quat = quaternion_from_euler(0.0, 0.0, self.ego_pose[2])
        ego_t.rotation.x = ego_quat[0]
        ego_t.rotation.y = ego_quat[1]
        ego_t.rotation.z = ego_quat[2]
        ego_t.rotation.w = ego_quat[3]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = "/map"
        ego_ts.child_frame_id = "ego_racecar/base_link"

        opp_t = Transform()
        opp_t.translation.x = self.opp_pose[0]
        opp_t.translation.y = self.opp_pose[1]
        opp_t.translation.z = 0.0
        opp_quat = quaternion_from_euler(0.0, 0.0, self.opp_pose[2])
        opp_t.rotation.x = opp_quat[0]
        opp_t.rotation.y = opp_quat[1]
        opp_t.rotation.z = opp_quat[2]
        opp_t.rotation.w = opp_quat[3]

        opp_ts = TransformStamped()
        opp_ts.transform = opp_t
        opp_ts.header.stamp = ts
        opp_ts.header.frame_id = "/map"
        opp_ts.child_frame_id = "opp_racecar/base_link"

        self.br.sendTransform(ego_ts)
        self.br.sendTransform(opp_ts)

        # publish static obstacles
        for i, pos in enumerate(self.static_obstacles_poses):
            obs_t = Transform()
            obs_t.translation.x = pos[0] - self._lr * np.cos(pos[2])
            obs_t.translation.y = pos[1] - self._lr * np.sin(pos[2])
            obs_t.translation.z = 0.0
            obs_quat = quaternion_from_euler(0.0, 0.0, pos[2])
            obs_t.rotation.x = obs_quat[0]
            obs_t.rotation.y = obs_quat[1]
            obs_t.rotation.z = obs_quat[2]
            obs_t.rotation.w = obs_quat[3]

            obs_ts = TransformStamped()
            obs_ts.transform = obs_t
            obs_ts.header.stamp = ts
            obs_ts.header.frame_id = "/map"
            obs_ts.child_frame_id = "static_obstacle_" + str(i) + "/base_link"

            self.br.sendTransform(obs_ts)

    def publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = quaternion_from_euler(0.0, 0.0, self.ego_steer)
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[0]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[3]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = "ego_racecar/front_left_hinge"
        ego_wheel_ts.child_frame_id = "ego_racecar/front_left_wheel"
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = "ego_racecar/front_right_hinge"
        ego_wheel_ts.child_frame_id = "ego_racecar/front_right_wheel"
        self.br.sendTransform(ego_wheel_ts)

        opp_wheel_ts = TransformStamped()
        opp_wheel_quat = quaternion_from_euler(0.0, 0.0, self.opp_steer)
        opp_wheel_ts.transform.rotation.x = opp_wheel_quat[0]
        opp_wheel_ts.transform.rotation.y = opp_wheel_quat[1]
        opp_wheel_ts.transform.rotation.z = opp_wheel_quat[2]
        opp_wheel_ts.transform.rotation.w = opp_wheel_quat[3]
        opp_wheel_ts.header.stamp = ts
        opp_wheel_ts.header.frame_id = "opp_racecar/front_left_hinge"
        opp_wheel_ts.child_frame_id = "opp_racecar/front_left_wheel"
        self.br.sendTransform(opp_wheel_ts)
        opp_wheel_ts.header.frame_id = "opp_racecar/front_right_hinge"
        opp_wheel_ts.child_frame_id = "opp_racecar/front_right_wheel"
        self.br.sendTransform(opp_wheel_ts)

    def publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        ego_scan_ts.transform.rotation.w = 1.0
        ego_scan_ts.header.stamp = ts
        # TODO: check frame names
        ego_scan_ts.header.frame_id = "ego_racecar/base_link"
        ego_scan_ts.child_frame_id = "ego_racecar/laser"
        self.br.sendTransform(ego_scan_ts)

        opp_scan_ts = TransformStamped()
        opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        opp_scan_ts.transform.rotation.w = 1.0
        opp_scan_ts.header.stamp = ts
        # TODO: check frame names
        opp_scan_ts.header.frame_id = "opp_racecar/base_link"
        opp_scan_ts.child_frame_id = "opp_racecar/laser"
        self.br.sendTransform(opp_scan_ts)

    def publish_static_obstacle_markers(self, ts):
        marker_array = MarkerArray()
        for i, pos in enumerate(self.static_obstacles_poses):
            # baselink to center of obstacle
            pos_center_x = 0.5 * (self._lr + self._lr)
            pos_center_y = 0.0

            marker = Marker()
            marker.header.frame_id = "static_obstacle_" + str(i) + "/base_link"
            # marker.header.frame_id = "map"
            marker.header.stamp = ts
            marker.ns = "static_obstacles"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = pos_center_x
            marker.pose.position.y = pos_center_y
            marker.pose.position.z = 0.6
            quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            marker.scale.x = self._length
            marker.scale.y = self._width
            marker.scale.z = 2.0
            marker.color.a = 1.0
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5

            marker_array.markers.append(marker)
        self.static_obstacles_markers_pub.publish(marker_array)

    def publish_collision_check(self, ts):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = ts
        marker.ns = "collision_check"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.ego_pose[0]
        marker.pose.position.y = self.ego_pose[1]
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.7
        if self.ego_collision:
            # red
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            # green
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        self.collision_check_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("gym_bridge")
    gym_bridge = GymBridge()
    rospy.spin()
