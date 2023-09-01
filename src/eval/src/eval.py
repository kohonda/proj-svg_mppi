#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Evaluate local planner on a simulation."""
import os
import rospy
import time
import csv
from std_msgs.msg import Bool, Float32
import message_filters
from eval_local_planner.msg import RaceInfo
from mppi_metrics_msgs.msg import MPPIMetrics


class Evaluator:
    def __init__(self, log_dir: str, eval_name: str, trial_num: int = 100):
        # existence check of log_dir
        if not os.path.exists(log_dir):
            raise FileNotFoundError("log_dir: {} does not exist.".format(log_dir))

        self._log_dir = log_dir
        self._eval_name = eval_name

        self.TRIAL_NUM = trial_num
        self.MAX_ELAPSED_TIME = 50.0

        self._lock = False
        self._done_initial_reset = False

        self._num_of_trials = 0
        self._success_list = [0] * self.TRIAL_NUM
        self._num_obstacles = 0
        self._static_obstacles_collision_patterns = []
        self._collision_list = [0] * self.TRIAL_NUM
        self._timeout_list = [0] * self.TRIAL_NUM
        self._seed_list = [0] * self.TRIAL_NUM
        self._state_cost_list_per_lap = []
        self._collision_cost_list_per_lap = []
        self._input_error_list_per_lap = []
        self._mean_state_cost_list = [1e6] * self.TRIAL_NUM
        self._mean_collision_cost_list = [1e6] * self.TRIAL_NUM
        self._mean_input_error_list = [1e6] * self.TRIAL_NUM
        self._calculation_time_per_lap = []
        self._mean_calculation_time_list = [0.0] * self.TRIAL_NUM
        self._max_calculation_time_list = [0.0] * self.TRIAL_NUM
        self._min_calculation_time_list = [1e6] * self.TRIAL_NUM

        # Pub and Sub
        race_info_topic = "race_info"
        self._sub_race_info = message_filters.Subscriber(
            race_info_topic, RaceInfo, queue_size=1
        )

        mppi_metrics_topic = "mppi/eval_metrics"
        self._mppi_metrics_sub = message_filters.Subscriber(
            mppi_metrics_topic, MPPIMetrics, queue_size=1
        )

        ts = message_filters.ApproximateTimeSynchronizer(
            [self._sub_race_info, self._mppi_metrics_sub],
            10,
            0.1,
            allow_headerless=False,
        )
        ts.registerCallback(self._callback)

        reset_env_topic = "reset_gym_env"
        self._pub_reset_env = rospy.Publisher(reset_env_topic, Bool, queue_size=1)

    def _callback(self, race_info_msg, mppi_metrics_msg):
        if not self._done_initial_reset:
            init_seed = 0
            self._reset_env(seed=init_seed)
            self._num_of_trials = 0
            self._seed_list[self._num_of_trials] = init_seed
            self._done_initial_reset = True
            self._static_obstacles_collision_patterns = []

        if self._num_of_trials >= self.TRIAL_NUM:
            self._end_eval()
            return

        if self._lock:
            return
        else:
            self._lock = True

        ego_collision = race_info_msg.ego_collision
        lap_count = race_info_msg.ego_lap_count
        elapsed_time = race_info_msg.ego_elapsed_time
        self._num_obstacles = race_info_msg.num_obstacles
        self._state_cost_list_per_lap.append(mppi_metrics_msg.state_cost)
        self._collision_cost_list_per_lap.append(mppi_metrics_msg.collision_cost)
        self._input_error_list_per_lap.append(mppi_metrics_msg.input_error)
        self._calculation_time_per_lap.append(mppi_metrics_msg.calculation_time)
        if ego_collision:
            collision_pattern = race_info_msg.static_obstacle_collisions
            if collision_pattern not in self._static_obstacles_collision_patterns:
                # Check if the collision pattern is not already registered
                self._static_obstacles_collision_patterns.append(collision_pattern)

        done = False
        # End of lap
        if lap_count > 0:
            if len(self._static_obstacles_collision_patterns) == 0:
                self._success_list[self._num_of_trials] = 1
            done = True

        # Timeout
        if elapsed_time > self.MAX_ELAPSED_TIME:
            self._timeout_list[self._num_of_trials] = 1
            done = True

        if done:
            # count collision patterns
            self._collision_list[self._num_of_trials] = len(
                self._static_obstacles_collision_patterns
            )

            # count costs per lap
            if len(self._state_cost_list_per_lap) == 0:
                self._mean_state_cost_list[self._num_of_trials] = 0.0
                self._mean_collision_cost_list[self._num_of_trials] = 0.0
                self._mean_input_error_list[self._num_of_trials] = 0.0
            else:
                self._mean_state_cost_list[self._num_of_trials] = sum(
                    self._state_cost_list_per_lap
                ) / float(len(self._state_cost_list_per_lap))

                self._mean_collision_cost_list[self._num_of_trials] = sum(
                    self._collision_cost_list_per_lap
                ) / float(len(self._collision_cost_list_per_lap))

                self._mean_input_error_list[self._num_of_trials] = sum(
                    self._input_error_list_per_lap
                ) / float(len(self._input_error_list_per_lap))
            self._state_cost_list_per_lap = []
            self._collision_cost_list_per_lap = []
            self._input_error_list_per_lap = []

            # calculate mean/max/min calculation time per lap
            if len(self._calculation_time_per_lap) == 0:
                self._mean_calculation_time_list[self._num_of_trials] = 0.0
                self._max_calculation_time_list[self._num_of_trials] = 0.0
                self._min_calculation_time_list[self._num_of_trials] = 0.0
            else:
                self._mean_calculation_time_list[self._num_of_trials] = sum(
                    self._calculation_time_per_lap
                ) / float(len(self._calculation_time_per_lap))
                self._max_calculation_time_list[self._num_of_trials] = max(
                    self._calculation_time_per_lap
                )
                self._min_calculation_time_list[self._num_of_trials] = min(
                    self._calculation_time_per_lap
                )

            self._num_of_trials += 1
            self._static_obstacles_collision_patterns = []

            # seed should be enough larger than previous seed
            # because the series of seeds are used in the same order
            if self._num_of_trials < self.TRIAL_NUM:
                seed = self._num_of_trials * (self._num_obstacles + 5)
                self._seed_list[self._num_of_trials] = seed
                self._reset_env(seed=seed)

        self._lock = False

    def _publish_reset_topic(self, rate: float = 0.1, duration: float = 0.5):
        start_time = time.time()
        while time.time() - start_time < duration:
            self._pub_reset_env.publish(True)
            time.sleep(0.1)

    def _reset_env(self, seed: int):
        # set rosparam
        rospy.set_param("/seed", seed)
        rospy.set_param("/ego_initial_x", 0.0)
        rospy.set_param("/ego_initial_y", 0.0)
        rospy.set_param("/ego_initial_theta", 3.14)
        rospy.set_param("/opp_initial_x", 100.0)
        rospy.set_param("/opp_initial_y", 0.0)
        rospy.set_param("/opp_initial_theta", 1.57)

        if self._num_of_trials != 0:
            mean_state_cost = 0.0
            mean_collision_cost = 0.0
            mean_input_error = 0.0
            for i in range(self._num_of_trials):
                mean_state_cost += self._mean_state_cost_list[i]
                mean_collision_cost += self._mean_collision_cost_list[i]
                mean_input_error += self._mean_input_error_list[i]
            mean_state_cost /= float(self._num_of_trials)
            mean_collision_cost /= float(self._num_of_trials)
            mean_input_error /= float(self._num_of_trials)
            rospy.logwarn(
                "Seed: {}, Trial num: {} / {}, Collision num: {} / {}, Mean state cost: {}, Mean collision cost: {}, Mean input error: {}".format(
                    self._seed_list[self._num_of_trials - 1],
                    self._num_of_trials,
                    self.TRIAL_NUM,
                    sum(self._collision_list),
                    self._num_of_trials * self._num_obstacles,
                    mean_state_cost,
                    mean_collision_cost,
                    mean_input_error,
                )
            )
        else:
            rospy.logwarn("Start {} trials.".format(self.TRIAL_NUM))

        # publish reset topic for 1 seconds
        self._publish_reset_topic()

    def _end_eval(self):
        # Save results to csv
        file_path = os.path.join(self._log_dir, self._eval_name + ".csv")
        with open(file_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "seed",
                    "success",
                    "collision",
                    "timeout",
                    "state_cost",
                    "collision_cost",
                    "input_error",
                    "mean_calculation_time",
                    "max_calculation_time",
                    "min_calculation_time",
                ]
            )
            for i in range(self.TRIAL_NUM):
                writer.writerow(
                    [
                        self._seed_list[i],
                        self._success_list[i],
                        self._collision_list[i],
                        self._timeout_list[i],
                        self._mean_state_cost_list[i],
                        self._mean_collision_cost_list[i],
                        self._mean_input_error_list[i],
                        self._mean_calculation_time_list[i],
                        self._max_calculation_time_list[i],
                        self._min_calculation_time_list[i],
                    ]
                )

        rospy.logwarn_once("Evaluation finished.")
        rospy.logwarn_once("Trial num: {}".format(self._num_of_trials))
        rospy.logwarn_once("Success num: {}".format(self._success_list.count(1)))
        rospy.logwarn_once(
            "Collision num: {} / {}".format(
                sum(self._collision_list), self._num_of_trials * self._num_obstacles
            )
        )
        rospy.logwarn_once("Not goal num: {}".format(self._timeout_list.count(1)))

        rospy.signal_shutdown("Evaluation finished.")


if __name__ == "__main__":
    rospy.init_node("eval_node")

    # get parameters
    trial_num = rospy.get_param("eval/trial_num", 100)
    log_dir = rospy.get_param("eval/log_dir", "")
    eval_name = rospy.get_param("eval/eval_name", "unknown")

    rospy.logwarn("Evaluation info: ")
    rospy.logwarn("Trial num: {}".format(trial_num))
    rospy.logwarn("Log dir: {}".format(log_dir))
    rospy.logwarn("Eval name: {}".format(eval_name))

    eval = Evaluator(log_dir=log_dir, trial_num=trial_num, eval_name=eval_name)
    rospy.spin()
