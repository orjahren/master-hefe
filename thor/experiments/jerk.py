#!/usr/bin/env python

"""
This metric calculates the jerk of the ego vehicle and
dumps it to a json file.

"""

import math
import matplotlib.pyplot as plt

from srunner.metrics.examples.basic_metric import BasicMetric
from srunner.metrics.mine.acceleration import get_acceleration


class JerkOfEgoVehicle(BasicMetric):
    """
    Metric class JerkOfEgoVehicle
    """

    def _create_metric(self, town_map, log, criteria):
        """
        Implementation of the metric. This is an example to show how to use the recorder,
        accessed via the log.
        """

        # Get the ID of the vehicle
        ego_id = log.get_ego_vehicle_id()

        acc_list = []
        frames_list = []

        prev_vel = None

        # Get the frames both actors were alive
        start_ego, end_ego = log.get_actor_alive_frames(ego_id)
        start = start_ego
        end = end_ego

        # Get the jerk of the ego vehicle
        for i in range(start + 1, end + 1):
            # use get_acceleration function
            acc_magnitude = get_acceleration(log, ego_id, i, prev_vel)
            if acc_magnitude is not None:
                acc_list.append(acc_magnitude)
                frames_list.append(i)
            prev_vel = log.get_actor_velocity(ego_id, i)

        # calculate jerk
        jerk_list = []
        jerk_frames = []
        for i in range(1, len(acc_list)):
            if frames_list[i] == frames_list[i - 1]:
                continue
            delta_t = log.get_elapsed_time(
                frames_list[i]) - log.get_elapsed_time(frames_list[i - 1])
            if delta_t > 0:
                jerk = (acc_list[i] - acc_list[i - 1]) / delta_t
                jerk_list.append(jerk)
                # Only append when jerk is calculated
                jerk_frames.append(frames_list[i])

        # Use matplotlib to show the results
        plt.plot(jerk_frames, jerk_list)
        plt.ylabel('Jerk [m/s^3]')
        plt.xlabel('Frame number')
        plt.title('Jerk of the ego vehicle over time')
        plt.show()
        # plt.savefig("jerk_of_ego_vehicle.png")
