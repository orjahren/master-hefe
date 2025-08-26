#!/usr/bin/env python

"""
This metric calculates the acceleration of the ego vehicle and
dumps it to a json file.

"""

import math
import matplotlib.pyplot as plt

from srunner.metrics.examples.basic_metric import BasicMetric


def get_acceleration(log, ego_id, i, prev_vel):
    ego_velocity = log.get_actor_velocity(ego_id, i)
    if prev_vel is not None:
        # get_elapsed_time expects frame index (starting from 0)
        prev_time = log.get_elapsed_time(i - 2)
        curr_time = log.get_elapsed_time(i - 1)
        delta_t = curr_time - prev_time
        if delta_t > 0:
            acc_x = (ego_velocity.x - prev_vel.x) / delta_t
            acc_y = (ego_velocity.y - prev_vel.y) / delta_t
            acc_z = (ego_velocity.z - prev_vel.z) / delta_t
            acc_magnitude = math.sqrt(
                acc_x**2 + acc_y**2 + acc_z**2)
            return acc_magnitude
    return None


class AccelerationOfEgoVehicle(BasicMetric):
    """
    Metric class AccelerationOfEgoVehicle
    """

    def _create_metric(self, town_map, log, criteria):
        """
        Implementation of the metric. This is an example to show how to use the recorder,
        accessed via the log.
        """

        # Get the ID of the two vehicles
        ego_id = log.get_ego_vehicle_id()
        adv_id = log.get_actor_ids_with_role_name(
            "scenario")[0]  # Could have also used its type_id

        acceleration_list = []
        frames_list = []

        prev_vel = None

        # Get the frames both actors were alive
        start_ego, end_ego = log.get_actor_alive_frames(ego_id)
        start_adv, end_adv = log.get_actor_alive_frames(adv_id)
        start = max(start_ego, start_adv)
        end = min(end_ego, end_adv)

        # Get the acceleration of the ego vehicle
        for i in range(start + 1, end + 1):
            acc_magnitude = get_acceleration(log, ego_id, i, prev_vel)
            if acc_magnitude is not None:
                acceleration_list.append(acc_magnitude)
                frames_list.append(i)
            prev_vel = log.get_actor_velocity(ego_id, i)

        # Use matplotlib to show the results
        plt.plot(frames_list, acceleration_list)
        plt.ylabel('Acceleration [m/s^2]')
        plt.xlabel('Frame number')
        plt.title('Acceleration of the ego vehicle over time')
        plt.show()
        # plt.savefig("acceleration_of_ego_vehicle.png")
