#!/usr/bin/env python

"""
This metric calculates the average displacement error (ADE) of the ego vehicle
and shows it with pyplot.

"""

import math
import matplotlib.pyplot as plt

from srunner.metrics.examples.basic_metric import BasicMetric
from srunner.metrics.mine.acceleration import get_acceleration


def get_displacement_error(trajectory1, trajectory2):
    """
    Calculate the displacement error between two trajectories.

    :param trajectory1: List of (x, y) tuples representing the first trajectory.
    :param trajectory2: List of (x, y) tuples representing the second trajectory.
    :return: The average displacement error.
    """
    if len(trajectory1) != len(trajectory2):
        raise ValueError("Trajectories must have the same length.")

    total_error = 0.0
    for (x1, y1), (x2, y2) in zip(trajectory1, trajectory2):
        error = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        total_error += error

    return total_error / len(trajectory1)


class AverageDisplacementError(BasicMetric):
    """
    Metric class AverageDisplacementError
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

        # Calculate average displacement error
        trajectory = []
        for i in range(start, end + 1):
            loc = log.get_actor_location(ego_id, i)
            trajectory.append((loc.x, loc.y))
            frames_list.append(i)
        ade = get_displacement_error(trajectory, trajectory)
        acc_list = [ade] * len(frames_list)

        print(f"Average Displacement Error of the ego vehicle: {ade} meters")

        # Use matplotlib to show the results
        plt.plot(frames_list, acc_list)
        plt.ylabel('Average Displacement Error [meters]')
        plt.xlabel('Frame number')
        plt.title('Average Displacement Error of the ego vehicle over time')
        plt.show()
        # plt.savefig("ade_of_ego_vehicle.png")
