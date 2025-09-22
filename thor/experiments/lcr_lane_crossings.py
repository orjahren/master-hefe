#!/usr/bin/env python

"""
This metric calculates the number of lane crossings by the ego vehicle 
and shows it with pyplot, also dumping it to a json file.

"""

import math
import json

from srunner.metrics.examples.basic_metric import BasicMetric

import matplotlib.pyplot as plt


class LcrLaneCrossings(BasicMetric):
    """
    Metric class LcrLaneCrossings
    """

    def _create_metric(self, town_map, log, criteria):
        """
        Implementation of the metric.
        """

        # Get ego vehicle id
        ego_id = log.get_ego_vehicle_id()

        dist_list = []
        frames_list = []

        # Get the frames the ego actor was alive and its transforms
        start, end = log.get_actor_alive_frames(ego_id)

        # Calculate distance to lane center
        for i in range(start, end + 1):
            transform = log.get_actor_transform(ego_id, i)
            if transform is None:
                continue
            lane_center = town_map.get_waypoint(
                transform.location, project_to_road=True, lane_type=1)
            if lane_center is None:
                continue
            dist = lane_center.transform.location.distance(transform.location)
            dist_list.append(dist)
            frames_list.append(i)
        # Print number oflane crossings
        lane_crossings = 0
        for i in range(1, len(dist_list)):
            if dist_list[i] * dist_list[i - 1] < 0:
                lane_crossings += 1
        print(f"Number of lane crossings: {lane_crossings}")

        with open('lane_crossings.json', 'w') as f:
            json.dump({'lane_crossings': lane_crossings}, f)

        # Use matplotlib to show the results
        plt.plot(frames_list, dist_list)
        plt.ylabel('Distance to Lane Center [meters]')
        plt.xlabel('Frame number')
        plt.title('Distance to Lane Center of the ego vehicle over time')
        plt.axhline(0, color='red', linestyle='--')
        plt.show()
