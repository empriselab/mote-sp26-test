import numpy as np
import math

from nav_msgs.msg import OccupancyGrid

MAX_LIDAR_RANGE = 5.0


def raycast(map, map_size, origin, direction, max_ittr):
    x, y = origin
    width, height = map_size
    dx, dy = direction

    for i in range(int(max_ittr)):
        x += dx
        y += dy

        # Check if the ray is out of bounds
        if x < 0 or x >= width or y < 0 or y >= height:
            return max_ittr

        # Check if the ray hit something
        if map[int(y) * width + int(x)] == 100:
            return i

    return max_ittr


def gen_lidar_scan(particle, map: OccupancyGrid, num_rays):
    # Extract data from the particle
    x = int((particle[0] - map.info.origin.position.x) / map.info.resolution)
    y = int((particle[1] - map.info.origin.position.y) / map.info.resolution)
    theta = particle[2]
    map_data = np.array(map.data)
    map_ranges = []

    angles = np.linspace(-3.14, 3.14, num_rays)

    # Get ground truth from the map
    for angle in angles:
        map_ranges.append(
            raycast(
                map_data,
                (map.info.width, map.info.height),
                (x, y),
                (np.cos(angle + theta), np.sin(angle + theta)),
                MAX_LIDAR_RANGE / map.info.resolution,
            )
            * map.info.resolution
        )

    return map_ranges