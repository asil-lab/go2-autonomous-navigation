""" This script is responsible for reading the map and creating waypoints around the obstacles.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

import re
import numpy as np
import matplotlib.pyplot as plt

import cv2 as cv
from scipy.ndimage import gaussian_filter
from skimage.measure import find_contours

MAP_CELL_UNKNOWN    = 205
MAP_CELL_FREE       = 254
MAP_CELL_OCCUPIED   = 0

# TODO: Parameterize these values
MAP_PADDING = 5
MAP_SMOOTHING_SIGMA = 1.0
MAP_CONTINUITY_THRESHOLD = 0.8
MAP_CONTOUR_SIZE_THRESHOLD = 50
MAP_WAYPOINT_DIVISION = 10
MAP_MONTE_CARLO_ITERATIONS = 50000
MAP_MONTE_CARLO_REGION_THRESHOLD = 128
MAP_ORIENTATION_KERNEL_SIZE = 5

class MapReader:
    """ TODO:
    - [ ] Configure the metadata of the map.
    - [ ] Read the map, and create a path around the obstacles.
    - [ ] 
    """

    def __init__(self):
        self.map = None
        self.transformed_map = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        self.contours = None
        self.waypoints = None
        self.orientations = None

    def configure_metadata(self, resolution: float, origin: list, width: int, height: int) -> None:
        self.resolution = resolution
        self.origin = origin
        self.width = width
        self.height = height

    def read(self) -> np.ndarray:
        # Transform the map into XY coordinates
        self.transformed_map = self.transform_map(self.map)

        # Preprocess the map
        padded_map = self.pad_map(self.map)
        smoothed_map = self.smooth_map(padded_map)
        sampled_map = self.monte_carlo(smoothed_map)
        truncated_map = self.truncate_map(sampled_map)
        self.map = self.smooth_map(truncated_map)

        # Extract the contours from the map
        contours = self.extract_contours(self.map)
        self.contours = self.filter_contours(contours)

        # Determine the waypoints and orientations
        self.waypoints = self.determine_waypoints(self.contours)
        self.orientations = self.determine_orientation(self.waypoints, truncated_map)

        return self.combine_waypoints(self.waypoints, self.orientations)

    def read_map_list(self, map: list) -> None:
        self.map = np.array(map).reshape(self.height, self.width).astype(np.uint8)

        # Set the unknown cells to occupied
        self.map = np.where(self.map == MAP_CELL_UNKNOWN, MAP_CELL_OCCUPIED, self.map)

    def read_map_pgm(self, filename: str, byteorder='>') -> None:
        self.map = plt.imread(filename)
    
        # Set the unknown cells to occupied
        # self.map = np.where(self.map == MAP_CELL_UNKNOWN, MAP_CELL_OCCUPIED, self.map)

    def pad_map(self, input: np.ndarray) -> np.ndarray:
        return np.pad(input, MAP_PADDING, mode='constant', constant_values=MAP_CELL_UNKNOWN)

    def truncate_map(self, input: np.ndarray) -> np.ndarray:
        return input[MAP_PADDING:-MAP_PADDING, MAP_PADDING:-MAP_PADDING]

    def smooth_map(self, input: np.ndarray) -> np.ndarray:
        return gaussian_filter(input, sigma=MAP_SMOOTHING_SIGMA)
    
    def transform_map(self, input: np.ndarray) -> np.ndarray:
        assert self.resolution is not None, 'Resolution is not set'
        assert self.origin is not None, 'Origin is not set'
        
        # Create a meshgrid for the map
        x = np.linspace(0.0, self.width * self.resolution, self.width)
        y = np.linspace(0.0, self.height * self.resolution, self.height)
        xx, yy = np.meshgrid(x, y)

        # # Center the map around the origin
        xx += self.origin[0]
        yy += self.origin[1]

        # Transform the map
        return np.stack((yy, xx, input), axis=-1)

    def extract_contours(self, input: np.ndarray) -> list:
        return find_contours(input, MAP_CONTINUITY_THRESHOLD)
    
    def filter_contours(self, contours: list) -> list:
        return [contour for contour in contours if len(contour) > MAP_CONTOUR_SIZE_THRESHOLD]
    
    def monte_carlo(self, input: np.ndarray) -> np.ndarray:
        output = np.zeros_like(input)
        LOWER_BOUND = MAP_PADDING * np.ones_like(input.shape, dtype=int)
        UPPER_BOUND = (np.array(input.shape) - MAP_PADDING).astype(int)

        for _ in range(MAP_MONTE_CARLO_ITERATIONS):
            random_point = np.random.randint(LOWER_BOUND, UPPER_BOUND)
            
            # Skip if the random point is on occupied or unknown cell
            if input[random_point[0], random_point[1]] < MAP_CELL_FREE:
                continue

            # Get the circular region around the random point
            y, x = np.ogrid[
                -random_point[0]:input.shape[0]-random_point[0], 
                -random_point[1]:input.shape[1]-random_point[1]]
            mask = x**2 + y**2 <= MAP_PADDING**2
            region = input[mask]

            # Skip if the mean of the region is less than the continuity threshold
            if np.mean(region) < MAP_MONTE_CARLO_REGION_THRESHOLD:
                continue

            # Sample the random point
            output[random_point[0], random_point[1]] = MAP_CELL_FREE

        return output
    
    def determine_waypoints(self, contours: list) -> list:
        waypoints = []
        for contour in contours:
            for point in contour[::MAP_WAYPOINT_DIVISION]:
                waypoints.append(point)
        return waypoints

    def determine_orientation(self, waypoints: list, map: np.ndarray) -> list:
        orientations = []

        for waypoint in waypoints:
            # Get the orientation of the waypoint
            x, y = waypoint.astype(int)
            y_min = max(y - MAP_PADDING, 0)
            y_max = min(y + MAP_PADDING, self.map.shape[1])
            x_min = max(x - MAP_PADDING, 0)
            x_max = min(x + MAP_PADDING, self.map.shape[0])

            # Get the region around the waypoint
            # TODO: For somee reason, the region is not being extracted correctly
            # when using numpy slicing
            region = np.zeros((2*MAP_PADDING, 2*MAP_PADDING))
            for i in range(x_min, x_max):
                for j in range(y_min, y_max):
                    region[i - x_min, j - y_min] = map[i, j]
            
            # Get the gradient of the region
            dx = cv.Sobel(region, cv.CV_64F, 1, 0, ksize=MAP_ORIENTATION_KERNEL_SIZE)
            dy = cv.Sobel(region, cv.CV_64F, 0, 1, ksize=MAP_ORIENTATION_KERNEL_SIZE)

            # Calculate the orientation of the waypoint
            orientation = np.arctan2(dy.mean(), dx.mean()) + np.pi
            orientations.append(orientation)

        return orientations
    
    def combine_waypoints(self, waypoints: list, orientations: list) -> list:
        output = np.zeros((len(waypoints), 3))
        for i, waypoint in enumerate(waypoints):
            image_y, image_x = waypoint[:2].astype(int)
            pos_y, pos_x = self.transformed_map[image_y, image_x][:2]
            output[i] = np.array([pos_x, pos_y, orientations[i]])
        return output