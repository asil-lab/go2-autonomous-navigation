""" This script is responsible for reading the map and creating waypoints around the obstacles.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

import numpy as np
import matplotlib.pyplot as plt

import cv2 as cv
from scipy.ndimage import gaussian_filter
from skimage.measure import find_contours

MAP_CELL_LIST_UNKNOWN   = -1
MAP_CELL_LIST_FREE      = 0
MAP_CELL_LIST_OCCUPIED  = 1

MAP_CELL_PGM_UNKNOWN    = 205
MAP_CELL_PGM_FREE       = 254
MAP_CELL_PGM_OCCUPIED   = 0

MAP_SMOOTHING_SIGMA = 3
MAP_CRISP_THRESHOLD = 250
MAP_CONTOUR_SKIP = 20
MAP_GRID_SIZE = 10
MAP_MERGE_DISTANCE = 10

class MapReader:
    """ TODO:
    - [ ] Configure the metadata of the map.
    - [ ] Read the map, and create a path around the obstacles.
    - [ ] 
    """

    def __init__(self):
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        self.map = None
        self.waypoints = None

    def configure_metadata(self, resolution: float, origin: list, width: int, height: int) -> None:
        self.resolution = resolution
        self.origin = origin
        self.width = width
        self.height = height

    def read(self, plot=False) -> np.ndarray:
        self.plot_map(self.map, 'Original Map', plot)

        # Apply Gaussian smoothing to the map
        smoothed_map = gaussian_filter(self.map, sigma=MAP_SMOOTHING_SIGMA)
        self.plot_map(smoothed_map, 'Smoothed Map', plot)

        # Crisp the map
        crisp_map = np.zeros(smoothed_map.shape)
        crisp_map[smoothed_map >= MAP_CRISP_THRESHOLD] = 1
        self.plot_map(crisp_map, 'Crisp Map', plot)

        # Find the contours of the map
        contours = find_contours(crisp_map, 0.8, fully_connected='high')

        # Skip several points in each contour
        waypoints = []
        for contour in contours:
            waypoints.extend(contour[::MAP_CONTOUR_SKIP])

        # Create a 2D grid of points
        x = np.arange(0, self.map.shape[1], MAP_GRID_SIZE)
        y = np.arange(0, self.map.shape[0], MAP_GRID_SIZE)
        xx, yy = np.meshgrid(x, y)
        grid = np.vstack((yy.ravel(), xx.ravel())).T

        # Fit the grid into the crisp map
        grid = grid[crisp_map[grid[:, 0].astype(int), grid[:, 1].astype(int)] == 1]

        # Merge the waypoints and the grid
        waypoints.extend(grid)

        # Combine points that are close to each other
        merged_waypoints = []
        for point in waypoints:
            if not merged_waypoints:
                merged_waypoints.append(point)
            else:
                if np.linalg.norm(merged_waypoints[-1] - point) > MAP_MERGE_DISTANCE:
                    merged_waypoints.append(point)
        
        waypoints = merged_waypoints

        return np.array(waypoints)

    def read_map_list(self, map: list) -> None:
        self.map = np.array(map).reshape(self.height, self.width)
        self.map[self.map == MAP_CELL_LIST_UNKNOWN] = MAP_CELL_PGM_UNKNOWN
        self.map[self.map == MAP_CELL_LIST_FREE] = MAP_CELL_PGM_FREE
        self.map[self.map == MAP_CELL_LIST_OCCUPIED] = MAP_CELL_PGM_OCCUPIED

    def read_map_pgm(self, filename: str) -> None:
        self.map = plt.imread(filename, format='pgm')
    
    def plot_map(self, map: np.ndarray, title: str, plot=False) -> None:
        if plot:
            plt.figure(figsize=(5, 5))
            plt.imshow(map, cmap='gray')
            plt.title(title)
            plt.show()

    def plot_current_map(self) -> None:
        self.plot_map(self.map)
