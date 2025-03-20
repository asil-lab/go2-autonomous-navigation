""" This script is responsible for reading the map and creating waypoints around the obstacles.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

import os
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

import cv2 as cv
from scipy.ndimage import gaussian_filter
from skimage.measure import find_contours
from skimage.morphology import thin, skeletonize

MAP_CELL_LIST_UNKNOWN   = -1
MAP_CELL_LIST_FREE      = 0
MAP_CELL_LIST_OCCUPIED  = 100

MAP_CELL_PGM_UNKNOWN    = 205
MAP_CELL_PGM_FREE       = 254
MAP_CELL_PGM_OCCUPIED   = 0

MAP_SMOOTHING_SIGMA = 4
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
        # Metadata
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        # Map data
        self.map = None
        self.waypoints = None
        self.results_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), '..', 'results', 
            datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))

    def configure_metadata(self, resolution: float, origin: np.ndarray, width: int, height: int) -> None:
        self.resolution = resolution
        self.origin = origin
        self.width = width
        self.height = height

    def configure_metadata(self, resolution: float, origin: list, width: int, height: int) -> None:
        self.resolution = resolution
        self.origin = np.array(origin)
        self.width = width
        self.height = height

    def read(self, plot=False) -> np.ndarray:
        # self.plot_map(self.map, 'Original Map', plot)
        self.save_map_as_png(self.map, 'original_map')

        # Adjust the map to only have free and occupied cells
        # Unknown cells are set as occupied
        adjusted_map = self.map.copy()
        adjusted_map[adjusted_map < 254] = 0
        self.save_map_as_png(adjusted_map, 'adjusted_map')

        # Smooth the map
        fuzzied_map = gaussian_filter(adjusted_map, sigma=3)
        crisp_map = np.zeros_like(fuzzied_map)
        crisp_map[fuzzied_map >= 128] = 255
        self.save_map_as_png(crisp_map, 'crisp_map')

        # Erode the map to remove small obstacles
        kernel = np.ones((3, 3), np.uint8)
        filtered_map = cv.erode(crisp_map, kernel, iterations=1)
        self.save_map_as_png(filtered_map, 'filtered_map')

        # Get the largest area of the map by finding the largest contour
        contours, _ = cv.findContours(filtered_map, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv.contourArea)
        contour_map = np.zeros_like(filtered_map)
        cv.drawContours(contour_map, [largest_contour], -1, (255), -1)
        self.save_map_as_png(contour_map, 'contour_map')

        # Get the skeleton of the area
        # thinned_map = thin(contour_map) #, max_iter=25)
        skeleton_map = skeletonize(contour_map)
        self.save_map_as_png(skeleton_map, 'skeleton_map')

        # Convert every point in the skeleton into Cartesian coordinates
        skeleton_points = np.array(np.where(skeleton_map)).T
        waypoints = np.zeros_like(skeleton_points).astype(np.float64)
        for i, point in enumerate(skeleton_points):
            waypoints[i] = point * self.resolution + self.origin[:2]

        return waypoints[:, [1, 0]] # Convert to (x, y) format

    def read_map_list(self, map: list) -> None:
        self.map = np.array(map).reshape(self.height, self.width).astype(np.uint8)
        self.map = np.invert(self.map)

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

    def save_map_as_png(self, map: np.ndarray, title: str) -> None:
        # Assuming the map is in grayscale
        map = Image.fromarray(map)
        map.save(os.path.join(self.results_dir, f'{title}.png'))
