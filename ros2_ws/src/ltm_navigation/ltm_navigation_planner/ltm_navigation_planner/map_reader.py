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
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        self.map = None
        self.waypoints = None

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

        adjusted_map = self.map.copy()
        adjusted_map[adjusted_map < 254] = 0

        fuzzied_map = gaussian_filter(adjusted_map, sigma=3)
        
        crisp_map = np.zeros_like(fuzzied_map)
        crisp_map[fuzzied_map >= 128] = 255

        kernel = np.ones((12, 12), np.uint8)
        filtered_map = cv.erode(crisp_map, kernel, iterations=1)

        thinned_map = thin(filtered_map, max_iter=25)
        skeleton_map = skeletonize(thinned_map)

        scale = 0.5
        downscaled_skeleton = cv.resize(skeleton_map.astype(np.uint8), (0, 0), fx=scale, fy=scale)
        upscaled_skeleton = cv.resize(downscaled_skeleton, (skeleton_map.shape[1], skeleton_map.shape[0]))
        upscaled_skeleton = thin(upscaled_skeleton)

        waypoints = np.array(np.where(upscaled_skeleton)).T
        transformed_skeleton = np.zeros_like(waypoints).astype(np.float64)
        for i, point in enumerate(waypoints):
            transformed_skeleton[i] = point * self.resolution + self.origin[:2]

        return waypoints

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
