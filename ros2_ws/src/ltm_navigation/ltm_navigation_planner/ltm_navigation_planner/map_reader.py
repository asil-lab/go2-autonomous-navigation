""" This script is responsible for reading the map and creating waypoints around the obstacles.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

import numpy as np

MAP_CELL_UNKNOWN    = -1
MAP_CELL_FREE       =  0
MAP_CELL_OCCUPIED   =  1

class MapReader:
    """ TODO:
    - [ ] Configure the metadata of the map.
    - [ ] Read the map, and create a path around the obstacles.
    - [ ] 
    """

    def __init__(self):
        self.map = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

    def configure_metadata(self, resolution: float, origin: list, width: int, height: int) -> None:
        self.resolution = resolution
        self.origin = origin
        self.width = width
        self.height = height

    def read_map(self, map: list) -> None:
        self.map = np.array(map).reshape(self.height, self.width).astype(np.uint8)

        # Set the unknown cells to occupied
        self.map = np.where(self.map == MAP_CELL_UNKNOWN, MAP_CELL_OCCUPIED, self.map)

    def create_path(self) -> list:
        # Create a path around the obstacles
        pass