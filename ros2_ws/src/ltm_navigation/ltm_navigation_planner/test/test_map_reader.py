""" Test for map_reader.py
Revision: 1.0
Date: 19-08-2024
"""

import pytest
import numpy as np

from ltm_navigation_planner.map_reader import MapReader, \
    MAP_CELL_UNKNOWN, MAP_CELL_FREE, MAP_CELL_OCCUPIED

def test_map_reader_init():
    map_reader = MapReader()
    assert map_reader.map is None
    assert map_reader.resolution is None
    assert map_reader.origin is None
    assert map_reader.width is None
    assert map_reader.height is None

def test_map_reader_configure_metadata():
    map_reader = MapReader()

    # Set the metadata of the map
    # resolution = 0.05 m, origin = [0.0 m, 0.0 m, 0.0 rad], width = 100 cells, height = 100 cells
    map_reader.configure_metadata(0.05, [0.0, 0.0, 0.0], 100, 100)
    assert map_reader.resolution == 0.05 # meters
    assert map_reader.origin == [0.0, 0.0, 0.0] # meters, meters, radians
    assert map_reader.width == 100  # cells
    assert map_reader.height == 100 # cells
