""" This is the main node for the ltm_state_machine package.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-23
"""

from itertools import groupby

def reduce_consecutive_duplicates(lst) -> list:
    """ Reduces consecutive duplicates in a list. """
    return [key for key, _ in groupby(lst)]