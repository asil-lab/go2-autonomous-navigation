""" This is the main node for the ltm_state_machine package.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-23
"""

def reduce_consecutive_duplicates(lst) -> list:
    """ Reduces consecutive duplicates that only appear at the end of a list."""
    if len(lst) == 0:
        return lst
    if lst[-1] == lst[-2]:
        lst.pop()
    return lst