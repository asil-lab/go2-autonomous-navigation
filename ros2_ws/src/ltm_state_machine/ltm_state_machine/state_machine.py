""" This is the main node for the ltm_state_machine package.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-07-28
"""

import os
from ltm_shared_msgs.msg import MissionState

class StateMachine:
    """ This is the state machine for the Lava Tube Mapping project. 
    This class is responsible for managing the state of the mission 
    and the state of the robot.
    """
    
    def __init__(self) -> None:
        self.current_state = MissionState.STATE_BOOTUP
        
    def get_current_state(self) -> int:
        return self.current_state
    
    def change_state(self, new_state) -> None:
        self.current_state = new_state
    
    def transition(self, input) -> bool:
        """ This function transitions the state machine to the next state.
        The input parameter is the input that triggers the transition to the next state.
        This parameter can be a sensor reading, a user input, or any other input that
        triggers a state transition.
        """
        # Bootup state
        if self.current_state == MissionState.STATE_BOOTUP:
            # Create map state if no input is provided
            if not input:
                self.change_state(MissionState.STATE_CREATE_MAP)
                return True
            # Check if the input file exists
            if os.path.exists(input):
                self.current_state = MissionState.STATE_LOAD_MAP
                return True
            else:
                self.current_state = MissionState.STATE_ERROR
                return False
        
        # Load map state
        
            
    def error_occured(self) -> bool:
        """ This function is called when an error occurs in the state machine.
        This function should log the error and transition the state machine to an error state.
        """
        self.current_state = MissionState.STATE_ERROR
        return True