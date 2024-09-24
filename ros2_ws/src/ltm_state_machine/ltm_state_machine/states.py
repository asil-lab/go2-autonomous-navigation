""" Here contains the states.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-05
"""

import re
from typing import Any, AnyStr

from ltm_shared_msgs.srv import PerformState

class State:
    """ State class is the base class for all states.
    """

    def __init__(self, name, id):
        self.name = name
        self.id = id
        self.input = None

    def transition(self):
        pass

    def request(self) -> tuple:
        return (self.get_service_name(), self.get_service_request())

    def __str__(self):
        return self.name
    
    def __repr__(self):
        return self.name + " " + str(self.id)
    
    def __eq__(self, other):
        return self.id == other.id
    
    def __ne__(self, other):
        return self.id != other.id
    
    def __hash__(self) -> int:
        return self.id
    
    def set_input(self, input: str):
        self.input = input

    def get_service_name(self):
        return "state_machine/" + re.sub(r'(?<!^)(?=[A-Z])', '_', self.name).lower()
    
    def get_service_request(self):
        request = PerformState.Request()
        request.current_state = self.id
        return request
    
    # def emergency_stop(self):
    #     return EmergencyStop()
    
    def error(self):
        return Error()


class Undefined(State):
    """ Undefined class is the state that is used to represent
    an undefined state.
    """

    def __init__(self):
        super().__init__("Undefined", 0)


class BootUp(State):
    """ BootUp class is the initial state of the state machine.
    """

    def __init__(self):
        super().__init__("BootUp", 1)

    def transition(self):
        # if self.input == "load_map":
        #     return LoadMap()
        # else:
        #     return CreateMap()
        return Shutdown()
        

# class LoadMap(State):
#     """ LoadMap class is the state that loads the map.
#     """

#     def __init__(self):
#         super().__init__("LoadMap", 2)

#     def transition(self):
#         return Localize()
    

# class CreateMap(State):
#     """ CreateMap class is the state that creates a map.
#     """

#     def __init__(self):
#         super().__init__("CreateMap", 3)

#     def transition(self):
#         if self.input == "stop":
#             return Localize()
#         else:
#             return CreateMap()


# class Localize(State):
#     """ Localization class is the state that localizes the robot
#     with respect to the map.
#     """

#     def __init__(self):
#         super().__init__("Localize", 4)

#     def transition(self):
#         return Idle()
    

# class Idle(State):
#     """ Idle class is the state that waits for a command.
#     """

#     def __init__(self):
#         super().__init__("Idle", 5)

#     def transition(self):
#         if self.input == "start":
#             return PlanPath()
#         else:
#             return Idle()
            

# class PlanPath(State):
#     """ PlanPath class is the state that creates a list of waypoints
#     to follow, and creates a path to follow.
#     """

#     def __init__(self):
#         super().__init__("PlanPath", 6)

#     def transition(self):
#         return Navigate()
    

# class Navigate(State):
#     """ Navigate class is the state that moves the robot to the
#     next waypoint.
#     """

#     def __init__(self):
#         super().__init__("Navigate", 7)

#     def transition(self):
#         if self.input == "reached":
#             return Scan()
#         elif self.input == "failed":
#             return ManualControl()
#         else:
#             return Navigate()


# class ManualControl(State):
#     """ ManualControl class is the state that allows manual control
#     of the robot.
#     """

#     def __init__(self):
#         super().__init__("ManualControl", 8)

#     def transition(self):
#         if self.input == "stop":
#             return Scan()
#         else:
#             return ManualControl()

# class Scan(State):
#     """ Scan class is the state that scans the environment at the
#     current waypoint in 360 degrees.
#     TODO: Implement smaller scanning state machine.
#     """

#     def __init__(self):
#         super().__init__("Scan", 9)

#     def transition(self):
#         if self.input == "stop":
#             return Home()
#         else:
#             return Navigate()
        

# class Home(State):
#     """ Home class is the state that returns the robot to the
#     starting position.
#     """

#     def __init__(self):
#         super().__init__("Home", 10)

#     def transition(self):
#         return Shutdown()
    

class Shutdown(State):
    """ Shutdown class is the state that shuts down the robot.
    """

    def __init__(self):
        super().__init__("Shutdown", 11)

    def transition(self):
        return Shutdown()

# class EmergencyStop(State):
#     """ EmergencyStop class is the state that stops the robot
#     immediately.
#     """

#     def __init__(self):
#         super().__init__("EmergencyStop", 12)

#     def transition(self):
#         return Shutdown()
    

# class Error(State):
#     """ Error class is the state that handles errors.
#     """

#     def __init__(self):
#         super().__init__("Error", 13)

#     def transition(self):
#         return Shutdown()
    

def get_all_service_names() -> list:
    return [state.get_service_name() for state in State.__subclasses__()]

def get_all_states() -> list:
    return [state() for state in State.__subclasses__()]

def get_state_by_id(id: int) -> State:
    for state in State.__subclasses__():
        if state().id == id:
            return state()
    return None