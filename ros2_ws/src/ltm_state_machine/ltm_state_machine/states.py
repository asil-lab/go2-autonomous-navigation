""" Here contains the states.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-05
"""

class State:
    """ State class is the base class for all states.
    """

    def __init__(self, name, id):
        self.name = name
        self.id = id

    def transition(self):
        pass

    def __str__(self):
        return self.name
    
    def __repr__(self):
        return self.name + " " + str(self.id)
    
    def __eq__(self, other):
        return self.id == other.id
    
    def __ne__(self, other):
        return self.id != other.id
    

class BootUp(State):
    """ BootUp class is the initial state of the state machine.
    """

    def __init__(self):
        super().__init__("BootUp", 0)
        self.is_map_loaded = False

    def transition(self):
        if self.is_map_loaded:
            return LoadMap()
        else:
            return CreateMap()
        

class LoadMap(State):
    """ LoadMap class is the state that loads the map.
    """

    def __init__(self):
        super().__init__("LoadMap", 1)

    def transition(self):
        return Localize()
    

class CreateMap(State):
    """ CreateMap class is the state that creates a map.
    """

    def __init__(self):
        super().__init__("CreateMap", 2)
        self.is_button_pressed = False

    def transition(self):
        if self.is_button_pressed:
            return Localize()
        else:
            return CreateMap()
    

class Localize(State):
    """ Localization class is the state that localizes the robot
    with respect to the map.
    """

    def __init__(self):
        super().__init__("Localize", 3)

    def transition(self):
        return Idle()
    

class Idle(State):
    """ Idle class is the state that waits for a command.
    """

    def __init__(self):
        super().__init__("Idle", 4)
        self.is_button_pressed = False

    def transition(self):
        if self.is_button_pressed:
            return PlanPath()
        else:
            return Idle()
            

class PlanPath(State):
    """ PlanPath class is the state that creates a list of waypoints
    to follow, and creates a path to follow.
    """

    def __init__(self):
        super().__init__("PlanPath", 5)

    def transition(self):
        return Navigate()
    

class Navigate(State):
    """ Navigate class is the state that moves the robot to the
    next waypoint.
    """

    def __init__(self):
        super().__init__("Navigate", 6)

    def transition(self):
        return Scan()
    

class Scan(State):
    """ Scan class is the state that scans the environment at the
    current waypoint in 360 degrees.
    TODO: Implement smaller scanning state machine.
    """

    def __init__(self):
        super().__init__("Scan", 7)
        self.is_at_final_waypoint = False

    def transition(self):
        if self.is_at_final_waypoint:
            return Idle()
        else:
            return PlanPath()
        

class Home(State):
    """ Home class is the state that returns the robot to the
    starting position.
    """

    def __init__(self):
        super().__init__("Home", 8)

    def transition(self):
        return Shutdown()
    

class Shutdown(State):
    """ Shutdown class is the state that shuts down the robot.
    """

    def __init__(self):
        super().__init__("Shutdown", 9)

    def transition(self):
        pass


class EmergencyStop(State):
    """ EmergencyStop class is the state that stops the robot
    immediately.
    """

    def __init__(self):
        super().__init__("EmergencyStop", 10)

    def transition(self):
        return Shutdown()
    

class Error(State):
    """ Error class is the state that handles errors.
    """

    def __init__(self):
        super().__init__("Error", 11)

    def transition(self):
        return Shutdown()
    

class ManualControl(State):
    """ ManualControl class is the state that allows manual control
    of the robot.
    """

    def __init__(self):
        super().__init__("ManualControl", 12)

    def transition(self):
        return Idle()