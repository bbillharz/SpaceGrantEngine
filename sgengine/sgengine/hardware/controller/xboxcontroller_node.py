# thanks stack overflow : https://stackoverflow.com/questions/46506850/how-can-i-get-input-from-an-xbox-one-controller-in-python

import sys
import time
import subprocess
import rclpy
import math
import threading
from inputs import get_gamepad
from rclpy.node import Node
from sgengine_messages.msg import XboxInput

class XboxControllerNode(Node):
    '''Node for handling input from an Xbox Controller'''

    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self) -> None:
        # ros stuff
        Node.__init__(self, "xboxcontroller")
        self._publisher = self.create_publisher(XboxInput, "xbox/input", 10)
        self._launched_auton = False

        # controller setup
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self): # publish the buttons/triggers that you care about in this methode      
        msg = XboxInput()
        msg.first = self.LeftJoystickX
        msg.second = self.LeftJoystickY
        msg.A = self.A
        msg.B = self.B
        msg.X = self.X
        msg.Y = self.Y
        print("Joystick X: " + self.LeftJoystickX)
        print("Joystick Y: " + self.LeftJoystickY)
        self._publisher.publish(msg)


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxControllerNode.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxControllerNode.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state

def main(args=None):
    """
    Main function which exclusively launches the XboxController node
    """
    # If the inputs package not found, exit
    if "inputs" not in sys.modules:
        return
    rclpy.init(args=args)
    controller = XboxControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    joy = XboxControllerNode()
    while True:
        joy.read()