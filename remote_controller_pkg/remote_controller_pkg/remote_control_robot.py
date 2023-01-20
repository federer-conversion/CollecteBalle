from evdev import InputDevice, ecodes, list_devices
import re
import sys
import select
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

# -----------------------------------------------------------------------------------------------------------------------

MAX_LIN_VEL = 1.
MAX_ANG_VEL = 1.5

threshold = 1000

acquisition_frequency = 30

# -----------------------------------------------------------------------------------------------------------------------

def select_devices(device_dir='/dev/input'):
    """
    This function will search the controler available in /dev/input and ask the user which one will be used.
    A error message can appear if no game controlers are detected or if the user give a wrong controller id
    Function get from the source web site (with little modification: can choose only 1 controler not multiple)
    """

    def devicenum(device_path):
        """
        Function to sort the devices list with their port number
        """
        digits = re.findall(r'\d+$', device_path)
        return [int(i) for i in digits]

    # Get all the detected input devices and sorted them
    devices = sorted(list_devices(device_dir), key=devicenum)
    
    # Convert them into InputDevice object
    devices = [InputDevice(path) for path in devices]
    
    # If no devices are detected then send an error to stop the programm
    if not devices:
        msg = 'error: no input devices found (do you have rw permission on %s/*?)'
        print(msg % device_dir, file=sys.stderr)
        sys.exit(1)
    
    # Print in the terminal the devices list
    dev_format = '{0:<3} {1.path:<20} {1.name:<35} {1.phys:<35} {1.uniq:<4}'
    dev_lines = [dev_format.format(num, dev) for num, dev in enumerate(devices)]
    print('ID  {:<20} {:<35} {:<35} {}'.format('Device', 'Name', 'Phys', 'Uniq'))
    print('-' * len(max(dev_lines, key=len)))
    print('\n'.join(dev_lines))
    print()

    # Ask the user to chose one
    choices = input('Select devices [0-%s]: ' % (len(dev_lines) - 1))

    # Check user input
    try:
        choices = devices[int(choices)]
    except ValueError:
        choices = None

    # If wrong value
    if not choices:
        msg = 'error: invalid input - please enter one available number'
        print(msg, file=sys.stderr)
        sys.exit(1)

    # Return selected device
    return choices

# -----------------------------------------------------------------------------------------------------------------------

class ControlPublisher(Node):

    def __init__(self,fd_to_device):
        super().__init__('remote_controller_node')
        self.fd_to_device = fd_to_device
        self.msg = Twist()
        self.publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.publisher.publish(self.msg)
        self.acquisition_loop()

    def acquisition_loop(self):
        linear_speed_state = 0
        while True:
            # controler input acquisition with a time_break (if no new command then break => avoid blocking function)
            r, w, e = select.select(self.fd_to_device, [], [], 0.5/acquisition_frequency)
            for fd in r:
                # When a new event is detected
                for event in self.fd_to_device[fd].read():
                    # Check if the event is not an info like sync (if not its an input)
                    if event.type != ecodes.EV_SYN:
                        if int(event.code)==311 and int(event.value)==1:
                            self.msg.angular.z = -1*MAX_ANG_VEL
                            self.publisher.publish(self.msg)
                        if int(event.code)==311 and int(event.value)==0:
                            self.msg.angular.z = MAX_ANG_VEL
                            self.publisher.publish(self.msg)
                            self.msg.angular.z = 0.
                            self.publisher.publish(self.msg)
                        if int(event.code)==310 and int(event.value)==1:
                            self.msg.angular.z = MAX_ANG_VEL
                            self.publisher.publish(self.msg)
                        if int(event.code)==310 and int(event.value)==0:
                            self.msg.angular.z = -1*MAX_ANG_VEL
                            self.publisher.publish(self.msg)
                            self.msg.angular.z = 0.
                            self.publisher.publish(self.msg)
                        if int(event.code)==17 and int(event.value)==1:
                            linear_speed_state = -1
                            self.msg.linear.x = -1*MAX_LIN_VEL
                            self.publisher.publish(self.msg)
                        if int(event.code)==17 and int(event.value)==0:
                            if linear_speed_state ==1:
                                self.msg.linear.x = -1*MAX_LIN_VEL
                                self.publisher.publish(self.msg)
                                time.sleep(5./acquisition_frequency)
                            if linear_speed_state ==-1:
                                self.msg.linear.x = MAX_LIN_VEL
                                self.publisher.publish(self.msg)
                                time.sleep(5./acquisition_frequency)
                            self.msg.linear.x = 0.
                            self.publisher.publish(self.msg)
                            linear_speed_state = 0
                        if int(event.code)==17 and int(event.value)==-1:
                            linear_speed_state = 1
                            self.msg.linear.x = MAX_LIN_VEL
                            self.publisher.publish(self.msg)

                        

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

# -----------------------------------------------------------------------------------------------------------------------

def main(args=None):
    devices = select_devices()
    print("{} have been selected!".format(devices.name))
    print("------------------------")

    fd_to_device = {dev.fd: dev for dev in [devices]}

    rclpy.init(args=args)

    control_publisher = ControlPublisher(fd_to_device)

    rclpy.spin(control_publisher)

    control_publisher.destroy_node()
    rclpy.shutdown()


# -----------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    main()
