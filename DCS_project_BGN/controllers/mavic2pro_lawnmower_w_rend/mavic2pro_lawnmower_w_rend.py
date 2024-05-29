# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example of Python controller for Mavic patrolling around the house.
   Open the robot window to see the camera view.
   This demonstrates how to go to specific world coordinates using its GPS, imu and gyroscope.
   The drone reaches a given altitude and patrols from waypoint to waypoint."""

from controller import Supervisor
from controller import Node
from controller import CameraRecognitionObject
import sys
import json
import time
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

def read_rend_json():
        # Read the JSON file
        try:
            with open('rendezvous.json', 'r') as json_file:
                rendezvous = json.load(json_file)
                print(type(rendezvous))
        except FileNotFoundError:
            print("JSON file 'rendezvous.json' not found.")
        return rendezvous
    
def write_rend_json(rendezvous_json):
    # Write the JSON file
    try:
        with open('rendezvous.json', 'w') as json_file:
            json.dump(rendezvous_json, json_file)
    except FileNotFoundError:
        print("JSON file 'rendezvous.json' not found.")


class Mavic (Supervisor):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0          # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    # Precision between the target position and the robot position in meters
    target_precision = 1.5

    def __init__(self):
        Supervisor.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Get and enable devices.
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.camera.recognitionEnable(self.time_step)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        self.camera_pitch_motor = self.getDevice("camera pitch")
        # self.camera_pitch_motor.setPosition(0.7)
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0

        self.robot_node_myself = self.getSelf()
        self.my_def = self.robot_node_myself.getDef()

    def set_position(self, pos):
        """
        Set the new absolute position of the robot
        Parameters:
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos


    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        """
        Move the robot to the given coordinates
        Parameters:
            waypoints (list): list of X,Y coordinates
            verbose_movement (bool): whether to print remaning angle and distance or not
            verbose_target (bool): whether to print targets or not
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """

        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):

            self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = 0
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                print("Target reached! New target: ",
                      self.target_position[0:2])

        # This will be in ]-pi;pi]
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])
        # This is now in ]-2pi;2pi[
        angle_left = self.target_position[2] - self.current_pose[5]
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        # non proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + (
                (self.target_position[1] - self.current_pose[1]) ** 2))
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance
            
    # Heat coming from wounded people detection function
    def detect_heat(self, identified_ppl):
        red_obj = self.camera.getRecognitionObjects()
        # for x in red_obj:
        #         print(type(x))
        already_found = False
        # if it is the begin of the simulation and the past-detections array is empty, add the object to the array if it has exactly one color
        # (e.g. is a person and emits heat)
        if len(red_obj) != 0 and len(identified_ppl) == 0:
            for z in red_obj:
                # number of colors of the current element of the loop in the real-time recognition image
                n_of_colors = z.getNumberOfColors()
                # check on the element array of colors length
                if n_of_colors == 1:
                    identified_ppl.append(z)
                    print("------ "+ self.my_def.upper() +" HEAT DETECTION ------")
                    person = self.getFromId(z.getId())
                    print(self.my_def.upper() + " at position " + str(self.current_pose[3:]) + 
                        ", person found --> Id: " + str(z.getId()) + "; Coordinates: " + str(person.getPosition())) # this getPosition() method works only
                                                                                                                    # for Pose-related nodes. Luckily it is:
                                                                                                                    # Pose --> Solid --> Robot
        # if it is NOT the begin of the simulation and the past-detections array has some elements, we must first check if the real time camera-detected elements
        # haven't been already recognized and saved
        elif len(identified_ppl) != 0:
            # Passing through all the elements that the drone camera recognize
            for x in red_obj:
                # number of colors of the current element of the real time recognition image
                n_of_colors = x.getNumberOfColors()
                if n_of_colors != 1:
                    continue
                # Passing through all the PREVIOUSLY AND ALREADY SAVED detected elements 
                for p in identified_ppl:
                    if (p.getId() == x.getId()):
                        #If the drone is recognizing an already detected person, set this var to True
                        already_found = True
                if(already_found == False):
                    # Saving the new detection
                    identified_ppl.append(x)
                    print("------ "+ self.my_def.upper() +" HEAT DETECTION ------")
                    person = self.getFromId(x.getId())
                    print(self.my_def.upper() + " at position " + str(self.current_pose[3:]) + 
                    ", person found --> Id: " + str(x.getId()) + "; Coordinates: " + str(person.getPosition()))
                else:
                    pass       


    def run(self, s, alt_meters):
        t1 = self.getTime()
        self.target_altitude = alt_meters
        rend_pos = []
        rend_mode = False
        z_targ = 0.2

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0
        x_offset = -15
        y_offset = 0

        if (self.my_def == "drone_0"):
            waypoints = waypoints_dict[s]["drone_0"]
            x_offset = -16
            y_offset = 1
            print("--- drone_0 waypoints ---")
            for cord in waypoints:
                cord[0] = cord[0] + x_offset
                cord[1] = cord[1] + y_offset
            print(waypoints)
        elif (self.my_def == "drone_1"):
            x_offset = -16
            y_offset = -1
            waypoints = waypoints_dict[s]["drone_1"]
            print("--- drone_1 waypoints ---")
            for cord in waypoints:
                cord[0] = cord[0] + x_offset
                cord[1] = cord[1] + y_offset
            print(waypoints)
        elif (self.my_def == "drone_2"):
            x_offset = -14
            y_offset = 1
            waypoints = waypoints_dict[s]["drone_2"]
            print("--- drone_2 waypoints ---")
            for cord in waypoints:
                cord[0] = cord[0] + x_offset
                cord[1] = cord[1] + y_offset
            print(waypoints)
        elif (self.my_def == "drone_3"):
            x_offset = -14
            y_offset = -1
            waypoints = waypoints_dict[s]["drone_3"]
            print("--- drone_3 waypoints ---")
            for cord in waypoints:
                cord[0] = cord[0] + x_offset
                cord[1] = cord[1] + y_offset
            print(waypoints)
        else:
            pass

        while self.step(self.time_step) != -1:
            # get the initial coord of the drone and prepare the rendezvous JSON
            if len(rend_pos) == 0:
                init_pos = self.gps.getValues()
                rend_pos.append(init_pos)
                rend_pos[0][2] = max_alt
                # Read the JSON, in order to add the new part of the dictionary related to this drone
                rendezvous = read_rend_json()
                print(rendezvous)
                # The new element of the JSON file is the one related to the specific mavic which is using this controller, having as key its def. and as value its pos.
                rendezvous.update({self.my_def : rend_pos})
                print(rendezvous)
                print(type(rendezvous))
                # write_rend_json(rendezvous)
            
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])
            
            if altitude > self.target_altitude - 1:
                # as soon as it reach the target altitude, compute the disturbances to go to the given waypoints.
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(
                        waypoints)
                    t1 = self.getTime()

            # Find people (represented by the red blocks) in the camera image
            self.detect_heat(identified_ppl)

            
            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)


# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5
robot = Mavic()
    # 800, 600, 400
# altitudes = [40, 30, 20]

            # 650, 450, 350
altitudes = [32.5, 22.5, 17.5]
waypoints_dict = json.load(open("waypoints.json"))
identified_ppl = []
max_alt = altitudes[0]
for s in waypoints_dict:
    if (s.lower() == "high"):
        print(robot.my_def + " flying in high setup")
        alt_meters = altitudes[0]
    elif (s.lower() == "medium"):
        print(robot.my_def + " flying in medium setup")
        alt_meters = altitudes[1]
    elif (s.lower() == "low"):
        print(robot.my_def + " flying in low setup")
        alt_meters = altitudes[2]
    else:
        pass
    robot.run(s, alt_meters)


