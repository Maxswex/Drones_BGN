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

from mimetypes import init
from controller import Supervisor
from controller import CameraRecognitionObject
import sys
from pathlib import Path
project_root = Path(__file__).resolve().parents[3]
mqtt_parameters_path = project_root / 'mqtt_parameters.py'
sys.path.append(str(project_root))
from mqtt_parameters import MqttConfigurationParameters
import json
import paho.mqtt.client as mqtt
import time
import os
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")


#---------- MQTT communications methods ----------

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

def publish_device_info(drone_id):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.SCANNING_TOPIC,
        drone_id,
        MqttConfigurationParameters.DRONE_INFO_TOPIC)
    # Get only device info
    mqtt_client.publish(target_topic, drone_id, 0, True)
    print(f"Vehicle Info Published: Topic: {target_topic} Payload: {drone_id}")

def publish_telemetry_data(drone_id, drone_pos, timestamp):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.SCANNING_TOPIC,
        drone_id,
        MqttConfigurationParameters.DRONE_TELEMETRY_TOPIC)
    # Get only telemetry and timestamp
    device_payload_string = f"drone position: {drone_pos}, time: {timestamp}"
    mqtt_client.publish(target_topic, device_payload_string, 0, False)
    print(f"Telemetry Data Published: Topic: {target_topic} \nPayload: {device_payload_string}\n")


def publish_sensible_coordinates(drone_id, detection_id,  heat_pos, timestamp):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.SCANNING_TOPIC,
        drone_id,
        MqttConfigurationParameters.DRONE_SENSIBLE_COORDINATES_TOPIC)
    # Get id detection, heat position and timestamp
    device_payload_string = f"heat detection id: {detection_id}, heat position: {heat_pos}, time: {timestamp}"
    mqtt_client.publish(target_topic, device_payload_string, 0, False)
    print(f"FOUND A SENSIBLE COORDINATES!! Published to the following topic: {target_topic} \nPayload: {device_payload_string}")


#------------------------------------------


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

def read_json(file):
        # Read the JSON file
        try:
            with open(file, 'r') as json_file:
                msg = json.load(json_file)
        except FileNotFoundError:
            print("JSON file not found.")
        return msg
    
def write_json(msg, file, indentation):
    # Write the JSON file
    try:
        with open(file, 'w') as json_file:
            json.dump(msg, json_file, indent=indentation)
    except FileNotFoundError:
        print("JSON file not found.")


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
    # Precision between the target position and the robot position in meters, increased with respect to the original controller
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
        # non-proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + (
                (self.target_position[1] - self.current_pose[1]) ** 2))
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance
            
    # Heat (wounded people) detection function
    def detect_heat(self, identified_ppl, count):
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
                    timestamp= time.strftime('%Y-%M-%DT%H:%M:%S', time.localtime())
                    print("------ "+ str(self.my_def).strip().upper() +" HEAT DETECTION ------")
                    person = self.getFromId(z.getId())
                    print(str(self.my_def).strip().upper() + " at position " + str(self.current_pose[3:]) +
                        ", person found --> Id: " + str(z.getId()) + "; Coordinates: " + str(person.getPosition())) # this getPosition() method works only
                                                                                                                    # for Pose-related nodes. Luckily it is:
                                                                                                                    # Pose --> Solid --> Robot

                    # ---------- MQTT ----------

                    # Preparing data to publish
                    heat_pos = {
                        "x": person.getPosition()[0],
                        "y": person.getPosition()[1],
                        "z": person.getPosition()[2]}

                    # Publishing wounded coord in MQTT
                    publish_sensible_coordinates(str(self.my_def).strip().upper(), str(z.getId()),
                                                 str(heat_pos), str(timestamp))
                    # -------------------------

        # if it is NOT the beginning of the simulation and the past-detections array has some elements, we must first check if the real time camera-detected elements
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
                # Preparing data to publish
                timestamp = time.strftime('%Y-%M-%DT%H:%M:%S', time.localtime())
                drone_pos = {
                    "x": self.current_pose[3],
                    "y": self.current_pose[4],
                    "z": self.current_pose[5]}
                if(already_found == False):
                    # Saving the new detection
                    identified_ppl.append(x)
                    print("------ "+ str(self.my_def).strip().upper() +" HEAT DETECTION ------")
                    person = self.getFromId(x.getId())
                    print(str(self.my_def).strip().upper() + " at position " + str(self.current_pose[3:]) +
                    ", person found --> Id: " + str(x.getId()) + "; Coordinates: " + str(person.getPosition()))

                    # ---------- MQTT ----------

                    # Preparing data to publish
                    heat_pos = {
                        "x": person.getPosition()[0],
                        "y": person.getPosition()[1],
                        "z": person.getPosition()[2]}

                    # Publishing wounded coord in MQTT
                    publish_sensible_coordinates(str(self.my_def).strip().upper(), str(x.getId()),
                                                 str(heat_pos), str(timestamp))

                else:
                    #if count%100 == 0:
                    #   publish_telemetry_data(str(self.my_def).strip().upper(), self.current_pose[3:], str(timestamp))
                    pass       


    def run(self):
        count = 0
        t1 = self.getTime()
        self.target_altitude = alt_meters
        rend_waypoint = [0, 0, 0]

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0
        x_offset = -15
        y_offset = 0

        if (str(self.my_def).strip().upper() == "S0"):
            waypoints_law = waypoints_dict[s]["S0"]
            x_offset = -16
            y_offset = 1
            print("--- drone S0 waypoints ---")
            for cord in waypoints_law:
                cord[0] = cord[0] + x_offset
                cord[1] = cord[1] + y_offset
            print(waypoints_law)
        elif (str(self.my_def).strip().upper() == "S1"):
            x_offset = -16
            y_offset = -1
            waypoints_law = waypoints_dict[s]["S1"]
            print("--- drone S1 waypoints ---")
            for cord in waypoints_law:
                cord[0] = cord[0] + x_offset
                cord[1] = cord[1] + y_offset
            print(waypoints_law)
        elif (str(self.my_def).strip().upper() == "S2"):
            x_offset = -14
            y_offset = 1
            waypoints_law = waypoints_dict[s]["S2"]
            print("--- drone S2 waypoints ---")
            for cord in waypoints_law:
                cord[0] = cord[0] + x_offset
                cord[1] = cord[1] + y_offset
            print(waypoints_law)
        elif (str(self.my_def).strip().upper() == "S3"):
            x_offset = -14
            y_offset = -1
            waypoints_law = waypoints_dict[s]["S3"]
            print("--- drone S3 waypoints ---")
            for cord in waypoints_law:
                cord[0] = cord[0] + x_offset
                cord[1] = cord[1] + y_offset
            print(waypoints_law)
        else:
            pass

        while self.step(self.time_step) != -1:
            count += 1
            # get the initial coord of the drone and prepare the rendezvous JSON
            if rend_waypoint == [0, 0, 0]:
                rend_waypoint = self.gps.getValues()
                # Substituting to the initial coordinates the the max altitudes to the z-coord: this in order to safely return to the initial position for the
                # rendevousz since we did not build an obstacle avoidance controller
                rend_waypoint.pop(2)
                # Checking the initial coordinates, which will be the rendevousz ones
                print(rend_waypoint)
                waypoints_rend = []
                waypoints_rend.append(rend_waypoint)

            # Checking if the drone are in "Rendevousz" mode and setting as consequence the waypoints and the altitude
            rend_states = read_json('rend_states.json')
            if rend_states["rend_states"] != [0, 0, 0, 0]:
                waypoints = waypoints_rend
                self.target_altitude = 35
                # If the "rend_states" is different from the null vector it means that at least one drone is in rend mode and is far from its rendevousz waypoint.
                # Then it is checked at each time step if it has reached the waypoint: if it has it, rendevousz completed and set the state to "0"
                # (if this wass not done before); otherwise the drone will keep travel to the waypoint.
                if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(rend_waypoint, self.current_pose[0:2])]):
                    if str(self.my_def).strip().upper() == "S0":
                        if rend_states["rend_states"][0] != 0:
                            rend_states["rend_states"][0] = 0
                            write_json(rend_states, 'rend_states.json', 0)
                    elif str(self.my_def).strip().upper() == "S1":
                        if rend_states["rend_states"][1] != 0:
                            rend_states["rend_states"][1] = 0
                        write_json(rend_states, 'rend_states.json', 0)
                    elif str(self.my_def).strip().upper() == "S2":
                        if rend_states["rend_states"][2] != 0:
                            rend_states["rend_states"][2] = 0
                            write_json(rend_states, 'rend_states.json', 0)
                    else:
                        if rend_states["rend_states"][3] != 0:
                            rend_states["rend_states"][3] = 0
                            write_json(rend_states, 'rend_states.json', 0)
            else:
                waypoints = waypoints_law
                self.target_altitude = alt_meters               
                    

            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            # Publish telemetry data once in a while
            # ---------- MQTT ----------
            if count % 500 == 0:
                timestamp = time.strftime('%Y-%M-%DT%H:%M:%S', time.localtime())
                publish_telemetry_data(str(self.my_def).strip().upper(), self.current_pose[3:], str(timestamp))
            
            if altitude > self.target_altitude - 1:
                # as soon as it reach the target altitude, compute the disturbances to go to the given waypoints.
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(
                        waypoints)
                    t1 = self.getTime()

            # Find people (represented by the red blocks) in the camera image
            self.detect_heat(identified_ppl, count)

            
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

#---------- MQTT communications section ----------

user_id = "{0}".format(MqttConfigurationParameters.MQTT_USERNAME)
message_limit = 10000

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
mqtt_client.on_connect = on_connect
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME,
    MqttConfigurationParameters.MQTT_PASSWORD)

mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)
mqtt_client.loop_start()

# Publishing drone id only once at the beginning of the simulation
publish_device_info(str(robot.my_def).strip().upper())

#------------------------------------------


#---------- Webots world section ----------

# real world alt. = 800, 600, 400
# altitudes = [40, 30, 20]

            # 650, 450, 350
altitudes = [32.5, 22.5, 17.5]
waypoints_dict = json.load(open("waypoints.json"))
identified_ppl = []
max_alt = altitudes[0]
for s in waypoints_dict:
    if (s.lower() == "high"):
        print(str(robot.my_def).strip().upper() + " flying in high setup")
        alt_meters = altitudes[0]
    elif (s.lower() == "medium"):
        print(str(robot.my_def).strip().upper() + " flying in medium setup")
        alt_meters = altitudes[1]
    elif (s.lower() == "low"):
        print(str(robot.my_def).strip().upper() + " flying in low setup")
        alt_meters = altitudes[2]
    else:
        pass
    robot.run()
    # Stopping the client
    mqtt_client.loop_stop()


