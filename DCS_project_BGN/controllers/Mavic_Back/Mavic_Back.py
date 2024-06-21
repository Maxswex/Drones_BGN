from mimetypes import init
import json
from controller import Supervisor
import sys
from pathlib import Path
project_root = Path(__file__).resolve().parents[3]
mqtt_parameters_path = project_root / 'mqtt_parameters.py'
sys.path.append(str(project_root))
from mqtt_parameters import MqttConfigurationParameters
import paho.mqtt.client as mqtt
import time
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")


# ---------- MQTT communications methods ----------


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    # Subscribe to start topics
    device_start_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.BACKBONE_TOPIC,
        robot.my_def,
        MqttConfigurationParameters.DRONE_START_TOPIC)
    mqtt_client.subscribe(device_start_topic)

    print("Subscribed to: " + device_start_topic)

def on_message(client, userdata, message):
    try:
        # Decodifica del payload JSON
        message_payload = json.loads(message.payload.decode('utf-8'))
        print(f"Received IoT Message: Topic: {message.topic}) #Payload: {message_payload}")
        if "target_pos" in message_payload:
            robot.target_position = message_payload["target_pos"]
    except json.JSONDecodeError as e:
        print(f"Errore di decodifica JSON: {e}")


def publish_device_info(drone_id):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.BACKBONE_TOPIC,
        drone_id,
        MqttConfigurationParameters.DRONE_INFO_TOPIC)
    # Get only device info
    mqtt_client.publish(target_topic, drone_id, 0, True)
    print(f"Vehicle Info Published: Topic: {target_topic} Payload: {drone_id}")


def publish_telemetry_data(drone_id, drone_pos, timestamp):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.BACKBONE_TOPIC,
        drone_id,
        MqttConfigurationParameters.DRONE_TELEMETRY_TOPIC)
    # Get only telemetry and timestamp
    device_payload_string = f"drone position: {drone_pos}, time: {timestamp}"
    mqtt_client.publish(target_topic, device_payload_string, 0, False)
    print(f"Telemetry Data Published: Topic: {target_topic} \nPayload: {device_payload_string}\n")

def wp_reached(drone_id, drone_pos, timestamp):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.BACKBONE_TOPIC,
        drone_id,
        MqttConfigurationParameters.DRONE_WAYPOINT_REACHED_TOPIC)
    # Get only telemetry and timestamp
    device_payload_string = f"drone position: {drone_pos}, time: {timestamp}"
    mqtt_client.publish(target_topic, device_payload_string, 0, False)
    print(f"Waypoint Reached: Topic: {target_topic} \nPayload: {device_payload_string}\n")

def status(drone_id, drone_pos, timestamp):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.BACKBONE_TOPIC,
        drone_id,
        MqttConfigurationParameters.DRONE_STATUS_TOPIC)
    # Get only telemetry and status
    device_payload_string = f"drone position: {drone_pos}, time: {timestamp}"
    mqtt_client.publish(target_topic, device_payload_string, 0, False)
    print(f"Backbone Drone {drone_id} Status Data Published: Topic: {target_topic} \nPayload: {device_payload_string}\n")



def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class Mavic(Supervisor):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0          # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    target_precision = 0.5

    def __init__(self):
        Supervisor.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Get and enable devices.
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
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
        self.camera_pitch_motor.setPosition(0.7)
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.robot_node_myself = self.getSelf()
        self.my_def = self.robot_node_myself.getDef()

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        '''if str(self.my_def).strip().upper() == "B0":
            self.target_position = [-45, -40, 32.5]
        else:
            self.target_position = [-55, 40, 32.5]'''
        self.target_position = [0, 0, 0]
        self.target_altitude = 32.5

    def set_position(self, pos):
        self.current_pose = pos

    def move_to_target(self, verbose_movement=False, verbose_target=False):
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):
            # if the message about the reaching of the target has already been published, do not publish it again,
            # since it is fixed for the backbone drones
            global msg_wp_reached
            if msg_wp_reached:
                pass
            else:
                # ---------- MQTT ----------
                timestamp = time.strftime('%Y-%M-%D T%H:%M:%S', time.localtime())
                wp_reached(str(self.my_def).strip().upper(), self.current_pose[3:], str(timestamp))
                msg_wp_reached = True
            if verbose_target:
                print(f"Target reached! Backbone drone: {str(self.my_def.upper())} at position: ", self.current_pose[0:2])




        self.target_position[2] = np.arctan2(self.target_position[1] - self.current_pose[1],
                                            self.target_position[0] - self.current_pose[0])

        angle_left = self.target_position[2] - self.current_pose[3]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if angle_left > np.pi:
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according to the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        pitch_disturbance = clamp(np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) +
                                   ((self.target_position[1] - self.current_pose[1]) ** 2))
            print("Remaining angle: {:.4f}, remaining distance: {:.4f}".format(angle_left, distance_left))

        return yaw_disturbance, pitch_disturbance

    def run(self):
        count = 0
        t1 = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        while self.step(self.time_step) != -1:
            count += 1
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, yaw, pitch, roll])

            # Publish telemetry data once in a while
            # ---------- MQTT ----------
            if count % 500 == 0:
                timestamp = time.strftime('%Y-%M-%D T%H:%M:%S', time.localtime())
                publish_telemetry_data(str(self.my_def).strip().upper(), self.current_pose[3:], str(timestamp))

            if altitude > self.target_altitude - 1:
                # as soon as it reaches the target altitude, compute disturbances to go to the given waypoints.
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target()
                    t1 = self.getTime()

            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            # Motor inputs
            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            # Set motor velocities
            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)

if __name__ == "__main__":
    robot = Mavic()
    msg_wp_reached = False

    # ---------- MQTT communications section ----------

    user_id = "{0}".format(MqttConfigurationParameters.MQTT_USERNAME)
    message_limit = 10000

    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME,
                                MqttConfigurationParameters.MQTT_PASSWORD)

    mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)
    mqtt_client.loop_start()


    # Publishing drone id only once at the beginning of the simulation
    publish_device_info(str(robot.my_def).strip().upper())

    # ------------------------------------------

    robot.run()

