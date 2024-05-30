import paho.mqtt.client as mqtt
import time
import os

import drone_descriptor
from drone_descriptor import DroneDescriptor
from drone_telemetry_data import DroneTelemetryData
from sensible_coordinates import SensibleElementsGeoLocation
from mqtt_parameters import MqttConfigurationParameters
from utilities import json_manipulations

# Path of this directory
dir_path = os.path.dirname(__file__)

json_path = os.path.join(dir_path, 'DCS_project_BGN', 'controllers',
                                  'mavic2pro_LawRend_st', 'sensible_coord.json')

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

def publish_device_info(data):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.DRONE_TOPIC,
        data['id_drone'],
        MqttConfigurationParameters.DRONE_INFO_TOPIC)
    # Get only device info from the dictionary (e.g. the JSON)
    device_payload_string = data['id_drone']
    mqtt_client.publish(target_topic, device_payload_string, 0, True)
    print(f"Vehicle Info Published: Topic: {target_topic} Payload: {device_payload_string}")

def publish_telemetry_data(data):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.DRONE_TOPIC,
        data['id_drone'],
        MqttConfigurationParameters.DRONE_TELEMETRY_TOPIC)
    # Get only telemetry and timestamp from the dictionary (e.g. the JSON)
    device_payload_string = {key:data[key] for key in ['drone_position', 'time']}
    mqtt_client.publish(target_topic, str(device_payload_string), 0, False)
    print(f"Telemetry Data Published: Topic: {target_topic} \nPayload: {device_payload_string}\n")


def publish_sensible_coordinates(data):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.DRONE_TOPIC,
        data['id_drone'],
        MqttConfigurationParameters.DRONE_SENSIBLE_COORDINATES_TOPIC)
    # Get id detection, heat position and timestamp from the dictionary (e.g. the JSON)
    device_payload_string = {key:data[key] for key in ['id_detection', 'heat_pos', 'time']}
    mqtt_client.publish(target_topic, str(device_payload_string), 0, False)
    print(f"FOUND A SENSIBLE COORDINATES!! Published to the following topic: {target_topic} \nPayload: {device_payload_string}")


vehicle_id = "{0}".format(MqttConfigurationParameters.MQTT_USERNAME)
message_limit = 1000

mqtt_client = mqtt.Client(vehicle_id)
mqtt_client.on_connect = on_connect
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME,
    MqttConfigurationParameters.MQTT_PASSWORD)

mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)
mqtt_client.loop_start()

# Reading data from the JSON file
data = json_manipulations.read_json(json_path)
# Initial log of the device info
# publish_device_info(data)

current_drone_id = None
for message_id in range(message_limit):
    # Reading data from the JSON file
    data = json_manipulations.read_json(json_path)

    # Check if the drone_id has changed
    if data['id_drone'] != current_drone_id:
        current_drone_id = data['id_drone']
        publish_device_info(data)

    publish_telemetry_data(data)
    time.sleep(10)
    publish_sensible_coordinates(data)
    time.sleep(5)

mqtt_client.loop_stop()