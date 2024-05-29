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

def publish_device_info():
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.DRONE_TOPIC,
        drone_descriptor.ID,
        MqttConfigurationParameters.DRONE_INFO_TOPIC)
    device_payload_string = drone_descriptor.to_json()
    mqtt_client.publish(target_topic, device_payload_string, 0, True)
    print(f"Vehicle Info Published: Topic: {target_topic} Payload: {device_payload_string}")

def publish_telemetry_data():
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.DRONE_TOPIC,
        drone_descriptor.ID,
        MqttConfigurationParameters.DRONE_TELEMETRY_TOPIC)
    device_payload_string = drone_telemetry_data.to_json()
    mqtt_client.publish(target_topic, device_payload_string, 0, False)
    print(f"Telemetry Data Published: Topic: {target_topic} \nPayload: {device_payload_string}\n")


def publish_sensible_coordinates():
    sensible_coord = json_manipulations.read_json(json_path)
    target_topic = "{0}/{1}/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        int(sensible_coord['id_drone']),
        MqttConfigurationParameters.DRONE_SENSIBLE_COORDINATES_TOPIC)
    mqtt_client.publish(target_topic, str(sensible_coord), 0, False)
    print(f"FOUND A SENSIBLE COORDINATES!! Published to the following topic: {target_topic} \nPayload: {sensible_coord}")



vehicle_id = "{0}".format(MqttConfigurationParameters.MQTT_USERNAME)
message_limit = 1000

mqtt_client = mqtt.Client(vehicle_id)
mqtt_client.on_connect = on_connect
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME,
    MqttConfigurationParameters.MQTT_PASSWORD)

mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)
mqtt_client.loop_start()

#drone_descriptor = DroneDescriptor("S_1", "scanning")
#drone_telemetry_data = DroneTelemetryData()
#sensible_coordinates = SensibleElementsGeoLocation()
# publish_device_info()
infos = json_manipulations.read_json(json_path)
drone_info = {key:infos[key] for key in ['id_drone', 'drone_position', 'time']}
print(drone_info)

for message_id in range(message_limit):
    #drone_telemetry_data.update_measurement()
    #publish_telemetry_data()
    #time.sleep(10)
    #sensible_coordinates.update_measurement()
    publish_sensible_coordinates()
    time.sleep(5)

mqtt_client.loop_stop()