import paho.mqtt.client as mqtt
import time

import drone_descriptor
from drone_descriptor import DroneDescriptor
from drone_telemetry_data import DroneTelemetryData
from mqtt_conf_params import MqttConfigurationParameters

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))


def publish_telemetry_data():
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.VEHICLE_TOPIC,
        drone_descriptor.ID,
        MqttConfigurationParameters.VEHICLE_TELEMETRY_TOPIC)
    device_payload_string = drone_telemetry_data.to_json()
    mqtt_client.publish(target_topic, device_payload_string, 0, False)
    print(f"Telemetry Data Published: Topic: {target_topic} Payload: {device_payload_string}")


def publish_device_info():
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.VEHICLE_TOPIC,
        drone_descriptor.ID,
        MqttConfigurationParameters.VEHICLE_INFO_TOPIC)
    device_payload_string = drone_descriptor.to_json()
    mqtt_client.publish(target_topic, device_payload_string, 0, True)
    print(f"Vehicle Info Published: Topic: {target_topic} Payload: {device_payload_string}")


vehicle_id = "{0}".format(MqttConfigurationParameters.MQTT_USERNAME)
message_limit = 1000

mqtt_client = mqtt.Client(vehicle_id)
mqtt_client.on_connect = on_connect
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME,
    MqttConfigurationParameters.MQTT_PASSWORD)

mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)
mqtt_client.loop_start()

drone_descriptor = DroneDescriptor("S_1", "scanning")
drone_telemetry_data = DroneTelemetryData()
publish_device_info()

for message_id in range(message_limit):
    drone_telemetry_data.update_measurement()
    publish_telemetry_data()
    time.sleep(15)

mqtt_client.loop_stop()