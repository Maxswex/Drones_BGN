import paho.mqtt.client as mqtt
from mqtt_parameters import MqttConfigurationParameters
import json

def on_connect(client, userdata, flags, rc):
    device_info_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.SCANNING_TOPIC,
        MqttConfigurationParameters.DRONE_INFO_TOPIC)
    mqtt_client.subscribe(device_info_topic)

    print("Subscribed to: " + device_info_topic)

    device_sensible_coordinates_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.SCANNING_TOPIC,
        MqttConfigurationParameters.DRONE_SENSIBLE_COORDINATES_TOPIC)
    mqtt_client.subscribe(device_sensible_coordinates_topic)

    print(f"Subscribed to: {device_sensible_coordinates_topic}")

def on_message(client, userdata, message):
    message_payload = str(message.payload.decode("utf-8"))
    print(f"Received IoT Message: Topic: {message.topic}) #Payload: {message_payload}")



vehicle_id = "Consumer: {0}".format(MqttConfigurationParameters.MQTT_USERNAME)
message_limit = 1000

mqtt_client = mqtt.Client(vehicle_id)
mqtt_client.on_message = on_message
mqtt_client.on_connect = on_connect
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME, MqttConfigurationParameters.MQTT_PASSWORD)

mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)
mqtt_client.loop_forever()