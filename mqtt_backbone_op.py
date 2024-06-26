import paho.mqtt.client as mqtt
from mqtt_parameters import MqttConfigurationParameters
import json

class IoTMessage:
    def __init__(self, topic, payload, retain):
        self.topic = topic
        self.payload = payload
        self.retain = retain

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, indent=4)

def on_connect(client, userdata, flags, rc):
    device_info_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.BACKBONE_TOPIC,
        MqttConfigurationParameters.DRONE_INFO_TOPIC)
    mqtt_client2.subscribe(device_info_topic)

    print("Subscribed to: " + device_info_topic)

    device_telemetry_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.BACKBONE_TOPIC,
        MqttConfigurationParameters.DRONE_TELEMETRY_TOPIC)
    mqtt_client2.subscribe(device_telemetry_topic)

    print("Subscribed to: " + device_telemetry_topic)

    device_wp_reached_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.BACKBONE_TOPIC,
        MqttConfigurationParameters.DRONE_WAYPOINT_REACHED_TOPIC)
    mqtt_client2.subscribe(device_wp_reached_topic)

    print("Subscribed to: " + device_wp_reached_topic)

    device_status_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.BACKBONE_TOPIC,
        MqttConfigurationParameters.DRONE_STATUS_TOPIC)
    mqtt_client2.subscribe(device_telemetry_topic)

    print("Subscribed to: " + device_status_topic)

def on_message(client, userdata, message):
    message_payload = str(message.payload.decode("utf-8"))
    try:
        message_payload = json.loads(message_payload)
    except json.JSONDecodeError:
        message_payload = message_payload
    iot_message = IoTMessage(message.topic, message_payload, message.retain)
    print(f"Received IoT Message: {iot_message.to_json()}")


vehicle_id = "Consumer: {0}".format(MqttConfigurationParameters.MQTT_USERNAME)
message_limit = 1000

mqtt_client2 = mqtt.Client(vehicle_id + "_2")
mqtt_client2.on_message = on_message
mqtt_client2.on_connect = on_connect
mqtt_client2.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME, MqttConfigurationParameters.MQTT_PASSWORD)

mqtt_client2.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)
mqtt_client2.loop_forever()