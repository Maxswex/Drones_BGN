import paho.mqtt.client as mqtt
from mqtt_parameters import MqttConfigurationParameters
import json


class IoTMessage:
    def __init__(self, topic, payload):
        self.topic = topic
        self.payload = payload

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, indent=4)

def remove_time_from_payload(payload):
    try:
        payload_dict = json.loads(payload)      #Payload conversion in dict
        if 'time' in payload_dict:
            del payload_dict['time']            #Delete time field
        return json.dumps(payload_dict, indent=4)
    except json.JSONDecodeError:        #errors management
        return payload


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


last_messages = {}
def on_message(client, userdata, message):
    message_payload = str(message.payload.decode("utf-8"))
    clean_payload = remove_time_from_payload(message_payload)       #Remove time field from the payload

    #Verificy the uniqueness of the message
    if message.topic not in last_messages or last_messages[message.topic] != clean_payload:
        iot_message = IoTMessage(message.topic, message_payload)
        print(f"Received IoT Message: {iot_message.to_json()}")
        last_messages[message.topic] = clean_payload

vehicle_id = "Consumer: {0}".format(MqttConfigurationParameters.MQTT_USERNAME)
message_limit = 1000

mqtt_client = mqtt.Client(vehicle_id)
mqtt_client.on_message = on_message
mqtt_client.on_connect = on_connect
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME, MqttConfigurationParameters.MQTT_PASSWORD)

mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)
mqtt_client.loop_forever()