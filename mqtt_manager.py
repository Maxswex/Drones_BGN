import paho.mqtt.client as mqtt
from mqtt_parameters import MqttConfigurationParameters
import json


def on_connect(client, userdata, flags, rc):
    # Subscribe to telemetry topics, both for scanning and for backbone functionalities
    device_telemetry_topic = "{0}/+/+/{1}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.DRONE_TELEMETRY_TOPIC)
    mqtt_client.subscribe(device_telemetry_topic)

    print("Subscribed to: " + device_telemetry_topic)

    # Subscribe to wp_reached topic only for scanning drones: rescue_op must concern only about helping and rescuing civilians
    device_wp_reached_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.SCANNING_TOPIC,
        MqttConfigurationParameters.DRONE_WAYPOINT_REACHED_TOPIC)
    mqtt_client.subscribe(device_wp_reached_topic)

    print("Subscribed to: " + device_wp_reached_topic)


def start(message):
    target_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.BACKBONE_TOPIC,
        message["drone_id"],
        MqttConfigurationParameters.DRONE_START_TOPIC)
    # Send drone_id and waypoint
    json_msg = json.dumps(message)
    mqtt_client.publish(target_topic, json_msg, 2, True)
    print(f"Publishing: starting backbone drones: Topic: {target_topic} \nPayload: {json_msg}\n")


def on_message(client, userdata, message):
    message_payload = str(message.payload.decode("utf-8"))
    print(f"Received IoT Message: Topic: {message.topic}) #Payload: {message_payload}")


vehicle_id = "Consumer: {0}".format(MqttConfigurationParameters.MQTT_USERNAME)
message_limit = 1000
bb_waypoints = [[-45, -40, 32.5], [-55, 40, 32.5]]
bb_start = False

mqtt_client = mqtt.Client(vehicle_id)
mqtt_client.on_message = on_message
mqtt_client.on_connect = on_connect
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME, MqttConfigurationParameters.MQTT_PASSWORD)

mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)
#  Calling the start method only once
if bb_start == False:
    # If the number of BB drones increase, increase the range
    for i in range(2):
        message = {"drone_id": ("B" + str(i)), "target_pos": bb_waypoints[i]}
        start(message)
    bb_start = True
mqtt_client.loop_forever()
