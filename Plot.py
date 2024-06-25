
# For this example we rely on the Paho MQTT Library for Python
# You can install it through the following command: pip install paho-mqtt
import paho.mqtt.client as mqtt
from mqtt_parameters import MqttConfigurationParameters
import time

import matplotlib
from matplotlib import use
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import re

# Create figure for plotting
fig, axs = plt.subplots(2, 2, figsize = (10,8))
fig.suptitle('SCANNING DRONES POSITION', fontsize=25, fontweight = 'bold', color='red')

# Define 4 empty list for drones coordinates
drone_data = {
    "S0": {"xs": [], "ys": [], "color": "black"},
    "S1": {"xs": [], "ys": [], "color": "green"},
    "S2": {"xs": [], "ys": [], "color": "blue"},
    "S3": {"xs": [], "ys": [], "color": "purple"}
}


def updategraph():
    anim = animation.FuncAnimation(fig, animate, fargs=(drone_data,), interval=1000, cache_frame_data=False)
    plt.show()


# This function is called periodically from FuncAnimation
def animate(i, drone_data):
    # Cycle for each drones and its datas
    for idx, (drone_id, data) in enumerate(drone_data.items()):
        # Select the correct subplot
        ax = axs[idx // 2, idx % 2]
        ax.clear()
        xs, ys = data["xs"], data["ys"]

        # Keep the last 1000 points
        xs = xs[-1000:]
        ys = ys[-1000:]

        ax.plot(xs, ys, label=f'Drone {drone_id}', color=data["color"], marker='o', linestyle='-', linewidth=2, markersize=1)

        # Set limits to view values between -100 and 100
        ax.set_xlim([-100, 100])
        ax.set_ylim([-100, 100])
        ax.set_title(f'Drone {drone_id}', color= data["color"], fontweight = 'bold')
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.grid(True)  # Add grid


    plt.subplots_adjust(top=0.85, bottom=0.1, left=0.1, right=0.9, hspace=0.6, wspace=0.6)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)


# Full MQTT client creation with all the parameters. The only one mandatory in the ClientId that should be unique
# mqtt_client = Client(client_id="", clean_session=True, userdata=None, protocol=MQTTv311, transport=”tcp”)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    mqtt_client.subscribe(default_topic)
    print("Subscribed to: " + default_topic)


# Define a callback method to receive asynchronous messages
def on_message(client, userdata, message):
    print("\n##########################################################")
    print("message received: ", str(message.payload.decode("utf-8")))
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)
    print("##########################################################")

    # Determine which drone the message is from
    topic_parts = message.topic.split('/')
    drone_id = topic_parts[-2] if len(topic_parts) > 2 else "S0"

    # getting numbers from string, using re library to get float number
    temp = re.findall(r'-?\d+\.\d+', str(message.payload.decode("utf-8")))
    res = list(map(float, temp))

    if len(res) >= 3:
        x = res[0]
        y = res[1]
        if drone_id in drone_data:
            drone_data[drone_id]["xs"].append(x)
            drone_data[drone_id]["ys"].append(y)
            print(f"Extracted coordinates for {drone_id}: x={x}, y={y}")
    else:
        print("Error: Could not extract coordinates")


# Configuration variables
client_id = "consumer"
broker_ip = "localhost"
broker_port = 1883
default_topic = "{0}/{1}/+/{2}".format(
    MqttConfigurationParameters.MQTT_BASIC_TOPIC,
    MqttConfigurationParameters.SCANNING_TOPIC,
    MqttConfigurationParameters.DRONE_TELEMETRY_TOPIC)

# Create a new MQTT Client
mqtt_client = mqtt.Client(client_id)

# Attach Paho OnMessage Callback Method
mqtt_client.on_message = on_message  # ogni volta che riceviamo un messaggio avviene la funzione on message
mqtt_client.on_connect = on_connect  # iscrizione al topic

# Connect to the target MQTT Broker
mqtt_client.connect(broker_ip, broker_port)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.

mqtt_client.loop_start()

updategraph()
