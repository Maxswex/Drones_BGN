# For this example we rely on the Paho MQTT Library for Python
# You can install it through the following command: pip install paho-mqtt
import paho.mqtt.client as mqtt
import time

import matplotlib
from matplotlib import use
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import re

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []


def updategraph():
    anim = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
    plt.show()

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    xs = xs[-1000:]
    ys = ys[-1000:]

    ax.clear()
    ax.plot(xs, ys)

    plt.xlim([0, 1200])
    plt.ylim([0, 700])

    plt.ylim(max(plt.ylim()), min(plt.ylim()))

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Heavy AGV position')


# Set up plot to call animate() function periodically


# Full MQTT client creation with all the parameters. The only one mandatory in the ClientId that should be unique
# mqtt_client = Client(client_id="", clean_session=True, userdata=None, protocol=MQTTv311, transport=”tcp”)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    mqtt_client.subscribe(default_topic)
    print("Subscribed to: " + default_topic)
    #t = Thread(target=updategra)
    #t.run()



# Define a callback method to receive asynchronous messages
def on_message(client, userdata, message):
    print("\n##########################################################")
    print("message received: ", str(message.payload.decode("utf-8")))
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)
    print("##########################################################")


    # getting numbers from string 
    temp = re.findall(r'\d+', str(message.payload.decode("utf-8")))
    res = list(map(int, temp))

    x = res[0]
    y = res[2]

    xs.append(x)
    ys.append(y)



# Configuration variables
client_id = "consumer"
broker_ip = "mqtt.eclipseprojects.io"
broker_port = 1883
default_topic = "HLAGV/position"

# Create a new MQTT Client
mqtt_client = mqtt.Client(client_id)

# Attack Paho OnMessage Callback Method
mqtt_client.on_message = on_message         #ogni volta che riceviamo un messaggio avviene la funzione on message
mqtt_client.on_connect = on_connect         #iscrizione al topic



# Connect to the target MQTT Broker
mqtt_client.connect(broker_ip, broker_port)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.

mqtt_client.loop_start()

updategraph()







