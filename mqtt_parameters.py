class MqttConfigurationParameters(object):
    BROKER_ADDRESS = "localhost"
    BROKER_PORT = 1883
    MQTT_USERNAME = "MAX"
    MQTT_PASSWORD = "passw"
    MQTT_BASIC_TOPIC = "/iot/user/{0}".format(MQTT_USERNAME)
    DRONE_TOPIC = "drone"       #topic for receiving message related to vehicles
    DRONE_TELEMETRY_TOPIC = "telemetry"   #topic for receiving message related to telemetry
    DRONE_INFO_TOPIC = "info"     #topic for receiving message related to info
    DRONE_SENSIBLE_COORDINATES_TOPIC = "sensible_coordinates"