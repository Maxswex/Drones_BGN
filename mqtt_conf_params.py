class MqttConfigurationParameters(object):
    BROKER_ADDRESS = "localhost"
    BROKER_PORT = 1883
    MQTT_USERNAME = "MAX"
    MQTT_PASSWORD = "<1332>"
    MQTT_BASIC_TOPIC = "/iot/user/{0}".format(MQTT_USERNAME)
    VEHICLE_TOPIC = "drone"       #topic for receiving message related to vehicles
    VEHICLE_TELEMETRY_TOPIC = "telemetry"   #topic for receiving message related to telemetry
    VEHICLE_INFO_TOPIC = "info"     #topic for receiving message related to info