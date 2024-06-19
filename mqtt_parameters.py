class MqttConfigurationParameters(object):
    BROKER_ADDRESS = "localhost"
    BROKER_PORT = 1883
    MQTT_USERNAME = "MAX"
    MQTT_PASSWORD = "passw"
    MQTT_BASIC_TOPIC = "/BGN_core/objective" # basic topic
    SCANNING_TOPIC = "scanning"   # scanning functionality topic
    BACKBONE_TOPIC = "backbone"   # backbone functionality topic
    MANAGER_TOPIC = "manager"   # manager application topic
    DRONE_TELEMETRY_TOPIC = "telemetry"   # topic for receiving telemtry messages
    DRONE_INFO_TOPIC = "info"     # topic for receiving vehicles info
    DRONE_SENSIBLE_COORDINATES_TOPIC = "sensible_coord"   # topic for receiving coordinates of wounded people (for scanning drone)
    DRONE_WAYPOINT_REACHED_TOPIC = "waypoint_reached"   # topic for receiving message when the drone reached a waypoint (for scanning drone)
    DRONE_START_TOPIC = "start"   # starting the simulation topic
    DRONE_RENDEV_TOPIC = "rendevousz"   # rendevousz command topic(for scanning drone)
    DRONE_STATUS_TOPIC = "status"   # rendevousz command topic (for BB drone)
    DRONE_NEW_POS_TOPIC = "new_position"   # new position command topic (for BB drone)




