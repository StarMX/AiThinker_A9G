#ifndef __MQTT_TASK_H_
#define __MQTT_TASK_H_

#define MQTT_TASK_STACK_SIZE       (2048 * 4)
#define MQTT_TASK_PRIORITY         1
#define MQTT_TASK_NAME             "MQTT Task"

#define MQTT_INTERVAL              1000
#define MQTT_WATCHDOG_INTERVAL     2 * 60


#define MQTT_STATE_TOPIC_FORMAT    "tracker/%s/state"
#define MQTT_DATA_TOPIC_FORMAT     "tracker/%s/data"

#define MQTT_PAYLOAD_STATE_ONLINE  "online"
#define MQTT_PAYLOAD_STATE_OFFLINE "offline"
#define MQTT_PAYLOAD_ON            "ON"
#define MQTT_PAYLOAD_OFF           "OFF"


typedef enum {
    MQTT_STATUS_DISCONNECTED = 0,
    MQTT_STATUS_CONNECTING,
    MQTT_STATUS_CONNECTED,
    MQTT_STATUS_PUBLISHING_ONLINE,
    MQTT_STATUS_ONLINE,
    MQTT_STATUS_LOCATION_PUBLISHED,
    MQTT_STATUS_OFFLINE,
    MQTT_STATUS_MAX
} MQTT_Status_t;


extern HANDLE semMqttStart;


void MqttPublishStateOffline(void);
void MqttPublishIgnPayload(char *mqttPayload);
void MqttPublishGsm(int gsm_level);
MQTT_Status_t getMqttState(void);
void MqttTaskInit(void);


#endif /* __MQTT_TASK_H_ */
