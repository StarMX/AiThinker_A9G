#include <string.h>
#include <stdio.h>

#include <api_os.h>
#include <api_info.h>
#include <api_gps.h>
#include <api_debug.h>
#include <api_mqtt.h>
#include <api_socket.h>

#include <api_hal_watchdog.h>
#include <api_hal_gpio.h>
#include <api_hal_uart.h>

#include <api_sim.h>

#include <gps.h>
#include <gps_parse.h>

#include "pm_task.h"
#include "mqtt_task.h"
#include "adc_task.h"
#include "secret.h"
#include "api_network.h"

static HANDLE mqttTaskHandle = NULL;
HANDLE semMqttStart = NULL;

uint8_t BROKER_IP[16];

MQTT_Connect_Info_t ci;
MQTT_Client_t *mqttClient = NULL;

const char *ca_crt = CA_CRT;
const char *client_crt = CLIENT_CRT;
const char *client_key = CLIENT_KEY;

char imei[16] = "";

char mqttDataTopic[64] = "";
char mqttStateTopic[64] = "";
char mqttBuffer[1024] = "";

MQTT_Status_t mqttStatus = MQTT_STATUS_DISCONNECTED;

MQTT_Status_t getMqttState(void)
{
    return mqttStatus;
}

void UratPrint(char *mqttDataBuffer)
{
    Trace(7, "uart1 print data,length:%d:data:%s", strlen(mqttDataBuffer), mqttDataBuffer);
    UART_Write(UART1, mqttDataBuffer, strlen(mqttDataBuffer));
    UART_Write(UART1, "\r\n", 2);
    memset(mqttDataBuffer, 0, sizeof(mqttDataBuffer));
}

void OnMqttConnect(MQTT_Client_t *client, void *arg, MQTT_Connection_Status_t status)
{
    Trace(1, "MQTT connection status: %d", status);

    if (status == MQTT_CONNECTION_ACCEPTED)
    {
        Trace(1, "MQTT succeed connect to broker");
        mqttStatus = MQTT_STATUS_CONNECTED;
    }
    else
    {
        mqttStatus = MQTT_STATUS_DISCONNECTED;
    }

    WatchDog_KeepAlive();
}

void MqttConnect(void)
{
    memset(BROKER_IP, 0, sizeof(BROKER_IP));
    if (DNS_GetHostByName2(BROKER_HOSTNAME, BROKER_IP) != 0)
    {
        Trace(1, "get ip error");
        return;
    }
    else
    {
        Trace(1, "DNS,domain:%s,ip:%s,strlen(ip):%d", BROKER_HOSTNAME, BROKER_IP, strlen(BROKER_IP));
    }

    Trace(1, "MqttConnect");

    MQTT_Error_t err = MQTT_Connect(mqttClient, BROKER_IP, BROKER_PORT, OnMqttConnect, NULL, &ci);

    if (err != MQTT_ERROR_NONE)
    {
        Trace(1, "MQTT connect fail, error code: %d", err);
    }

    mqttStatus = MQTT_STATUS_CONNECTING;
}

void OnPublishStateOnline(void *arg, MQTT_Error_t err)
{
    if (err == MQTT_ERROR_NONE)
    {
        Trace(1, "MQTT publish state online success");
        mqttStatus = MQTT_STATUS_ONLINE;
    }
    else
    {
        Trace(1, "MQTT publish state online error, error code: %d", err);
        mqttStatus = MQTT_STATUS_CONNECTED;
    }

    WatchDog_KeepAlive();
}

void MqttPublishStateOnline(void)
{
    Trace(1, "MqttPublishStateOnline");

    MQTT_Error_t err = MQTT_Publish(mqttClient, mqttStateTopic, MQTT_PAYLOAD_STATE_ONLINE, strlen(MQTT_PAYLOAD_STATE_ONLINE), 1, 2, 1, OnPublishStateOnline, NULL);

    if (err != MQTT_ERROR_NONE)
        Trace(1, "MQTT publish state online error, error code: %d", err);

    mqttStatus = MQTT_STATUS_PUBLISHING_ONLINE;
}

void OnPublishStateOffline(void *arg, MQTT_Error_t err)
{
    if (err == MQTT_ERROR_NONE)
    {
        Trace(1, "MQTT publish state offline success");
        mqttStatus = MQTT_STATUS_OFFLINE;
    }
    else
    {
        Trace(1, "MQTT publish state offline error, error code: %d", err);
    }

    WatchDog_KeepAlive();
}

void MqttPublishStateOffline(void)
{
    Trace(1, "MqttPublishStateOffline");

    MQTT_Error_t err = MQTT_Publish(mqttClient, mqttStateTopic, MQTT_PAYLOAD_STATE_OFFLINE, strlen(MQTT_PAYLOAD_STATE_OFFLINE), 1, 2, 1, OnPublishStateOffline, NULL);

    if (err != MQTT_ERROR_NONE)
        Trace(1, "MQTT publish state offline error, error code: %d", err);
}

void OnPublishIgn(void *arg, MQTT_Error_t err)
{
    if (err == MQTT_ERROR_NONE)
    {
        Trace(1, "MQTT publish IGN success");
    }
    else
    {
        Trace(1, "MQTT publish IGN error, error code: %d", err);
    }

    WatchDog_KeepAlive();
}

void MqttPublishIgnPayload(char *mqttPayload)
{
    Trace(1, "MqttPublishIgnPayload");

    if (mqttStatus < MQTT_STATUS_CONNECTED)
        return;
    snprintf(mqttBuffer, sizeof(mqttBuffer), "{\"action\":\"ing\",\"data\":\"%s\"}", mqttPayload);
    MQTT_Error_t err = MQTT_Publish(mqttClient, mqttDataTopic, mqttBuffer, strlen(mqttBuffer), 1, 1, 1, OnPublishIgn, NULL);

    if (err != MQTT_ERROR_NONE)
        Trace(1, "MQTT publish IGN error, error code: %d", err);
    UratPrint(mqttBuffer);
}

void MqttPublishIgn(void)
{
    Trace(1, "MqttPublishIgn");

    if (PmGetIngState() == true)
    {
        MqttPublishIgnPayload(MQTT_PAYLOAD_ON);
    }
    else
    {
        MqttPublishIgnPayload(MQTT_PAYLOAD_OFF);
    }
}

void OnPublishGsm(void *arg, MQTT_Error_t err)
{
    if (err == MQTT_ERROR_NONE)
    {
        Trace(1, "MQTT publish GSM success");
    }
    else
    {
        Trace(1, "MQTT publish GSM error, error code: %d", err);
    }

    WatchDog_KeepAlive();
}

void MqttPublishGsm(int gsm_level)
{
    if (mqttStatus < MQTT_STATUS_CONNECTED)
        return;

    Trace(1, "MqttPublishGsm");
    snprintf(mqttBuffer, sizeof(mqttBuffer), "{\"action\":\"gsmlive\",\"data\":%d}", gsm_level);

    MQTT_Error_t err = MQTT_Publish(mqttClient, mqttDataTopic, mqttBuffer, strlen(mqttBuffer), 1, 2, 1, OnPublishGsm, NULL);

    if (err != MQTT_ERROR_NONE)
        Trace(1, "MQTT publish GSM error, error code: %d", err);
    UratPrint(mqttBuffer);
}

void OnPublishBattery(void *arg, MQTT_Error_t err)
{
    if (err == MQTT_ERROR_NONE)
    {
        Trace(1, "MQTT publish battery success");
    }
    else
    {
        Trace(1, "MQTT publish battery error, error code: %d", err);
    }

    WatchDog_KeepAlive();
}

void MqttPublishBattery(void)
{
    Trace(1, "MqttPublishBattery");

    snprintf(mqttBuffer, sizeof(mqttBuffer), "{\"action\":\"battery\",\"data\":%.02f}", GetBatteryVoltage());

    MQTT_Error_t err = MQTT_Publish(mqttClient, mqttDataTopic, mqttBuffer, strlen(mqttBuffer), 1, 0, 1, OnPublishBattery, NULL);

    if (err != MQTT_ERROR_NONE)
        Trace(1, "MQTT publish battery error, error code: %d", err);
    UratPrint(mqttBuffer);
}

void MqttPublishLiion(void)
{
    Trace(1, "MqttPublishLiion");

    snprintf(mqttBuffer, sizeof(mqttBuffer), "{\"action\":\"liion\",\"data\":%.02f}", GetLiionVoltage());

    MQTT_Error_t err = MQTT_Publish(mqttClient, mqttDataTopic, mqttBuffer, strlen(mqttBuffer), 1, 0, 1, OnPublishBattery, NULL);

    if (err != MQTT_ERROR_NONE)
        Trace(1, "MQTT publish liion error, error code: %d", err);
    UratPrint(mqttBuffer);
}

void OnPublishSpeed(void *arg, MQTT_Error_t err)
{
    if (err == MQTT_ERROR_NONE)
    {
        Trace(1, "MQTT publish speed success");
    }
    else
    {
        Trace(1, "MQTT publish speed error, error code: %d", err);
    }

    WatchDog_KeepAlive();
}

void MqttPublishSpeed(void)
{
    Trace(1, "MqttPublishSpeed");

    GPS_Info_t *gpsInfo = Gps_GetInfo();
    uint8_t isFixed = gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type ? gpsInfo->gsa[0].fix_type : gpsInfo->gsa[1].fix_type;
    snprintf(mqttBuffer, sizeof(mqttBuffer), "{\"action\":\"speed\",\"data\":0.0}");
    if (PmGetIngState() == true)
    {
        if (isFixed == 2 || isFixed == 3)
        { /* We have 2D or 3D fix */
            memset(mqttBuffer, 0, sizeof(mqttBuffer));
            snprintf(mqttBuffer, sizeof(mqttBuffer), "{\"action\":\"speed\",\"data\":%.02f}", minmea_tofloat(&gpsInfo->vtg.speed_kph));
        }
    }
    MQTT_Error_t err = MQTT_Publish(mqttClient, mqttDataTopic, mqttBuffer, strlen(mqttBuffer), 1, 1, 1, OnPublishSpeed, NULL);
    if (err != MQTT_ERROR_NONE)
        Trace(1, "MQTT publish speed error, error code: %d", err);
    UratPrint(mqttBuffer);
}

void OnPublishTracker(void *arg, MQTT_Error_t err)
{
    if (err == MQTT_ERROR_NONE)
    {
        Trace(1, "MQTT publish tracker success");
        if (mqttStatus == MQTT_STATUS_ONLINE)
        {
            mqttStatus = MQTT_STATUS_LOCATION_PUBLISHED;
        }
    }
    else
    {
        Trace(1, "MQTT publish tracker error, error code: %d", err);
    }

    WatchDog_KeepAlive();
}

void MqttPublishTracker(void)
{
    Trace(1, "MqttPublishTracker");

    GPS_Info_t *gpsInfo = Gps_GetInfo();
    uint8_t isFixed = gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type ? gpsInfo->gsa[0].fix_type : gpsInfo->gsa[1].fix_type;

    if (isFixed == 2 || isFixed == 3)
    { /* We have 2D or 3D fix */

        char *isFixedStr;
        if (isFixed == 2)
            isFixedStr = "2D fix";
        else if (isFixed == 3)
        {
            if (gpsInfo->gga.fix_quality == 1)
                isFixedStr = "3D fix";
            else if (gpsInfo->gga.fix_quality == 2)
                isFixedStr = "3D/DGPS fix";
        }
        else
            isFixedStr = "no fix";

        // float latitude = minmea_tocoord(&(gpsInfo->rmc.latitude));
        // float longitude = minmea_tocoord(&(gpsInfo->rmc.longitude));
        //convert unit ddmm.mmmm to degree(°)
        int temp = (int)(gpsInfo->rmc.latitude.value / gpsInfo->rmc.latitude.scale / 100);
        double latitude = temp + (double)(gpsInfo->rmc.latitude.value - temp * gpsInfo->rmc.latitude.scale * 100) / gpsInfo->rmc.latitude.scale / 60.0;
        temp = (int)(gpsInfo->rmc.longitude.value / gpsInfo->rmc.longitude.scale / 100);
        double longitude = temp + (double)(gpsInfo->rmc.longitude.value - temp * gpsInfo->rmc.longitude.scale * 100) / gpsInfo->rmc.longitude.scale / 60.0;

        snprintf(mqttBuffer, sizeof(mqttBuffer), "{\"action\":\"gps\",\"data\":{\"longitude\": %f,\"latitude\": %f,\"altitude\": %.02f,\"gps_accuracy\": %.f,\"battery_level\": %.02f}}",
                 longitude, latitude, minmea_tofloat(&gpsInfo->gga.altitude), minmea_tofloat(&gpsInfo->gga.hdop) * 25, GetLiionLevel());

        MQTT_Error_t err = MQTT_Publish(mqttClient, mqttDataTopic, mqttBuffer, strlen(mqttBuffer), 1, 1, 1, OnPublishTracker, NULL);
        if (err != MQTT_ERROR_NONE)
            Trace(1, "MQTT publish tracker error, error code: %d", err);
        /*snprintf(mqttBuffer,sizeof(mqttBuffer),"GPS fix mode:%d, BDS fix mode:%d, hdop:%f, satellites tracked:%d, gps sates total:%d, is fixed:%s, coordinate:WGS84, Latitude:%f, Longitude:%f, unit:degree, altitude:%f",gpsInfo->gsa[0].fix_type, gpsInfo->gsa[1].fix_type,
                                                                 minmea_tofloat(&gpsInfo->gga.hdop), gpsInfo->gga.satellites_tracked, gpsInfo->gsv[0].total_sats, isFixedStr, latitude,longitude, minmea_tofloat(&gpsInfo->gga.altitude));*/
        /*snprintf(mqttBuffer,sizeof(mqttBuffer),"GPS fix mode:%d, BDS fix mode:%d, fix quality:%d, satellites tracked:%d, gps sates total:%d, is fixed:%s, coordinate:WGS84, Latitude:%f, Longitude:%f, unit:degree,altitude:%f", gpsInfo->gsa[0].fix_type, gpsInfo->gsa[1].fix_type,
                  gpsInfo->gga.fix_quality, gpsInfo->gga.satellites_tracked, gpsInfo->gsv[0].total_sats, isFixedStr, latitude, longitude, gpsInfo->gga.altitude);*/

        UratPrint(mqttBuffer);
    }
}

void MqttPublishTelemetry(void)
{
    static int cnt = 0;

    /* We have to publish IGN periodically even without change 
       because it has expiring timeout. One time per 10 min will be enough. */
    if (cnt % 600 == 0)
    {
        MqttPublishIgn();
    }

    /* One time per minute */
    if (cnt % 60 == 15)
    {
        MqttPublishBattery();
    }

    /* One time per minute */
    if (cnt % 60 == 45)
    {
        MqttPublishLiion();
    }

    /* Each 10s with 5s timeshift */
    if (cnt % 10 == 5)
    {
        MqttPublishSpeed();
    }

    /* Each 10s */
    /* TODO: Make it flexible */
    if (cnt % 10 == 0)
    {
        MqttPublishTracker();
    }

    cnt++;
}

void MqttInit(void)
{
    Trace(1, "MQTT init");
    //     char iccid[21];
    // char imsi[16];
    char *iccid = (uint8_t *)malloc(21);
    SIM_GetICCID(iccid);
    Trace(1, "ICCID: %s", iccid);
    char *imsi = (uint8_t *)malloc(16);
    SIM_GetIMSI(imsi);
    Trace(1, "IMSI: %s", imsi);

    memset(imei, 0, sizeof(imei));
    INFO_GetIMEI(imei);
    Trace(1, "IMEI: %s", imei);

    snprintf(mqttBuffer, sizeof(mqttBuffer), "{\"action\":\"system\",\"data\",{\"imei\":\"%s\",\"imsi\":\"%s\",\"iccid\":\"%s\"}", imei, imsi, iccid);
    free(iccid);
    free(imsi);
    iccid = imsi = NULL;

    UratPrint(mqttBuffer);

    snprintf(mqttStateTopic, sizeof(mqttStateTopic), MQTT_STATE_TOPIC_FORMAT, imei);
    snprintf(mqttDataTopic, sizeof(mqttDataTopic), MQTT_DATA_TOPIC_FORMAT, imei);
    mqttClient = MQTT_ClientNew();
    memset(&ci, 0, sizeof(MQTT_Connect_Info_t));

    ci.client_id = imei;
    ci.client_user = CLIENT_USER;
    ci.client_pass = CLIENT_PASS;
    ci.keep_alive = 20;
    ci.clean_session = 1;
    ci.use_ssl = false;
#if 1 // Set connection status offline
    ci.will_qos = 2;
    ci.will_topic = mqttStateTopic;
    ci.will_retain = 1;
    ci.will_msg = MQTT_PAYLOAD_STATE_OFFLINE;
#endif
#if 0 // TLS
    ci.use_ssl = true;
    ci.ssl_verify_mode = MQTT_SSL_VERIFY_MODE_OPTIONAL;
    ci.ca_cert = ca_crt;
    ci.ca_crl = NULL;
    ci.client_cert = client_crt;
    ci.client_key  = client_key;
    ci.client_key_passwd = NULL;
    ci.broker_hostname = BROKER_HOSTNAME;
    ci.ssl_min_version = MQTT_SSL_VERSION_TLSv1_1;
    ci.ssl_max_version = MQTT_SSL_VERSION_TLSv1_2;
    ci.entropy_custom = "GPRS_A9";
#endif
}

void MqttTask(void *pData)
{
    WatchDog_Open(WATCHDOG_SECOND_TO_TICK(MQTT_WATCHDOG_INTERVAL));

    semMqttStart = OS_CreateSemaphore(0);
    OS_WaitForSemaphore(semMqttStart, OS_WAIT_FOREVER);
    OS_DeleteSemaphore(semMqttStart);
    semMqttStart = NULL;

    WatchDog_KeepAlive();
    /*
    // Randomize the TCP Sequence number
    uint8_t count = clock() & 0x07; // Semi-random value from 0 - 6
    for(uint8_t i = 0; i < count; i++) {
        Socket_TcpipClose(Socket_TcpipConnect(TCP, "127.0.0.1", 12345));
    }
*/
    MqttInit();

    while (1)
    {
        switch (mqttStatus)
        {
        case MQTT_STATUS_DISCONNECTED:
            MqttConnect();
            break;

        case MQTT_STATUS_CONNECTING:
            Trace(1, "MQTT is connecting...");
            break;

        case MQTT_STATUS_CONNECTED:
            MqttPublishStateOnline();
            MqttPublishIgn();
            break;

        case MQTT_STATUS_PUBLISHING_ONLINE:
            Trace(1, "MQTT is going online...");
            break;

        case MQTT_STATUS_ONLINE:
        case MQTT_STATUS_LOCATION_PUBLISHED:
            MqttPublishTelemetry();
            break;

        case MQTT_STATUS_OFFLINE:
            break;

        default:
            break;
        }

        OS_Sleep(MQTT_INTERVAL);
    }
}

void MqttTaskInit(void)
{
    mqttTaskHandle = OS_CreateTask(MqttTask,
                                   NULL, NULL, MQTT_TASK_STACK_SIZE, MQTT_TASK_PRIORITY, 0, 0, MQTT_TASK_NAME);
}
