#include <string.h>
#include <stdio.h>
#include <api_os.h>
#include <api_call.h>
#include <api_gps.h>
#include <api_event.h>
#include <api_hal_uart.h>
#include <api_debug.h>
#include "buffer.h"
#include "gps_parse.h"
#include "math.h"
#include "gps.h"
#include "api_hal_pm.h"
#include "api_mqtt.h"
#include "api_key.h"
#include "time.h"
#include "api_info.h"
#include "assert.h"
#include "api_socket.h"
#include "api_network.h"
#include "api_hal_gpio.h"
#include "api_hal_adc.h"
#include "api_hal_pm.h"
#include "api_hal_watchdog.h"

#include "led.h"
#include "call.h"
#include "pm_task.h"
#include "adc_task.h"
#include "gps_task.h"
#include "mqtt_task.h"

#define MAIN_TASK_STACK_SIZE (2048 * 4)
#define MAIN_TASK_PRIORITY 0
#define MAIN_TASK_NAME "Main Task"

#define PDP_CONTEXT_APN "cmiot"
#define PDP_CONTEXT_USERNAME ""
#define PDP_CONTEXT_PASSWD ""

static HANDLE mainTaskHandle = NULL;

// GPIO_config_t stateLed = {
//     .mode = GPIO_MODE_OUTPUT,
//     .pin = GPIO_PIN28,
//     .defaultLevel = GPIO_LEVEL_HIGH};

static void OnUart1ReceivedData(UART_Callback_Param_t param)
{
    UART_Write(UART1, param.buf, param.length);
    Trace(1, "uart1 interrupt received data,length:%d:data:%s", param.length, param.buf);
}

UART_Config_t configUart = {
    .baudRate = UART_BAUD_RATE_115200,
    .dataBits = UART_DATA_BITS_8,
    .stopBits = UART_STOP_BITS_1,
    .parity = UART_PARITY_NONE,
    .rxCallback = OnUart1ReceivedData,
    .useEvent = false,
};

bool AttachActivate()
{
    uint8_t status;
    bool ret = Network_GetAttachStatus(&status);

    if (!ret)
    {
        Trace(2, "Get attach status fail");
        return false;
    }

    Trace(2, "Attach status: %d", status);

    if (!status)
    {
        ret = Network_StartAttach();

        if (!ret)
        {
            Trace(2, "Network attach fail");
            return false;
        }
        LED_SetBlink(LED_LED1, LED_BLINK_FREQ_4HZ, LED_BLINK_DUTY_HALF);
    }
    else
    {
        ret = Network_GetActiveStatus(&status);

        if (!ret)
        {
            Trace(2, "Get activate staus fail");
            return false;
        }
        else
        {
            LED_SetBlink(LED_LED1, LED_BLINK_FREQ_2HZ, LED_BLINK_DUTY_HALF);
        }

        Trace(2, "Activate status: %d", status);

        if (!status)
        {
            Network_PDP_Context_t context = {
                .apn = PDP_CONTEXT_APN,
                .userName = PDP_CONTEXT_USERNAME,
                .userPasswd = PDP_CONTEXT_PASSWD};
            Network_StartActive(context);
        }
    }

    return true;
}

void EventDispatch(API_Event_t *pEvent)
{
    // int keyDownAt = 0;

    switch (pEvent->id)
    {
    case API_EVENT_ID_NO_SIMCARD:
        Trace(10, "!!NO SIM CARD%d!!!!", pEvent->param1);
        break;
    // case API_EVENT_ID_KEY_DOWN:
    //     Trace(1, "key down, key:0x%02x", pEvent->param1);
    //     if (pEvent->param1 == KEY_POWER)
    //     {
    //         LED_TurnOn(LED_LED1);
    //         LED_TurnOn(LED_LED2);
    //         keyDownAt = time(NULL);
    //     }
    //     break;
    // case API_EVENT_ID_KEY_UP:
    //     Trace(1, "key release, key:0x%02x", pEvent->param1);
    //     if (pEvent->param1 == KEY_POWER)
    //     {
    //         if (keyDownAt != 0 && time(NULL) - keyDownAt > 3)
    //         {
    //             LED_SetBlink(LED_LED1, LED_BLINK_FREQ_1HZ, LED_BLINK_DUTY_FULL);
    //             LED_SetBlink(LED_LED2, LED_BLINK_FREQ_1HZ, LED_BLINK_DUTY_FULL);
    //             OS_Sleep(3000);
    //             PM_ShutDown();
    //         }
    //     }
    //     break;
    case API_EVENT_ID_NETWORK_REGISTER_DENIED:
        Trace(2, "network register denied");
        // LED_SetBlink(LED_LED1, LED_BLINK_FREQ_1HZ, LED_BLINK_DUTY_FULL);
        break;
    case API_EVENT_ID_SYSTEM_READY:
        LED_SetBlink(LED_LED1, LED_BLINK_FREQ_8HZ, LED_BLINK_DUTY_HALF);
        Trace(2, "system initialize complete");
        break;
    case API_EVENT_ID_NETWORK_REGISTER_NO:
        // LED_SetBlink(LED_LED1, LED_BLINK_FREQ_1HZ, LED_BLINK_DUTY_FULL);
        Trace(2, "network register no");
        break;
    case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
    case API_EVENT_ID_NETWORK_REGISTERED_HOME:
    case API_EVENT_ID_NETWORK_DETACHED:
    case API_EVENT_ID_NETWORK_ATTACH_FAILED:
    case API_EVENT_ID_NETWORK_ATTACHED:
    case API_EVENT_ID_NETWORK_DEACTIVED:
    case API_EVENT_ID_NETWORK_ACTIVATE_FAILED:
        AttachActivate();
        break;
    case API_EVENT_ID_NETWORK_ACTIVATED:
        Trace(2, "network activate success");

        if (semMqttStart)
        {
            LED_SetBlink(LED_LED1, LED_BLINK_FREQ_1HZ, LED_BLINK_DUTY_EMPTY);
            OS_ReleaseSemaphore(semMqttStart);
        }

        break;

    case API_EVENT_ID_GPS_UART_RECEIVED:
        // Trace(1, "received GPS data, length:%d, data:%s, flag:%d", pEvent->param1, pEvent->pParam1, flag);
        //LED_TurnOn(LED_LED1);
        GPS_Update(pEvent->pParam1, pEvent->param1);
        //LED_TurnOff(LED_LED1);
        break;

    case API_EVENT_ID_CALL_DIAL: //param1: isSuccess, param2:error code(CALL_Error_t)
                                 //Trace(1,"Is dial success: %d, error code: %d", pEvent->param1, pEvent->param2);
        CallFinish();
        break;

    case API_EVENT_ID_SIGNAL_QUALITY:
        Trace(2,"CSQ: %d", pEvent->param1);
        MqttPublishGsm(pEvent->param1);
        break;
    case API_EVENT_ID_NETWORK_GOT_TIME:
    {
        //char buf[64];
        char *buf = (char *)malloc(64);
        // memset(buf, 0, 64);
        RTC_Time_t *t = (RTC_Time_t *)pEvent->pParam1;
        sprintf(buf, "NETWORK_GOT_TIME %04d-%02d-%02d %02d:%02d:%02d+%d,%d", t->year, t->month, t->day, t->hour, t->minute, t->second, t->timeZone, t->timeZoneMinutes);
        Trace(1, buf);
        free(buf);
        buf = NULL;
        break;
    }
    default:
        break;
    }
}

/**
 * The Main task
 *
 * Init HW and run MQTT task. 
 *
 * @param  pData Parameter passed in when this function is called
 */
void app_MainTask(void *pData)
{
    API_Event_t *event = NULL;

    Trace(1, "Main Task started");

    TIME_SetIsAutoUpdateRtcTime(true); // Local time will be synced with GPS

    PM_PowerEnable(POWER_TYPE_VPAD, true); // GPIO0  ~ GPIO7  and GPIO25 ~ GPIO36    2.8V
    //PM_SetSysMinFreq(PM_SYS_FREQ_312M);

    // GPIO_Init(stateLed);

    LED_Init();
    LED_SetBlink(LED_LED1, LED_BLINK_FREQ_1HZ, LED_BLINK_DUTY_FULL);
    // LED_SetBlink(LED_LED2, LED_BLINK_FREQ_1HZ, LED_BLINK_DUTY_FULL);

    UART_Init(UART1, configUart);

    // Init call button
    CallInit();

    // Create PM task
    PmTaskInit();

    // Create ADC task
    AdcTaskInit();

    // Create MQTT task
    MqttTaskInit();

    GpsTaskInit();

    // Wait and process system events
    while (1)
    {
        if (OS_WaitEvent(mainTaskHandle, (void **)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}

/**
 * The entry point of application. Just create the main task.
 */
void app_Main(void)
{
    mainTaskHandle = OS_CreateTask(app_MainTask,
                                   NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&mainTaskHandle);
}
