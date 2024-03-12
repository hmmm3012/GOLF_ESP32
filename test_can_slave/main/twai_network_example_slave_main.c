/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a slave node in a TWAI network. The slave
 * node is responsible for sending data messages to the master. The example will
 * execute multiple iterations, with each iteration the slave node will do the
 * following:
 * 1) Start the TWAI driver
 * 2) Listen for ping messages from master, and send ping response
 * 3) Listen for start command from master
 * 4) Send data messages to master and listen for stop command
 * 5) Send stop response to master
 * 6) Stop the TWAI driver
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

/* --------------------- Definitions and static variables ------------------ */
// Example Configuration
#define DATA_PERIOD_MS 50
#define NO_OF_ITERS 3
#define ITER_DELAY_MS 1000
#define RX_TASK_PRIO 8   // Receiving task priority
#define TX_TASK_PRIO 9   // Sending task priority
#define CTRL_TSK_PRIO 10 // Control task priority
#define TX_GPIO_NUM CONFIG_EXAMPLE_TX_GPIO_NUM
#define RX_GPIO_NUM CONFIG_EXAMPLE_RX_GPIO_NUM
#define TAG "TWAI Slave"

#define ID_MASTER_STOP_CMD 0x0A0
#define ID_MASTER_START_CMD 0x0A1
#define ID_MASTER_PING 0x0A2
#define ID_SLAVE_STOP_RESP 0x0B0
#define ID_SLAVE_DATA 0x0B1
#define ID_SLAVE_PING_RESP 0x0B2

typedef enum
{
    TX_SEND_PING_RESP,
    TX_SEND_DATA,
    TX_SEND_STOP_RESP,
    TX_TASK_EXIT,
} tx_task_action_t;

typedef enum
{
    RX_RECEIVE_PING,
    RX_RECEIVE_START_CMD,
    RX_RECEIVE_STOP_CMD,
    RX_TASK_EXIT,
} rx_task_action_t;

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_message_t ping_resp = {.identifier = ID_SLAVE_PING_RESP, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
static const twai_message_t stop_resp = {.identifier = ID_SLAVE_STOP_RESP, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
twai_message_t data_message = {.identifier = ID_SLAVE_DATA, .data_length_code = 4, .data = {0, 0, 0, 0, 0, 0, 0, 0}};

static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t stop_data_sem;
static SemaphoreHandle_t done_sem;

/* --------------------------- Tasks and Functions -------------------------- */
static void twai_alert_all()
{
    twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);
    uint32_t alerts;
    twai_read_alerts(&alerts, pdMS_TO_TICKS(100));
    if (alerts & TWAI_ALERT_ABOVE_ERR_WARN)
    {
        ESP_LOGI(TAG, "Surpassed Error Warning Limit");
    }
    if (alerts & TWAI_ALERT_ERR_PASS)
    {
        ESP_LOGI(TAG, "Entered Error Passive state");
    }
    if (alerts & TWAI_ALERT_BUS_OFF)
    {
        ESP_LOGI(TAG, "Bus Off state");
        // Prepare to initiate bus recovery, reconfigure alerts to detect bus recovery completion
        // twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);
        for (int i = 3; i > 0; i--)
        {
            ESP_LOGW(TAG, "Initiate bus recovery in %d", i);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        twai_initiate_recovery(); // Needs 128 occurrences of bus free signal
        ESP_LOGI(TAG, "Initiate bus recovery");
    }
    if (alerts & TWAI_ALERT_BUS_RECOVERED)
    {
        // Bus recovery was successful, exit control task to uninstall driver
        ESP_LOGI(TAG, "Bus Recovered");
        twai_start();
    }
    if (alerts & TWAI_ALERT_TX_RETRIED)
    {
        ESP_LOGI(TAG, "Message Transmit Retried");
    }
    if (alerts & TWAI_ALERT_TX_FAILED)
    {
        ESP_LOGI(TAG, "Message Transmit Failed");
    }
    // Add alerts for any remaining alerts here
    if (alerts & TWAI_ALERT_TX_IDLE)
    {
        ESP_LOGI(TAG, "Transmit Idle");
    }
    if (alerts & TWAI_ALERT_RX_DATA)
    {
        ESP_LOGI(TAG, "Receive Data");
    }
}
static void twai_receive_task(void *arg)
{
    while (1)
    {
        // twai_message_t rx_msg;
        // esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(500));
        // if (ret == ESP_OK)
        //     ESP_LOGI(TAG, "Received message from %ld - %d", rx_msg.identifier, rx_msg.data[0]);
        // twai_alert_all();
        // vTaskDelay(pdMS_TO_TICKS(20));
        rx_task_action_t action;
        xQueueReceive(rx_task_queue, &action, portMAX_DELAY);
        if (action == RX_RECEIVE_PING)
        {
            // Listen for pings from master
            twai_message_t rx_msg;
            while (1)
            {
                twai_receive(&rx_msg, pdMS_TO_TICKS(200));
                ESP_LOGI(TAG, "Received message from %ld: [%d][%d][%d][%d][%d][%d]", rx_msg.identifier, rx_msg.data[2], rx_msg.data[3], rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
                twai_alert_all();
                if (rx_msg.identifier == ID_MASTER_PING)
                {
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        }
        else if (action == RX_RECEIVE_START_CMD)
        {
            // Listen for start command from master
            twai_message_t rx_msg;
            while (1)
            {
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_MASTER_START_CMD)
                {
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        }
        else if (action == RX_RECEIVE_STOP_CMD)
        {
            // Listen for stop command from master
            twai_message_t rx_msg;
            while (1)
            {
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_MASTER_STOP_CMD)
                {
                    xSemaphoreGive(stop_data_sem);
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        }
        else if (action == RX_TASK_EXIT)
        {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg)
{
    while (1)
    {
        tx_task_action_t action;
        xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

        if (action == TX_SEND_PING_RESP)
        {
            // Transmit ping response to master
            twai_transmit(&ping_resp, portMAX_DELAY);
            ESP_LOGI(TAG, "Transmitted ping response");
            xSemaphoreGive(ctrl_task_sem);
        }
        else if (action == TX_SEND_DATA)
        {
            // Transmit data messages until stop command is received
            ESP_LOGI(TAG, "Start transmitting data");
            while (1)
            {
                // FreeRTOS tick count used to simulate sensor data
                uint32_t sensor_data = xTaskGetTickCount();
                for (int i = 0; i < 4; i++)
                {
                    data_message.data[i] = (sensor_data >> (i * 8)) & 0xFF;
                }
                twai_transmit(&data_message, portMAX_DELAY);
                ESP_LOGI(TAG, "Transmitted data value %" PRIu32, sensor_data);
                vTaskDelay(pdMS_TO_TICKS(DATA_PERIOD_MS));
                if (xSemaphoreTake(stop_data_sem, 0) == pdTRUE)
                {
                    break;
                }
            }
        }
        else if (action == TX_SEND_STOP_RESP)
        {
            // Transmit stop response to master
            twai_transmit(&stop_resp, portMAX_DELAY);
            ESP_LOGI(TAG, "Transmitted stop response");
            xSemaphoreGive(ctrl_task_sem);
        }
        else if (action == TX_TASK_EXIT)
        {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_control_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_task_action_t tx_action;
    rx_task_action_t rx_action;

    for (int iter = 0; iter < NO_OF_ITERS; iter++)
    {
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(TAG, "Driver started");

        // Listen of pings from master
        rx_action = RX_RECEIVE_PING;
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        // Send ping response
        tx_action = TX_SEND_PING_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        // Listen for start command
        rx_action = RX_RECEIVE_START_CMD;
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        // Start sending data messages and listen for stop command
        tx_action = TX_SEND_DATA;
        rx_action = RX_RECEIVE_STOP_CMD;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        // Send stop response
        tx_action = TX_SEND_STOP_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        // Wait for bus to become free
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        while (status_info.msgs_to_tx > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            twai_get_status_info(&status_info);
        }

        ESP_ERROR_CHECK(twai_stop());
        ESP_LOGI(TAG, "Driver stopped");
        vTaskDelay(pdMS_TO_TICKS(ITER_DELAY_MS));
    }

    // Stop TX and RX tasks
    tx_action = TX_TASK_EXIT;
    rx_action = RX_TASK_EXIT;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

    // Delete Control task
    xSemaphoreGive(done_sem);
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Add short delay to allow master it to initialize first
    for (int i = 3; i > 0; i--)
    {
        printf("Slave starting in %d\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Create semaphores and tasks
    tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
    rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
    ctrl_task_sem = xSemaphoreCreateBinary();
    stop_data_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);

    // Install TWAI driver, trigger tasks to start
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");

    xSemaphoreGive(ctrl_task_sem);           // Start Control task
    xSemaphoreTake(done_sem, portMAX_DELAY); // Wait for tasks to complete

    // Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(TAG, "Driver uninstalled");

    // Cleanup
    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(stop_data_sem);
    vSemaphoreDelete(done_sem);
    vQueueDelete(tx_task_queue);
    vQueueDelete(rx_task_queue);
}
