#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include <esp_http_server.h>
#include "sys_config.h"
#include "cJSON.h"
#include "twai_connect.h"
#include "ws_server.h"
static int node_send = 0;
static int automatic = 0;
static const char *TAG = "ws_echo_server";
extern const unsigned char control_start[] asm("_binary_control_html_start");
extern const unsigned char control_end[] asm("_binary_control_html_end");
/*
 * Structure holding server handle
 * and internal socket fd in order
 * to use out of request send
 */
struct async_resp_arg
{
    httpd_handle_t hd;
    int fd;
};

/*
 * async send function, which we put into the httpd work queue
 */
static void ws_async_send(void *arg)
{
    static const char *data = "Async data";
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)data;
    ws_pkt.len = strlen(data);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
}

static esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    if (resp_arg == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    esp_err_t ret = httpd_queue_work(handle, ws_async_send, resp_arg);
    if (ret != ESP_OK)
    {
        free(resp_arg);
    }
    return ret;
}

/*
 * This handler echos back the received ws data
 * and triggers an async send if certain message received
 */
static esp_err_t echo_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    const char *mode = req->user_ctx;
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    if (mode[0] == 'a' && automatic == 0)
    {
        return ESP_OK;
    }
    if (ws_pkt.len)
    {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);

        const char *ws_payload = (const char *)ws_pkt.payload;
        cJSON *root = cJSON_Parse(ws_payload);
        if (root != NULL)
        {
            cJSON *steering_msg = cJSON_GetObjectItemCaseSensitive(root, "steer");
            cJSON *speed_msg = cJSON_GetObjectItemCaseSensitive(root, "speed");
            cJSON *brake_msg = cJSON_GetObjectItemCaseSensitive(root, "brake");
            cJSON *automatic_msg = cJSON_GetObjectItemCaseSensitive(root, "automatic");

            if (brake_msg != NULL)
            {
                // Send CAN here
                char brake_data[MAX_LENGTH_DATA];
                sprintf(brake_data, "#%d=%d!\r\n", ID_TARGET_EGN_CTRL_NODE, brake_msg->valueint);
                twai_msg brake_msg = {
                    .type_id = {
                        .msg_type = ID_MSG_TYPE_CMD_FRAME,
                        .target_type = ID_TARGET_EGN_CTRL_NODE,
                    },
                    .msg = brake_data,
                    .msg_len = strlen(brake_data),
                };
                twai_transmit_msg(&brake_msg);
            }
            else if (steering_msg != NULL && speed_msg != NULL)
            {
                int speed = speed_msg->valueint;
                int steering = steering_msg->valueint;
                printf("Speed: %d, Steering: %d\n", speed, steering);
                // Send CAN here
                char steer_data[MAX_LENGTH_DATA];
                char eng_data[MAX_LENGTH_DATA];
                sprintf(steer_data, "#%d=%d\r\n", ID_TARGET_STEER_CTRL_NODE, steering);
                twai_msg steer_msg = {
                    .type_id = {
                        .msg_type = ID_MSG_TYPE_CMD_FRAME,
                        .target_type = ID_TARGET_STEER_CTRL_NODE,
                    },
                    .msg = steer_data,
                    .msg_len = strlen(steer_data),
                };

                sprintf(eng_data, "#%d=%d\r\n", ID_TARGET_EGN_CTRL_NODE, speed);
                twai_msg egn_msg = {
                    .type_id = {
                        .msg_type = ID_MSG_TYPE_CMD_FRAME,
                        .target_type = ID_TARGET_EGN_CTRL_NODE,
                    },
                    .msg = eng_data,
                    .msg_len = strlen(eng_data),
                };
                twai_transmit_msg(&steer_msg);
                twai_transmit_msg(&egn_msg);
            }
            else if (automatic_msg != NULL)
            {
                automatic = automatic_msg->valueint;
                printf("Automatic: %d\n", automatic);
            }
            else
            {
                ESP_LOGI(TAG, "RCV: %s", ws_payload);
            }
        }
    }
    // ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char *)ws_pkt.payload, "Trigger async") == 0)
    {
        free(buf);
        return trigger_async_send(req->handle, req);
    }
    // ret = httpd_ws_send_frame(req, &ws_pkt);
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
    // }
    free(buf);
    return ret;
}
static esp_err_t home_handler(httpd_req_t *req)
{
    const size_t control_size = (control_end - control_start);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_send(req, (const char *)control_start, control_size);
    ESP_LOGI(TAG, "Sent html");
    return ESP_OK;
}
static const httpd_uri_t ws_automatic = {
    .uri = "/automatic",
    .method = HTTP_GET,
    .handler = echo_handler,
    .user_ctx = "automatic",
    .is_websocket = true,
};
static const httpd_uri_t ws_manual = {
    .uri = "/manual",
    .method = HTTP_GET,
    .handler = echo_handler,
    .user_ctx = "manual",
    .is_websocket = true,
};
static const httpd_uri_t home = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = home_handler,
    .user_ctx = NULL,
};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Registering the ws handler
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &ws_automatic);
        httpd_register_uri_handler(server, &ws_manual);
        httpd_register_uri_handler(server, &home);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

static void disconnect_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server)
    {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK)
        {
            *server = NULL;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

static void connect_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server == NULL)
    {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
