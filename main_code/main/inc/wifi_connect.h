/*
 * wifi_app.h
 */

#ifndef MAIN_WIFI_APP_H_
#define MAIN_WIFI_APP_H_

#define EXAMPLE_ESP_MAXIMUM_RETRY 1
#define ESP_WIFI_AP_SSID "GOLF CAR"
#define ESP_WIFI_AP_PASSWD ""
#define WIFI_SSID "UIT_CAR_RACING_2023"
#define WIFI_PASS "sinhvien_CEEC"
#define ESP_WIFI_CHANNEL 1
#define MAX_STA_CONN 1

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

void event_handler(void *, esp_event_base_t, int32_t, void *);
void wifi_init(void);
#endif /* MAIN_WIFI_APP_H_ */
