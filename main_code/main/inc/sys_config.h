#ifndef _SYSCONFIG__H
#define _SYSCONFIG__H

/**
 * Debugger?
 */

// #define DEBUG

#define CHECK_MSG_CRC

#define NODE_ID 5

#define MAX_PACKET_NUMBER 8

#define MAX_NODE_NUMBER 50

#define TWAI_TRANSMIT_WAIT 500

#define LOG_ENABLE_TWAI 1

#define LOG_ENABLE_MQTT 0

// #define INA_219
/**
 * GPIOs defs
 */
// #define LED_BUILDING         ( 2 )
// #define LED_BUILDING         ( 27 )
// #define GPIO_OUTPUT_PIN_SEL  ( 1ULL<<LED_BUILDING )

#define I2C_PORT 0
#define I2C_ADDR INA219_ADDR_GND_GND
#define SDA_GPIO 21
#define SCL_GPIO 22

#define BUTTON_PIN (0)
#define GPIO_INPUT_PIN_SEL (1ULL << BUTTON)
#define BUTTON_PIN_1 (32)
#define GPIO_INPUT_PIN_SEL (1ULL << BUTTON)
#define BUTTON_PIN_2 (33)
#define GPIO_INPUT_PIN_SEL (1ULL << BUTTON)

#define MAX_LENGTH_TOPIC 50
#define MAX_LENGTH_DATA 200

#define RX_TASK_PRIO 1 // Receiving task priority
#define TX_TASK_PRIO 3 // Sending task priority

#define TX_GPIO_NUM CONFIG_TX_GPIO_NUM
#define RX_GPIO_NUM CONFIG_RX_GPIO_NUM

#define ID_MSG_TYPE_ALL_NODE 0x0
#define ID_MSG_TYPE_CMD_FRAME 0x1 // Command Frame
#define ID_MSG_TYPE_ACK_CMD_FRAME 0x2
#define ID_MSG_TYPE_R_N_FRAME 0x3 // Remote/Notice frame
#define ID_MSG_TYPE_DATA_FRAME 0x4

#define ID_MSG_TYPE_TEST_ALL_NODE 0x8
#define ID_MSG_TYPE_TEST_CMD_FRAME 0x9
#define ID_MSG_TYPE_TEST_ACK_CMD_FRAME 0xA
#define ID_MSG_TYPE_TEST_R_N_FRAME 0xB
#define ID_MSG_TYPE_TEST_DATA_FRAME 0xC

#define ID_TARGET_ALL_NODE 0x0
#define ID_TARGET_EGN_CTRL_NODE 0x1       // Engine Control Node
#define ID_TARGET_LIGHT_GPS_CTRL_NODE 0x2 // Light-Speaker and GPS Node
#define ID_TARGET_MASTER_NODE 0x3
#define ID_TARGET_STEER_CTRL_NODE 0x4    // Steering Control Node
#define ID_TARGET_SENSOR_NODE 0x5        // Obstacle Sensor Node Engine
#define ID_TARGET_PW_MANAGEMENT_NODE 0x6 // Power Management Node

#define ID_END_FRAME 0x0
#define ID_FIRST_FRAME 0x1
#define ID_SECOND_FRAME 0x2
#define ID_THIRD_FRAME 0x3
#define ID_FOURTH_FRAME 0x4
#define ID_FIFTH_FRAME 0x5
#define ID_SIX_FRAME 0x6
#define ID_SEVEN_FRAME 0x7

#define ID_ALL_NODE 0x0
#define ID_EGN_CTRL_NODE 0x1       // Engine Control Node
#define ID_LIGHT_GPS_CTRL_NODE 0x2 // Light-Speaker and GPS Node
#define ID_MASTER_NODE 0x3
#define ID_STEER_CTRL_NODE 0x4 // Steering Control Node
#define ID_SENSOR_NODE 0x5     // Obstacle Sensor Node Engine
#define ID_PW_MANAGEMENT_NODE 0x6

#define TYPE_DATA_MSG_SET_ENGINE_SPEED 1
#define TYPE_DATA_MSG_SET_STEER_ANGLE 2
#define TYPE_DATA_MSG_SET_LIGHT 3
#define TYPE_DATA_MSG_SET_BUZZER 4
#define TYPE_DATA_MSG_ENABLE_PID 5

#define TYPE_DATA_MSG_CAR_VELOCITY 10
#define TYPE_DATA_MSG_GPS_DATA 11
#define TYPE_DATA_MSG_POWER_MEASURE 12
#define TYPE_DATA_MSG_DISTANCE_SENSOR 13
#define TYPE_DATA_MSG_IMU_EULER_DATA 14
#define TYPE_DATA_MSG_IMU_ACCEL_DATA 15
#define TYPE_DATA_MSG_IMU_GYRO_DATA 16

#define FIRST_PACKET_SIZE 6
#define NORMAL_PACKET_SIZE 7
/**
 * Info wifi your ssid & passwd
 */

/**
 * Mqtt config
 */
#define MQTT_ADDRESS "192.168.1.101"
// #define MQTT_ADDRESS 		"192.168.137.154"
// #define MQTT_ADDRESS 		"192.168.137.1"
#define MQTT_PORT 1883

/**
 * Topic
 */
#define SPEED_TOPIC_PUB "Data/Speed"
#define SENSOR_TOPIC_PUB "Data/Sensor"
#define POWER_TOPIC_PUB "Data/Power"
#define GPS_TOPIC_PUB "Data/Location"
#define IMU_EULER_TOPIC_PUB "Data/IMUEuler"
#define IMU_ACCEL_TOPIC_PUB "Data/IMUAccel"
#define IMU_GYRO_TOPIC_PUB "Data/IMUGyro"
#define CONNECT_TOPIC_PUB "Data/Connected"
#define DISCONNECT_TOPIC_PUB "Data/Disconnected"

#define SPEED_TOPIC_SUB (const char *)"CarControl/Speed"
#define STEER_ANGLE_TOPIC_SUB (const char *)"CarControl/SteerAngle"
#define LIGHT_TOPIC_SUB (const char *)"CarControl/Light"
#define BUZZER_TOPIC_SUB (const char *)"CarControl/Buzzer"
#define PID_TOPIC_SUB (const char *)"CarControl/EnablePID"
#define MSG_TOPIC_SUB (const char *)"CarControl/Msg"

/**
 * Globals defs
 */
#define TRUE 1
#define FALSE 0

#endif