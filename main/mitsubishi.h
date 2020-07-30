#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define INFO_PACKET_LEN              22
#define CONNECT_PACKET_LEN            8

// Power recv_data[8]
#define POWER_OFF                  0x00
#define POWER_ON                   0x01

// Operating mode recv_data[9]
#define HEAT_MODE                  0x01
#define DRY_MODE                   0x02
#define COOL_MODE                  0x03
#define FAN_MODE                   0x07
#define AUTO_MODE                  0x08

// Fan mode recv_data[11]
#define FAN_AUTO                   0x00
#define FAN_QUIET                  0x01
#define FAN_SPEED1                 0x02
#define FAN_SPEED2                 0x03
#define FAN_SPEED3                 0x05
#define FAN_SPEED4                 0x06

// Vane mode recv_data[12]
#define VANE_SWING                 0x07

// Control bit mask
#define POWER_CONTROL              0x01
#define MODE_CONTROL               0x02
#define TEMP_CONTROL               0x04
#define FAN_CONTROL                0x08
#define VANE_CONTROL               0x10


#ifdef __cplusplus
}
#endif 