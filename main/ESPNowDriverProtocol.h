#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

// #define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
// #define ESPNOW_WIFI_IF WIFI_IF_AP
#define ESPNOW_WIFI_IF WIFI_IF_STA
#define ESPNOW_WIFI_PHY_RATE WIFI_PHY_RATE_MCS7_SGI
// #define ESPNOW_WIFI_PHY_RATE WIFI_PHY_RATE_54M
// #define ESPNOW_WIFI_PHY_RATE WIFI_PHY_RATE_11M_S

#define ESPNOW_QUEUE_SIZE 6
#define ESPNOW_MAXDELAY 512
#define ESPNOW_CHANNEL 6
#define ESPNOW_PMK "pmk123456FFB"

#include "esp_now.h"
#include <stdint.h>

#define ESPNOW_MAX_DATA_SIZE 250
#define ESPNOW_PROTOCOL_VERSION 1

constexpr uint8_t ESPNOW_BROADCAST_ADDR[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                             0xFF, 0xFF, 0xFF};

enum class CmdType : uint8_t {
  Unknown = 0,
  Connect,
  Connect_ack,
  Disconnect,
  Heartbeat,
  EncoderPos,
  ApplyTorque,
};

enum class EventType { Send, Receive };

struct EventCallbackData {
  EventType type;

  union {
    struct {
      uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    } send;

    struct {
      uint8_t mac_addr[ESP_NOW_ETH_ALEN];
      uint8_t *data;
      uint32_t len;
    } receive;
  };
};

#endif