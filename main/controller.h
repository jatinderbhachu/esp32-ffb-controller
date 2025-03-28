#ifndef ESPNOW_DRIVER_H
#define ESPNOW_DRIVER_H

#include "ESPNowDriverProtocol.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <array>
#include <atomic>

class Controller {
public:
  void init();
  void deinit();

  static void recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data,
                      int len);
  static void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
  static void run_espnow_task(void *data);
  static void run_motor_control_task(void *data);
  static void run_controller_task(void *data);

  static void run_gpio_task(void *data);

  void init_motor();

private:
  void handle_receive_msg(const uint8_t *mac_addr, const uint8_t *data,
                          const uint32_t len);

  void send_state();

  bool m_connected = false;

  uint8_t m_button_values;
  std::array<int, 2> m_axis_values;
  std::array<uint8_t, ESP_NOW_ETH_ALEN> m_receiver_addr;
};

#endif
