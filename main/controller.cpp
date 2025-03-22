#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "controller.h"

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include <esp_private/wifi.h>

#include "esp_simplefoc.h"
#include "sensors/Encoder.h"

#include "esp_adc/adc_oneshot.h"

#include <memory>

#define TAG "Controller"

// simplefoc stuff
// static BLDCMotor motor = BLDCMotor(7, 16.4, 90);
static BLDCMotor motor = BLDCMotor(7);
// static BLDCDriver3PWM driver = BLDCDriver3PWM(18, 16, 17, 21); // mini v1.0
static BLDCDriver3PWM driver = BLDCDriver3PWM(38, 37, 36, 35);
static Encoder encoder = Encoder(8, 9, 1024);
static Commander command = Commander(Serial);

static Controller s_controller;
static QueueHandle_t s_espnow_queue = nullptr;

// channel A and B callbacks
void encoderDoA() { encoder.handleA(); }
void encoderDoB() { encoder.handleB(); }

// void doMotor(char* cmd) { command.motor(&motor, cmd); }
void Controller::init_motor() {
  // SimpleFOCDebug::enable();
  // Serial.begin(115200);
  encoder.init();
  encoder.enableInterrupts(encoderDoA, encoderDoB);

  // command.add('T', doMotor, const_cast<char *>("motor"));
  // motor.useMonitoring(Serial);
  // motor.monitor_downsample = 0; // disable monitor at first - optional
  command.verbose = VerboseMode::machine_readable;
  driver.voltage_power_supply = 8;

  // Use LEDC or MCPWM https://docs.simplefoc.com/esp_mcu#pwm-drivers
  // auto ret = driver.init({1, 2, 3}); // LEDC
  auto ret = driver.init(0); // MCPWM
  if (ret == 0) {
    ESP_LOGE(TAG, "driver.init() failed");
  }
  motor.linkSensor(&encoder);
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.voltage_limit = 8; // Volts
  motor.current_limit = 2; // amps

  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.00;
  motor.LPF_velocity.Tf = 0.05;

  motor.torque_controller = TorqueControlType::voltage;

  motor.init();
  motor.initFOC();
  motor.target = 0;
}

void Controller::init() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  cfg.ampdu_tx_enable = 0;
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_wifi_internal_set_fix_rate(ESPNOW_WIFI_IF, true,
                                                 ESPNOW_WIFI_PHY_RATE));

  s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(EventCallbackData));
  if (s_espnow_queue == NULL) {
    ESP_LOGE(TAG, "failed to create espnow queue");
  }

  ESP_ERROR_CHECK(esp_now_init());

  // ESP_ERROR_CHECK(esp_now_register_send_cb(this->send_cb));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(this->recv_cb));

  ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)ESPNOW_PMK));

  // Add broadcast address to peer list
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = ESPNOW_WIFI_IF;
  peer.encrypt = false;

  memcpy(peer.peer_addr, ESPNOW_BROADCAST_ADDR, ESP_NOW_ETH_ALEN);
  ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void Controller::deinit() { esp_now_deinit(); }

void Controller::recv_cb(const esp_now_recv_info_t *recv_info,
                         const uint8_t *data, int len) {
  EventCallbackData event;
  event.type = EventType::Receive;
  uint8_t *mac_addr = recv_info->src_addr;

  if (mac_addr == NULL) {
    ESP_LOGE(TAG, "Send cb arg error");
    return;
  }

  memcpy(event.receive.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  event.receive.data = (uint8_t *)malloc(len);
  event.receive.len = len;
  memcpy(event.receive.data, data, len);

  if (xQueueSend(s_espnow_queue, &event, ESPNOW_MAXDELAY) != pdTRUE) {
    ESP_LOGW(TAG, "Send send queue fail");
  }
}

void Controller::handle_receive_msg(const uint8_t *mac_addr,
                                    const uint8_t *data, const uint32_t len) {

  // parse received data
  CmdType cmd_type = (CmdType)data[0];

  // ESP_LOGI(TAG, "handle receive msg %d", (int)cmd_type);

  switch (cmd_type) {
  case CmdType::Connect_ack: {
    memcpy(m_receiver_addr.data(), mac_addr, m_receiver_addr.size());
    ESP_LOGI(TAG, "Connected to %X:%X:%X:%X:%X:%X", mac_addr[0], mac_addr[1],
             mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    m_connected = true;
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = ESPNOW_WIFI_IF;
    peer.encrypt = false;

    memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);

    if (!esp_now_is_peer_exist(peer.peer_addr)) {
      esp_now_add_peer(&peer);
    }
  } break;
  case CmdType::ApplyTorque: {
    // int16_t power = *((int16_t*)&data[1]);
    int16_t power = 0;
    memcpy(&power, &data[1], sizeof(int16_t));

    float torque = ((float)power / (float)INT16_MAX) * 5.0f;
    motor.target = torque;
    // ESP_LOGI(TAG, "torque: %1.2f -> %d", torque, power);
  } break;
  default: {
    // ESP_LOGI(TAG, "unknown msg type %d", (int)cmd_type);
  } break;
  }
}

void Controller::run_espnow_task(void *data) {
  Controller *controller = static_cast<Controller *>(data);

  EventCallbackData event;
  while (xQueueReceive(s_espnow_queue, &event, portMAX_DELAY) == pdTRUE) {
    switch (event.type) {
    case EventType::Receive: {
      controller->handle_receive_msg(event.receive.mac_addr, event.receive.data,
                                     event.receive.len);

      free(event.receive.data);
    } break;
    case EventType::Send: {
    } break;
    }
  }
}

static void send_connect_req() {
  uint8_t data[ESPNOW_MAX_DATA_SIZE];
  uint32_t len = 0;

  data[len++] = (uint8_t)CmdType::Connect;
  data[len++] = (uint8_t)ESPNOW_PROTOCOL_VERSION;

  if (esp_now_send(ESPNOW_BROADCAST_ADDR, &data[0], len) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send conect request");
  }
}

static void send_pos(const std::array<uint8_t, ESP_NOW_ETH_ALEN> &dst) {
  uint8_t data[ESPNOW_MAX_DATA_SIZE];
  uint32_t len = 0;
  data[len++] = (uint8_t)CmdType::EncoderPos;
  float encoder_angle = encoder.getSensorAngle();
  memcpy(&data[len], &encoder_angle, sizeof(encoder_angle));
  len += sizeof(int32_t);

  // ESP_LOGI(TAG, "pos %1.2f\n", encoder_angle);
  if (esp_now_send(dst.data(), &data[0], len) != ESP_OK) {
    // ESP_LOGE(TAG, "Send pos error\n");
  }
}

void Controller::run_motor_control_task(void *data) {
  Controller *controller = static_cast<Controller *>(data);
  uint64_t last_pos_time = 0;

  while (true) {
    if (controller->m_connected == false) {
      motor.target = 0.0f;
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }

    motor.loopFOC();
    motor.move();

    vTaskDelay(1);
  }
}

void Controller::run_controller_task(void *data) {
  Controller *controller = static_cast<Controller *>(data);
  uint64_t last_pos_time = 0;

  while (true) {
    if (controller->m_connected == false) {
      send_connect_req();
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }

    // in microseconds
    constexpr uint64_t pos_interval = 500;

    uint64_t now = esp_timer_get_time();
    if (now - last_pos_time >= pos_interval) {
      last_pos_time = now;
      send_pos(controller->m_receiver_addr);
    }

    vTaskDelay(1);
  }
}

extern "C" void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  s_controller.init();
  s_controller.init_motor();

  ESP_ERROR_CHECK(ret);

  xTaskCreate(s_controller.run_motor_control_task, "motor_control", 1024 * 8,
              &s_controller, 20, NULL);

  xTaskCreate(s_controller.run_controller_task, "controller", 1024 * 2,
              &s_controller, 18, NULL);

  xTaskCreate(s_controller.run_espnow_task, "espnow_recv", 1024 * 8, &s_controller,
              18, NULL);
}
