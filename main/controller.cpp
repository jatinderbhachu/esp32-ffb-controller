#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "controller.h"

#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "esp_err.h"
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

static adc_oneshot_unit_handle_t s_adc2_handle;

static std::vector<uint8_t> s_recv_buffer;
static uint32_t s_recv_offset = 0;
constexpr uint32_t RECV_BUFFER_SIZE = 1024;

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

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void Controller::run_gpio_task(void *arg) {
  Controller *controller = static_cast<Controller *>(arg);
  uint32_t gpio_num;
  while (true) {
    if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
      int val = gpio_get_level((gpio_num_t)gpio_num);
      if (val != 0) {
        controller->m_button_values |= (0x1 << (gpio_num - 4));
      } else {
        controller->m_button_values &= ~(0x1 << (gpio_num - 4));
      }
    } else {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
}

void Controller::init() {

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  adc_oneshot_unit_init_cfg_t adc_init_config = {
      .unit_id = ADC_UNIT_2,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &s_adc2_handle));

  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(s_adc2_handle, ADC_CHANNEL_4, &config));
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(s_adc2_handle, ADC_CHANNEL_5, &config));

  gpio_config_t io_conf{
      .pin_bit_mask = ((1 << 4) | (1 << 5)),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };
  gpio_config(&io_conf);

  // create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
  gpio_isr_handler_add(GPIO_NUM_4, gpio_isr_handler, (void *)4);
  gpio_isr_handler_add(GPIO_NUM_5, gpio_isr_handler, (void *)5);

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

  if (s_recv_buffer.size() < RECV_BUFFER_SIZE) {
    s_recv_buffer.resize(RECV_BUFFER_SIZE);
  }

  ESP_ERROR_CHECK(esp_now_init());

  ESP_ERROR_CHECK(esp_now_register_send_cb(this->send_cb));
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

  memcpy(event.receive.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  event.receive.data = &s_recv_buffer[s_recv_offset];
  event.receive.len = len;
  memcpy(event.receive.data, data, len);

  if (s_recv_offset + len > s_recv_buffer.size()) {
    s_recv_offset = 0;
  }

  if (xQueueSend(s_espnow_queue, &event, ESPNOW_MAXDELAY) != pdTRUE) {
    ESP_LOGW(TAG, "espnow recv queue fail");
  }
}

void Controller::handle_receive_msg(const uint8_t *mac_addr,
                                    const uint8_t *data, const uint32_t len) {

  // parse received data
  CmdType cmd_type = (CmdType)data[0];

  switch (cmd_type) {
  case CmdType::Connect_ack: {
    memcpy(m_receiver_addr.data(), mac_addr, m_receiver_addr.size());
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = ESPNOW_WIFI_IF;
    peer.encrypt = false;

    memcpy(peer.peer_addr, m_receiver_addr.data(), ESP_NOW_ETH_ALEN);

    if (!esp_now_is_peer_exist(peer.peer_addr)) {
      esp_err_t err = esp_now_add_peer(&peer);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer %s", esp_err_to_name(err));
      } else {
        ESP_LOGI(TAG, "Connected to %X:%X:%X:%X:%X:%X", mac_addr[0],
                 mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
                 mac_addr[5]);
        m_connected = true;
      }
    }
  } break;
  case CmdType::ApplyTorque: {
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

SemaphoreHandle_t s_espnow_sem = NULL;
void Controller::send_cb(const uint8_t *mac_addr,
                         esp_now_send_status_t status) {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(s_espnow_sem, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE)
    taskYIELD();
}

void Controller::send_state() {
  uint8_t data[ESPNOW_MAX_DATA_SIZE];
  uint32_t len = 0;
  data[len++] = (uint8_t)CmdType::ControllerState;

  adc_oneshot_read(s_adc2_handle, ADC_CHANNEL_4, &m_axis_values[0]);
  adc_oneshot_read(s_adc2_handle, ADC_CHANNEL_5, &m_axis_values[1]);

  ControllerState state{
      .encoder_angle = encoder.getSensorAngle(),
      .axis1 = (uint16_t)m_axis_values[0],
      .axis2 = (uint16_t)m_axis_values[1],
      .buttons = m_button_values,
  };
  memcpy(&data[len], &state, sizeof(state));

  len += sizeof(state);

  esp_err_t err = esp_now_send(m_receiver_addr.data(), &data[0], len);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send controller state size %s",
             esp_err_to_name(err));
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
  uint64_t last_update_time = 0;
  s_espnow_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(s_espnow_sem);
  while (true) {
    if (controller->m_connected == false) {
      send_connect_req();
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }

    if (xSemaphoreTake(s_espnow_sem, portMAX_DELAY) == pdTRUE) {
      // in microseconds
      constexpr uint64_t update_interval = 500;

      uint64_t now = esp_timer_get_time();
      if (now - last_update_time >= update_interval) {
        last_update_time = now;
        controller->send_state();
      }
    }
  }
}

extern "C" void app_main(void) {
  s_controller.init();
  s_controller.init_motor();

  xTaskCreate(s_controller.run_motor_control_task, "motor_control", 1024 * 8,
              &s_controller, 20, NULL);

  xTaskCreate(s_controller.run_controller_task, "controller", 1024 * 4,
              &s_controller, 18, NULL);

  xTaskCreate(s_controller.run_espnow_task, "espnow_recv", 1024 * 8,
              &s_controller, 18, NULL);

  xTaskCreate(s_controller.run_gpio_task, "gpio", 1024 * 4, &s_controller, 10,
              NULL);
}
