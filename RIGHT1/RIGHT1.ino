
extern "C"
{
#include "hal_i2s.h"
}
#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1
#define I2S_DOUT      26
#define I2S_BCLK      5
#define I2S_LRC       25
#define I2S_DIN       35
#define I2SR (i2s_port_t)0
#define BLOCK_SIZE 256
//Amp power enable
#define PW GPIO_NUM_21     
#define GAIN GPIO_NUM_23       

 const i2s_config_t i2s_configR = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX ), //  transfer
      .sample_rate = 44100,                  
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // 
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, //
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
      .dma_buf_count = 4,                           // number of buffers
      .dma_buf_len = BLOCK_SIZE                     // samples per buffer
  };
  
      i2s_pin_config_t pin_configR=
      {
      .bck_io_num = I2S_BCLK,    // BCKL
      .ws_io_num = I2S_LRC ,    // LRCL
      .data_out_num = I2S_DOUT,  // DOUT
      .data_in_num = I2S_DIN    // DIN
      };




// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void setup() {
  Serial.begin(115200);

  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  ESP_ERROR_CHECK( esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M));    

  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

// init i2s default rates
   i2s_driver_install(I2SR, &i2s_configR,0,NULL);
   i2s_set_pin(I2SR, &pin_configR);
   i2s_set_clk(I2SR, 44100, (i2s_bits_per_sample_t)16, (i2s_channel_t)1);
   i2s_zero_dma_buffer(I2SR);

  // power enable
  gpio_reset_pin(PW);
  gpio_set_direction(PW, GPIO_MODE_OUTPUT);      
  gpio_set_level(PW, 1); 

  
  gpio_reset_pin(GAIN);
  gpio_set_direction(GAIN, GPIO_MODE_OUTPUT);  
 // gpio_set_pull_mode(GAIN, GPIO_PULLDOWN_ONLY);   // 15dB   
  gpio_set_level(GAIN, 0);      // 12dB
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  char b[250];
  size_t t;

  i2s_write(I2SR, data, data_len, &t, 1000);
}

void loop() {
delay(10);
}
