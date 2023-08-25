


#include "esp_now.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "driver/i2s.h"
#include "freertos/ringbuf.h"

#define I2S_DOUT      26
#define I2S_BCLK      5
#define I2S_LRC       25
#define I2S_DIN       35
#define I2SR (i2s_port_t)0
//Amp power enable
#define PW GPIO_NUM_21     
#define GAIN GPIO_NUM_23  

 #define bSize 480  

    wifi_scan_config_t config;
    wifi_ap_record_t *rec;
    wifi_ap_record_t *r;  

    uint8_t b[4000];
    uint16_t n_AP;
    int n=0;
    int i;
    int j;
    int l;
    int sendOK ;
    esp_err_t ret;
    size_t t;
    esp_now_peer_info_t slave;
    FILE* f;
    uint8_t mac [6];
    char macStr[20];
    char dev_name [30];
    static uint32_t m_pkt_cnt = 0;   
    static esp_a2d_audio_state_t m_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;    
    uint32_t sampleRate;

uint8_t L[bSize/2];
uint8_t R[bSize/2];

    

 const i2s_config_t i2s_configR = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX ), //  transfer
      .sample_rate = 44100,                  
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,       // 
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT ,       //
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,           // Interrupt level 1
      .dma_buf_count = 8,                                 // number of buffers
      .dma_buf_len = 1024                                 // samples per buffer
  };
  
      i2s_pin_config_t pin_configR=
      {
      .bck_io_num = I2S_BCLK,                             // BCKL
      .ws_io_num = I2S_LRC ,                              // LRCL
      .data_out_num = I2S_DOUT,                           // DOUT
      .data_in_num = I2S_DIN                              // DIN
      };
#define CHANNEL 1

RingbufHandle_t RingBuf;


// esp_now_send callback routine
// status is kept to allow a retry in case of error
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
static int n = 0;  
if(status == ESP_OK) sendOK = 1;
else    
{   
sendOK = -1;
printf("Retry no %d\n", n++);
}

}

//Send an esp-now buffer handling errors
void sendNow(uint8_t* p, uint8_t* b, size_t t)
{
  sendOK = 0;
  esp_now_send(p, b, t);
 /* 
  while(sendOK != 1)
  {
    delay(1);
    if(sendOK == -1)
    {
      sendOK = 0;
      esp_now_send(p, b, t);
    }
  }
  */
}
static void splitChannelsAndSend(void* data)
{
//static int N = 0;
//static int T = 0;
size_t t;
int i, j;
//printf("SplitAndSend\n");
while(1)
{
//get record
uint8_t* item = (uint8_t*)xRingbufferReceive(RingBuf, &t, portMAX_DELAY);

//audio data

//separate channels

  for(i=0; i<t/2; i+=2)
  {
    L[i] = item[i+i];
    L[i+1] = item[i+i+1];
    R[i] = item[i+i+2];
    R[i+1] = item[i+i+3];
  }

  vRingbufferReturnItem(RingBuf, (void *)item);

  sendNow(slave.peer_addr,R, t/2);
  vTaskDelay(2/portTICK_PERIOD_MS);  // difficulty with this delay.... (< 10ms)
  i2s_write(I2SR, L, t/2, &t, portMAX_DELAY );
}
}

void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
uint32_t l;
//printf("11\n");
size_t t;
uint8_t* d;
l = len;
d = (uint8_t*)data;
while(l>0)
{
if(l > bSize) 
  {
  l= l - bSize;
  t = bSize;
  }
else
  {
  t = l;
  l = 0;
  }
 xRingbufferSend(RingBuf, (void*)d, t, (TickType_t) portMAX_DELAY);
 d = d + t;
}   
}

 // callback for A2DP sink
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *p_param)
{
  ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
  esp_a2d_cb_param_t *a2d = NULL;
  switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "a2dp conn_state_cb, state %d", a2d->conn_stat.state);
        break;
      }
      a2d = (esp_a2d_cb_param_t *)(p_param);
      ESP_LOGI(BT_AV_TAG, "a2dp audio_state_cb state %d", a2d->audio_stat.state);
      m_audio_state = a2d->audio_stat.state;
      if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
        m_pkt_cnt = 0;
      case ESP_A2D_AUDIO_STATE_EVT: {

        }
        break;
      }
    case ESP_A2D_AUDIO_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "a2dp audio_cfg_cb , codec type %d", a2d->audio_cfg.mcc.type);
        // for now only SBC stream is supported
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
          int sample_rate = 16000;
          char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
          if (oct0 & (0x01 << 6)) {
            sample_rate = 32000;
          } else if (oct0 & (0x01 << 5)) {
            sample_rate = 44100;
          } else if (oct0 & (0x01 << 4)) {
            sample_rate = 48000;
          }
          printf("sampleRate : %d\n",sample_rate);
//          sampleRate = sample_rate;
//          i2s_set_clk(I2SN, sample_rate, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);

          ESP_LOGI(BT_AV_TAG, "configure audio player %x-%x-%x-%x\n",
                   a2d->audio_cfg.mcc.cie.sbc[0],
                   a2d->audio_cfg.mcc.cie.sbc[1],
                   a2d->audio_cfg.mcc.cie.sbc[2],
                   a2d->audio_cfg.mcc.cie.sbc[3]);
          ESP_LOGI(BT_AV_TAG, "audio player configured, samplerate=%d", sample_rate);
        }
        break;
      }
    default:
      ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
      break;
  }
} 
void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param)
{
  esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
  uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);
  memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
  attr_text[rc->meta_rsp.attr_length] = 0;
  printf("================> %s\n", (char*) attr_text);

  rc->meta_rsp.attr_text = attr_text;
}
void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{


  switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
      bt_app_alloc_meta_buffer(param);
      break;
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
      esp_avrc_ct_send_metadata_cmd(0, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
      break;
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        //        bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t), NULL);
        break;
      }
    default:
      ESP_LOGE(BT_AV_TAG, "Invalid AVRC event: %d", event);
      break;
  }
}

  
void setup()
{
  Serial.begin(115200);
  Serial.println();

// Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

/////////////////////////////////////////////////////////////////////    
// Initialize WiFi   
////////////////////////////////////////////////////////////////////
 
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M));    

////////////////////////////////////////////////
//Finding out the peer (Slave with SSID == "Slave...")
///////////////////////////////////////////////

// scan all softAP / channel == 1
    config.channel = CHANNEL;
    config.show_hidden = true;
    config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    config.scan_time.active.min = 1000;
    config.scan_time.active.max = 1400;
    config.scan_time.passive = 1500;
    config.ssid = NULL;
    config.bssid = NULL;   
    ESP_ERROR_CHECK( esp_wifi_scan_start( &config, true));
// read softAP record  
    ESP_ERROR_CHECK( esp_wifi_scan_get_ap_num(&n_AP));
    printf("%d access points found\n", n_AP);
    
    rec = (wifi_ap_record_t *)malloc(n_AP * sizeof( wifi_ap_record_t ));
//    
    ESP_ERROR_CHECK(  esp_wifi_scan_get_ap_records(&n_AP, rec));
    
// find out "Slave_n" softAP  
    for(i=0; i<n_AP; i++)
    {
     wifi_ap_record_t *r;
     r = rec + i;
     printf("%s\n", (char*)r-> ssid);
     if(strcmp((char*)r-> ssid, "Slave_1") == 0)
     {
     for(j=0; j<6; j++) slave.peer_addr[j] = r->bssid[j];
     slave.channel = CHANNEL;
     slave.encrypt = 0;
     break;
     }  
     } 

// Initialize ESPNOW and register sending callback function. 
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    
// register slave 
     esp_err_t addStatus = esp_now_add_peer(&slave);
     printf("status = %x\n",addStatus);
     free(rec);



/////////////////////////////////////////////////////////////////////////
// init bluetooth : A2DP and AVRCP profiles
//
/////////////////////////////////////////////////////////////////////////

//init Classic BT mode
     btStart();
     ESP_ERROR_CHECK( esp_bluedroid_init());
     ESP_ERROR_CHECK( esp_bluedroid_enable());
printf("1\n");
//init BT device name => MUSE_SPEAKER-abcdef
     esp_read_mac((uint8_t*)&mac, ESP_MAC_WIFI_STA);
     sprintf(macStr,"-%2x%2x%2x", mac[3], mac[4], mac[5]);
     strcpy(dev_name, "MUSE_NOW");
     strcat(dev_name, macStr);
     esp_bt_dev_set_device_name(dev_name);
printf("2\n");     
//initialize A2DP sink
     esp_a2d_register_callback(&bt_app_a2d_cb);
     esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
     esp_a2d_sink_init();
printf("3\n");
//initialize AVRC controller
     esp_avrc_ct_init();
     esp_avrc_ct_register_callback(bt_app_rc_ct_cb);  
printf("4\n");
//set discoverable and connectable mode, wait to be connected
     esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
     esp_avrc_ct_send_metadata_cmd(0, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);  
printf("5\n");

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
  gpio_set_pull_mode(GAIN, GPIO_PULLDOWN_ONLY);   // 15dB 
/////////////////////////////////////////////////////////////////////
// Starting   app  
/////////////////////////////////////////////////////////////////////
RingBuf = xRingbufferCreate(20000,RINGBUF_TYPE_NOSPLIT);
//        printf("COREpp %d\n", xPortGetCoreID());
xTaskCreatePinnedToCore(splitChannelsAndSend, "splitChannelsAndSend", 5000, NULL,1,NULL,0);
}


void loop()
{  
delay(1);


}
