/* ESPRESSIF MIT License
 * 
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 * 
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * CAMERA + PROCESS + OUTPUT
 *      CONFIG_FACENET_INPUT + CONFIG_FACENET_PROCESS + CONFIG_FACENET_OUTPUT
 *
 * camera_single:
 *      app_main.c
 *      app_camera.c/h
 *      app_facenet.c/h
 */
#include "sdkconfig.h"
#include "app_camera.h"
#include "app_facenet.h"


#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "driver/uart.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "esp_event_loop.h"
#include "esp_http_server.h"

#include "config.h"


#include "quirc.h"
#include "qr_recoginize.h"

static const char* TAG = "camera";
#define CAM_USE_WIFI

#define ESP_WIFI_SSID "M5Psram_Cam"
#define ESP_WIFI_PASS ""
#define MAX_STA_CONN  1

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
SemaphoreHandle_t print_mux = NULL;
static EventGroupHandle_t s_wifi_event_group;
esp_err_t jpg_httpd_handler(httpd_req_t *req);
esp_err_t jpg_stream_httpd_handler(httpd_req_t *req);
static ip4_addr_t s_ip_addr;
const int CONNECTED_BIT = BIT0;
static httpd_handle_t server;
static httpd_uri_t jpeg_uri = {
        .uri = "/jpg",
        .method = HTTP_GET,
        .handler = jpg_httpd_handler,
        .user_ctx = NULL
    };

static httpd_uri_t jpeg_stream_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = jpg_stream_httpd_handler,
        .user_ctx = NULL
};
static uint8_t function_flag = 'u';
static uint8_t last_function_flag = 'u';
static uint8_t cam_wifi_flag = false;
static uint8_t cam_face_flag = false;
//static uint8_t cam_wifi_flag = false;
static const int RX_BUF_SIZE = 1024;
static void wifi_init_softap();
static esp_err_t http_server_init();

static void uart_msg_task(void *pvParameters);
static void uart_init();
extern void led_brightness(int duty);
extern mtmn_config_t init_config();

camera_config_t config;
void app_camera_init()

{

   // camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;

    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = CAM_PIN_D0;

    config.pin_d1 = CAM_PIN_D1;

    config.pin_d2 = CAM_PIN_D2;

    config.pin_d3 = CAM_PIN_D3;

    config.pin_d4 = CAM_PIN_D4;

    config.pin_d5 = CAM_PIN_D5;

    config.pin_d6 = CAM_PIN_D6;

    config.pin_d7 = CAM_PIN_D7;

    config.pin_xclk = CAM_PIN_XCLK;

    config.pin_pclk = CAM_PIN_PCLK;

    config.pin_vsync = CAM_PIN_VSYNC;

    config.pin_href = CAM_PIN_HREF;

    config.pin_sscb_sda = CAM_PIN_SIOD;

    config.pin_sscb_scl = CAM_PIN_SIOC;

    config.pin_pwdn = PWDN_GPIO_NUM;

    config.pin_reset = CAM_PIN_RESET;//RESET_GPIO_NUM;

    config.xclk_freq_hz = CAM_XCLK_FREQ;
      // s->set_pixformat(s, PIXFORMAT_JPEG);
                 //  s->set_framesize(s, FRAMESIZE_QVGA);
    // f
    config.pixel_format = PIXFORMAT_JPEG;
    //config.frame_size = FRAMESIZE_SVGA;
    config.frame_size = FRAMESIZE_HQVGA;
//FRAMESIZE_VGA
    //FRAMESIZE_HQVGA
    //FRAMESIZE_QVGA
    //q
    //config.pixel_format = PIXFORMAT_GRAYSCALE;//YUV422,GRAYSCALE,RGB565,JPEG
    //config.frame_size = FRAMESIZE_QQVGA;//QQVGA-UXGA Do not use sizes above QVGA when

    config.jpeg_quality = 15;
    config.fb_count = 2;



    // camera init

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK) {

        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        for(;;){
            vTaskDelay(10);
        }
        return;
    } else {
        led_brightness(20);
    }
}

void Gray(camera_fb_t * fb, uint8_t * gry_buf)
{
	if((fb == NULL) || (gry_buf == NULL)){
        return;
    }
    
    memset(gry_buf,0,160 * 120 * 3);
   // fb->format

    fmt2rgb888(fb->buf, fb->len, fb->format, gry_buf);
        
    //灰度化
    for(int i = 0; i < 160 * 120; i++){
      gry_buf[i] = 0.299 * gry_buf[3*i] + 0.578* gry_buf[3*i + 1] + 0.114 * gry_buf[3*i + 2];
    }
}

void app_main(void)
{

    esp_log_level_set("wifi", ESP_LOG_INFO);
    
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }
    wifi_init_softap();
    //config.pixel_format = PIXFORMAT_JPEG;
    //config.frame_size = FRAMESIZE_HQVGA;
    app_camera_init();

    uart_init();
    xTaskCreatePinnedToCore(uart_msg_task, "uart_task", 3 * 1024, NULL, 5, NULL, tskIDLE_PRIORITY);


    size_t frame_num = 0;
    dl_matrix3du_t *image_matrix = NULL;
    //camera_fb_t *fb = NULL;
    mtmn_config_t mtmn_config = init_config();
     
    char tx_buffer[200] = { '\0' };
    uint32_t data_len = 0;
    
    tx_buffer[0] = 0xFF;
    tx_buffer[1] = 0xD8;
    tx_buffer[2] = 0xEA;
    tx_buffer[3] = 0x01;
    sensor_t *s = esp_camera_sensor_get();
    //s->set_pixformat(s, PIXFORMAT_JPEG);

   //uint8_t * rgb_buf = malloc(sizeof(uint8_t) * 160 * 120 * 3);
    //if(rgb_buf == NULL){
       // ESP_LOGE(TAG, "malloc failed");
        //res = ESP_FAIL;
    //}
    //s->set_pixformat(s, PIXFORMAT_JPEG);
    //s->set_framesize(s, FRAMESIZE_HQVGA);

   // config.pixel_format = PIXFORMAT_JPEG;
   // config.frame_size = FRAMESIZE_HQVGA;
#ifdef FISH_EYE_CAM
    // flip img, other cam setting view sensor.h
    //sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#endif
#if 0
    print_mux = xSemaphoreCreateMutex();
    app_facenet_main();

#ifdef CAM_USE_WIFI
    wifi_init_softap();

    vTaskDelay(100 / portTICK_PERIOD_MS);
    http_server_init();
#endif
#else

 camera_fb_t * fb = NULL;
   //app_facenet_main();
   while(true){
    //uart_msg_task(NULL); 
    //printf("function_flag = %c\r\n",function_flag);
    if(last_function_flag != function_flag){
        last_function_flag = function_flag;
     if((function_flag == 'q') || (function_flag == 'Q')){
      // s->reset(s);
      // s->init_status(s);
      // s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
        s->set_framesize(s, FRAMESIZE_QQVGA);
       //s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
      // s->set_quality(s, 20);
       
       
     //PIXFORMAT_RGB565
     //PIXFORMAT_RGB565,    // 2BPP/RGB565
    //PIXFORMAT_YUV422,    // 2BPP/YUV422
    //PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
    //PIXFORMAT_JPEG,      // JPEG/COMPRESSED
    //PIXFORMAT_RGB888,    // 3BPP/RGB888
    //PIXFORMAT_RAW,       // RAW
    //PIXFORMAT_RGB444,    // 3BP2P/RGB444
    //PIXFORMAT_RGB555,    // 3BP2P/RGB555
    }else{
       
       s->set_framesize(s, FRAMESIZE_HQVGA);
      // s->set_pixformat(s, PIXFORMAT_JPEG);
      // esp_camera_deinit();
       //config.pixel_format = PIXFORMAT_JPEG;
       //config.frame_size = FRAMESIZE_HQVGA;
       //app_camera_init();
       
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    //if((function_flag != 'w') && (function_flag != 'W'))
      //httpd_stop(server);
    switch(function_flag){
      case 'u':
      case 'U':
        fb = esp_camera_fb_get();
       
        tx_buffer[4] = (uint8_t)((fb->len & 0xFF0000) >> 16) ;
        tx_buffer[5] = (uint8_t)((fb->len & 0x00FF00) >> 8 ) ;
        tx_buffer[6] = (uint8_t)((fb->len & 0x0000FF) >> 0 );

    
        uart_write_bytes(UART_NUM_1, (char *)tx_buffer, 7);
        uart_write_bytes(UART_NUM_1, (char *)fb->buf, fb->len);
        
        data_len =(uint32_t)(tx_buffer[4] << 16) | (tx_buffer[5] << 8) | tx_buffer[6];
        printf("should %d, print a image, len: %d\r\n",fb->len, data_len);

        esp_camera_fb_return(fb);
        if(cam_wifi_flag){
          cam_wifi_flag = false;
          ESP_ERROR_CHECK(httpd_unregister_uri_handler(server, jpeg_uri.uri, jpeg_uri.method));
          ESP_ERROR_CHECK(httpd_unregister_uri_handler(server, jpeg_stream_uri.uri, jpeg_stream_uri.method));
          httpd_stop(server);
        }
        break;

        case 'w':
        case 'W':
#ifdef CAM_USE_WIFI
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if(cam_wifi_flag) break;
    //wifi_init_softap();
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
    http_server_init();
    //msleep(1000);
    cam_wifi_flag = true;
#endif
    break;
    case 'q':
    case 'Q':

   //  if(cam_wifi_flag){
       //   cam_wifi_flag = false;
        //  ESP_ERROR_CHECK(httpd_unregister_uri_handler(server, jpeg_uri.uri, jpeg_uri.method));
        // ESP_ERROR_CHECK(httpd_unregister_uri_handler(server, jpeg_stream_uri.uri, jpeg_stream_uri.method));
         // httpd_stop(server);
        //  http_server_init();
    //msleep(1000);
        
   // }
    //http_server_init();
    //msleep(1000);
    //cam_wifi_flag = true;

      fb = esp_camera_fb_get();
      if(!fb) {
         ESP_LOGE(TAG, "Camera capture failed");
      } 
      //Gray(fb, &rgb_buf);

      //image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
      //uint32_t re = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
      // // if (true != re)
       // {
        //    ESP_LOGE(TAG, "fmt2rgb888 failed, fb: %d", fb->len);
            //dl_matrix3du_free(image_matrix);
          //  continue;
       // }
       //printf("sizeof(uint8_t) = %d\r\n",sizeof(uint8_t));
     // uint8_t *buf  = malloc(sizeof(uint8_t) * 160 * 120 * 3);
     // uint8_t * rgb_buf = malloc(sizeof(uint8_t) * 160 * 120);

     // memset(buf, 0,160 * 120 * 3);
   // fb->format

     // fmt2rgb888(fb->buf, fb->len, fb->format, buf);
        
    //灰度化
      //for(int i = 0; i < 19200; i++){
       // rgb_buf[i] = 0.299 * buf[3*i] + 0.578* buf[3*i + 1] + 0.114 * buf[3*i + 2];
      //}
      //free(buf);
     // qr_recoginze(fb, rgb_buf);
      qr_recoginze(fb, NULL);
     //qr_recoginze(void *pdata, char *buf) 
      //free(rgb_buf);
      esp_camera_fb_return(fb);
       //dl_matrix3du_free(image_matrix);
      vTaskDelay(10 / portTICK_PERIOD_MS);

      
     break;
    case 'f':
    case 'F':
/*
     if(cam_wifi_flag){
          cam_wifi_flag = false;
          ESP_ERROR_CHECK(httpd_unregister_uri_handler(server, jpeg_uri.uri, jpeg_uri.method));
          ESP_ERROR_CHECK(httpd_unregister_uri_handler(server, jpeg_stream_uri.uri, jpeg_stream_uri.method));
          httpd_stop(server);
        }
        */

      fb = esp_camera_fb_get();
      if (!fb){
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }
      image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
      uint32_t res = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
        if (true != res)
        {
            ESP_LOGE(TAG, "fmt2rgb888 failed, fb: %d", fb->len);
            dl_matrix3du_free(image_matrix);
            continue;
        }

        esp_camera_fb_return(fb);
        
        box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
        if (net_boxes)
        {
            frame_num++;
            printf("FACE FOUND:(%d,%d)\r\n",(int)(net_boxes->box->box_p[0] + net_boxes->box->box_p[2])/2,(int)(net_boxes->box->box_p[1] + net_boxes->box->box_p[3])/2);
            //ESP_LOGI(TAG, "net_boxes->len = %d\n",net_boxes->len);
            free(net_boxes->box);
            free(net_boxes->landmark);
            free(net_boxes);
        }

        dl_matrix3du_free(image_matrix);

       
     break;
     default:break;
     }
    
    }

    //app_facenet_main();
//}
#endif
}

//}




#ifdef CAM_USE_WIFI
esp_err_t jpg_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t fb_len = 0;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    res = httpd_resp_set_type(req, "image/jpeg");
    if(res == ESP_OK){
        res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    }

    if(res == ESP_OK){
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    ESP_LOGI(TAG, "JPG: %uKB %ums", (uint32_t)(fb_len/1024), (uint32_t)((fr_end - fr_start)/1000));
    return res;
}
 

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

   //uint8_t * rgb_buf = malloc(sizeof(uint8_t) * 160 * 120 * 3);
   // if(rgb_buf == NULL){
    //    ESP_LOGE(TAG, "malloc failed");
        //res = ESP_FAIL;
   // }
    while(true){
        //xSemaphoreTake(print_mux, portMAX_DELAY);
        fb = esp_camera_fb_get();
        //Gray(fb, rgb_buf);
       // printf("fb mode = %d",fb->format);
        //xSemaphoreGive(print_mux);
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
        } else {
            if(fb->format != PIXFORMAT_JPEG){
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                if(!jpeg_converted){
                    ESP_LOGE(TAG, "JPEG compression failed");
                    esp_camera_fb_return(fb);
                    res = ESP_FAIL;
                }
            } else {
              _jpg_buf_len = fb->len;
              _jpg_buf = fb->buf;
            }
        }
        
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(fb->format != PIXFORMAT_JPEG){
            free(_jpg_buf);
        }
        //qr_recoginze(fb);
        esp_camera_fb_return(fb);
        
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        //ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)",
            //(uint32_t)(_jpg_buf_len/1024),
            //(uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    }

    last_frame = 0;
    return res;
}

static esp_err_t http_server_init(){
    /*
    httpd_handle_t server;
    #if 1
    httpd_uri_t jpeg_uri = {
        .uri = "/jpg",
        .method = HTTP_GET,
        .handler = jpg_httpd_handler,
        .user_ctx = NULL
    };

    httpd_uri_t jpeg_stream_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = jpg_stream_httpd_handler,
        .user_ctx = NULL
    };
    #else
       httpd_uri_t jpeg_uri;
       jpeg_uri.uri = "/jpg";
       jpeg_uri.method = HTTP_GET;
       jpeg_uri .handler = jpg_httpd_handler;
       jpeg_uri .user_ctx = NULL;
    httpd_uri_t jpeg_stream_uri;mkae
       jpeg_stream_uri .uri = "/";
      jpeg_stream_uri  .method = HTTP_GET;
      jpeg_stream_uri  .handler = jpg_stream_httpd_handler;
      jpeg_stream_uri  .user_ctx = NULL;
 
    #endif
     */
    httpd_config_t http_options = HTTPD_DEFAULT_CONFIG();
    //!add
    //http_options.stack_size = 111500;

    ESP_ERROR_CHECK(httpd_start(&server, &http_options));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &jpeg_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &jpeg_stream_uri));

    return ESP_OK;
}
static esp_err_t event_handler(void* ctx, system_event_t* event) 
{
  switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
      s_ip_addr = event->event_info.got_ip.ip_info.ip;
      xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d", MAC2STR(event->event_info.sta_connected.mac),
               event->event_info.sta_connected.aid);
      xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d", MAC2STR(event->event_info.sta_disconnected.mac),
               event->event_info.sta_disconnected.aid);
      xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      esp_wifi_connect();
      xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    default:
      break;
  }
  return ESP_OK;
}

static void wifi_init_softap() 
{
  s_wifi_event_group = xEventGroupCreate();

  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t wifi_config = {
      .ap = {.ssid = ESP_WIFI_SSID,
             .ssid_len = strlen(ESP_WIFI_SSID),
             .password = ESP_WIFI_PASS,
             .max_connection = MAX_STA_CONN,
             .authmode = WIFI_AUTH_WPA_WPA2_PSK},
  };
  if (strlen(ESP_WIFI_PASS) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  uint8_t addr[4] = {192, 168, 4, 1};
  s_ip_addr = *(ip4_addr_t*)&addr;

  ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
           ESP_WIFI_SSID, ESP_WIFI_PASS);
}

#endif




static void uart_msg_task(void *pvParameters) {
  uint8_t rx_buffer;

  while(true) 
  {

       rx_buffer = getchar();
       if(rx_buffer < 255){
         if((rx_buffer == 'u') || (rx_buffer == 'U') ||
          (rx_buffer == 'w') || (rx_buffer == 'W') ||
          (rx_buffer == 'q') || (rx_buffer == 'Q') ||
          (rx_buffer == 'f') || (rx_buffer == 'F')){
            //last_function_flag = function_flag;
            function_flag = rx_buffer;
             //sensor_t *s = esp_camera_sensor_get();
             // if((rx_buffer == 'q') || (rx_buffer == 'Q')){
             //     s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
              //    s->set_framesize(s, FRAMESIZE_QQVGA);
             // }else{
             //      s->set_pixformat(s, PIXFORMAT_JPEG);
              //     s->set_framesize(s, FRAMESIZE_HQVGA);
              //}
           //  config.pixel_format = PIXFORMAT_JPEG;
           // config.frame_size = FRAMESIZE_QVGA;
            /*
           // sensor_t *s = esp_camera_sensor_get();
            if((rx_buffer == 'u') || (rx_buffer == 'U')){
                  s->set_pixformat(s, PIXFORMAT_JPEG);
                  s->set_framesize(s, FRAMESIZE_HQVGA);
            }else if((rx_buffer == 'w') || (rx_buffer == 'W')){
                  s->set_pixformat(s, PIXFORMAT_JPEG);
                  s->set_framesize(s, FRAMESIZE_HQVGA);
            }else if((rx_buffer == 'q') || (rx_buffer == 'Q')){
                  s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
                  s->set_framesize(s, FRAMESIZE_QQVGA);
            }else if((rx_buffer == 'f') || (rx_buffer == 'F')){
                   s->set_pixformat(s, PIXFORMAT_JPEG);
                   s->set_framesize(s, FRAMESIZE_QVGA);
            }    */
            printf("%c ok\r\n",rx_buffer);
         }
         else{
          printf("error\r\n");
         }
         //printf("rx_buffer = %c\r\n",rx_buffer);
       }
       vTaskDelay(10 / portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

static void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);

    uart_set_pin(UART_NUM_1, CAM_PIN_TX, CAM_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}