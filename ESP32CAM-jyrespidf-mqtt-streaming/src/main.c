#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>     /* malloc, free, rand */

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "esp_camera.h"
#include <esp_http_server.h>

#define ESP_WIFI_SSID      "ssid"
#define ESP_WIFI_PASS      "password"
#define IP_ADDRESS_PREFIX  "192.168.1."  // la dernière valeur est dans NVS (IP_number) modifiable par MQTT_TOPIC_CONFIG
#define DEFAULT_IP_NUMBER  20
#define GATEWAY            "192.168.1.1"
#define NETMASK            "255.255.255.0"
#define ESP_MAXIMUM_RETRY  5
#define MQTT_BROKER_URL    "mqtt://192.168.1.136"
#define MQTT_USERNAME      "username"
#define MQTT_PASSWORD      "password"
#define MQTT_TOPIC_PREFIX  "ESP32_CAM_"

#define GPIO_OUTPUT_FLASH_LED    4   // PIN commun avec les DATA du port série donc l'eclairage de la LED vascille
//#define GPIO_OUTPUT_FLASH_LED    3   //  (on peut toutefois bidouiller en supprimant R13 et y connecter le pin 3)
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_FLASH_LED))

static uint8_t IP_number=DEFAULT_IP_NUMBER;
static char IP_ADDRESS[20];
static char MQTT_TOPIC_BOOT[35];
static char MQTT_TOPIC_PHOTO[35];
static char MQTT_TOPIC_CONFIG[35];
static char MQTT_TOPIC_PICTURE[35];

static camera_config_t camera_config = {
  .ledc_channel = LEDC_CHANNEL_0,
  .ledc_timer   = LEDC_TIMER_0,
  .pin_d0       = 5,
  .pin_d1       = 18,
  .pin_d2       = 19,
  .pin_d3       = 21,
  .pin_d4       = 36,
  .pin_d5       = 39,
  .pin_d6       = 34,
  .pin_d7       = 35,
  .pin_xclk     = 0,
  .pin_pclk     = 22,
  .pin_vsync    = 25,
  .pin_href     = 23,
  .pin_sscb_sda = 26,
  .pin_sscb_scl = 27,
  .pin_pwdn     = 32,
  .pin_reset    = -1,
  .xclk_freq_hz = 20000000,
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size   = FRAMESIZE_UXGA, // taille de frame au maximum à l'initialisation (375ko par buffer)
  .jpeg_quality = 10,
  .fb_count     = 2               // c'est plus rapide avec 2 frames en mémoire (375ko x 2)
};

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static int motionDetection=1;
static int motionThrethold=50;
static int motionNbPhoto=3;
esp_mqtt_client_handle_t client;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// pour testMalloc
uint8_t* buffer;
size_t   bufferSize=0;
int testMalloc(size_t size);
void boucleTestMalloc();

//-------------------------------------------------------------------------------------------------
void write_nvs(nvs_handle_t handle, const char * key, uint8_t  value) {
    nvs_handle_t h;
    esp_err_t err;
    if (handle==-1) {
        err = nvs_open("storage", NVS_READWRITE, &h);
        if (err != ESP_OK) 
            printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
        h=handle;
    printf("Save nvs : %s=%d \n", key, value);
    err = nvs_set_u8(h, key, value);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    err = nvs_commit(h);
    printf((err != ESP_OK) ? "nvs_commit Failed!\n" : "nvs_commit Done\n");
    if (handle==-1) 
        nvs_close(h);
} 

//-------------------------------------------------------------------------------------------------
void init_globals(void) {
    IP_number=DEFAULT_IP_NUMBER;
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) 
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    else {
        err = nvs_get_u8(my_handle, "IP_number", &IP_number);
        switch (err) {
            case ESP_OK:
                printf("IP_number dans NVS=%d\n", IP_number);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("IP_number is not initialized yet!\n");
                IP_number=DEFAULT_IP_NUMBER;
                write_nvs(my_handle, "IP_number", IP_number);
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
                IP_number=DEFAULT_IP_NUMBER;
            }
        nvs_close(my_handle);
    }
    // La valeur en NVS identifie l'esp-cam: adresse IP + topics MQTT
    snprintf(IP_ADDRESS,         sizeof(IP_ADDRESS)+1,         "%s%d",              IP_ADDRESS_PREFIX, IP_number);
    snprintf(MQTT_TOPIC_BOOT,    sizeof(MQTT_TOPIC_BOOT)+1,    "%s%d/Boot",         MQTT_TOPIC_PREFIX, IP_number);
    snprintf(MQTT_TOPIC_PHOTO,   sizeof(MQTT_TOPIC_PHOTO)+1,   "%s%d/TakeAPicture", MQTT_TOPIC_PREFIX, IP_number);
    snprintf(MQTT_TOPIC_CONFIG,  sizeof(MQTT_TOPIC_CONFIG)+1,  "%s%d/JSONConfig",   MQTT_TOPIC_PREFIX, IP_number);
    snprintf(MQTT_TOPIC_PICTURE, sizeof(MQTT_TOPIC_PICTURE)+1, "%s%d/PICTURE",      MQTT_TOPIC_PREFIX, IP_number);
}

//-------------------------------------------------------------------------------------------------
void init_gpio(void) {
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_FLASH_LED, 0);
}

static int s_retry_num = 0;
//-------------------------------------------------------------------------------------------------
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            printf("retry to connect to the AP\n");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        printf("connect to the AP fail\n");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("\ngot ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

//-------------------------------------------------------------------------------------------------
void wifi_init(void)
{
    // FreeRTOS event group
    s_wifi_event_group = xEventGroupCreate();
    // stack TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // wifi & static IP
    esp_netif_t *my_sta = esp_netif_create_default_wifi_sta();
    esp_netif_dhcpc_stop(my_sta);
    esp_netif_ip_info_t ip_info;
    ip4addr_aton(IP_ADDRESS, (ip4_addr_t *) &ip_info.ip.addr);
    ip4addr_aton(GATEWAY,    (ip4_addr_t *) &ip_info.gw.addr);
    ip4addr_aton(NETMASK,    (ip4_addr_t *) &ip_info.netmask.addr);
    esp_netif_set_ip_info(my_sta, &ip_info);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    printf("wifi_init_sta finished.\n");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        printf("\nconnected to ap SSID:%s \n", ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        printf("Failed to connect to SSID:%s \n", ESP_WIFI_SSID);
    } else {
        printf("UNEXPECTED EVENT\n");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

//-------------------------------------------------------------------------------------------------
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    client = event->client;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("MQTT_EVENT_CONNECTED \n");
            esp_mqtt_client_publish(client, (const char *)MQTT_TOPIC_BOOT, "boot", 0, 0, 0);
            esp_mqtt_client_subscribe(client, MQTT_TOPIC_PHOTO, 0);
            esp_mqtt_client_subscribe(client, MQTT_TOPIC_CONFIG, 1);
            break;
        case MQTT_EVENT_DISCONNECTED:
            printf("MQTT_EVENT_DISCONNECTED \n");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            printf("MQTT_EVENT_SUBSCRIBED, msg_id=%d \n", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            printf("MQTT_EVENT_UNSUBSCRIBED, msg_id=%d \n", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            printf("MQTT_EVENT_PUBLISHED, msg_id=%d \n", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            printf("MQTT_EVENT_DATA \n");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            cJSON *root = cJSON_Parse(event->data);
            for (int i=0; i<cJSON_GetArraySize(root); i++) {
                printf("%s=%d \n",cJSON_GetArrayItem(root,i)->string, cJSON_GetArrayItem(root,i)->valueint);
                if (strcmp(cJSON_GetArrayItem(root,i)->string, "TakeAPicture")==0) {
                    camera_fb_t *fb = esp_camera_fb_get();
                    if (!fb) printf("Camera capture failed\n"); else {
                        printf("fb->len=%d \n",fb->len);
                        esp_mqtt_client_publish(client, MQTT_TOPIC_PICTURE, (const char*)(fb->buf), fb->len, 0, 0);
                        esp_camera_fb_return(fb);
                    }
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "framesize")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_framesize(s, cJSON_GetArrayItem(root,i)->valueint);
                }              
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "flash")==0) {
                    gpio_set_level(GPIO_OUTPUT_FLASH_LED, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "motionDetection")==0) {
                    motionDetection=cJSON_GetArrayItem(root,i)->valueint;
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "motionThrethold")==0) {
                    motionThrethold=cJSON_GetArrayItem(root,i)->valueint;
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "motionNbPhoto")==0) {
                    motionNbPhoto=cJSON_GetArrayItem(root,i)->valueint;
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "IP_number")==0) {
                    write_nvs(-1, "IP_number", cJSON_GetArrayItem(root,i)->valueint );
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "reboot")==0) {
                    esp_restart();
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "vflip")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_vflip(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "hmirror")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_hmirror(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "colorbar")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_colorbar(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "special_effect")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_special_effect(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "quality")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_quality(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "brightness")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_brightness(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "contrast")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_contrast(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "saturation")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_saturation(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "sharpness")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_sharpness(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "denoise")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_denoise(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "awb_gain")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_awb_gain(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else if (strcmp(cJSON_GetArrayItem(root,i)->string, "wb_mode")==0) {
                    sensor_t * s = esp_camera_sensor_get();
                    if (s!=NULL) s->set_wb_mode(s, cJSON_GetArrayItem(root,i)->valueint);
                }
                else 
                    printf("undefined: %s=%d \n",cJSON_GetArrayItem(root,i)->string, cJSON_GetArrayItem(root,i)->valueint);
            }
            break;
        case MQTT_EVENT_ERROR:
            printf("MQTT_EVENT_ERROR \n");
            break;
        default:
            printf("Other event id:%d \n", event->event_id);
            break;
    }
    return ESP_OK;
}

//-------------------------------------------------------------------------------------------------
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    printf("Event dispatched from event loop base=%s, event_id=%d \n", base, event_id);
    mqtt_event_handler_cb(event_data);
}


//-------------------------------------------------------------------------------------------------
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = MQTT_BROKER_URL,
        .username = MQTT_USERNAME,
        .password = MQTT_PASSWORD,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

//-------------------------------------------------------------------------------------------------
static esp_err_t camera_init()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)  {
        printf("\n=====> Camera Init Failed \n");
        return err;
    }
    // Après allocation du buffer maximum, on revient à une resolution moindre     // 1024x768
    sensor_t * s = esp_camera_sensor_get();
    if (s!=NULL) s->set_framesize(s, FRAMESIZE_XGA);

    return ESP_OK;
}

//--------------------------------------------------------------------------------------
static esp_err_t index_handler(httpd_req_t *req){
    static char index_response[32767];
    char * p = index_response;
    *p++ = ' ';
    p+=sprintf(p, "<html><head><title>ESP32-CAM jyrmqtt esp-idf</title>");
    p+=sprintf(p, "</head><body><table border=0>");

    p+=sprintf(p, "<tr><td><a href=/ >Home</a></td></tr>");
    p+=sprintf(p, "<tr><td><a href=/capture target=_new>Take photo</a></td></tr>");
    p+=sprintf(p, "<tr><td><a href=/stream  target=_new>Streaming</a></td></tr>");
    p+=sprintf(p, "<tr><td>Status:</td></tr>");
    p+=sprintf(p, "<tr><td>motionDetection=%d</td></tr>",motionDetection);
    p+=sprintf(p, "<tr><td>motionThrethold=%d</td></tr>",motionThrethold);
    p+=sprintf(p, "<tr><td>motionNbPhoto=%d</td></tr>",motionNbPhoto);

    p+=sprintf(p, "</table>");
    p+=sprintf(p, "</body></html>");
    *p++ = 0;
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_response, strlen(index_response));
}

//--------------------------------------------------------------------------------------
static esp_err_t capture_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;

    fb = esp_camera_fb_get();
    if (!fb) {
        printf("Camera capture failed\n");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // La photo en HTTP response
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

//--------------------------------------------------------------------------------------
static esp_err_t stream_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK)
        return res;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            printf("Camera capture failed\n");
            res = ESP_FAIL;
        } 
        else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK) {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if(res != ESP_OK){
            break;
        }
    }

    return res;
}

//-------------------------------------------------------------------------------------------------
void start_httpserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };
    httpd_uri_t capture_uri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };
   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    // Start the httpd server
    printf("Starting HTTP server on port: %d \n", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &capture_uri);
        httpd_register_uri_handler(server, &stream_uri);
    }
    else
        printf("Error starting HTTP server! \n");
}

//-------------------------------------------------------------------------------------------------
void start_motionDetection(void) {
    // attente 5 sec après reboot
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    int32_t current_frame = 0;
    int32_t prev_frame = 0;
    int firstTime = 0;
    int take_nb_photo=0;
    
    while(true){
        if (motionDetection) {
            camera_fb_t * fb =esp_camera_fb_get();
            if (fb) {
                current_frame = 0;
                for (uint32_t i = 0; i < fb->len; i++) {
                    current_frame += fb->buf[i];
                }
                // compare to previous frame (sauf les 3 premières apres boot)
                if (firstTime>3) {
                    float f=(float)prev_frame/(float)current_frame;
                    if ( f > (1+(motionThrethold/100.0)) || f < (1-(motionThrethold/100.0)) ) {
                        printf("Motion Detected...%d <> %d (%f)\n", prev_frame, current_frame, f );
                        take_nb_photo=motionNbPhoto;
                    }
                }
                else
                    firstTime++;
                // serie de photos après motion detection
                if (take_nb_photo>0) {
                    take_nb_photo--;
                    // send image through MQTT
                    esp_mqtt_client_publish(client, MQTT_TOPIC_PICTURE, (const char*)(fb->buf), fb->len, 0, 0);
                    printf("take_nb_photo=%d \n",take_nb_photo);
                }
                // current to prev frame
                prev_frame = current_frame;
                esp_camera_fb_return(fb);
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
            else
                vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        else
            vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

//-------------------------------------------------------------------------------------------------
void app_main() {
    // init GPIO pour la Flash LED
    init_gpio();
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // init variables depuis NVS
    init_globals();
    //Initialize wifi
    wifi_init();
    // start MQTT
    mqtt_app_start();
    //Initialize camera
    camera_init();
    // HTTP streaming
    start_httpserver();
    // motion detection
    start_motionDetection();

    //vTaskDelay(3000 / portTICK_PERIOD_MS);
    //boucleTestMalloc();   //  --> 3,3 Mo maximum testé
}






//-------------------------------------------------------------------------------------------------
int testMalloc(size_t size) {
    if (size == 0) {
        return 0;
    }
    if (bufferSize == 0) {
        buffer = (uint8_t*)malloc(size);
    } else {
        uint8_t* newBuffer = (uint8_t*)realloc(buffer, size);
        if (newBuffer != NULL) {
            buffer = newBuffer;
        } else {
            printf("realloc(%d) KO \n", size);
            return 0;
        }
    }
    bufferSize = size;
    return (buffer != NULL);
}

//-------------------------------------------------------------------------------------------------
void boucleTestMalloc() {
  for (int i=50000; i<5000000; i+=50000) {
    printf("%d\n",i);
      if (!(testMalloc(i))) {
              printf("=============>   malloc KO \n");
              break;
      }
  }
}
