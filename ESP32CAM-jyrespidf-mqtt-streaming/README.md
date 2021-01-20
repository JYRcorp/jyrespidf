# ESP32CAM wifi mqtt http : video surveillance
Inspiration prise ici : https://github.com/ldab/ESP32-CAM-MQTT
 mais ce projet utilise le framework arduino qui a une grosse limitation
 sur le buffer size des images publiées en MQTT,
 à cause du malloc() qui n'utilise pas la Flash RAM de l'ESP32-cam.

Voici ma réécriture avec PlatformIO et le framework esp-idf.
L'idée est de faire fonctionner sur une ESP32-CAM :
 - le wifi en adresse IP fixe
 - l'abonnement sur MQTT pour la configuration de l'ESP32-CAM (utiliser [JSON à importer dans node-red](for_node-red.json) )
 - la publication sur MQTT de photos capturées par la camera
 - l'activation du serveur HTTP pour afficher l'image de la camera
 - le streaming video de la camera
 - le stockage de l'adresse IP en NVS pour uploader le même code partout
 - la mise à jour du code par OTA déclenchée avec MQTT_TOPIC_CONFIG

Adapter la configuration de l'ESP32-CAM à flasher :
```
#define ESP_WIFI_SSID             "ssid"
#define ESP_WIFI_PASS             "password"
#define IP_ADDRESS_PREFIX         "192.168.1."  // la dernière valeur est dans NVS (IP_number) modifiable par MQTT_TOPIC_CONFIG
#define DEFAULT_IP_NUMBER          20
#define GATEWAY                   "192.168.1.1"
#define NETMASK                   "255.255.255.0"
#define OTA_FIRMWARE_UPGRADE_URL  "https://192.168.1.136:8070/firmware.bin" 
#define OTA_VERSION               "v1.2 du 2021/01/20 15:24"
#define MQTT_BROKER_URL           "mqtt://192.168.1.136"
#define MQTT_USERNAME             "mqtt_username"
#define MQTT_PASSWORD             "mqtt_password"
#define MQTT_TOPIC_PREFIX         "ESP32_CAM_"
#define ESP_MAXIMUM_RETRY          5
```

Pour l'utilisation d'OTA (voir le détail : https://github.com/espressif/esp-idf/tree/master/examples/system/ota), j'utilise "Windows Subsystem for Linux" :
```
cd jyrespidf/ESP32CAM-jyrespidf-mqtt-streaming/.pio/build/esp32cam
openssl req -x509 -newkey rsa:2048 -keyout ca_key.pem -out ca_cert.pem -days 365 -nodes
cp ca_cert.pem   jyrespidf/ESP32CAM-jyrespidf-mqtt-streaming/server_certs/
openssl s_server -WWW -key ca_key.pem -cert ca_cert.pem -port 8070
```

**Attention**, l'ajout de la librairie esp32-camera ne fonctionne pas correctement sous PlatformIO.
Le problème se situe au niveau de la génération du fichier sdkconfig qui ne conserve pas les informations
de la caméra au moment du build.
La solution est de ne pas installer la librairie au travers de PlatformIO, mais de le faire manuellement :

```
cd C:\Users\<user-account>\.platformio\packages\framework-espidf\components\
git clone https://github.com/espressif/esp32-camera.git
```

Lors du build dans PlatformIO, si les paramètres suivants ne sont pas corrects dans sdkconfig alors cela ne fonctionnera pas :
```
#
# Camera configuration
#
# CONFIG_OV7670_SUPPORT is not set
# CONFIG_OV7725_SUPPORT is not set
# CONFIG_NT99141_SUPPORT is not set
CONFIG_OV2640_SUPPORT=y
# CONFIG_OV3660_SUPPORT is not set
# CONFIG_OV5640_SUPPORT is not set
# CONFIG_SCCB_HARDWARE_I2C_PORT0 is not set
CONFIG_SCCB_HARDWARE_I2C_PORT1=y
CONFIG_CAMERA_CORE0=y
# CONFIG_CAMERA_CORE1 is not set
# CONFIG_CAMERA_NO_AFFINITY is not set
# end of Camera configuration
```

