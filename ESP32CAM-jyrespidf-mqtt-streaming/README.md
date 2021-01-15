# ESP32CAM wifi mqtt http : video surveillance
Inspiration prise ici : https://github.com/ldab/ESP32-CAM-MQTT
 mais cela utilise le framework arduino qui a une grosse limitation
 sur le buffer size des images publiées en MQTT
 à cause du malloc() qui n'utilise pas la Flash RAM de 4Mo de l'ESP32-cam.

Voici ma réécriture avec PlatformIO et le framework esp-idf.
L'idée est de faire fonctionner sur une ESP32-CAM :
 - le wifi en adresse IP fixe
 - l'abonnement sur MQTT pour la configuration de l'ESP32-CAM (utiliser [JSON à importer dans node-red](for_node-red.json) )
 - la publication sur MQTT de photos capturées par la camera
 - l'activation du serveur HTTP afficher l'image de la camera
 - le streaming video de la camera

Modifier la configuration de l'ESP32-CAM à flasher :
```
#define ESP_WIFI_SSID      "ssid"
#define ESP_WIFI_PASS      "password"
#define ESP_MAXIMUM_RETRY  5
#define IP_ADDRESS         "192.168.1.20"
#define GATEWAY            "192.168.1.1"
#define NETMASK            "255.255.255.0"
#define MQTT_BROKER_URL    "mqtt://192.168.1.136"
#define MQTT_USERNAME      "user_mqtt"
#define MQTT_PASSWORD      "pass_mqtt"
#define MQTT_TOPIC_BOOT    "ESP32_CAM_20/Boot"
#define MQTT_TOPIC_PHOTO   "ESP32_CAM_20/TakeAPicture"
#define MQTT_TOPIC_CONFIG  "ESP32_CAM_20/JSONConfig"
#define MQTT_TOPIC_PICTURE "ESP32_CAM_20/PICTURE"
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
