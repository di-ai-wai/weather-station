///////////////////////////////////////////////////////////////////////////
//   SETTINGS

// Settings for WiFi
#define           WIFI_SSID           "<MySSID>"
#define           WIFI_PASS           "<secret>"

// Settings for MQTT
#define           MQTT_USER           "<MQTT-User>"
#define           MQTT_PASS           "<secret>"
#define           MQTT_SERVER         "<url.to.mqtt.server>"
#define           MQTT_PORT           "1883"
#define           MQTT_FINGERPRINT    ""

// board properties
#define           PIN_LED                   LED_BUILTIN //D4

// define serial interface pins for particle sensors
// Serial confusion: These definitions are based on SoftSerial
// TX (transmitting) pin on one side goes to RX (receiving) pin on other side
// SoftSerial RX PIN is D1 and goes to SDS TX
// SoftSerial TX PIN is D2 and goes to SDS RX
#if defined(ESP8266)
#define PM_SERIAL_RX D1
#define PM_SERIAL_TX D2
#endif

// define pins for I2C
#define I2C_PIN_SCL D4
#define I2C_PIN_SDA D3

// SDS011, der etwas teuerere Feinstaubsensor
#define SDS_READ 1
#define SDS_API_PIN 1

// DHT22, temperature, humidity
#define DHT_READ 1
#define DHT_TYPE DHT22
#define DHT_PIN 7

// Texte
//const char DBG_TXT_SENDING_TO_LUFTDATEN[] PROGMEM = "## Sending to Luftdaten.info ";

// TLS support, make sure to edit the fingerprint
//#define           TLS
#define           DEBUG                       // enable debugging
///////////////////////////////////////////////////////////////////////////