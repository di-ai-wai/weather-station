/***********************************************************************************
 * weather-station
 * 
 * All-in-One Weather Station based on a Nodemcu esp8266
 * providing the following sensor data
 * - temperature
 * - humidity
 * - air-pressure
 * - light
 * - luftdaten (particles)
 * - rain
 * - wind
 * 
 * using MQTT for message transmission
 * OTA-Update possibility activated by an MQTT message
 * standard system messages also transmitted voa MQTT
 * every device uses its own esp-device id for identificatuon purpose
 * 
 * used sensors:
 * - SDS011
 * 
 * for passwords, IPs and sensor-gpios please use the seperate config-file
 * 
 * 
  Libraries :
    - ESP8266 core for Arduino :  https://github.com/esp8266/Arduino
    - ESP8266httpUpdate:          auto update esp8266 sketch
    - PubSubClient:               https://github.com/knolleary/pubsubclient

************************************************************************************
*/

#include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino
#include <ESP8266httpUpdate.h>  // ota update for esp8266
#include <PubSubClient.h>       // https://github.com/knolleary/pubsubclient/releases/tag/v2.6
#include <Wire.h>
#include <SoftwareSerial.h>
#include "./DHT.h"
#include <Adafruit_BME280.h>
#include <time.h>
#include <coredecls.h>
#include "config.h"             // local config

// macros for debugging
#ifdef DEBUG
  #define         DEBUG_PRINT(x)    Serial.print(x)
  #define         DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define         DEBUG_PRINT(x)
  #define         DEBUG_PRINTLN(x)
#endif

#define           CONNECTED                 true
#define           NOT_CONNECTED             false

// Board properties
#define           FW_VERSION                "0.1.0"
#define           ALIAS                     "aio-weather-station"
#define           LOCATION                  "garden"
#define           CHIP                      "esp8266"

// MQTT
#define           MQTT_ON_PAYLOAD           "1"
#define           MQTT_OFF_PAYLOAD          "0"
#define           MQTT_ENDPOINT_SIZE        30

#define           MQTT_TOPIC_BASE           		""
#define           MQTT_TOPIC_REGISTRY       		"registry"
#define           MQTT_TOPIC_SYSTEM_VERSION 		"system/version"
#define           MQTT_TOPIC_SYSTEM_ALIAS   		"system/alias"
#define           MQTT_TOPIC_SYSTEM_CHIP    		"system/chip"
#define           MQTT_TOPIC_SYSTEM_LOC     		"system/location"
#define           MQTT_TOPIC_SYSTEM_UPDATE  		"system/update"
#define           MQTT_TOPIC_SYSTEM_RESET   		"system/reset"
#define           MQTT_TOPIC_SYSTEM_ONLINE  		"system/online"
#define           MQTT_TOPIC_SYSTEM_WIFI_SIGNAL_DB	"system/wifi/signaldb"
#define           MQTT_TOPIC_SYSTEM_WIFI_SIGNAL_PER	"system/wifi/signalper"
#define           MQTT_TOPIC_SYSTEM_WIFI_IP			"system/wifi/ip"

#define           MQTT_SENSOR_PM10           		"feinstaub/pm10"
#define           MQTT_SENSOR_PM25           		"feinstaub/pm25"
#define           MQTT_SENSOR_DHT_TEMP         		"feinstaub/dht/temp"
#define           MQTT_SENSOR_DHT_HUM         		"feinstaub/dht/hum"
#define           MQTT_SENSOR_BME_TEMP         		"feinstaub/bme/temp"
#define           MQTT_SENSOR_BME_HUM         		"feinstaub/bme/hum"
#define           MQTT_SENSOR_BME_PRESS        		"feinstaub/bme/press"

/******************************************************************
 * Constants                                                      *
 ******************************************************************/
//const unsigned long SAMPLETIME_MS = 30000;
//const unsigned long SAMPLETIME_SDS_MS = 1000;
const unsigned long WARMUPTIME_SDS_MS = 15000;
const unsigned long READINGTIME_SDS_MS = 5000;
const unsigned long ONE_DAY_IN_MS = 24 * 60 * 60 * 1000;
//const unsigned long PAUSE_BETWEEN_UPDATE_ATTEMPTS_MS = ONE_DAY_IN_MS;        // check for firmware updates once a day
const unsigned long DURATION_BEFORE_FORCED_RESTART_MS = ONE_DAY_IN_MS * 28;  // force a reboot every ~4 weeks


char              MQTT_CLIENT_ID[9]                               = {0};
char              MQTT_CHIP_VERSION[30]                           = {0};    
char              MQTT_ENDPOINT_REGISTRY[MQTT_ENDPOINT_SIZE]      = {0};
char              MQTT_ENDPOINT_SYS_VERSION[MQTT_ENDPOINT_SIZE]   = {0};
char              MQTT_ENDPOINT_SYS_ALIAS[MQTT_ENDPOINT_SIZE]     = {0};
char              MQTT_ENDPOINT_SYS_CHIP[MQTT_ENDPOINT_SIZE]      = {0};
char              MQTT_ENDPOINT_SYS_LOCATION[MQTT_ENDPOINT_SIZE]  = {0};
char              MQTT_ENDPOINT_SYS_UPDATE[MQTT_ENDPOINT_SIZE]    = {0};
char              MQTT_ENDPOINT_SYS_RESET[MQTT_ENDPOINT_SIZE]     = {0};
char              MQTT_ENDPOINT_SYS_ONLINE[MQTT_ENDPOINT_SIZE]    = {0};
char              MQTT_ENDPOINT_SYS_WIFI_SIGNAL_DB[MQTT_ENDPOINT_SIZE]  = {0};
char              MQTT_ENDPOINT_SYS_WIFI_SIGNAL_PER[MQTT_ENDPOINT_SIZE]	= {0};
char              MQTT_ENDPOINT_SYS_WIFI_IP[MQTT_ENDPOINT_SIZE]    = {0};

char              MQTT_ENDPOINT_SENSOR_PM10[MQTT_ENDPOINT_SIZE]       = {0};
char              MQTT_ENDPOINT_SENSOR_PM25[MQTT_ENDPOINT_SIZE]       = {0};
char              MQTT_ENDPOINT_SENSOR_DHT_TEMP[MQTT_ENDPOINT_SIZE]   = {0};
char              MQTT_ENDPOINT_SENSOR_DHT_HUM[MQTT_ENDPOINT_SIZE]    = {0};
char              MQTT_ENDPOINT_SENSOR_BME_TEMP[MQTT_ENDPOINT_SIZE]   = {0};
char              MQTT_ENDPOINT_SENSOR_BME_HUM[MQTT_ENDPOINT_SIZE]    = {0};
char              MQTT_ENDPOINT_SENSOR_BME_PRESS[MQTT_ENDPOINT_SIZE]  = {0};

char              MQTT_ENDPOINT_POWER[MQTT_ENDPOINT_SIZE]          = {0};


#ifdef TLS
WiFiClientSecure  wifiClient;
#else
WiFiClient        wifiClient;
#endif
PubSubClient      mqttClient(wifiClient);

/*****************************************************************
 * Configuration                                                 +
 *****************************************************************/

namespace cfg {
	char wlanssid[35] = WIFI_SSID;
	char wlanpwd[65] = WIFI_PASS;

	bool sds_read = SDS_READ;
	bool dht_read = DHT_READ;
	bool bme_read = BME_READ;

	unsigned long sending_intervall_ms = 300000;

	//int  debug = DEBUG;
}

struct Sds {
	double p10;
	double p25;
};

struct Dht {
	double temp;
	double hum;
};

struct Bme280 {
	double temp;
	double hum;
	double press;
};

unsigned long starttime;
unsigned long act_micro;
unsigned long act_milli;
unsigned long last_micro = 0;
unsigned long min_micro = 1000000000;
unsigned long max_micro = 0;
unsigned long sending_time = 0;

unsigned long time_point_device_start_ms;

long int sample_count = 0;

/*****************************************************************
 * SDS011 declarations                                           *
 *****************************************************************/
SoftwareSerial serialSDS(PM_SERIAL_RX, PM_SERIAL_TX, false, 128);

template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
	return N;
}

const char data_first_part[] PROGMEM = "{\"software_version\": \"{v}\", \"sensordatavalues\":[";

#define msSince(timestamp_before) (act_milli - (timestamp_before))

enum class PmSensorCmd {
	Start,
	Stop,
	ContinuousMode,
	VersionDate
};

bool is_SDS_running = true;
bool send_now = false;

int sds_pm10_sum = 0;
int sds_pm25_sum = 0;
int sds_val_count = 0;
int sds_pm10_max = 0;
int sds_pm10_min = 20000;
int sds_pm25_max = 0;
int sds_pm25_min = 20000;

String esp_chipid;

/*****************************************************************
 * DHT declaration                                               *
 *****************************************************************/
DHT dht(DHT_PIN, DHT_TYPE);
//DHT dht(DHT_PIN, DHT_TYPE);

/*****************************************************************
 * BME280 declaration                                            *
 *****************************************************************/
Adafruit_BME280 bme280;
bool bme_init_failed = false;

bool sntp_time_is_set = false;
bool got_ntp = false;

double last_value_SDS_P1 = -1.0;
double last_value_SDS_P2 = -1.0;
double last_value_DHT_T = -128.0;
double last_value_DHT_H = -1.0;
double last_value_BME280_T = -128.0;
double last_value_BME280_H = -1.0;
double last_value_BME280_P = -1.0;

// PREDEFINED FUNCTIONS
void reconnect();
void updateFW(char* fwUrl);
void reset();
void publishData(char* topic, char* payload);
void publishState(char* topic, int state);


///////////////////////////////////////////////////////////////////////////
//   MQTT with SSL/TLS
///////////////////////////////////////////////////////////////////////////
/*
  Function called to verify the fingerprint of the MQTT server certificate
 */
#ifdef TLS
void verifyFingerprint() {
  DEBUG_PRINT(F("INFO: Connecting to "));
  DEBUG_PRINTLN(MQTT_SERVER);

  if (!wifiClient.connect(MQTT_SERVER, atoi(MQTT_PORT))) {
    DEBUG_PRINTLN(F("ERROR: Connection failed. Halting execution"));
    reset();
  }

  if (wifiClient.verify(MQTT_FINGERPRINT, MQTT_SERVER)) {
    DEBUG_PRINTLN(F("INFO: Connection secure"));
  } else {
    DEBUG_PRINTLN(F("ERROR: Connection insecure! Halting execution"));
    reset();
  }
}
#endif

/*****************************************************************
 * convert float to string with a                                *
 * precision of two (or a given number of) decimal places        *
 *****************************************************************/
String Float2String(const double value, uint8_t digits) {
	// Convert a float to String with two decimals.
	char temp[15];

	dtostrf(value, 13, digits, temp);
	String s = temp;
	s.trim();
	return s;
}

String Float2String(const double value) {
	return Float2String(value, 2);
}

/*****************************************************************
 * convert value to json string                                  *
 *****************************************************************/
String Value2Json(const String& type, const String& value) {
	String s = F("{\"value_type\":\"{t}\",\"value\":\"{v}\"},");
	s.replace("{t}", type);
	s.replace("{v}", value);
	return s;
}


/*****************************************************************
 * send SDS011 command (start, stop, continuous mode, version    *
 *****************************************************************/
static bool SDS_cmd(PmSensorCmd cmd) {
	static constexpr uint8_t start_cmd[] PROGMEM = {
		0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB
	};
	static constexpr uint8_t stop_cmd[] PROGMEM = {
		0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB
	};
	static constexpr uint8_t continuous_mode_cmd[] PROGMEM = {
		0xAA, 0xB4, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x07, 0xAB
	};
	static constexpr uint8_t version_cmd[] PROGMEM = {
		0xAA, 0xB4, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB
	};
	constexpr uint8_t cmd_len = array_num_elements(start_cmd);

	uint8_t buf[cmd_len];
	switch (cmd) {
	case PmSensorCmd::Start:
		memcpy_P(buf, start_cmd, cmd_len);
		break;
	case PmSensorCmd::Stop:
		memcpy_P(buf, stop_cmd, cmd_len);
		break;
	case PmSensorCmd::ContinuousMode:
		memcpy_P(buf, continuous_mode_cmd, cmd_len);
		break;
	case PmSensorCmd::VersionDate:
		memcpy_P(buf, version_cmd, cmd_len);
		break;
	}
	serialSDS.write(buf, cmd_len);
	return cmd != PmSensorCmd::Stop;
}

/*****************************************************************
 * read SDS011 sensor values                                     *
 *****************************************************************/
Sds sensorSDS() {
	//String s = "";
	Sds result;
	char buffer;
	int value;
	int len = 0;
	int pm10_serial = 0;
	int pm25_serial = 0;
	int checksum_is = 0;
	int checksum_ok = 0;

	if (msSince(starttime) < (cfg::sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS))) {
		if (is_SDS_running) {
			is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
		}
	} else {
		if (! is_SDS_running) {
			is_SDS_running = SDS_cmd(PmSensorCmd::Start);
		}

		while (serialSDS.available() > 0) {
			buffer = serialSDS.read();
			//debug_out(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO, 1);
//			"aa" = 170, "ab" = 171, "c0" = 192
			value = int(buffer);
			switch (len) {
			case (0):
				if (value != 170) {
					len = -1;
				};
				break;
			case (1):
				if (value != 192) {
					len = -1;
				};
				break;
			case (2):
				pm25_serial = value;
				checksum_is = value;
				break;
			case (3):
				pm25_serial += (value << 8);
				break;
			case (4):
				pm10_serial = value;
				break;
			case (5):
				pm10_serial += (value << 8);
				break;
			case (8):
				//debug_out(FPSTR(DBG_TXT_CHECKSUM_IS), DEBUG_MED_INFO, 0);
				//debug_out(String(checksum_is % 256), DEBUG_MED_INFO, 0);
				//debug_out(FPSTR(DBG_TXT_CHECKSUM_SHOULD), DEBUG_MED_INFO, 0);
				//debug_out(String(value), DEBUG_MED_INFO, 1);
				if (value == (checksum_is % 256)) {
					checksum_ok = 1;
				} else {
					len = -1;
				};
				break;
			case (9):
				if (value != 171) {
					len = -1;
				};
				break;
			}
			if (len > 2) { checksum_is += value; }
			len++;
			if (len == 10 && checksum_ok == 1 && (msSince(starttime) > (cfg::sending_intervall_ms - READINGTIME_SDS_MS))) {
				if ((! isnan(pm10_serial)) && (! isnan(pm25_serial))) {
					sds_pm10_sum += pm10_serial;
					sds_pm25_sum += pm25_serial;
					if (sds_pm10_min > pm10_serial) {
						sds_pm10_min = pm10_serial;
					}
					if (sds_pm10_max < pm10_serial) {
						sds_pm10_max = pm10_serial;
					}
					if (sds_pm25_min > pm25_serial) {
						sds_pm25_min = pm25_serial;
					}
					if (sds_pm25_max < pm25_serial) {
						sds_pm25_max = pm25_serial;
					}
					/* 
					debug_out(F("PM10 (sec.) : "), DEBUG_MED_INFO, 0);
					debug_out(Float2String(double(pm10_serial) / 10), DEBUG_MED_INFO, 1);
					debug_out(F("PM2.5 (sec.): "), DEBUG_MED_INFO, 0);
					debug_out(Float2String(double(pm25_serial) / 10), DEBUG_MED_INFO, 1);
					*/
					sds_val_count++;
				}
				len = 0;
				checksum_ok = 0;
				pm10_serial = 0.0;
				pm25_serial = 0.0;
				checksum_is = 0;
			}
			yield();
		}

	}
	if (send_now) {
		last_value_SDS_P1 = -1;
		last_value_SDS_P2 = -1;
		if (sds_val_count > 2) {
			sds_pm10_sum = sds_pm10_sum - sds_pm10_min - sds_pm10_max;
			sds_pm25_sum = sds_pm25_sum - sds_pm25_min - sds_pm25_max;
			sds_val_count = sds_val_count - 2;
		}
		if (sds_val_count > 0) {
			last_value_SDS_P1 = double(sds_pm10_sum) / (sds_val_count * 10.0);
			last_value_SDS_P2 = double(sds_pm25_sum) / (sds_val_count * 10.0);
			/*
			debug_out("PM10:  " + Float2String(last_value_SDS_P1), DEBUG_MIN_INFO, 1);
			debug_out("PM2.5: " + Float2String(last_value_SDS_P2), DEBUG_MIN_INFO, 1);
			debug_out("----", DEBUG_MIN_INFO, 1);
			*/
			result = { last_value_SDS_P1, last_value_SDS_P2 };
			//s += Value2Json("SDS_P1", Float2String(last_value_SDS_P1));
			//s += Value2Json("SDS_P2", Float2String(last_value_SDS_P2));
		}
		sds_pm10_sum = 0;
		sds_pm25_sum = 0;
		sds_val_count = 0;
		sds_pm10_max = 0;
		sds_pm10_min = 20000;
		sds_pm25_max = 0;
		sds_pm25_min = 20000;
		if ((cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS))) {
			is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
		}
	}

	//debug_out(String(FPSTR(DBG_TXT_END_READING)) + FPSTR(SENSORS_SDS011), DEBUG_MED_INFO, 1);

	return result;
}

/*****************************************************************
 * read DHT11/DHT22 sensor values                                *
 *****************************************************************/
Dht sensorDHT() {
	//String s;
	//debug_out(String(FPSTR(DBG_TXT_START_READING)) + "DHT22", DEBUG_MED_INFO, 1);

	// Check if valid number if non NaN (not a number) will be send.
	last_value_DHT_T = -128;
	last_value_DHT_H = -1;

	sensors_event_t event;

	int count = 0;
	const int MAX_ATTEMPTS = 5;
	while ((count++ < MAX_ATTEMPTS) /* && (result == nullptr)*/) {
		auto h = dht.readHumidity();
		auto t = dht.readTemperature();
		if (isnan(t) || isnan(h)) {
			delay(100);
			h = dht.readHumidity();
			t = dht.readTemperature(false);
		}
		if (isnan(t) || isnan(h)) {
			//debug_out(String(FPSTR(SENSORS_DHT22)) + FPSTR(DBG_TXT_COULDNT_BE_READ), DEBUG_ERROR, 1);
		} else {
			/*
			debug_out(FPSTR(DBG_TXT_TEMPERATURE), DEBUG_MIN_INFO, 0);
			debug_out(String(t) + u8"°C", DEBUG_MIN_INFO, 1);
			debug_out(FPSTR(DBG_TXT_HUMIDITY), DEBUG_MIN_INFO, 0);
			debug_out(String(h) + "%", DEBUG_MIN_INFO, 1);
			*/
			last_value_DHT_T = t;
			last_value_DHT_H = h;
			//s += Value2Json(F("temperature"), Float2String(last_value_DHT_T));
			//s += Value2Json(F("humidity"), Float2String(last_value_DHT_H));
		}
	}

	//debug_out("----", DEBUG_MIN_INFO, 1);
	Dht result = { last_value_DHT_T, last_value_DHT_H };

	//debug_out(String(FPSTR(DBG_TXT_END_READING)) + "DHT22", DEBUG_MED_INFO, 1);

	return result;
}

/*****************************************************************
 * read BME280 sensor values                                     *
 *****************************************************************/
Bme280 sensorBME280() {
	//String s;

	//DEBUG_PRINTLN(String(FPSTR(DBG_TXT_START_READING)) + FPSTR(SENSORS_BME280));

	bme280.takeForcedMeasurement();
	const auto t = bme280.readTemperature();
	const auto h = bme280.readHumidity();
	const auto p = bme280.readPressure();
	if (isnan(t) || isnan(h) || isnan(p)) {
		last_value_BME280_T = -128.0;
		last_value_BME280_H = -1.0;
		last_value_BME280_P = -1.0;
		//DEBUG_PRINTLN(String(FPSTR(SENSORS_BME280)) + FPSTR(DBG_TXT_COULDNT_BE_READ));
	} else {
		/*
		debug_out(FPSTR(DBG_TXT_TEMPERATURE), DEBUG_MIN_INFO, 0);
		debug_out(Float2String(t) + " C", DEBUG_MIN_INFO, 1);
		debug_out(FPSTR(DBG_TXT_HUMIDITY), DEBUG_MIN_INFO, 0);
		debug_out(Float2String(h) + " %", DEBUG_MIN_INFO, 1);
		debug_out(FPSTR(DBG_TXT_PRESSURE), DEBUG_MIN_INFO, 0);
		debug_out(Float2String(p / 100) + " hPa", DEBUG_MIN_INFO, 1);
		 */
		last_value_BME280_T = t;
		last_value_BME280_H = h;
		last_value_BME280_P = p;
		/*
		s += Value2Json(F("BME280_temperature"), Float2String(last_value_BME280_T));
		s += Value2Json(F("BME280_humidity"), Float2String(last_value_BME280_H));
		s += Value2Json(F("BME280_pressure"), Float2String(last_value_BME280_P));
		 */
	}
	//debug_out("----", DEBUG_MIN_INFO, 1);

	//debug_out(String(FPSTR(DBG_TXT_END_READING)) + FPSTR(SENSORS_BME280), DEBUG_MED_INFO, 1);

	Bme280 result = { last_value_BME280_T, last_value_BME280_H, last_value_BME280_P };

	return result;
}

/*****************************************************************
 * Init BME280                                                   *
 *****************************************************************/
bool initBME280(char addr) {
	DEBUG_PRINT(F("Trying BME280 sensor on "));
	DEBUG_PRINT(String(addr, HEX));

	if (bme280.begin(addr)) {
		DEBUG_PRINTLN(F(" ... found"));
		bme280.setSampling(
			Adafruit_BME280::MODE_FORCED,
			Adafruit_BME280::SAMPLING_X1,
			Adafruit_BME280::SAMPLING_X1,
			Adafruit_BME280::SAMPLING_X1,
			Adafruit_BME280::FILTER_OFF);
		return true;
	} else {
		DEBUG_PRINTLN(F(" ... not found"));
		return false;
	}
}

static void powerOnTestSensors() {

	DEBUG_PRINTLN(F("powerOnTestSensors"));
	if (cfg::sds_read) {
		DEBUG_PRINTLN(F("check SDS"));
		DEBUG_PRINTLN(F("Read SDS..."));
		SDS_cmd(PmSensorCmd::Start);
		delay(100);
		SDS_cmd(PmSensorCmd::ContinuousMode);
		delay(100);
		DEBUG_PRINTLN(F("Stopping SDS011..."));
		is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
		DEBUG_PRINTLN(F("check SDS: ok"));
	}

	if (cfg::dht_read) {
		DEBUG_PRINTLN(F("check DHT"));
		dht.begin();                                        // Start DHT
		sensor_t sensor;
		DEBUG_PRINTLN(F("Read DHT..."));
		//dht.temperature().getSensor(&sensor);
		//DEBUG_PRINT(F("Sensor Type "));
		//DEBUG_PRINTLN(sensor.name);
		DEBUG_PRINTLN(F("check DHT: ok"));
	}


	if (cfg::bme_read) {
		DEBUG_PRINTLN(F("check BME"));
		DEBUG_PRINTLN(F("Read BME280..."));
		if (!initBME280(0x76) && !initBME280(0x77)) {
			DEBUG_PRINTLN(F("Check BME280 wiring"));
			bme_init_failed = 1;
		} else {
			DEBUG_PRINTLN(F("check BME: ok"));
		}
	}
}

///////////////////////////////////////////////////////////////////////////
//   MQTT
///////////////////////////////////////////////////////////////////////////
/*
   Function called when a MQTT message arrived
   @param topic   The topic of the MQTT message
   @param _payload The payload of the MQTT message
   @param length  The length of the payload
*/
void callback(char* topic, byte* _payload, unsigned int length) {
  // handle the MQTT topic of the received message
  _payload[length] = '\0';
  char* payload = (char*) _payload;
  if (strcmp(topic, MQTT_ENDPOINT_SYS_RESET)==0) {
    if (strcmp(payload, MQTT_ON_PAYLOAD)==0) {
      publishState(MQTT_ENDPOINT_SYS_RESET, LOW);
      reset();
    }
  } else if (strcmp(topic, MQTT_ENDPOINT_SYS_UPDATE)==0) {
    if (length > 0) {
      updateFW(payload);
    }
  } 
  /*
  else if (strcmp(topic, MQTT_ENDPOINT_TOPIC_2)==0) {
    if (strcmp(payload, MQTT_ON_PAYLOAD)==0) {
      DEBUG_PRINTLN(F("Info: topic 2"));
      // Do what you need
    }
  }
   */
}

/*
  Function called to pulish payload data
*/
void publishData(char* topic, char* payload) {
  if (mqttClient.publish(topic, payload, true)) {
    DEBUG_PRINT(F("INFO: MQTT message publish succeeded. Topic: "));
    DEBUG_PRINT(topic);
    DEBUG_PRINT(F(" Payload: "));
    DEBUG_PRINTLN(payload);
  } else {
    DEBUG_PRINTLN(F("ERROR: MQTT message publish failed, either connection lost, or message too large"));
  }
}

void publishData(char* topic, String s) {
	publishData(topic, s.c_str());
}

void publishData(char* topic, double f) {
	char payload[41];  // Need to technically hold float max, 39 digits and minus sign.
	dtostrf(f, 0, 2, payload);
	publishData(topic, payload);
}

void publishData(char* topic, int32_t i) {
	char payload[11];
	ultoa(i, payload, 10);
	publishData(topic, payload);
}

void publishData(char* topic, uint32_t i) {
	char payload[12];
	ultoa(i, payload, 10);
	publishData(topic, payload);
}


/*
  Function called to publish a state
*/
void publishState(char* topic, int state) {
  publishData(topic, (char*) (state == 1 ? MQTT_ON_PAYLOAD : MQTT_OFF_PAYLOAD));
}

/*
  Function called to connect/reconnect to the MQTT broker
 */
void reconnect() {
  // test if the module has an IP address
  // if not, restart the module
  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINTLN(F("ERROR: The module isn't connected to the internet"));
    reset();
  }

  // try to connect to the MQTT broker
  while (!mqttClient.connected()) {
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, MQTT_ENDPOINT_SYS_ONLINE, 0, 1, "0")) {
      DEBUG_PRINTLN(F("INFO: The client is successfully connected to the MQTT broker"));
    } else {
      DEBUG_PRINTLN(F("ERROR: The connection to the MQTT broker failed"));
      delay(1000);
    }
  }

  mqttClient.subscribe(MQTT_ENDPOINT_SYS_RESET);
  mqttClient.subscribe(MQTT_ENDPOINT_SYS_UPDATE);

  //mqttClient.subscribe(MQTT_TOPIC_1);
}

///////////////////////////////////////////////////////////////////////////
//   SYSTEM
///////////////////////////////////////////////////////////////////////////
/*
 Function called to update chip firmware
 */
void updateFW(char* fwUrl) {
  DEBUG_PRINTLN(F("INFO: updating firmware ..."));
  t_httpUpdate_return ret = ESPhttpUpdate.update(fwUrl);
#ifdef DEBUG
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      DEBUG_PRINT(F("ERROR: firmware update failed: "));
      DEBUG_PRINTLN(ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      DEBUG_PRINTLN(F("ERROR: no new firmware"));
      break;
    case HTTP_UPDATE_OK:
      DEBUG_PRINTLN(F("Info: firmware update ok"));
      break;
  }
#endif
}

/*
  Function called to reset / restart the switch
 */
void reset() {
  DEBUG_PRINTLN(F("INFO: Reset..."));
  ESP.reset();
  delay(1000);
}


static void checkForceRestart() {
	if (msSince(time_point_device_start_ms) > DURATION_BEFORE_FORCED_RESTART_MS) {
		ESP.restart();
	}
}

/*
  Function called to connect to Wifi
 */
void connectWiFi() {
  // Connect to WiFi network
  DEBUG_PRINT(F("INFO: Connecting to "));
  DEBUG_PRINTLN(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  DEBUG_PRINTLN(F(""));
  DEBUG_PRINTLN(F("INFO: WiFi connected"));
}

static int32_t calcWiFiSignalQuality(int32_t rssi) {
	if (rssi > -50) {
		rssi = -50;
	}
	if (rssi < -100) {
		rssi = -100;
	}
	return (rssi + 100) * 2;
}


void time_is_set (void) {
	sntp_time_is_set = true;
}

static bool acquireNetworkTime() {
	int retryCount = 0;
	DEBUG_PRINTLN(F("Setting time using SNTP"));
	time_t now = time(nullptr);
	DEBUG_PRINTLN(ctime(&now));
	DEBUG_PRINTLN(F("Lokal:"));
	settimeofday_cb(time_is_set);
	configTime(8 * 3600, 0, "fritz.box");
	while (retryCount++ < 20) {
		// later than 2000/01/01:00:00:00
		if (sntp_time_is_set) {
			now = time(nullptr);
			DEBUG_PRINTLN(ctime(&now));
			return true;
		}
		delay(500);
		DEBUG_PRINT(".");
	}
	DEBUG_PRINTLN(F("\nrouter/gateway:"));
	retryCount = 0;
	configTime(0, 0, WiFi.gatewayIP().toString().c_str());
	while (retryCount++ < 20) {
		// later than 2000/01/01:00:00:00
		if (sntp_time_is_set) {
			now = time(nullptr);
			DEBUG_PRINTLN(ctime(&now));
			return true;
		}
		delay(500);
		DEBUG_PRINT(".");
	}
	return false;
}


///////////////////////////////////////////////////////////////////////////
//   Setup() and loop()
///////////////////////////////////////////////////////////////////////////
void setup() {	

#ifdef DEBUG
  	Serial.begin(115200);
#endif
  	DEBUG_PRINTLN(F(""));
  	DEBUG_PRINTLN(F(""));
  	DEBUG_PRINTLN(F("Info: booted"));

	Wire.begin(I2C_PIN_SDA, I2C_PIN_SCL);

	//esp_chipid = String(ESP.getChipId());

  	// connect to wifi
  	connectWiFi();

  	got_ntp = acquireNetworkTime();
	DEBUG_PRINT(F("\nNTP time "));
	DEBUG_PRINTLN(String(got_ntp?"":"not ")+F("received"));

  // get the Chip ID of the switch and use it as the MQTT client ID
  sprintf(MQTT_CLIENT_ID, "%06x", ESP.getChipId());
  DEBUG_PRINT(F("INFO: MQTT client ID/Hostname: "));
  DEBUG_PRINTLN(MQTT_CLIENT_ID);
  DEBUG_PRINT(F("INFO: CHIP: "));
  DEBUG_PRINTLN(ESP.getFullVersion());

  // create MQTT endpoints
  sprintf(MQTT_ENDPOINT_REGISTRY, "%s%s", MQTT_TOPIC_BASE, MQTT_TOPIC_REGISTRY);

  sprintf(MQTT_ENDPOINT_SYS_VERSION, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_TOPIC_SYSTEM_VERSION);
  sprintf(MQTT_ENDPOINT_SYS_ALIAS, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_TOPIC_SYSTEM_ALIAS);
  sprintf(MQTT_ENDPOINT_SYS_CHIP, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_TOPIC_SYSTEM_CHIP);
  sprintf(MQTT_ENDPOINT_SYS_LOCATION, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_TOPIC_SYSTEM_LOC);
  sprintf(MQTT_ENDPOINT_SYS_UPDATE, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_TOPIC_SYSTEM_UPDATE);
  sprintf(MQTT_ENDPOINT_SYS_RESET, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_TOPIC_SYSTEM_RESET);
  sprintf(MQTT_ENDPOINT_SYS_ONLINE, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_TOPIC_SYSTEM_ONLINE);
  sprintf(MQTT_ENDPOINT_SYS_WIFI_SIGNAL_DB, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_TOPIC_SYSTEM_WIFI_SIGNAL_DB);
  sprintf(MQTT_ENDPOINT_SYS_WIFI_SIGNAL_PER, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_TOPIC_SYSTEM_WIFI_SIGNAL_PER);
  sprintf(MQTT_ENDPOINT_SYS_WIFI_IP, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_TOPIC_SYSTEM_WIFI_IP);

  sprintf(MQTT_ENDPOINT_SENSOR_PM10, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_SENSOR_PM10);
  sprintf(MQTT_ENDPOINT_SENSOR_PM25, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_SENSOR_PM25);
  sprintf(MQTT_ENDPOINT_SENSOR_DHT_TEMP, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_SENSOR_DHT_TEMP);
  sprintf(MQTT_ENDPOINT_SENSOR_DHT_HUM, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_SENSOR_DHT_HUM);
  sprintf(MQTT_ENDPOINT_SENSOR_BME_TEMP, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_SENSOR_BME_TEMP);
  sprintf(MQTT_ENDPOINT_SENSOR_BME_HUM, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_SENSOR_BME_HUM);
  sprintf(MQTT_ENDPOINT_SENSOR_BME_PRESS, "%s%s/%s", MQTT_TOPIC_BASE, MQTT_CLIENT_ID, MQTT_SENSOR_BME_PRESS);


#ifdef TLS
  // check the fingerprint of io.adafruit.com's SSL cert
  verifyFingerprint();
#endif

  // configure MQTT
  mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT));
  mqttClient.setCallback(callback);

  // connect to the MQTT broker
  reconnect();

  // register device
  publishData(MQTT_ENDPOINT_REGISTRY, (char *)MQTT_CLIENT_ID);

  // publish running firmware version
  publishData(MQTT_ENDPOINT_SYS_VERSION, (char*) FW_VERSION);
  publishData(MQTT_ENDPOINT_SYS_ALIAS, (char*) ALIAS);
  publishData(MQTT_ENDPOINT_SYS_CHIP, (char *)CHIP);
  publishData(MQTT_ENDPOINT_SYS_LOCATION, (char*) LOCATION);
  publishState(MQTT_ENDPOINT_SYS_ONLINE, 1);
  publishData(MQTT_ENDPOINT_SYS_WIFI_IP, WiFi.localIP());

  // starting sensors
  powerOnTestSensors();

  DEBUG_PRINTLN(F("SETUP complete."));
}


void loop() {
	Sds result_SDS;
	Dht result_DHT;
	//Bme280 result_BME280;

	unsigned long sum_send_time = 0;
	unsigned long start_send;

	act_micro = micros();
	act_milli = millis();
	send_now = msSince(starttime) > cfg::sending_intervall_ms;
	time_t now;

  	// keep the MQTT client connected to the broker
  	if (!mqttClient.connected()) {
    	reconnect();
  	}
  	mqttClient.loop();

  	yield();

  	if (send_now) {
		/*
		DEBUG_PRINTLN(F("Creating data string:"));
		String data = FPSTR(data_first_part);
		data.replace("{v}", FW_VERSION);
		String data_sample_times  = Value2Json(F("samples"), String(sample_count));
		data_sample_times += Value2Json(F("min_micro"), String(min_micro));
		data_sample_times += Value2Json(F("max_micro"), String(max_micro));
		*/

		int32_t signal_strength = WiFi.RSSI();
		DEBUG_PRINT(F("WLAN signal strength: "));
		DEBUG_PRINT(String(signal_strength));
		DEBUG_PRINTLN(F(" dBm"));
		DEBUG_PRINTLN(F("----"));
		// Sending WiFi Quality each time Data is sent
		publishData(MQTT_ENDPOINT_SYS_WIFI_SIGNAL_DB, signal_strength);
		publishData(MQTT_ENDPOINT_SYS_WIFI_SIGNAL_PER, calcWiFiSignalQuality(signal_strength));

		//yield();

		//if ((msSince(starttime_SDS) > SAMPLETIME_SDS_MS) || send_now) {
		DEBUG_PRINTLN(F("Start reading SDS"));
		result_SDS = sensorSDS();
			//starttime_SDS = act_milli;
		//}

		DEBUG_PRINTLN(F("Start reading DHT"));
		result_DHT = sensorDHT();

		if (cfg::sds_read) {
			//data += result_SDS;
			//if (cfg::send2dusti) {
				now = time(nullptr);
				DEBUG_PRINTLN(ctime(&now));
				DEBUG_PRINT(String(F("Luftdaten ")) + F("(SDS): "));
				DEBUG_PRINT(result_SDS.p10);
				DEBUG_PRINT(F(", "));
				DEBUG_PRINTLN(result_SDS.p25);
				start_send = millis();
				publishData(MQTT_ENDPOINT_SENSOR_PM10, result_SDS.p10);
				publishData(MQTT_ENDPOINT_SENSOR_PM25, result_SDS.p25);
				//sendLuftdaten(result_SDS, SDS_API_PIN, HOST_DUSTI, HTTP_PORT_DUSTI, URL_DUSTI, true, "SDS_");
				sum_send_time += millis() - start_send;
			//}
		}
		if (cfg::dht_read) {
			DEBUG_PRINTLN(String(FPSTR(F("Luftdaten "))) + F("(DHT): "));
			DEBUG_PRINT(result_DHT.temp);
			DEBUG_PRINT(F(", "));
			DEBUG_PRINTLN(result_DHT.hum);
			start_send = millis();
			publishData(MQTT_ENDPOINT_SENSOR_DHT_TEMP, result_DHT.temp);
			publishData(MQTT_ENDPOINT_SENSOR_DHT_HUM, result_DHT.hum);
			//sendLuftdaten(result_DHT, DHT_API_PIN, HOST_DUSTI, HTTP_PORT_DUSTI, URL_DUSTI, true, "DHT_");
			sum_send_time += millis() - start_send;
		}

		/* 
		data_sample_times += Value2Json("signal", signal_strength);
		data += data_sample_times;

		if ((unsigned)(data.lastIndexOf(',') + 1) == data.length()) {
			data.remove(data.length() - 1);
		}
		data += "]}";
		*/

		//sum_send_time += sendDataToOptionalApis(data);

		checkForceRestart();

		/*
		if (msSince(last_update_attempt) > PAUSE_BETWEEN_UPDATE_ATTEMPTS_MS) {
			checkForUpdates();
		}
		 */

		sending_time = (4 * sending_time + sum_send_time) / 5;
		DEBUG_PRINT(F("Time for sending data (ms): "));
		DEBUG_PRINTLN(String(sending_time));
		// reconnect to WiFi if disconnected
		/*
		if (WiFi.status() != WL_CONNECTED) {
			DEBUG_PRINT(F("Connection lost, reconnecting "));
			WiFi.reconnect();
			waitForWifiToConnect(20);
			DEBUG_PRINTLN("");
		}
		 */

		// Resetting for next sampling
		sample_count = 0;
		last_micro = 0;
		min_micro = 1000000000;
		max_micro = 0;
		sum_send_time = 0;
		starttime = millis();                               // store the start time
	}
	yield();
}
