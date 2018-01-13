
/*****************************************************************
/* Software is based on OK LAB Particulate Matter Sensor         * 
/* but adopted to own requirements                               * 
/*                                                               *
/*      - SparkFun ESP8266 Thing                                 *
/*      - Nova SDS0111                                           *
/*  ﻿http://inovafitness.com/en/Laser-PM2-5-Sensor-SDS011-35.html *
/*                                                               *
/* Wiring Instruction:                                           *
/*      - SDS011 Pin 1  (TX)   -> Pin GPIO5                      *
/*      - SDS011 Pin 2  (RX)   -> Pin GPIO4                      *
/*      - SDS011 Pin 3  (GND)  -> GND                            *
/*      - SDS011 Pin 4  (2.5m) -> unused                         *
/*      - SDS011 Pin 5  (5V)   -> VU                             *
/*      - SDS011 Pin 6  (1m)   -> unused                         *
/*                                                               *
/*****************************************************************

/* Extension: DHT22 (AM2303)                                     *
/*  ﻿http://www.aosong.com/en/products/details.asp?id=117         *
/*                                                               *
/* DHT22 Wiring Instruction                                      *
/* (left to right, front is perforated side):                    *
/*      - DHT22 Pin 1 (VDD)     -> Pin 3V3 (3.3V)                *
/*      - DHT22 Pin 2 (DATA)    -> Pin D7 (GPIO13)               *
/*      - DHT22 Pin 3 (NULL)    -> unused                        *
/*      - DHT22 Pin 4 (GND)     -> Pin GND                       *
/*                                                               *
/*****************************************************************


/*****************************************************************
/* Includes                                                      *
/*****************************************************************/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include "credentials.h"

#define SDS_PIN_RX 5
#define SDS_PIN_TX 4

#define DHT_PIN 13

// Definition der Debuglevel
#define DEBUG_ERROR 1
#define DEBUG_WARNING 2
#define DEBUG_MIN_INFO 3
#define DEBUG_MED_INFO 4
#define DEBUG_MAX_INFO 5

int  debug = 3;

const char* place           = "DUSTSENSOR";
const char* sensordata      = "sensordata";
const char* typeTemperature = "TEMPERATURE";
const char* typeHumidity    = "HUMIDITY";
const char* typeDustPM10    = "DUST_PM10";
const char* typeDustPM25    = "DUST_PM25";

const char* temperatureTopic = "sensordata/dustsensor/temperature";
const char* humidityTopic    = "sensordata/dustsensor/humidity";
const char* dustPM10Topic    = "sensordata/dustsensor/dustpm10";
const char* dustPM25Topic    = "sensordata/dustsensor/dustpm25";

char json[512];

WiFiClient espClient;
PubSubClient client(espClient);


/*****************************************************************
/* SDS011 declarations                                           *
/*****************************************************************/
SoftwareSerial serialSDS(SDS_PIN_RX, SDS_PIN_TX, false, 128);
/*****************************************************************
/* DHT declaration                                               *
/*****************************************************************/
DHT dht(DHT_PIN, DHT22);



/*****************************************************************
/* Variable Definitions for PPD24NS                              *
/* P1 for PM10 & P2 for PM25                                     *
/*****************************************************************/


bool send_now = false;
unsigned long starttime;
unsigned long starttime_SDS;
unsigned long act_micro;
unsigned long act_milli;
unsigned long last_micro = 0;
unsigned long min_micro = 1000000000;
unsigned long max_micro = 0;
unsigned long diff_micro = 0;

//const unsigned long sampletime_ms = 30000;

const unsigned long sampletime_SDS_ms = 1000;
const unsigned long warmup_time_SDS_ms = 15000;
const unsigned long reading_time_SDS_ms = 5000;
bool is_SDS_running = true;


unsigned long sending_intervall_ms = 145000;
unsigned long sending_time = 0;

int sds_pm10_sum = 0;
int sds_pm25_sum = 0;
int sds_val_count = 0;
int sds_pm10_max = 0;
int sds_pm10_min = 20000;
int sds_pm25_max = 0;
int sds_pm25_min = 20000;

String last_value_SDS_P1 = "";
String last_value_SDS_P2 = "";

/*****************************************************************
/* Debug output                                                  *
/*****************************************************************/
void debug_out(const String& text, const int level, const bool linebreak) {
    if (level <= debug) {
        if (linebreak) {
            Serial.println(text);
        } else {
            Serial.print(text);
        }
    }
}

/**************************************************************************/
/*
    Try to connect to the WIFI 
*/
/**************************************************************************/
boolean wifiConnect(void) {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }
    WiFi.begin(ssid, password);
    // Wait for connection
    int tries = 10;
    boolean retVal = true;
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        tries--;
        if (tries <= 0) {
            // got no connection to WiFi, going to sleep
            return false;
        }
    }
    return retVal;
}

/**************************************************************************/
/*
    Try to connect to the MQTT broker, after 10 tries, we return false
*/
/**************************************************************************/
boolean mqttConnect(void) {

    if (!wifiConnect()) {
        return false;
    }
    
    client.setServer(mqttServer, 1883);

    int  tries = 10;
    while (!client.connected()) {
        tries--;
        if (tries == 0) {
            return false;
        }
        if (!client.connect(place, mqttUser, mqttPassword)) {
            delay(5000);
        }
    }
    return true;
}

void sendDHT22Data() {

    float humidity    = dht.readHumidity();
    float temperature = dht.readTemperature();
    
    StaticJsonBuffer<256> jsonBuffer;
    JsonObject&           root = jsonBuffer.createObject();

    if (isnan(humidity) || isnan(temperature)) {
        root["error"] = "DHT22 read error";
        root.printTo(json, sizeof(json));
        client.publish("sensordata/test/status", json);
    } else {

        root.set("place", place);
        root.set("sensor", "DHT22");
        root.set("type", typeTemperature);
        root.set("temperature", temperature, 2);  

        root.printTo(json, sizeof(json));
        client.publish(temperatureTopic, json);
    
        root.remove("temperature");
    
        root.set("type", typeHumidity);
        root.set("humidity", humidity, 2);
        root.printTo(json, sizeof(json));
        client.publish(humidityTopic, json);
    }
    delay(250);    
}

/*****************************************************************
/* start SDS011 sensor                                           *
/*****************************************************************/
void start_SDS() {
    const uint8_t start_SDS_cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};
    serialSDS.write(start_SDS_cmd, sizeof(start_SDS_cmd)); is_SDS_running = true;
}

/*****************************************************************
/* stop SDS011 sensor                                            *
/*****************************************************************/
void stop_SDS() {
    const uint8_t stop_SDS_cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
    serialSDS.write(stop_SDS_cmd, sizeof(stop_SDS_cmd)); is_SDS_running = false;
}


/*****************************************************************
/* read SDS011 sensor values                                     *
/*****************************************************************/
String SDS_version_date() {
    const uint8_t version_SDS_cmd[] = {0xAA, 0xB4, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
    String s = "";
    String value_hex;
    char buffer;
    int value;
    int len = 0;
    String version_date = "";
    String device_id = "";
    int checksum_is;
    int checksum_ok = 0;
 
    debug_out(F("Start reading SDS011 version date"), DEBUG_MED_INFO, 1);

    start_SDS();

    delay(100);

    serialSDS.write(version_SDS_cmd, sizeof(version_SDS_cmd));

    delay(500);

    while (serialSDS.available() > 0) {
        buffer = serialSDS.read();
        debug_out(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MED_INFO, 1);
//      "aa" = 170, "ab" = 171, "c0" = 192
        value = int(buffer);
        switch (len) {
        case (0): if (value != 170) { len = -1; }; break;
        case (1): if (value != 197) { len = -1; }; break;
        case (2): if (value != 7) { len = -1; }; break;
        case (3): version_date  = String(value); checksum_is = 7 + value; break;
        case (4): version_date += "-" + String(value); checksum_is += value; break;
        case (5): version_date += "-" + String(value); checksum_is += value; break;
        case (6): if (value < 0x10) {device_id  = "0" + String(value, HEX);} else {device_id  = String(value, HEX);}; checksum_is += value; break;
        case (7): if (value < 0x10) {device_id += "0";}; device_id += String(value, HEX); checksum_is += value; break;
        case (8):
            debug_out(F("Checksum is: "), DEBUG_MED_INFO, 0);
            debug_out(String(checksum_is % 256), DEBUG_MED_INFO, 0);
            debug_out(F(" - should: "), DEBUG_MED_INFO, 0);
            debug_out(String(value), DEBUG_MED_INFO, 1);
            if (value == (checksum_is % 256)) { checksum_ok = 1; } else { len = -1; }; break;
        case (9): if (value != 171) { len = -1; }; break;
        }
        len++;
        if (len == 10 && checksum_ok == 1) {
            s = version_date + "(" + device_id + ")";
            debug_out(F("SDS version date : "), DEBUG_MIN_INFO, 0);
            debug_out(version_date, DEBUG_MIN_INFO, 1);
            debug_out(F("SDS device ID:     "), DEBUG_MIN_INFO, 0);
            debug_out(device_id, DEBUG_MIN_INFO, 1);
            len = 0; checksum_ok = 0; version_date = ""; device_id = ""; checksum_is = 0;
        }
        yield();
    }

    debug_out(F("End reading SDS011 version date"), DEBUG_MED_INFO, 1);
    return s;
}




/*****************************************************************
/* read SDS011 sensor values                                     *
/*****************************************************************/
void sensorSDS() {
    String value_hex;
    char buffer;
    int value;
    int len = 0;
    int pm10_serial = 0;
    int pm25_serial = 0;
    int checksum_is;
    int checksum_ok = 0;

    debug_out(F("Start reading SDS011"), DEBUG_MED_INFO, 1);
    if (long(act_milli - starttime) < (long(sending_intervall_ms) - long(warmup_time_SDS_ms + reading_time_SDS_ms))) {
        if (is_SDS_running) {
            stop_SDS();
        }
    } else {
        if (! is_SDS_running) {
            start_SDS();
        }

        while (serialSDS.available() > 0) {
            buffer = serialSDS.read();
            debug_out(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO, 1);
//          "aa" = 170, "ab" = 171, "c0" = 192
            value = int(buffer);
            switch (len) {
            case (0): if (value != 170) { len = -1; }; break;
            case (1): if (value != 192) { len = -1; }; break;
            case (2): pm25_serial = value; checksum_is = value; break;
            case (3): pm25_serial += (value << 8); checksum_is += value; break;
            case (4): pm10_serial = value; checksum_is += value; break;
            case (5): pm10_serial += (value << 8); checksum_is += value; break;
            case (6): checksum_is += value; break;
            case (7): checksum_is += value; break;
            case (8):
                debug_out(F("Checksum is: "), DEBUG_MED_INFO, 0); debug_out(String(checksum_is % 256), DEBUG_MED_INFO, 0);
                debug_out(F(" - should: "), DEBUG_MED_INFO, 0); debug_out(String(value), DEBUG_MED_INFO, 1);
                if (value == (checksum_is % 256)) { checksum_ok = 1; } else { len = -1; }; break;
            case (9): if (value != 171) { len = -1; }; break;
            }
            len++;
            if (len == 10 && checksum_ok == 1 && (long(act_milli - starttime) > (long(sending_intervall_ms) - long(reading_time_SDS_ms)))) {
                if ((! isnan(pm10_serial)) && (! isnan(pm25_serial))) {
                    sds_pm10_sum += pm10_serial;
                    sds_pm25_sum += pm25_serial;
                    if (sds_pm10_min > pm10_serial) { sds_pm10_min = pm10_serial; }
                    if (sds_pm10_max < pm10_serial) { sds_pm10_max = pm10_serial; }
                    if (sds_pm25_min > pm25_serial) { sds_pm25_min = pm25_serial; }
                    if (sds_pm25_max < pm25_serial) { sds_pm25_max = pm25_serial; }
                    sds_val_count++;
                }
                len = 0; checksum_ok = 0; pm10_serial = 0.0; pm25_serial = 0.0; checksum_is = 0;
            }
            yield();
        }

    }
    if (send_now) {

        last_value_SDS_P1 = "";
        last_value_SDS_P2 = "";
        if (sds_val_count > 2) {
            sds_pm10_sum = sds_pm10_sum - sds_pm10_min - sds_pm10_max;
            sds_pm25_sum = sds_pm25_sum - sds_pm25_min - sds_pm25_max;
            sds_val_count = sds_val_count - 2;
        }
        if (sds_val_count > 0  && mqttConnect()) {
            StaticJsonBuffer<512> jsonBuffer;
            JsonObject&           root = jsonBuffer.createObject();

            root.set("place", place);
            root.set("sensor", "SDS011");
            root.set("type", typeDustPM10);
            root.set("pm10", float(sds_pm10_sum) / (sds_val_count * 10.0), 2);  
            
            root.printTo(json, sizeof(json));
            client.publish(dustPM10Topic, json);

            root.set("type", typeDustPM25);
            root.remove("pm10");            
            root.set("pm25", float(sds_pm25_sum) / (sds_val_count * 10.0), 2);  

            root.printTo(json, sizeof(json));
            client.publish(dustPM25Topic, json);

            delay(250); 
        }
        sds_pm10_sum = 0; 
        sds_pm25_sum = 0; 
        sds_val_count = 0;
        sds_pm10_max = 0; 
        sds_pm10_min = 20000; 
        sds_pm25_max = 0; 
        sds_pm25_min = 20000;
        if ((sending_intervall_ms > (warmup_time_SDS_ms + reading_time_SDS_ms))) {
            stop_SDS();
        }
    }

    debug_out(F("End reading SDS011"), DEBUG_MED_INFO, 1);
}



/*****************************************************************
/* The Setup                                                     *
/*****************************************************************/
void setup() {
    Serial.begin(9600);                 // Output to Serial at 9600 baud

    serialSDS.begin(9600);
    dht.begin();    // Start DHT
    delay(10);

    debug_out(F("Lese SDS..."), DEBUG_MIN_INFO, 1);
    debug_out(SDS_version_date(), DEBUG_MIN_INFO, 1) ;
    
    
    debug_out(F("Stoppe SDS011..."), DEBUG_MIN_INFO, 1);
    stop_SDS();
    
    starttime = millis();                   // store the start time
    starttime_SDS = millis();

    wifiConnect();
}

/*****************************************************************
/* And action                                                    *
/*****************************************************************/
void loop() {

    act_micro = micros();
    act_milli = millis();
    send_now = (act_milli - starttime) > sending_intervall_ms;

    if (last_micro != 0) {
        diff_micro = act_micro - last_micro;
        if (max_micro < diff_micro) { max_micro = diff_micro;}
        if (min_micro > diff_micro) { min_micro = diff_micro;}
        last_micro = act_micro;
    } else {
        last_micro = act_micro;
    }


    if (((act_milli - starttime_SDS) > sampletime_SDS_ms) || ((act_milli - starttime) > sending_intervall_ms)) {
        debug_out(F("Call sensorSDS"), DEBUG_MAX_INFO, 1);
        sensorSDS();
        starttime_SDS = act_milli;
    }

    if (send_now && mqttConnect()) {
        sendDHT22Data();

        // Resetting for next sampling
        last_micro = 0;
        min_micro = 1000000000;
        max_micro = 0;
        starttime = millis(); // store the start time
    }
    yield();
}

