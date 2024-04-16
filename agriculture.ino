#include <Adafruit_Sensor.h>
#include <DHT.h>                   // Data ---> D3 VCC ---> 3V3 GND ---> GND
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// WiFi parameters
#define WLAN_SSID       "iPhone"
#define WLAN_PASS       "12345678"

// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "deepaksarun"
#define AIO_KEY         "aio_UeXo90Tlqga2S8TVR5FUU02aR5HM"

WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish Temperature1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature1");
Adafruit_MQTT_Publish Humidity1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity1");
Adafruit_MQTT_Publish Moisture1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisture1");

#define DHTPIN 2     // Digital pin connected to the DHT sensor
const int soilPin = 34; // Analog pin connected to the soil moisture sensor
const int dryValue = 4095; // Analog reading in dry soil conditions
const int wetValue = 0; // Analog reading in wet soil conditions

#define DHTTYPE DHT22   // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  Serial.println(F("Adafruit IO Example"));

  // Connect to WiFi access point.
  Serial.println();
  Serial.println(F("Connecting to WiFi"));
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());

  // connect to Adafruit IO
  connect();

  // initialize DHT22 sensor
  dht.begin();
}

// connect to Adafruit IO via MQTT
void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

void loop() {
  // ping Adafruit IO a few times to make sure we remain connected
  if (!mqtt.ping()) {
    // reconnect to Adafruit IO
    if (!mqtt.connected())
      connect();
  }
  
  // read temperature and humidity data from DHT22 sensor
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  float soil = analogRead(soilPin);
  
  int moisturePercentage = map(soil, dryValue, wetValue, 0, 100);
  if (isnan(temp) || isnan(hum)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    delay(2000);
    return;
  }

  Serial.print(F("Temperature: ")); Serial.print(temp); Serial.println(" °C");
  Serial.print(F("Humidity: ")); Serial.print(hum); Serial.println(" %");
  Serial.print(F("Soil Moisture: ")); Serial.print(moisturePercentage); Serial.println(" %");
  
  // publish data to Adafruit IO
  if (!Temperature1.publish(temp)) {
    Serial.println(F("Failed to publish temperature"));
  } else {
    Serial.println(F("Temperature published"));
  }

  if (!Humidity1.publish(hum)) {
    Serial.println(F("Failed to publish humidity"));
  } else {
    Serial.println(F("Humidity published"));
  }

  if (!Moisture1.publish(moisturePercentage)) {
    Serial.println(F("Failed to publish moisture"));
  } else {
    Serial.println(F("Moisture published"));
  }

  // delay for a short interval before the next reading
  delay(10000);
}
