//DEEEEEEEEEEEEEEEEEEEEEEEEEEEEEMMMMMMMMMMOOOOOOOOOOOOOOOO

#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#define WLAN_SSID       "InssV2"
#define WLAN_PASS       "tozyyyyy"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "Inssso"
//#define AIO_KEY         "aio_VYaE20M5aYC6iVK44yeD5yf7F2iz"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
// Setup a feed called 'temperatureAmbiante' for publishing.
Adafruit_MQTT_Publish temperatureAmbiante = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatureAmbiante");
void MQTT_connect();

/*Partie capteur ; BME680 sensor*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#define SEALEVELPRESSURE_HPA (1013.25)
#define NEW_ADDRESS (0x77)    ///< The default I2C address
Adafruit_BME680 bme;
/*FIN BME680 sensor*/

/* Partie ADC/LM35 */
#include <driver/adc.h>
#define LED_PIN 13
#define SEUIL 22
float temperature = 0;
/* FIN ADC/LM35 */

////Pour la demo
int compteur = 0;

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println(F("Adafruit MQTT demo"));
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  /*BME680 sensor*/
  while (!Serial);
  Serial.println(F("BME680 test"));

  if (!bme.begin(NEW_ADDRESS)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  /*FIN BME680 sensor*/

  /* Partie ADC/LM35 */
  pinMode(LED_PIN,OUTPUT);
  /* FIN ADC/LM35 */

  /*END SET-UP*/
}

void loop() {
  /*BME680 MQTT*/  
  MQTT_connect();
  BME_check();
  /*FIN BME680 MQTT*/

  /* Partie ADC/LM35 */
  ADC_mesure_temperature();
  alerte_incendie();
  compteur ++;
  if (compteur>=10){
    MQTT_publish_data();
    compteur = 0;
  }
  /* FIN ADC/LM35 */


}
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

void MQTT_publish_data(){
  Serial.print(F("\nTempérature mesurer : "));
  Serial.print(temperature);
  Serial.print(" degré");
  if (! temperatureAmbiante.publish(bme.temperature)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("\t Data publié avec succès!"));
  }
}

void BME_check(){
    if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
}

void ADC_mesure_temperature(){
  adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0); 
	int value = adc1_get_raw(ADC1_CHANNEL_0);
	float voltage = (float)value*1100./4095.; // Convert the read value to millivolt, using the full-scale configured with the attenuation (3.9V) and the number of bits (4095)
  temperature = voltage/10;
	Serial.println("Temp: " + String(temperature));
  alerte_incendie();
	delay(500);
}

void alerte_incendie (){
  if (temperature >= SEUIL){
    digitalWrite(LED_PIN,HIGH);
    MQTT_publish_data();
    Serial.println("Envoie de l'alerte par mail en cours : " );
    delay(1000);
    Serial.print(" . . ");
    delay(250);
    Serial.print(" . . ");
    delay(500);
    Serial.print(" . . \n"); 
    delay(500);
    Serial.print(" . . \n"); 
    delay(500);
    Serial.print(" . . \n"); 
    Serial.println("Mail envoyer à l'adresse : 'rosalieblush@gmail.com' " );

  }
  else {
      digitalWrite(LED_PIN,LOW);
  }
}