//DEEEEEEEEEEEEEEEEEEEEEEEEEEEEEMMMMMMMMMMOOOOOOOOOOOOOOOO

#include <WiFi.h>
#include <Arduino.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#define WLAN_SSID       "InssV2"
#define WLAN_PASS       "tozyyyyy"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "Inssso"
#define AIO_KEY         "aio_VYaE20M5aYC6iVK44yeD5yf7F2iz"

// Partie Mail 
#include <ESP_Mail_Client.h>

#define WIFI_SSID "InssV2"
#define WIFI_PASSWORD "tozyyyyy"

/** The smtp host name e.g. smtp.gmail.com for GMail or smtp.office365.com for Outlook or smtp.mail.yahoo.com */
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465

/* The sign in credentials */
#define AUTHOR_EMAIL "pocesiea@gmail.com"
#define AUTHOR_PASSWORD "vhwa hyai xcoe mrfk"

/* Recipient's email*/
#define RECIPIENT_EMAIL "rosalieblush@gmail.com"

/* Declare the global used SMTPSession object for SMTP transport */
SMTPSession smtp;

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status);
bool sendMessage(Session_Config config, SMTP_Message message);

 /* Declare the Session_Config for user defined session credentials */
  Session_Config config;
  /* Declare the message class */
  SMTP_Message message;

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
#define SEUIL 21
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
  // Mail 
    /*  Set the network reconnection option */
  MailClient.networkReconnect(true);

  /** Enable the debug via Serial port
   * 0 for no debugging
   * 1 for basic level debugging
   *
   * Debug port can be changed via ESP_MAIL_DEFAULT_DEBUG_PORT in ESP_Mail_FS.h
   */
  smtp.debug(1);

  /* Set the callback function to get the sending results */
  smtp.callback(smtpCallback);


  /* Set the session config */
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = "";

  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 3;
  config.time.day_light_offset = 0;


  /* Set the message headers */
  message.sender.name = F("ESP");
  message.sender.email = AUTHOR_EMAIL;
  message.subject = F("ESP Test Email");
  message.addRecipient(F("Sara"), RECIPIENT_EMAIL);
      
  //Send raw text message
  String textMsg = "Il y a le feu - Sent from ESP board";
  message.text.content = textMsg.c_str();
  message.text.charSet = "us-ascii";
  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
  
  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_low;
  message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;


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

bool sendMessage(Session_Config config, SMTP_Message message){
  //// Connect to the server 
  if (!smtp.connect(&config)){
    ESP_MAIL_PRINTF("Connection error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return false;
  }

  if (!smtp.isLoggedIn()){
    Serial.println("\nNot yet logged in.");
  }
  else{
    if (smtp.isAuthenticated())
      Serial.println("\nSuccessfully logged in.");
    else
      Serial.println("\nConnected with no Auth.");
  }

  /* Start sending Email and close the session */
  if (!MailClient.sendMail(&smtp, &message))
    ESP_MAIL_PRINTF("Error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());

  return true;

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
  Serial.print(bme.temperature);
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
    if(sendMessage(config, message)){
      Serial.println("envoyer");
    }
    MQTT_publish_data();
  }
  else {
      digitalWrite(LED_PIN,LOW);
  }
}

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status){
  /* Print the current status */
  Serial.println(status.info());

  /* Print the sending result */
  if (status.success()){
    // ESP_MAIL_PRINTF used in the examples is for format printing via debug Serial port
    // that works for all supported Arduino platform SDKs e.g. AVR, SAMD, ESP32 and ESP8266.
    // In ESP8266 and ESP32, you can use Serial.printf directly.

    Serial.println("----------------");
    ESP_MAIL_PRINTF("Message sent success: %d\n", status.completedCount());
    ESP_MAIL_PRINTF("Message sent failed: %d\n", status.failedCount());
    Serial.println("----------------\n");

    for (size_t i = 0; i < smtp.sendingResult.size(); i++)
    {
      /* Get the result item */
      SMTP_Result result = smtp.sendingResult.getItem(i);

      // In case, ESP32, ESP8266 and SAMD device, the timestamp get from result.timestamp should be valid if
      // your device time was synched with NTP server.
      // Other devices may show invalid timestamp as the device time was not set i.e. it will show Jan 1, 1970.
      // You can call smtp.setSystemTime(xxx) to set device time manually. Where xxx is timestamp (seconds since Jan 1, 1970)
      
      ESP_MAIL_PRINTF("Message No: Alerte incendie !!!!!!!\n", i + 1);
      ESP_MAIL_PRINTF("Status: %s\n", result.completed ? "success" : "failed");
      ESP_MAIL_PRINTF("Date/Time: %s\n", MailClient.Time.getDateTimeString(result.timestamp, "%B %d, %Y %H:%M:%S").c_str());
      ESP_MAIL_PRINTF("Recipient: %s\n", result.recipients.c_str());
      ESP_MAIL_PRINTF("Subject: %s\n", result.subject.c_str());
    }
    Serial.println("----------------\n");

    // You need to clear sending result as the memory usage will grow up.
    smtp.sendingResult.clear();
  }
}