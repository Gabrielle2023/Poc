#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"

#define uS_TO_mS_FACTOR 1000ULL
#define TIME_TO_SLEEP 30000
RTC_DATA_ATTR unsigned int last_timer_wake_up = 0;

RTC_DATA_ATTR unsigned int ulp_adc_value = 0;
unsigned int ulp_adc_threshold = 100;  // milivots 176 ou 707 ?

const ulp_insn_t ulp_adc_program[] = {
    I_DELAY(32000),                   // Wait until ESP32 goes to deep sleep
    M_LABEL(1),                       // LABEL 1
    I_ADC(R0, 0, 0),                  // Read ADC value to reg. R0
    M_BGE(2, ulp_adc_threshold),      // If average ADC value from reg. R0 is higher or equal than threshold, go to LABEL 2
    M_BX(1),                          // Go to LABEL 1
    M_LABEL(2),                       // LABEL 3
    I_MOVI(R1, ((unsigned int)&ulp_adc_value - (unsigned int)RTC_SLOW_MEM) / 4), // Set reg. R1 to adress of ulp_adc_value
    I_ST(R0, R1, 0),                                                             // Copy result of ADC to R1 adress, so ulp_adc_value
    I_WAKE(),                                                                    // Wake up ESP32
    I_END(),                                                                     // Stop ULP program timer
    I_HALT() 
};

unsigned int get_time_ms() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

//////INIT DE BASE
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#define WLAN_SSID       "InssV2"
#define WLAN_PASS       "tozyyyyy"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "Inssso"
#define AIO_KEY         "aio_VYaE20M5aYC6iVK44yeD5yf7F2iz"

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
#define SEUIL 19
float temperature = 0;
//////INIT DE BASE
void setup()
{
    Serial.begin(115200);

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch(wakeup_reason)
    {
      
      case ESP_SLEEP_WAKEUP_UNDEFINED : // First run, initialize ULP program
        Serial.printf("First run: %d\n",wakeup_reason); 
        init_ulp_program();
        last_timer_wake_up = get_time_ms();
        break;

      case ESP_SLEEP_WAKEUP_TIMER : 
        Serial.println("Wakeup caused by Timer"); 
        last_timer_wake_up = get_time_ms();
        // ***** HERE YOUR SKETCH *****
        initialisation_basics();
        BME_check();
        MQTT_connect();
        MQTT_publish_data();
        // ***** HERE YOUR SKETCH *****

        break;

      case ESP_SLEEP_WAKEUP_ULP : 
        Serial.println("Wakeup caused by ULP program"); 
        // ***** HERE YOUR SKETCH *****
        ADC_mesure_temperature();
        alerte_incendie();
        initialisation_basics();


        // ***** HERE YOUR SKETCH *****
        ulp_adc_value &= UINT16_MAX; 
        printf("ULP Value=%d was above threshold (%d)\n", ulp_adc_value, ulp_adc_threshold);

        // *** Do not forget to reactivate adc1_ulp_enable wich is disabled by adc1_get_raw
        adc1_ulp_enable();
        break;

      default : 
        Serial.printf("Wakeup was not caused by defined deep sleep mode: %d\n",wakeup_reason); 
        break;
    }

    delay(100);

    // Activate the ULP wakeup source
    ESP_ERROR_CHECK(ulp_run(0));
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

    // Activate and configure the timer wakeup source, computing the remaining time to wait (especially if the current wakeup source is not the timer)
    int remaining_timer_wait_time = max((int)(TIME_TO_SLEEP - (get_time_ms() - last_timer_wake_up)), 0);
    esp_sleep_enable_timer_wakeup(remaining_timer_wait_time * uS_TO_mS_FACTOR);

    // Go to deep sleep
    esp_deep_sleep_start();
}

void loop(){}

static void init_ulp_program()
{
    /* Configure ADC channel */
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_ulp_enable();

    /* Set ULP wake up period to 100ms */
    ulp_set_wakeup_period(0, 100 * 1000);

    size_t size = sizeof(ulp_adc_program) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_adc_program, &size);
    ADC_mesure_temperature();
}


///////////////////
void wifi_connexion(){
  delay(10);
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
}
void initialisation_basics(){
  wifi_connexion();
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
	delay(500);
}

void alerte_incendie (){
  if (temperature >= SEUIL){
    digitalWrite(LED_PIN,HIGH);
  }
  else {
      digitalWrite(LED_PIN,LOW);
  }
}