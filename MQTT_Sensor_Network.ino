#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "driver/adc.h"
#include "D:/credentials.h"
#include <MPU6050_light.h>


#define I2C_SERIAL_PIN 21
#define I2C_CLK_PIN 22
#define I2C_FREQUENCY 3400000

#include <PubSubClient.h>
#include <WiFi.h>
#include "D:/credentials.h"

#define BUILTIN_LED 2

// MQTT defines
#define ENV 3 // 1 for testing at home, 2 for wlan at HAW, 3 for mobile hotspot (laptop)
#if ENV == 1
  #define SSID SSID_HOME
  #define WIFI_PW WIFI_PW_HOME
  #define MQTT_SERVER_IP "lennardjoensson.de"
#elif ENV == 2
  #define SSID "Muecke"
  #define WIFI_PW "mosquitto23"
  #define MQTT_SERVER_IP "192.168.42.1"
#elif ENV == 3
  #define SSID "vanilla-coke"
  #define WIFI_PW "terminus123"
  #define MQTT_SERVER_IP "192.168.137.1"
#endif

#define MQTT_PORT 1883

#define MODE_MEASUREMENT 99 // mode for measuring the pressure
#define MODE_CONTINOUS 98 // normal mode for contiuous measurement of sensors 
#define MODE_MPU6050_ONLY 97

#define EXEC_MODE MODE_MPU6050_ONLY // define the mode of the uc

// Analog read defines
#define ANALOG_X ADC1_CHANNEL_4
#define ANALOG_Y ADC1_CHANNEL_5

// BMP280 defines

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor(); 

// Interrupt pins defines
#define JOYSTICK_SW_PIN 15
#define EMERGENCY_SWITCH 25

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

MPU6050 mpu(Wire);

enum States : char {
  Ruhe = 0,
  Fernsteuerung,
  Transport
};
States state; // states we want to classify

void publishState(int state){
  snprintf(msg, MSG_BUFFER_SIZE, "%d", state);
  client.publish("state", msg);
}

void getAccData(){
  float accX = mpu.getAccX();
  float accY = mpu.getAccY();
  float accZ = mpu.getAccZ();

  // publish data
  snprintf(msg, MSG_BUFFER_SIZE, "%f", accX);
  client.publish("accelerationX", msg);
  snprintf(msg, MSG_BUFFER_SIZE, "%f", accY);
  client.publish("accelerationY", msg);
  snprintf(msg, MSG_BUFFER_SIZE, "%f", accZ);
  client.publish("accelerationZ", msg);
}

void getGyroData(){
  float gyroX = mpu.getGyroX();
  float gyroY = mpu.getGyroY();
  float gyroZ = mpu.getGyroZ();

  // publish data
  snprintf(msg, MSG_BUFFER_SIZE, "%f", gyroX);
  client.publish("gyrometerX", msg);
  snprintf(msg, MSG_BUFFER_SIZE, "%f", gyroY);
  client.publish("gyrometerY", msg);
  snprintf(msg, MSG_BUFFER_SIZE, "%f", gyroZ);
  client.publish("gyrometerZ", msg);
}

void getAngles(){
  float angleX = mpu.getAngleX();
  float angleY = mpu.getAngleY();
  float angleZ = mpu.getAngleZ();

  // publish data
  snprintf(msg, MSG_BUFFER_SIZE, "%f", angleX);
  client.publish("angleX", msg);
  snprintf(msg, MSG_BUFFER_SIZE, "%f", angleY);
  client.publish("angleY", msg);
  snprintf(msg, MSG_BUFFER_SIZE, "%f", angleZ);
  client.publish("angleZ", msg); 
}

void getAccAngleData(){
  float accAngleX = mpu.getAccAngleX();
  float accAngleY = mpu.getAccAngleY();

  snprintf(msg, MSG_BUFFER_SIZE, "%f", accAngleX);
  client.publish("accAngleX", msg); 
  snprintf(msg, MSG_BUFFER_SIZE, "%f", accAngleY);
  client.publish("accAngleY", msg); 
}


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, WIFI_PW);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("Message arrived [%s] ", topic);

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  
  if(strcmp(topic, "new_state") == 0){
    int state_i = payload[0] - '0' ;
    state = (States)state_i;
    Serial.printf("New state is %d.\n", state);
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("new_state");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

static bool joy_switch_pressed = false;
static bool emer_switch_pressed = false;

void ICACHE_RAM_ATTR joy_switch_isr(){
  joy_switch_pressed = true;
}
void ICACHE_RAM_ATTR emer_switch_isr(){
  emer_switch_pressed = true;
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output

  // ADC init
  adc1_config_width(ADC_WIDTH_BIT_10);
  adc1_config_channel_atten(ANALOG_X, ADC_ATTEN_MAX);
  adc1_config_channel_atten(ANALOG_Y, ADC_ATTEN_MAX);

  // Serial interface must be initialized after ADC channels
  Serial.begin(115200);
  delay(5);
  Serial.println("MQTT Sensor Network Node");

  // MQTT init
  setup_wifi();
  client.setServer(MQTT_SERVER_IP, MQTT_PORT);
  client.setCallback(callback);

  // temp & pres sensor init
  bmp.begin();  // Initializing BMP280 sensor
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
    Adafruit_BMP280::STANDBY_MS_500 /* Standby time. */
  );
  bmp_temp->printSensorDetails();
  // Wire.begin(I2C_SERIAL_PIN, I2C_CLK_PIN, I2C_FREQUENCY);

  // Attach interrupts on pins
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  pinMode(EMERGENCY_SWITCH, INPUT_PULLUP);
  // attachInterrupt(JOYSTICK_SW_PIN, joy_switch_isr, FALLING);
  // attachInterrupt(EMERGENCY_SWITCH, emer_switch_isr, FALLING);

  // Initialize MPU6050
  byte status = mpu.begin();
  if(status){
    Serial.println("Error while initializing MPU6050");
  }
  mpu.calcOffsets(true,true); // gyro and accelero
}



void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 100) {
    lastMsg = now;
    mpu.update();
    sensors_event_t temp_event, pressure_event;
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);

    float temp = temp_event.temperature;
    float pres = pressure_event.pressure;
    //Serial.printf("Temperature: %.2f °C\n", temp);
    //Serial.printf("Pressure: %.2f hPa \n", pres);

#if EXEC_MODE == MODE_MEASUREMENT
  static int times_measured = 300;
  if(times_measured  < 300){
      snprintf(msg, MSG_BUFFER_SIZE, "%f", pres);
      client.publish("pressure", msg);
      Serial.printf("%d: %f \n", times_measured, pres);
      times_measured++; 
  }

if(adc1_get_raw(ANALOG_X) < 10){
  times_measured = 0; 
  Serial.println("Starting measurement...");
}
#endif

#if EXEC_MODE == MODE_CONTINOUS // %%%%%%%%% ----- %%%%%%%%%%%%
    
    snprintf(msg, MSG_BUFFER_SIZE, "%f", pres);
    client.publish("pressure", msg);
    
    snprintf(msg, MSG_BUFFER_SIZE, "%f", temp);
    client.publish("temperature", msg);


    int sensorValueX = adc1_get_raw(ANALOG_X);
    //Serial.printf("X-Tilt: %d \n", sensorValueX);
    snprintf(msg, MSG_BUFFER_SIZE, "%d", sensorValueX);
    client.publish("tiltX", msg);

    int sensorValueY = adc1_get_raw(ANALOG_Y);
    //Serial.printf("Y-Tilt: %d \n", sensorValueY);
    snprintf(msg, MSG_BUFFER_SIZE, "%d", sensorValueY);
    client.publish("tiltY", msg);

    if(joy_switch_pressed){
      Serial.println("Joystick was pressed.");
      snprintf(msg, MSG_BUFFER_SIZE, "ON", NULL);
      client.publish("joySwitch", msg);
      joy_switch_pressed = false;
    }

    if(emer_switch_pressed){
      Serial.println("Joystick was pressed.");
      snprintf(msg, MSG_BUFFER_SIZE, "ON", NULL);
      client.publish("emergencySwitch", msg);
      emer_switch_pressed = false;
    }
#endif // %%%%%%%%%%%% --- %%%%%%%%%%%

#if EXEC_MODE == MODE_MPU6050_ONLY
  static int times_measured = 30;
  if(times_measured  < 30){
    getAccData();
    getGyroData();
    getAngles();
    getAccAngleData();
    publishState(state);
    if(times_measured == 29){
      Serial.println("Measurement done.");
    }
    times_measured++; 
  }
  if(digitalRead(EMERGENCY_SWITCH) == LOW){
    times_measured = 0; 
    Serial.println("Beginning measurement...");
  }
#endif // %%%%%%%%%%%% --- %%%%%%%%%%%

  }
}
