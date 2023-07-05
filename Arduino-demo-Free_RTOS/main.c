
// code chuan


#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>
#include "SPIFFS.h"
#include "FS.h"
#include <Wire.h>


const char* ssid = "13traicaT6";
const char* password = "13traicat6";


// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;
static float motorForces[4];

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;
 #define MAX_LINE_LENGTH (64)
#define deltat 100
#define armLength  0.82f
// led ////
#define LED_PIN_1  18 // ESP32 pin GIOP18 connected to LED
#define LED_PIN_2  5
#define LED_PIN_3  17
#define LED_PIN_4  16
int brightness[4];
int fadeAmount = 0;


#define MAX_LINE_LENGTH (64)
#define deltat 100
#define armLength  0.082f
static const char *TAG = "CHECK MEMORY";
float yaw, pitch, roll;
#define GPIO_SECOND 19
#define GPIO_INTERRUPT 35
void read_mpu(void *pvParameters);
void control(void *pvParameters);
void read_vl53l1(void *pvParameters);
void Int_INIT( void );
void pwm_task(void *pvParameters );
void leb_blink();
void IRAM_ATTR button_isr_handle( void*);
void initSPIFFS();
String getGyroReadings();
String getAccReadings();
String getTemperature();
void initWiFi();
void Int_INIT_gpio();
SemaphoreHandle_t xSemaphore = NULL ;
// Define Queue handle
QueueHandle_t QueueHandle;
QueueHandle_t QueueHandle_1;
const int QueueElementSize = 10;

float index_safety;
typedef struct{
  float accelerometerX;
  float accelerometerY;
  float accelerometerZ;
  float gyroscopeX;
  float gyroscopeY;
  float gyroscopeZ;
} mpu_data;
typedef struct{
  float distance;
} vl53l1_data;

// The setup function runs once when you press reset or power on the board.
void setup() {
  // Initialize serial communication at 115200 bits per second:
 Serial.begin(115200);
  initWiFi();
  initSPIFFS();
 Int_INIT();

  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    gyroY=0;
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroY=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
  /// MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
  ///////
  while(!Serial){delay(10);}
  Int_INIT_gpio();
  // Create the queue which will have <QueueElementSize> number of elements, each of size `message_t` and pass the address to <QueueHandle>.
  QueueHandle = xQueueCreate(QueueElementSize, sizeof(mpu_data));
  QueueHandle_1 = xQueueCreate( QueueElementSize, sizeof(vl53l1_data) );
  xSemaphore = xSemaphoreCreateBinary();

  // Check if the queue was successfully created
  if(QueueHandle == NULL){
    Serial.println("Queue could not be created. Halt.");
    while(1) delay(1000); // Halt at this point as is not possible to continue
  }

  // Set up two tasks to run independently.
  xTaskCreate(
    read_mpu
    ,  "read_mpu" 
    ,  2048        
    ,  NULL        
    ,  2 
    ,  NULL 
    );

  xTaskCreate(
    control
    ,  "control"
    ,  2048  
    ,  NULL  
    ,  1  
    ,  NULL 
    );
 xTaskCreate(
    read_vl53l1
    ,  "read_vl53l1"
    ,  2048  
    ,  NULL   
    ,  2  
    ,  NULL
    );
    
     xTaskCreate(
        pwm_task
        , "pwm_task "
        , 2048
        , NULL
        , 1
        , NULL);
        
        
       
        ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
  Serial.printf("\nAnything you write will return as echo.\nMaximum line length is %d characters (+ terminating '0').\nAnything longer will be sent as a separate line.\n\n", MAX_LINE_LENGTH-1);
}


void loop() {
 if ((millis() - lastTime) > gyroDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getGyroReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getAccReadings().c_str(),"accelerometer_readings",millis());
    lastTimeAcc = millis();
  }
  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getTemperature().c_str(),"temperature_reading",millis());
    lastTimeTemperature = millis();
  }

  delay(1200); // While not being used yield the CPU to other tasks
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


  void control(void *pvParameters){  // This is a task.
   mpu_data message_1;
   vl53l1_data message_2;
   float dlength = 4.8;
  for (;;){ // A Task shall never return or exit.
    // One approach would be to poll the function (uxQueueMessagesWaiting(QueueHandle) and call delay if nothing is waiting.
    // The other approach is to use infinite time to wait defined by constant `portMAX_DELAY`:
    if(QueueHandle != NULL && QueueHandle_1 != NULL){ // Sanity check just to make sure the queue actually exists
      int ret = xQueueReceive(QueueHandle, (void *)&message_1, portMAX_DELAY);
      int ret_1 = xQueueReceive(QueueHandle_1, (void *)&message_2, portMAX_DELAY);
      if(ret == pdPASS && ret_1 == pdPASS){
      // index_safety = (message_2.distance*message_2.distance)/(message_1.accelerometerX+message_1.accelerometerY-message_1.accelerometerZ)
      //   +((message_2.distance*message_2.distance)/(message_1.gyroscopeX+message_1.gyroscopeY-message_1.gyroscopeZ)); 
      // yaw pitch row/////

  float accX = (float)message_1.accelerometerX / 16384.0; // Chia cho 16384 để đưa về gốc tọa độ [-1, 1]
  float accY = (float)message_1.accelerometerY / 16384.0;
  float accZ = (float)message_1.accelerometerZ / 16384.0;
  pitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI ;
  roll = atan2(-accX, accZ) * 180.0 / PI;

  float gyroXrad = (float)message_1.gyroscopeX / 131.0 * (PI / 180.0); // Chia cho 131 để đưa về gốc tọa độ (rad/giây)
  float gyroYrad = (float)message_1.gyroscopeY / 131.0 * (PI / 180.0);
  float gyroZrad = (float)message_1.gyroscopeZ / 131.0 * (PI / 180.0);
  yaw += gyroZrad + 0.04;

    Serial.print("Yaw: ");
  Serial.print(yaw);
 //  vTaskDelay( 200 /portTICK_PERIOD_MS );
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  // vTaskDelay( 200 /portTICK_PERIOD_MS );
  Serial.print(" Roll: ");
  Serial.println(roll);
   //vTaskDelay( 200 /portTICK_PERIOD_MS );
  Serial.println("");
  ///////
      Serial.printf(" data of mpu read from queue : %f %f %f %f %f %f\n", message_1.accelerometerX, message_1.accelerometerY,message_1.accelerometerZ,message_1.gyroscopeX, message_1.gyroscopeY,  message_1.gyroscopeZ );
      Serial.printf(" data of vl53l1 read from queue : %f \n", message_2.distance); 
       static float thrustToTorque = 0.005964552f;

// thrust = a * pwm^2 + b * pwm
static float pwmToThrustA = 0.091492681f;
static float pwmToThrustB = 0.067673604f;
static float thrust = 0.6f;
       const float arm = 0.707106781f * armLength;
  const float rollPart = 0.25f / arm *  0.02;
  const float pitchPart = 0.25f / arm *  0.05;
  const float thrustPart = 0.25f *  thrust; // N (per rotor)
  const float yawPart = 0.25f *  0.05 / thrustToTorque;
  motorForces[0] = abs(thrustPart - roll - pitch - yaw) + dlength;
  motorForces[1] = abs(thrustPart - roll + pitch + yaw);
  motorForces[2] = abs(thrustPart + roll + pitchPart - yawPart);
  motorForces[3] = abs(thrustPart + roll - pitch + yaw) + dlength;
 Serial.print("motorForces[0]:");
 Serial.println(motorForces[0]);
 Serial.print("motorForces[1]:");
 Serial.println(motorForces[1]);
 Serial.print("motorForces[2]:");
 Serial.println(motorForces[2]);
 Serial.print("motorForces[3]:");
 Serial.println(motorForces[3]);
    } // Sanity check
  }
  digitalWrite(GPIO_SECOND, LOW);
  vTaskDelay( 1000 /portTICK_PERIOD_MS ); // Infinite loop
}
}

      


      

void read_mpu(void *pvParameters){  // This is a task.


    mpu_data message;

  for (;;){
    
   sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
   message.accelerometerX = a.acceleration.x;
  message.accelerometerY = a.acceleration.y;
  message.accelerometerZ = a.acceleration.z;
  message.gyroscopeX = g.gyro.x;
  message.gyroscopeY = g.gyro.y;
  message.gyroscopeZ = g.gyro.z;
      if(QueueHandle != NULL && uxQueueSpacesAvailable(QueueHandle) > 0){
        int ret = xQueueSendToBack(QueueHandle, (void*) &message, 0);
        if(ret == pdTRUE){
          Serial.println("The data mpu send to the Queue");
          // The message was successfully sent.
        }else if(ret == errQUEUE_FULL){
          Serial.println("The `TaskReadFromSerial` was unable to send data into the Queue");
        } 
    }else{
      delay(1000);
    } 
 
  }
   

}
void read_vl53l1(void *pvParameters){  // This is a task.
  vl53l1_data message;
  message.distance = 8.0;
  

  for (;;){
      if(QueueHandle_1 != NULL && uxQueueSpacesAvailable(QueueHandle_1) > 0){
        int ret = xQueueSendToBack(QueueHandle_1, (void*) &message, 0);
        if(ret == pdTRUE){
          Serial.println("The data vl53l1 send to the Queue");
          // The message was successfully sent.
        }else if(ret == errQUEUE_FULL){
          // Since we are checking uxQueueSpacesAvailable this should not occur, however if more than one task should
          //   write into the same queue it can fill-up between the test and actual send attempt
          Serial.println("The `TaskReadFromSerial` was unable to send data into the Queue");
        } 
    }else{
      delay(1000); // Allow other tasks to run when there is nothing to read
    } // Serial buffer check
  }
  vTaskDelay( 1000 /portTICK_PERIOD_MS ); // Infinite loop
}
void button_isr_handle()
{
  xSemaphoreGiveFromISR(xSemaphore, NULL);
}
/// PWM task///////

void pwm_task( void *pvParameters)
 {

   for(;;){ 
  analogWrite(LED_PIN_1, brightness[0] +60);
  analogWrite(LED_PIN_2, brightness[1] + 60);
  analogWrite(LED_PIN_3, brightness[2] +60 );
  analogWrite(LED_PIN_4, brightness[3] +60 );

  // change the brightness for next time through the loop:
  //brightness = fadeAmount + 60 ;

  // reverse the direction of the fading at the ends of the fade:
 
    for( int i = 0 ; i < 3 ; i++)
    {
       if (brightness[i] <= 0 || brightness[i] >= 255) {
    if(motorForces[i] > 0  )
    {
    brightness[i] = motorForces[i];
    }
    else 
    {
     brightness[i]  = -(motorForces[i]);
    }
  }
  delay(30);
       
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

 }

void Int_INIT( void )
{
  pinMode(GPIO_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(GPIO_INTERRUPT, button_isr_handle , RISING);
}
void leb_blink( void )
{
  pinMode(GPIO_SECOND, OUTPUT );
  digitalWrite(GPIO_SECOND, HIGH );
  delay( 400 );
  digitalWrite( GPIO_SECOND , LOW);
  delay( 400);
}
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(" * ");
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());
}

String getGyroReadings(){
  mpu.getEvent(&a, &g, &temp);

  float gyroX_temp = g.gyro.x;
  if(abs(gyroX_temp) > gyroXerror)  {
    gyroX += gyroX_temp/50.00;
  }
  
  float gyroY_temp = g.gyro.y;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp/70.00;
  }

  float gyroZ_temp = g.gyro.z;
  if(abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp/90.00;
  }

  readings["gyroX"] = String(gyroX);
  readings["gyroY"] = String(gyroY);
  readings["gyroZ"] = String(gyroZ);

  String jsonString = JSON.stringify(readings);
  return jsonString;
}
String getAccReadings() {
  mpu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  readings["accX"] = String(accX);
  readings["accY"] = String(accY);
  readings["accZ"] = String(accZ);
  String accString = JSON.stringify (readings);
  return accString;
}

String getTemperature(){
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
  return String(temperature);
}

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}
void Int_INIT_gpio( void )
{
  pinMode(GPIO_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(GPIO_INTERRUPT, button_isr_handle , RISING);
}
