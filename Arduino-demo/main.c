
// code chuan
#define MAX_LINE_LENGTH (64)

#define GPIO_SECOND 22
#define GPIO_INTERRUPT 21
void read_mpu(void *pvParameters);
void control(void *pvParameters);
void read_vl53l1(void *pvParameters);
void Int_INIT( void );
void pwm_task(void *pvParameters );
void IRAM_ATTR button_isr_handle( void*);
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
  while(!Serial){delay(10);}
  Int_INIT();
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
    ,  "read_mpu" // A name just for humans
    ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL        // No parameter is used
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL // Task handle is not used here
    );

  xTaskCreate(
    control
    ,  "control"
    ,  2048  // Stack size
    ,  NULL  // No parameter is used
    ,  1  // Priority
    ,  NULL // Task handle is not used here
    );
 xTaskCreate(
    read_vl53l1
    ,  "read_vl53l1"
    ,  2048  // Stack size
    ,  NULL  // No parameter is used
    ,  2  // Priority
    ,  NULL // Task handle is not used here
    );
    
     xTaskCreate(
        pwm_task
        , "pwm_task "
        , 2048
        , NULL
        , 1
        , NULL);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  Serial.printf("\nAnything you write will return as echo.\nMaximum line length is %d characters (+ terminating '0').\nAnything longer will be sent as a separate line.\n\n", MAX_LINE_LENGTH-1);
}

void loop(){
  // Loop is free to do any other work

  delay(1000); // While not being used yield the CPU to other tasks
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void control(void *pvParameters){  // This is a task.
   mpu_data message_1;
   vl53l1_data message_2;
   
  for (;;){ // A Task shall never return or exit.
    // One approach would be to poll the function (uxQueueMessagesWaiting(QueueHandle) and call delay if nothing is waiting.
    // The other approach is to use infinite time to wait defined by constant `portMAX_DELAY`:
    if(QueueHandle != NULL && QueueHandle_1 != NULL){ // Sanity check just to make sure the queue actually exists
      int ret = xQueueReceive(QueueHandle, (void *)&message_1, portMAX_DELAY);
      int ret_1 = xQueueReceive(QueueHandle_1, (void *)&message_2, portMAX_DELAY);
      if(ret == pdPASS && ret_1 == pdPASS){
      index_safety = (message_2.distance*message_2.distance)/(message_1.accelerometerX+message_1.accelerometerY-message_1.accelerometerZ)
        +((message_2.distance*message_2.distance)/(message_1.gyroscopeX+message_1.gyroscopeY-message_1.gyroscopeZ)); 
      
      Serial.printf(" data of mpu read from queue : %f %f %f %f %f %f\n", message_1.accelerometerX, message_1.accelerometerY,message_1.accelerometerZ,message_1.gyroscopeX, message_1.gyroscopeY,  message_1.gyroscopeZ );
      Serial.printf(" data of vl53l1 read from queue : %f \n", message_2.distance); 
      Serial.printf(" Index safety is: %f \n", index_safety);
      if( index_safety > 30.0 )
      {
         Serial.println("ESC TURN ON \n ");
          digitalWrite(GPIO_SECOND, HIGH);
      }
      else 
      {
        Serial.println("ESC TURN OFF \n ");
      }
      }else if(ret == pdFALSE | ret_1 == pdFALSE){
        Serial.println("The `TaskWriteToSerial` was unable to receive data from the Queue");
      }
      
    } // Sanity check
  }
  vTaskDelay( 1000 /portTICK_PERIOD_MS ); // Infinite loop
}

      

void read_mpu(void *pvParameters){  // This is a task.
  mpu_data message;
  message.accelerometerX = 3.5;
  message.accelerometerY = 3.5;
  message.accelerometerZ = 3.5;
  message.gyroscopeX = 4.0;
  message.gyroscopeY = 4.0;
  message.gyroscopeZ = 4.0;

  for (;;){
      if(QueueHandle != NULL && uxQueueSpacesAvailable(QueueHandle) > 0){
        int ret = xQueueSendToBack(QueueHandle, (void*) &message, 0);
        if(ret == pdTRUE){
          Serial.println("The data mpu send to the Queue");
          // The message was successfully sent.
        }else if(ret == errQUEUE_FULL){
          // Since we are checking uxQueueSpacesAvailable this should not occur, however if more than one task should
          //   write into the same queue it can fill-up between the test and actual send attempt
          Serial.println("The `TaskReadFromSerial` was unable to send data into the Queue");
        } 
    }else{
      delay(1000); // Allow other tasks to run when there is nothing to read
    } 
  //  message.accelerometerX += 1/2;
  // message.accelerometerY += 1/2;
  // message.accelerometerZ += 1/2;
  // message.gyroscopeX += 1/2;
  // message.gyroscopeY += 1/2;
  // message.gyroscopeZ += 1/2;
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
void pwm_task( void *pvParameters)
 {
    
    //bool buttonPressed = false;
    while(1) {
      if( xSemaphoreTake( xSemaphore , portMAX_DELAY) == pdTRUE )
      {
        if(index_safety < 30 && index_safety >25 ) {
            
            Serial.println(" Thurst_1 and Thurst_2 is increating \n");
            }
          else if(index_safety < 35 && index_safety  > 30  )
          {
           
            Serial.println(" Thurst_3 and Thurst_4 is increating \n ");
          }
          else if( index_safety < 40 && index_safety > 35 )
          {
            Serial.println(" Thurst_1 and Thurst_3 is increating \n ");
          }
          else if(index_safety < 45 && index_safety > 40)
          {
            Serial.println(" Thurst_2 and Thurst_4 is increating \n ");
          }
        } 
           
       
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    } 



void Int_INIT( void )
{
  pinMode( GPIO_SECOND , OUTPUT);
  digitalWrite( GPIO_SECOND, LOW);
  pinMode(GPIO_INTERRUPT, INPUT_PULLUP);
  digitalWrite( GPIO_INTERRUPT, LOW);
  attachInterrupt(GPIO_INTERRUPT, button_isr_handle , RISING);
}




