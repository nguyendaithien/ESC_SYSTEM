
#include <Wire.h>
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_VL53L1X.h>

QueueHandle_t xQueue_MPU  ;
QueueHandle_t xQueue_VL53L1 ;
#define MAX_QUEUE_SIZE 64
//Adafruit_MPU6050 mpu;
//Adafruit_VL53L1X vl53l1;

typedef struct  
{
  float accelerometerX;
  // float accelerometerY;
  // float accelerometerZ;
  // float gyroscopeX;
  // float gyroscopeY;
  // float gyroscopeZ;
}MPU_data;
typedef struct  
{
  float distance;
}VL53L1_data;
int i = 1;
void task_read_data_MPU( void *pvParamerters )
{
    MPU_data mpu_data;
    
 // mpu.begin();

  for(;;) {
    //sensors_event_t a, g, temp;
   // mpu.getEvent(&a, &g, &temp);
     mpu_data.accelerometerX = i*0.01;
    //  mpu_data.accelerometerY = i*0.02; 
    //  mpu_data.accelerometerZ =i*0.03;
    //  mpu_data.gyroscopeX = i*0.04;
    //  mpu_data.gyroscopeY = i*0.05;
    //  mpu_data.gyroscopeZ = i*0.06;

    //  mpu_data.accelerometerX = a.acceleration.x;
    //  mpu_data.accelerometerY = a.acceleration.y; 
    //  mpu_data.accelerometerZ = a.acceleration.z;
    //  mpu_data.gyroscopeX = g.gyro.x;
    //  mpu_data.gyroscopeY = g.gyro.y;
    //  mpu_data.gyroscopeZ = g.gyro.z;
    int ret = xQueueSend(xQueue_MPU, (void*) &mpu_data, 0);
        if(ret == pdTRUE){
          
    
    Serial.printf(" Data of MPU6050 send to FIFO : %f \n",mpu_data.accelerometerX);
    //, mpu_data.accelerometerY, mpu_data.accelerometerZ );
    vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    
  }
}

void task_read_data_VL53L1(void *pvParemeters)
{
 
  VL53L1_data vl53l1_data[25] = { 3.5, 4.3, 6.5 ,8.6, 4.3, 6.0, 4.8, 5.6, 2.3, 4.7, 3.5, 4.3, 6.5, 8.6, 4.3, 6.0, 4.8, 5.6, 2.3, 4.7, 3.5, 4.3, 6.5, 8.6, 4.3 };
  for(;;)
  {
    
 int ret = xQueueSend(xQueue_MPU, (void*) &vl53l1_data, 0);
        if(ret == pdTRUE){
        
  Serial.printf("Data of VL53L1 send to FIFO: %f", vl53l1_data[1] );
        
  vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}
}


void task_control( void *pvParameters )
{
  float index_safety;
 
      MPU_data mpu_data;
      VL53L1_data vl53l1;     
      for(;;)
      {
        int ret =xQueueReceive(xQueue_MPU, &mpu_data, 0) ;
        int ret_1 =xQueueReceive(xQueue_MPU, &vl53l1, 0) ;
      if(ret == pdPASS && ret_1 == pdPASS ){
        index_safety = ((vl53l1.distance*vl53l1.distance)/(mpu_data.accelerometerX));
        // index_safety =  ((vl53l1.distance*vl53l1.distance)/(mpu_data.accelerometerX+mpu_data.accelerometerY-mpu_data.accelerometerZ))
        // +((vl53l1.distance*vl53l1.distance)/(mpu_data.gyroscopeX+mpu_data.gyroscopeY-mpu_data.gyroscopeZ));
        if( index_safety > 50.0 )
        {
          Serial.println(" ESC is ON !!!!!!!! , Drone will soon ballance \n");
        }
        else 
        {
          Serial.println(" ESC is OFF !!!!!!!! , Drone being ballance \n");
        }
          vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
  
}
}
void setup (void)
{
  Serial.begin(9600);
  while(!Serial){delay(10);}
  xQueue_MPU = xQueueCreate(MAX_QUEUE_SIZE, sizeof(MPU_data));
  //xQueue_VL53L1 = xQueueCreate(MAX_QUEUE_SIZE, sizeof(float));

  if(xQueue_MPU == NULL){
    Serial.println("Queue could not be created. Halt.");
    while(1) delay(1000); // Halt at this point as is not possible to continue
  }
  xTaskCreate( task_read_data_MPU,"task_read_data_MPU", 128, (void *)xQueue_MPU, 5, NULL );
  xTaskCreate( task_read_data_VL53L1,"task_read_data_VL53L1",128, (void *)xQueue_VL53L1, 4, NULL );
  xTaskCreate( task_control,"task_control", 128, NULL, 6, NULL );
 // mpu.begin(0x52);
  //mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
 // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
 // mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
}
void loop()
{
  delay(1000);

}

