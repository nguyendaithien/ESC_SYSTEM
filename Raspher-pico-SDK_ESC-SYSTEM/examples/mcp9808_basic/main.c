#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "i2c_dma.h"
#include <hardware/pwm.h>
#include "MahonyAHRS.h"
#include "VL53L1X_api.h"
#include "VL53L1X_platform.h"
// #include "vector.h"
#include "semphr.h"
#include "math3d.h"


static const uint8_t MCP6050_DEV_ADDR  = 0x68;
static const uint8_t MCP6050_POW_ADDR  = 0x6B;
static const uint8_t MCP6050_POW_RESET = 0x80;
static const uint8_t MCP6050_POW_WAKE  = 0x00;
static const uint8_t MCP6050_ACC_ADDR  = 0x3B;
static const uint8_t MCP6050_GYR_ADDR  = 0x43;
static const uint8_t MCP6050_WHO_AMI   = 0x75;

static float tau_xy = 0.3;
static float zeta_xy = 0.85; // this gives good performance down to 0.4, the lower the more aggressive (less damping)

static float tau_z = 0.3;
static float zeta_z = 0.85;

// time constant of body angle (thrust direction) control
static float tau_rp = 0.25;
// what percentage is yaw control speed in terms of roll/pitch control speed \in [0, 1], 0 means yaw not controlled
static float mixing_factor = 1.0;

// time constant of rotational rate control
static float tau_rp_rate = 0.015;
static float tau_yaw_rate = 0.0075;

// minimum and maximum thrusts
static float coll_min = 1;
static float coll_max = 18;
static float thrust_reduction_fairness = 0.25;

// minimum and maximum body rates
static float omega_rp_max = 30;
static float omega_yaw_max = 10;
static float heuristic_rp = 12;
static float heuristic_yaw = 5;

static float mass = 0.077f;

static struct mat33 CRAZYFLIE_INERTIA =
    {{{48.6e-6f, 1.83e-6f, 1.82e-6f},
      {1.83e-6f, 52.6e-6f, 3.8e-6f},
      {1.82e-6f, 3.8e-6f, 64.3e-6f}}};

#define I2C_DEV_ADDR 0x29

static SemaphoreHandle_t imu_write,lidar_write,rc_write;
  

quatf orientation;
float lidar_height=0,yaw_sepoint=0.0,height_sepoint=1.0;
vec3f accel_w={0.0,0.0,0.0},rotrate={0.0,0.0,0.0},velocity={0.0,0.0,0.0};

static void blink_led_task(void *args) {
  (void) args;
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, 1);
  gpio_put(PICO_DEFAULT_LED_PIN, !PICO_DEFAULT_LED_PIN_INVERTED);

  while (true) {
    gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
    vTaskDelay(500);
  }
}


static void mpu6050_task(void *args) {
  i2c_dma_t *i2c_dma = (i2c_dma_t *) args;
  
  uint8_t hello;
  int status ;
  status = i2c_dma_write_byte(i2c_dma,MCP6050_DEV_ADDR,MCP6050_POW_ADDR,MCP6050_POW_RESET);
  vTaskDelay(100);
  status = i2c_dma_write_byte(i2c_dma,MCP6050_DEV_ADDR,MCP6050_POW_ADDR,MCP6050_POW_WAKE);
  status = i2c_dma_read_byte(i2c_dma,MCP6050_DEV_ADDR,MCP6050_WHO_AMI,&hello);
  float accum[3]={0.0f,0.0f,0.0f};
  while(1){
    uint16_t raw_accx,raw_accy,raw_accz,raw_gyrx,raw_gyry,raw_gyrz;
    status+= i2c_dma_read_word_swapped(i2c_dma,MCP6050_DEV_ADDR,MCP6050_ACC_ADDR  ,&raw_accx);
    status+= i2c_dma_read_word_swapped(i2c_dma,MCP6050_DEV_ADDR,MCP6050_ACC_ADDR+2,&raw_accy);
    status+= i2c_dma_read_word_swapped(i2c_dma,MCP6050_DEV_ADDR,MCP6050_ACC_ADDR+4,&raw_accz);
    status+= i2c_dma_read_word_swapped(i2c_dma,MCP6050_DEV_ADDR,MCP6050_GYR_ADDR  ,&raw_gyrx);
    status+= i2c_dma_read_word_swapped(i2c_dma,MCP6050_DEV_ADDR,MCP6050_GYR_ADDR+2,&raw_gyry);
    status+= i2c_dma_read_word_swapped(i2c_dma,MCP6050_DEV_ADDR,MCP6050_GYR_ADDR+4,&raw_gyrz);

    int16_t ax = raw_accx;
    int16_t ay = raw_accy;
    int16_t az = raw_accz;

    int16_t gx = raw_gyrx;
    int16_t gy = raw_gyry;
    int16_t gz = raw_gyrz;

    float accel[3]={0.0f,0.0f,0.0f},gyros[3]={0.0f,0.0f,0.0f};
    accel[1] =  ( 777.5f - ax)/1677.0f;
    accel[0] =  ( 30.5f + ay)/1673.0f;
    accel[2] =  ( 645.0f + az)/1705.0f;

    gyros[1] = (-gx + 4328.0f)/6366.0f;
    gyros[0] = ( gy - 452.0f )/6366.0f;
    gyros[2] = ( gz - 73.0f  )/6366.0f; 
    MahonyAHRSupdateIMU(gyros[0], gyros[1], gyros[2], accel[0], accel[1], accel[2]);
    quatf new_orientation = mkquat(q1, q2, q3 , q0);
    vec3f new_acc = mkvec(accel[0], accel[1], accel[2]);
    vec3f new_rate = mkvec(gyros[0], gyros[2], gyros[3]);
    vec3f new_acc_w = qvrot(new_orientation , new_acc );
    new_acc_w.z -= 9.8;
    // mprintf("raw %-2.4f %-2.4f %-2.4f\n", accel[0] ,accel[1] ,accel[2]);
    // mprintf("new %-2.4f %-2.4f %-2.4f\n", new_acc_w.x , new_acc_w.y , new_acc_w.z);
    // PrintVector(new_acc_w);
    xSemaphoreTake(imu_write, portMAX_DELAY);
    orientation = new_orientation;
    accel_w = new_acc_w;
    rotrate = new_rate;
    xSemaphoreGive(imu_write);
    // mprintf("%-2.4f %-2.4f %-2.4f %-2.4f\n", q0 ,q1 ,q2 ,q3);
    vTaskDelay(10);
  }
  
}

static void vlx51_task(void *args) {
  
  i2c_init(i2c1, 400 * 1000);
  gpio_set_function(2, GPIO_FUNC_I2C);
  gpio_set_function(3, GPIO_FUNC_I2C);
  gpio_pull_up(2);
  gpio_pull_up(3);
  uint8_t sensorState,status_lid;
  do {
    status_lid += VL53L1X_BootState(I2C_DEV_ADDR, &sensorState);
    VL53L1X_WaitMs(I2C_DEV_ADDR, 2);
  } while (sensorState == 0);
  sleep_ms(200);

  // Initialize and configure sensor
  status_lid = VL53L1X_SensorInit(I2C_DEV_ADDR);
  status_lid += VL53L1X_SetDistanceMode(I2C_DEV_ADDR, 2);
  status_lid += VL53L1X_SetTimingBudgetInMs(I2C_DEV_ADDR, 100);
  status_lid += VL53L1X_SetInterMeasurementInMs(I2C_DEV_ADDR, 100);
  status_lid += VL53L1X_StartRanging(I2C_DEV_ADDR);

  bool first_range = true;
    while(1){
        VL53L1X_Status_t status;
        VL53L1X_Result_t results;
        uint8_t dataReady;
            do {
                status = VL53L1X_CheckForDataReady(I2C_DEV_ADDR, &dataReady);
                vTaskDelay(10);
            } while (dataReady == 0);
            status += VL53L1X_GetResult(I2C_DEV_ADDR, &results);
            // mprintf("Status = %2d, dist = %5d\n",results.status, results.distance);
            xSemaphoreTake(lidar_write, portMAX_DELAY);
            lidar_height = results.distance/1000.0;
            // mprintf("lidar_height = %ld\n",lidar_height);
            xSemaphoreGive(lidar_write);
            status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
            if (first_range) {  // Clear twice on first measurement
              status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
              first_range = false;
            }
            vTaskDelay( 100);
        }  
}

static uint32_t idleThrust = 0;
static float armLength = 0.082f; // m;
static float thrustToTorque = 0.005964552f;

// thrust = a * pwm^2 + b * pwm
static float pwmToThrustA = 0.091492681f;
static float pwmToThrustB = 0.067673604f;


vec4f powerDistributionForceTorque(float thrust, vec3f *torque) {
  static float motorForces[4];

  const float arm = 0.707106781f * armLength;
  const float rollPart = 0.25f / arm *  torque->x;
  const float pitchPart = 0.25f / arm *  torque->y;
  const float thrustPart = 0.25f *  thrust; // N (per rotor)
  const float yawPart = 0.25f *  torque->y / thrustToTorque;

  motorForces[0] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[1] = thrustPart - rollPart + pitchPart + yawPart;
  motorForces[2] = thrustPart + rollPart + pitchPart - yawPart;
  motorForces[3] = thrustPart + rollPart - pitchPart + yawPart;
  vec4f motorForce = {motorForces[0], motorForces[1], motorForces[2], motorForces[3]};
  float list[4];
  for (int motorIndex = 0; motorIndex < 4; motorIndex++) {
    float motorForce = motorForces[motorIndex];
    if (motorForce < 0.0f) {
      motorForce = 0.0f;
    }

    float motor_pwm = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * motorForce)) / (2.0f * pwmToThrustA);
    list[motorIndex] = motor_pwm ;//* UINT16_MAX;
  }
  vec4f motorPWM ={list[0],list[1],list[2],list[3]};
  pvec4(motorPWM, "motor");
  pvec4(motorForce , "force");
  return motorPWM;
}

static void control_task(void *args) {
  while(1){
    yaw_sepoint = 0.0;
    vec3f a_des = {0.0, 0.0, 9.8};
    xSemaphoreTake(imu_write, portMAX_DELAY);
    xSemaphoreTake(lidar_write, portMAX_DELAY);
    quatf current_att = orientation;
    vec3f curr_omega = rotrate;
    vec3f curr_acc = accel_w;
    xSemaphoreGive(lidar_write);
    xSemaphoreGive(imu_write);
    vec3f ldray = qvrot(current_att , vbasis(2));
    float height = lidar_height * ldray.z ;
    // mprintf("height %lf %lf \n",height,lidar_height);
    quatf attDesired = computeDesiredQuat(yaw_sepoint, a_des);
    quatf attInverse = qinv(current_att);
    quatf attError = qqmul(attInverse, attDesired);

    // correct rotation
    if (attError.w < 0) {
      attError = qneg(attError);
      attDesired = qqmul(current_att, attError);
    }

    attError = qnormalize(attError);
    attDesired = qnormalize(attDesired);
   
    vec3f control_omega;
    control_omega.x = attError.x;
    control_omega.y = attError.y;
    control_omega.z = attError.z;
    vscl(2.0f / tau_rp , control_omega);
    float scaling = 1;
    scaling = fmax(scaling, fabsf(control_omega.x) / omega_rp_max);
    scaling = fmax(scaling, fabsf(control_omega.y) / omega_rp_max);
    scaling = fmax(scaling, fabsf(control_omega.z) / omega_yaw_max);

    vscl(1.0f / scaling , control_omega);
    
    vec3f omegaErr =vsub(control_omega , curr_omega);
    vec3f omegaCoeff = mkvec(tau_rp_rate,tau_rp_rate,tau_yaw_rate);
    omegaErr = veltdiv(omegaErr,omegaCoeff);
    // update the commanded body torques based on the current error in body rates
    vec3f control_torque = mvmul(CRAZYFLIE_INERTIA, omegaErr);
    float control_thrust = vdot(a_des, ldray)*mass;// ham vdot tinh khoang cach hai diem A,B
    powerDistributionForceTorque(control_thrust, &control_torque);
    // pvec(control_omega , "OMEGA");

    // PrintVector(velocity); 
    vTaskDelay(10);
  }
  
}
 static void pwm_task(void *args) {
    const uint LED_PIN_0 = 0;
    const uint LED_PIN_1 = 1;
    const uint LED_PIN_2 = 14;
    const uint LED_PIN_3 = 15;
    const uint MAX_PWM_LEVEL = 52000;

    gpio_set_function(LED_PIN_0, GPIO_FUNC_PWM);
    gpio_set_function(LED_PIN_1, GPIO_FUNC_PWM);
    gpio_set_function(LED_PIN_2, GPIO_FUNC_PWM);
    gpio_set_function(LED_PIN_3, GPIO_FUNC_PWM);

    uint sliceNum_0 = pwm_gpio_to_slice_num(LED_PIN_0);
    uint sliceNum_1 = pwm_gpio_to_slice_num(LED_PIN_1);
    uint sliceNum_2 = pwm_gpio_to_slice_num(LED_PIN_2);
    uint sliceNum_3 = pwm_gpio_to_slice_num(LED_PIN_3);

    pwm_config config = pwm_get_default_config();

    pwm_init(sliceNum_0, &config, true);
    pwm_init(sliceNum_1, &config, true);
    pwm_init(sliceNum_2, &config, true);
    pwm_init(sliceNum_3, &config, true);

    int level = 0;
    bool up = true;

    while (true) {
        pwm_set_gpio_level(LED_PIN_0, level);
        pwm_set_gpio_level(LED_PIN_1, level);
        pwm_set_gpio_level(LED_PIN_2, level);
        pwm_set_gpio_level(LED_PIN_3, level);

        if (level <= 0) {
            up = true;
        } else if (level >= MAX_PWM_LEVEL-10) {
            up = false;
        }
        level = up ? level + 5 : level -5;

        // mprintf("Level: %d\n", level);
        sleep_us(50);
    }
  }

int main(void) {
  stdio_init_all();
  sleep_ms(500);
  static i2c_dma_t *i2c0_dma;
  const int rc = i2c_dma_init(&i2c0_dma, i2c0, (400 * 1000), 4, 5);
  if (rc != PICO_OK) {
    mprintf("can't configure I2C0\n");
    return rc;
  }

  imu_write = xSemaphoreCreateMutex();
  lidar_write = xSemaphoreCreateMutex();
  rc_write = xSemaphoreCreateMutex();

  
  xTaskCreate(
    blink_led_task,
    "blink-led-task",
    configMINIMAL_STACK_SIZE,
    NULL,
    -1,
    NULL
  );

  xTaskCreate(
    mpu6050_task,
    "mpu6050-task",
    configMINIMAL_STACK_SIZE,
    i2c0_dma,
    configMAX_PRIORITIES - 2,
    NULL
  );

  xTaskCreate(
    vlx51_task,
    "vlx51-task",
    configMINIMAL_STACK_SIZE,
    NULL,
    configMAX_PRIORITIES - 2,
    NULL
  );

  xTaskCreate(
    control_task,
    "control-task",
    configMINIMAL_STACK_SIZE,
    NULL,
    configMAX_PRIORITIES - 2,
    NULL
  );

  xTaskCreate(
    pwm_task,
    "pwm-task",
    configMINIMAL_STACK_SIZE,
    NULL,
    configMAX_PRIORITIES - 2,
    NULL
  );

  vTaskStartScheduler();
}
