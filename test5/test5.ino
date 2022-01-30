///////////////////////////5th version of test code for MFWF/////////////////////////////
/*Version notes:
-MPU6050 needs to be updated to newer SEN-15335 9-axis sensor
-IMU may not need filter like the lidar.  Slow performance atm, will need to look into optimizing. Might be okay to just take the values, as it's an integration anyway. Could also look at manually integreating Adafruit_sensor value rather than using MPU6050 library
-Some code can still be optimized
-Specify motor output scale from 0-180
*/
#include <Servo.h> //Servo library
#include <Adafruit_Sensor.h>  //General sensor data library
#include "MPU6050_6Axis_MotionApps20.h"  //0x68 serial address
#include "I2Cdev.h"
#include "Wire.h"  //Required to talk to multiple components
#include <Adafruit_VL53L0X.h>  //0x29 serial address
#include <SimpleKalmanFilter.h>  //Filter Library
#include "MFWF_control.h"
#include "MFWF_control_private.h"

//Pin specifications 
#define ESCpin 10 //ESC PWM
#define pitchpin 9 //Servo
#define yawpin 8 //Servo
#define rollpin 7 //Servo

//Potentiometer pin, which can be used to control pitch, yaw and roll for testing
#define pot A0//Test potentiometer

Adafruit_VL53L0X lox = Adafruit_VL53L0X();


MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

//Variables
double sensorValue = 0;
double duration;
double distance;
float z; //height
float ze; //height
int potval; //Use for testing with potentiometer
float potpos; // Test potentiometer position
float pitchval; //Current pitch
float yawval; //Current yaw
float rollval; //Current roll
float ypr[3]; //Current Yaw, pitch and roll
float ypre[3]; //Filteres yaw, pitch and roll
//Other necessary Variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

//Servo names
Servo pitch;
Servo yaw;
Servo roll;

//Filter Setup
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

////////////////////SETUP//////////////////////////

void setup() {
  
  Serial.begin(115200); //Higher rate
  //Begin communication with i2c for serial data
  Wire.begin();
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  while (!Serial); //will pause until serial console opens

  //ESC & Motor Setup
  pinMode(ESCpin, OUTPUT);
  //Servo Setup and bringing servos to 'start' position
  pitch.attach(pitchpin);
  yaw.attach(yawpin);
  roll.attach(rollpin);

  //Servo Test (Manually input position here)
  pitch.write(0);
  yaw.write(0);
  roll.write(0);
  
  //Check for IMU sensor (NEEDS TO BE CHANGED FOR NEW IMU)
  //IMU code. Uses MPU6050 library, use for yaw, pitch and roll
  mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  Serial.println("");

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin(0x29,true)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  //Set lidar to long-range mode
  lox.configSensor(Adafruit_VL53L0X :: VL53L0X_SENSE_LONG_RANGE);

  //Potentiometer (Not required, automatic input)
  pinMode(0, INPUT);


 // initialize controller
 MFWF_control_initialize();
}

void loop() {
  //Note for future: 3 LEDs for Up-Hover-Fall phases if there are extra pins

  //Servo Actuation
  potval = analogRead(pot);
  potpos = map(potval, 0, 1023, 0, 180);
  pitch.write(potpos);
  yaw.write(potpos);
  roll.write(potpos);
  Serial.print("Servo Position:\t");
  Serial.println(potpos);
  
  //Reading Yaw, pitch and roll
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
  
  //Filtering Data from IMU
  ypre[0] = simpleKalmanFilter.updateEstimate(ypr[0]);
  ypre[1] = simpleKalmanFilter.updateEstimate(ypr[1]);
  ypre[2] = simpleKalmanFilter.updateEstimate(ypr[2]);
  
  //Outputing YPR data
  Serial.print("ypre\t");
  Serial.print(ypre[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypre[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypre[2] * 180/M_PI);
  }
  
  //Reading Lidar Data
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Current Height: ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    z = measure.RangeMilliMeter; //Unfiltered height
    float ze = simpleKalmanFilter.updateEstimate(z);  //Filtered height

  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization
  //Complete following loop after a certain amount of data has been read for accurate filtration
    if (millis() > refresh_time) {
      //Serial.println(z,4);  //Unfiltered geight
      //Serial.print(",");
      Serial.print(ze,4);
      Serial.println();
    
      refresh_time = millis() + SERIAL_REFRESH_TIME;
    } else {
      Serial.println(" out of range ");
    }
  }

  //Motor Check.  Will be developed to match profile later, PID to be integrated as well. Possibly in an outside function that can be called each time it is needed
  //Note that analog values are chosen arbitrarily to give a general idea of relative power in each phase
  if (millis()<10000){
    //10s to colloect sensor data before flight starts
    analogWrite(ESCpin, 0);
  }else if (millis()>=10000||millis()<20000){
    analogWrite(ESCpin, 180);
  }else if (millis()>=20000||millis()<80000){
    analogWrite(ESCpin, 90);
    //Will also need altitude check for increasing/decreasing power
    while(z<1000){
      analogWrite(ESCpin, 120);
    }
    while(z>1500){
      analogWrite(ESCpin, 60);
    }
  }else if (millis()>=80000||millis()<90000){
    analogWrite(ESCpin, 45);
  }else{
    analogWrite(ESCpin, LOW);
  }

  // Controller step
  MFWF_control_U.C_in_control_z = 0; // control signal
  MFWF_control_U.C_in_control_phi = 0; 
  MFWF_control_U.C_in_sensor_phi = ypr[2];
  MFWF_control_update();
//  
  MFWF_control_output();

  Serial.print(" Controller: ");
  Serial.println(MFWF_control_Y.C_out_pitch*180/M_PI+90);
  pitch.write(MFWF_control_Y.C_out_pitch*180/M_PI+90);
  delay(15);
}

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MFWF_control.c
 *
 * Code generated for Simulink model 'MFWF_control'.
 *
 * Model version                  : 8.23
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Wed Jan 26 21:55:59 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */



/* Block signals (default storage) */
BlockIO_MFWF_control MFWF_control_B;

/* Block states (default storage) */
D_Work_MFWF_control MFWF_control_DWork;

/* External inputs (root inport signals with default storage) */
ExternalInputs_MFWF_control MFWF_control_U;

/* External outputs (root outports fed by signals with default storage) */
ExternalOutputs_MFWF_control MFWF_control_Y;

/* Real-time model */
static RT_MODEL_MFWF_control MFWF_control_M_;
RT_MODEL_MFWF_control *const MFWF_control_M = &MFWF_control_M_;

/* Model output function */
void MFWF_control_output(void)
{
  real_T rtb_Sum;
  real_T rtb_z_error;
  real_T u0;

  /* Sum: '<S1>/Sum' incorporates:
   *  Inport: '<Root>/C_in_control_phi'
   *  Inport: '<Root>/C_in_sensor_phi'
   */
  rtb_Sum = MFWF_control_U.C_in_control_phi - MFWF_control_U.C_in_sensor_phi;

  /* Gain: '<S38>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S30>/Filter'
   *  Gain: '<S29>/Derivative Gain'
   *  Sum: '<S30>/SumD'
   */
  MFWF_control_B.FilterCoefficient = ((0.8 * rtb_Sum) -
    MFWF_control_DWork.Filter_DSTATE) * 100.0;

  /* Sum: '<S2>/Sum' incorporates:
   *  Inport: '<Root>/C_in_control_z'
   *  Inport: '<Root>/C_in_sensor_z'
   */
  rtb_z_error = MFWF_control_U.C_in_control_z - MFWF_control_U.C_in_sensor_z;

  /* Gain: '<S81>/Integral Gain' */
  MFWF_control_B.IntegralGain = 2.29563854998445 * rtb_z_error;

  /* Gain: '<S87>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S79>/Filter'
   *  Gain: '<S78>/Derivative Gain'
   *  Sum: '<S79>/SumD'
   */
  MFWF_control_B.FilterCoefficient_b = ((0.0 * rtb_z_error) -
    MFWF_control_DWork.Filter_DSTATE_d) * 100.0;

  /* Outport: '<Root>/C_out_flapping_freq' incorporates:
   *  DiscreteIntegrator: '<S84>/Integrator'
   *  Gain: '<S89>/Proportional Gain'
   *  Sum: '<S93>/Sum'
   */
  MFWF_control_Y.C_out_flapping_freq = ((0.00114781927499223 * rtb_z_error) +
    MFWF_control_DWork.Integrator_DSTATE) + MFWF_control_B.FilterCoefficient_b;

  /* Sum: '<S44>/Sum' incorporates:
   *  Gain: '<S40>/Proportional Gain'
   */
  u0 = (0.5 * rtb_Sum) + MFWF_control_B.FilterCoefficient;

  /* Saturate: '<S42>/Saturation' */
  if (u0 > 0.52359877559829882) {
    /* Outport: '<Root>/C_out_pitch' */
    MFWF_control_Y.C_out_pitch = 0.52359877559829882;
  } else if (u0 < (-0.52359877559829882)) {
    /* Outport: '<Root>/C_out_pitch' */
    MFWF_control_Y.C_out_pitch = (-0.52359877559829882);
  } else {
    /* Outport: '<Root>/C_out_pitch' */
    MFWF_control_Y.C_out_pitch = u0;
  }

  /* End of Saturate: '<S42>/Saturation' */
}

/* Model update function */
void MFWF_control_update(void)
{
  /* Update for DiscreteIntegrator: '<S30>/Filter' */
  MFWF_control_DWork.Filter_DSTATE += 0.001 * MFWF_control_B.FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S79>/Filter' */
  MFWF_control_DWork.Filter_DSTATE_d += 0.001 *
    MFWF_control_B.FilterCoefficient_b;

  /* Update for DiscreteIntegrator: '<S84>/Integrator' */
  MFWF_control_DWork.Integrator_DSTATE += 0.001 * MFWF_control_B.IntegralGain;
}

/* Model initialize function */
void MFWF_control_initialize(void)
{
  MFWF_control_Y.C_out_pitch = 0;
  MFWF_control_Y.C_out_roll = 0;
  MFWF_control_Y.C_out_yaw = 0;
  MFWF_control_Y.C_out_flapping_freq = 0;
  MFWF_control_U.C_in_sensor_z = 0;
  MFWF_control_U.C_in_sensor_y = 0;
  MFWF_control_U.C_in_sensor_y = 0;
  MFWF_control_U.C_in_sensor_phi = 0;
  MFWF_control_U.C_in_control_phi = 0;
  
}

/* Model terminate function */
void MFWF_control_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
