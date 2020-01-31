//PID algorithm library
#include <PID_v1.h>

//libraries for controlling the gyroscope
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//ilbrary for controlling the servo motors
#include <Adafruit_PWMServoDriver.h>

#define MinSpeed = 150
#define MaxSpees = 300

uint8_t system, gyro, accel, mag;   //same as byte type   --   values for the gyroscope callibration
sensors_event_t event;  //this variable contains the values of the gyroscope axis x, y, z

//variable to be received from the serial port
int incomingByte = 0; // for incoming serial data

byte* rpiTX;
byte* rpiTXBack;

short int counter = 0;

//PID algorithm parametres

//Define Variables we'll be connecting to
double SetpointRoll, InputRoll, OutputRoll;

//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPIDRoll(&InputRoll, &OutputRoll, &SetpointRoll, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double SetpointPitch, InputPitch, OutputPitch;
PID myPIDPitch(&InputPitch, &OutputPitch, &SetpointPitch, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double SetpointYaw, InputYaw, OutputYaw;
PID myPIDYaw(&InputYaw, &OutputYaw, &SetpointYaw, Kp, Ki, Kd, DIRECT);

void setup() {
  rpiTX = (byte*) calloc(64, sizeof(byte));  //reserve 64 bytes of memory in the variable to be readed from the serial port
  rpiTXBack = (byte*) calloc(64, sizeof(byte)); //reserve 64 bytes of memory in the variable to be written into the serial port
  
  if (rpiTX == null || rpiTXBack == null) //if it is not possible to reserve the memory -> infinite loop
    while (True) {
      ;
    }

  //initialize connection with the raspberry pi with the serial port 9600
  Serial.begin(9600);
  //wait for the serial port to be connected
  while (!Serial) {
    ;
  }

  //Turn on the warning led.
  digitalWrite(12, HIGH);

  /* Initialise the gyroscope */
  if (!bno.begin()) {
    Serial.print("No gyroscope detected");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  //start the servo motors
  void Adafruit_PWMServoDriver::begin(uint8_t prescale = 0);

  //set the frecuency of the motors to 50 hz
  pwm.setPWMFreq(50);

  //turn on the motors in a low throttle
  setSpeedMotor(1, 1024);
  setSpeedMotor(2, 1024);
  setSpeedMotor(3, 1024);
  setSpeedMotor(4, 1024);

  //tunnig the PID algorithm
  SetpointPitch = 0;
  SetpointYaw = 0;
  SetpointRoll = 0;

  myPIDRoll.SetMode(AUTOMATIC);
  myPIDPitch.SetMode(AUTOMATIC);
  myPIDYaw.SetMode(AUTOMATIC);

  //the giroscope calibration -- the gyroscope should not be used until the system values is greater than 1
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  if (!system) { //the system variable is 0 untill the gyroscope is callibrated  --  the calibrating qualitie varies from 0 to 3 being 3 the optimal callibration
    while (1); //this line is for the system not to execute if the values of the gyroscope are wrong -- remove if in doesn't work
  }

  //When everything is done, turn off the led.
  digitalWrite(12, LOW);
}

void loop() {
  //read the information from the raspberry pi
  int8_t header;
  int16_t pitch, roll, yaw, throttle;
  int32_t raspberryThrotte;
  if (Serial.available() > 0) {
    // read the incoming byte:
    *rpiTX = Serial.read();

    header = rpiTX[0];
    roll = rpiTX[1] | (rpiTX[2] << 8);
    pitch = rpiTX[3] | (rpiTX[4] << 8);
    yaw = rpiTX[5] | (rpiTX[6] << 8);
    raspberryThrottle = rpiTX[7] | (rpiTX[8] << 8) | (rpiTX[9] << 16) | (rpiTX[10] << 24);

    //the setpoints must be taken from the raspberry pi
    SetpointYaw = (float)yaw;
    SetpointPitch = (float)pitch;
    SetpointRoll = (float)roll;

    //this function to read the sensors return the values in degrees per second
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    //input values -- taken from the gyroscope
    InputRoll = euler.x;   //Gyro pid input is deg/sec.
    InputPitch = euler.y;  //Gyro pid input is deg/sec.
    InputYaw = euler.z;    //Gyro pid input is deg/sec.

    //Calcuate the PID
    myPIDRoll.Compute();
    myPIDYaw.Compute();
    myPIDPitch.Compute();

    throttle = (float)raspberryThrottle;

    if (throttle > 3072) throttle = 3072;                                   //We need some room to keep full control at full throttle.

    esc_1 = throttle - OutputPitch + OutputRoll - OutputYaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + OutputPitch + OutputRoll + OutputYaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + OutputPitch - OutputRoll - OutputYaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - OutputPitch - OutputRoll + OutputYaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (esc_1 < 1024) esc_1 = 1024;                                         //Keep the motors running.
    if (esc_2 < 1024) esc_2 = 1024;                                         //Keep the motors running.
    if (esc_3 < 1024) esc_3 = 1024;                                         //Keep the motors running.
    if (esc_4 < 1024) esc_4 = 1024;                                         //Keep the motors running.

    if (esc_1 > 3072)esc_1 = 3072;                                          //Limit the esc-1 pulse.
    if (esc_2 > 3072)esc_2 = 3072;                                          //Limit the esc-2 pulse.
    if (esc_3 > 3072)esc_3 = 3072;                                          //Limit the esc-3 pulse.
    if (esc_4 > 3072)esc_4 = 3072;                                          //Limit the esc-4 pulse.

    short unsigned int speedMotor1 = setSpeedMotor(1, esc_1);
    short unsigned int speedMotor2 = setSpeedMotor(2, esc_2);
    short unsigned int speedMotor3 = setSpeedMotor(3, esc_3);
    short unsigned int speedMotor4 = setSpeedMotor(4, esc_4);

    //send to the raspberry pi the new values of the motors

    for(int i = 0 ; i < 64 ; i++)
    {
      rpiTXBack[i] = 0x00;
    }
    
    byte header = 0x4F;
    rpiTXBack[0] = header;
    
    counter++;
    rpiTXBack[1] = (counter >> 8) & 0xFF;
    rpiTXBack[2] = counter & 0xFF;
    
    float axisX, axisY, axisZ, axisW;
    float accelerometerAxisX, accelerometerAxisY, accelerometerAxisZ;
    
    imu::Quaternion quat = bno.getQuat();
    axisW = (float) quat.w();
    rpiTXBack[3] = (axisW >> 24) & 0xFF;
    rpiTXBack[4] = (axisW >> 16) & 0xFF;
    rpiTXBack[5] = (axisW >> 8) & 0xFF;
    rpiTXBack[6] = axisW & 0xFF;
    
    axisY = (float) quat.y();
    rpiTXBack[7] = (axisY >> 24) & 0xFF;
    rpiTXBack[8] = (axisY >> 16) & 0xFF;
    rpiTXBack[9] = (axisY >> 8) & 0xFF;
    rpiTXBack[10] = axisY & 0xFF;
    
    axisX = (float) quat.x();
    rpiTXBack[11] = (axisX >> 24) & 0xFF;
    rpiTXBack[12] = (axisX >> 16) & 0xFF;
    rpiTXBack[13] = (axisX >> 8) & 0xFF;
    rpiTXBack[14] = axisX & 0xFF;
    
    axisZ = (float) quat.z();
    rpiTXBack[15] = (axisZ >> 24) & 0xFF;
    rpiTXBack[16] = (axisZ >> 16) & 0xFF;
    rpiTXBack[17] = (axisZ >> 8) & 0xFF;
    rpiTXBack[18] = axisZ & 0xFF;

    imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER)
    accelerometerAxisX = accelerometer.x();
    rpiTXBack[19] = (accelerometerAxisX >> 24) & 0xFF;
    rpiTXBack[20] = (accelerometerAxisX >> 16) & 0xFF;
    rpiTXBack[21] = (accelerometerAxisX >> 8) & 0xFF;
    rpiTXBack[22] = accelerometerAxisX & 0xFF;
    
    accelerometerAxisY = accelerometer.y();
    rpiTXBack[23] = (accelerometerAxisY >> 24) & 0xFF;
    rpiTXBack[24] = (accelerometerAxisY >> 16) & 0xFF;
    rpiTXBack[25] = (accelerometerAxisY >> 8) & 0xFF;
    rpiTXBack[26] = accelerometerAxisY & 0xFF;
    
    accelerometerAxisZ = accelerometer.z();
    rpiTXBack[27] = (accelerometerAxisZ >> 24) & 0xFF;
    rpiTXBack[28] = (accelerometerAxisZ >> 16) & 0xFF;
    rpiTXBack[29] = (accelerometerAxisZ >> 8) & 0xFF;
    rpiTXBack[30] = accelerometerAxisZ & 0xFF;

    Serial.write(rpiTXBack, 64);
  }
}

short unsigned int setSpeedMotor(int n, int speedM){
    int speedMotor = map(speedM , 0, 4096, MinSpeed, MaxSpeed);
    pwm.setPWM(n, 0, speedMotor);
    return (short)speedMotor;
}
