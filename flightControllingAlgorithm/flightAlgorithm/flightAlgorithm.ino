//flight controlling algorithm
  
//PID algorithm library
#include <PID_v1.h>

//communication library
#include <util/crc16.h>

//libraries for controlling the gyroscope
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


//ilbrary for controlling the servo motors
#include <Adafruit_PWMServoDriver.h>

#define MinSpeed 200
#define MaxSpeed 400

//communication global variables
#define START_BYTE 0x9F
#define END_BYTE 0x8F
#define ESCAPE_BYTE 0x7F

#define S_WAITING_HEADER 0
#define S_READING_MSG 1
#define S_READING_ESCAPE 2

uint8_t systemm, gyro, accel, mag;   //same as byte type   --   values for the gyroscope callibration
sensors_event_t event;  //this variable contains the values of the gyroscope axis x, y, z

//variable to be received from the serial port
int incomingByte = 0; // for incoming serial data

int lostPakages = 0;

unsigned long loop_timer;

byte* rpiTX;
byte* rpiTXBack;

bool firstLoop = false;

short int counter = 0;

typedef union { //data structure used for the communication (transforms a float variable into an array of bytes due to the way the data is stored)
  float fval;
  byte bval[4];
} floatAsBytes;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//PID algorithm parametres
//Define Variables we'll be connecting to
double SetpointRoll, InputRoll, OutputRoll,
       InputRollPrev = 0.0, InputPitchPrev = 0.0, InputYawPrev = 0.0;

//Specify the links and initial tuning parameters
double Kp = 1, Ki = 0, Kd = 0;
PID myPIDRoll(&InputRoll, &OutputRoll, &SetpointRoll, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double SetpointPitch, InputPitch, OutputPitch;
PID myPIDPitch(&InputPitch, &OutputPitch, &SetpointPitch, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double SetpointYaw, InputYaw, OutputYaw;
double yawKp = 1, yawKi = 0, yawKd = 0;
PID myPIDYaw(&InputYaw, &OutputYaw, &SetpointYaw, yawKp, yawKi, yawKd, DIRECT);

//function prototipes

void emergencyLanding();
short unsigned int setSpeedMotor(int n, int speedM);
void sendDroneMsg(byte* msg, size_t len);
bool recvDroneMsg(byte* msg, size_t maxLen);
void emergencyLanding();
void quatToEuler(imu::Vector<3> *output, imu::Quaternion *input);

void setup() {
  rpiTX = (byte*) calloc(64, sizeof(byte));  //reserve 64 bytes of memory in the variable to be readed from the serial port
  rpiTXBack = (byte*) calloc(64, sizeof(byte)); //reserve 64 bytes of memory in the variable to be written into the serial port
  
  if (rpiTX == NULL || rpiTXBack == NULL) //if it is not possible to reserve the memory -> infinite loop
    while (true) {
      ;
    }

  //initialize connection with the raspberry pi with the serial port 9600
  Serial.begin(115200);
  //wait for the serial port to be connected
  while (!Serial) {
    ;
  }

  lostPakages = 0;

  //Turn on the warning led.
  digitalWrite(12, HIGH);

  /* Initialise the gyroscope */
  if (!bno.begin()) {
    Serial.print("No gyroscope detected");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  //set the frecuency of the motors to 50 hz
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  myPIDRoll.SetOutputLimits(-800, 800);
  myPIDPitch.SetOutputLimits(-800, 800);
  myPIDYaw.SetOutputLimits(-800, 800);
  myPIDRoll.SetSampleTime(1);
  myPIDPitch.SetSampleTime(1);
  myPIDYaw.SetSampleTime(1);

  //tunnig the PID algorithm
  SetpointPitch = 0;
  SetpointYaw = 0;
  SetpointRoll = 0;

  myPIDRoll.SetMode(AUTOMATIC);
  myPIDPitch.SetMode(AUTOMATIC);
  myPIDYaw.SetMode(AUTOMATIC);

  //the giroscope calibration -- the gyroscope should not be used until the system values is greater than 1
  uint8_t systemm, gyro, accel, mag;
  systemm = gyro = accel = mag = 0;

  bno.setExtCrystalUse(true);
  
  bno.getCalibration(&systemm, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  while (!systemm){
    bno.getCalibration(&systemm, &gyro, &accel, &mag);
  }

  //When everything is done, turn off the led.
  digitalWrite(12, LOW);
}

void loop() {
  //get starting time 
  unsigned long firstTime = micros();
  
  //read the information from the raspberry pi
  int8_t header;
  int16_t pitch = 2048, roll = 2048, yaw = 2048;
  uint16_t raspberryThrottle = 1024;

  bool errorCommunication = false;
  
  //read the incoming bytes 
  if (recvDroneMsg(rpiTX, 64)) 
  {
    lostPakages = 0;
    
    header = rpiTX[0];
    roll = ((int16_t) rpiTX[0]) | (((int16_t) rpiTX[1]) << 8);
    pitch = ((int16_t) rpiTX[2]) | (((int16_t) rpiTX[3]) << 8);
    yaw = ((int16_t) rpiTX[4]) | (((int16_t) rpiTX[5]) << 8);
    raspberryThrottle = ((uint16_t) rpiTX[6]) | (((uint16_t) rpiTX[7]) << 8);

    //the setpoints must be taken from the raspberry pi
    //SetpointYaw = (float)yaw;
    //SetpointPitch = (float)pitch;
    //SetpointRoll = (float)roll;
  }
  else
    lostPakages++;
  
  uint16_t esc_1;
  uint16_t esc_2;
  uint16_t esc_3;
  uint16_t esc_4;
  
  if(raspberryThrottle == 1024){
    esc_1 = 1024;
    esc_2 = 1024;
    esc_3 = 1024;
    esc_4 = 1024;
  }
  else{
    //read values from the giroscope
    imu::Quaternion quaternion = bno.getQuat();
    imu::Vector<3> angle = imu::Vector<3>(0,0,0);
    quatToEuler(&angle, &quaternion);
    imu::Vector<3> gyro = (bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE));

    // /w angle correction
    if (firstLoop) {
      InputRoll = gyro.x();
      InputPitch = -gyro.y();
      InputYaw = gyro.z();
    } else {
      InputRoll = (InputRoll * 0.7) + (gyro.x() * 0.3);   //Gyro pid input is deg/sec --> we substract from the previous measurement to get it
      InputPitch = (InputPitch * 0.7) + (-gyro.y() * 0.3);  //Gyro pid input is deg/sec.
      InputYaw = (InputYaw * 0.7) + (gyro.z() * 0.3);    //Gyro pid input is deg/sec.
    }

    if (!isnan(InputRoll)) { InputRollPrev = gyro.x();   /*Gyro pid input is deg/sec.*/ } else { InputRoll = InputRollPrev; }
    if (!isnan(InputPitch)) { InputPitchPrev = -gyro.y();   /*Gyro pid input is deg/sec.*/ } else { InputPitch = InputPitchPrev; }
    if (!isnan(InputYaw)) { InputYawPrev = gyro.z();   /*Gyro pid input is deg/sec*/ } else { InputYaw = InputYawPrev; }

    firstLoop = false;

    Serial.println(angle.x());
    Serial.println(angle.y());
  
    SetpointYaw = 0;
    SetpointPitch = 0;
    SetpointRoll = 0;
    
    // We also add a certain amount of deadband, in order to improve flight
    if (yaw < 2040 || yaw > 2054) {
      int16_t deadzoneCorr = 0;
      if (yaw > 2054) { deadzoneCorr = 2054; }
      else if (yaw < 2040) { deadzoneCorr = 2040; }
      //Serial.println(yaw - deadzoneCorr);
      SetpointYaw =  ((double) (yaw - deadzoneCorr)) / 6.19;
    }
    if (pitch < 2040 || pitch > 2054) {
      int16_t deadzoneCorr = 0;
      if (pitch > 2054) { deadzoneCorr = 2054; }
      else if (pitch < 2040) { deadzoneCorr = 2040; }
      //Serial.println((euler.z() * 30.72), 4);
      SetpointPitch = (((double) (pitch - deadzoneCorr)) - (-angle.y() * 30.72)) / 6.19;
    }
    if (roll < 2040 || roll > 2054) {
      int16_t deadzoneCorr = 0;
      if (roll > 2054) { deadzoneCorr = 2054; }
      else if (roll < 2040) { deadzoneCorr = 2040; }
      SetpointRoll = (((double) (roll - deadzoneCorr)) - (angle.x() * 30.72)) / 6.19;
    }
  
    //pakeges lost
   
    //Calcuate the PID
    myPIDRoll.Compute();
    myPIDYaw.Compute();
    myPIDPitch.Compute();

    myPIDRoll.Compute();
    myPIDYaw.Compute();
    myPIDPitch.Compute();
  
    while(firstTime + 10000 > micros());
  }
   
  if(raspberryThrottle == 1024){
    esc_1 = 1024;
    esc_2 = 1024;
    esc_3 = 1024;
    esc_4 = 1024;
  }
  else{
    imu::Quaternion quaternion = bno.getQuat();
    imu::Vector<3> angle = imu::Vector<3>(0,0,0);
    quatToEuler(&angle, &quaternion);
    imu::Vector<3> gyro = (bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE));

    // TODO test this new code
    //input values -- taken from the gyroscope
    // /w angle correction
    if (firstLoop) {
      InputRoll = gyro.x();
      InputPitch = -gyro.y();
      InputYaw = gyro.z();
    } else {
      InputRoll = (InputRoll * 0.7) + (gyro.x() * 0.3);   //Gyro pid input is deg/sec --> we substract from the previous measurement to get it
      InputPitch = (InputPitch * 0.7) + (-gyro.y() * 0.3);  //Gyro pid input is deg/sec.
      InputYaw = (InputYaw * 0.7) + (gyro.z() * 0.3);    //Gyro pid input is deg/sec.
    }

    if (!isnan(InputRoll)) { InputRollPrev = gyro.x();   /*Gyro pid input is deg/sec.*/ } else { InputRoll = InputRollPrev; }
    if (!isnan(InputPitch)) { InputPitchPrev = -gyro.y();   /*Gyro pid input is deg/sec.*/ } else { InputPitch = InputPitchPrev; }
    if (!isnan(InputYaw)) { InputYawPrev = gyro.z();   /*Gyro pid input is deg/sec*/ } else { InputYaw = InputYawPrev; }

    firstLoop = false;

    Serial.println(angle.x());
    Serial.println(angle.y());

    SetpointYaw = 0;
    SetpointPitch = 0;
    SetpointRoll = 0;
    // We also add a certain amount of deadband, in order to improve flight
      if (yaw < 2040 || yaw > 2054) {
        int16_t deadzoneCorr = 0;
        if (yaw > 2054) { deadzoneCorr = 2054; }
        else if (yaw < 2040) { deadzoneCorr = 2040; }
        //Serial.println(yaw - deadzoneCorr);
        SetpointYaw =  ((double) (yaw - deadzoneCorr)) / 6.19;
      }
      if (pitch < 2040 || pitch > 2054) {
        int16_t deadzoneCorr = 0;
        if (pitch > 2054) { deadzoneCorr = 2054; }
        else if (pitch < 2040) { deadzoneCorr = 2040; }
        //Serial.println((euler.z() * 30.72), 4);
        SetpointPitch = (((double) (pitch - deadzoneCorr)) - (-angle.y() * 30.72)) / 6.19;
      }
      if (roll < 2040 || roll > 2054) {
        int16_t deadzoneCorr = 0;
        if (roll > 2054) { deadzoneCorr = 2054; }
        else if (roll < 2040) { deadzoneCorr = 2040; }
        SetpointRoll = (((double) (roll - deadzoneCorr)) - (angle.x() * 30.72)) / 6.19;
      }

    //pakeges lost
    //Calcuate the PID
    myPIDRoll.Compute();
    myPIDYaw.Compute();
    myPIDPitch.Compute();

    myPIDRoll.Compute();
    myPIDYaw.Compute();
    myPIDPitch.Compute();

    if (raspberryThrottle > 3072) raspberryThrottle = 3072;                                   //We need some room to keep full control at full throttle.

    uint16_t esc_1 = raspberryThrottle - (int16_t)OutputPitch + (int16_t)OutputRoll - (int16_t)OutputYaw; //Calculate the pulse for esc 1 (front-right - CCW)
    uint16_t esc_2 = raspberryThrottle + (int16_t)OutputPitch + (int16_t)OutputRoll + (int16_t)OutputYaw; //Calculate the pulse for esc 2 (rear-right - CW)
    uint16_t esc_3 = raspberryThrottle + (int16_t)OutputPitch - (int16_t)OutputRoll - (int16_t)OutputYaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    uint16_t esc_4 = raspberryThrottle - (int16_t)OutputPitch - (int16_t)OutputRoll + (int16_t)OutputYaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (esc_1 < 1024) esc_1 = 1024;                                         //Keep the motors running.
    if (esc_2 < 1024) esc_2 = 1024;                                         //Keep the motors running.
    if (esc_3 < 1024) esc_3 = 1024;                                         //Keep the motors running.
    if (esc_4 < 1024) esc_4 = 1024;                                         //Keep the motors running.

    if (esc_1 > 2500)esc_1 = 3072;                                          //Limit the esc-1 pulse.
    if (esc_2 > 2500)esc_2 = 3072;                                          //Limit the esc-2 pulse.
    if (esc_3 > 2500)esc_3 = 3072;                                          //Limit the esc-3 pulse.
    if (esc_4 > 2500)esc_4 = 3072;                                          //Limit the esc-4 pulse.
  }
  
  short unsigned int speedMotor1 = setSpeedMotor(0, esc_1);
  short unsigned int speedMotor2 = setSpeedMotor(1, esc_2);
  short unsigned int speedMotor3 = setSpeedMotor(2, esc_3);
  short unsigned int speedMotor4 = setSpeedMotor(3, esc_4);

  //send to the raspberry pi the new values of the motors

  for(int i = 0 ; i < 64 ; i++) //initialize the arrays of bytes to zero
    rpiTXBack[i] = 0x01;
    
  rpiTXBack[1] = (counter >> 8) & 0xFF;
  rpiTXBack[0] = counter & 0xFF;
  counter++;

  rpiTXBack[2] = esc_4 & 0xFF;
  rpiTXBack[3] = (esc_4 >> 8) & 0xFF;
  rpiTXBack[4] = esc_1 & 0xFF;
  rpiTXBack[5] = (esc_1 >> 8) & 0xFF;
  rpiTXBack[6] = esc_3 & 0xFF;
  rpiTXBack[7] = (esc_3 >> 8) & 0xFF;
  rpiTXBack[8] = esc_2 & 0xFF;
  rpiTXBack[9] = (esc_2 >> 8) & 0xFF;

  floatAsBytes axisX, axisY, axisZ, axisW;
  floatAsBytes accelerometerAxisX, accelerometerAxisY, accelerometerAxisZ;

  imu::Quaternion quat = bno.getQuat();

  axisW.fval = (float) quat.w();
  rpiTXBack[10] = axisW.bval[0];
  rpiTXBack[11] = axisW.bval[1];
  rpiTXBack[12] = axisW.bval[2];
  rpiTXBack[13] = axisW.bval[3];

  axisX.fval = (float) quat.x();
  rpiTXBack[14] = axisX.bval[0];
  rpiTXBack[15] = axisX.bval[1];
  rpiTXBack[16] = axisX.bval[2];
  rpiTXBack[17] = axisX.bval[3];

  axisX.fval = (float) quat.x();
  rpiTXBack[18] = axisY.bval[0];
  rpiTXBack[19] = axisY.bval[1];
  rpiTXBack[20] = axisY.bval[2];
  rpiTXBack[21] = axisY.bval[3];

  axisZ.fval = (float) quat.z();
  rpiTXBack[22] = axisZ.bval[0];
  rpiTXBack[23] = axisZ.bval[1];
  rpiTXBack[24] = axisZ.bval[2];
  rpiTXBack[25] = axisZ.bval[3];

  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  accelerometerAxisX.fval = (float) accelerometer.x();
  rpiTXBack[26] = accelerometerAxisX.bval[0];
  rpiTXBack[27] = accelerometerAxisX.bval[1];
  rpiTXBack[28] = accelerometerAxisX.bval[2];
  rpiTXBack[29] = accelerometerAxisX.bval[3];

  accelerometerAxisY.fval = (float) accelerometer.y();
  rpiTXBack[30] = accelerometerAxisY.bval[0];
  rpiTXBack[31] = accelerometerAxisY.bval[1];
  rpiTXBack[32] = accelerometerAxisY.bval[2];
  rpiTXBack[33] = accelerometerAxisY.bval[3];

  accelerometerAxisZ.fval = (float) accelerometer.z();
  rpiTXBack[34] = accelerometerAxisZ.bval[0];
  rpiTXBack[35] = accelerometerAxisZ.bval[1];
  rpiTXBack[36] = accelerometerAxisZ.bval[2];
  rpiTXBack[37] = accelerometerAxisZ.bval[3];

  //send data back to the raspberry pi
  sendDroneMsg(rpiTXBack, 64);

  while(firstTime + 20000 > micros());
}

//motor speed function
short unsigned int setSpeedMotor(int n, int speedM){
    int speedMotor = map(speedM, 1000, 4096, MinSpeed, MaxSpeed);
    pwm.setPWM(n, 0, speedMotor);
    return (short)speedMotor;
}

//gyroscope functions
void quatToEuler(imu::Vector<3> *output, imu::Quaternion *input) {
  double w = input->w();
  double x = input->x();
  double y = input->y();
  double z = input->z();

  output->x() = atan2(2.0 * (z * y + w * x) , 1.0 - 2.0 * (x * x + y * y));
  output->y() = asin(2.0 * (y * w - z * x));
  output->z() = atan2(2.0 * (z * w + x * y) , - 1.0 + 2.0 * (w * w + x * x));
}

//communication functions

//send data to the serial port
void sendDroneMsg(byte* msg, size_t len) {
  uint16_t crc = 0;
  Serial.write(START_BYTE); // Write message start byte
  for (int i = 0; i < len; i++) {
    if (msg[i] == END_BYTE || msg[i] == ESCAPE_BYTE) {
      Serial.write(ESCAPE_BYTE);
    }
    Serial.write(msg[i]);
    crc = _crc16_update(crc, msg[i]);
  }
  if ((crc & 0xFF) == END_BYTE || (crc & 0xFF) == ESCAPE_BYTE) {
    Serial.write(ESCAPE_BYTE);
  }
  Serial.write(crc & 0xFF);
  if (((crc >> 8) & 0xFF) == END_BYTE || ((crc >> 8) & 0xFF) == ESCAPE_BYTE) {
    Serial.write(ESCAPE_BYTE);
  }
  Serial.write((crc >> 8) & 0xFF);
  Serial.write(END_BYTE); // Write message end byte
}

//recive data from the serial port
bool recvDroneMsg(byte* msg, size_t maxLen) {
  int tries = 100;
  int bytesParsed = 0;
  byte parsedByte = 0;
  int state = S_WAITING_HEADER; // Current parsing state
  bool doneParsing = false;

  // Receive loop
  while (tries > 0 && bytesParsed <= maxLen + 1 && !doneParsing) {
    // Read from serial
    //Serial.write("GET");
    parsedByte = Serial.read();
    if (parsedByte == 255 || parsedByte == -1) { // Avoid blocking by identifying lack of messages
      tries -= 1;
    } else {
      switch (state) {
        case S_WAITING_HEADER: // We are waiting for a header
          if (parsedByte == START_BYTE) { // Got header?
            state = S_READING_MSG; // Start reading message
          }
          break;
        case S_READING_MSG: // We are reading the message
          if (parsedByte == END_BYTE) { // End of message?
            doneParsing = true; // We are done!
          } else if (parsedByte == ESCAPE_BYTE) { // Escape byte?
            state = S_READING_ESCAPE; // Read next byte literally
          } else { // Other?
            if (bytesParsed < maxLen) {
              msg[bytesParsed] = parsedByte; // Just read the byte
              bytesParsed++;
            }
          }
          break;
        case S_READING_ESCAPE:
          if (bytesParsed < maxLen) {
            msg[bytesParsed] = parsedByte; // Read next byte literally
            bytesParsed++;
          }
          state = S_READING_MSG;
          break;
      }
    }
  }

  // Check message is at least 3 bytes
  if (bytesParsed < 3) return false;

  // Check message CRC
  uint16_t obtainedCRC = msg[bytesParsed - 2] | (msg[bytesParsed - 1] << 8);
  uint16_t expectedCRC = 0; // Initial value

  for (int i = 0; i < bytesParsed - 2; i++) {
    expectedCRC = _crc16_update(expectedCRC, msg[i]);
  }

  if (expectedCRC != obtainedCRC) {
    return false;
  }
  return true;
}

//emergency landing function
void emergencyLanding()
{
  SetpointRoll = InputRoll = OutputRoll = 0;
}
