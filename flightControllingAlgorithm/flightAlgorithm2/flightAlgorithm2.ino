//PID algorithm library
#include <PID_v1.h>

//communication library
#include <util/crc16.h>

//libraries for controlling the gyroscope
#include <EEPROM.h>
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

int lostPackages = 0;

byte* rpiTX;
byte* rpiTXBack;

short int counter = 0;

typedef union { //data structure used for the communication (transforms a float variable into an array of bytes due to the way the data is stored)
  float fval;
  byte bval[4];
} floatAsBytes;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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

//function prototipes

void emergencyLanding();
short unsigned int setSpeedMotor(int n, int speedM);
void sendDroneMsg(byte* msg, size_t len);
bool recvDroneMsg(byte* msg, size_t maxLen);
void emergencyLanding();

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

  Serial.println("HELLO");

  /* Initialise the gyroscope */
  if (!bno.begin()) {
    Serial.print("No gyroscope detected");
    while (1);
  }

  //pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  //pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  //pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  //turn on the motors in a low throttle
  setSpeedMotor(0, 512);
  setSpeedMotor(1, 512);
  setSpeedMotor(2, 512);
  setSpeedMotor(3, 512);

  //tunnig the PID algorithm
  SetpointPitch = 0;
  SetpointYaw = 0;
  SetpointRoll = 0;

  myPIDRoll.SetMode(AUTOMATIC);
  myPIDPitch.SetMode(AUTOMATIC);
  myPIDYaw.SetMode(AUTOMATIC);

  //the giroscope calibration -- the gyroscope should not be used until the system values is greater than 1
  uint8_t sys, gyro, accel, mag;
  long bnoID;
  int eeAddress = 0;
  sys = gyro = accel = mag = 0;

  EEPROM.get(eeAddress, bnoID);
  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  
  bno.getSensor(&sensor);
  if (bnoID == sensor.sensor_id) {
    Serial.println("CALIB");
    // Found some pre-existing calibration data!
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    bno.setSensorOffsets(calibrationData);
  }

  bno.setExtCrystalUse(true);

  bno.getCalibration(&sys, &gyro, &accel, &mag);
  while (sys == 0) {
    Serial.println("WAIT");
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(sys);
    Serial.print(gyro);
    Serial.print(accel);
    Serial.println(mag);
    delay(100);
  }

  Serial.println("READY");
}

void loop() {
  //read the information from the raspberry pi
  int8_t header;
  int16_t pitch, roll, yaw;
  uint16_t throttle;
  uint16_t esc_1, esc_2, esc_3, esc_4;
  bool errorCommunication = false;
 
  //read the incoming bytes 
  if (recvDroneMsg(rpiTX, 64)) 
  {
    lostPackages = 0;
    
    roll = ((int16_t) rpiTX[0]) | (((int16_t) rpiTX[1]) << 8);
    pitch = ((int16_t) rpiTX[2]) | (((int16_t) rpiTX[3]) << 8);
    yaw = ((int16_t) rpiTX[4]) | (((int16_t) rpiTX[5]) << 8);
    throttle = ((uint16_t) rpiTX[6]) | (((uint16_t) rpiTX[7]) << 8);

    //the setpoints must be taken from the raspberry pi
    SetpointYaw = (float)yaw;
    SetpointPitch = (float)pitch;
    SetpointRoll = (float)roll;
  }
  else
    lostPackages++;

  //pakeges lost
  if(lostPackages > 5)
  {
    SetpointYaw = 0;
    SetpointPitch = 0;
    SetpointRoll = 0;
  }
  if(lostPackages > 40)
    throttle = throttle * 0.96; //reduce the throttle speed


  //this function to read the sensors return the values in degrees per second
  imu::Vector<3> euler = (bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE) * 180) / PI;

  //input values -- taken from the gyroscope
  InputRoll = euler.x();   //Gyro pid input is deg/sec.
  InputPitch = euler.y();  //Gyro pid input is deg/sec.
  InputYaw = euler.z();    //Gyro pid input is deg/sec.

  //Calcuate the PID
  myPIDRoll.Compute();
  myPIDYaw.Compute();
  myPIDPitch.Compute();

  if (throttle > 3072) throttle = 3072;                                   //We need some room to keep full control at full throttle.

  esc_1 = throttle - (int16_t) OutputPitch + (int16_t) OutputRoll - (int16_t) OutputYaw; //Calculate the pulse for esc 1 (front-right - CCW)
  esc_2 = throttle + (int16_t) OutputPitch + (int16_t) OutputRoll + (int16_t) OutputYaw; //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = throttle + (int16_t) OutputPitch - (int16_t) OutputRoll - (int16_t) OutputYaw; //Calculate the pulse for esc 3 (rear-left - CCW)
  esc_4 = throttle - (int16_t) OutputPitch - (int16_t) OutputRoll + (int16_t) OutputYaw; //Calculate the pulse for esc 4 (front-left - CW)

  if (esc_1 < 1024) esc_1 = 1024;                                         //Keep the motors running.
  if (esc_2 < 1024) esc_2 = 1024;                                         //Keep the motors running.
  if (esc_3 < 1024) esc_3 = 1024;                                         //Keep the motors running.
  if (esc_4 < 1024) esc_4 = 1024;                                         //Keep the motors running.

  if (esc_1 > 3072) esc_1 = 3072;                                          //Limit the esc-1 pulse.
  if (esc_2 > 3072) esc_2 = 3072;                                          //Limit the esc-2 pulse.
  if (esc_3 > 3072) esc_3 = 3072;                                          //Limit the esc-3 pulse.
  if (esc_4 > 3072) esc_4 = 3072;                                          //Limit the esc-4 pulse.

  short unsigned int speedMotor1 = setSpeedMotor(0, esc_1);
  short unsigned int speedMotor2 = setSpeedMotor(1, esc_2);
  short unsigned int speedMotor3 = setSpeedMotor(2, esc_3);
  short unsigned int speedMotor4 = setSpeedMotor(3, esc_4);

  //send to the raspberry pi the new values of the motors

  for(int i = 0 ; i < 64 ; i++)
  {
    rpiTXBack[i] = 0x01;
  }

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

  //sendDroneMsg1(rpiTXBack, 64);
  sendDroneMsg(rpiTXBack, 64);
}

//motor speed function
short unsigned int setSpeedMotor(int n, int speedM){
    int speedMotor = map(speedM , 0, 4096, MinSpeed, MaxSpeed);
    //pwm.setPWM(n, 0, speedMotor);
    return (short)speedMotor;
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
