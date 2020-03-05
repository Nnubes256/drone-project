//PID algorithm library
#include <PID_v1.h>

//libraries for controlling the gyroscope
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

//ilbrary for controlling the servo motors
#include <Adafruit_PWMServoDriver.h>

//library to calculate crc
#include <util/crc16.h>

#define MinSpeed 200
#define MaxSpeed 400

#define START_BYTE 0x9F
#define END_BYTE 0x8F
#define ESCAPE_BYTE 0x7F

typedef union {
    float fval;
    byte bval[4];
} floatAsBytes;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint8_t sys, gyro, accel, mag;   //same as byte type   --   values for the gyroscope callibration
sensors_event_t event;  //this variable contains the values of the gyroscope axis x, y, z

//variable to be received from the serial port
int incomingByte = 0; // for incoming serial data

byte* rpiTX;
byte* rpiTXBack;

unsigned long loop_timer;
short int counter = 0;

//PID algorithm parametres

//Define Variables we'll be connecting to
double SetpointRoll, InputRoll, OutputRoll, InputRollPrev = 0.0;

//Specify the links and initial tuning parameters
double Kp = 20, Ki = 0, Kd = 0;
PID myPIDRoll(&InputRoll, &OutputRoll, &SetpointRoll, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double SetpointPitch, InputPitch, OutputPitch, InputPitchPrev = 0.0;
PID myPIDPitch(&InputPitch, &OutputPitch, &SetpointPitch, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double yawKp = 30, yawKi = 0, yawKd = 0;
double SetpointYaw, InputYaw, OutputYaw, InputYawPrev = 0.0;
PID myPIDYaw(&InputYaw, &OutputYaw, &SetpointYaw, yawKp, yawKi, yawKd, DIRECT);

bool firstRun = true;

uint16_t
crc16_update(uint16_t crc, uint8_t a)
{
int i;

crc ^= a;
for (i = 0; i < 8; ++i)
{
    if (crc & 1)
   crc = (crc >> 1) ^ 0xA001;
    else
   crc = (crc >> 1);
}

return crc;
}

/*void sendDroneMsg(byte* msg, size_t len) {
  uint16_t crc = 0;
  Serial.print("BEGIN ");
  Serial.println(START_BYTE, HEX); // Write message start byte
  //Serial.write(START_BYTE);
  for (int i = 0; i < len; i++) {
    if (msg[i] == END_BYTE || msg[i] == ESCAPE_BYTE) { // If message byte is equal to end byte, it gets escaped
      //Serial.write(ESCAPE_BYTE);
      Serial.print(ESCAPE_BYTE, HEX);
      Serial.print(" ESCAPES ");
    }
    Serial.print(msg[i], HEX);
    //Serial.write(msg[i]);
    //Serial.print(" CRC ");
    //crc = crc16_update(crc, msg[i]);
    //Serial.println(crc);
  }
  Serial.print(" FINALCRC ");
  Serial.println(crc, HEX);
  Serial.println(crc & 0xFF, HEX);
  Serial.println((crc >> 8) & 0xFF, HEX);
  if (crc & 0xFF == END_BYTE || crc & 0xFF == ESCAPE_BYTE) {
    //Serial.write(ESCAPE_BYTE);
     Serial.print(ESCAPE_BYTE, HEX);
     Serial.print(" ESCAPES ");
  }
  Serial.print(crc & 0xFF, HEX);
  //Serial.write(crc & 0xFF, HEX);
  if ((crc >> 8) & 0xFF == END_BYTE || ((crc >> 8) & 0xFF) == ESCAPE_BYTE) {
    //Serial.write(ESCAPE_BYTE);
    Serial.print(ESCAPE_BYTE, HEX);
    Serial.print(" ESCAPES ");
  }
  Serial.print((crc >> 8) & 0xFF, HEX);
  Serial.print(END_BYTE, HEX); // Write message end byte
  //Serial.write(END_BYTE); // Write message end byte
  Serial.println("END");
}*/


void sendDroneMsg1(byte* msg, size_t len) {
  uint16_t crc = 0;
  //Serial.print("BEGIN ");
  //Serial.println(START_BYTE); // Write message start byte
  //Serial.write(START_BYTE);
  for (int i = 0; i < len; i++) {
    if (msg[i] == END_BYTE || msg[i] == ESCAPE_BYTE) { // If message byte is equal to end byte, it gets escaped
      Serial.write(ESCAPE_BYTE);
      //Serial.print(ESCAPE_BYTE);
      //Serial.print(" ESCAPES ");
    }
    //Serial.print(msg[i]);
    Serial.write(msg[i]);
    //Serial.print(" CRC ");
    crc = crc16_update(crc, msg[i]);
    //Serial.println(crc);
  }
  //Serial.print(" FINALCRC ");
  //Serial.println(crc);
  //Serial.println(crc & 0xFF);
  //Serial.println((crc >> 8) & 0xFF);
  if ((crc & 0xFF) == END_BYTE || (crc & 0xFF) == ESCAPE_BYTE) {
    while (!Serial.availableForWrite()) {};
    Serial.write(ESCAPE_BYTE);
  }
  while (!Serial.availableForWrite()) {};
  Serial.write(crc & 0xFF);
  if (((crc >> 8) & 0xFF) == END_BYTE || ((crc >> 8) & 0xFF) == ESCAPE_BYTE) {
    while (!Serial.availableForWrite()) {};
    Serial.write(ESCAPE_BYTE);
  }
  while (!Serial.availableForWrite());
  Serial.write((crc >> 8) & 0xFF);
  //Serial.print(END_BYTE); // Write message end byte
  while (!Serial.availableForWrite());
  //Serial.write(END_BYTE); // Write message end byte
  //Serial.println("END");
}

void sendDroneMsg(byte* msg, size_t len) {
  uint16_t crc = 0;
  //Serial.print("BEGIN ");
  //Serial.println(START_BYTE); // Write message start byte
  Serial.write(START_BYTE);
  for (int i = 0; i < len; i++) {
    if (msg[i] == END_BYTE || msg[i] == ESCAPE_BYTE) { // If message byte is equal to end byte, it gets escaped
      Serial.write(ESCAPE_BYTE);
      //Serial.print(ESCAPE_BYTE);
      //Serial.print(" ESCAPES ");
    }
    //Serial.print(msg[i]);
    Serial.write(msg[i]);
    //Serial.print(" CRC ");
    crc = crc16_update(crc, msg[i]);
    //Serial.println(crc);
  }
  //Serial.print(" FINALCRC ");
  //Serial.println(crc);
  //Serial.println(crc & 0xFF);
  //Serial.println((crc >> 8) & 0xFF);
  if ((crc & 0xFF) == END_BYTE || (crc & 0xFF) == ESCAPE_BYTE) {
    Serial.write(ESCAPE_BYTE);
  }
  Serial.write(crc & 0xFF);
  if (((crc >> 8) & 0xFF) == END_BYTE || ((crc >> 8) & 0xFF) == ESCAPE_BYTE) {
    Serial.write(ESCAPE_BYTE);
  }
  Serial.write((crc >> 8) & 0xFF);
  //Serial.print(END_BYTE); // Write message end byte
  Serial.write(END_BYTE); // Write message end byte
  //Serial.println("END");
}

#define S_WAITING_HEADER 0
#define S_READING_MSG 1
#define S_READING_ESCAPE 2

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

void setup() {
  
  //initialize connection with the raspberry pi with the serial port 9600
  Serial.begin(115200);

  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  /* Initialise the gyroscope */
  if (!bno.begin()) {
    Serial.print("No gyroscope detected");
    while (1);
  }
  
  //wait for the serial port to be connected
  while (!Serial) {
    analogWrite(LED_BUILTIN, !analogRead(LED_BUILTIN));
  }

  //while (!Serial.available()) {}

  myPIDRoll.SetOutputLimits(-400, 400);
  myPIDPitch.SetOutputLimits(-400, 400);
  myPIDYaw.SetOutputLimits(-400, 400);
  myPIDRoll.SetSampleTime(20);
  myPIDPitch.SetSampleTime(20);
  myPIDYaw.SetSampleTime(20);

  Serial.println("HELLO");
  rpiTX = (byte*) calloc(64, sizeof(byte));  //reserve 64 bytes of memory in the variable to be readed from the serial port
  rpiTXBack = (byte*) calloc(64, sizeof(byte)); //reserve 64 bytes of memory in the variable to be written into the serial port

  if (rpiTX == NULL || rpiTXBack == NULL) { //if it is not possible to reserve the memory -> infinite loop
    Serial.println("BAD");
    while (1) {}
  }

  //Turn on the warning led.
  //digitalWrite(12, HIGH);

  Serial.println("1");

  //tunnig the PID algorithm
  SetpointPitch = 0;
  SetpointYaw = 0;
  SetpointRoll = 0;

  myPIDRoll.SetMode(AUTOMATIC);
  myPIDPitch.SetMode(AUTOMATIC);
  myPIDYaw.SetMode(AUTOMATIC);

  Serial.println("2");

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
    // Found some pre-existing calibration data!
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    bno.setSensorOffsets(calibrationData);
  }

  
  bno.setExtCrystalUse(true);

  bno.getCalibration(&sys, &gyro, &accel, &mag);
  while (sys == 0) {
    bno.getCalibration(&sys, &gyro, &accel, &mag);
  }

  /* The data should be ignored until the system calibration is > 0 */
  /*if (!system) { //the system variable is 0 untill the gyroscope is callibrated  --  the calibrating qualitie varies from 0 to 3 being 3 the optimal callibration
    while (1); //this line is for the system not to execute if the values of the gyroscope are wrong -- remove if in doesn't work
  }*/
}

void quatToEuler(imu::Vector<3> *output, imu::Quaternion *input) {
  double w = input->w();
  double x = input->x();
  double y = input->y();
  double z = input->z();
  
  /*double q2sqr = y * y;
  double t0 = -2.0 * (q2sqr + z * z) + 1.0;
  double t1 = +2.0 * (x * y + w * z);
  double t2 = -2.0 * (x * z - w * y);
  double t3 = +2.0 * (y * z + w * x);
  double t4 = -2.0 * (x * x + q2sqr) + 1.0;

  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;*/

  output->x() = atan2(2.0 * (z * y + w * x) , 1.0 - 2.0 * (x * x + y * y));
  output->y() = asin(2.0 * (y * w - z * x));
  output->z() = atan2(2.0 * (z * w + x * y) , - 1.0 + 2.0 * (w * w + x * x));
  /*output->x() = (asin(t2) * 180) / PI;
  output->y() = (atan2(t3, t4) * 180) / PI;
  output->z() = (atan2(t1, t0) * 180) / PI;*/
}

void loop() {
  //read the information from the raspberry pi
  int8_t header;
  float throttle;
  int16_t pitch, roll, yaw;
  uint16_t raspberryThrottle;
  uint16_t esc_1, esc_2, esc_3, esc_4;
    // read the incoming byte:
    //Serial.println("lop");
    bool msgRecv = recvDroneMsg(rpiTX, 64);

    if (!msgRecv) {
      //do something
      roll = 2048;
      pitch = 2048;
      yaw = 2048;
      raspberryThrottle = 1050;
    } else {
      roll = ((int16_t) rpiTX[0]) | (((int16_t) rpiTX[1]) << 8);
      pitch = ((int16_t) rpiTX[2]) | (((int16_t) rpiTX[3]) << 8);
      yaw = ((int16_t) rpiTX[4]) | (((int16_t) rpiTX[5]) << 8);
      raspberryThrottle = ((uint16_t) rpiTX[6]) | (((uint16_t) rpiTX[7]) << 8);
    }

    //Serial.println(rpiTX[0]);
    //Serial.println(rpiTX[1]);

    // Angle correction

    //this function to read the sensors return the values in degrees
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> angle = imu::Vector<3>(0,0,0);
    quatToEuler(&angle, &quat);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    // TODO test this new code
    //input values -- taken from the gyroscope
    // /w angle correction
    if (firstRun) {
      InputRoll = gyro.x();
      InputPitch = -gyro.y();
      InputYaw = gyro.z();
    } else {
      InputRoll = (InputRoll * 0.7) + (gyro.x() * 0.3);   //Gyro pid input is deg/sec --> we substract from the previous measurement to get it
      InputPitch = (InputPitch * 0.7) + (-gyro.y() * 0.3);  //Gyro pid input is deg/sec.
      InputYaw = (InputYaw * 0.7) + (gyro.z() * 0.3);    //Gyro pid input is deg/sec.
    }

    if (!isnan(InputRoll)) { InputRollPrev = gyro.y();   /*Gyro pid input is deg/sec.*/ } else { InputRoll = InputRollPrev; }
    if (!isnan(InputPitch)) { InputPitchPrev = gyro.z();   /*Gyro pid input is deg/sec.*/ } else { InputPitch = InputPitchPrev; }
    if (!isnan(InputYaw)) { InputYawPrev = gyro.x();   /*Gyro pid input is deg/sec*/ } else { InputYaw = InputYawPrev; }
    uint8_t sys, gyro2, accel, mag;
    sys = gyro2 = accel = mag = 0;
    bno.getCalibration(&sys, &gyro2, &accel, &mag);
    //Serial.println(sys, 4);
    //Serial.println(gyro.x(), 4);
    //Serial.println(InputYaw, 4);
    //Serial.println(InputYawPrev, 4);

    //Serial.println(gyro.y(), 4);
    //Serial.println(gyro.z(), 4);
    //Serial.println(gyro.x(), 4);

    //Serial.write(((int16_t) InputRoll) & 0xFF);
    //Serial.write((((int16_t) InputRoll) >> 8) & 0xFF);
    //Serial.write(((int16_t) InputYaw) & 0xFF);
    //Serial.write((((int16_t) InputYaw) >> 8) & 0xFF);

    //the setpoints must be taken from the raspberry pi

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
      SetpointPitch = (((double) (pitch - deadzoneCorr)) - (angle.z() * 30.72)) / 6.19;
    }
    if (roll < 2040 || roll > 2054) {
      int16_t deadzoneCorr = 0;
      if (roll > 2054) { deadzoneCorr = 2054; }
      else if (roll < 2040) { deadzoneCorr = 2040; }
      SetpointRoll = (((double) (roll - deadzoneCorr)) - (angle.y() * 30.72)) / 6.19;
    }


    /*floatAsBytes rollDeb;
    rollDeb.fval = SetpointRoll;

    Serial.write(rollDeb.bval, 4);

    floatAsBytes pitchDeb;
    pitchDeb.fval = SetpointPitch;

    Serial.write(pitchDeb.bval, 4);*/

    //Serial.write(((uint16_t) SetpointRoll) & 0xFF);
    //Serial.write((((uint16_t) SetpointRoll) >> 8) & 0xFF);
    //Serial.write(((uint16_t) SetpointPitch) & 0xFF);
    //Serial.write((((uint16_t) SetpointPitch) >> 8) & 0xFF);
    //Serial.write(((uint16_t) SetpointYaw) & 0xFF);
    //Serial.write((((uint16_t) SetpointYaw) >> 8) & 0xFF);


    /*InputRoll = 0;
    InputPitch = 0;
    InputYaw = 0;*/
  
    //Calcuate the PID
    myPIDRoll.Compute();
    myPIDYaw.Compute();
    myPIDPitch.Compute();
    //calculate_pid(InputRoll, InputPitch, InputYaw, SetpointRoll, SetpointPitch, SetpointYaw, &OutputRoll, &OutputPitch, &OutputYaw);

    if (raspberryThrottle > 3072) raspberryThrottle = 3072;                                   //We need some room to keep full control at full throttle.


    esc_1 = raspberryThrottle - (int16_t) OutputPitch + (int16_t) OutputRoll - (int16_t) OutputYaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = raspberryThrottle + (int16_t) OutputPitch + (int16_t) OutputRoll + (int16_t) OutputYaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = raspberryThrottle + (int16_t) OutputPitch - (int16_t) OutputRoll - (int16_t) OutputYaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = raspberryThrottle - (int16_t) OutputPitch - (int16_t) OutputRoll + (int16_t) OutputYaw; //Calculate the pulse for esc 4 (front-left - CW)

    Serial.println(InputRoll, 4);
    Serial.println(SetpointRoll, 4);
    Serial.println(OutputRoll, 4);

    /*floatAsBytes rollDeb;
    rollDeb.fval = OutputRoll;

    Serial.write(rollDeb.bval, 4);

    floatAsBytes pitchDeb;
    pitchDeb.fval = OutputPitch;

    Serial.write(pitchDeb.bval, 4);*/

    if (esc_1 < 1024) esc_1 = 1024;                                         //Keep the motors running.
    if (esc_2 < 1024) esc_2 = 1024;                                         //Keep the motors running.
    if (esc_3 < 1024) esc_3 = 1024;                                         //Keep the motors running.
    if (esc_4 < 1024) esc_4 = 1024;                                         //Keep the motors running.

    if (esc_1 > 1200)esc_1 = 1200;                                          //Limit the esc-1 pulse.
    if (esc_2 > 1200)esc_2 = 1200;                                          //Limit the esc-2 pulse.
    if (esc_3 > 1200)esc_3 = 1200;                                          //Limit the esc-3 pulse.
    if (esc_4 > 1200)esc_4 = 1200;                                          //Limit the esc-4 pulse.

    while(micros() - loop_timer < 20000);                                      //We wait until 20000us are passed.
    loop_timer = micros();
    
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

    if (firstRun) { firstRun = false; }
}

#define MAX_ROLL 400
#define MAX_PITCH 400
#define MAX_YAW 400

double pid_i_mem_roll = 0, pid_last_roll_d_error = 0;
double pid_i_mem_pitch = 0, pid_last_pitch_d_error = 0;
double pid_i_mem_yaw = 0, pid_last_yaw_d_error = 0;

void calculate_pid(double roll_i, double pitch_i, double yaw_i, double roll_s, double pitch_s, double yaw_s, double *roll_o, double *pitch_o, double *yaw_o) {
  //Roll calculations
  double pid_error_temp;
  pid_error_temp = roll_i - roll_s;
  pid_i_mem_roll += Ki * pid_error_temp;
  if(pid_i_mem_roll > MAX_ROLL)pid_i_mem_roll = MAX_ROLL;
  else if(pid_i_mem_roll < MAX_ROLL * -1)pid_i_mem_roll = MAX_ROLL * -1;

  *roll_o = Kp * pid_error_temp + pid_i_mem_roll + Kd * (pid_error_temp - pid_last_roll_d_error);
  if(*roll_o > MAX_ROLL)*roll_o = MAX_ROLL;
  else if(*roll_o < MAX_ROLL * -1)*roll_o = MAX_ROLL * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = pitch_i - pitch_s;
  pid_i_mem_pitch += Ki * pid_error_temp;
  if(pid_i_mem_pitch > MAX_PITCH)pid_i_mem_pitch = MAX_PITCH;
  else if(pid_i_mem_pitch < MAX_PITCH * -1)pid_i_mem_pitch = MAX_PITCH * -1;

  *pitch_o = Kp * pid_error_temp + pid_i_mem_pitch + Kd * (pid_error_temp - pid_last_pitch_d_error);
  if(*pitch_o > MAX_PITCH)*pitch_o = MAX_PITCH;
  else if(*pitch_o < MAX_PITCH * -1)*pitch_o = MAX_PITCH * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = yaw_i - yaw_s;
  pid_i_mem_yaw += yawKi * pid_error_temp;
  if(pid_i_mem_yaw > MAX_YAW)pid_i_mem_yaw = MAX_YAW;
  else if(pid_i_mem_yaw < MAX_YAW * -1)pid_i_mem_yaw = MAX_YAW * -1;

  *yaw_o = yawKp * pid_error_temp + pid_i_mem_yaw + yawKd * (pid_error_temp - pid_last_yaw_d_error);
  if(*yaw_o > MAX_YAW)*yaw_o = MAX_YAW;
  else if(*yaw_o < MAX_YAW * -1)*yaw_o = MAX_YAW * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
       Serial.print("0x");
       for (int i=0; i<length; i++) {
         if (data[i]<0x10) {Serial.print("0");}
         Serial.print(data[i],HEX);
         Serial.print(" ");
       }
}

short unsigned int setSpeedMotor(int n, int speedM){
    int speedMotor = map(speedM , 0, 4096, MinSpeed, MaxSpeed);
    pwm.setPWM(n, 0, speedMotor);
    return (short)speedMotor;
}
