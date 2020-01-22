//PID algorithm library
#include <PID_v1.h>

#include <Wire.h>                          

//libraries for controlling the gyroscope
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//ilbrary for controlling the servo motors
#include <Adafruit_PWMServoDriver.h>

uint8_t system, gyro, accel, mag;   //same as byte type   --   values for the gyroscope callibration
sensors_event_t event;  //this variable contains the values of the gyroscope axis x, y, z

//PID algorithm parametres

//Define Variables we'll be connecting to
double SetpointRoll, InputRoll, OutputRoll;
 
//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPIDRoll(&InputRoll, &OutputRoll, &SetpointRoll, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double SetpointPitch, InputPitch, OutputPitch;
PID myPIDPitch(&InputPitch, &OutputPitch, &SetpointPitch, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double SetpointYaw, InputYaw, OutputYaw;
PID myPIDYaw(&InputYaw, &OutputYaw, &SetpointYaw, Kp, Ki, Kd, DIRECT);

void setup(){
  //initialize connection with the raspberry pi with the serial port 9600                                                                           
  Serial.begin(9600);
  //wait for the serial port to be connected
  while(!Serial){;}
  
  //Turn on the warning led.
  digitalWrite(12,HIGH);  
                                                    
  /* Initialise the gyroscope */
  if(!bno.begin()){
    Serial.print("Ups, no gyroscope detected");
    while(1);
  }
  delay(1000); 
  bno.setExtCrystalUse(true);
  
  //start the servo motors
  void Adafruit_PWMServoDriver::begin(uint8_t prescale = 0);
  
  //set the frecuency of the motors to 50 hz
  pwm.setPWMFreq(50);
  
  //turn on the motors in a low throttle
  pwm.setPWM(1, 1024, 3072);
  pwm.setPWM(2, 1024, 3072);
  pwm.setPWM(3, 1024, 3072);
  pwm.setPWM(4, 1024, 3072);

  delay(2000);

  //turn off the motors
  pwm.setPWM(1, 0, 4096);
  pwm.setPWM(2, 0, 4096);
  pwm.setPWM(3, 0, 4096);
  pwm.setPWM(4, 0, 4096);
  
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
  Serial.print("\t");
  if (!system){ //the system variable is 0 untill the gyroscope is callibrated  --  the calibrating qualitie varies from 0 to 3 being 3 the optimal callibration
    while(1);//this line is for the system not to execute if the values of the gyroscope are wrong -- remove if in doesn't work
  }

  //When everything is done, turn off the led.
  digitalWrite(12,LOW);  
}

void loop(){
  //read the information from the raspberry pi
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);

    //read the information from the gyroscope
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    //event.orientation.y x, or z gets the floating point data of the gyroscope axis
    
    //this function to read the sensors return the values in degrees per second 
    //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    //the setpoints must be taken from the raspberry pi
    SetpointYaw = ;
    SetpointPitch = ;
    SetpointRoll = ;
               
    //input values -- taken from the gyroscope                                                                                                                                                 
    InputRoll = event.orientation.x;   //Gyro pid input is deg/sec.   
    InputPitch = event.orientation.y;  //Gyro pid input is deg/sec.
    InputYaw = event.orientation.z;    //Gyro pid input is deg/sec.

    //Calcuate the PID
    myPIDRoll.Compute();
    myPIDYaw.Compute();
    myPIDPitch.Compute();
    
    //The battery voltage is needed for compensation.
    //A complementary filter is used to reduce noise.
    //0.09853 = 0.08 * 1.2317.
    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  
    throttle = read input throttle from the raspberry pi;
  
    if (throttle > 3072) throttle = 3072;                                   //We need some room to keep full control at full throttle.                                   
  
    esc_1 = throttle - OutputPitch + OutputRoll - OutputYaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + OutputPitch + OutputRoll + OutputYaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + OutputPitch - OutputRoll - OutputYaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - OutputPitch - OutputRoll + OutputYaw; //Calculate the pulse for esc 4 (front-left - CW)
    
    if (esc_1 < 1024) esc_1 = 1024;                                         //Keep the motors running.                                                                     
    if (esc_2 < 1024) esc_2 = 1024;                                         //Keep the motors running.
    if (esc_3 < 1024) esc_3 = 1024;                                         //Keep the motors running.
    if (esc_4 < 1024) esc_4 = 1024;                                         //Keep the motors running.
  
    if(esc_1 > 3072)esc_1 = 3072;                                           //Limit the esc-1 pulse.                                                            
    if(esc_2 > 3072)esc_2 = 3072;                                           //Limit the esc-2 pulse.
    if(esc_3 > 3072)esc_3 = 3072;                                           //Limit the esc-3 pulse.
    if(esc_4 > 3072)esc_4 = 3072;                                           //Limit the esc-4 pulse.  
  
    pwm.setPWM(1, esc_1, 4096 - esc_1);
    pwm.setPWM(2, esc_2, 4096 - esc_2);
    pwm.setPWM(3, esc_3, 4096 - esc_3);
    pwm.setPWM(4, esc_4, 4096 - esc_4);

    //send to the raspberry pi the new values of the motors
    imu::Quaternion quat = bno.getQuat();
    Serial.write(quat.w(), 4);
    Serial.write(quat.y(), 4);
    Serial.write(quat.x(), 4);
    Serial.write(quat.z(), 4);
  }
}
