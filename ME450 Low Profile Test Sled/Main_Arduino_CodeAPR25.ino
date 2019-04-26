			////////////////////////////////////////////////////////////////////////
			//				ME450: Low Profile Sled								                      //
			// Dylan Keyes, Kris Cenko, Tomas Szot, Jonathan Guerra, and Zelin Pu //
			////////////////////////////////////////////////////////////////////////

//Include Libraries
#include <PID_v1.h>  													        //Library for PID by:Brett Beauregard
#include <math.h> 														        //Library for angle difference calucations
#include "quaternionFilters.h" 											  //Library for filters associated with IMU
#include "MPU9250.h" 													        //IMU Library by: SparkFun Electronics
#include "SparkFun_Ublox_Arduino_Library.h" 					//GPS Library http://librarymanager/All#SparkFun_Ublox_GPS
#include <Wire.h> 														        //Needed for I2C communications

//GPS intended path coordinates -- should be input by user before running application
double PathLats[] = {42.29374267,42.29374317}; 				//intended path latitudes in order of route taken, measured with RTK Rover
double PathLongs[] = {-83.71471700,-83.71470250}; 		//intended path longtitudes in order of route taken, measured with RTK Rover
double NumOfGPSPoints = sizeof(PathLats);							//total number of path points

//Pin assignments on Arduino Mega
int LeftMotorPWMPin = 2; 												      //input pin for left motor PWM 
int LeftMotorDIR = 6; 													      //input pin for left motor direction
int LeftMotorEncoderPin = 18;											    //input pin for left motor encoder 
int RightMotorPWMPin = 3; 												    //input pin for right motor PWM
int RightMotorDIR = 7; 													      //input pin for right motor direction
int RightMotorEncoderPin = 19;											  //input pin for right motor encoder 

//Physical parameters used in further calculations
double CPR = 32;  														        //encoder's counts per shaft revolution per channel
double MotorGearRatio = 30; 											    //motor gear ratio, fixed from manufacturer
double WheelDiameter = 3.53 ; 											  //outside diameter of wheel as measured with dial calipers [in] 

//integer initializiations
volatile double LeftMotorEncoderCount = 0; 						//counter for left motor's pulses
volatile double RightMotorEncoderCount = 0; 					//counter for right motor's pulses
double MotorTotalDistance = 0.0; 										  //motor total driven distance [in]
double shaftPulses = 0; 												      //counter of encoder shaft pulses
double dt = 0; 															          //change in time 
long loopStartTime = 0; 												      //initial loop start time [s]
long currentTime = 0; 													      //current time [s]
double RightWheelSpeed = 0; 											    //right wheel speed
double LeftWheelSpeed = 0; 												    //left wheel speed
double CurrentLat = 0;													      //GPS Latitude
double CurrentLong = 0;													      //GPS Longitude
long lastTime = 0;
double PosBuffer = .00000100; 											  //postional buffer to account for inaccuracy with GPS reciever

//PID Initializations								
double Setpoint, Input, Output;											  //intialization of PID variables
double Kp=3000000, Ki=2000000, Kd=2;									//defining PID parameters, trial and error until ideal response is reached.

//IMU Initializations
#define AHRS true         												    // Set to false for basic data read
#define SerialDebug true  												    // Set to true to get Serial output for debugging
#define I2Cclock 400000													      // Set I2C clockspeed
#define I2Cport Wire													        // Establish I2C communication
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   				// Use either this line or the next to select which I2C address
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

//Sensor class initializations
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);				//PID initialization
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);						    //IMU initialization
SFE_UBLOX_GPS myGPS;													                    //GPS initialization




//////Set-up Code only runs once//////
void setup(){
  Serial.begin(115200); 																	                                    //establish serial baud rate, ensure serial monitor matches
  attachInterrupt(digitalPinToInterrupt(RightMotorEncoderPin), CountPulsesRight, CHANGE);	    //establish right motor's encoder as an interupt, when interuppted run CountPulsesRight ISR
  attachInterrupt(digitalPinToInterrupt(LeftMotorEncoderPin), CountPulsesLeft, CHANGE);		    //establish left motor's encoder as an interupt, when interuppted run CountPulsesleft ISR
  pinMode(LeftMotorPWMPin, OUTPUT);															                              //establish left motor pwm as output pin
  pinMode(LeftMotorDIR, OUTPUT);															                                //establish left motor dir as output pin
  myPID.SetMode(AUTOMATIC); 																                                  //turn  myPID on

  Wire.begin(); 																			              //begin I2C Communication
  myGPS.begin(); 																			              //begin GPS Communication
  
  //MotorControlledIMUCalibration(); 															  //CalibrateIMU, not being utilized due to bad IMU/Yaw sensor. Should be done in later versions. 
}

//////Main loop which runs continutously//////
void loop(){

/* Debug code to obtain values while running, comment/uncomment as neccessarry.  
		//Left Encoder Test: used to determine if interupts and encoder counts are working properly. 
//Serial.print("Left Encoder Test:");
//Serial.print(LeftMotorEncoderCount); 
//Serial.print("\t");
    
		//Right Encoder Test: used to determine if interupts and encoder counts are working properly. 
//Serial.print("Right Encoder Test:");
//Serial.print(RightMotorEncoderCount);
//Serial.println("\t");

		//Encoder to Linear Distance Test: used to determine if encoder counts can be used to determine travel distance. 
//ShaftPulsesToLinearDistance(RightMotorEncoderCount);
//Serial.print("Right Wheel Total Distance:");
//Serial.print(MotorTotalDistance); 
//Serial.println("\t"); 

		//Right Wheel Speed Test: used to view right wheel speed 
//RightWheelSpeed = CalculateRightWheelSpeed(RightMotorEncoderCount);
//Serial.print("Right Wheel Speed ");
//Serial.print(RightWheelSpeed,9);
//Serial.println("\t");

		//Left Wheel Speed Test: used to view left wheel speed 
//LeftWheelSpeed = CalculateLeftWheelSpeed(LeftMotorEncoderCount);
//Serial.print("Left Wheel Speed");
//Serial.print(LeftWheelSpeed,9);
//Serial.print("\t");
    
		//PID test: used to view the output PWM value for PID motor control. 
//Serial.print("Output");
//Serial.print(Output,9);
//Serial.print("\t");

		//Get Yaw Function call. 
//GetYaw();

		//Calibrate IME Function Call
//CalibrateIMU();

		//GPS Test: used to view the output of the GPS system. 
//ReturnLLPosition(Lat,Long);
//Serial.print("Lat ");
//Serial.print(Lat,9);
//Serial.print("\t");
//Serial.print("Long ");
//Serial.println(Long,9);
*/

////Non Debug Code (REAL)///

ReturnLLPosition(CurrentLat,CurrentLong); 													              //Returns current positions

//rotate until heading is matched with projected heading
int i = 1; 																					                              //incrementanal counter to determine what path points have been reached. 
double NextLat = PathLats[i]; 																                    //establish Next lat as next element in Path lat array, update once point is reached.
double NextLong = PathLongs[i]; 															                    //establish Next long as next element in Path long array, update once point is reached
RotateUntilCorrectHeading(CurrentLat, CurrentLong, NextLat, NextLong);						//rotate until heading is correct, NOT WORKING due to IMU issues. 

//move forward until GPS coordinates match
while (1>0){																				                              //force loop to execute coniuously
  //double LatDiff = NextLat - CurrentLat;                                        //debug variable initializations
  //double LongDiff = NextLong - CurrentLong; 
  //Serial.print("Lat: ");                                                        //serial print debug messages for GPS system
  //Serial.print(CurrentLat,9);
  //Serial.print("\t");
  //Serial.print("Long: ");
  //Serial.print(CurrentLong,9);
  //Serial.print("     NextLat: ");
  //Serial.print(NextLat,9);
  //Serial.print("\t");
  //Serial.print("NextLong: ");
  //Serial.print(NextLong,9);
  //Serial.print("     LattDiff: ");
  //Serial.print(LatDiff,9);
  //Serial.print("\t");
  //Serial.print("LongDiff: ");
  //Serial.println(LongDiff,9);
  ReturnLLPosition(CurrentLat, CurrentLong);												                        //get current lat and long
  if ( ((CurrentLat != (NextLat+PosBuffer)) && (CurrentLat != (NextLat-PosBuffer))) || 		  //If current Lat/Long don't match intended, move forward
       ((CurrentLong != (NextLong+PosBuffer)) && (CurrentLong != (NextLong-PosBuffer))) )
     {
     MoveForward2();																		                                    //move forward if current lat/long don't match intended, utilized MoveForward2, which is just PWM inputs. 
     //Serial.print("inside if");															                              //debug serial print to show when inside if statements. 
     }
     
  else{																						                                          //Else, Lat/Long do match then exit loop.
      Serial.print("break");																                                //debug serial print to show when lat/long match.
      break;																				
      }
  i++;																						                                          //update counter for next GPS path point
  analogWrite(LeftMotorPWMPin, 0); 														                              //debug serial write to force rover to stop when lat/long match.
  analogWrite(RightMotorPWMPin, 0);														                              //debug serial write to force rover to stop when lat/long match.
}
}

               ////////////////////////
              //                    //
  /////////////                    /////////////////              
 ////           Created Functions              ////
//////////////                    ////////////////  
           //                    //
          ////////////////////////

//function to rotate rover until looking heading matches required heading, also calls function to determine required heading.
//currently doesn't work properly due to bad GetYaw() function (IMU not accurate). Force it to be 0, to continue development
//of rover after assuming rover rotated properly to the next point. 
void RotateUntilCorrectHeading(double Lat1, double Long1, double Lat2, double Long2)
{
  double RequiredHeading = angleFromCoordinate(Lat1, Long1, Lat2, Long2);					          //determine required angle between two known points. 
  double CurrentHeading = GetYaw(); 														                            //determine current heading
  double HeadingDifference = RequiredHeading - CurrentHeading;								              //difference between current heading and required heading
  HeadingDifference = 0; 																	                                  //force Heading Difference to be 0, include in code for now to continue development. 

  if (HeadingDifference > 0) 																                                //turn "right" required scenario
  {	
   analogWrite(LeftMotorPWMPin, 100); 														                          //apply more power to left wheel to turn right.						
   analogWrite(RightMotorPWMPin, 0);														
  }

  if (HeadingDifference < 0) 																                                //Turn "left" required scenario
  {
   analogWrite(LeftMotorPWMPin, 0); 
   analogWrite(RightMotorPWMPin, 100);														                          //apply more power to right wheel to turn left
  }
  
return;
}

//function to update current latitude and current longitiude from GPS system. 
void ReturnLLPosition(double &CurrentLat, double &CurrentLong){
  CurrentLat = myGPS.getLatitude() * .0000001; 												                        //query GPS for Latitude, convert to match coordinate system.
  CurrentLong = myGPS.getLongitude() * .0000001;											                        //query GPS for longitiude, convert to match coordinate system.

return;
}

//function to calibrate IMU sensor. Expected total magnetic field near Ann Arbor, MI. 534 mG per NOAA
void CalibrateIMU(){
  
	//Read the WHO_AM_I register, this is a good test of communication
	byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); 							                    //MPU9250 is the Accelerometer/Gyrometer sensor
	Serial.print(F("MPU9250 I AM 0x"));
	Serial.print(c, HEX);
	Serial.print(F(" I should be 0x"));
	Serial.println(0x71, HEX);
  
	byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963); 								                    //AK8963 is the Magnetometer sensor
	Serial.print("AK8963 ");
	Serial.print("I AM 0x");
	Serial.print(d, HEX);
	Serial.print(" I should be 0x");
	Serial.println(0x48, HEX);
 
    myIMU.initAK8963(myIMU.factoryMagCalibration); 											                          //initialize device for active mode read of magnetometer
    
    myIMU.getAres(); 																		                                          //get sensor resolutions, only need to do this once
    myIMU.getGres();
    myIMU.getMres();

    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale); 									                        //delays for 4 seconds, and then records about 15 seconds of data to calculate bias and scale.
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)"); 
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
	Serial.println(myIMU.magScale[2]);
}

//Function returns yaw calculation from IMU sensor
double GetYaw(){ 
  
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } 

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

      myIMU.delt_t = millis() - myIMU.count;

      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of ann arbor, mi is -6.57° on 2019-04-25 per NOAA
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 6.57;
      myIMU.roll *= RAD_TO_DEG;

      if(SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");
      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
  }

//Function used to control rover during IMU calibration to ensure consistent physical movment between test runs. 
//both motors should have power and also variable positions in earth. This is an attempt to correct for any 
//alterations due to the EMF/Mag field of the motors and also in earth. 
void MotorControlledIMUCalibration(){
   
   double StartTime = 0; 
   double StartTime2 = 0; 
   double IMUCalTime = 0;
      
   StartTime = millis();																//time since power on and calibration start. [ms]
   
   while(IMUCalTime < 10000){								
      //Serial.println("inside while"); 								//debug serial print
      //Serial.print("IMUCalTime:");										//debug serial print
      //Serial.println(IMUCalTime);											//debug serial print
      analogWrite(LeftMotorPWMPin, 125); 								//apply power to both motors before calibration
      analogWrite(RightMotorPWMPin, 60);
      CalibrateIMU();																	  //calibration IMU Function call
      StartTime2 = millis();												
      IMUCalTime = StartTime2 - StartTime; 							//how long IMU has been calibrating
      Serial.print("StartTime:");												//debug serial print
      Serial.println(StartTime);												//debug serial print
      Serial.print("StartTime2:");											//debug serial print
      Serial.println(StartTime2);												//debug serial print
      }
      
   analogWrite(LeftMotorPWMPin, 0); 										//remove power from left motor as calibration is completed
   analogWrite(RightMotorPWMPin, 0);										//remove power from right motor as calibration is completed
} 

//Function to move rover forward utilizing by appling straight PWM values (0-255).
//difference between the two in moving forward what determined experimentally to obtain
//a straight path. 
void MoveForward2(){
  analogWrite(LeftMotorPWMPin, 80); 													//Apply left PWM power
  analogWrite(RightMotorPWMPin, 90); 													//Apply right PWM power
  }

//Function to move rover forward utilizing motor encoders and a PID. Not being currently used as motor PID response is not good enough. 
//Testing included having 1 motor PID controlled to a wheel speed setpoint - which worked quite well. However, while the final speed 
//would closely match the setpoint speed, it was assumed that implementing the same controller on both wheels, would not mean the 
//response of both wheels would be the same - this should be tested in later versions. 
//What is currently being down below is, having a "master motor" speed set, and then have the wheel speed difference between the 
//two wheels PID controlled to 0. It was found to have a better response by giving both motors some initial values based off of experimentally
//determined values. 
void MoveForward()
  {
  analogWrite(LeftMotorPWMPin, 80); 													                //establish some power to left motor
  analogWrite(RightMotorPWMPin, 90);													                //establish some power to right motor
  
  LeftWheelSpeed = CalculateLeftWheelSpeed(LeftMotorEncoderCount);						//determine left wheel speed from encoder count
  RightWheelSpeed = CalculateRightWheelSpeed(RightMotorEncoderCount);   			//determine right wheel speed from encoder count
  
  double WheelSpeedDiff = RightWheelSpeed - LeftWheelSpeed;								    //calculate difference between both wheels
 

 // Serial.print("WheelSpeedDiff ");													                //debug serial print line
 // Serial.println(WheelSpeedDiff,9);													                //debug serial print line
 
  Input = WheelSpeedDiff;																                      //PID varible input
  Setpoint = 0;																			                          //Set wheel difference to setpoint of 0
      
  if (RightWheelSpeed > LeftWheelSpeed){            									        //Turning left scenario
    myPID.Compute();																	                        //calulate output from PID controller
    analogWrite(RightMotorPWMPin, Output);											            	//apply output from PID controller to right motor
    //Serial.print("right wheel faster:");												            //debug serial print line
    //Serial.print("\t");																                      //debug serial print line
    //Serial.print("Output ");															                  //debug serial print line
    //Serial.println(Output,9);															                  //debug serial print line
    }
    
  else {                                            									        //Turning right scenario
    myPID.Compute();																	                        //calulate output from PID controller
    analogWrite(RightMotorPWMPin, Output);												            //apply output from PID controller to right motor
    //Serial.print("left wheel faster:");												              //debug serial print line
    //Serial.print("\t");																                      //debug serial print line
    //Serial.print("Output ");															                  //debug serial print line
    //Serial.println(Output,9);															                  //debug serial print line
    }
}

//Determine right wheel speed utilizing motor encoder counts [in/microsecond]
  double CalculateRightWheelSpeed(volatile double RightMotorSide){
  
  double Rdx1 = ShaftPulsesToLinearDistance(RightMotorSide); 							//get initial right motor total distance traveled
  double Rdt1 = micros(); 																                //get initial time when right motor total distance is calculated
  delay(100);
  
  RightMotorSide = RightMotorEncoderCount;												        //udpate RightMotorSide with latest right motor encoder count
  double Rdx2 = ShaftPulsesToLinearDistance(RightMotorSide);							//get final right motor total distance traveled
  double Rdt2 = micros(); 																                //get final time when right motor total distance is calculated
  double RightWheelSpeed = (Rdx2 - Rdx1)/ (Rdt2-Rdt1); 									  //determine  right wheel speed

  return RightWheelSpeed; 
}

//Determine left wheel speed utilizing motor encoder counts [in/microsecond]
double CalculateLeftWheelSpeed(volatile double LeftMotorSide){
  
  double Ldx1 = ShaftPulsesToLinearDistance(LeftMotorSide);  							//get initial right motor total distance traveled
  double Ldt1 = micros(); 																                //get initial time when right motor total distance is calculated
  delay(100);
  
  LeftMotorSide = LeftMotorEncoderCount;												          //udpate RightMotorSide with latest right motor encoder count
  double Ldx2 = ShaftPulsesToLinearDistance(LeftMotorSide);								//get final right motor total distance traveled
  double Ldt2 = micros();  																                //get final time when right motor total distance is calculated
     
  double LeftWheelSpeed = (Ldx2 - Ldx1)/ (Ldt2-Ldt1); 									  //determine  right wheel speed
    
  return LeftWheelSpeed; 
}

//function to convert motor shaft pulses into a total linear distance[in]. 
double ShaftPulsesToLinearDistance(double ShaftPulses){
 MotorTotalDistance = ((ShaftPulses / CPR) / MotorGearRatio) * (PI*WheelDiameter);              //convert total encoder counts to made, to a linear distance traveled.
 return MotorTotalDistance;
 }

//function to measure the angle between two lats and longs, input must be in radians
    double angleFromCoordinate(double lat1, double long1, double lat2, double long2) {

    double dLon = (long2 - long1);

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    double brng = atan2(y, x); //measured in radians

    brng = brng * 180 / PI;
    //brng = (brng + 360) % 360;
    //brng = 360 - brng;                                                                         // count degrees counter-clockwise - remove to make clockwise

    return brng;
}

//ISR, Interupt Service Routine. Increment needed for right encoder pulse counter
void CountPulsesRight(){
  RightMotorEncoderCount++;
  }

//ISR, Interupt Service Routine. Increment needed for left encoder pulse counter
void CountPulsesLeft(){
  LeftMotorEncoderCount++;
  }
