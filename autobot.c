#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
///int abc = 0;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
int readVal;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
int stable = 1000;
const byte analogPin = A8;
int a1 = 0;
float diff, abc = 0;
int frontRight1 = 22, frontRightpwm = 6, frontLeft1 = 23, frontLeftpwm = 3, backRight1 = 25, backRightpwm = 7, backLeft1 = 24, backLeftpwm = 8;
int proportional, high_speed = 55, high_speed1 = 70, high_speed2 = 80, low_speed = 25, KP = 70, pwm;
int weight_index;
int jc = 0, a = 0, qq = 0;
int stablede = 1200;
int stable1 = 1000;
int weights[13] = {0, 4, 6, 8, 10, 12, 15, 18, 20, 25, 30, 35, 40};
int IR = 9;
int v=0;
int IRRead(int n);
void Throw();
void Stop();
int tz=1;
void Front(), Back();
void Left1(int FRPWM, int FLPWM, int BRPWM, int BLPWM);
void Correction(float);
void Right1(int FRPWM, int FLPWM, int BRPWM, int BLPWM);
void Fordward1(int FRPWM, int FLPWM, int BRPWM, int BLPWM);
void Right(int FRPWM, int FLPWM, int BRPWM, int BLPWM);
void Left(int FRPWM, int FLPWM, int BRPWM, int BLPWM);
void Forward(int FRPWM, int FLPWM, int BRPWM, int BLPWM);
void Backward(int FRPWM, int FLPWM, int BRPWM, int BLPWM);
int counter = 0;
int d=0;

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  for (byte i = 46; i <= 53; i++) {
    pinMode(i, INPUT);
  }
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
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
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(20, HIGH);
  pinMode(frontRight1, OUTPUT);
  pinMode(A8, INPUT);
  pinMode(36, INPUT);
  pinMode(frontRightpwm, OUTPUT);
  pinMode(frontLeft1, OUTPUT);

  pinMode(frontLeftpwm, OUTPUT);
  pinMode(backRight1, OUTPUT);

  pinMode(backRightpwm, OUTPUT);
  pinMode(backLeft1, OUTPUT);

  pinMode(backLeftpwm, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  pinMode(IR, INPUT);
  // configure LED for output
}




void loop() {

  readVal = analogRead(analogPin);

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
    stable--;
    if (stable == 2)
    {
      a1 = ypr[0] * 180 / M_PI;
      abc = a1;
      Serial.print(" A1  ");
      Serial.println(a1);
    }
#endif
    if (stable < 0) {
      Serial.println("Working");
      a = ypr[0] * 180 / M_PI;
      Serial.print(a);
      diff = a1 - a;
      Serial.print(diff);
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);

      // blink LED to indicate activity
      /* blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        Serial.println(" ");
        Serial.print(digitalRead(46));
        Serial.print(digitalRead(47));
        Serial.print(digitalRead(48));
        Serial.print(digitalRead(49));
        Serial.print(digitalRead(50));
        Serial.print(digitalRead(51));
        Serial.print(digitalRead(52));*/
      Serial.print(digitalRead(36));
      Serial.println();
      if ( digitalRead(36))
      { jc = jc + 1;
        Serial.println("Junction Count\n");
        // digitalWrite(20, LOW);
        if (jc == 1) {
          Serial.print(digitalRead(36));
          /* Serial.print(digitalRead(47));
            Serial.print(digitalRead(48));
            Serial.print(digitalRead(49));
            Serial.print(digitalRead(50));
            Serial.print(digitalRead(51));
            Serial.print(digitalRead(52));
            Serial.print(digitalRead(53));*/
          Back();
        }
        Serial.println(jc);
        Serial.println("Main aagaya Waapis...........");   
        }



        if (digitalRead(34)==LOW)  {

          if(jc==4){
          Throw();
          delay(1000);
          }
          }



       if (jc==5) {
          Serial.print(digitalRead(36));
        //  delay(1000);
           Serial.print(digitalRead(47));
            Serial.print(digitalRead(48));
            Serial.print(digitalRead(49));
            Serial.print(digitalRead(50));
            Serial.print(digitalRead(51));
            Serial.print(digitalRead(52));
            Serial.print(digitalRead(53));
            jc++;
          Front();
        }

         if (jc == 8) {
          Stop();
          delay(1000);
          high_speed=55;
          v++;
         
          
          if((v<2 && tz==1) || (v<=5 && tz==2))
          {
          jc=2;
          Back();
          Serial.println("22222222222222222");
          }
          else 
          {
            jc=0;
            tz=2;
            v=0;
          }
        }

     /* if (jc == 5) {
        Serial.print(digitalRead(36));
        
        /*Serial.print(digitalRead(47));
          Serial.print(digitalRead(48));
          Serial.print(digitalRead(49));
          Serial.print(digitalRead(50));
          Serial.print(digitalRead(51));
          Serial.print(digitalRead(52));
          Serial.print(digitalRead(53));
        Throw();
      }*/

    /* if (jc == 6) {
        Serial.print(digitalRead(36));
         Serial.print("AAAAAAAAAAAAAAAAAAAAAAAA");
        /* Serial.print(digitalRead(47));
          Serial.print(digitalRead(48));
          Serial.print(digitalRead(49));
          Serial.print(digitalRead(50));
          Serial.print(digitalRead(51));
          Serial.print(digitalRead(52));
          Serial.print(digitalRead(53));
        //Forward(40, 40, 40, 40);
      */





      else if (abs(diff) <= 0.2) {
        qq = 0;
        Serial.println(qq);
      }
      else if (abs(diff) > 0.2 && abs(diff) <= 0.4) {
        qq = 1;
        Serial.println(qq);
      }
      else if (abs(diff) > 0.4 && abs(diff) <= 0.6) {
        qq = 2;
        Serial.println(qq);
      }
      else if (abs(diff) > 0.6 && abs(diff) <= 0.8) {
        qq = 3;
        Serial.println(qq);
      }
      else if (abs(diff) > 0.8 && abs(diff) <= 1.2) {
        qq = 4;
        Serial.println(qq);
      }
      else if (abs(diff) > 1.2 && abs(diff) <= 1.6) {
        qq = 5;
        Serial.println(qq);
      }
      else if (abs(diff) > 1.6 && abs(diff) <= 1.8) {
        qq = 6;
        Serial.println(qq);
      }
      else if (abs(diff) > 1.8 && abs(diff) <= 2.2) {
        qq = 7;
        Serial.println(qq);
      }
      else if (abs(diff) > 2.2 && abs(diff) <= 2.8) {
        qq = 8;
        Serial.println(qq);
      }
      else if (abs(diff) > 2.8 && abs(diff) <= 3.6) {
        qq = 9;
        Serial.println(qq);
      }
      else if (abs(diff) > 3.6 && abs(diff) <= 4.6) {
        qq = 10;
        Serial.println(qq);
      }
      else if (abs(diff) > 4.6 && abs(diff) <= 7) {
        qq = 11;
        Serial.println(qq);
      }
      else if (abs(diff) > 7) {
        qq = 12;
        Serial.println(qq);
      }

      switch (qq) {
        case 0:
          Serial.print("case 0:");
          pwm = weights[0];
          Forward(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
          Serial.println("FORARD");
          break;
        case 1:
          Serial.print("case 1:");
          pwm = weights[1];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;

        case 2:
          Serial.print("case 2:");
          pwm = weights[2];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;

        case 3:
          Serial.print("case 3:");
          pwm = weights[3];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;
        case 4:
          Serial.print("case 4:");
          pwm = weights[4];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;
        case 5:
          Serial.print("case 5:");
          pwm = weights[5];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;

        case 6:
          Serial.print("case 6:");
          pwm = weights[6];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;

        case 7:
          Serial.print("case 7:");
          pwm = weights[7];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;
        case 8:
          Serial.print("case 8:");
          pwm = weights[8];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;
        case 9:
          Serial.print("case 9:");
          pwm = weights[9];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;
        case 10:
          Serial.print("case 10:");
          pwm = weights[10];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;
        case 11:
          Serial.print("case 11:");
          pwm = weights[11];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;
        case 12:
          Serial.print("case 12:");
          pwm = weights[12];
          if (diff < 0)
          {
            Left(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
            Serial.println("Left");
          }
          if (diff > 0)
          {
            Right(high_speed - pwm, high_speed + pwm, high_speed - pwm, high_speed + pwm);
            Serial.println("Right");
          }
          break;
        default:
          break;
      }
      Serial.print("PWM:");
      Serial.print(pwm);


    }

  }
}


void Back()
{
if(tz==2)
{jc=2;}

  digitalWrite(frontRight1, LOW);

  digitalWrite(frontLeft1, HIGH);

  digitalWrite(backLeft1, LOW);

  digitalWrite(backRight1, HIGH);

  analogWrite(frontRightpwm, 0);
  analogWrite(frontLeftpwm, 0);
  analogWrite(backLeftpwm, 0);
  analogWrite(backRightpwm, 0);
  Serial.print("\nDown PWM:");
  digitalWrite(20, HIGH);
  delay(1000);
  int i = 0;

  while (true)
  {

    int ircount = 0, b = 0;
    /* ircount=IRRead(31);
      if(ircount>(31-ircount)){
      Serial.println("There");
      b=IRRead(20);
    */
 if (jc == 2){
                Serial.println("ballllllllllllllll");
                while(digitalRead(IR)==HIGH){Serial.println("111111111111");
                   analogWrite(frontRightpwm, 0);
            analogWrite(frontLeftpwm, 0);
              analogWrite(backLeftpwm, 0);  
             analogWrite(backRightpwm, 0); delay(1000);
             
            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                      // display Euler angles in degrees
                      mpu.dmpGetQuaternion(&q, fifoBuffer);
                      mpu.dmpGetGravity(&gravity, &q);
                      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                      Serial.print("ypr\t");
                      Serial.print(ypr[0] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.print(ypr[1] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.println(ypr[2] * 180 / M_PI);

                      abc = ypr[0] * 180 / M_PI;

#endif
                      diff = a1 - abc;

                while (abs(diff) > 0.4)
                {     
                                  // if programming failed, don't try to do anything
                    Serial.println("Start");
                    if (!dmpReady) return;

                    // wait for MPU interrupt or extra packet(s) available
                    while (!mpuInterrupt && fifoCount < packetSize) {
                      // other program behavior stuff here
                      // .
                      // .
                      // .
                      // if you are really paranoid you can frequently test in between other
                      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
                      // while() loop to immediately process the MPU data
                      // .
                      // .
                      // .
                    }

                    // reset interrupt flag and get INT_STATUS byte
                    mpuInterrupt = false;
                    mpuIntStatus = mpu.getIntStatus();

                    // get current FIFO count
                    fifoCount = mpu.getFIFOCount();

                    // check for overflow (this should never happen unless our code is too inefficient)
                    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                      // reset so we can continue cleanly
                      mpu.resetFIFO();
                      Serial.println(F("FIFO overflow!"));

                      // otherwise, check for DMP data ready interrupt (this should happen frequently)
                    } else if (mpuIntStatus & 0x02) {
                      // wait for correct available data length, should be a VERY short wait
                      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

                      // read a packet from FIFO
                      mpu.getFIFOBytes(fifoBuffer, packetSize);

                      // track FIFO count here in case there is > 1 packet available
                      // (this lets us immediately read more without waiting for an interrupt)
                      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
                      // display Euler angles in degrees
                      mpu.dmpGetQuaternion(&q, fifoBuffer);
                      mpu.dmpGetGravity(&gravity, &q);
                      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                      Serial.print("ypr\t");
                      Serial.print(ypr[0] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.print(ypr[1] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.println(ypr[2] * 180 / M_PI);

                      //abc = ypr[0] * 180 / M_PI;

#endif
                  
                      Correction(diff);
                      abc = ypr[0] * 180 / M_PI;
                      diff = a1-abc;
                      Serial.println("Diffvalue");
                      Serial.println(diff);}}
             }
     while(b<50){ b++;
      delay(100);
      Serial.println(b);
      Serial.println("There");
                
                
                }
                
Backward(50,50,50,50);
  analogWrite(frontRightpwm, 0);
  analogWrite(frontLeftpwm, 0);
  analogWrite(backLeftpwm, 0);
  analogWrite(backRightpwm, 0);
  delay(200);
                }
    


    while ((digitalRead(IR) == LOW && jc>=2 )|| jc<2)
    {Serial.println("hjgjhghygjgk");
      if(jc!=1){
        analogWrite(frontRightpwm, 0);
  analogWrite(frontLeftpwm, 0);
  analogWrite(backLeftpwm, 0);  
   analogWrite(backRightpwm, 0); 
      b++;
      delay(100);
      Serial.println(b);
      Serial.println("There");}
      while ((b > 50 && jc>=2)||jc<2) {

       /* while (digitalRead(IR) == HIGH)
        {
          Serial.print("BALLL BALLL BALLLL");
          //PUT BACKWARD ka code
        }*/


        // if programming failed, don't try to do anything
        Serial.println("Start");
        if (!dmpReady) return;

        // wait for MPU interrupt or extra packet(s) available
        while (!mpuInterrupt && fifoCount < packetSize) {
          // other program behavior stuff here
          // .
          // .
          // .
          // if you are really paranoid you can frequently test in between other
          // stuff to see if mpuInterrupt is true, and if so, "break;" from the
          // while() loop to immediately process the MPU data
          // .
          // .
          // .
        }

        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          Serial.println(("FIFO overflow!"));

          // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
          // wait for correct available data length, should be a VERY short wait
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);

          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          Serial.print("ypr\t");
          Serial.print(ypr[0] * 180 / M_PI);
          Serial.print("\t");
          Serial.print(ypr[1] * 180 / M_PI);
          Serial.print("\t");
          Serial.println(ypr[2] * 180 / M_PI);
          stable--;
          if (stable == 2)
          {
            // a1 = ypr[0] * 180 / M_PI;
            Serial.print(" A1  ");
            Serial.println(a1);
          }
#endif

          if (stable < 0) {
            Serial.println("Working");
            a = ypr[0] * 180 / M_PI;
            Serial.print(a);
            diff = a1 - a;
            Serial.print(diff);
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);

            // blink LED to indicate activity
            blinkState = !blinkState;
            high_speed = 80;
            Serial.print("READVALLLLLL");
            Serial.print(analogRead(analogPin));
            readVal = analogRead(analogPin);
            Serial.println("");
            if (readVal < 400 && readVal > 200 || readVal > 680 && readVal < 800)

            { jc = jc + 1;

              //Serial.print(digitalRead(46));
              /*Serial.print(digitalRead(47));
                Serial.print(digitalRead(48));
                Serial.print(digitalRead(49));
                Serial.print(digitalRead(50));
                Serial.print(digitalRead(51));
                Serial.print(digitalRead(52));
                Serial.print(digitalRead(53));*/

              Serial.println("Junction Count!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
              //digitalWrite(20,LOW);

              Serial.println(jc);
              delay(500);
              if (jc == 2){delay(500); 
                Serial.println("ballllllllllllllll");
                while(digitalRead(IR)==HIGH){Serial.println("111111111111");
                
                analogWrite(frontRightpwm, 0);
            analogWrite(frontLeftpwm, 0);
              analogWrite(backLeftpwm, 0);  
             analogWrite(backRightpwm, 0); delay(1000);
             
             #ifdef OUTPUT_READABLE_YAWPITCHROLL
                      // display Euler angles in degrees
                      mpu.dmpGetQuaternion(&q, fifoBuffer);
                      mpu.dmpGetGravity(&gravity, &q);
                      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                      Serial.print("ypr\t");
                      Serial.print(ypr[0] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.print(ypr[1] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.println(ypr[2] * 180 / M_PI);

                      abc = ypr[0] * 180 / M_PI;

#endif
                      diff = a1 - abc;

                while (abs(diff) > 0.4)
                {     
                                  // if programming failed, don't try to do anything
                    Serial.println("Start");
                    if (!dmpReady) return;

                    // wait for MPU interrupt or extra packet(s) available
                    while (!mpuInterrupt && fifoCount < packetSize) {
                      // other program behavior stuff here
                      // .
                      // .
                      // .
                      // if you are really paranoid you can frequently test in between other
                      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
                      // while() loop to immediately process the MPU data
                      // .
                      // .
                      // .
                    }

                    // reset interrupt flag and get INT_STATUS byte
                    mpuInterrupt = false;
                    mpuIntStatus = mpu.getIntStatus();

                    // get current FIFO count
                    fifoCount = mpu.getFIFOCount();

                    // check for overflow (this should never happen unless our code is too inefficient)
                    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                      // reset so we can continue cleanly
                      mpu.resetFIFO();
                      Serial.println(F("FIFO overflow!"));

                      // otherwise, check for DMP data ready interrupt (this should happen frequently)
                    } else if (mpuIntStatus & 0x02) {
                      // wait for correct available data length, should be a VERY short wait
                      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

                      // read a packet from FIFO
                      mpu.getFIFOBytes(fifoBuffer, packetSize);

                      // track FIFO count here in case there is > 1 packet available
                      // (this lets us immediately read more without waiting for an interrupt)
                      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
                      // display Euler angles in degrees
                      mpu.dmpGetQuaternion(&q, fifoBuffer);
                      mpu.dmpGetGravity(&gravity, &q);
                      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                      Serial.print("ypr\t");
                      Serial.print(ypr[0] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.print(ypr[1] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.println(ypr[2] * 180 / M_PI);

                      //abc = ypr[0] * 180 / M_PI;

#endif
                  
                      Correction(diff);
                      abc = ypr[0] * 180 / M_PI;
                      diff = a1-abc;
                      Serial.println("Diffvalue");
                      Serial.println(diff);

                    }
                    }
             
             
     
             
             
             }
     while(b<50){ b++;
      delay(100);
      Serial.println(b);
      Serial.println("There");
                
                
                }
                
  Backward(50,50,50,50);
  analogWrite(frontRightpwm, 0);
  analogWrite(frontLeftpwm, 0);
  analogWrite(backLeftpwm, 0);
  analogWrite(backRightpwm, 0);
  delay(200);
  }
              if (jc == 3)
              {
                jc = jc + 1;
                digitalWrite(frontRight1, LOW);

                digitalWrite(frontLeft1, HIGH);

                digitalWrite(backLeft1, LOW);

                digitalWrite(backRight1, HIGH);

                analogWrite(frontRightpwm, 0);
                analogWrite(frontLeftpwm, 0);
                analogWrite(backLeftpwm, 0);
                analogWrite(backRightpwm, 0);

                delay(1000);
                Serial.print("\nDown PWM:&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&77");
                return;

              }
            }

            else if (abs(diff) <= 0.2) {
              qq = 0;
              Serial.println(qq);
            }
            else if (abs(diff) > 0.2 && abs(diff) <= 0.8) {
              qq = 1;
              Serial.println(qq);
            }
            else if (abs(diff) > 0.8 && abs(diff) <= 1.2) {
              qq = 2;
              Serial.println(qq);
            }
            else if (abs(diff) > 1.2 && abs(diff) <= 1.6) {
              qq = 3;
              Serial.println(qq);
            }
            else if (abs(diff) > 1.6 && abs(diff) <= 2) {
              qq = 4;
              Serial.println(qq);
            }
            else if (abs(diff) > 2 && abs(diff) <= 2.4) {
              qq = 5;
              Serial.println(qq);
            }
            else if (abs(diff) > 2.4 && abs(diff) <= 2.8) {
              qq = 6;
              Serial.println(qq);
            }
            else if (abs(diff) > 2.8 && abs(diff) <= 3.2) {
              qq = 7;
              Serial.println(qq);
            }
            else if (abs(diff) > 3.2 && abs(diff) <= 3.8) {
              qq = 8;
              Serial.println(qq);
            }
            else if (abs(diff) > 3.8 && abs(diff) <= 4.6) {
              qq = 9;
              Serial.println(qq);
            }
            else if (abs(diff) > 4.6 && abs(diff) <= 5.6) {
              qq = 10;
              Serial.println(qq);
            }
            else if (abs(diff) > 5.6 && abs(diff) <= 7) {
              qq = 11;
              Serial.println(qq);
            }
            else if (abs(diff) > 7) {
              qq = 12;
              Serial.println(qq);
            }

            switch (qq) {
              case 0:
                Serial.print("case 0:");
                pwm = weights[0];
                Forward1(high_speed + pwm, high_speed - pwm, high_speed + pwm, high_speed - pwm);
                Serial.println("FORARD12");
                break;
              case 1:
                Serial.print("case 1:");
                pwm = weights[1];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;

              case 2:
                Serial.print("case 2:");
                pwm = weights[2];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;

              case 3:
                Serial.print("case 3:");
                pwm = weights[3];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;
              case 4:
                Serial.print("case 4:");
                pwm = weights[4];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;
              case 5:
                Serial.print("case 5:");
                pwm = weights[5];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;

              case 6:
                Serial.print("case 6:");
                pwm = weights[6];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;

              case 7:
                Serial.print("case 7:");
                pwm = weights[7];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }

                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;
              case 8:
                Serial.print("case 8:");
                pwm = weights[8];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;
              case 9:
                Serial.print("case 9:");
                pwm = weights[9];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;
              case 10:
                Serial.print("case 10:");
                pwm = weights[10];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;
              case 11:
                Serial.print("case 11:");
                pwm = weights[11];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;
              case 12:
                Serial.print("case 12:");
                pwm = weights[12];
                if (diff < 0)
                {
                  Left1(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
                  Serial.println("Left");
                }
                if (diff > 0)
                {
                  Right1(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
                  Serial.println("Right");
                }
                break;
              default:
                break;
            }
          }

        }
      }
    }
  }
}
/*

  else
  {Serial.println("Not There");       // delay in between reads for stability
  }
  }
  }*/



/*int IRRead(int n)
  {
  int i = 0;
  int t = 0, nt = 0;
  while (i < n)
  {
    int buttonState = digitalRead(IR);
    // print out the state of the button:
    Serial.println(buttonState);
    if (buttonState == 0)
    { t++; i++;
      delay(100);
    }
    else
    { nt++; i++;
      delay(100);
    }
  }
  return (t);
  }*/


void Right(int FRPWM, int FLPWM, int BRPWM, int BLPWM) {
  digitalWrite(frontRight1, LOW);

  digitalWrite(frontLeft1, HIGH);

  digitalWrite(backLeft1, LOW);

  digitalWrite(backRight1, HIGH);

  analogWrite(frontRightpwm, FRPWM);
  analogWrite(frontLeftpwm, FLPWM);
  analogWrite(backLeftpwm, BLPWM);
  analogWrite(backRightpwm, BRPWM);
  Serial.println(FRPWM);
  Serial.println(FLPWM);
  Serial.println(BRPWM);
  Serial.println(BLPWM);
  Serial.print("\nRight PWM:");
}
void Left(int FRPWM, int FLPWM, int BRPWM, int BLPWM) {
  digitalWrite(frontRight1, LOW);
  digitalWrite(frontLeft1, HIGH);
  digitalWrite(backLeft1, LOW);

  digitalWrite(backRight1, HIGH);

  analogWrite(frontRightpwm, FRPWM);
  analogWrite(frontLeftpwm, FLPWM);
  analogWrite(backLeftpwm, BLPWM);
  analogWrite(backRightpwm, BRPWM);
  Serial.println(FRPWM);
  Serial.println(FLPWM);
  Serial.println(BRPWM);
  Serial.println(BLPWM);
  Serial.print("\nLeft PWM:");
}
void Forward(int FRPWM, int FLPWM, int BRPWM, int BLPWM) {
  digitalWrite(frontRight1, LOW);

  digitalWrite(frontLeft1, HIGH);

  digitalWrite(backLeft1, LOW);

  digitalWrite(backRight1, HIGH);


  analogWrite(frontRightpwm, FRPWM);
  analogWrite(frontLeftpwm, FLPWM);
  analogWrite(backLeftpwm, BLPWM);
  analogWrite(backRightpwm, BRPWM);
  Serial.println(FRPWM);
  Serial.println(FLPWM);
  Serial.println(BRPWM);
  Serial.println(BLPWM);
  Serial.print("\nUp pwm:");

}



void  Backward(int FRPWM, int FLPWM, int BRPWM, int BLPWM) {
  digitalWrite(frontRight1, HIGH);
  digitalWrite(frontLeft1, LOW);
  digitalWrite(backLeft1, LOW);
  digitalWrite(backRight1, LOW);


  analogWrite(frontRightpwm, FRPWM);
  analogWrite(frontLeftpwm, FLPWM);
  analogWrite(backLeftpwm, BLPWM);
  analogWrite(backRightpwm, BRPWM);
  Serial.println(FRPWM);
  Serial.println(FLPWM);
  Serial.println(BRPWM);
  Serial.println(BLPWM);
  Serial.print("\nGOING BACK******:");
  delay(800);
  return;
}

void Right1(int FRPWM, int FLPWM, int BRPWM, int BLPWM) {
  digitalWrite(frontRight1, LOW);
  digitalWrite(frontLeft1, LOW);
  digitalWrite(backLeft1, LOW);
  digitalWrite(backRight1, LOW);
  analogWrite(frontRightpwm, FRPWM);
  analogWrite(frontLeftpwm, FLPWM);
  analogWrite(backLeftpwm, BLPWM);
  analogWrite(backRightpwm, BRPWM);
  Serial.println(FRPWM);
  Serial.println(FLPWM);
  Serial.println(BRPWM);
  Serial.println(BLPWM);
  Serial.print("\nRight PWM:");
}
void Left1(int FRPWM, int FLPWM, int BRPWM, int BLPWM) {
  digitalWrite(frontRight1, LOW);
  digitalWrite(frontLeft1, LOW);
  digitalWrite(backLeft1, LOW);
  digitalWrite(backRight1, LOW);
  analogWrite(frontRightpwm, FRPWM);
  analogWrite(frontLeftpwm, FLPWM);
  analogWrite(backLeftpwm, BLPWM);
  analogWrite(backRightpwm, BRPWM);
  Serial.println(FRPWM);
  Serial.println(FLPWM);
  Serial.println(BRPWM);
  Serial.println(BLPWM);
  Serial.print("\nLeft PWM:");
}

void Forward1(int FRPWM, int FLPWM, int BRPWM, int BLPWM) {
  digitalWrite(frontRight1, LOW);
  digitalWrite(frontLeft1, LOW);
  digitalWrite(backLeft1, LOW);
  digitalWrite(backRight1, LOW);

  analogWrite(frontRightpwm, FRPWM);
  analogWrite(frontLeftpwm, FLPWM);
  analogWrite(backLeftpwm, BLPWM );
  analogWrite(backRightpwm, BRPWM);
  Serial.println(FRPWM);
  Serial.println(FLPWM);
  Serial.println(BRPWM);
  Serial.println(BLPWM);
  Serial.print("\nUp pwm:");

}



void Throw()
{

  if (tz==1)
  {
    d=250;
  }
  
  if (tz==2)
  {
    d=370;                                                                                                                                    0;
  }
  
  if (tz==3)
  {
    d=180;
  }
  
  digitalWrite(frontRight1, LOW);
  digitalWrite(frontLeft1, HIGH);
  digitalWrite(backLeft1, LOW);
  digitalWrite(backRight1, HIGH);
  analogWrite(frontRightpwm, 0);
  analogWrite(frontLeftpwm, 0);
  analogWrite(backLeftpwm, 0);
  analogWrite(backRightpwm, 0);
  Serial.print("\nGOING TO THrow check PWM:");
  digitalWrite(20, HIGH);
  delay(500);
  int b = 0;
while (true)
                  {

                    // if programming failed, don't try to do anything
                    Serial.println("Start");
                    if (!dmpReady) return;

                    // wait for MPU interrupt or extra packet(s) available
                    while (!mpuInterrupt && fifoCount < packetSize) {
                      // other program behavior stuff here
                      // .
                      // .
                      // .
                      // if you are really paranoid you can frequently test in between other
                      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
                      // while() loop to immediately process the MPU data
                      // .
                      // .
                      // .
                    }

                    // reset interrupt flag and get INT_STATUS byte
                    mpuInterrupt = false;
                    mpuIntStatus = mpu.getIntStatus();

                    // get current FIFO count
                    fifoCount = mpu.getFIFOCount();

                    // check for overflow (this should never happen unless our code is too inefficient)
                    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                      // reset so we can continue cleanly
                      mpu.resetFIFO();
                      Serial.println(F("FIFO overflow!"));

                      // otherwise, check for DMP data ready interrupt (this should happen frequently)
                    } else if (mpuIntStatus & 0x02) {
                      // wait for correct available data length, should be a VERY short wait
                      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

                      // read a packet from FIFO
                      mpu.getFIFOBytes(fifoBuffer, packetSize);

                      // track FIFO count here in case there is > 1 packet available
                      // (this lets us immediately read more without waiting for an interrupt)
                      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
                      // display Euler angles in degrees
                      mpu.dmpGetQuaternion(&q, fifoBuffer);
                      mpu.dmpGetGravity(&gravity, &q);
                      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                      Serial.print("ypr\t");
                      Serial.print(ypr[0] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.print(ypr[1] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.println(ypr[2] * 180 / M_PI);

                      abc = ypr[0] * 180 / M_PI;

#endif
                      diff = a1 - abc;

                while (abs(diff) > 0.9)
                {     
                                  // if programming failed, don't try to do anything
                    Serial.println("Start");
                    if (!dmpReady) return;

                    // wait for MPU interrupt or extra packet(s) available
                    while (!mpuInterrupt && fifoCount < packetSize) {
                      // other program behavior stuff here
                      // .
                      // .
                      // .
                      // if you are really paranoid you can frequently test in between other
                      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
                      // while() loop to immediately process the MPU data
                      // .
                      // .
                      // .
                    }

                    // reset interrupt flag and get INT_STATUS byte
                    mpuInterrupt = false;
                    mpuIntStatus = mpu.getIntStatus();

                    // get current FIFO count
                    fifoCount = mpu.getFIFOCount();

                    // check for overflow (this should never happen unless our code is too inefficient)
                    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                      // reset so we can continue cleanly
                      mpu.resetFIFO();
                      Serial.println(F("FIFO overflow!"));

                      // otherwise, check for DMP data ready interrupt (this should happen frequently)
                    } else if (mpuIntStatus & 0x02) {
                      // wait for correct available data length, should be a VERY short wait
                      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

                      // read a packet from FIFO
                      mpu.getFIFOBytes(fifoBuffer, packetSize);

                      // track FIFO count here in case there is > 1 packet available
                      // (this lets us immediately read more without waiting for an interrupt)
                      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
                      // display Euler angles in degrees
                      mpu.dmpGetQuaternion(&q, fifoBuffer);
                      mpu.dmpGetGravity(&gravity, &q);
                      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                      Serial.print("ypr\t");
                      Serial.print(ypr[0] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.print(ypr[1] * 180 / M_PI);
                      Serial.print("\t");
                      Serial.println(ypr[2] * 180 / M_PI);

                      //abc = ypr[0] * 180 / M_PI;

#endif
                  
                      Correction(diff);
                      abc = ypr[0] * 180 / M_PI;
                      diff = a1-abc;
                      Serial.println("Diffvalue");
                      Serial.println(diff);

                    }
                    }
                    break;

                }
               // break;
              }
               // Stop();

              
            



  analogWrite(frontRightpwm, 0);
  analogWrite(frontLeftpwm, 0);
  analogWrite(backLeftpwm, 0);
  analogWrite(backRightpwm, 0);
  delay(100);
  
  while (true)
  {

    b = 0;
    while (digitalRead(IR) == LOW)


    {
      b++;
      delay(100);
      Serial.println(b);
      Serial.println("READY TO THROW!!!!!!!!");
      Serial.println("JCCCCCCC");
      Serial.println(jc);
      while (b > 20) {

        digitalWrite(10, HIGH);
        delay(d);
        digitalWrite(10, LOW);
        Serial.println("!!!!!!!THROWN!!!!!!!!");
        delay(3000);
        analogWrite(10,90);
        delay(6);
        analogWrite(10,0);
        jc = jc + 1;

        
        Serial.println("DKJCCCC");
        Serial.println(jc);
       // high_speed=40;
        return;

      }
    }

    while (digitalRead(IR) == HIGH)
    {
      delay(100);
      Serial.println(b);
      Serial.println("Cancled!!!");
    }
  }
}





void Stop()
{

  digitalWrite(frontRight1, LOW);
  digitalWrite(frontLeft1, HIGH);
  digitalWrite(backLeft1, LOW);
  digitalWrite(backRight1, HIGH);
  analogWrite(frontRightpwm, 0);
  analogWrite(frontLeftpwm, 0);
  analogWrite(backLeftpwm, 0);
  analogWrite(backRightpwm, 0);
  Serial.print("\nSTOPPED");
  delay(1000);
}
void Front()
{


  digitalWrite(frontRight1, LOW);
  digitalWrite(frontLeft1, HIGH);
  digitalWrite(backLeft1, LOW);
  digitalWrite(backRight1, HIGH);
  analogWrite(frontRightpwm, 0);
  analogWrite(frontLeftpwm, 0);
  analogWrite(backLeftpwm, 0);
  analogWrite(backRightpwm, 0);
  Serial.print("\nDown PWM:");
  digitalWrite(20, HIGH);
  delay(1000);

  while (true)
  {

  if (jc >=7 ) 
          {Serial.println("7777777777777777777777777777");
          if(digitalRead(53)==LOW)
             {jc=8;
             Serial.println("88888888888888888888888888");
             return;}
            
          }
      
    // if programming failed, don't try to do anything
    Serial.println("Start");
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
      stable--;
      if (stable == 2)
      {
        //a1 = ypr[0] * 180 / M_PI;
        Serial.print(" A1  ");
        Serial.println(a1);
      }
#endif

      if (stable < 0) {
        Serial.println("Working");
        a = ypr[0] * 180 / M_PI;
        Serial.print(a);
        diff = a1 - a;
        Serial.println(diff);
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        // blink LED to indicate activity
        blinkState = !blinkState;
        high_speed = 80;
        Serial.print("READVALLLLLLLLLLLL");
        Serial.print(readVal);
        readVal = analogRead(analogPin);
        if (readVal < 400 && readVal > 200 || readVal > 680 && readVal < 800)

        { jc = jc + 1;

          Serial.print(digitalRead(46));
          Serial.print(digitalRead(47));
          Serial.print(digitalRead(48));
          Serial.print(digitalRead(49));
          Serial.print(digitalRead(50));
          Serial.print(digitalRead(51));
          Serial.print(digitalRead(52));
          Serial.print(digitalRead(53));

          Serial.println("Junction Count!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
          //digitalWrite(20,LOW);

          Serial.println(jc);
          delay(100);
          
        }

        if (jc >=7 ) 
          {Serial.println("7777777777777777777777777777");
          if(digitalRead(53)==LOW)
             {jc=8;
             Serial.println("88888888888888888888888888");
             return;}
            
          }

        else if (abs(diff) <= 0.2) {
          qq = 0;
          Serial.println(qq);
        }
        else if (abs(diff) > 0.2 && abs(diff) <= 0.8) {
          qq = 1;
          Serial.println(qq);
        }
        else if (abs(diff) > 0.8 && abs(diff) <= 1.2) {
          qq = 2;
          Serial.println(qq);
        }
        else if (abs(diff) > 1.2 && abs(diff) <= 1.6) {
          qq = 3;
          Serial.println(qq);
        }
        else if (abs(diff) > 1.6 && abs(diff) <= 2) {
          qq = 4;
          Serial.println(qq);
        }
        else if (abs(diff) > 2 && abs(diff) <= 2.4) {
          qq = 5;
          Serial.println(qq);
        }
        else if (abs(diff) > 2.4 && abs(diff) <= 2.8) {
          qq = 6;
          Serial.println(qq);
        }
        else if (abs(diff) > 2.8 && abs(diff) <= 3.2) {
          qq = 7;
          Serial.println(qq);
        }
        else if (abs(diff) > 3.2 && abs(diff) <= 3.8) {
          qq = 8;
          Serial.println(qq);
        }
        else if (abs(diff) > 3.8 && abs(diff) <= 4.6) {
          qq = 9;
          Serial.println(qq);
        }
        else if (abs(diff) > 4.6 && abs(diff) <= 5.6) {
          qq = 10;
          Serial.println(qq);
        }
        else if (abs(diff) > 5.6 && abs(diff) <= 7) {
          qq = 11;
          Serial.println(qq);
        }
        else if (abs(diff) > 7) {
          qq = 12;
          Serial.println(qq);
        }

        switch (qq) {
          case 0:
            Serial.print("case 0:");
            pwm = weights[0];
            Forward2(high_speed, high_speed , high_speed, high_speed);
            Serial.println("FORARD2222222222222");
            Serial.println(jc);
            break;
          case 1:
            Serial.print("case 1:");
            pwm = weights[1];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;

          case 2:
            Serial.print("case 2:");
            pwm = weights[2];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;

          case 3:
            Serial.print("case 3:");
            pwm = weights[3];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;
          case 4:
            Serial.print("case 4:");
            pwm = weights[4];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;
          case 5:
            Serial.print("case 5:");
            pwm = weights[5];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;

          case 6:
            Serial.print("case 6:");
            pwm = weights[6];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;

          case 7:
            Serial.print("case 7:");
            pwm = weights[7];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;
          case 8:
            Serial.print("case 8:");
            pwm = weights[8];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;
          case 9:
            Serial.print("case 9:");
            pwm = weights[9];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;
          case 10:
            Serial.print("case 10:");
            pwm = weights[10];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;
          case 11:
            Serial.print("case 11:");
            pwm = weights[11];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;
          case 12:
            Serial.print("case 12:");
            pwm = weights[12];
            if (diff < 0)
            {
              Left2(high_speed + pwm, high_speed + pwm, high_speed - pwm, high_speed - pwm);
              Serial.println("Left");
            }
            if (diff > 0)
            {
              Right2(high_speed - pwm, high_speed - pwm, high_speed + pwm, high_speed + pwm);
              Serial.println("Right");
            }
            break;
          default:
            break;
        }
      }
    }
  }
}



void Right2(int BRPWM, int BLPWM, int FRPWM, int FLPWM) {
  digitalWrite(frontRight1, HIGH);
  digitalWrite(frontLeft1, HIGH);
  digitalWrite(backLeft1, HIGH);
  digitalWrite(backRight1, HIGH);
  analogWrite(frontRightpwm, FRPWM);
  analogWrite(frontLeftpwm, FLPWM);
  analogWrite(backLeftpwm, BLPWM);
  analogWrite(backRightpwm, BRPWM);
  Serial.println(FRPWM);
  Serial.println(FLPWM);
  Serial.println(BRPWM);
  Serial.println(BLPWM);
  Serial.print("\nRight222 PWM:");
}
void Left2(int BRPWM, int BLPWM, int FRPWM, int FLPWM) {
  digitalWrite(frontRight1, HIGH);
  digitalWrite(frontLeft1, HIGH);
  digitalWrite(backLeft1, HIGH);
  digitalWrite(backRight1, HIGH);
  analogWrite(frontRightpwm, FRPWM);
  analogWrite(frontLeftpwm, FLPWM);
  analogWrite(backLeftpwm, BLPWM);
  analogWrite(backRightpwm, BRPWM);
  Serial.println(FRPWM);
  Serial.println(FLPWM);
  Serial.println(BRPWM);
  Serial.println(BLPWM);
  Serial.print("\nLeft2222 PWM:");
}
/*void SlideRight(int FLPWM,int BRPWM){
  digitalWrite(frontLeft1,LOW);
  digitalWrite(frontLeft2,HIGH);
  digitalWrite(frontLeften,LOW);
  analogWrite(frontLeftpwm,FLPWM);
  digitalWrite(backRight1,HIGH);
  digitalWrite(backRight2,LOW);
  digitalWrite(backRighten,LOW);
  analogWrite(backRightpwm,BRPWM);
  }
  void SlideLeft(int FLPWM,int BRPWM){
  digitalWrite(frontLeft1,HIGH);
  digitalWrite(frontLeft2,LOW);
  digitalWrite(frontLeften,LOW);
  analogWrite(frontLeftpwm,FLPWM);
  digitalWrite(backRight1,LOW);
  digitalWrite(backRight2,HIGH);
  digitalWrite(backRighten,LOW);
  analogWrite(backRightpwm,BRPWM);
  }*/
void Forward2(int BRPWM, int BLPWM, int FRPWM, int FLPWM) {
  digitalWrite(frontRight1, HIGH);
  digitalWrite(frontLeft1, HIGH);
  digitalWrite(backLeft1, HIGH);
  digitalWrite(backRight1, HIGH);

  analogWrite(frontRightpwm, FRPWM);
  analogWrite(frontLeftpwm, FLPWM);
  analogWrite(backLeftpwm, BLPWM);
  analogWrite(backRightpwm, BRPWM);
  Serial.println(FRPWM);
  Serial.println(FLPWM);
  Serial.println(BRPWM);
  Serial.println(BLPWM);
  Serial.print("\nUp pwm222222222:");

}






void Correction(float rt) {


  //abc=ypr[0] * 180 / M_PI;
  if (rt > 0) {
    digitalWrite(frontRight1, HIGH);
    digitalWrite(frontLeft1, HIGH);
    digitalWrite(backLeft1, LOW);
    digitalWrite(backRight1, LOW);

    /* analogWrite(frontRightpwm,FRPWM);
      analogWrite(frontLeftpwm,FLPWM);
      analogWrite(backLeftpwm,BLPWM);
      analogWrite(backRightpwm,BRPWM);*/
    analogWrite(frontRightpwm, 40);
    analogWrite(frontLeftpwm, 40);
    analogWrite(backLeftpwm, 40);
    analogWrite(backRightpwm,40);
    Serial.println("HUVSDuihdbuiva");
    return;

  }
  else {
    digitalWrite(frontRight1, LOW);

    digitalWrite(frontLeft1, LOW);

    digitalWrite(backLeft1, HIGH);

    digitalWrite(backRight1, HIGH);
    analogWrite(frontRightpwm, 40);
    analogWrite(frontLeftpwm, 40);
    analogWrite(backLeftpwm, 40);
    analogWrite(backRightpwm, 40);
    Serial.println("tahrfbetgrjtra");
    return;  
  }

  //return (abc);
}

auto viggy.txt
Displaying auto viggy.txt.
