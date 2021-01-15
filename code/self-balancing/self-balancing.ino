#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include <Wire.h>
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define enA 3
#define in1 4
#define in2 5
#define enB 9
#define in3 7
#define in4 8


//MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

//orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];
float pitch;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output;
double cumError, rateError;

//set value based on system
float kp = 1.5;
float ki = 0;
float kd = 0;
int setPoint = 0;

int computePID(double inp) {
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);

  error = setPoint - inp;
  cumError += error * elapsedTime;
  rateError = (error - lastError) / elapsedTime;

  double out = kp * error + ki * cumError + kd * rateError;
  lastError = error;
  previousTime = currentTime;

  return out;
}

double computeAngle() {
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();        // reset so we can continue cleanly
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();    // wait for correct available data length, should be a VERY short wait
    mpu.getFIFOBytes(fifoBuffer, packetSize);       // read a packet from FIFO
    fifoCount -= packetSize;          // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt


    #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // display pitch
        input = ypr[1] * 180 / M_PI;
    #endif
  }
  return input;
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEVBUILTIN_FASTWIRE
    Wire.begin();
    TWBR = 24;
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(9600);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections.."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection succesful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(43);
  mpu.setYGyroOffset(-21);
  mpu.setZGyroOffset(-395);
  mpu.setZAccelOffset(15112);

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
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

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

}

void loop() {
  input = computeAngle();
  Serial.print(F("angle: "));
  Serial.println(input);
  
  output = computePID(input); 
  if(output >= 1000){
    output = 1000;
  }
  else if (output <= -1000){
    output = -1000;
  }

  if (input > 0){
    forward();
//    Serial.print("input: ");
//    Serial.println(input);
//    Serial.print("output: ");
//    Serial.println(output);
  }

  else if (input < 0){
    backward();
//    Serial.println(input);
//    Serial.println(output);
  }

  
}

void forward(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  int val = map(output, 0, 1000,0 ,255);
  analogWrite(enA, val);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, val);
}

void backward(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  int val = map(output, 0, 1000,0 ,255);
  analogWrite(enA, val);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, val);
}
