#include <AFMotor.h>
#include <QTRSensors.h>
#include "MPU6050_DMP6.h"
#define OUTPUT_READABLE_YAWPITCHROLL



AF_DCMotor motor1(4);  // Motor 1 -> Sol ön
AF_DCMotor motor2(3);  // Motor 2 -> Sağ ön
AF_DCMotor motor3(1);  // Motor 3 -> Sol arka
AF_DCMotor motor4(2);  // Motor 4 -> Sağ arka


typedef struct{
  byte rightPWM;
  byte leftPWM;
  int position;
  byte sensorArray[8];
}Message;

Message msg;


boolean rightForward = true;
boolean leftForward = true;

int rightMotorSpeed;
int leftMotorSpeed;

QTRSensors qtr;

const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];
int QTR_Sensor_Array[SensorCount];
unsigned int position;

unsigned long timer = 0;
int sampleTime = 500;


byte BASE_SPEED = 150;

int TCRT5000 =53;    //IR senoru sag
int TCRT5000_1 = 51;  // IR sensoru sol

boolean solIr = false;
boolean sagIr = false;


void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);

  MPU_init();

 

  qtr.setTypeRC();
  //qtr.setSensorPins((const uint8_t[]) {30, 32, 34, 36, 38, 40, 42, 44}, SensorCount);
  qtr.setSensorPins((const uint8_t[]) {44, 42, 40, 38, 36, 34, 32, 30}, SensorCount);

  qtr_calibrate();
  delay(2000);
  
}
void loop() {

  int Sag_IR = digitalRead(TCRT5000);
  int Sol_IR = digitalRead(TCRT5000_1);

  if (Sag_IR == 1 && Sol_IR == 0 ){
    sagIr = true;

  }
  if(Sag_IR == 0 && Sol_IR == 1){
    solIr = true;
  }


  Serial.print("Sag = "); 
  Serial.println(String(Sag_IR));
  Serial.print("Sol = "); 
  Serial.println(String(Sol_IR));
  //delay(500); 5ms gecikmeli

      if (!dmpReady) return;  // Eğer programlama başarısız olduysa hiçbir şey yapma.

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Son gelen veriyi oku
      
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // EULER hesaplamalarına göre dereceleri hesapla
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


            float yawAngle = ypr[0] * 180 / M_PI; // Yaw açısını derece cinsinden al

            Serial.print("IMU\t");
            Serial.println(yawAngle);
            

            // eğer sağ ır 1 ıse
            if(Sag_IR == 0 && Sol_IR == 0){
              position = qtr.readLineBlack(sensorValues); // Siyah çizgiyi takip et

            } else if (sagIr){


             int yawDeger = yawAngle + 90 ;

             

              if(yawAngle <= yawDeger){

              leftMotorSpeed = 250;
              rightMotorSpeed = 250;        //sag
              leftForward = true;
              rightForward = false;

              Serial.println("******************** Sag IR 1 degerinde ********************");

              if(yawAngle == yawDeger){
                sagIr = false;
              }
              }/*else if(yawDeger == yawAngle){
                sagIr =false;
              }*/
 
            }else if(solIr){
             int yawDeger = yawAngle - 90 ;
              
              if(yawDeger <= yawAngle){

              leftMotorSpeed = 250;
              rightMotorSpeed = 250;     //sol
              rightForward = true;
              leftForward = false;

              Serial.println("******************** Sol IR 1 degerinde ********************");
              if(yawAngle == yawDeger){
                solIr = false;
              }
              }/*else if(yawDeger == yawAngle){
                solIr = false;
              }*/


            }

            /*
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif

        
    }

  qtr.read(sensorValues); // Sensör değerlerini oku

  position = qtr.readLineBlack(sensorValues); // Siyah çizgiyi takip et
  //setQTRSensorArray();

  //msg.position = position;
  //Serial.println("Konum: " + String(position));
  
  for (int i = 0; i < 8; i++) {
    //Serial.print(sensorValues[i]);
    //Serial.print('\t');
  }

  if(position <= 2500){
    if(position > 1750){
      leftMotorSpeed = 120;
      rightMotorSpeed = 70;
      rightForward = true;
      leftForward = true;
    }
    else if(position > 1000){
      leftMotorSpeed = 80;
      rightMotorSpeed = 80;        //sag
      leftForward = true;
      rightForward = false;
    }
  }
  else if(position <= 4000){
    leftMotorSpeed = 120;
    rightMotorSpeed = 120;
    rightForward = true;
    leftForward = true;
  }
  else if(position <= 5100){
    if(position > 4500){
      leftMotorSpeed = 80;
      rightMotorSpeed = 80;     //sol
      rightForward = true;
      leftForward = false;
    }
    else if(position > 4000){
      leftMotorSpeed = 70;
      rightMotorSpeed = 120;
      leftForward = true;
      rightForward = true;
    }
  }
  //Serial1.println(position);

  //msg.leftPWM = leftMotorSpeed;
  //msg.rightPWM = rightMotorSpeed;

  motor1.setSpeed(leftMotorSpeed);
  motor2.setSpeed(rightMotorSpeed);
  motor3.setSpeed(leftMotorSpeed);
  motor4.setSpeed(rightMotorSpeed);

  
  //LoRa_Send(msg);

  /*
  Serial.println("\nSağ motor pwm : " + String(rightMotorSpeed));
  Serial.println("Sol motor pwm : " + String(leftMotorSpeed));
  Serial.println("\n ******************** \n");
  */

  //Serial1.write(msg.leftPWM);
  //Serial1.write(msg.rightPWM);

  //for (int i = 0; i < 8; i++) {
  //  Serial1.write(msg.sensorArray[i]);
  //}

  if(rightForward){
    motor2.run(FORWARD);
    motor4.run(FORWARD);
  }
  else{
    motor2.run(BACKWARD);
    motor4.run(BACKWARD);
  }

  if(leftForward){
    motor1.run(FORWARD);
    motor3.run(FORWARD);
  }
  else{
    motor1.run(BACKWARD);
    motor3.run(BACKWARD);
  }

  if((millis() - timer) > sampleTime){
    AGV_Telemetry_UART_Send();
    timer = millis();
  }
}


void qtr_calibrate() {
  Serial.println("QTR-8RC Kalibrasyon Başladı.");

  for (byte i = 0; i < 250; i++) {
    qtr.calibrate();
   
    delay(10);
  }
  //bekle(); // motorları durdur
  //Serial.println("QTR-8RC Kalibrasyon Bitti.");

  Serial.println("\nQTR-8RC Calibration Min Values;");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  //Serial.println("QTR-8RC Calibration Max Values;");
  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}


void bekle() {
  // Motorları durdurur.

  motor1.setSpeed(BASE_SPEED);
  motor2.setSpeed(BASE_SPEED);
  motor3.setSpeed(BASE_SPEED);
  motor4.setSpeed(BASE_SPEED);
 
  Serial.println("Dur");
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}


void setQTRSensorArray(){
  for(int i = 0; i < 8; i++){
    if(sensorValues[i] > 700)
      QTR_Sensor_Array[i] = 1;
    else
      QTR_Sensor_Array[i] = 0;
  }

  //msg.sensorArray[8] = QTR_Sensor_Array[8];

  Serial.println("Backend'e gonderilecek QTR Sensor Dizisi;");
  for(int i = 0; i < 8; i++){
    Serial.print(String(QTR_Sensor_Array[i]) + "\t");
  }
  Serial.println();
}



void AGV_Telemetry_UART_Send(){
  String msg = "";
  msg += String(position) + "," + String(leftMotorSpeed) + "," + String(rightMotorSpeed) + ",";

  for(int i = 0; i < 8; i++){
    if(sensorValues[i] > 700){
      msg += "1";
    }
    else{
      msg += "0";
    }
  }
  msg += ",";
  Serial1.println(msg);
  Serial.println(msg);
}