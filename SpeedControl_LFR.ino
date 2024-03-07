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

int rightMotorSpeed =150;
int leftMotorSpeed =150;
int speedDifference =130;

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

  hizAyari();

  


  Serial.print("Sag = "); 
  Serial.println(String(Sag_IR));
  Serial.print("Sol = "); 
  Serial.println(String(Sol_IR));
  //delay(500); 5ms gecikmeli


  if (!dmpReady) return;  // Eğer programlama başarısız olduysa hiçbir şey yapma.


  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
     // Son gelen veriyi oku   
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // EULER hesaplamalarına göre dereceleri hesapla
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      float yawAngle = ypr[0] * 360 / M_PI;  // Yaw açısını derece cinsinden al
      int yawDeger ;

      if (Sag_IR == 1 && Sol_IR == 0 ){
    sagIr = true;
     yawDeger = int(yawAngle) + 50;

  }
  if(Sag_IR == 0 && Sol_IR == 1){
    solIr = true;
     yawDeger = int(yawAngle) - 50;
  }

      

      Serial.print("IMU\t");
      Serial.println(yawAngle);

        if(leftMotorSpeed != 0 && rightMotorSpeed != 0 && speedDifference != 0 ){ 
          if(!sagIr && !solIr){
            if(position <= 2500){
              if(position > 1750){
                motor1.setSpeed(speedDifference);
                motor1.run(FORWARD);
                motor2.setSpeed(speedDifference);
                motor2.run(FORWARD);
                motor3.setSpeed(speedDifference);
                motor3.run(FORWARD);
                motor4.setSpeed(speedDifference);
                motor4.run(FORWARD);
              }
              else if(position > 1000){
                motor1.setSpeed(leftMotorSpeed);
                motor1.run(FORWARD);
                motor2.setSpeed(speedDifference);
                motor2.run(BACKWARD);
                motor3.setSpeed(leftMotorSpeed);
                motor3.run(FORWARD);
                motor4.setSpeed(speedDifference);
                motor4.run(BACKWARD);
                Serial.println("SAĞA DÖNÜYOR");
              }
            }
            else if(position <= 4000){
              motor1.setSpeed(leftMotorSpeed);
              motor1.run(FORWARD);
              motor2.setSpeed(rightMotorSpeed);
              motor2.run(FORWARD);
              motor3.setSpeed(leftMotorSpeed);
              motor3.run(FORWARD);
              motor4.setSpeed(rightMotorSpeed);
              motor4.run(FORWARD);
            }
            else if(position <= 5100){
              if(position > 4500){
                motor1.setSpeed(speedDifference);
                motor1.run(BACKWARD);
                motor2.setSpeed(rightMotorSpeed);
                motor2.run(FORWARD);
                motor3.setSpeed(speedDifference);
                motor3.run(BACKWARD);
                motor4.setSpeed(rightMotorSpeed);
                motor4.run(FORWARD);
                Serial.println("SOLA DÖNÜYOR");
              }
              else if(position > 4000){
                motor1.setSpeed(speedDifference);
                motor1.run(FORWARD);
                motor2.setSpeed(speedDifference);
                motor2.run(FORWARD);
                motor3.setSpeed(speedDifference);
                motor3.run(FORWARD);
                motor4.setSpeed(speedDifference);
                motor4.run(FORWARD);
              }
            }
         } else if(sagIr){
            while(yawAngle <= yawDeger){
              if (!dmpReady) return;  // Eğer programlama başarısız olduysa hiçbir şey yapma.
                if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                    // Son gelen veriyi oku                      
                  #ifdef OUTPUT_READABLE_YAWPITCHROLL
                  // EULER hesaplamalarına göre dereceleri hesapla
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetGravity(&gravity, &q);
                  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                  yawAngle = ypr[0] * 360 / M_PI; // Yaw açısını derece cinsinden al              
                  motor1.setSpeed(speedDifference);
                  motor1.run(FORWARD);
                  motor2.setSpeed(speedDifference);
                  motor2.run(BACKWARD);
                  motor3.setSpeed(speedDifference);
                  motor3.run(FORWARD);
                  motor4.setSpeed(speedDifference);
                  motor4.run(BACKWARD);
                  Serial.println("******************** Sag IR 1 degerinde ********************");

                  Serial.print("Yaw Deger = " );
                  Serial.println(yawDeger);
                  Serial.print("IMU\t");
                  Serial.println(yawAngle);
                  
                  #endif

                    }

              }
              
                sagIr = false;                           
            
          } else if(solIr){
              while(yawDeger <= yawAngle){

                if (!dmpReady) return;  // Eğer programlama başarısız olduysa hiçbir şey yapma.
                if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                    // Son gelen veriyi oku                      
                  #ifdef OUTPUT_READABLE_YAWPITCHROLL
                  // EULER hesaplamalarına göre dereceleri hesapla
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetGravity(&gravity, &q);
                  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                  yawAngle = ypr[0] * 360 / M_PI; // Yaw açısını derece cinsinden al 

                  motor1.setSpeed(speedDifference);
                  motor1.run(BACKWARD);
                  motor2.setSpeed(speedDifference);
                  motor2.run(FORWARD);
                  motor3.setSpeed(speedDifference);
                  motor3.run(BACKWARD);
                  motor4.setSpeed(speedDifference);
                  motor4.run(FORWARD);
                  
                  Serial.println("******************** Sol IR 1 degerinde ********************");
                  Serial.print("Yaw Deger = " );
                  Serial.println(yawDeger);
                  Serial.print("IMU\t");
                  Serial.println(yawAngle);

                  #endif

                }
              } 
             solIr=false;

            } 
      }
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

  
  //Serial1.println(position);

  //msg.leftPWM = leftMotorSpeed;
  //msg.rightPWM = rightMotorSpeed;

 
  
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
void hizAyari(){
  if (Serial1.available() > 0) {
    String value = Serial1.readString();
    Serial.println("value =  " + value);
    bekle();
    delay(200);
    if(value == "start") {
      rightMotorSpeed =  120;
      leftMotorSpeed =  120;
      speedDifference = 70; 
    }
    if(value =="stop" ){
      rightMotorSpeed =  0;
      leftMotorSpeed =  0;
      speedDifference = 0;
    }
    if(value =="1" ){
      rightMotorSpeed =  120;
      leftMotorSpeed =  120;
      speedDifference = 70;
    }
    if(value == "2" ){
      rightMotorSpeed =  150;
      leftMotorSpeed =  150;
      speedDifference = 100;
    }
    if(value =="3" ){
      rightMotorSpeed =  180;
      leftMotorSpeed =  180;
      speedDifference= 130;
    }

    Serial.println("right: " + String(rightMotorSpeed));
    Serial.println("left: " + String(leftMotorSpeed));
 }
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