  //Kadir Branch

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
    byte ir[2];
    byte sensorArray[8];
  }Message;

  Message msg;

  boolean rightForward = true;
  boolean leftForward = true;

  boolean leftIr = false;
  boolean rightIr = false;

  int rightMotorSpeed =  120;
  int leftMotorSpeed =  120;
  int speedDifference = 70;

  QTRSensors qtr;

  const uint8_t SensorCount = 8;
  unsigned int sensorValues[SensorCount];
  int QTR_Sensor_Array[SensorCount];
  unsigned int position;

  String UART_DATA = "";
  byte UART_LENGTH = 0;
  boolean UART_COMPLETE = false;

  unsigned long timer = 0;
  int sampleTime = 500;

  String value = "";
  int leftFrontIRSensorPin = 51;
  int rightFrontIRSensorPin = 53;

  void setup() {

    Serial1.begin(9600);
    Serial.begin(9600);

    MPU_init();

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]) {44, 42, 40, 38, 36, 34, 32, 30}, SensorCount);
    qtr_calibrate();
    delay(2000);
  }

  void loop() {
    UART_Control();

    qtr.read(sensorValues); // Sensör değerlerini oku
    position = qtr.readLineBlack(sensorValues);
    //Serial.println("Konum: " + String(position));

    int leftFrontIRValue = digitalRead(leftFrontIRSensorPin);
    int rightFrontIRValue = digitalRead(rightFrontIRSensorPin);
    
    hizAyari();

    if (!dmpReady) return;  // Eğer programlama başarısız olduysa hiçbir şey yapma.

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Son gelen veriyi oku
      
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // EULER hesaplamalarına göre dereceleri hesapla
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float yawAngle = ypr[0] * 360 / M_PI; // Yaw açısını derece cinsinden al
        int yawDeger ;    

        if (rightFrontIRValue == 1 && leftFrontIRValue == 0 ){
          rightIr = true;
          yawDeger = int(yawAngle) + 45;
        }
        if(rightFrontIRValue == 0 && leftFrontIRValue == 1){
          leftIr = true;
          yawDeger = int(yawAngle) - 45;
        }
        if(leftMotorSpeed != 0 && rightMotorSpeed != 0 && speedDifference != 0 ){ 
          if(!rightIr && !leftIr){
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
          }
        } else if(rightIr){
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
            rightIr = false;                           
          }else if(leftIr){
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
             leftIr=false;

        } 
      }
    #endif  
  

    if((millis() - timer) > sampleTime){
        AGV_Telemetry_UART_Send();
        timer = millis();
    }
  }


  void hizAyari(){
    bekle();
    delay(200);

    if(value == "4") {
      rightMotorSpeed =  120;
      leftMotorSpeed =  120;
      speedDifference = 70;
    }
    if(value == "0" ){
      bekle();
    }
    if(value == "1" ){
      rightMotorSpeed =  120;
      leftMotorSpeed =  120;
      speedDifference = 70;
    }
    if(value == "2" ){
      rightMotorSpeed =  150;
      leftMotorSpeed =  150;
      speedDifference = 100;
    }
    if(value == "3" ){
       rightMotorSpeed =  180;
      leftMotorSpeed =  180;
      speedDifference= 130;
    }
  }


  void qtr_calibrate() {
    Serial.println("QTR-8RC Kalibrasyon Başladı.");
    for (byte i = 0; i < 250; i++) {
      qtr.calibrate();
      delay(10);
    }
    Serial.println("QTR-8RC Kalibrasyon Bitti.");

    Serial.println("\nQTR-8RC Calibration Min Values;");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();

    Serial.println("QTR-8RC Calibration Max Values;");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
  }

  // Motorları durdurma fonksiyonu
  void bekle() {
    motor1.setSpeed(0);
    motor1.run(RELEASE);
    motor2.setSpeed(0);
    motor2.run(RELEASE);
    motor3.setSpeed(0);
    motor3.run(RELEASE);
    motor4.setSpeed(0);
    motor4.run(RELEASE);
  }

  void UART_Control(){
    //Serial Buffer = "ileri", Serial Buffer Size = 5, Serial.available() return -> 5.
    while(Serial1.available() > 0){
      char ch = Serial1.read();  //İlk okumada 'i', 'l', ..., her read() metodu çağrıldıktan sonra ilgili veri buffer'dan silinir ve buffer size 1 azaltılır.
      if(ch != '\n')
        UART_DATA += ch;  //"i", "il", "ile", "iler", "ileri"
      else{
        UART_COMPLETE = true;
        Serial.println("Gelen Veri: " + UART_DATA);
        break;
      }
    }

    if(UART_COMPLETE == true){
      
      if(UART_DATA[0] == 'd'){
        value = "0";
      }
      else if(UART_DATA[0] == 'x'){
        value = "1";
      }
      else if(UART_DATA[0] == 'y'){
        value = "2";
      }
      else if(UART_DATA[0] == 'z'){
        value = "3";
      }
      else if(UART_DATA[0] == 'b'){
        value = "4";
      }
      else{
        Serial.println("Gecersiz Komut");
      }

      UART_COMPLETE = false;
      UART_DATA = "";
    }
  }

  void AGV_Telemetry_UART_Send(){
    String msg = "";
    msg += String(position) + "," + String(leftMotorSpeed) + "," + String(rightMotorSpeed) + "," + String(rightFrontIRSensorPin) + String(leftFrontIRSensorPin) + ","  ;

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