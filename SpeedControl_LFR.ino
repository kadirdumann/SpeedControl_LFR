  //Kadir Branch

  #include <AFMotor.h>
  #include <QTRSensors.h>

  AF_DCMotor motor1(4);  // Motor 1 -> Sol ön
  AF_DCMotor motor2(3);  // Motor 2 -> Sağ ön
  AF_DCMotor motor3(1);  // Motor 3 -> Sol arka
  AF_DCMotor motor4(2);  // Motor 4 -> Sağ arka

  typedef struct{
    byte rightPWM;
    byte leftPWM;
    int position;
    byte Rir;
    byte Lir;
    byte sensorArray[8];

  }Message;

  Message msg;

  int rightMotorSpeed;
  int leftMotorSpeed ;

  QTRSensors qtr;

  const uint8_t SensorCount = 8;
  unsigned int sensorValues[SensorCount];
  int QTR_Sensor_Array[SensorCount];
  unsigned int position;

  String value = "";
  const int leftFrontIRSensorPin = A15;
  const int rightFrontIRSensorPin = A14;

  void setup() {

    Serial1.begin(9600);
    Serial.begin(9600);

    pinMode(leftFrontIRSensorPin, INPUT);
    pinMode(rightFrontIRSensorPin, INPUT);

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]) {44, 42, 40, 38, 36, 34, 32, 30}, SensorCount);
    qtr_calibrate();
    delay(2000);
  }

  void loop() {
    qtr.read(sensorValues); // Sensör değerlerini oku
    position = qtr.readLineBlack(sensorValues);
    Serial.println("Konum: " + String(position));

    int leftFrontIRValue = digitalRead(leftFrontIRSensorPin);
    int rightFrontIRValue = digitalRead(rightFrontIRSensorPin);
    //Serial.println("Sağ IR = " + String(rightFrontIRValue));
    //Serial.println("Sol IR = " + String(leftFrontIRValue));

    Serial.println("Sağ PWM= " +  String(rightMotorSpeed));
    Serial.println("Sol PWM= " +  String(leftMotorSpeed));


    hizAyari();

  
    if(position <= 2500){
      if(position > 1750){
        motor1.setSpeed(leftMotorSpeed);
        motor1.run(FORWARD);
        motor2.setSpeed(rightMotorSpeed - (rightMotorSpeed * 0.2));
        motor2.run(FORWARD);
        motor3.setSpeed(leftMotorSpeed);
        motor3.run(FORWARD);
        motor4.setSpeed(rightMotorSpeed - (rightMotorSpeed * 0.2));
        motor4.run(FORWARD);
      }
      else if(position > 1000){
        motor1.setSpeed(leftMotorSpeed);
        motor1.run(FORWARD);
        motor2.setSpeed(rightMotorSpeed);
        motor2.run(BACKWARD);
        motor3.setSpeed(leftMotorSpeed);
        motor3.run(FORWARD);
        motor4.setSpeed(rightMotorSpeed);
        motor4.run(BACKWARD);
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
        motor1.setSpeed(leftMotorSpeed);
        motor1.run(BACKWARD);
        motor2.setSpeed(rightMotorSpeed);
        motor2.run(FORWARD);
        motor3.setSpeed(leftMotorSpeed);
        motor3.run(BACKWARD);
        motor4.setSpeed(rightMotorSpeed);
        motor4.run(FORWARD);
      }
      else if(position > 4000){
        motor1.setSpeed(leftMotorSpeed - (leftMotorSpeed * 0.2));
        motor1.run(FORWARD);
        motor2.setSpeed(rightMotorSpeed);
        motor2.run(FORWARD);
        motor3.setSpeed(leftMotorSpeed - (leftMotorSpeed * 0.2));
        motor3.run(FORWARD);
        motor4.setSpeed(rightMotorSpeed);
        motor4.run(FORWARD);
      }
    }
  

   if(leftFrontIRValue == 1 && rightFrontIRValue== 0){
      /*
      motor1.setSpeed(leftMotorSpeed);
      motor1.run(BACKWARD);
      motor2.setSpeed(rightMotorSpeed);
      motor2.run(FORWARD);
      motor3.setSpeed(leftMotorSpeed);
      motor3.run(BACKWARD);
      motor4.setSpeed(rightMotorSpeed);
      motor4.run(FORWARD);
      */
      Serial.println("SOLA 90 DÖNÜYOR");
    }
    if(rightFrontIRValue== 1 && leftFrontIRValue == 0 ){
      /*
      motor1.setSpeed(leftMotorSpeed);
      motor1.run(FORWARD);
      motor2.setSpeed(rightMotorSpeed);
      motor2.run(BACKWARD);
      motor3.setSpeed(leftMotorSpeed);
      motor3.run(FORWARD);
      motor4.setSpeed(rightMotorSpeed);
      motor4.run(BACKWARD);
      */
      Serial.println("SAĞA 90 DÖNÜYOR");
    }
  }


  void hizAyari(){
    if (Serial1.available() > 0) {
      String value = Serial1.readString();
      Serial.println("Value =  " + value);
      if(value == "start") {
        rightMotorSpeed =  120;
        leftMotorSpeed =  120;
      }
      if(value == "stop" ){
        stopMotors();
      }
      if(value == "1" ){
        rightMotorSpeed =  80;
        leftMotorSpeed =  80;
      }
      if(value == "2" ){
        rightMotorSpeed =  100;
        leftMotorSpeed =  100;
      }
      if(value == "3" ){
        rightMotorSpeed =  120;
        leftMotorSpeed =  120;
      }

      //Serial.println("right: " + String(rightMotorSpeed));
      //Serial.println("left: " + String(leftMotorSpeed));
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
  void stopMotors() {
    motor1.setSpeed(0);
    motor1.run(RELEASE);
    motor2.setSpeed(0);
    motor2.run(RELEASE);
    motor3.setSpeed(0);
    motor3.run(RELEASE);
    motor4.setSpeed(0);
    motor4.run(RELEASE);
  }

  void AGV_Telemetry_UART_Send(){
    String msg = "";
    msg += String(position) + "," + String(leftMotorSpeed) + "," + String(rightMotorSpeed) + "," + String(rightFrontIRValue) + "," + String(leftFrontIRValue) + ",";

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