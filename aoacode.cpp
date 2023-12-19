// Bibliotecas
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Thread.h>
#include <Servo.h>
// Pinagem
#define som 9
#define ledverde 6
#define ledvermelho 5
#define ledamarelo 7
#define pinServo 10

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

Servo meuServo;

Thread myThread = Thread();

float accelX;


void setup(){
  pinMode(som, OUTPUT);
  pinMode(ledverde, OUTPUT);
  pinMode(ledvermelho, OUTPUT);
  pinMode(ledamarelo, OUTPUT);

  meuServo.attach(pinServo); // Inicializa o servo no pino especificado
  meuServo.write(90);

  tone(som,2700,500);
  delay(150);
  tone(som,1900,500);
  delay(150);
  
  Serial.begin(9600);

  if(!accel.begin()){
    Serial.println("Nenhum sensor detectado!");
    while(1);
  }
   myThread.onRun(checkAcceleration);
   myThread.setInterval(10);
   myThread.run();


}

void checkAcceleration() {
  while (1) {
    
    sensors_event_t event; 
    accel.getEvent(&event);
    accelX = event.acceleration.x;

    int servoPos = map(accelX, -4.0, 4.0, 180, 0);
    servoPos = constrain(servoPos, 0, 180);
    meuServo.write(servoPos);

    if (accelX >= 4.0 || accelX <= -4.0) {
      alerta();
      luzverdeoff();
    }
      else {
      luzverde();
      }

      if (accelX >= 4.0) {
        luzvermelha();
      }
      else {
        luzvermelhaoff();
      }

      if (accelX <= -4.0) {
        luzamarela();
      }
      else {
        luzamarelaoff();
      }
  

    delay(10);
  }
}

void loop(){
 
    // Mostra os valores X, Y e Z do acelerômetro em tempo real com delay de 500

    //sensors_event_t event;
    //accel.getEvent(&event);
    //Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    //Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    //Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");
    //Serial.println("m/s^2 ");
    //delay(500);
  delay(10);
}

void alerta(){

  tone(som, 1000, 100);
  delay(100);
  tone(som, 1200, 100);
  delay(100);
  tone(som, 1400, 100);
  delay(100);
  noTone(som);  // Desliga o buzzer
  delay(200);

  Serial.println("Alerta! Estol!");
}

void luzverde(){

  digitalWrite(ledverde, HIGH);
}
void luzverdeoff(){

  digitalWrite(ledverde, LOW);
}

void luzvermelha(){

  digitalWrite(ledvermelho, HIGH);
}
void luzvermelhaoff(){

  digitalWrite(ledvermelho, LOW);
}

void luzamarela(){

  digitalWrite(ledamarelo, HIGH);
}
void luzamarelaoff(){

  digitalWrite(ledamarelo, LOW);
}