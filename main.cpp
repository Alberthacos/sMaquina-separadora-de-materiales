#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>

// Entradas de los sensores
#define SensCap_PET 6
#define SensCap_Vid 7
#define SensInd_Alu 8

// LEDS SENSORES
#define LedPET 11
#define LedVid 12
#define LedAlu 13

// Salidas driver para motor a pasos
#define stepPin 22
#define dirPin 23
#define sHall 24

// Boton que controla el inicio del proceso
#define BotonInicio 26

// Sensor de salida de rampa
#define SensInfraRampa 27

// Variable que controla la secuencia de la maquina de estados
int Etapa;

int sentidoRot;

// Variable que guarda el valor de lectura de cada sensor
bool sensPET;
bool sensVid;
bool sensAlu;

Servo servoMotor;

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

void LEDS(); // Funcion que controla los leds que indican la deteccion de cada sensor

void setup()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.clear();

  Serial.begin(9600); // Initialize serial

  // Pines motor a pasos
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(sHall, INPUT);

  // Pines sensores capacitivos, inductivo y sus respectivos leds
  pinMode(SensCap_PET, INPUT);
  pinMode(SensCap_Vid, INPUT);
  pinMode(SensInd_Alu, INPUT);
  pinMode(LedPET, OUTPUT);
  pinMode(LedVid, OUTPUT);
  pinMode(LedAlu, OUTPUT);

  // Boton de inicio y sensor salida de rampa
  pinMode(SensInfraRampa, INPUT);
  pinMode(BotonInicio, INPUT);

  // Pines driver para motor PAP
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(sHall, INPUT);

  // Salida servo que eleva la rampa
  servoMotor.attach(9);
  
  // Se inicializa la FSM en la etapa primer etapa (0)
  Etapa = 0;
}

void loop()
{
  sensAlu = digitalRead(SensInd_Alu);
  sensPET = digitalRead(SensCap_PET);
  sensVid = digitalRead(SensCap_Vid);

  switch (Etapa)
  {
  case 0:
    // Mientras no detecte ningun envase se solicitara colocar uno
    lcd.setCursor(0, 0);
    lcd.print("Coloca un envase");
    Serial.println("Coloca un envase");
    delay(200);

    // Verificar si se detecta algun envase
    if (sensPET == LOW || sensVid == LOW || sensAlu == HIGH)
    {
      lcd.clear();
      Etapa = 1; // Se detecta algun envase, cambia de etapa
      Serial.println("Envase detectado al inicio ");
    }
    break;

  case 1:
    if (digitalRead(BotonInicio) == HIGH) // Se presiona el boton de inicio de proceso
    {
      lcd.clear();
      Etapa = 2; // Cambia a la siguiente etapa // Discriminacion material
    }
    lcd.setCursor(0, 0);
    lcd.print("Presiona boton");
    lcd.setCursor(3, 1);
    lcd.print("de inicio");
    delay(200);
    break;

  case 2:
    // lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ingresaste:");
    delay(100);

    // Indica en la LCD el tipo de material ingresado
    if (sensPET == LOW || SensCap_Vid == LOW || sensAlu == HIGH)
    { // si algun sensor detecta se ejecuta

      lcd.setCursor(6, 2);
      // Muestra en pantalla que esta detectando Lata
      if (sensAlu == HIGH && sensVid == LOW && sensPET == LOW)
      {
        lcd.print("Lata");
        Serial.println("Lata");

        // Se mueve el motor PAP hasta la posicion de la rampa de salida de LATAS:
        // Derecha
        digitalWrite(dirPin, LOW);
        sentidoRot = 1;
        // gira en sentido hacia la rampa de salida de LATAS
        for (int x = 0; x < 50; x++)
        {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(500);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(500);
        }
        //////////////////////////////////

        delay(1000);
      }

      // Muestra en pantalla que esta detectando vidrio
      else if (digitalRead(SensCap_Vid) == LOW && digitalRead(SensCap_PET) == LOW)
      {
        lcd.print("Vidrio");
        Serial.println("Vidrio");

        // Se mueve el motor PAP hasta la posicion de la rampa de salida de Vidrio:
        // IZQUIERDA
        digitalWrite(dirPin, HIGH);
        sentidoRot = 2;
        // gira en sentido hacia la rampa de salida de Vidrio
        for (int x = 0; x < 50; x++)
        {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(500);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(500);
        }
        //////////////////////////////////

        delay(1000);
      }

      // Muestra en pantalla que esta detectando PET
      else if (sensPET == LOW)
      {
        lcd.print("PET");

        // POSICION CENTRAL, NO ROTA
        digitalWrite(stepPin, LOW); // no gira el motor central, su salida esla rampa del centro
        sentidoRot = 0;

        delay(2000);
      }

      delay(100);  // se puede eliminar
      lcd.clear(); // Limpia LCD
      Etapa = 3;
    }

    /*
    Agregar un else para un estado de error ya que no hay envase, o no funcionan correctamente los sensores
    Estado de error despues de un tiempo de no deteccion */

    break; // si no funciona, mover arriba de la llave

  case 3: /*En este estado se indica al usuario que se esta depositando el envase
          Y detecta su salida de la rampa*/

    /* lcd.clear();
     En este estado se eleva la rampa pequeña para empujar el envase hacia la rampa de salida*/
    lcd.setCursor(0, 0);
    lcd.print("Depositando");
    lcd.setCursor(4, 1);
    lcd.print("Envase");

    servoMotor.write(90); // Se eleva la rampa

    delay(200);

    /*if (sentidoRot = 1)
    { // giro a la derecha
      // Rota en el sentido opuesto hasta el centro cuando detecte el sHall
      digitalWrite(dirPin, HIGH);
      // gira en sentido hacia la rampa de salida de Vidrio
      for (int x = 0; x < 50; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
      sentidoRot = 5;
      }
    }
    else if (sentidoRot = 2)
    { // giro a la izquierda
      // Rota en el sentido opuesto hasta el centro cuando detecte el sHall
      digitalWrite(dirPin, LOW);
      // gira en sentido hacia la rampa de salida de Vidrio
      for (int x = 0; x < 50; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
      }
      sentidoRot = 5;
    }
    else
    {
      // No retorna porque se mantuvo en la posicion
      sentidoRot = 5;
    }*/
    // revisar si el tiempo de arriba es menor al tiempo que le toma al envase llegar al sensor de salida de la rampa (200ms)

    if (digitalRead(SensInfraRampa) == HIGH) // Sensore de salida de rampa detecta que el envase a ha salido
    {
      lcd.clear();
      lcd.print("LISTO PATRON");
      delay(1500);

      servoMotor.write(0); // Regresa a la posicion inicial(plano, horizontal)

      Etapa = 0; // Se reinicia la FSM, vuelve al estado inicial 0

      lcd.clear(); // Limpia la LCD
    }

    break;

  default:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ESTO NO DEBERIA ESTAR SUCEDIENDO");
    delay(500);
    break;
  }
}

void LEDS()
{

  if (sensPET == LOW)
  {
    digitalWrite(LedPET, HIGH);
  }
  else
  {
    digitalWrite(LedPET, LOW);
  }

  if (sensVid == LOW)
  {
    digitalWrite(LedVid, HIGH);
  }
  else
  {
    digitalWrite(LedVid, LOW);
  }

  if (sensAlu == HIGH)
  {
    digitalWrite(LedAlu, HIGH);
  }
  else
  {
    digitalWrite(LedAlu, LOW);
  }
}
