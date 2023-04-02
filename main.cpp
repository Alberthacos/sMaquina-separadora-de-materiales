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

// Salidas motor a pasos
#define stepPin 22
#define dirPin 23
#define sHall 24

// Boton que controla el inicio del proceso
#define BotonInicio 26
#define BotonDeposito 27

// Variables que almacenan el estado de cada se sensor
/*bool SensPET;
bool SensVid;
bool SensAlu;*/

// Variable que controla la secuencia del proceso
int Etapa;

// Variable que guarda la direccion de rotacion del motor PAP
int sentidoRot; // tiene 3 valores, 0 para posicion central, 1 para derecha, -1 para izquierda

Servo servoMotor;

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.clear();
  Serial.begin(9600);
  // Pines motor a pasos
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(sHall, INPUT);

  pinMode(SensCap_PET, INPUT);
  pinMode(SensCap_Vid, INPUT);
  pinMode(SensInd_Alu, INPUT);
  pinMode(LedPET, OUTPUT);
  pinMode(LedVid, OUTPUT);
  pinMode(LedAlu, OUTPUT);

  pinMode(BotonDeposito, INPUT);
  pinMode(BotonInicio, INPUT);

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(sHall, INPUT);
  servoMotor.attach(9);
  /*
    while (sHall == LOW)
    { // Mientras no detecte se mueve
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(700); // by changing this time delay between the steps we can change the rotation speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(700);
    }*/

  Etapa = 0;
}

void loop()
{
  // SensPET == digitalRead(SensCap_PET);
  // SensAlu == digitalRead(SensInd_Alu);
  // SensVid == digitalRead(SensCap_Vid);

  // Usuario coloca el envase en el espacio designado
  // Se muestra en pantalla "Ingresa un envase y presiona el boton de inicio "

  if (digitalRead(SensCap_PET) == LOW)
  {
    digitalWrite(LedPET, HIGH);
  }
  else
  {
    digitalWrite(LedPET, LOW);
  }

  if (digitalRead(SensCap_Vid) == LOW)
  {
    digitalWrite(LedVid, HIGH);
  }
  else
  {
    digitalWrite(LedVid, LOW);
  }

  if (digitalRead(SensInd_Alu) == HIGH)
  {
    digitalWrite(LedAlu, HIGH);
  }
  else
  {
    digitalWrite(LedAlu, LOW);
  }

  switch (Etapa)
  {
  case 0:
    // Mientras no detecte ningun envase se solicitara colocar uno

    lcd.setCursor(0, 0);
    lcd.print("Coloca un envase");
    Serial.println("Coloca un envase");
    delay(200);
    // checar estados de activacion
    if ((digitalRead(SensCap_PET) == LOW || digitalRead(SensCap_Vid) == LOW || digitalRead(SensInd_Alu) == HIGH))
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
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ingresaste:");
    delay(200);
    // Indica en la LCD el tipo de material ingresado
    if (digitalRead(SensCap_PET) == LOW || digitalRead(SensCap_Vid) == LOW || digitalRead(SensInd_Alu) == HIGH)
    { // si algun sensor detecta se ejecuta esto

      lcd.setCursor(6, 2);
      // Muestra en pantalla que esta detectando Lata aluminio
      if (digitalRead(SensInd_Alu) == HIGH && digitalRead(SensCap_Vid) == LOW && digitalRead(SensCap_PET) == LOW)
      {
        lcd.print("Lata");
        delay(1000);
        ///////////////////////////////////////
        // Se mueve el motor PAP hasta la posicion de la rampa de salida de LATAS:
        // DERECHA
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
        delay(100);
        //////////////////////////////////
      }
      // Muestra en pantalla que esta detectando vidrio
      else if (digitalRead(SensCap_Vid) == LOW && digitalRead(SensCap_PET) == LOW)
      {
        lcd.print("Vidrio");
        delay(1000);
        Serial.println("Vidrio");
        // Se mueve el motor PAP hasta la posicion de la rampa de salida de Vidrio:
        // IZQUIERDA
        ///////////////////////////////////////
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

        delay(100);
      }
      // Muestra en pantalla que esta detectando PET
      else if (digitalRead(SensCap_PET) == LOW)
      {
        lcd.print("PET");
        delay(1000);
        // POSICION CENTRAL, NO ROTA
        digitalWrite(stepPin, LOW); // no gira el motor central, su salida esla rampa del centro
        sentidoRot = 0;
      }

      delay(200);
      lcd.clear();
      Etapa = 3;
      break;
    }
    break;

  case 3:
    // lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Depositando"); // Activa el servo para empujar el envase
    lcd.setCursor(4, 1);
    lcd.print("Envase");
    delay(100);
    servoMotor.write(90); // Se inclina la rampa

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

    if (digitalRead(BotonDeposito) == HIGH) // Sensore de salida de rampa detecta el envase que ha salido
    {
      lcd.clear();
      lcd.print("LISTO PATRON");
      delay(1500);
      servoMotor.write(0); // Regresa a la posicion inicial(plano, horizontal)
      Etapa = 0;

      lcd.clear();
    }

    break;

  default:
    lcd.print("HOLA");
    break;
    delay(500);
  }
}

// hall e infra mandan 0 al detectar