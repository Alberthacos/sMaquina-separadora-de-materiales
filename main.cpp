#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <AccelStepper.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 2, 5); // (Type of driver: with 2 pins, STEP, DIR)

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

  // Set maximum speed value for the stepper
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(1000); // Set acceleration value for the stepper
  stepper1.setCurrentPosition(0); // Set the current position to 0 steps

  Etapa = 0;
}

void loop()
{
  // SensPET == digitalRead(SensCap_PET);
  // SensAlu == digitalRead(SensInd_Alu);
  // SensVid == digitalRead(SensCap_Vid);

  // Usuario coloca el envase en el espacio designado
  // Se muestra en pantalla "Ingresa un envase y presiona el boton de inicio "

  // ESTADO LEDS DE SENSORES
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

  // MAQUINA DE ESTADOS GENERAL
  switch (Etapa)
  {
  case 0: // SOLICITA ENVASE

    // Mientras no detecte ningun envase se solicitara colocar uno
    lcd.setCursor(0, 0);
    lcd.print("Coloca un envase");
    Serial.println("Coloca un envase (caso 0)");
    delay(200);

    // leer estados de sensores
    if ((digitalRead(SensCap_PET) == LOW || digitalRead(SensCap_Vid) == LOW || digitalRead(SensInd_Alu) == HIGH))
    {
      lcd.clear();
      Serial.println("Envase detectado");
      Etapa = 1; // Se detecta algun envase, cambia de estado
    }
    break;

  case 1:                                 // SOLICITA PRESIONAR BOTON PARA INICIAR
    if (digitalRead(BotonInicio) == HIGH) // si se presiona el boton de inicio de proceso
                                          // Cambia a la siguiente etapa que es discriminacion material
    {
      lcd.clear();
      Etapa = 2;
    }
    // Se mantiene el siguiente texto hasta que se presione le boton de inicio
    lcd.setCursor(0, 0);
    lcd.print("Presiona boton");
    lcd.setCursor(3, 1);
    lcd.print("de inicio");
    delay(200);
    break;

  case 2: // INDICA TIPO DE  MATERIAL EN LCD Y ROTA RAMPA
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ingresaste:");
    delay(200);

    // INDICA en la LCD el tipo de MATERIAL INGRESADO

    if (digitalRead(SensCap_PET) == LOW || digitalRead(SensCap_Vid) == LOW || digitalRead(SensInd_Alu) == HIGH)
    { // si algun sensor detecta se ejecuta esto

      lcd.setCursor(6, 2);

      // Muestra en pantalla que esta DETECTANDO LATA ALUMINIO
      if (digitalRead(SensInd_Alu) == HIGH && digitalRead(SensCap_Vid) == LOW && digitalRead(SensCap_PET) == LOW)
      {
        lcd.print("Lata");
        delay(1000);

        // Se MUEVE el motor PAP hasta la posicion de la rampa de salida de LATAS:
        // DERECHA
        // MODIFICAR CON EL VALOR CORRECTO
        stepper1.moveTo(800);     // Set desired move: 800 steps (in quater-step resolution that's one rotation)
        stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in  position
        delay(100);
      }

      // Muestra en pantalla que esta DETECTANDO VIDRIO
      else if (digitalRead(SensCap_Vid) == LOW && digitalRead(SensCap_PET) == LOW)
      {
        lcd.print("Vidrio");
        delay(1000);
        Serial.println("Vidrio");

        // Se mueve el motor PAP hasta la posicion de la rampa de salida de Vidrio:
        // IZQUIERDA
        // MODIFICAR CON EL VALOR CORRECTO
        stepper1.moveTo(800);     // Set desired move: 800 steps (in quater-step resolution that's one rotation)
        stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
        delay(100);
      }

      // Muestra en pantalla que esta DETECTANDO PET
      else if (digitalRead(SensCap_PET) == LOW)
      {
        lcd.print("PET");
        delay(1000);
        // POSICION CENTRAL, NO ROTA
        // MODIFICAR CON EL VALOR CORRECTO
        stepper1.moveTo(0);       // Set desired move: 800 steps (in quater-step resolution that's one rotation)
        stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
      }

      delay(200);
      lcd.clear();
      Etapa = 3;
      break;
    }
    break;

  case 3:
    // Indica que el envase se va a depositar en el contenedor
    //  Activa el servo para empujar el envase y se detecta su salida con un sensor al final de la rampa
    lcd.print("Depositando");
    lcd.setCursor(0, 0);
    lcd.setCursor(4, 1);
    lcd.print("Envase");
    delay(100);
    servoMotor.write(90); // Se eleva la rampa

    //MODIFICAR VALOR CORRECTO PARA MOVERLO AL CENTRO
    stepper1.moveTo(800);     // Set desired move: 800 steps (in quater-step resolution that's one rotation)
    stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position

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