#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <AccelStepper.h>

// Salidas para driver motor a pasos
#define DirPin 3                           // Pin dirrecion driver
#define StepPin 2                          // Pin #pasos driver
#define EnablePin 22                       // Pin enable driver
AccelStepper stepper1(1, StepPin, DirPin); // (Type of driver: with 2 pins, STEP, DIR)
boolean logica_enable = LOW;               // El driver se activa al recibir 0

// Entradas de los sensores
#define SensInd_Alu 5
#define SensCap_PET 6
#define SensCap_Vid 7

#define Pos_Sens_Centro 11
#define Pos_Sens_Vidrio 12 // DERECHA
#define Pos_Sens_LATA 13

// Pin servomotor
#define PinServo 8
Servo servoMotor;

// Boton que controla el inicio del proceso
#define BotonInicio 9
#define BotonSecundario 10
// Boton que detecta la salida del envase de la rampa
#define SensorSalida 28

// Variable que controla la secuencia de la maquina de estados
int Estado;
// Iterador mov motor condiciones normales
int a = 0;

// Variable que cuenta el numero de envases ingresados
int contador_envases = 0;
boolean Adicion_Envases = LOW;

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
// Posiciones para cada salida de material, Numero de pasos

void zero_motor();

void setup()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.clear();

  Serial.begin(9600);

  // Pines motor a pasos
  pinMode(StepPin, OUTPUT);
  pinMode(DirPin, OUTPUT);

  // Pines sensores
  pinMode(SensCap_PET, INPUT);
  pinMode(SensCap_Vid, INPUT);
  pinMode(SensInd_Alu, INPUT);
  // Pines Posicion sensores
  pinMode(Pos_Sens_Centro, INPUT);
  pinMode(Pos_Sens_LATA, INPUT);
  pinMode(Pos_Sens_Vidrio, INPUT);

  /*pinMode(LedPET, OUTPUT);
  pinMode(LedVid, OUTPUT);
  pinMode(LedAlu, OUTPUT);*/

  pinMode(BotonInicio, INPUT);
  pinMode(SensorSalida, INPUT);

  servoMotor.attach(PinServo);
  servoMotor.write(0);

  pinMode(DirPin, OUTPUT);
  pinMode(StepPin, OUTPUT);

  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);  // Set acceleration value for the stepper
  stepper1.setCurrentPosition(0); // Set the current position to 0 steps

  zero_motor(); // BUsca la posicion de origen
  Estado = 0;
}

void loop()
{
  // MAQUINA DE ESTADOS
  switch (Estado)
  {
  case 0: // SOLICITA ENVASE
    // Mientras no detecte ningun envase se solicitara colocar uno
    digitalWrite(EnablePin, HIGH); // Desactiva driver PAP
    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.print("Coloca  un  envase");
    Serial.println("Coloca un envase (caso 0)");
    delay(100);

    // leer estado de sensores
    if ((digitalRead(SensCap_PET) == LOW || digitalRead(SensCap_Vid) == LOW || digitalRead(SensInd_Alu) == HIGH))
    {
      lcd.clear();
      Serial.println("Envase detectado, se cambia al estado 1");
      Estado = 1; // Se detecta algun envase, cambia de estado
    }
    break;

  case 1: // SOLICITA PRESIONAR BOTON PARA INICIAR
    if (digitalRead(BotonInicio) == HIGH)
    {
      Serial.println("El usuario ha presionado el boton de inicio");
      lcd.clear();
      delay(300);
      // Cambia a la siguiente etapa que es discriminacion material
      Estado = 2;
    }

    // Mantiene el siguiente texto hasta que se presione el boton de inicio
    Serial.println("Se le solicita al usuario presionar el boton de inicio");
    lcd.setCursor(1, 0);
    lcd.print("Presiona el boton");
    lcd.setCursor(5, 2);
    lcd.print("de inicio");

    if (Adicion_Envases)
    {
      lcd.setCursor(2, 3);
      lcd.print("No de envases:");
      lcd.setCursor(16, 3);
      lcd.print(contador_envases);
    }
    delay(100);
    break;

  case 2: // INDICA TIPO DE  MATERIAL EN LCD Y ROTA RAMPA
    Serial.println("Se ha cambiado al estado 2, donde se indica el material detectado");

    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.print("Ingresaste:");
    lcd.setCursor(13, 1);
    delay(100);

    if (digitalRead(SensCap_PET) == LOW || digitalRead(SensCap_Vid) == LOW || digitalRead(SensInd_Alu) == HIGH)
    { // si algun sensor detecta se ejecuta esto
      // MODIFICAR O COMPROBAR SI es necesaria la linea superior

      // Muestra en pantalla que esta DETECTANDO LATA ALUMINIO
      if ((digitalRead(SensInd_Alu) == HIGH) && (digitalRead(SensCap_Vid) == LOW) && (digitalRead(SensCap_PET) == LOW))
      {
        Serial.println("se detecto Lata aluminio");
        lcd.print("Lata");
        // Se mueve el motor PAP hasta la posicion de la rampa de salida de Lata:  Derecha
        while (digitalRead(Pos_Sens_LATA)==HIGH) // Mientras no detecte la posicion de salida de lata, derecha
        {
          digitalWrite(EnablePin, logica_enable); // Activa driver
          stepper1.moveTo(a);                     // Set desired move:
          stepper1.runToPosition();               // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
          a--;
          Serial.println("wail DERECHA");
        }
        // Posicion_Material = Posicion_Lata; // cambiar por char
      }

      // Muestra en pantalla que esta DETECTANDO VIDRIO
      else if (digitalRead(SensCap_Vid) == LOW && digitalRead(SensCap_PET) == LOW)
      {
        lcd.print("Vidrio");
        Serial.println("se detecto Vidrio");
        // delay(200);
        //  Se mueve el motor PAP hasta la posicion de la rampa de salida de Vidrio:  IZQUIERDA
        while (digitalRead(Pos_Sens_Vidrio)==HIGH) // Mientras no detecte el sensor de posicion izquierda, salida vidrio
        {                                           // izquierda
          digitalWrite(EnablePin, logica_enable);   // Activa driver
          stepper1.moveTo(a);                       // Set desired move:
          stepper1.runToPosition();                 // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
          a++;
          Serial.println("wail 1 IZQUIERDA");
        }
        // Posicion_Material = Posicion_Vidrio;
      }

      // Muestra en pantalla que esta DETECTANDO PET
      else if ((digitalRead(SensCap_PET) == LOW) && digitalRead(SensCap_Vid) == HIGH && digitalRead(SensInd_Alu) == LOW)
      {
        lcd.print("PET");
        Serial.println("Se detecto PET");

        // delay(200);
        //  Se mueve el motor PAP hasta la posicion de la rampa de salida de Vidrio:  Centro (0)
        //digitalWrite(EnablePin, !logica_enable); // Desactiva driver
      }

      else
      {
        lcd.clear();
        lcd.setCursor(2, 1);
        lcd.print("Material no ");
        lcd.setCursor(2, 2);
        lcd.print("Reconocido");
        Serial.println("No se ha determinado que material se ha ingresado");
        // Posicion_Material = Posicion_Material; // Se mantiene en la misma posicion
      }

      digitalWrite(EnablePin, !logica_enable); // Desactiva driver PAP

      if (Adicion_Envases) // Si esta habilitado el conteo de envases
      {
        lcd.setCursor(2, 3);
        lcd.print("No de envases:");
        lcd.setCursor(16, 3);
        lcd.print(contador_envases); // Muestra el numero actual de envases
      }

      delay(500);
      lcd.clear();
      Estado = 3;
      break;
    }
    break;

  case 3:
    /* Indica que el envase se va a depositar en el contenedor
    Activa el servo para empujar el envase y se detecta su salida con un sensor al final de la rampa*/
    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.print("Depositando Envase");
    if (Adicion_Envases)
    {
      lcd.setCursor(2, 3);
      lcd.print("No de envases:");
      lcd.setCursor(16, 3);
      lcd.print(contador_envases);
    }
    delay(200);

    Serial.println("rota 90 grados el servomotor para elevar la rampa");
    servoMotor.write(90); // Se eleva la rampa con ayuda del servomotor

    if (digitalRead(SensorSalida) == HIGH) // Sensor salida de rampa detecta el envase que ha salido
    {
      Serial.println("Ha detectado la salida del envase de la rampa");
      lcd.clear();
      lcd.setCursor(3, 1);
      lcd.print("Ya quedo jefe");

      contador_envases++;
      lcd.setCursor(2, 3);
      lcd.print("No de envases:");
      lcd.setCursor(16, 3);
      lcd.print(contador_envases);

      servoMotor.write(0); // Regresa a la posicion inicialservomotor (horizontal)
      delay(500);

      // Vuelve a la posicion de origen el motor PAP
      Serial.println("retorna al centro el motor PAP y a su posicion inicial el servo");

      if (Pos_Sens_Centro == LOW) // ESTA EN EL CENTRO
      {
        Estado = 4;                              // en el estado 4 se pregunta si quiere agregar mas botellas o ya ha finalizado y se activa la
        digitalWrite(EnablePin, !logica_enable); // Desactiva driver PAP
      }
      else
      {
        if (digitalRead(Pos_Sens_Vidrio) == LOW) // Se encuentra en la posicion de salida del vidrio, izquierda
        {
          while (digitalRead(Pos_Sens_Centro == HIGH)) // Mientras no detecte la posicion de centro
          {
            digitalWrite(EnablePin, logica_enable); // Activa driver
            stepper1.moveTo(a);                     // Set desired move:
            stepper1.runToPosition();
            a--;
            Serial.println("wail DERECHA");
          }
        }
        else if (digitalRead(Pos_Sens_LATA) == LOW) // Se encuentra en la posicion de salida del lata, derecha
        {
          while (digitalRead(Pos_Sens_Centro == HIGH)) // Mientras no detecte la posicion de centro
          {
            digitalWrite(EnablePin, logica_enable); // Activa driver
            stepper1.moveTo(a);                     // Set desired move:
            stepper1.runToPosition();
            a++;
            Serial.println("wail IZQUIERDA");
          }
        }
      }

      // Se reinicia la maquina de estados
      // variable adicion envases
    }
    break;

  case 4:
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Agregar mas envases?");
    lcd.setCursor(0, 3);
    lcd.print("<-- SI ");
    lcd.setCursor(13, 3);
    lcd.print("NO -->");

   if (digitalRead(BotonSecundario)) // recibe pulso alto indicando que "NO"
   {
     lcd.clear();
     // Muestra el total de envases ingresados
     lcd.print("Ingresaste x envases de pet, x latas y x de vidrio");
     contador_envases = 0; // Reinicio de contador de envases
     Adicion_Envases = LOW;
     delay(3000);
     Estado = 0;
    }
    else if (digitalRead(BotonInicio)) 
    { // recibe pulso alto indicando que "SI"
      Adicion_Envases = HIGH;
      Estado = 0;
    }
    break;

  default:
    lcd.clear();
    lcd.setCursor(7, 2);
    lcd.print("ERROR 404");
    delay(200);
    break;
  }
}

void zero_motor()
{ // Esta funcion busca el punto establecido como cero u origen para el funcionamiento del PAP
  // limites
  int i  = 0;
  if (digitalRead(Pos_Sens_Centro) == LOW) // Sensor detecta el iman de la ubicacion de origen
  {
    stepper1.setCurrentPosition(0);// Set the current position to 0 steps
    digitalWrite(EnablePin, HIGH); // Desactiva driver PAP
    Serial.println("CENTRO");
    return;
  }
  else
  {
    // Mientras No detecta el centro ni el sensor de posicion del vidrio
    while ((digitalRead(Pos_Sens_Centro) == HIGH) && digitalRead(Pos_Sens_Vidrio) == HIGH) // Mueve hacia la derecha para buscar el sensor de ese limite
    {                                                                                      // derecha
      digitalWrite(EnablePin, LOW);                                              // Activa driver PAP
      stepper1.moveTo(i);                                                                  // Set desired move:
      stepper1.runToPosition();                                                            // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
      i--;
      Serial.println("wail DERECHA");
    }
    if (digitalRead(Pos_Sens_Vidrio) == LOW) // DETECTA EL SENSOR DE LA DERECHA
    {
      Serial.println("SEGUNDO IF");
      while (digitalRead(Pos_Sens_Centro) == HIGH) // Mientras no detecta el centro se aproxima a el
      {                                            // Izquierda
        digitalWrite(EnablePin, LOW);    // Activa driver PAP
        stepper1.moveTo(i);                         // Set desired move:
        stepper1.runToPosition();                  // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
        i++;
        Serial.println("wail IZQUIERDA");
      }
    }
  }
}

/*COMENTARIOS FUNCIONAMIENTO O CONEXION
Agregar contador y menu para continuar con la adicion de nuevos envases o finalizar el proceso
AGREGAR CONTADOR PARA CADA ENVASE O INDIVIDUAL en la zona correcta donde debe aumentar
agregar indicadores (flechas) para la opcion de cada boton, suponiendo que estan junto a la lcd
*/
