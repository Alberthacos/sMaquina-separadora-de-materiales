#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <AccelStepper.h>

// Salidas para driver motor a pasos
#define DirPin 3
#define StepPin 2
// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, StepPin, DirPin); // (Type of driver: with 2 pins, STEP, DIR)
#define EnablePin 30

// Entradas de los sensores
#define SensInd_Alu 4
#define SensCap_PET 5
#define SensCap_Vid 6

// LEDS SENSORES
#define LedAlu 7
#define LedVid 9
#define LedPET 10

// Pin servomotor
#define PinServo 11

// Boton que controla el inicio del proceso
#define BotonInicio 12
// Boton que detecta la salida del envase de la rampa
#define SensorSalida 13

// Variable que configura la posicion de origen
#define sensor_origen 22

// Variable que controla la secuencia del proceso
int Estado;

Servo servoMotor;

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

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

  pinMode(SensCap_PET, INPUT);
  pinMode(SensCap_Vid, INPUT);
  pinMode(SensInd_Alu, INPUT);

  pinMode(LedPET, OUTPUT);
  pinMode(LedVid, OUTPUT);
  pinMode(LedAlu, OUTPUT);

  pinMode(BotonInicio, INPUT);
  pinMode(SensorSalida, INPUT);

  pinMode(DirPin, OUTPUT);
  pinMode(StepPin, OUTPUT);

  servoMotor.attach(PinServo);

  pinMode(sensor_origen, INPUT);

  servoMotor.write(0);
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);  // Set acceleration value for the stepper
  stepper1.setCurrentPosition(0); // Set the current position to 0 steps

  zero_motor();
  Estado = 0;
}

void loop()
{
  // MAQUINA DE ESTADOS
  switch (Estado)
  {
  case 0: // SOLICITA ENVASE

    digitalWrite(EnablePin, HIGH);
    // Mientras no detecte ningun envase se solicitara colocar uno
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Coloca un envase");
    Serial.println("Coloca un envase (caso 0)");

    delay(100);

    // leer estados de sensores
    if ((digitalRead(SensCap_PET) == LOW || digitalRead(SensCap_Vid) == LOW || digitalRead(SensInd_Alu) == HIGH))
    {
      lcd.clear();
      Serial.println("Envase detectado, se cambia al estado 1");
      Estado = 1; // Se detecta algun envase, cambia de estado
    }
    break;

  case 1:                                 // SOLICITA PRESIONAR BOTON PARA INICIAR
    if (digitalRead(BotonInicio) == HIGH) // si se presiona el boton de inicio de proceso
                                          // Cambia a la siguiente etapa que es discriminacion material
    {
      Serial.println("El usuario ha presionado el boton de inicio");
      lcd.clear();
      delay(500);
      Estado = 2;
    }
    // Se mantiene el siguiente texto hasta que se presione le boton de inicio
    Serial.println("Se le solicita al usuario presionar el boton de inicio");
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
    Serial.println("Se ha cambiado al estado 2, donde se indica el material detectado");
    delay(200);

    // INDICA en la LCD el tipo de MATERIAL INGRESADO

    if (digitalRead(SensCap_PET) == LOW || digitalRead(SensCap_Vid) == LOW || digitalRead(SensInd_Alu) == HIGH)
    { // si algun sensor detecta se ejecuta esto

      lcd.setCursor(6, 2);

      // Muestra en pantalla que esta DETECTANDO LATA ALUMINIO
      if ((digitalRead(SensInd_Alu) == HIGH) && (digitalRead(SensCap_Vid) == LOW) && (digitalRead(SensCap_PET) == LOW))
      {
        lcd.print("Lata");
        Serial.println("se detecto Lata aluminio");

        // Se MUEVE el motor PAP hasta la posicion de la rampa de salida de LATAS:
        // DERECHA
        // MODIFICAR CON EL VALOR CORRECTO
        digitalWrite(EnablePin, LOW); //ACTIVA DRIVER
        delay(100);
        stepper1.moveTo(155);     // Set desired move: 800 steps (in quater-step resolution that's one rotation)
        stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in  position
        delay(400);
       // digitalWrite(EnablePin,LOW); //DESACTIVA DRIVER
      }

      // Muestra en pantalla que esta DETECTANDO VIDRIO
      else if (digitalRead(SensCap_Vid) == LOW && digitalRead(SensCap_PET) == LOW)
      {
        lcd.print("Vidrio");
        Serial.println("se detecto Vidrio");
        delay(200);

        // Se mueve el motor PAP hasta la posicion de la rampa de salida de Vidrio:
        // IZQUIERDA
        // MODIFICAR CON EL VALOR CORRECTO
        digitalWrite(EnablePin,LOW);
        delay(100);
        stepper1.moveTo(-155);    // Set desired move: 800 steps (in quater-step resolution that's one rotation)
        stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
        delay(400);
        digitalWrite(EnablePin,HIGH);
      }

      // Muestra en pantalla que esta DETECTANDO PET
      else if ((digitalRead(SensCap_PET) == LOW) && digitalRead(SensCap_Vid) == HIGH && digitalRead(SensInd_Alu) == LOW)
      {
        lcd.print("PET");
        Serial.println("Se detecto PET");
        delay(200);
        // POSICION CENTRAL, NO ROTA
        // MODIFICAR CON EL VALOR CORRECTO
        digitalWrite(EnablePin,LOW);
        delay(100);
        stepper1.moveTo(0);       // Set desired move: 800 steps (in quater-step resolution that's one rotation)
        stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
        delay(500);
        digitalWrite(EnablePin,HIGH);     
      }
      else
      {
        lcd.clear();
        lcd.setCursor(2, 1);
        lcd.print("Material no");
        lcd.setCursor(2, 2);
        lcd.print("Reconocido");
        Serial.println("No se ha determinado que material se ha ingresado");
      }

      delay(500);
      lcd.clear();
      Estado = 3;
      break;
    }
    break;

  case 3:
    // Indica que el envase se va a depositar en el contenedor
    //  Activa el servo para empujar el envase y se detecta su salida con un sensor al final de la rampa
    lcd.clear();
    lcd.setCursor(2, 1);
    lcd.print("Depositando");
    lcd.setCursor(4, 2);
    lcd.print("Envase");
    delay(200);

    Serial.println("rota 90 grados el servomotor para elevar la rampa");
    servoMotor.write(90); // Se eleva la rampa con ayuda del servomotor

    if (digitalRead(SensorSalida) == HIGH) // Sensore de salida de rampa detecta el envase que ha salido
    {
      Serial.println("Ha detectado la salida del envase de la rampa");
      lcd.clear();
      lcd.print("Ya quedo jefe");
      delay(100);
      servoMotor.write(0); // Regresa a la posicion inicial(plano, horizontal)
      delay(1000);

      // MODIFICAR VALOR CORRECTO PARA MOVERLO AL CENTRO
      Serial.println("retorna al centro el motor PAP");
      digitalWrite(EnablePin,LOW);
      delay(100);
      stepper1.moveTo(0);       // Set desired move: 800 steps (in quater-step resolution that's one rotation)
      stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
      digitalWrite(EnablePin,HIGH);
      delay(500);

      Serial.println("El sensor al final de la rampa ha detectado la salida del envasase y vuelven los motores a la posicion incial ");
      // Se reinicia la maquina de estados
      if (digitalRead(sensor_origen)==LOW)
      {
      Estado = 0;
      }
      else {
      zero_motor();
      }
      
    }
    break;

  default:
    lcd.clear();
    lcd.print("HOLA, NO deberias leer esto");
    delay(200);
    break;
  }
} 
void zero_motor(){
  //Variables control PAP
  int i = 0;
  int lim_inf =-110;
  int lim_sup = 220;
  
  if (digitalRead(sensor_origen) == LOW ){ //Sensor detecta el iman del punto de origen 
    //Definicion de posicion correcta del origen 
    stepper1.setCurrentPosition(0); // Set the current position to 0 steps
    digitalWrite(EnablePin,HIGH); //DESACTIVA
    return;
  }
  else 
  {
    while(digitalRead(sensor_origen) == HIGH && i >= lim_inf){ //derecha
      digitalWrite(EnablePin,LOW);
      stepper1.moveTo(i);     // Set desired move: 800 steps (in quater-step resolution that's one rotation)
      stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
      i--;
      Serial.println("wail 1");
    }
    if (digitalRead(sensor_origen) == LOW){
      stepper1.setCurrentPosition(0); // Set the current position to 0 steps
      digitalWrite(EnablePin,HIGH);
      return; 
    }
    else {  
      
      while(digitalRead(sensor_origen) == HIGH && i <= lim_sup){ //izquierda
        digitalWrite(EnablePin,LOW); //HIGH
        stepper1.moveTo(i);     // Set desired move: 800 steps (in quater-step resolution that's one rotation)
        stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
        i++;
        Serial.println("wail 2");
      }
    }
  }
  
}

/*COMENTARIOS FUNCIONAMIENTO O CONEXION

 hall e infra mandan 0 al detectar
Agregar caso default si todos los sensores no detectan su combinacion
*/