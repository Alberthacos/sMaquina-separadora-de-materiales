#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>

// Entradas de los sensores
#define SensInd_Alu 5
#define SensCap_PET 6
#define SensCap_Vid 7

#define Pos_Sens_Centro 24
#define Pos_Sens_Vidrio 26 // DERECHA
#define Pos_Sens_LATA 22   // IZQUIERDA

// Pin servomotor
#define PinServo 4
Servo servoMotor;

// Boton que controla el inicio del proceso
#define BotonInicio 2
#define BotonSecundario 3
// Boton que detecta la salida del envase de la rampa
#define SensorSalida 12

// Pines control puente H
#define En 8
#define In_1 9
#define In_2 10

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

  // Pines sensores
  pinMode(SensCap_PET, INPUT);
  pinMode(SensCap_Vid, INPUT);
  pinMode(SensInd_Alu, INPUT);
  // Pines Posicion sensores
  pinMode(Pos_Sens_Centro, INPUT);
  pinMode(Pos_Sens_LATA, INPUT);
  pinMode(Pos_Sens_Vidrio, INPUT);

  pinMode(BotonInicio, INPUT);
  pinMode(SensorSalida, INPUT);

  pinMode(En, OUTPUT);
  pinMode(In_1, OUTPUT);
  pinMode(In_2, OUTPUT);

  digitalWrite(En, LOW);
  digitalWrite(In_1, LOW);
  digitalWrite(In_2, LOW);

  servoMotor.attach(PinServo);
  servoMotor.write(0);

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
    else
    {
      Estado = 0;
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
        while (digitalRead(Pos_Sens_LATA) == HIGH) // Mientras no detecte la posicion de salida de lata, derecha
        {
          analogWrite(En, 25);
          // Salidas de motor con sentido horario
          digitalWrite(In_1, LOW);
          digitalWrite(In_2, HIGH);

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
        analogWrite(En, 25);
        //  Se mueve el motor PAP hasta la posicion de la rampa de salida de Vidrio:  IZQUIERDA
        while (digitalRead(Pos_Sens_Vidrio) == HIGH) // Mientras no detecte el sensor de posicion izquierda, salida vidrio
        {                                            // izquierda
          // Salidas de motor con sentido antihorario
          digitalWrite(In_1, HIGH);
          digitalWrite(In_2, LOW);
          Serial.println("wail 1 IZQUIERDA");
        }
        // Posicion_Material = Posicion_Vidrio;
      }

      // Muestra en pantalla que esta DETECTANDO PET
      else if ((digitalRead(SensCap_PET) == LOW) && digitalRead(SensCap_Vid) == HIGH && digitalRead(SensInd_Alu) == LOW)
      {
        lcd.print("PET");
        Serial.println("Se detecto PET");
        // Salidas a motor apagadas
        digitalWrite(In_1, LOW);
        digitalWrite(In_2, LOW);
      }

      else
      {
        lcd.clear();
        lcd.setCursor(2, 1);
        lcd.print("Material no ");
        lcd.setCursor(2, 2);
        lcd.print("Reconocido");
        Serial.println("No se ha determinado que material se ha ingresado");
        digitalWrite(In_1, LOW);
        digitalWrite(In_2, LOW);
      }

      if (Adicion_Envases) // Si esta habilitado el conteo de envases
      {
        lcd.setCursor(2, 3);
        lcd.print("No de envases:");
        lcd.setCursor(16, 3);
        lcd.print(contador_envases); // Muestra el numero actual de envases
      }

      delay(500);
      lcd.clear();
      digitalWrite(En, LOW);
      Serial.println("rota 90 grados el servomotor para elevar la rampa");
      servoMotor.write(90); // Se eleva la rampa con ayuda del servomotor
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

    Serial.println("rota 90 grados el servomotor para elevar la rampa");
    servoMotor.write(90); // Se eleva la rampa con ayuda del servomotor
    delay(100);

    if (digitalRead(SensorSalida) == LOW) // Sensor salida de rampa detecta el envase que ha salido
    {
      Serial.println("Ha detectado la salida del envase de la rampa");
      lcd.clear();
      lcd.setCursor(3, 1);
      lcd.print("Ya quedo jefe");

      servoMotor.write(0); // Regresa a la posicion inicialservomotor (horizontal)
      delay(100);
      Estado = 4;

      // Vuelve a la posicion de origen el motor PAP
      Serial.println("retorna al centro el motor PAP y a su posicion inicial el servo");

      // Se reinicia la maquina de estados
      // variable adicion envases
    }
    break;

  case 4:
    zero_motor();
    Estado = 0;
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
  analogWrite(En, 25);

  // Mientras No detecta el centro ni el sensor de posicion del vidrio
  while ((digitalRead(Pos_Sens_Centro) == HIGH) && digitalRead(Pos_Sens_Vidrio) == HIGH) // Mueve hacia la derecha para buscar el sensor de ese limite
  {
    // ROTA DERECHA                                                             // derecha
    digitalWrite(In_1, HIGH);
    digitalWrite(In_2, LOW);

    Serial.println("Rota a derecha");
  }

  if (digitalRead(Pos_Sens_Vidrio) == LOW && digitalRead(Pos_Sens_Centro) == HIGH && digitalRead(Pos_Sens_LATA) == HIGH) // POSICION DERECHA
  {
    Serial.println("derecha encontrada");
    // ROTA IZQUIERDA HASTA EL CENTRO
    while (digitalRead(Pos_Sens_Centro) == HIGH)
    {
      Serial.println("ROTANDO IZQUIERDA");
      digitalWrite(In_1, LOW);
      digitalWrite(In_2, HIGH);
    }
  }
  //

  if (digitalRead(Pos_Sens_Centro) == LOW)
  {
    Serial.println("CENTRO ENCONTRADO");
    digitalWrite(En, LOW);
    return;
  }
  //
  // if (digitalRead(Pos_Sens_Centro == LOW))
  //{
  //  Serial.println("CENTRO ENCONTRADO");
  //  digitalWrite(En, LOW);
  //  return;
  //}
  //
  //// if (digitalRead(Pos_Sens_Vidrio) == LOW) // DETECTA EL SENSOR DE LA DERECHA
  //{
  //   while (digitalRead(Pos_Sens_Centro) == HIGH) // Mientras no detecta el centro se aproxima a el
  //   {                                            // Izquierda
  //     digitalWrite(In_1, LOW);
  //     digitalWrite(In_2, HIGH);
  //     Serial.println("wail IZQUIERDA");
  //   }
  // }
}

/*COMENTARIOS FUNCIONAMIENTO O CONEXION
Agregar contador y menu para continuar con la adicion de nuevos envases o finalizar el proceso
AGREGAR CONTADOR PARA CADA ENVASE O INDIVIDUAL en la zona correcta donde debe aumentar
agregar indicadores (flechas) para la opcion de cada boton, suponiendo que estan junto a la lcd
SERVOMOTOR APAGAR AL INICIO
*/
