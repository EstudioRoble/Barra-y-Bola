/*
   _200_BarraBola_v3.0
   Firmware para Arduino UNO del proyecto Barra y Bola v2
   descrito en:

   Puedes comprar tu kit en:
   https://roble.uno/product/barra-y-bola/
   bajo Licencia Creative Commons Atribución-CompartirIgual 4.0 Internacional.
   https://creativecommons.org/licenses/by-sa/4.0/

   4 de diciembre de 2019
   por Angel Espeso
   ESTUDIO ROBLE
   www.roble.uno

*/

#include <TaskScheduler.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TiCoServo.h>
#include <EEPROM.h>
#include <Wire.h>
#include <hd44780.h>  // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h>  // i2c expander i/o class header
#include <Encoder.h>
#include <Keypad.h>

const int pidMillis = 50;
const int lcdrMillis = 200;
const int buttonMillis = 100;
// task prototypes
Scheduler runner;
Task t1(pidMillis, TASK_FOREVER, &pid);
Task t2(lcdrMillis, TASK_FOREVER, &lcdr);
Task t3(buttonMillis, TASK_FOREVER, &button);
// *****************************************************************************
#define sensorPin 0 //Pin Analogico donde esta conectada la señal del Sensor de distancia
#define PIN       4 // neoPixels
#define PINencA   3
#define PINencB   2
#define PINservo  9
#define pinButton 5
#define Piezo     A3
//      LCD       A4, A5

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      32
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

uint32_t azul = pixels.Color(0, 0, 3);
uint32_t blanco = pixels.Color(10, 10, 10);
uint32_t rojo = pixels.Color(60, 0, 0);
uint32_t rojoclaro = pixels.Color(4, 0, 0);
uint32_t verde = pixels.Color(0, 60, 0);
uint32_t verdeclaro = pixels.Color(0, 4, 0);

int measure; // Lo que mide el sensor. Son ADCs.
float kdist = 0.4; // Kalman Gain para dist
int ciclos = 0; // Contamos los ciclos de medida para que no nos arruine la graficas
float unfilteredDist; // measure después de calibracion
float kvel = 0.4; // Kalman Gain para vel
float dist; // unfilteredDistdespues de filtro kalman
float prevDist; // Valor anterior de Distancia para calcular la velocidad

float unfilteredVel; // velocidad como diferencia entre posiciones
float vel; // unfilteredVel despues de filtro Kalman
float I; // Valor Integral

int setPointTime = 4; // tiempo en segundos para alternar entre setPoints
unsigned long pidBeginTime;
float setPoint;
boolean setPointAoB = true; // True:A  False:B
float setPointA = 6;
float setPointB = 20;
boolean skin = true; // True:punto  False:laser

float error;
float Kp = 12; //12;
float Kd = 260; //280;
float Ki = 0.5; //0.5;
int Rint = 1;
int Rext = 3;

Adafruit_TiCoServo myservo;
float pos;
float reposo = 1510; // valor que mantiene la barra horizontal
float barrainclinada = 100; // Desviación para calibracion

// Mapa de Memoria
//[ 0 -  3] SetPoints   [Sin usar en esta version]
//[ 4 - 15] kp, kd, ki  [Sin usar en esta version]
//[16 - 19] Valor Horizontal de Reposo
//[20 - 59] Lecturas del sensor en calPoint
float calPoint[] = {0, 4, 8, 12, 16, 20, 23, 27, 31};
int nCalPoints = sizeof(calPoint) / 4; // Numero de puntos de calibracion. / 4 porque 1 float ocupa 4 bytes
float calValue[sizeof(calPoint) / 4];
/*
  0  0.00  969
  1 4.00  711
  2 8.00  553
  3 12.00 448
  4 16.00 377
  5 20.00 329
  6 23.00 305
  7 27.00 276
  8 31.00 252
*/
hd44780_I2Cexp lcd; // declare lcd object: auto locate & config exapander chip
const int LCD_COLS = 20;
const int LCD_ROWS = 4;
char c = B11111111;

Encoder myEnc(PINencA, PINencB);
//   avoid using pins with LEDs attached
long newPosition;
long oldPosition  = 0;

unsigned long buttonTime;
int buttonState = 1;
int buttonState0 = 1;

int state = 10;
boolean in = false;

int options [] = {0, 5, 4, 1, 1}; // Numero de opciones que hay en cada pantalla
int cursorPID [] = {0, 20, 40, 60, 72, 76}; // Posicion de opciones en pantalla PID
int cursorSetP [] = {0, 20, 30, 40, 60}; // Posicion de opciones en pantalla SetPoint
int cursorNiv [] = {0, 47}; // Posicion de opciones en pantalla Nivelacion
int cursorCal [] = {0, 65}; // Posicion de opciones en pantalla Calibacion

const byte rowsCount = 4;
const byte columsCount = 4;
char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[rowsCount] = { 6, 7, 8, 10 };
byte columnPins[columsCount] = { 11, 12, A1, A2 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, columnPins, rowsCount, columsCount);

void setup() {
  tone(Piezo, 1175, 100);
  pinMode(pinButton, INPUT_PULLUP);
  Serial.begin(115200);

  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.lineWrap();
  lcd.clear();

  runner.init();
  runner.addTask(t1);
  t1.enable();
  runner.addTask(t2);
  t2.enable();
  runner.addTask(t3);
  t3.enable();

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(255);

  analogReference(EXTERNAL); // AREF conectado a 3.3V
  myservo.attach(PINservo);  // attaches the servo on pin X to the servo object
  // Poblamos reposo (Horizontal) con dato leido de la EEPROM
  EEPROM.get(16, reposo);
  if (0) {
    Serial.print("Valor de reposo rescatado de la EEPROM: ");
    Serial.print("\t");
    Serial.println(reposo);
  }
  // Poblamos calValue[] con datos leidos de la EEPROM
  for (int i = 0; i < nCalPoints; i++) {
    EEPROM.get(20 + i * 4, calValue[i]);
    // Mostramos puntos de calibración
    if (0) {
      Serial.print(i);
      Serial.print("\t");
      Serial.print(calPoint[i]);
      Serial.print("\t");
      Serial.println(calValue[i]);
    }
  }
  pidBeginTime = millis();
  tone(Piezo, 1245, 100);
  setPoint = setPointA; // Empezamos Bien
}
void loop() {
  runner.execute();
  newPosition = (myEnc.read() + 1 ) / 4; // + 1 para que no quede al borde de los saltos de 4 en 4
  if (newPosition != oldPosition) { // Cambio la posicion del ENCODER
    if (!in) { // Si no estamos dentro (in) de una opcion: Variar entre opciones
      if (newPosition  > oldPosition) {
        tone(Piezo, 1661, 10);
        if (state % 10 < options[state / 10]) {
          state++;
        } else {
          state = (state / 10) * 10;
        }
      }
      if (newPosition  < oldPosition) {
        tone(Piezo, 1568, 10);
        if (state % 10 == 0) {
          state += options[state / 10];
        } else {
          state--;
        }
      }
    } else { // Opcion seleccionada
      switch (state) {
        case 21: // Cambia SetPointA Rojo
          if (newPosition  > oldPosition) {
            setPointA ++;
          } else {
            setPointA --;
          }
          setPointA = constrain(setPointA, 0, 31);
          break;
        case 22: // Cambia SetPointB Verde
          if (newPosition  > oldPosition) {
            setPointB ++;
          } else {
            setPointB --;
          }
          setPointB = constrain(setPointB, 0, 31);
          break;
        case 31:
          if (newPosition  > oldPosition) {
            reposo ++;
          } else {
            reposo --;
          }
          myservo.writeMicroseconds(reposo);
          break;
      }
    }
    oldPosition = newPosition;
  }
}
void pid() {
  measure = analogRead(sensorPin); // RAW data
  prevDist = dist; // Guardamos el valor anterior de dist para calcular la velocidad = dist; // Guardamos el valor anterior de dist para calcular la velocidad
  // Aplicamos curva de Calibracion de ADC a mm
  /* #  calPoint  calValue
        cm        ADC
    0  0.00  969.00
    1 8.00  545.00
    2 16.00 369.00
    3 23.00 297.00
    4 31.00 249.00
  */
  measure = constrain(measure, calValue[nCalPoints], calValue[0]);
  for (int i = 0; i < nCalPoints; i++) {
    if (measure >= calValue[i + 1] && measure < calValue[i]) {
      unfilteredDist = calPoint[i] + (measure - calValue[i]) * (calPoint[i + 1] - calPoint[i]) / (calValue[i + 1] - calValue[i]);
    }
  }
  dist = kdist * unfilteredDist + (1 - kdist) * dist; // Filtro Kalman
  // Neopixels con toque azul
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, azul);
  }
  if (skin) { // skin PUNTO
    // Blanco posicion Bola
    pixels.setPixelColor(dist, blanco);
    if (setPointAoB) { // Vamos al Rojo A
      if (setPointTime != 0) { // Si no es 0, hay verde
        pixels.setPixelColor(setPointB, verdeclaro);
      }
      pixels.setPixelColor(setPointA, rojo);
    } else {  // Vamos al Verde B
      pixels.setPixelColor(setPointA, rojoclaro);
      pixels.setPixelColor(setPointB, verde);
    }
  } else { // skin LASER
    if (setPointAoB) { // Vamos al Rojo A
      if (setPointA <= dist) { // Bola a la Dch del SetPoint
        for (int i = setPointA; i <= dist; i++) {
          pixels.setPixelColor(i, rojoclaro);
        }
      } else { // Bola a la Izq del SetPoint
        for (int i = dist; i <= setPointA ; i++) {
          pixels.setPixelColor(i, rojoclaro);
        }
      }
      if (setPointTime != 0) { // Si no es 0, hay verde
        pixels.setPixelColor(setPointB, verdeclaro);
      }
      pixels.setPixelColor(setPointA, rojo); // SetPointA Rojo Vivo
    } else { // Vamos al Verde B
      if (setPointB <= dist) { // Bola a la Dch del SetPoint
        for (int i = setPointB; i <= dist; i++) {
          pixels.setPixelColor(i, verdeclaro);
        }
      } else { // Bola a la Izq del SetPoint
        for (int i = dist; i <= setPointB ; i++) {
          pixels.setPixelColor(i, verdeclaro);
        }
      }
      pixels.setPixelColor(setPointA, rojoclaro);
      pixels.setPixelColor(setPointB, verde);  // SetPointB Verde Vivo
    }
  }
  pixels.show();
  // Calculo de la VELOCIDAD
  unfilteredVel = (dist - prevDist); // Ponemos un dato nuevo
  vel = kvel * unfilteredVel + (1 - kvel) * vel; // Filtro Kalman de Velocidad
  // Integral
  if (abs(error) >= Rint && abs(error) <= Rext) { // Solo si esta dentro de [-Rext,Rext] y fuera de [-Rint,Rint]
    I = I + error * Ki;
  }
  else {
    I = 0;
  }
  // A que setPoint vamos?
  if ((setPointTime != 0) && (millis() - pidBeginTime) > (setPointTime * 1000.00)) {  // ...alterna entre A y SetPointB cuando transcurre setPointTime
    pidBeginTime = millis();
    if (setPointAoB) {
      setPointAoB = false;
    } else {
      setPointAoB = true;
    }
  }
  if (setPointAoB) { // Vamos al A
    error = dist - setPointA;
  } else {           // Vamos al B
    error = dist - setPointB;
  }
  // Calculamos posicion del servo
  pos = Kp * error + Kd * vel + I;
  myservo.writeMicroseconds(reposo + pos);
  ciclos ++;
  if (ciclos > 60) { // Esperamos llenar el filtrado Kalman para que no nos arruine la grafica
    if (1) { // Enviamos tramas para Graficas de posicion y velocidad
      // 16.31  1.28   16.00
      // dist vel*10  SetPoint
      Serial.print(dist);
      Serial.print("\t");
      Serial.print(vel * 10); // Ampliamos para que no se vea tan pequeña
      Serial.print("\t");
      if (setPointAoB) { // Vamos al A
        Serial.println(setPointA);
      } else {           // Vamos al B
        Serial.println(setPointB);
      }
    }
    if (0) { // Enviamos tramas para Graficas de Componentes
      // 103.23 -254.35 0.00
      // Kp Kd  I
      Serial.print(Kp * error);
      Serial.print("\t");
      Serial.print(Kd * vel);
      Serial.print("\t");
      Serial.println(I);
    }
  }
}
void cursorOption(int options[], int siz, int selected) { // Pone el cursor en la opcion seleccionada
  // Inside the called function, the sizeof operator tells something about the argument,
  // so it tells you the size of the pointer, not the size of what it is pointing to. davekw7x
  // ... por eso uso matriz options[] con numero de opciones de cada pantalla

  for (int i = 0; i < siz / 2 ; i++) {
    lcd.setCursor(options[i] % 20, options[i] / 20);
    if ( i == selected) {
      lcd.print(c);
    } else {
      lcd.print(" ");
    }
  }
}
void lcdr() { // LCD Refresco
  if (in) { // Si estamos dentro: CursorFlecha
    c = B01111110;
  } else { // si no: CursorLleno
    c = B11111111;
  }
  switch (state / 10) { // PANTALLAS
    case 1: // Control PID
      lcd.setCursor(1, 0);
      lcd.print("=== CONTROL PID ===");
      lcd.setCursor(1, 1);
      lcd.print("kp=");
      lcd.print(Kp);
      lcd.setCursor(1, 2);
      lcd.print("kd=");
      lcd.print(Kd);
      lcd.setCursor(1, 3);
      lcd.print("ki=");
      lcd.print(Ki);
      lcd.setCursor(13, 3);
      lcd.print("Ri");
      lcd.print(Rint);
      lcd.setCursor(17, 3);
      lcd.print("Re");
      lcd.print(Rext);
      cursorOption (cursorPID, sizeof(cursorPID), state % 10);
      break;
    case 2: // SetPoints
      lcd.setCursor(1, 0);
      lcd.print("==== SETPOINTS ====");
      lcd.setCursor(1, 1);
      lcd.print("Rojo: ");
      lcd.print(int(setPointA));
      lcd.print(" ");
      lcd.setCursor(11, 1);
      lcd.print("Verde: ");
      lcd.print(int(setPointB));
      if (setPointB < 10) { // Si tiene 2 digitos saltamos a la otra linea con " " y parpadea
        lcd.print(" ");
      }
      lcd.setCursor(1, 2);
      lcd.print("Tiempo: ");
      lcd.print(setPointTime);
      lcd.print("s ");
      lcd.setCursor(1, 3);
      lcd.print("Skin: ");
      if (skin) {
        lcd.print("Punto");
      } else {
        lcd.print("Laser");
      }
      cursorOption (cursorSetP, sizeof(cursorSetP), state % 10);
      break;
    case 3: // Calibracion Horizontal
      lcd.setCursor(1, 0);
      lcd.print("=== NIVELACION ====");
      lcd.setCursor(8, 2);
      lcd.print(reposo);
      cursorOption (cursorNiv, sizeof(cursorNiv), state % 10);
      break;
    case 4: // Calibracion Sensor
      lcd.setCursor(1, 0);
      lcd.print("=== CALIBRACION ===");
      lcd.print(" ===   SENSOR    ===");
      lcd.setCursor(6, 3);
      lcd.print("<Empezar>");
      cursorOption (cursorCal, sizeof(cursorCal), state % 10);
      break;
  }
}
void button() {
  buttonState = digitalRead(pinButton);
  if (buttonState != buttonState0) {
    if (buttonState == LOW) { // Pulsado
      tone(Piezo, 1661, 200);
      buttonTime = millis(); // Veremos pulsacion larga
      if (state % 10 == 0) { // Estados X0: cambiamos de pantalla
        switch (state) {
          case 10:
            state = 20;
            lcd.clear();
            break;
          case 20:
            state = 10;
            lcd.clear();
            break;
          case 30:
            state = 40;
            apaga();
            lcd.clear();
            break;
          case 40:
            tone(Piezo, 1661, 100);
            delay(100);
            tone(Piezo, 1865, 100);
            delay(100);
            tone(Piezo, 1760, 300);
            t1.enable();
            state = 10;
            lcd.clear();
            break;
        }
      } else { // Estados X!0: hacemos cosas
        if (in) { // Salimos de seleccion
          switch (state) {
            case 31:
              EEPROM.put(16, float(reposo));
              state = 40;
              apaga();
              break;
          }
          in = false;
        } else {// Entramos en seleccion
          in = true;
          switch (state) {
            case 24:
              skin = !skin ;
              in = false;
              break;
          }
        }
        switch (state) {
          case 31:
            myservo.writeMicroseconds(reposo);
            break;
          case 41:
            myservo.writeMicroseconds(reposo - barrainclinada);
            calibra();
            in = false;
            break;
        }
      }
    } else { // Soltado
      if (millis() - buttonTime > 1000 && state % 10 == 0) {
        tone(Piezo, 1661, 100);
        delay(100);
        tone(Piezo, 1865, 100);
        delay(100);
        tone(Piezo, 1760, 300);
        state = 30;
        myservo.writeMicroseconds(reposo); // Ponemos la barra en reposo para comprobar ya
        t1.disable(); // Detiene funcionamiento PID.
        apaga();
        for (int i = 25; i < 30; i++) { // Pixeles Verdes sobre el nivel
          pixels.setPixelColor(i, pixels.Color(0, 1, 0));
        }
        pixels.show();
      } else {
        switch (state) {
          case 11:
            myservo.writeMicroseconds(reposo - barrainclinada);
            Kp = input(24, 4);
            break;
          case 12:
            myservo.writeMicroseconds(reposo - barrainclinada);
            Kd = input(44, 4);
            break;
          case 13:
            myservo.writeMicroseconds(reposo - barrainclinada);
            Ki = input(64, 4);
            break;
          case 14:
            myservo.writeMicroseconds(reposo - barrainclinada);
            Rint = input(75, 1);
            break;
          case 15:
            myservo.writeMicroseconds(reposo - barrainclinada);
            Rext = input(79, 1);
            break;
          case 23: // Cambia tiempo entre
            setPointTime = input(49, 4);
            if (setPointTime == 0) {
              setPointAoB = true;
            }
            break;
        }
      }
      lcd.clear();
    }
    buttonState0 = buttonState;
  }
}
void calibra () {
  while (digitalRead(pinButton) == LOW) {
    // Esperamos a que suelte el boton porque No queremos saltarnos el primer punto de calibracion
  }
  lcd.clear();
  for (int point = 0; point < nCalPoints ; point++) { // /2 xq un int ocupa 2 bytes
    pixels.clear();
    pixels.setPixelColor(calPoint[point], blanco);
    pixels.show();
    while (digitalRead(pinButton) == HIGH) {
      // Espero que coloquen la bola y aprieta el boton
      lcd.setCursor(0, 0);
      lcd.print("Punto #");
      lcd.print(point);
      lcd.setCursor(0, 1);
      lcd.print("Valor Anterior: ");
      lcd.print(calValue[point]);
      lcd.setCursor(0, 2);
      lcd.print("Valor Nuevo   : ");
      lcd.print(analogRead(A0));
    }
    // Tomamos n lecturas, las promediamos => calRead
    int n = 100; // Nº de lecturas tomadas para calcular calRead
    long calRead = 0;
    for (int i = 0; i < n; i++) {
      int reading = analogRead(A0);
      calRead += reading;
    }
    calRead = calRead / n;
    //Almacenamos en la EEPROM a partir de la posicion 16
    EEPROM.put(20 + point * 4, float(calRead));
    calValue[point] = calRead;
    while (digitalRead(pinButton) == LOW) {
      // Espero que suelte el boton
    }
  }
  state = 40;
}
void apaga() { // Pixeles
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
  }
  pixels.show();
}
float input(int pos, int digitos) {
  apaga();
  float value;
  String text = "";
  char key;
  boolean comma = false; // Solo una coma decimal por numero
  boolean writting = true;
  int place = 0; // Dígito 01234

  lcdr(); // Para poner flecha
  lcd.setCursor(pos % 20, pos / 20);
  if (digitos == 1) { // Para que no chafe en Ri Re
    lcd.print(" ");
  } else {
    lcd.print("       ");
  }
  while (writting) {
    lcd.setCursor(pos % 20 , pos / 20);
    lcd.print(text);
    char key = keypad.getKey();
    if (key) {
      tone(Piezo, 740, 10);
      switch  (key) {
        case '*': // .
          if (!comma) {
            comma = true;
            text = text + '.';
            place ++;
          }
          break;
        case '#': // del
          if (place > 0) {
            text.remove(text.length() - 1, 1);
            place = place - 1;
            lcd.setCursor(pos % 20 , pos / 20);
            lcd.print(text);
            lcd.print(" ");
          }
          break;
        case 'D': // Enter
          if (place != 0) {
            value = text.toFloat();
          } else { // Dejamos el valor que había
            switch (pos) {
              case 24:
                value = Kp;
                break;
              case 44:
                value = Kd;
                break;
              case 64:
                value = Ki;
                break;
              case 75:
                value = Rint;
                break;
              case 79:
                value = Rext;
                break;
              case 49:
                value = setPointTime;
                break;
            }
          }
          writting = false;
          break;
        case 'A':
          break;
        case 'B':
          break;
        case 'C':
          break;
        default:
          text = text + key;
          place ++;
          break;
      }
    }
    if (place == digitos) { // alcanzado numero maximo de digitos, salimos.
      writting = false;
      value = text.toFloat();
    }
  }
  in = false;
  return value;
}
