#include <LiquidCrystal_I2C.h>
#include <Servo.h>


//LDRs
const int LDR_TOP_LEFT = A0;
const int LDR_TOP_RIGHT = A1;
const int LDR_BOTTOM_LEFT = A2;
const int LDR_BOTTOM_RIGHT = A3;
const int LIGHT_THRESHOLD = 10;

//Servos
Servo servo_horizontal;
Servo servo_vertical;
int pos_sh = 90;
int pos_sv = 90;
const int UPPER_LIMIT_POS = 160;  //Límite superior de los servos
const int LOWER_LIMIT_POS = 20;   //Límite inferior de los servos

//Sensor de corriente ACS712 5A
const int CURRENT_SENSOR = A6;
const float SENSIBILITY = 0.185;
const int CURRENT_SAMPLES = 10;

unsigned long lastTime = 0;
unsigned long threshold = 2000;

//Pantalla LCD con I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);  //LCD de 16 columnas y 2 filas


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(500);
  lcd.init();
  delay(2000);
  lcd.backlight();
  servo_vertical.attach(5);
  servo_horizontal.attach(6);
  servo_vertical.write(90);
  servo_horizontal.write(90);
  pos_sv = servo_vertical.read();
  pos_sh = servo_horizontal.read();
}

void loop() {
  //Leemos los 4 LDRs
  int ldr_tl_value = analogRead(LDR_TOP_LEFT);
  int ldr_tr_value = analogRead(LDR_TOP_RIGHT);
  int ldr_bl_value = analogRead(LDR_BOTTOM_LEFT);
  int ldr_br_value = analogRead(LDR_BOTTOM_RIGHT);

  int average_top = (ldr_tl_value + ldr_tr_value) / 2; //Media de los 2 LDR de arriba
  int average_bottom = (ldr_bl_value + ldr_br_value) / 2; //Media de los 2 LDR de abajo
  int average_left = (ldr_tl_value + ldr_bl_value) / 2; //Media de los 2 LDR de la izquierda
  int average_right = (ldr_tr_value + ldr_br_value) / 2; //Media de los 2 LDR de la derecha

  //Movemos el solar tracker
  moveSolarTracker(average_top, average_bottom, average_left, average_right);

  //Medimos la corriente
  if ((millis() - lastTime) > threshold) {
    lastTime = millis();
    float current_measured = medirCorriente(CURRENT_SENSOR, SENSIBILITY, CURRENT_SAMPLES);
    //Mostramos la corriente en el LCD
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("Corriente:");
    lcd.setCursor(2, 1);
    Serial.print(current_measured);
    lcd.print(String(current_measured));
    lcd.print("A");
  }



  //Delay de 30 ms para que los servos no se muevan demasiado rápido
  delay(30);
}

void moveSolarTracker(int average_top, int average_bottom, int average_left, int average_right) {
  //Movemos el solar tracker hacia arriba o hacia abajo
  if ((average_top - average_bottom) > LIGHT_THRESHOLD && pos_sv < UPPER_LIMIT_POS) {
    pos_sv++;
    servo_vertical.write(pos_sv);
  }
  else if ((average_bottom - average_top) > LIGHT_THRESHOLD && pos_sv > LOWER_LIMIT_POS) {
    pos_sv--;
    servo_vertical.write(pos_sv);
  }

  //Movemos el solar tracker hacia la derecha o hacia la izquierda
  if ((average_left - average_right) > LIGHT_THRESHOLD && pos_sh < UPPER_LIMIT_POS) {
    pos_sh++;
    servo_horizontal.write(pos_sh);
  }
  else if ((average_right - average_left) > LIGHT_THRESHOLD && pos_sh > LOWER_LIMIT_POS) {
    pos_sh--;
    servo_horizontal.write(pos_sh);
  }
}

float medirCorriente(int current_sensor, float sensibility, int samples) {
  float sensor_read = 0;
  float current_measured = 0;
  for (int i = 0; i < samples; i++) {
    sensor_read += analogRead(CURRENT_SENSOR);
  }

  sensor_read = sensor_read / samples;
  sensor_read = sensor_read * (5.0 / 1023.0);
  current_measured = (sensor_read - 2.5) / sensibility;
  Serial.println(sensor_read);

  return current_measured;
}
