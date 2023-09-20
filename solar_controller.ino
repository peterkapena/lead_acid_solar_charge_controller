#include <LiquidCrystal.h>

#define BAT_CHARGE_CONTROL_PIN 3
#define SOLAR_LOAD_CONTROL_PIN 4
#define BATTERY_LOAD_CONTROL_PIN 5

#define MAX_SOLAR_VOLT_DIVIDER 22
#define MAX_BAT_VOLT_DIVIDER 15.6
#define MAX_BAT_VOLT_THRESHOLD 14.4
#define MIN_BAT_VOLT 5

#define VOLT_READING_SAMPLES 1000

#define SERIAL_OUTPUT_DELAY 500

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

float solar_volt = 0;    //  solar panel voltage
float battery_volt = 0;  //  battery voltage
float pwm_value = 0;     // pwm out put to npn base

unsigned long previous_millis_lcd = 0;
unsigned long previous_millis_voltage_serial = 0;

void setup() {
  TCCR2B = (TCCR2B & 0b11111000) | 0x05;
  Serial.begin(9600);

  for (uint8_t i = 3; i < 6; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  lcd.begin(16, 2);
  lcd.print("LUMUMBA 217016375");
}

void loop() {
  senseVoltage();
  chargeControl();
  chargeIndicators();
  loadControl();
}

void loadControl() {
  if (solar_volt > 10) {
    digitalWrite(BATTERY_LOAD_CONTROL_PIN, LOW);
    digitalWrite(SOLAR_LOAD_CONTROL_PIN, HIGH);
  } else if (battery_volt > MIN_BAT_VOLT) {
    digitalWrite(SOLAR_LOAD_CONTROL_PIN, LOW);
    digitalWrite(BATTERY_LOAD_CONTROL_PIN, HIGH);
  } else {
    digitalWrite(SOLAR_LOAD_CONTROL_PIN, LOW);
    digitalWrite(BATTERY_LOAD_CONTROL_PIN, LOW);
  }

  //Just for safety reasons in case it happens
  if (digitalRead(SOLAR_LOAD_CONTROL_PIN) && digitalRead(BATTERY_LOAD_CONTROL_PIN)) {
    digitalWrite(SOLAR_LOAD_CONTROL_PIN, LOW);
    digitalWrite(BATTERY_LOAD_CONTROL_PIN, LOW);
  }
}

void chargeIndicators() {
  lcd.setCursor(0, 1);
  float charged_percent = battery_volt * 100 / MAX_BAT_VOLT_THRESHOLD;
  float battery_indication = battery_volt;


  if (battery_indication >= 14.1) {
    charged_percent = 100;
    battery_indication = 14.4;
  }
  char* battery_str = toOnePrecision(battery_indication);
  char* charged_percent_str = toOnePrecision(charged_percent);


  lcd.print(" " + String(battery_str) + "V " + String(charged_percent_str) + "%");

  // Free the dynamically allocated memory
  free(battery_str);
  free(charged_percent_str);
}

char* toOnePrecision(float value) {
  char* str = (char*)malloc(10);  // Allocate memory for the string
  dtostrf(value, 4, 1, str);
  return str;
}

void chargeControl() {
  if (solar_volt > battery_volt) {
    if (battery_volt < 5) {
      pwm_value = 250;  // Charge at maximum PWM when battery voltage is very low
    } else if (battery_volt > 14.4) {
      pwm_value = 0;  // Stop charging when battery voltage exceeds 14.4V
    } else if (battery_volt >= 14.2 && battery_volt <= 14.4) {
      pwm_value = 200;  // Reduce charging to a lower PWM when battery voltage is in the float range (14.2V to 14.4V)
    } else {
      pwm_value = 250;  // Charge at maximum PWM for other voltage ranges
    }
  } else {
    pwm_value = 0;  // Stop charging if solar voltage is less than battery voltage
  }

  analogWrite(BAT_CHARGE_CONTROL_PIN, pwm_value);
}

void senseVoltage() {
  float solar_sample = 0;
  float bat_sample = 0;
  for (int i = 0; i < VOLT_READING_SAMPLES; i++) {
    bat_sample += analogRead(A0);
    solar_sample += analogRead(A1);
  }

  bat_sample /= VOLT_READING_SAMPLES;
  solar_sample /= VOLT_READING_SAMPLES;

  solar_volt = mapFloat(solar_sample, 0, 1023, 0, MAX_SOLAR_VOLT_DIVIDER);
  battery_volt = mapFloat(bat_sample, 0, 1023, 0, MAX_BAT_VOLT_DIVIDER);
  unsigned long currentMillis = millis();

  if (currentMillis - previous_millis_voltage_serial >= SERIAL_OUTPUT_DELAY) {
    previous_millis_voltage_serial = currentMillis;
    Serial.println();
    Serial.print("solar input voltage :");
    Serial.println(solar_volt);
    Serial.print("battery voltage :");
    Serial.println(battery_volt);
    Serial.print("pwm duty cycle is : ");
    Serial.println(pwm_value);
    Serial.print("battery loaded : ");
    Serial.println(digitalRead(BATTERY_LOAD_CONTROL_PIN));
    Serial.print("solar loaded : ");
    Serial.println(digitalRead(SOLAR_LOAD_CONTROL_PIN));
  }
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  if (fromHigh == fromLow) return toLow;
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}
