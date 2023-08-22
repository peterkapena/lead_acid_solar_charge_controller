#include <LiquidCrystal.h>
#define BAT_CHARGE_CONTROL_PIN 3
#define LOAD_CONTROL_PIN 4
#define MAX_SOLAR_VOLT_DIVIDER 22
#define MAX_BAT_VOLT_DIVIDER 15.6
#define VOLT_READING_SAMPLES 200
#define LCD_OUTPUT_DELAY 10
#define SERIAL_OUTPUT_DELAY 70
#define MIN_BAT_VOLT 5

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

float solar_volt = 0;    //  solar panel voltage
float battery_volt = 0;  //  battery voltage
float pwm_value = 0;     // pwm out put to npn base

unsigned long previous_millis_lcd = 0;
bool lcd_shifted_left = false;
unsigned long previous_millis_voltage_serial = 0;

void setup() {
  TCCR2B = (TCCR2B & 0b11111000) | 0x03;
  Serial.begin(9600);

  pinMode(LOAD_CONTROL_PIN, OUTPUT);
  pinMode(BAT_CHARGE_CONTROL_PIN, OUTPUT);

  digitalWrite(BAT_CHARGE_CONTROL_PIN, LOW);
  digitalWrite(LOAD_CONTROL_PIN, LOW);

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
  if (solar_volt > 10 && battery_volt > 7) {
    digitalWrite(LOAD_CONTROL_PIN, HIGH);
  } else if (battery_volt > MIN_BAT_VOLT + 2) {
    digitalWrite(LOAD_CONTROL_PIN, HIGH);
  } else
    digitalWrite(LOAD_CONTROL_PIN, LOW);
}

void chargeIndicators() {
  unsigned long current_millis = millis();
  if (current_millis - previous_millis_lcd >= LCD_OUTPUT_DELAY) {
    previous_millis_lcd = current_millis;
    if (lcd_shifted_left)
      lcd.scrollDisplayRight();
    else
      lcd.scrollDisplayLeft();
    lcd_shifted_left = !lcd_shifted_left;
    lcd.setCursor(0, 1);
    float charged_percent = battery_volt * 100 / 14;
    if (charged_percent >= 90)
      charged_percent = 100;
    lcd.print(String(" ") + battery_volt + String("V ") + charged_percent + "%");
  }
}

void chargeControl() {
  if (solar_volt > battery_volt) {
    if (battery_volt < 5) {
      pwm_value = 60;
    } else if ((battery_volt > 5) && (battery_volt <= 7)) {
      pwm_value = 90;
    } else if ((battery_volt > 7) && (battery_volt <= 12)) {
      pwm_value = 170;
    } else if ((battery_volt > 12) && (battery_volt <= 13)) {
      pwm_value = 180;
    } else if ((battery_volt > 13) && (battery_volt <= 14)) {
      pwm_value = 150;
    } else if (battery_volt >= 14.2) {
      pwm_value = 90;
    }
  }

  if ((battery_volt >= 14.4) or (solar_volt < battery_volt)) {
    pwm_value = 0;
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
  }
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  if (fromHigh == fromLow) return toLow;
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}
