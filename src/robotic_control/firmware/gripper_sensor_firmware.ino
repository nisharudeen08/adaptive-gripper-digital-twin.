/*
 * FSR Wiring:
 *   FSR Pin 1 → Arduino 5V
 *   FSR Pin 2 → Arduino A0 AND 10kΩ resistor to GND
 * 
 *       5V ──┬── FSR ──┬── A0
 *            │         │
 *           GND      10kΩ
 *                      │
 *                     GND
 */
#define CALIBRATE 0
#define CALIBRATION_FACTOR 20.0

unsigned long last_time = 0;

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  Serial.println("{\"status\":\"READY\",\"version\":\"1.0\"}");
}

void loop() {
  unsigned long now = millis();
  if (now - last_time >= 10) { // 100Hz
    last_time = now;
    
    int fsr_raw = analogRead(A0);
    int pos_raw = analogRead(A1);
    
    float voltage = fsr_raw * (5.0 / 1023.0);
    float resistance = (10000.0 * (5.0 - voltage)) / max(voltage, 0.001);
    float conductance = 1.0 / resistance;
    float force = conductance * CALIBRATION_FACTOR;
    
    float pos_rad = (pos_raw / 1023.0) * PI; // 0 to 180 deg
    
    if (CALIBRATE) {
      Serial.print("ADC: "); Serial.println(fsr_raw);
      delay(500);
      return;
    }
    
    Serial.print("{\"fsr\":");
    Serial.print(force);
    Serial.print(",\"pos\":");
    Serial.print(pos_rad);
    Serial.print(",\"ts\":");
    Serial.print(now);
    Serial.println(",\"mode\":\"HW\"}");
    
    digitalWrite(13, !digitalRead(13)); // Blink
  }
}
