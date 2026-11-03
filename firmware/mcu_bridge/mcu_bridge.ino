// mcu_bridge — jembatan Jetson <-> servo kemudi + ESC, tombol START, sensor E-STOP.
// Papan: Arduino Nano (ATmega328P) atau kompatibel. Build: arduino-cli compile -b arduino:avr:nano
//
// Protokol 115200 baud, baris teks:
//   Jetson -> MCU : "C <servo_us> <esc_us>\n"   (1000..2000 us)
//   MCU -> Jetson : "B START"  tombol start ditekan
//                   "E 1"/"E 0" e-stop ditekan/dilepas (juga dikirim ulang tiap 500 ms)
// Keselamatan:
//   - Watchdog: tanpa perintah > 300 ms -> ESC netral (Jetson hang / kabel USB lepas = mobil berhenti).
//   - E-stop: sakelar jamur NC memutus daya ESC SECARA FISIK; kontak bantu ke PIN_ESTOP hanya memberi tahu.
//   - Saat boot ESC diberi netral 2 detik (arming).
#include <Servo.h>

const uint8_t PIN_SERVO = 9;
const uint8_t PIN_ESC = 10;
const uint8_t PIN_START = 2;   // tombol NO ke GND (pull-up internal)
const uint8_t PIN_ESTOP = 3;   // kontak bantu e-stop ke GND saat DITEKAN (pull-up internal)
const uint8_t PIN_LED = 13;
const uint16_t NEUTRAL_US = 1500;
const uint16_t SERVO_CENTER_US = 1500;  // samakan dengan servo_center_us di config/robot.yaml
const unsigned long WATCHDOG_MS = 300;

Servo servo, esc;
char buf[32];
uint8_t len = 0;
unsigned long lastCmd = 0, lastReport = 0, lastStartEdge = 0;
uint16_t servoUs = SERVO_CENTER_US, escUs = NEUTRAL_US;
bool estop = false, startPrev = HIGH;

uint16_t clampUs(long v) { return v < 1000 ? 1000 : (v > 2000 ? 2000 : v); }

void handleLine() {
  buf[len] = 0;
  if (buf[0] == 'C') {
    char *p = buf + 1;
    long s = strtol(p, &p, 10);
    long e = strtol(p, &p, 10);
    if (s > 0 && e > 0) {
      servoUs = clampUs(s);
      escUs = clampUs(e);
      lastCmd = millis();
    }
  }
  len = 0;
}

void setup() {
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  servo.attach(PIN_SERVO);
  esc.attach(PIN_ESC);
  servo.writeMicroseconds(SERVO_CENTER_US);
  esc.writeMicroseconds(NEUTRAL_US);
  Serial.begin(115200);
  delay(2000);  // arming ESC
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') handleLine();
    else if (c != '\r' && len < sizeof(buf) - 1) buf[len++] = c;
    else if (len >= sizeof(buf) - 1) len = 0;  // baris rusak: buang
  }

  unsigned long now = millis();
  bool es = digitalRead(PIN_ESTOP) == LOW;
  if (es != estop || now - lastReport > 500) {
    estop = es;
    lastReport = now;
    Serial.print("E ");
    Serial.println(estop ? 1 : 0);
  }

  bool st = digitalRead(PIN_START);
  if (startPrev == HIGH && st == LOW && now - lastStartEdge > 300) {  // debounce + anti dobel tekan
    lastStartEdge = now;
    Serial.println("B START");
  }
  startPrev = st;

  bool alive = now - lastCmd < WATCHDOG_MS;
  esc.writeMicroseconds((alive && !estop) ? escUs : NEUTRAL_US);
  servo.writeMicroseconds(alive ? servoUs : SERVO_CENTER_US);
  digitalWrite(PIN_LED, estop ? (now / 100) % 2 : (alive ? HIGH : (now / 500) % 2));
}
