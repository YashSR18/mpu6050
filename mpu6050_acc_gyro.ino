#define SDA_PIN A4
#define SCL_PIN A5
#define MPU_ADDR 0x68

void toggle() {
  delayMicroseconds(5);
}

void initiate() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
}

void start() {
  digitalWrite(SDA_PIN, HIGH); toggle();
  digitalWrite(SCL_PIN, HIGH); toggle();
  digitalWrite(SDA_PIN, LOW);  toggle();
  digitalWrite(SCL_PIN, LOW);  toggle();
}

void stop() {
  digitalWrite(SDA_PIN, LOW);  toggle();
  digitalWrite(SCL_PIN, HIGH); toggle();
  digitalWrite(SDA_PIN, HIGH); toggle();
}

bool write(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDA_PIN, (data >> i) & 1); toggle();
    digitalWrite(SCL_PIN, HIGH); toggle();
    digitalWrite(SCL_PIN, LOW);  toggle();
  }
  pinMode(SDA_PIN, INPUT);
  digitalWrite(SCL_PIN, HIGH); toggle();
  bool ack = (digitalRead(SDA_PIN) == 0);
  digitalWrite(SCL_PIN, LOW);  toggle();
  pinMode(SDA_PIN, OUTPUT);
  return ack;
}

uint8_t read(bool ack) {
  uint8_t data = 0;
  pinMode(SDA_PIN, INPUT);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCL_PIN, HIGH); toggle();
    data |= (digitalRead(SDA_PIN) << i);
    digitalWrite(SCL_PIN, LOW); toggle();
  }
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, ack ? LOW : HIGH); toggle();
  digitalWrite(SCL_PIN, HIGH); toggle();
  digitalWrite(SCL_PIN, LOW); toggle();
  return data;
}

void mpu6050_write_register(uint8_t reg, uint8_t value) {
  start();
  write((MPU_ADDR << 1) | 0);
  write(reg);
  write(value);
  stop();
}

void mpu6050_read_bytes(uint8_t reg, uint8_t* buffer, int length) {
  start();
  write((MPU_ADDR << 1) | 0);
  write(reg);
  start();
  write((MPU_ADDR << 1) | 1);
  for (int i = 0; i < length; i++) {
    buffer[i] = read(i < (length - 1));
  }
  stop();
}

int16_t to_int16(uint8_t high, uint8_t low) {
  return (int16_t)((high << 8) | low);
}

void setup() {
  Serial.begin(9600);
  initiate();
  mpu6050_write_register(0x6B, 0x00); // Wake up MPU6050
  delay(100);
}

void loop() {
  uint8_t data[14];
  mpu6050_read_bytes(0x3B, data, 14);

  int16_t ax = to_int16(data[0], data[1]);
  int16_t ay = to_int16(data[2], data[3]);
  int16_t az = to_int16(data[4], data[5]);

  int16_t gx = to_int16(data[8], data[9]);
  int16_t gy = to_int16(data[10], data[11]);
  int16_t gz = to_int16(data[12], data[13]);

  Serial.print("Ax: "); Serial.print(ax); Serial.print("  ");
  Serial.print("Ay: "); Serial.print(ay); Serial.print("  ");
  Serial.print("Az: "); Serial.print(az); Serial.print("  ");

  Serial.print("Gx: "); Serial.print(gx); Serial.print("  ");
  Serial.print("Gy: "); Serial.print(gy); Serial.print("  ");
  Serial.print("Gz: "); Serial.println(gz);

  delay(500);
}
