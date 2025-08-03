# mpu6050
# MPU6050 Interface with no libraries on Arduino

This code manually toggles the signals and sends bit by bit data during transmissions.

## Wiring

The MPU6050 sensor is connected to the Arduino as follows:

VCC = 3.3V<br>
GND = GND<br>
SDA = A 4<br>
SCL = A 5<br>
AD0 = GND (ADDRESS IS 0x68, IF SET HIGH THEN ADDRESS CHANGES TO 0x69)<br>

## Logic used 

The code initializes the MPU6050 using direct I2C register writes and reads 14 bytes of data from the sensor, including:
- 6 bytes of accelerometer data
- 2 bytes of temperature data (ignored in this code)
- 6 bytes of gyroscope data

The retrieved raw data is converted into signed 16-bit integers and printed to the serial monitor with appropriate labels.

## Code Structure and Explanation

 **Pin Definitions**

```cpp 
#define SDA_PIN A4
#define SCL_PIN A5
#define MPU_ADDR 0x68
  ```
the pin definitions
```cpp
void toggle() {
  delayMicroseconds(5);      //adds delay between toggling od signals, our frequency is 50khz in this code
}
  ```
this adds the delay between manual toggle of the signal, we use 50khz as 100khz is the max freq in fstandrad i2c
 ```cpp
void initiate() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);      //set the i2c idle state with both lines set to high
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
}
  ```
initiate transfer by activating i2c line
 ```cpp
void start() {
  digitalWrite(SDA_PIN, HIGH); toggle();
  digitalWrite(SCL_PIN, HIGH); toggle();     //as defined in datasheet, start signal sent when SDA goes high to low when SCL is high
  digitalWrite(SDA_PIN, LOW);  toggle();
  digitalWrite(SCL_PIN, LOW);  toggle();
}
```
as per datasheet, start signal sent when SDA goes high to low when SCL is high
```cpp
void stop() {
  digitalWrite(SDA_PIN, LOW);  toggle();
  digitalWrite(SCL_PIN, HIGH); toggle();    //as defined in datasheet, stop signal sent when SDA goes low to high when SCL is high
  digitalWrite(SDA_PIN, HIGH); toggle();
}
```
as defined in datasheet, stop signal sent when SDA goes low to high when SCL is high
```cpp
bool write(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDA_PIN, (data >> i) & 1); toggle();  //this line sends bit by bit the write data
    digitalWrite(SCL_PIN, HIGH); toggle();
    digitalWrite(SCL_PIN, LOW);  toggle();
  }
  pinMode(SDA_PIN, INPUT);                          //this line sets the sda pin to recieve the ACK
  digitalWrite(SCL_PIN, HIGH); toggle();
  bool ack = (digitalRead(SDA_PIN) == 0);
  digitalWrite(SCL_PIN, LOW);  toggle();
  pinMode(SDA_PIN, OUTPUT);
  return ack;
}
```
the write sequence is done by bit by bit data transfer and signal toggling, and recieving ACK at the end of each byte transfer
```cpp
uint8_t read(bool ack) {
  uint8_t data = 0;
  pinMode(SDA_PIN, INPUT);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCL_PIN, HIGH); toggle();
    data |= (digitalRead(SDA_PIN) << i);       //this line reads the incoming bits bit by bit on the SDA pin
    digitalWrite(SCL_PIN, LOW); toggle();
  }
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, ack ? LOW : HIGH); toggle();   //this line sends the ack/nack 
  digitalWrite(SCL_PIN, HIGH); toggle();
  digitalWrite(SCL_PIN, LOW); toggle();
  return data;
}
```
the read is using bit by bit read on the sda pin and writes the bits into buffer called data
```cpp
void mpu6050_write_register(uint8_t reg, uint8_t value) {
  start();
  write((MPU_ADDR << 1) | 0);                          //write operation occurs when i2c address is sent with 8th bit set to 0 
  write(reg);                                          //send the register address
  write(value);                                        //send the value to be written
  stop();
}
```
### Write Operation:
1. Start condition
2. Send 7-bit I2C address with write bit (0)
3. Send register address
4. Send data byte
5. Stop condition
```cpp
void mpu6050_read_bytes(uint8_t reg, uint8_t* buffer, int length) {
  start();
  write((MPU_ADDR << 1) | 0);                                  //write must occur before read
  write(reg);
  start();
  write((MPU_ADDR << 1) | 1);                                //read signal sent with i2c address 8th bit set to 1
  for (int i = 0; i < length; i++) {
    buffer[i] = read(i < (length - 1));
  }
  stop();
}
```
### Read Operation:
1. Start condition
2. Send 7-bit I2C address with write bit (0)
3. Send register address
4. Repeated start
5. Send 7-bit I2C address with read bit (1)
6. Read data byte(s)
7. NACK after the final byte
8. Stop condition

```cpp
int16_t to_int16(uint8_t high, uint8_t low) {
  return (int16_t)((high << 8) | low);            //this line adds the MSB and LSB together into a 16bit 
}
```
this function combines the 8bits of LSB and MSB and adds the MSB and LSB together into a 16bit
```cpp
void setup() {
  Serial.begin(9600);
  initiate();
  mpu6050_write_register(0x6B, 0x00); // Wakes up the MPU6050    
  delay(100);
}
```
```cpp
void loop() {
  uint8_t data[14];                       //the buffer into which the recieved data will be written during read operation
  mpu6050_read_bytes(0x3B, data, 14);
```
the function called here starts reading 14 consecutive 8 bit registers from 0X3B register address, and writes recieved data <br>
into buffer called data.
```cpp
  int16_t ax = to_int16(data[0], data[1]);    //Ox3B holds accelerometer x value MSB , 0x3C holds LSB
  int16_t ay = to_int16(data[2], data[3]);    //Ox3D holds accelerometer y value MSB , 0x3E holds LSB 
  int16_t az = to_int16(data[4], data[5]);    //Ox3F holds accelerometer z value MSB , 0x40 holds LSB
```
combiining of LSB and MSB for each coordinate value
```cpp
  int16_t gx = to_int16(data[8], data[9]);     //Ox43 holds gyro x value MSB , 0x44 holds LSB
  int16_t gy = to_int16(data[10], data[11]);     //Ox45 holds gyro y value MSB , 0x46 holds LSB
  int16_t gz = to_int16(data[12], data[13]);     //Ox47 holds gyro z value MSB , 0x48 holds LSB 
```
combiining of LSB and MSB for each coordinate value
```cpp
  Serial.print("Ax: "); Serial.print(ax); Serial.print("  ");
  Serial.print("Ay: "); Serial.print(ay); Serial.print("  ");
  Serial.print("Az: "); Serial.print(az); Serial.print("  ");

  Serial.print("Gx: "); Serial.print(gx); Serial.print("  ");
  Serial.print("Gy: "); Serial.print(gy); Serial.print("  ");
  Serial.print("Gz: "); Serial.println(gz);

  delay(500);
}
```

The data is read from register address 0x3B onwards, covering the accelerometer (6 bytes), temperature (2 bytes, not used), and gyroscope (6 bytes), totaling 14 bytes per cycle.
Every send function waits for ack compulsorily.

## Output
Example:
The serial monitor will display raw signed 16-bit values for each axis:
Ax: -123 Ay: 456 Az: 16384 Gx: 12 Gy: -15 Gz: 6

