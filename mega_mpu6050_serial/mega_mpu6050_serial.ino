#include "Wire.h"

#define MPU_ADDR 0x68 // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
#define PIN_CAM_TRG 7
#define PIN_IMU_INT 2

byte stringChecksum(volatile char* s, int len) {
  byte c = 0;
  for(int i = 0; i< len; i++) c ^= s[i];
  return c;
};

volatile byte flag_imuready = LOW;
volatile byte flag_trigger  = 0;
void imuISR(){ flag_imuready  = HIGH; };

typedef union USHORT_UNION_{
    uint16_t ushort_;
    uint8_t bytes_[2];
} USHORT_UNION;

typedef union UINT_UNION_{
    uint32_t uint_;
    uint8_t bytes_[4];
} UINT_UNION;


int16_t ax_raw, ay_raw, az_raw; // variables for accelerometer raw data
int16_t gx_raw, gy_raw, gz_raw; // variables for gyro raw data
int16_t temp_raw; // variables for temp_raw data
USHORT_UNION u_ax, u_ay, u_az;
USHORT_UNION u_gx, u_gy, u_gz;
USHORT_UNION u_temp;


uint32_t time_curr;
uint32_t time_prev;
USHORT_UNION tsec;
UINT_UNION   tusec;

uint32_t dt = 0;

uint8_t  cnt     = 0;
uint32_t cnt_imu = 0;
uint32_t cnt_cam = 0;

char buf[64];

void setup() {
  // Pin settings
  pinMode(PIN_CAM_TRG, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuISR, RISING);
  
  Serial.begin(460800);
  
  Wire.begin();
  Wire.setClock(400000L); // 400 kHz I2C clock
  
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Sample rate divider
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);
  Wire.write(0x27); // 200 Hz : 0x27, 100 Hz : 0x4F, 40 Hz: 0xC8
  Wire.endTransmission(true);

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  // 0: +-2g (16384 LSB/g), 1: 4g 8192, 2: 8g 4096, 3: 16g 2048
  Wire.endTransmission(true);
  
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  // 0: 250 deg/s (131), 1: 500 deg/s (65.5), 2: 1000 deg/s (32.8), 3: 2000 deg/s (16.4)
  Wire.endTransmission(true);
  
  // Configure interrupt pin
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x37); //Talk to the INT_PIN_CFG
  Wire.write(0x02); // Data Ready
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x38); // 
  Wire.write(0x01); // 
  Wire.endTransmission(true);
}

void loop() {
  if(flag_imuready == HIGH){
    ++cnt_imu;
    ++cnt;
    // Triggering Cam
    if(cnt > 9){
      cnt = 0;
      ++cnt_cam;
      digitalWrite(PIN_CAM_TRG,HIGH);
      digitalWrite(PIN_CAM_TRG,HIGH);
      digitalWrite(PIN_CAM_TRG,HIGH);
      digitalWrite(PIN_CAM_TRG,HIGH);
      digitalWrite(PIN_CAM_TRG,LOW);
      flag_trigger = 1;
    }

    // Read timestamp
    time_curr    = micros(); // microseconds
    tsec.ushort_ = (uint16_t)(time_curr/1000000);
    tusec.uint_  = (uint32_t)(time_curr-((uint32_t)tsec.ushort_)*1000000);
    dt           = time_curr - time_prev;

    // Read IMU data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 7*2=14 registers
    
    ax_raw   = (Wire.read()<<8 | Wire.read()); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    ay_raw   = (Wire.read()<<8 | Wire.read()); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    az_raw   = (Wire.read()<<8 | Wire.read()); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    temp_raw = (Wire.read()<<8 | Wire.read()); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    gx_raw   = (Wire.read()<<8 | Wire.read()); // reading registers: 0x43 (gx_rawOUT_H) and 0x44 (gx_rawOUT_L)
    gy_raw   = (Wire.read()<<8 | Wire.read()); // reading registers: 0x45 (gy_rawOUT_H) and 0x46 (gy_rawOUT_L)
    gz_raw   = (Wire.read()<<8 | Wire.read()); // reading registers: 0x47 (gz_rawOUT_H) and 0x48 (gz_rawOUT_L)
  
    u_ax.ushort_   = (uint16_t) 32768 + ax_raw;   // 2
    u_ay.ushort_   = (uint16_t) 32768 + (uint16_t) ay_raw;   // 2
    u_az.ushort_   = (uint16_t) 32768 + (uint16_t) az_raw;   // 2
    u_temp.ushort_ = (uint16_t) 32768 + (uint16_t) temp_raw; // 2
    u_gx.ushort_   = (uint16_t) 32768 + (uint16_t) gx_raw;   // 2
    u_gy.ushort_   = (uint16_t) 32768 + (uint16_t) gy_raw;   // 2
    u_gz.ushort_   = (uint16_t) 32768 + (uint16_t) gz_raw;   // 2, total 14 bytes.

    // Print IMU data
    buf[0]  = u_ax.bytes_[0];   buf[1] = u_ax.bytes_[1];
    buf[2]  = u_ay.bytes_[0];   buf[3] = u_ay.bytes_[1];
    buf[4]  = u_az.bytes_[0];   buf[5] = u_az.bytes_[1];
    buf[6]  = u_temp.bytes_[0]; buf[7] = u_temp.bytes_[1];
    buf[8]  = u_gx.bytes_[0];   buf[9] = u_gx.bytes_[1];
    buf[10] = u_gy.bytes_[0];   buf[11] = u_gy.bytes_[1];
    buf[12] = u_gz.bytes_[0];   buf[13] = u_gz.bytes_[1];

    // Print Time data
    buf[14]  = tsec.bytes_[0];  // time (second part, low)
    buf[15]  = tsec.bytes_[1];  // time (second part, high)
    buf[16]  = tusec.bytes_[0]; // time (microsecond part, lowest)
    buf[17]  = tusec.bytes_[1]; // time (microsecond part, low)
    buf[18]  = tusec.bytes_[2]; // time (microsecond part, high)
    buf[19]  = tusec.bytes_[3]; // time (microsecond part, highest)
    buf[20]  = flag_trigger;                              
    
    byte c = stringChecksum((char*)buf,21);
    
    Serial.write('$');
    Serial.write((char*)buf,21);
    Serial.write('*');
    Serial.write(c);
    Serial.write('%');
    
    // Prev time changing.
    time_prev = time_curr;
    flag_trigger  = 0;
    flag_imuready = LOW;
  }
  
}
