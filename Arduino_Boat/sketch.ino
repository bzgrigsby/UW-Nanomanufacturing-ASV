#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <Arduino_Modulino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <cmath>
#include <vector>
#include <array>
#include <Servo.h>
#include <Adafruit_MLX90393.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <zephyr/kernel.h>

// Threads and Reset Logic
#define WATCHDOG_STACK_SIZE 1024
#define WATCHDOG_PRIORITY -1 
K_THREAD_STACK_DEFINE(watchdog_stack_area, WATCHDOG_STACK_SIZE);
struct k_thread watchdog_thread_data;
volatile bool force_software_reset = false;
bool pythonConnected = false;
unsigned long lastBridgeCall = 0;
 
// Global Variables and Methods
  bool PWR_Flag = false;
  void setPWR(bool on);
  // LoRa
    const long LORA_FREQ = 915.0E6;
    const int LORA_CS = 10;
    const int LORA_RST = 2;
    const int LORA_IRQ = 3;
    const int TX_PWR = 23;
    volatile bool txRequest = false;
    bool receiveFlag = false;
    bool newMsg = false;
    volatile uint8_t txBuffer[256];
    volatile int txSize = 0;
    std::array<uint8_t, 256> rxBuffer;
    bool transmitMessage();
    void py_transmitMessage(std::vector<uint8_t> data);
    void receiveMessage();
    std::array<uint8_t, 256> py_reqMsg();
    bool bBool(uint8_t rawBin);
  // Winch
    const int stopPin = 7;
    const int dirPin = 8;
    const int depthPin  = 9;
    const int WINCH_UP = 1;
    const int WINCH_DN = 2;
    const int WINCH_STOP = 3;
    int lastDuration = 0;
    float depth = 0.0;
    const float MAX_VEL = 0.08; // m/s
    void driverOff() {
      digitalWrite(stopPin, LOW);
    }
    void driverOn() {
      digitalWrite(stopPin, HIGH);
    }
    void winchUP() {
      if (depth <= 0.01) return;
      driverOn();
      digitalWrite(dirPin, HIGH);
    }
    void winchDN() {
      if (depth >= 10.01) return;
      driverOn();
      digitalWrite(dirPin, LOW);
    }
    void moveWinch();
    void changeWinchDir(int newDir, int speed);
    bool updateWinchFuncVals(float A_, float T_, float O_);
    bool generateWinchPoints();
    void winchFunction();
    void updateDepth();
    bool prevstopPinStatus = HIGH; 
    bool stopPinStatus = LOW;
    const float STEP_DISTANCE_M = 0.0002; // m per step
  // Motors/ESCs
    const int RIGHT_PIN = 6;
    const int LEFT_PIN = 5;
    const int IDLE_PWM = 1469;
    int lThrust = IDLE_PWM;
    int lThrust_desired = IDLE_PWM;
    int lThrust_delta = 0;
    int rThrust = IDLE_PWM;
    int rThrust_desired = IDLE_PWM;
    int rThrust_delta = 0;
    Servo leftMotor;
    Servo rightMotor;
    void rampThrusts();
  // GPS
    static const uint32_t GPS_BAUD = 9600;
    TinyGPSPlus gps;
    std::vector<int32_t> getNavInfo();
    bool loc_isValid = false;
    int32_t lat = 0;
    int32_t lng = 0;
    int32_t speed = 0;
    int32_t gpsHeading = 0;
  // IMU
    ModulinoMovement movement;
    float ax = 0; float ay = 0; float az = 0; // in G's (1g = 9.81m/s^2)
    float ax_bias, ay_bias, az_bias;
    float gx = 0; float gy = 0; float gz = 0; // in deg/s
    float gx_bias, gy_bias, gz_bias;
    float roll = 0;  // deg
    float pitch = 0; // deg
    float prev_roll = 0.0;
    float prev_pitch = 0.0;
    float alpha = 0.95;
    float beta = 1.0 - alpha;
    unsigned long prevTime = 0.0;
    void updateIMU();
    void getAngles();
  // Magnetometer
    int32_t heading = 0;
    float mx, my, mz; // Magnetometer Axis values in uT
    float mx_L_offset = -29.93;
    float my_L_offset = -3.43;
    float mz_L_offset = -0.18;
    float cal11 = 1.006; float cal12 = -0.002; float cal13 = 0.003;
    float cal21 = -0.002; float cal22 = 0.957; float cal23 = -0.002;
    float cal31 = 0.003; float cal32 = 0.007; float cal33 = 1.038;
    float mx_L, my_L, mz_L;
    Adafruit_MLX90393 mlx = Adafruit_MLX90393();
    void getHeading();
  // Battery
    float battPin = A3;
    float soc = 0.0;

std::vector<double> broadcastTelemetry();

void setup() {
  // Initial Setup and Motor and Winch Idling
    Monitor.begin(115200);
    Serial.begin(9600);
    Bridge.begin();
    leftMotor.attach(LEFT_PIN);
    leftMotor.writeMicroseconds(IDLE_PWM);
    rightMotor.attach(RIGHT_PIN);
    rightMotor.writeMicroseconds(IDLE_PWM);
    pinMode(dirPin,  OUTPUT);
    pinMode(stopPin, OUTPUT);
    pinMode(depthPin, INPUT);
    driverOff();
    delay(5000); // Allow monitor/bridge to initialize
    
  // Propeller Motor Setup
    delay(2000); // allow for ESC stability
    Bridge.provide("set_thrusts", py_setThrusts);

  // LoRa Radio Setup
    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    pinMode(LORA_IRQ, INPUT);
    int loraRetries = 0;
    while (!LoRa.begin(LORA_FREQ) && loraRetries < 5) {
      Serial.println("LoRa init FAILED");
      delay(1000);
      loraRetries++;
    }
    if (loraRetries >= 5) { NVIC_SystemReset(); }
    LoRa.setSpreadingFactor(7);   
    LoRa.setSignalBandwidth(125E3); 
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();
    LoRa.setTxPower(TX_PWR);
    delay(10);
    LoRa.receive();
    Bridge.provide("req_msg", py_reqMsg);
    Bridge.provide("transmit_msg", py_transmitMessage);
    Serial.println("LoRa init OK");

  // Winch Motor Setup
    Bridge.provide("move_winch", py_moveWinch);
    Bridge.provide("start_winch_auto", py_winchFunc);
    Serial.println("Winch OK");

  // IMU Setup
    Wire1.begin();
    Wire1.setTimeout(100);
    prevTime = micros();
    Modulino.begin(); // Initialize Modulino I2C communication
    movement.begin(); // Detect and connect to movement sensor module
    ax_bias = -0.01496; ay_bias = 0.03713; az_bias = 0;
    gx_bias = 0.13062; gy_bias = -0.00269; gz_bias = -0.06738;

  // Magnetometer Setup
    Serial.println("Beginning Magnetometer Setup");
    int magRetries =  0;
    while(!mlx.begin_I2C(0x18, &Wire1) && magRetries < 5) {
      Monitor.println("No MLX90393 found. Trying again...");
      delay(100);
      magRetries++;
    }
    Serial.println("MLX90393 Found");
    mlx.setGain(MLX90393_GAIN_1X);
    // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
    mlx.setResolution(MLX90393_X, MLX90393_RES_16);
    mlx.setResolution(MLX90393_Y, MLX90393_RES_16);
    mlx.setResolution(MLX90393_Z, MLX90393_RES_16);
    mlx.setOversampling(MLX90393_OSR_1); // Set oversampling
    mlx.setFilter(MLX90393_FILTER_3); // Set digital filtering

    // Initialize GPS
      Serial1.begin(GPS_BAUD);
      delay(50);
      Bridge.provide("get_nav_info", getNavInfo);
      Serial.println("GPS Bridge Started\n");

  Bridge.provide ("get_telemetry", broadcastTelemetry);
  Bridge.provide("set_pwr", setPWR);
  Bridge.notify("setArduinoReady", true);
  
  k_thread_create(&watchdog_thread_data, watchdog_stack_area,
                  K_THREAD_STACK_SIZEOF(watchdog_stack_area),
                  watchdog_thread_entry,
                  NULL, NULL, NULL,
                  WATCHDOG_PRIORITY, 0, K_NO_WAIT);
}

// -------------------------LORA-----------------------------
bool bBool(uint8_t rawBin) {
  if (rawBin > 0x40) { return true; }
  return false;
}

// Needed for RadioHead compatibility
void addHeaderBytes() {
  LoRa.write(0xFF);
  LoRa.write(0xFF);
  LoRa.write(0x00);
  LoRa.write(0x00);
}

// PYTHON CALLABLE
void py_transmitMessage(std::vector<uint8_t> data) {
  if (data.size() > 255) return;
  txSize = data.size();
  for (int i = 0; i < data.size(); i++) {
    txBuffer[i] = data[i];
  }
  txRequest = true;
}

bool transmitMessage() {
  txRequest = false;
  if (txSize == 0) {
    Serial.println("Buffer Size is 0");
    return false; 
  }
  LoRa.beginPacket();
  addHeaderBytes();  
  LoRa.write((uint8_t*)txBuffer, txSize);
  LoRa.endPacket();
  return true;
}

// CALLS PYTHON
void receiveMessage() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;
  rxBuffer[0] = packetSize;
  for (int i = 0; i <= packetSize; i++) {
    if (i+2 < 255) rxBuffer[i+2] = LoRa.read();
  }
  rxBuffer[1] = (uint8_t)std::abs(LoRa.packetRssi()); 
  Bridge.notify("py_receive");
}

// PYTHON CALLABLE
std::array<uint8_t, 256> py_reqMsg() {
  return rxBuffer;
}
// ---------------------------------------------------------

// --------------------------WINCH---------------------------
int winch_move_dir = WINCH_STOP;
bool winchManual = false;
bool enableWinchAuto = false;

// PYTHON CALLABLE
int prev_winch_dir = WINCH_STOP;
void py_moveWinch(int newDir) {
  winchManual = true;
  enableWinchAuto = false;
  if (!PWR_Flag) { 
    winch_move_dir = WINCH_STOP; return; 
  }
  if (newDir == WINCH_UP || newDir == WINCH_DN || newDir == WINCH_STOP) {
    winch_move_dir = newDir;
    if (winch_move_dir == WINCH_STOP)  { 
      winchManual = false;
    }
  }
}

// Moves winch if user has manually selected it to  move
// Non-blocking by using a timer
void moveWinch() {
  if (winch_move_dir == WINCH_STOP || !PWR_Flag) driverOff();
  else if (winch_move_dir == WINCH_UP) winchUP();
  else if (winch_move_dir == WINCH_DN) winchDN();
  prev_winch_dir = winch_move_dir;
}

float A = 5.0; // Max depth, m
float T = 100.0; // Time between max depth and return to surface, s
float O = 0.0; // Offset value, m
float points[10];
float minDelta =  A/10.0;
int winch_idx = 0;
unsigned long t_start = 0.0;
// Asks the winch to move given inputs from a only-positive sine wave
void winchFunction() {
  if (enableWinchAuto == false) return;
  unsigned long t_current = millis();
  float currPt = points[winch_idx];
  float delta_t = (t_current - t_start)/1000.0;
  if (delta_t > T/10.0) {
    winch_idx++;
    if (winch_idx  > 9) { winch_idx = 0; }
    float nxtPt = points[winch_idx];
    Serial.println("Moving winch " + String(depth) + "->" + String(nxtPt));
    if (nxtPt < depth - minDelta) winch_move_dir = WINCH_UP;
    else if (nxtPt > depth + minDelta) winch_move_dir = WINCH_DN;
    t_start = t_current;
  } else if (depth < currPt + minDelta && depth > currPt - minDelta) {
    winch_move_dir = WINCH_STOP;
  }
}

// PYTHON Callable
//  Called by python from webserver to tell the winch to move.
// A [m], T [s], O [cm]
bool py_winchFunc(float A_, float T_, float O_) {
  Serial.println("Trying to move the winch automagically....");
  if (!PWR_Flag || winchManual) return false;
  if (T_ <= 0 || A_ <= 0 || O_ > 1000) return false;
  A = A_; T = T_; O = O_/100.0;
  minDelta = A/10.0;
  t_start = millis();
  winch_idx = 0;
  enableWinchAuto = generateWinchPoints();
  return enableWinchAuto;
}

// Generate points for the winch to follow
bool generateWinchPoints() {
  for (int i = 0; i < 10; i++) {
    float t = T/10*i;
    points[i] = A*std::abs(std::sin((3.14159265/T)*t)) + O;
  }
  float pointsDelta = std::abs(points[0] - points[1]);
  float T_delta = T/10.0;
  if (pointsDelta/T_delta > MAX_VEL) { 
    Serial.println("ERROR. Above MAX VEL");
    return false; 
  }
  if (std::abs(points[0] - depth)/T_delta > MAX_VEL) {
    Serial.println("ERROR. Above MAX VEL");
    return false; 
  }
  return true;
}

unsigned long lastDepthSampleTime = 18000;
const int DEPTH_INTERVAL = 300;
const float D_FILTER_ALPHA = 0.2; 
const float D_FILTER_BETA = 1.0-D_FILTER_ALPHA; // NEEDED TO MAKE WORK
bool depthInit = false;
void updateDepth() {
  if (millis() - lastDepthSampleTime > DEPTH_INTERVAL) {
    unsigned long timeout = millis() + 7;
    while (digitalRead(depthPin) == LOW) {
    if (millis() > timeout) {
      lastDepthSampleTime = millis();
      return;
    }
    yield();
    }
    timeout = millis() + 7;
    unsigned long pulseStartTime = micros();
    while (digitalRead(depthPin) == HIGH) {
    if (millis() > timeout) {
      lastDepthSampleTime = millis();
      return;
    }
    yield();
    }
    unsigned long duration = micros() - pulseStartTime; 
    lastDuration = duration;
    if (duration > 450 && duration < 3100) {
      float rawDepthSample = (duration - 500) / 250.0;
      if (rawDepthSample < 0 || rawDepthSample > 30) rawDepthSample = 0;
      if (depthInit == false)  {
        depth = rawDepthSample;
        depthInit = true;
      } else {        
        float discrepancy = abs(rawDepthSample - depth);
        if (discrepancy > 0.2) {
          depth = rawDepthSample * 0.01 + depth * 0.99;
        } else {
          depth = rawDepthSample * D_FILTER_ALPHA + depth * D_FILTER_BETA;
        }
      }
    }
    lastDepthSampleTime = millis();
  }
}
// -----------------------------------------------------------


// -------------------------ESCs------------------------------
// PYTHON CALLABLE.
// Left and right need to be given in microseconds. Parse on python.
void py_setThrusts(int leftPWM, int rightPWM) {
  if (!PWR_Flag) return;
  lThrust_desired = leftPWM;
  rThrust_desired = rightPWM;
}

unsigned long l_lastThrustUpdate = millis();
unsigned long r_lastThrustUpdate = millis();
unsigned long thrustUpdateInterval = 50; // ms, how often we ramp thrust states
const int MTD = 10; // us Max Thrust Delta
void rampThrusts() {
  if (!PWR_Flag && (lThrust != IDLE_PWM || rThrust != IDLE_PWM)){
    lThrust = IDLE_PWM; rThrust = IDLE_PWM;
    lThrust_desired = IDLE_PWM; rThrust_desired = IDLE_PWM;
    leftMotor.writeMicroseconds(lThrust);
    rightMotor.writeMicroseconds(rThrust);
    return;
  }

  if (lThrust != lThrust_desired || rThrust != rThrust_desired) {
    lThrust_delta = lThrust_desired - lThrust;
    rThrust_delta = rThrust_desired - rThrust;
    unsigned long currTime = millis();
    if (std::abs(lThrust_delta) > MTD) {
      if (currTime - l_lastThrustUpdate > thrustUpdateInterval) {
        lThrust = lThrust + ((lThrust_delta > 0) ? MTD : -MTD);
        leftMotor.writeMicroseconds(lThrust);
        l_lastThrustUpdate = currTime;
      }
    } else {
      lThrust = lThrust_desired;
      leftMotor.writeMicroseconds(lThrust);
      l_lastThrustUpdate = currTime;
    }

    if (std::abs(rThrust_delta) > MTD) {
      if (currTime - r_lastThrustUpdate > thrustUpdateInterval) {
        rThrust = rThrust + ((rThrust_delta > 0) ? MTD : -MTD);
        rightMotor.writeMicroseconds(rThrust);
        r_lastThrustUpdate = currTime;
      }
    } else {
      rThrust = rThrust_desired;
      rightMotor.writeMicroseconds(rThrust);
      r_lastThrustUpdate = currTime;
    } 
  }
}

// -------------------GPS & Magnetometer------------------------

// Assume z-axis is vertical out of the magnetometer
// Global X (Along boat axis): Local -Z
// Global Y (Transverse axis): Local Y
// Global Z (Vertical Axis):   Local -X
float float_heading = 0.0;
float filtered_heading = 0.0;
bool headingInit = false;
float H_FILTER_ALPHA = 0.20;
int magFailures = 0;
void getHeading() {
  if (mlx.readData(&mx_L, &my_L, &mz_L)) {
    float hi_x = mx_L - mx_L_offset;
    float hi_y = my_L - my_L_offset;
    float hi_z = mz_L - mz_L_offset;

    float cal_x = (hi_x * cal11) + (hi_y * cal12) + (hi_z * cal13);
    float cal_y = (hi_x * cal21) + (hi_y * cal22) + (hi_z * cal23);
    float cal_z = (hi_x * cal31) + (hi_y * cal32) + (hi_z * cal33);

    mx = -cal_z; // Flip axes per new global definitions above
    my = cal_y;
    mz = -cal_x;

    float cos_r = cos(roll * PI / 180.0); // Account for roll and pitch affects on magnetometer readings
    float sin_r = sin(roll * PI / 180.0);
    float cos_p = cos(pitch * PI / 180.0);
    float sin_p = sin(pitch * PI / 180.0);
  
    // Use the remapped, calibrated axes (mx, my, mz)
    float X_comp = mx * cos_p + my * sin_r * sin_p + mz * cos_r * sin_p; // Compensated for roll and pitch
    float Y_comp = my * cos_r - mz * sin_r;
  
    float_heading = atan2(Y_comp, X_comp) * 180.0 / PI;
  
    float_heading += 15.5; // Apply magnetic declination
    if (float_heading < 0) { float_heading += 360; }

    if (!headingInit) {
      filtered_heading = float_heading;
      headingInit = true; return;
    }
    float heading_delta = float_heading - filtered_heading;
    while (heading_delta < -180.0) heading_delta += 360.0;
    while (heading_delta >  180.0) heading_delta -= 360.0;
    filtered_heading += heading_delta * H_FILTER_ALPHA; // Apply gentle smoothing to heading to prevent drastic jumps
    if (filtered_heading < 0)    { filtered_heading += 360.0; }
    if (filtered_heading >= 360) { filtered_heading -= 360.0; }
    heading = (int32_t)(filtered_heading * 100.0); // Convert for bridge transport
  } else {
    magFailures++;
    if (magFailures > 5) {
      Wire1.end();
      delay(10);
      Wire1.begin();
      Wire1.setTimeout(100);
      mlx.begin_I2C(0x18, &Wire1);
      magFailures = 0;
    }
  }
}

std::vector<int32_t> getNavInfo() {
  std::vector<int32_t> data = {1, lat, lng, speed, gpsHeading, heading};  
  pythonConnected = true;
  lastBridgeCall = millis();
  if (!loc_isValid) {
    data = {0, 0, 0, 0, 0, heading};
  }
  return data;
}
// ----------------------------------------------------------- 

// -------------------------IMU------------------------------- 
// IMU is mounted sideways
// Global X = IMU -Z
// Global Y = IMU -X
// Global Z = IMU +Y
void updateIMU() {
   movement.update();
  // Global = IMU
  ax = -movement.getZ()-ax_bias;
  ay = movement.getX()-ay_bias;
  az = movement.getY()+az_bias;
  gx = -movement.getYaw()-gx_bias;
  gy = movement.getRoll()-gy_bias;
  gz = movement.getPitch()-gz_bias;
  if (std::fabs(gx) < 0.01) { gx = 0; }
  if (std::fabs(gy) < 0.01) { gy = 0; }
  if (std::fabs(gz) < 0.01) { gy = 0; }
  if (std::fabs(ax) < 0.001) { ax = 0; }
  if (std::fabs(ay) < 0.001) { ay = 0; }
  if (std::fabs(az) < 0.001) { az = 0; }
}

void getAngles() {
  unsigned long currTime = micros();
  float accel_roll = -atan2(ay, az)*180.0/PI;
  float denom = sqrt(ay*ay + az*az);
  if (denom < 0.001) { denom = 0.001; }
  float accel_pitch = atan2(ax, denom)*180.0/PI;
  
  float dT = (currTime - prevTime) / 1000000.0;
  float gyro_roll = -gx*dT + prev_roll;
  float gyro_pitch = gy*dT + prev_pitch;
  roll = alpha*gyro_roll + beta*accel_roll;
  pitch = alpha*gyro_pitch + beta*accel_pitch;
  prevTime = currTime;
  prev_roll = roll;
  prev_pitch = pitch;
}
// -----------------------------------------------------------


// ------------------------General----------------------------
void updateSOC() {
  int rawValue = analogRead(battPin);
  float pinVoltage = (rawValue / 1023.0) * 3.3;
  float batteryVoltage = pinVoltage * 6.0;
  soc = ((batteryVoltage - 12.0) / (16.8 - 12.0)) * 100.0;
  if (soc > 100.0) soc = 100.0;
  if (soc < 0.0) soc = 0.0;
}

void setPWR(bool on) {
  py_setThrusts(IDLE_PWM, IDLE_PWM);
  driverOff();
  PWR_Flag = on;
}

// Non-time crictical telemetry for webserver updates
// SOC, Acceleration, orientation, depth.
std::vector<double> broadcastTelemetry() {
  std::vector<double> data = {soc, ax, ay, roll, pitch, gz, depth};
  return data;
}


// Program Reset Logic
void watchdog_thread_entry(void *p1, void *p2, void *p3) {
    while (1) {
        if (pythonConnected) {
            unsigned long currTime = millis();
            if (currTime - lastBridgeCall > 15000) {
                Serial.println("Preemptive Watchdog Tripped!");
                Serial.println("Main loop hung or Python is dead. Forcing Reset...");
                delay(100);
                py_setThrusts(IDLE_PWM, IDLE_PWM);
                NVIC_SystemReset();
            }
        }
        k_msleep(1000); 
    }
}

// Main Loop
const unsigned long LORA_INTERVAL = 2;
const unsigned long IMU_INTERVAL = 50;
const unsigned long MAG_INTERVAL = 70;
const unsigned long BATT_INTERVAL = 900;
const unsigned long GPS_INTERVAL = 20;
unsigned long loraMillis = 0;
unsigned long battMillis = 0;
unsigned long imuMillis = 0;
unsigned long magMillis = 0;
unsigned long gpsMillis = 0;
unsigned long loopCounter = 0;
void loop() {
  unsigned long currTime = millis();
  // Winch
    moveWinch(); // manual user input
    winchFunction(); // rectified sine wave for automatic control
    updateDepth();

  // LoRa
    if (txRequest) {
      transmitMessage();
      receiveFlag = false;
    } else if (!receiveFlag) {
      LoRa.receive();
      receiveFlag = true;
    } 
    if (digitalRead(LORA_IRQ) == HIGH) {
        receiveMessage();
        LoRa.receive();
    }
    
  // GPS
    int bytesRead = 0;
    while (Serial1.available() && bytesRead < 64) {
      gps.encode(Serial1.read());
      bytesRead++;
    }
    // Need to convert to int32. 
    // .deg gives the whole number, billionths gives the 6 decimals. 
    // Combine the whole number with the decimals at the same scale (1e6) to get the correct answer
    // ? -> Ternary operator, like a compact if-else
    // Condition ? Value_If_True : Value_If_False
    int32_t temp_lat = gps.location.rawLat().deg * 1000000UL + (gps.location.rawLat().billionths / 1000UL);
    lat = gps.location.rawLat().negative ? -temp_lat : temp_lat;
    int32_t temp_lng = gps.location.rawLng().deg * 1000000UL + (gps.location.rawLng().billionths / 1000UL);
    lng = gps.location.rawLng().negative ? -temp_lng : temp_lng;
    speed = (int32_t)(gps.speed.knots() * 100.0);
    gpsHeading = (int32_t)(gps.course.deg() * 100.0);
    loc_isValid = gps.location.isValid();
    
  // IMU
    if (currTime - imuMillis >= IMU_INTERVAL) {
      updateIMU();
      getAngles();
      imuMillis = currTime;
    }

  // Magnetometer
    if (currTime - magMillis >= MAG_INTERVAL) {
      getHeading();
      magMillis = currTime;
    }
    
  // Battery
    // Note: Always curr_time - counter > interval
    // counter will eventually overflow, needs to account for this
    if (currTime - battMillis >= BATT_INTERVAL) {
      updateSOC();
      battMillis = currTime;
      Serial.print("Loop  Counts: "); Monitor.println(loopCounter);
      Serial.println("\tWinch Dir = " + String(winch_move_dir));
      Serial.println("\tDepth = " + String(depth) + "m -> " + String(points[winch_idx]) + "m  Duration = " + String(lastDuration) + "us");
      Serial.println("\tPWR = " +  String(PWR_Flag));
      Serial.println("\tSOC = " + String(soc) + "%");
      Serial.println("\tThrusts:");
      Serial.println("\t\tL_thrust = " + String(lThrust) + "  Desired = " + String(lThrust_desired));
      Serial.println("\t\tR_thrust = " + String(rThrust) + "  Desired = " + String(rThrust_desired));
      loopCounter = 0;
      if (pythonConnected && currTime - lastBridgeCall > 10000) { 
        Serial.println("A reset is coming");
      }
    }

  // ESC
    rampThrusts();

  loopCounter++;
}