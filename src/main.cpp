#include <Arduino.h>
#include <WiFi.h>
#include <RPi_Pico_TimerInterrupt.h>
#include <Wire.h>
#include "TOF.h"
#include "robot.h"
#include "trajectories.h"
#include "followLine.h"

TOF stof;

unsigned long interval;
unsigned long currentMicros, previousMicros;
char received;

//void init_control(robot_t& robot);
//void control(robot_t& robot);

PID_pars_t wheel_PID_pars;
////////////////////LIDAR///////////////////////
#include "lds.h"

#define DISTANCE_MAX    1000      // 100.0 cm 

lds_scan_t lds_scan;

////////////////////YDLIDAR X4///////////////////////
#include "Ydlidarx4.h"

YDLidarX4 lidar;

//////////////////EKF/////////////////////////////
#include "utils.h"
#include "ekf.h"

//EKF ekf;

/////////////////////////GCHANNELS///////////////////////////

#include "gchannels.h"
#include "file_gchannels.h"

#include <WiFiUdp.h>
#include <LittleFS.h>
#include "http_ota.h"


#define max_wifi_str 32

char ssid[max_wifi_str];
char password[max_wifi_str];

const char* fname_wifi = "/wifi.txt";

int udp_on, ip_on;

WiFiUDP Udp;
unsigned int localUdpPort = 4224;  // local port to listen on

#define UDP_MAX_SIZE 1024
uint8_t UdpInPacket[UDP_MAX_SIZE];  // buffer for incoming packets
uint8_t UdpOutPacket[UDP_MAX_SIZE];  // buffer for outgoing packets
int UdpBufferSize = UDP_MAX_SIZE;

gchannels_t udp_commands;
gchannels_t serial_commands;
commands_list_t pars_list;

const char *pars_fname = "pars.cfg";
bool load_pars_requested = false ;
bool gotoXY_req = false;

// Scan monitoring
volatile bool scanDone = false;
unsigned long n_scans = 0;
unsigned long last_scan_time = 0;
unsigned long scan_interval_ms = 0;

// LIDAR status
bool lidarStarted = false;

// =============================================================================
// FollowLine Parameters Structure
// =============================================================================
struct FollowLinePars {
    float xi;       // Start X
    float yi;       // Start Y
    float xf;       // End X
    float yf;       // End Y
    float tf;       // Final theta
    bool enabled;   // Enable/disable followLine execution
    bool reset_requested;  // Request to reset state machine
} fl_pars;

void init_followline_pars() {
    fl_pars.xi = -0.785f;
    fl_pars.yi = -0.57f;
    fl_pars.xf = -0.785f;
    fl_pars.yf = -0.57f;
    fl_pars.tf = PI / 2;
    fl_pars.enabled = false;
    fl_pars.reset_requested = false;
}

void set_interval(float new_interval)
{
  interval = new_interval * 1000000L;   // In microseconds
  robot.dt = new_interval;   // In seconds
  wheel_PID_pars.dt = robot.dt;  
}

void process_command(command_frame_t frame)
{
  pars_list.process_read_command(frame);

  if (frame.command_is("mo")) { // The 'mo'de command ...
    robot.control_mode = (control_mode_t) frame.value;

  } else if (frame.command_is("u1")) { // The 'u1' command sets the voltage for motor 1
    robot.u1_req = frame.value;

  } else if (frame.command_is("u2")) { // The 'u2' command sets the voltage for motor 1
    robot.u2_req = frame.value;

  } else if (frame.command_is("w1")) { 
    robot.w1_req = frame.value;

  } else if (frame.command_is("w2")) {
    robot.w2_req = frame.value;

  }  else if (frame.command_is("gtx")) {
    robot.gotoX = frame.value;
    gotoXY_req=true;

  } else if (frame.command_is("gty")) { 
     robot.gotoY = frame.value;
     gotoXY_req=true;

  } else if (frame.command_is("gtt")) { 
     robot.gotoTheta = frame.value;
     gotoXY_req=true;

  } else if (frame.command_is("gtm")) { 
     state = (GoToXYState)(frame.value);;
     gotoXY_req=true;

  } else if (frame.command_is("dt")) { 
     set_interval(frame.value);

  } else if (frame.command_is("v")) { 
    robot.v_req = frame.value;    

  } else if (frame.command_is("w")) { 
    robot.w_req = frame.value;    

  } else if (frame.command_is("sl")) { 
    robot.solenoid_PWM = frame.value;    

  } else if (frame.command_is("xr")) { 
    robot.xe = frame.value;    

  } else if (frame.command_is("yr")) { 
    robot.ye = frame.value;    

  } else if (frame.command_is("tr")) { 
    robot.thetae = frame.value;    

  } else if (frame.command_is("Xxr")) { 
    ekf.XR(0) = frame.value;    

  } else if (frame.command_is("Xyr")) { 
    ekf.XR(1) = frame.value;    

  } else if (frame.command_is("Xtr")) { 
    ekf.XR(2) = frame.value;    

  } else if (frame.command_is("xt")) { 
    traj.xt = frame.value;    

  } else if (frame.command_is("yt")) { 
    traj.yt = frame.value;    

  } else if (frame.command_is("pl")) { 
    //load_commands(pars_fname, serial_commands);
    load_pars_requested = true;

  } else if (frame.command_is("ps")) {  
    save_commands(pars_fname, pars_list, serial_commands);
   
  } else if (frame.command_is("ssid")) { 
    strncpy(ssid, frame.text, max_wifi_str - 1);
    ssid[max_wifi_str - 1] = 0;
  
  } else if (frame.command_is("pass")) { 
    if (strlen(frame.text) < 8) return;
    strncpy(password, frame.text, max_wifi_str - 1);
    password[max_wifi_str - 1] = 0;    
    
  } else if (frame.command_is("wifi")) { 
    if (frame.value == 1) {
      if (WiFi.connected()) WiFi.end();
      WiFi.begin(ssid, password); 
    } else if (frame.value == 0) {
      WiFi.end();
    }

  } else if (frame.command_is("httpota")) { 
    if (WiFi.connected()) {
      http_ota.host = frame.text;
      http_ota.uri = "/picoRobot/firmware.bin"; 
      http_ota.requested = true;
    }

  // =========================================================================
  // Pose Reset Commands
  // =========================================================================
  } else if (frame.command_is("rpose")) { 
    if (frame.value != 0) {
      robot.xe = ekf.XR(0);
      robot.ye = ekf.XR(1);
      robot.thetae = ekf.XR(2);
    }

  } else if (frame.command_is("rpdef")) { 
    if (frame.value != 0) {
      float def_x = -0.785f;
      float def_y = -0.57f;
      float def_theta = 1.5708f;
      robot.xe = def_x;
      robot.ye = def_y;
      robot.thetae = def_theta;
      ekf.XR(0) = def_x;
      ekf.XR(1) = def_y;
      ekf.XR(2) = def_theta;
    }

  } else if (frame.command_is("setpose")) { 
    float px, py, pt;
    if (sscanf(frame.text, "%f,%f,%f", &px, &py, &pt) == 3) {
      robot.xe = px;
      robot.ye = py;
      robot.thetae = pt;
      ekf.XR(0) = px;
      ekf.XR(1) = py;
      ekf.XR(2) = pt;
    }

  // =========================================================================
  // LIDAR Control Commands (NEW)
  // =========================================================================
  } else if (frame.command_is("lidar")) { 
    // Start/stop LIDAR: lidar 1; to start, lidar 0; to stop
    if (frame.value != 0) {
      if (!lidarStarted) {
        lidar.start();
        lidarStarted = true;
        Serial.println("LIDAR started");
      }
    } else {
      lidar.stop();
      lidarStarted = false;
      Serial.println("LIDAR stopped");
    }
  // =========================================================================
  // FollowLine Commands
  // =========================================================================
  } else if (frame.command_is("fxi")) { 
    fl_pars.xi = frame.value;    

  } else if (frame.command_is("fyi")) { 
    fl_pars.yi = frame.value;    

  } else if (frame.command_is("fxf")) { 
    fl_pars.xf = frame.value;    

  } else if (frame.command_is("fyf")) { 
    fl_pars.yf = frame.value;    

  } else if (frame.command_is("ftf")) { 
    fl_pars.tf = frame.value;    

  } else if (frame.command_is("flen")) { 
    fl_pars.enabled = (frame.value != 0);
    if (!fl_pars.enabled) {
      robot.v_req = 0;
      robot.w_req = 0;
    }

  } else if (frame.command_is("flrst")) { 
    if (frame.value != 0) {
      fl_pars.reset_requested = true;
      fl_pars.enabled = true;
    }
  } 
}

void send_file(const char* filename, int log_high)
{
  File f;
  f = LittleFS.open(filename, "r");
  if (!f) {
    serial_commands.send_command("err", filename);
    return;
  }

  serial_commands.flush();
  Serial.flush();

  int c;
  byte b, mask;
  if (log_high) mask = 0x80;
  else mask = 0;

  while(1) {
    c = f.read(&b, 1);
    if (c != 1) break;
    serial_commands.send_char(b | mask);
  }
  f.close();

  serial_commands.flush();
  Serial.flush();  
}

int analogWriteBits = 10; 
int analogWriteMax = (1 << analogWriteBits) - 1; 

#ifdef NOPIN
#undef NOPIN
#endif

//SerialPIO SerialTiny(SerialPIO::NOPIN, 21);
SerialPIO SerialTiny((pin_size_t)-1, 21);

/////////////////////////////MOTORS/////////////////////////////
const int motor_left_in1 = 10;
const int motor_left_in2 = 11;
const int motor_right_in1 = 12;
const int motor_right_in2 = 13;

//////////////////////////////ENCODERS///////////////////////////////
#include "PicoEncoder.h"

#define ENCL_A 2 //LEFT
#define ENCL_B 3 
#define ENCR_A 6 //RIGHT 
#define ENCR_B 7 


#define NUM_ENCODERS 2
PicoEncoder encoders[NUM_ENCODERS];
pin_size_t encoder_pins[NUM_ENCODERS] = {ENCL_A, ENCR_A};

bool calibration_requested;

void read_PIO_encoders(void)
{
  encoders[0].update();
  encoders[1].update();  
  robot.enc1 = encoders[0].speed;
  robot.enc2 = encoders[1].speed;

}

void initializeEncoders()
{
  pinMode(ENCL_A, INPUT_PULLUP);
  pinMode(ENCL_B, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP);
  pinMode(ENCR_B, INPUT_PULLUP);

}

void initializeMotors()
{
  pinMode(motor_left_in1, OUTPUT);
  pinMode(motor_left_in2, OUTPUT);
  pinMode(motor_right_in1, OUTPUT);
  pinMode(motor_right_in2, OUTPUT);

}

float voltageToPWM(float voltage, float maxVoltage, float maxPWM)
{
  if (maxVoltage < 1e-3f) return 0;   // avoid NaN/Inf at boot

  int pwm = (voltage / maxVoltage) * maxPWM;
  pwm = constrain(pwm, -maxPWM, maxPWM);
  return pwm;
}

void setMotorsPWM(float u1, float u2)
{

  float maxVoltage = robot.battery_voltage; // Maximum voltage
  float maxPWM = 230;      // Maximum PWM value for 8-bit resolution
  
  int PWM1 = voltageToPWM(u1, maxVoltage, maxPWM);
  int PWM2 = voltageToPWM(u2, maxVoltage, maxPWM);

  if (PWM1 > 0){
      analogWrite(motor_left_in1, PWM1);
      digitalWrite(motor_left_in2, LOW);
  }else{
    PWM1 = -PWM1;
    analogWrite(motor_left_in2, PWM1);
    digitalWrite(motor_left_in1, LOW);
  }

  if (PWM2 > 0){
    analogWrite(motor_right_in1, PWM2);
    digitalWrite(motor_right_in2, LOW);
  }else{
    PWM2 = -PWM2;
    analogWrite(motor_right_in2, PWM2);
    digitalWrite(motor_right_in1, LOW);
  }

}

void printEncoders()
{
  Serial.print("Encoder Left:");
  Serial.print(robot.enc1);
  Serial.print(" Encoder Right:");
  Serial.print(robot.enc2);
}

void printOdometry()
{
  Serial.print(" X:");
  Serial.print(robot.xe);
  Serial.print(" Y:");
  Serial.print(robot.ye);
  Serial.print(" Theta:");
  Serial.print((robot.thetae*180)/PI);
  Serial.println("");
}

void printTof()
{  
  Serial.print(" TOF: ");
  Serial.print(stof.distance_tof);
}

void serial_write(const char *buffer, size_t size)
{
  Serial.write(buffer, size);
  if (udp_on) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(buffer, size);
    //Serial.print("Sent="); Serial.println(Udp.endPacket());
    Udp.endPacket();  
  } 
}

void serial_Beacons(){
  serial_commands.send_command("Bx0", ekf.BeaconCluster[0].x);
  serial_commands.send_command("By0", ekf.BeaconCluster[0].y);
  serial_commands.send_command("Bn0", ekf.BeaconCluster[0].n);

  serial_commands.send_command("Bx1", ekf.BeaconCluster[1].x);
  serial_commands.send_command("By1", ekf.BeaconCluster[1].y);
  serial_commands.send_command("Bn1", ekf.BeaconCluster[1].n);

  serial_commands.send_command("Bx2", ekf.BeaconCluster[2].x);
  serial_commands.send_command("By2", ekf.BeaconCluster[2].y);
  serial_commands.send_command("Bn2", ekf.BeaconCluster[2].n);

  serial_commands.send_command("Bx3", ekf.BeaconCluster[3].x);
  serial_commands.send_command("By3", ekf.BeaconCluster[3].y);
  serial_commands.send_command("Bn3", ekf.BeaconCluster[3].n);

  serial_commands.send_command("Bx4", ekf.BeaconCluster[4].x);
  serial_commands.send_command("By4", ekf.BeaconCluster[4].y);
  serial_commands.send_command("Bn4", ekf.BeaconCluster[4].n);

  serial_commands.send_command("Bx5", ekf.BeaconCluster[5].x);
  serial_commands.send_command("By5", ekf.BeaconCluster[5].y);
  serial_commands.send_command("Bn5", ekf.BeaconCluster[5].n);
}

void Spin360(){
  static bool spin_started = false;
  static uint32_t spin_t0_ms = 0;

  const float SPIN_TARGET = TWO_PI;   // 360º in rad
  const float SPIN_W      = 1.0f;     // rad/s (pick what you want)
  const float SPIN_TOL    = 0.0f;    // ~1.7º tolerance
  const uint32_t TIMEOUT_MS = 12000;  // safety stop

  if (!spin_started) {
    spin_started = true;
    robot.rel_theta = 0.0f;           // reset accumulated rotation
    spin_t0_ms = millis();
  }

  // keep spinning until we hit ~360º or timeout
  if (fabs(robot.rel_theta) < (SPIN_TARGET - SPIN_TOL) && (millis() - spin_t0_ms) < TIMEOUT_MS) {
    robot.setRobotVW(0.0f, SPIN_W);
  } else {
    robot.setRobotVW(0.0f, 0.0f);
  }
}

void serial_ComRobot()
{
  serial_commands.send_command("Vbat", robot.battery_voltage);

  serial_commands.send_command("Xst", ekf.XR(0));
  serial_commands.send_command("Yst", ekf.XR(1));
  serial_commands.send_command("Thetast", ekf.XR(2));

  serial_commands.send_command("xe", robot.xe);
  serial_commands.send_command("ye", robot.ye);
  serial_commands.send_command("te", robot.thetae);

  // FollowLine state feedback
  serial_commands.send_command("fls", (float)followLineState);
  serial_commands.send_command("gts", (float)state);
  serial_commands.send_command("flen", (float)(fl_pars.enabled ? 1 : 0));

  // Send scan statistics for debugging
  serial_commands.send_command("nscn", (float)lidar.getNumScans());
  serial_commands.send_command("scnms", (float)scan_interval_ms);
  serial_commands.send_command("lpts", (float)lidar.getLastScanPointCount()); 

  pars_list.send_sparse_commands(serial_commands);

  serial_commands.send_command("dbg", 5.0f); 
  serial_commands.send_command("loop", (float)(micros() - interval));  
    
  serial_commands.flush();   

  http_ota.handle();
}


void setup() {

  set_interval(0.04);  // 40 ms = 25 Hz

  analogReadResolution(10);

  pars_list.register_command("kf", &(wheel_PID_pars.Kf));
  pars_list.register_command("kc", &(wheel_PID_pars.Kc));
  pars_list.register_command("ki", &(wheel_PID_pars.Ki));
  pars_list.register_command("kd", &(wheel_PID_pars.Kd));
  pars_list.register_command("kfd", &(wheel_PID_pars.Kfd));
  pars_list.register_command("dz", &(wheel_PID_pars.dead_zone));

  pars_list.register_command("at", &(traj.thetat));
  pars_list.register_command("xt", &(traj.xt));
  pars_list.register_command("yt", &(traj.yt));

  pars_list.register_command("xi", &(traj.xi));
  pars_list.register_command("yi", &(traj.yi));

  pars_list.register_command("cx", &(traj.cx));
  pars_list.register_command("cy", &(traj.cy));

  pars_list.register_command("fv", &(robot.follow_v));
  pars_list.register_command("fk", &(robot.follow_k));
  pars_list.register_command("kt", &(traj.ktheta));

  // Velocity constants
  pars_list.register_command("van", &VEL_ANG_NOM);
  pars_list.register_command("vln", &VEL_LIN_NOM);
  pars_list.register_command("wda", &W_DA);
  pars_list.register_command("lda", &LinDeAccel);
  
  // Angle thresholds and gains
  pars_list.register_command("metf", &MAX_ETF);
  pars_list.register_command("hetf", &HIST_ETF);
  pars_list.register_command("gfwd", &GAIN_FWD);
  pars_list.register_command("dda", &DIST_DA);
  pars_list.register_command("gda", &GAIN_DA);
  
  // Distance thresholds
  pars_list.register_command("tfd", &TOL_FINDIST);
  pars_list.register_command("dnp", &DIST_NEWPOSE);
  pars_list.register_command("tnp", &THETA_NEWPOSE);
  pars_list.register_command("tda", &THETA_DA);
  pars_list.register_command("tft", &TOL_FINTHETA);
  
  // Line following thresholds
  pars_list.register_command("dnl", &DIST_NEWLINE);
  pars_list.register_command("dnel", &DIST_NEARLINE);

  // Line following omega gains
  pars_list.register_command("kdst", &K_DIST);
  pars_list.register_command("kang", &K_ANG);

  udp_commands.init(process_command, serial_write);
  serial_commands.init(process_command, serial_write);

  init_followline_pars();

  Serial.begin(115200);
  //Serial1.begin(230400);  // For LIDAR initialized by YDLidar library

  LittleFS.begin();
  
  float control_interval = 0.04;  // In seconds


  // All wheeel PID controllers share the same parameters
  wheel_PID_pars.Kf = 0.35;
  wheel_PID_pars.Kc = 0.7;
  wheel_PID_pars.Ki = 2.25;
  wheel_PID_pars.Kd = 0;
  wheel_PID_pars.Kfd = 0;
  wheel_PID_pars.dt = control_interval;
  wheel_PID_pars.dead_zone = 0;
  int i;
  for (i = 0; i < NUM_WHEELS; i++) {
    robot.PID[i].init_pars(&wheel_PID_pars);
  }

  // strcpy(ssid, "5DPO-NETWORK");
  // strcpy(password, "5dpo5dpo");^

  strcpy(ssid, "5DPO-NETWORK");
  strcpy(password, "5dpo5dpo");

  load_commands(pars_fname, serial_commands);

  // Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait until connected or timeout
  //Serial.print("Connecting to WiFi");
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    Serial.print(".");
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi.");
  }

  initializeMotors();

  //ldsInit(&lds_scan);

  if (!lidar.begin(Serial1, 128000)) {
  } else {
    delay(500);  // Give LIDAR time to stabilize
    lidar.start();
    lidarStarted = true;
  }

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();


  initializeEncoders();
  encoders[0].begin(encoder_pins[0]);
  encoders[1].begin(encoder_pins[1]);
  
  stof.initializeToFSensor();

  SerialTiny.begin(); //leitura da tensão da bateria

  robot.control_mode = cm_kinematics;
  
  last_scan_time = millis();
}

uint8_t b;

void loop() {
  
  if (SerialTiny.available()) { 
    b = SerialTiny.read();  
    robot.battery_voltage = 1e-3 * ((b >> 1) * 50 + 4800);
  }

  if (WiFi.connected() && !ip_on){
    serial_commands.send_command("msg", (String("Pico W is connected to WiFi network with SSID ") + WiFi.SSID()).c_str());
    ip_on = Udp.begin(localUdpPort);
  }

  if (load_pars_requested) {
    load_pars_requested = false;
    load_commands(pars_fname, serial_commands);
  }

  if (ip_on) {
    //ArduinoOTA.handle();

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      int i;
      udp_on = 1;
      int len = Udp.read(UdpInPacket, UdpBufferSize - 1);
      if (len > 0) {
        UdpInPacket[len] = 0;
      }
      for (i = 0; i < len; i++) {
        udp_commands.process_char(UdpInPacket[i]);
      }
    }      
  }

  // if(Serial1.available() > 0){
  //     if (ldsUpdate(&lds_scan, Serial1.read())) {
  //       //scanDone = false;
      
  //       for(int i=0; i<360; i++) {
  //         ekf.LaserValues(0,i) = lds_scan.data[i].range * 0.001;
  //       }
  //       scanDone = true; //in meters
  //     }
  // }
  
   // Process LIDAR data using class methods

  if (lidarStarted) {
    lidar.processData();
   
    // Check if a complete 360° scan is ready
    if (lidar.isScanReady()) {
      //Serial.print("lidarStarted:"); Serial.println(lidarStarted);
      // Get the 360-degree scan data (in meters)
      float* scan360 = lidar.get360Scan();
      
      // Copy to EKF
      for (int i = 0; i < 360; i++) {
        ekf.LaserValues(0, i) = scan360[i];
      }
      
      // Debug: Print main rays
      // Serial.print("Rays (m): ");
      // Serial.print("0:"); Serial.print(ekf.LaserValues(0, 0), 3); Serial.print("  ");
      // Serial.print("90:"); Serial.print(ekf.LaserValues(0, 90), 3); Serial.print("  ");
      // Serial.print("180:"); Serial.print(ekf.LaserValues(0, 180), 3); Serial.print("  ");
      // Serial.print("270:"); Serial.print(ekf.LaserValues(0, 270), 3); Serial.println();
      
      scanDone = true;
      // n_scans++; // Now handled by lidar class
      
      // Clear the flag
      lidar.clearScanReady();
    }
  }

  currentMicros = micros();
  if(currentMicros - previousMicros >= interval){
    previousMicros = currentMicros;

    read_PIO_encoders();
    //stof.calculateTOF();
      
    robot.odometry(); 

    ekf.predict(robot.ve, robot.we, robot.dt); //robot.dt??

    if(scanDone){
      scanDone = false;

      ekf.phaseAV();
      serial_Beacons();
      ekf.motionmodelEKF();
    }

    setPose(ekf.XR(0), ekf.XR(1), ekf.XR(2));

    if (fl_pars.reset_requested) {
      fl_pars.reset_requested = false;
      followLineState = Follow_Line;
      state = Rotation;
    }

    // Run followLine if enabled
    if (fl_pars.enabled) {
      followLine(fl_pars.xi, fl_pars.yi, fl_pars.xf, fl_pars.yf, fl_pars.tf);
    }

    robot.accelerationLimit(); 
    robot.calcMotorsVoltage(); 
    setMotorsPWM(robot.u1, robot.u2);

  
    serial_ComRobot();
    
  }
} 


