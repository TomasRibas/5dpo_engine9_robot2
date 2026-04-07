#include <Arduino.h>
#include <WiFi.h>
#include <RPi_Pico_TimerInterrupt.h>
#include <Wire.h>
#include "TOF.h"
#include "robot.h"
#include "trajectories.h"
#include "motionControl.h"
#include "servo_control.h"

bool solenoid_on = false;

TOF stof;

unsigned long interval;
unsigned long currentMicros, previousMicros;

PID_pars_t wheel_PID_pars;

////////////////////YDLIDAR X4///////////////////////
#include "Ydlidarx4.h"

YDLidarX4 lidar;

// YDLidar X4 configuration
#define YDLIDAR_BAUDRATE 128000

#define LIDAR_TX_PIN 0   // Pico TX -> LIDAR RX
#define LIDAR_RX_PIN 1   // Pico RX -> LIDAR TX

//////////////////EKF/////////////////////////////
#include "utils.h"
#include "ekf.h"

/////////////////////////GCHANNELS///////////////////////////
#include "gchannels.h"
#include "file_gchannels.h"

#include <WiFiUdp.h>
#include <LittleFS.h>
#include "http_ota.h"


#define max_wifi_str 32

int led = LED_BUILTIN;

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
bool load_pars_requested = false;
bool gotoXY_req = false;

volatile bool scanDone = false;
unsigned long n_scans = 0;
unsigned long last_scan_time = 0;
unsigned long scan_interval_ms = 0;
int points_in_scan = 0;

bool lidarStarted = false;

// =============================================================================
// FollowLine Parameters Structure
// =============================================================================
struct FollowLinePars {
    float xi;
    float yi;
    float xf;
    float yf;
    float tf;
    int   dir;              // 1 = forward, -1 = reverse
    bool enabled;
    bool reset_requested;
} fl_pars;

void init_followline_pars() {
    fl_pars.xi = -0.785f;
    fl_pars.yi = -0.57f;
    fl_pars.xf = -0.785f;
    fl_pars.yf = -0.57f;
    fl_pars.tf = PI / 2;
    fl_pars.dir = 1;
    fl_pars.enabled = false;
    fl_pars.reset_requested = false;
}

// --- 1. New parameter struct (alongside FollowLinePars) ---
 
// --- 2. Parameter struct (alongside FollowLinePars) ---
struct FollowCirclePars {
    float xc;
    float yc;
    float R;
    float angf;
    float tf;
    int   dir;      // +1 = CCW,  -1 = CW
    bool  enabled;
    bool  reset_requested;
} fc_pars;
 
void init_followcircle_pars() {
    fc_pars.xc      =  0.0f;
    fc_pars.yc      =  0.0f;   // default: robot at (0,0) is on top of circle
    fc_pars.R       =  0.1f;
    fc_pars.angf    =  1.57f;
    fc_pars.tf      =  1.57f;
    fc_pars.dir     =  1;
    fc_pars.enabled =  false;
    fc_pars.reset_requested = false;
}

bool rev_active = false;
float rev_speed  = 0.0f;

int analogWriteBits = 8; 
int analogWriteMax = (1 << analogWriteBits) - 1; 

const int solenoid_in1 = 14;
const int solenoid_in2 = 15;

void initializeSolenoid() {
  pinMode(solenoid_in1, OUTPUT);
  pinMode(solenoid_in2, OUTPUT);
  analogWrite(solenoid_in1, analogWriteMax);  // both HIGH = off (inverted logic)
  analogWrite(solenoid_in2, analogWriteMax);
}

void setSolenoidPWM(int pwm) {
  int PWM_max = analogWriteMax;  // 1023
  if (pwm == 0) {
    digitalWrite(solenoid_in1, HIGH);  // clean HIGH, not PWM
    digitalWrite(solenoid_in2, HIGH);
  } else if (pwm > 0) {
    analogWrite(solenoid_in1, PWM_max - pwm);
    analogWrite(solenoid_in2, PWM_max);
  } else {
    analogWrite(solenoid_in1, PWM_max);
    analogWrite(solenoid_in2, PWM_max + pwm);
  }
}

void set_interval(float new_interval)
{
  interval = new_interval * 1000000L;
  robot.dt = new_interval;
  wheel_PID_pars.dt = robot.dt;  
}

float voltageToPWM(float voltage, float maxVoltage, float maxPWM)
{
  if (maxVoltage < 1e-3f) return 0;

  int pwm = (voltage / maxVoltage) * maxPWM;
  pwm = constrain(pwm, -maxPWM, maxPWM);
  return pwm;
}

void process_command(command_frame_t frame)
{
  pars_list.process_read_command(frame);

  if (frame.command_is("mo")) {
    robot.control_mode = (control_mode_t) frame.value;

  } else if (frame.command_is("u1")) {
    robot.u1_req = frame.value;

  } else if (frame.command_is("u2")) {
    robot.u2_req = frame.value;

  } else if (frame.command_is("w1")) { 
    robot.w1_req = frame.value;

  } else if (frame.command_is("w2")) {
    robot.w2_req = frame.value;

  } else if (frame.command_is("gtx")) {
    robot.gotoX = frame.value;
    gotoXY_req = true;

  } else if (frame.command_is("gty")) { 
    robot.gotoY = frame.value;
    gotoXY_req = true;

  } else if (frame.command_is("gtt")) { 
    robot.gotoTheta = frame.value;
    gotoXY_req = true;

  } else if (frame.command_is("gtm")) { 
    state = (GoToXYState)(frame.value);
    gotoXY_req = true;

  } else if (frame.command_is("dt")) { 
    set_interval(frame.value);

  } else if (frame.command_is("v")) { 
    robot.v_req = frame.value;    

  } else if (frame.command_is("w")) { 
    robot.w_req = frame.value;    

  } else if (frame.command_is("sl")) { 
    if (frame.value > 0) {
      robot.solenoid_PWM = (int)voltageToPWM(5.0f, robot.battery_voltage, analogWriteMax);
      solenoid_on = true;
    } else {
      robot.solenoid_PWM = -(int)voltageToPWM(5.0f, robot.battery_voltage, analogWriteMax);
      setSolenoidPWM(robot.solenoid_PWM);
      robot.solenoid_PWM = 0;
      setSolenoidPWM(robot.solenoid_PWM);

      solenoid_on = false;
    }
  } else if (frame.command_is("sv")) {
    servoGoTo((int)frame.value);  // sv 90 → turns to 90° 
  
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

      resetFollowLine();
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
      resetFollowLine();
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
      resetFollowLine();
    }

  // =========================================================================
  // LIDAR Control Commands - Updated for YDLidarX4 driver
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
  } else if (frame.command_is("fldr")) {
    fl_pars.dir = (int)frame.value;  // 1=forward, -1=reverse

  } else if (frame.command_is("flen")) { 
    fl_pars.enabled = (frame.value != 0);
    if (!fl_pars.enabled) {
      robot.v_req = 0;
      robot.w_req = 0;
    } else {
      // Always reset state machine when re-enabling,
      // so stale state from a previous run never carries over
      resetFollowLine();
    }

  } else if (frame.command_is("flrst")) { 
    if (frame.value != 0) {
      //fl_pars.reset_requested = true;
      fl_pars.enabled = true;
      resetFollowLine();
      setPose(ekf.XR(0), ekf.XR(1), ekf.XR(2));
    }
  } else if (frame.command_is("fcxc")) { 
      fc_pars.xc   = frame.value;
  } else if (frame.command_is("fcyc")) { 
      fc_pars.yc   = frame.value;
  } else if (frame.command_is("fcR")) { 
      fc_pars.R    = frame.value;
  } else if (frame.command_is("fcaf")) { 
      fc_pars.angf = frame.value;
  } else if (frame.command_is("fctf")) { 
      fc_pars.tf   = frame.value;
  } else if (frame.command_is("fcdr")) { 
      fc_pars.dir  = (int)frame.value; 
  } else if (frame.command_is("fcen")) {
      fc_pars.enabled = (frame.value != 0);
      if (!fc_pars.enabled) { 
        robot.v_req = 0; robot.w_req = 0; 
      } else { 
        resetFollowCircle(); 
      }
  } else if (frame.command_is("fcrst")) {
      if (frame.value != 0) { fc_pars.reset_requested = true; fc_pars.enabled = true; }
  } else if (frame.command_is("revspd")) {
    rev_speed = frame.value;
    rev_active = (frame.value != 0.0f);
    robot.setRobotVW(rev_speed, 0.0f);
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
}

#ifdef NOPIN
#undef NOPIN
#endif


SerialPIO SerialTiny((pin_size_t)-1, 21);

const int motor_left_in1 = 10;
const int motor_left_in2 = 11;
const int motor_right_in1 = 12;
const int motor_right_in2 = 13;

//////////////////////////////ENCODERS///////////////////////////////
#include "PicoEncoder.h"

#define ENCL_A 2
#define ENCL_B 3 
#define ENCR_A 6
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


void setMotorsPWM(float u1, float u2)
{
  float maxVoltage = robot.battery_voltage;
  float maxPWM = 255;
  
  int PWM1 = voltageToPWM(u1, maxVoltage, maxPWM);
  int PWM2 = voltageToPWM(u2, maxVoltage, maxPWM);

  if (PWM1 > 0){
      analogWrite(motor_left_in1, PWM1);
      digitalWrite(motor_left_in2, LOW);
  } else {
    PWM1 = -PWM1;
    analogWrite(motor_left_in2, PWM1);
    digitalWrite(motor_left_in1, LOW);
  }

  if (PWM2 > 0){
    analogWrite(motor_right_in1, PWM2);
    digitalWrite(motor_right_in2, LOW);
  } else {
    PWM2 = -PWM2;
    analogWrite(motor_right_in2, PWM2);
    digitalWrite(motor_right_in1, LOW);
  }
}

void serial_write(const char *buffer, size_t size)
{
  Serial.write(buffer, size);
  if (udp_on) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(buffer, size);
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

void serial_ComRobot()
{
  serial_commands.send_command("Vbat", robot.battery_voltage);

  serial_commands.send_command("Xst", ekf.XR(0));
  serial_commands.send_command("Yst", ekf.XR(1));
  serial_commands.send_command("Thetast", ekf.XR(2));

  // serial_commands.send_command("xe", robot.xe);
  // serial_commands.send_command("ye", robot.ye);
  // serial_commands.send_command("te", robot.thetae);

  serial_commands.send_command("fls", (float)followLineState);
  serial_commands.send_command("gts", (float)state);
  serial_commands.send_command("flen", (float)(fl_pars.enabled ? 1 : 0));

  serial_commands.send_command("fxi", (float)fl_pars.xi);
  serial_commands.send_command("fyi", (float)fl_pars.yi);
  serial_commands.send_command("fxf", (float)fl_pars.xf);
  serial_commands.send_command("fyf", (float)fl_pars.yf);
  serial_commands.send_command("ftf", (float)fl_pars.tf);
  serial_commands.send_command("fldr", (float)fl_pars.dir);

  serial_commands.send_command("w1",     robot.w1e);
  serial_commands.send_command("w2",     robot.w2e);
  serial_commands.send_command("w1req", robot.w1ref);
  serial_commands.send_command("w2req", robot.w2ref);
  serial_commands.send_command("u1", robot.u1);
  serial_commands.send_command("u2", robot.u2);
  serial_commands.send_command("ve", robot.ve);
  serial_commands.send_command("we", robot.we);
  // FollowLine tuning constants
  serial_commands.send_command("van",  VEL_ANG_NOM);
  serial_commands.send_command("vln",  VEL_LIN_NOM);
  serial_commands.send_command("wda",  W_DA);
  serial_commands.send_command("lda",  LinDeAccel);
  serial_commands.send_command("metf", MAX_ETF);
  serial_commands.send_command("hetf", HIST_ETF);
  serial_commands.send_command("gfwd", GAIN_FWD);
  serial_commands.send_command("dda",  DIST_DA);
  serial_commands.send_command("gda",  GAIN_DA);
  serial_commands.send_command("tfd",  TOL_FINDIST);
  serial_commands.send_command("dnp",  DIST_NEWPOSE);
  serial_commands.send_command("tnp",  THETA_NEWPOSE);
  serial_commands.send_command("tda",  THETA_DA);
  serial_commands.send_command("tft",  TOL_FINTHETA);
  serial_commands.send_command("dnl",  DIST_NEWLINE);
  serial_commands.send_command("dnel", DIST_NEARLINE);
  serial_commands.send_command("kdst", K_DIST);
  serial_commands.send_command("kang", K_ANG);
  serial_commands.send_command("kvramp", KV_RAMP);
  serial_commands.send_command("kdsti", K_DIST_I);

  serial_commands.send_command("kc",  wheel_PID_pars.Kc);
  serial_commands.send_command("ki",  wheel_PID_pars.Ki);
  serial_commands.send_command("kf",  wheel_PID_pars.Kf);
  serial_commands.send_command("kd",  wheel_PID_pars.Kd);
  serial_commands.send_command("dz", wheel_PID_pars.dead_zone);

 
  // FollowCircle state & params
  serial_commands.send_command("fcs",      (float)followCircleState);
  serial_commands.send_command("fcen",     (float)(fc_pars.enabled ? 1 : 0));
  serial_commands.send_command("fcxc",     fc_pars.xc);
  serial_commands.send_command("fcyc",     fc_pars.yc);
  serial_commands.send_command("fcR",      fc_pars.R);
  serial_commands.send_command("fcaf",     fc_pars.angf);
  serial_commands.send_command("fctf",     fc_pars.tf);
  serial_commands.send_command("fcdr",     (float)fc_pars.dir);
  // FollowCircle tuning
  serial_commands.send_command("fcvln",    FC_VEL_LIN_NOM);
  serial_commands.send_command("fcvan",    FC_VEL_ANG_NOM);
  serial_commands.send_command("fclda",    FC_LinDeAccel);
  serial_commands.send_command("fcwda",    FC_W_DA);
  serial_commands.send_command("fctfd",    FC_TOL_FINDIST);
  serial_commands.send_command("fcdda",    FC_DIST_DA);
  serial_commands.send_command("fckang",   FC_K_ANG);
  serial_commands.send_command("fckrad",   FC_K_RAD);
  serial_commands.send_command("fckvramp", FC_KV_RAMP);

  serial_commands.send_command("revspd", rev_speed);

  serial_commands.send_command("tof", stof.distance_tof);

  serial_commands.send_command("sl", solenoid_on ? 1.0f : 0.0f);
  // serial_commands.send_command("nscn", (float)lidar.getNumScans());
  // serial_commands.send_command("scnms", (float)scan_interval_ms);
  // serial_commands.send_command("lpts", (float)lidar.getLastScanPointCount());
  // serial_commands.send_command("lhz", lidar.getScanRate());


  // static unsigned long last_scan_send = 0;
  // if (millis() - last_scan_send >= 1000) {
  //   last_scan_send = millis();


  //   serial_commands.send_command("LS", ekf.scanN);
  //   serial_commands.flush();

  //   for (uint16_t i = 0; i < ekf.scanN; i++) {
  //     serial_commands.send_command("Li", (float)i);
  //     serial_commands.send_command("La", (float)ekf.scanPts[i].ang);   // radians
  //     serial_commands.send_command("Ld", (float)ekf.scanPts[i].dist);  // meters
  //   }

  //   serial_commands.send_command("LE", 1.0f);
  //   serial_commands.flush();
  // }

  pars_list.send_sparse_commands(serial_commands);

  serial_commands.send_command("dbg", (float)robot.solenoid_PWM); 
  serial_commands.send_command("loop", (float)(micros() - interval));  
    
  serial_commands.flush();   

  http_ota.handle();
}

void processLidarData()
{

  if (!lidarStarted) return;
  
  lidar.processData();
  
  if (scanDone) return;

  if (lidar.isScanReady()) {
    uint16_t n = lidar.getLastScanPointCount();
    //Serial.print("n: "); Serial.println(n);
    ekf.setScan(lidar.ang_data.data(), lidar.dist_data.data(), n);

    scanDone = true;
  }
  lidar.clearScanReady();
}


bool initYDLidar()
{
  Serial1.setFIFOSize(1024);

  Serial1.setTX(LIDAR_TX_PIN);
  Serial1.setRX(LIDAR_RX_PIN);


  lidar.begin(Serial1, YDLIDAR_BAUDRATE);
  
  delay(50); 
  lidar.start();

  lidarStarted = true;
  
  return true;
}

void setup() {

  pinMode(led, OUTPUT);

  initializeSolenoid();

  initServo();
  
  set_interval(0.04);  // 40ms = 25Hz
  
  //analogWriteFreq(25000);
  
  //analogWriteResolution(10);

  analogReadResolution(10);

  // =========================================================================
  // PID Parameters
  // =========================================================================
  pars_list.register_command("kf", &(wheel_PID_pars.Kf));
  pars_list.register_command("kc", &(wheel_PID_pars.Kc));
  pars_list.register_command("ki", &(wheel_PID_pars.Ki));
  pars_list.register_command("kd", &(wheel_PID_pars.Kd));
  pars_list.register_command("kfd", &(wheel_PID_pars.Kfd));
  pars_list.register_command("dz", &(wheel_PID_pars.dead_zone));

  // =========================================================================
  // Trajectory Parameters
  // =========================================================================
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

  // =========================================================================
  // FollowLine Tuning Constants
  // =========================================================================
  
  pars_list.register_command("van", &VEL_ANG_NOM);
  pars_list.register_command("vln", &VEL_LIN_NOM);
  pars_list.register_command("wda", &W_DA);
  pars_list.register_command("lda", &LinDeAccel);
  
  pars_list.register_command("metf", &MAX_ETF);
  pars_list.register_command("hetf", &HIST_ETF);
  pars_list.register_command("gfwd", &GAIN_FWD);
  pars_list.register_command("dda", &DIST_DA);
  pars_list.register_command("gda", &GAIN_DA);
  
  pars_list.register_command("tfd", &TOL_FINDIST);
  pars_list.register_command("dnp", &DIST_NEWPOSE);
  pars_list.register_command("tnp", &THETA_NEWPOSE);
  pars_list.register_command("tda", &THETA_DA);
  pars_list.register_command("tft", &TOL_FINTHETA);
  
  pars_list.register_command("dnl", &DIST_NEWLINE);
  pars_list.register_command("dnel", &DIST_NEARLINE);

  pars_list.register_command("kdst", &K_DIST);
  pars_list.register_command("kang", &K_ANG);
  pars_list.register_command("kvramp", &KV_RAMP);
  pars_list.register_command("kdsti", &K_DIST_I);

  pars_list.register_command("fcvln",    &FC_VEL_LIN_NOM);
  pars_list.register_command("fcvan",    &FC_VEL_ANG_NOM);
  pars_list.register_command("fclda",    &FC_LinDeAccel);
  pars_list.register_command("fcwda",    &FC_W_DA);
  pars_list.register_command("fctfd",    &FC_TOL_FINDIST);
  pars_list.register_command("fcdda",    &FC_DIST_DA);
  pars_list.register_command("fcdnp",    &FC_DIST_NEWPOSE);
  pars_list.register_command("fctda",    &FC_THETA_DA);
  pars_list.register_command("fctnp",    &FC_THETA_NEWPOSE);
  pars_list.register_command("fctft",    &FC_TOL_FINTHETA);
  pars_list.register_command("fckang",   &FC_K_ANG);
  pars_list.register_command("fckrad",   &FC_K_RAD);
  pars_list.register_command("fckvramp", &FC_KV_RAMP);

  udp_commands.init(process_command, serial_write);
  serial_commands.init(process_command, serial_write);

  init_followline_pars();

  Serial.begin(115200);

  LittleFS.begin();
  
  float control_interval = 0.04;

  // PID defaults
  wheel_PID_pars.Kf = 0;
  wheel_PID_pars.Kc = 0.65;
  wheel_PID_pars.Ki = 2.6;
  wheel_PID_pars.Kd = 0;
  wheel_PID_pars.Kfd = 0;
  wheel_PID_pars.dt = control_interval;
  wheel_PID_pars.dead_zone = 1.5;
  
  for (int i = 0; i < NUM_WHEELS; i++) {
    robot.PID[i].init_pars(&wheel_PID_pars);
  }

  strcpy(ssid, "FEUP-I-108");
  strcpy(password, "5dpo5dpo");

  load_commands(pars_fname, serial_commands);

  // WiFi setup
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi.");
  }

  initYDLidar();

  initializeMotors();

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
 
  initializeEncoders();
  encoders[0].begin(encoder_pins[0]);
  encoders[1].begin(encoder_pins[1]);
  
  stof.initializeToFSensor();

  SerialTiny.begin();

  robot.control_mode = cm_kinematics;

  last_scan_time = millis();

}

uint8_t b;

void loop() {
  
  // Battery voltage reading
  if (SerialTiny.available()) { 
    b = SerialTiny.read();  
    robot.battery_voltage = 1e-3 * ((b >> 1) * 50 + 4800);
  }

  // WiFi connection handling
  if (WiFi.connected() && !ip_on) {
    serial_commands.send_command("msg", (String("Pico W connected to ") + WiFi.SSID()).c_str());
    ip_on = Udp.begin(localUdpPort);
  }

  // Process USB serial commands
  while (Serial.available() > 0) {
    serial_commands.process_char(Serial.read());
  }

  // Load parameters if requested
  if (load_pars_requested) {
    load_pars_requested = false;
    load_commands(pars_fname, serial_commands);
  }

  // Process UDP commands
  if (ip_on) {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      udp_on = 1;
      int len = Udp.read(UdpInPacket, UdpBufferSize - 1);
      if (len > 0) {
        UdpInPacket[len] = 0;
      }
      for (int i = 0; i < len; i++) {
        udp_commands.process_char(UdpInPacket[i]);
      }
    }      
  }

  processLidarData();

  currentMicros = micros();
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    //Serial.println("AAAAAAAAA");
    stof.calculateTOF();
    //Serial.print("Tof Value: ");Serial.println(stof.distance_tof);

    read_PIO_encoders();
    robot.odometry(); 
    
    ekf.predict(robot.ve, robot.we, robot.dt);
    

    if (scanDone) {
      scanDone = false;
      ekf.phaseAV();
      serial_Beacons();
      ekf.motionmodelEKF();
    }

    setPose(ekf.XR(0), ekf.XR(1), ekf.XR(2));

    static float last_xi = fl_pars.xi;
    static float last_yi = fl_pars.yi;
    static float last_xf = fl_pars.xf;
    static float last_yf = fl_pars.yf;
    static float last_tf = fl_pars.tf;
    
    if (fl_pars.enabled) {

        if (fl_pars.xi != last_xi || fl_pars.yi != last_yi ||
            fl_pars.xf != last_xf || fl_pars.yf != last_yf ||
            fl_pars.tf != last_tf) {

            resetFollowLine();

            last_xi = fl_pars.xi;
            last_yi = fl_pars.yi;
            last_xf = fl_pars.xf;
            last_yf = fl_pars.yf;
            last_tf = fl_pars.tf;
        }
    }
        
    // if (fl_pars.reset_requested) {
    //   fl_pars.reset_requested = false;
    //   resetFollowLine();
    // }

    if (fc_pars.reset_requested) {
        fc_pars.reset_requested = false;
        resetFollowCircle();
    }

    if (fl_pars.enabled) {
      followLine(fl_pars.xi, fl_pars.yi, fl_pars.xf, fl_pars.yf, fl_pars.tf, fl_pars.dir);
    }  else if (fc_pars.enabled) {
      followCircle(fc_pars.xc, fc_pars.yc, fc_pars.R, fc_pars.angf, fc_pars.tf, fc_pars.dir);
    } else if (rev_active) {
      robot.setRobotVW(rev_speed, 0.0f);
    }

    robot.accelerationLimit(); 
    robot.calcMotorsVoltage(); 
    setMotorsPWM(robot.u1, robot.u2);
    setSolenoidPWM(robot.solenoid_PWM);

    serial_ComRobot();
  }
}