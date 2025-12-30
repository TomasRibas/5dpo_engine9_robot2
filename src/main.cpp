#include <Arduino.h>

#include <WiFi.h>

#include <RPi_Pico_TimerInterrupt.h>
#include <Wire.h>
#include "TOF.h"
#include "robot.h"
#include "trajectories.h"

#include "lds_driver.hpp"
#include "followLine.h"

//FollowLineController fl;

TOF stof;

unsigned long interval;
unsigned long currentMicros, previousMicros;
char received;

// Create after optional Serial1 pin mapping, so use a pointer:
//lds::LFCDLaser* lidar = nullptr;

//void init_control(robot_t& robot);
//void control(robot_t& robot);

PID_pars_t wheel_PID_pars;
////////////////////LIDAR///////////////////////
#include "lds.h"



#define DISTANCE_MAX    1000      // 100.0 cm 

lds_scan_t lds_scan;

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

  // Map voltage to PWM
  int pwm = (voltage / maxVoltage) * maxPWM;
  
  // Constrain the PWM value to the valid range
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
  serial_commands.send_command("Bd0", ekf.BeaconCluster[0].dist);
  serial_commands.send_command("Bt0", ekf.BeaconCluster[0].angle);
  serial_commands.send_command("Bn0", ekf.BeaconCluster[0].n);

  serial_commands.send_command("Bx1", ekf.BeaconCluster[1].x);
  serial_commands.send_command("By1", ekf.BeaconCluster[1].y);
  serial_commands.send_command("Bd1", ekf.BeaconCluster[1].dist);
  serial_commands.send_command("Bt1", ekf.BeaconCluster[1].angle);
  serial_commands.send_command("Bn1", ekf.BeaconCluster[1].n);

  serial_commands.send_command("Bx2", ekf.BeaconCluster[2].x);
  serial_commands.send_command("By2", ekf.BeaconCluster[2].y);
  serial_commands.send_command("Bd2", ekf.BeaconCluster[2].dist);
  serial_commands.send_command("Bt2", ekf.BeaconCluster[2].angle);
  serial_commands.send_command("Bn2", ekf.BeaconCluster[2].n);

  serial_commands.send_command("Bx3", ekf.BeaconCluster[3].x);
  serial_commands.send_command("By3", ekf.BeaconCluster[3].y);
  serial_commands.send_command("Bd3", ekf.BeaconCluster[3].dist);
  serial_commands.send_command("Bt3", ekf.BeaconCluster[3].angle);
  serial_commands.send_command("Bn3", ekf.BeaconCluster[3].n);

  serial_commands.send_command("Bx4", ekf.BeaconCluster[4].x);
  serial_commands.send_command("By4", ekf.BeaconCluster[4].y);
  serial_commands.send_command("Bd4", ekf.BeaconCluster[4].dist);
  serial_commands.send_command("Bt4", ekf.BeaconCluster[4].angle);
  serial_commands.send_command("Bn4", ekf.BeaconCluster[4].n);

  serial_commands.send_command("Bx5", ekf.BeaconCluster[5].x);
  serial_commands.send_command("By5", ekf.BeaconCluster[5].y);
  serial_commands.send_command("Bd5", ekf.BeaconCluster[5].dist);
  serial_commands.send_command("Bt5", ekf.BeaconCluster[5].angle);
  serial_commands.send_command("Bn5", ekf.BeaconCluster[5].n);
}

void serial_ComRobot(){
  // Debug information
  
  serial_commands.send_command("Xst",ekf.XR(0));
  serial_commands.send_command("Yst",ekf.XR(1));
  serial_commands.send_command("Thetast",ekf.XR(2));

  serial_commands.send_command("u1", robot.u1);
  serial_commands.send_command("u2", robot.u2);

  serial_commands.send_command("e1", robot.enc1);
  serial_commands.send_command("e2", robot.enc2);

  serial_commands.send_command("Vbat", robot.battery_voltage);

  serial_commands.send_command("ve", robot.ve);
  serial_commands.send_command("we", robot.we);

  serial_commands.send_command("w1", robot.w1e);
  serial_commands.send_command("w2", robot.w2e);

  serial_commands.send_command("w1req", robot.w1_req);
  serial_commands.send_command("w2req", robot.w2_req);

  serial_commands.send_command("kc", wheel_PID_pars.Kc);
  serial_commands.send_command("ki", wheel_PID_pars.Ki);
  serial_commands.send_command("kd", wheel_PID_pars.Kd);
  serial_commands.send_command("kf", wheel_PID_pars.Kf);
  serial_commands.send_command("kfd", wheel_PID_pars.Kfd);

  serial_commands.send_command("gtx", robot.gotoX);
  serial_commands.send_command("gty", robot.gotoY);
  serial_commands.send_command("gtt", robot.gotoTheta);
  serial_commands.send_command("gtm", (int)(followLineState));


  serial_commands.send_command("sl", robot.solenoid_PWM);

  serial_commands.send_command("is", robot.i_sense);
  serial_commands.send_command("us", robot.u_sense);

  serial_commands.send_command("mode", robot.control_mode);

  serial_commands.send_command("IP", WiFi.localIP().toString().c_str());

  serial_commands.send_command("m1", robot.PWM_1);
  serial_commands.send_command("m2", robot.PWM_2);

  serial_commands.send_command("xe", robot.xe);
  serial_commands.send_command("ye", robot.ye);
  serial_commands.send_command("te", robot.thetae);

  pars_list.send_sparse_commands(serial_commands);

  Serial.print(" cmd: ");
  Serial.print(serial_commands.frame.command);
  Serial.print("; ");
    
  //debug = serial_commands.out_count;
  serial_commands.send_command("dbg", 5); 
  serial_commands.send_command("loop", micros() - interval);  
    
  serial_commands.flush();   
  Serial.println();

  http_ota.handle();
}


void setup() {

  set_interval(0.04);  // In seconds

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

  //pars_list.register_command("fk", &(robot.i_lambda));
  pars_list.register_command("kt", &(traj.ktheta));
  //pars_list.register_command("ssid", ssid, max_wifi_str);
  //pars_list.register_command("pass", password, max_wifi_str);

  udp_commands.init(process_command, serial_write);
  
  serial_commands.init(process_command, serial_write);

  

  Serial.begin(115200);
  Serial1.begin(230400);  // For LIDAR

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
 
  // Start WiFi with supplied parameters
  WiFi.begin(ssid, password);

  // Wait until connected or timeout
  Serial.print("Connecting to WiFi");
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    Serial.print(".");
    delay(500);
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

  ldsInit(&lds_scan);

  //delay(100);
  //Serial1.print("b");
  //lidar = new lds::LFCDLaser(230400);  // ctor signature per new header :contentReference[oaicite:3]{index=3}

  // Setup motor pin
  //analogWriteFreq(10000); // 10 kHz PWM
  //analogWriteRange(255);
   // 50% duty cycle


  Wire.setSDA(4);
  Wire.setSCL(5);

  Wire.begin();


  initializeEncoders();
  encoders[0].begin(encoder_pins[0]);
  encoders[1].begin(encoder_pins[1]);
  
  stof.initializeToFSensor();

  SerialTiny.begin(); //leitura da tensão da bateria

  robot.control_mode = cm_kinematics;
  //robot.control_mode = cm_kinematics;

}

uint8_t b;
bool scanDone = false;

void loop() {
  
  if (SerialTiny.available()) { 
    b = SerialTiny.read();  
    robot.battery_voltage = 1e-3 * ((b >> 1) * 50 + 4800);
  }

  if (WiFi.connected() && !ip_on){
    // Connection established
    serial_commands.send_command("msg", (String("Pico W is connected to WiFi network with SSID ") + WiFi.SSID()).c_str());
 
    ip_on = Udp.begin(localUdpPort);
    Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  }

  if (ip_on) {
    //ArduinoOTA.handle();

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      int i;
      udp_on = 1;
      // receive incoming UDP packets

      //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(UdpInPacket, UdpBufferSize - 1);
      if (len > 0) {
        UdpInPacket[len] = 0;
      }
      //Serial.printf("UDP packet contents (as string): %s\n", UdpInPacket);

      for (i = 0; i < len; i++) {
        udp_commands.process_char(UdpInPacket[i]);
        //Serial.write(UdpInPacket[i]);
      }
    }      
  }

  if(Serial1.available() > 0)
    {
      if (ldsUpdate(&lds_scan, Serial1.read())) {
        // Complete scan received
        // Now you can safely print or process the full scan
        scanDone = false;
      
        // Optional: Print complete scan data here instead
        for(int i=0; i<360; i++) {
          ekf.LaserValues(0,i) = lds_scan.data[i].range * 0.001;
        }
        scanDone = true; //in meters
      }
    }
      
  currentMicros = micros();
  if(currentMicros - previousMicros >= interval){
     previousMicros = currentMicros;

    if(scanDone){

     read_PIO_encoders();
     //Serial.println(lds_scan.motor_speed);
     stof.calculateTOF();
      
      robot.odometry(); 

      // ekf.predict(robot.ve, robot.we, robot.dt); //robot.dt??

      // ekf.phaseAV();

      // serial_Beacons();

      // ekf.motionmodelEKF();

      setPose(robot.xe, robot.ye, robot.thetae);
      printOdometry();
      followLine(-0.785, -0.50, -0.6, -0.2, 1.57);
      //robot.setRobotVW(0, 0.5);

      robot.accelerationLimit(); 
      robot.calcMotorsVoltage(); 
      setMotorsPWM(robot.u1, robot.u2);

      //serial_ComRobot();

    }
    else{
      Serial.println("No scan data");}
    
    
  }
}


