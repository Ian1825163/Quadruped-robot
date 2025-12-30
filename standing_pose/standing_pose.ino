#include <Arduino.h>
#include <math.h>
#include <IntervalTimer.h>

// -----------------------------
//  Config
// -----------------------------
static constexpr uint32_t BAUD_SERVO = 1000000;
static constexpr uint32_t BAUD_USB   = 115200;

static constexpr uint16_t CENTER_TICK = 2048;   // STS 12-bit center
static constexpr uint16_t GOAL_SPEED  = 1000;   // 0x03E8
static constexpr uint16_t GOAL_TIME   = 200;    // don't use 0 at first (gentler)

static constexpr bool ENABLE_TIMER = false;     // false: test stable position; true for real time control
static constexpr bool ENABLE_TRAJ  = false;     
static constexpr uint32_t CTRL_US  = 20000;     // 20ms => 50Hz

// Angle safety clamp (deg)
static constexpr double LIMIT_DEG = 70.0;

// -----------------------------
//  Utils
// -----------------------------
inline double clampd(double v, double lo, double hi){
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
inline double deg2rad(double d){ return d * M_PI / 180.0; }
inline double rad2deg(double r){ return r * 180.0 / M_PI; }

inline int16_t wrap4096(int32_t v){
  v %= 4096;
  if(v < 0) v += 4096;
  return (int16_t)v;
}

// rad -> tick offset (can be negative)
inline int32_t rad2tick_offset(double rad){
  return (int32_t)lround(rad * (4096.0 / (2.0*M_PI)));
}

// center + rad (safe wrap 0..4095)
inline uint16_t centerPlusRad(double rad){
  int32_t goal = (int32_t)CENTER_TICK + rad2tick_offset(rad);
  return (uint16_t)wrap4096(goal);
}

inline double wrapPi(double a){
  while(a <= -M_PI) a += 2.0*M_PI;
  while(a >   M_PI) a -= 2.0*M_PI;
  return a;
}
inline double angDiff(double a, double b){
  return wrapPi(a - b);
}

// -----------------------------
//  Debug shared (main loop prints)
// -----------------------------
volatile double debug_x = 0, debug_y = 0, debug_z = 0;
volatile double debug_a1 = 0, debug_a2 = 0, debug_a3 = 0;

unsigned long lastPrintTime = 0;

// -----------------------------
//  Leg class (你的 IK 保留，微調安全)
// -----------------------------
class Leg {
public:
  bool inited_c;
  double last_c_local;

  const double L1;
  const int L2;
  const int L3;
  const int L4;
  const double h;
  const int r;

  double angle1;
  double angle2;
  double angle3;
  double pos[3];

  Leg();
  void setTargetPosition(double x, double y, double z);
  void computeIK();
};

Leg::Leg()
  : inited_c(false),
    last_c_local(0.0),
    L1(26.086),
    L2(80),
    L3(80),
    L4(20),
    h(28.5),
    r(22),
    angle1(M_PI/2),
    angle2(M_PI/2),
    angle3(M_PI/2)
{
  pos[0] = 26.086;
  pos[1] = 0.0;
  pos[2] = -155.0;
}

void Leg::setTargetPosition(double x, double y, double z){
  pos[0] = x;
  pos[1] = y;
  pos[2] = z;
}

void Leg::computeIK(){
  double x = pos[0], y = pos[1], z = pos[2];

  double l2 = x*x + y*y + z*z;
  double l  = sqrt(l2);
  if(l < 1e-6) l = 1e-6;

  double d2 = l2 - L1*L1;
  if(d2 < 0) d2 = 0;
  double d  = sqrt(d2);
  if(d < 1e-6) d = 1e-6;

  // knee geometry
  double cosK = (L1*L1 + (double)L2*L2 + (double)L3*L3 - l2) / (2.0*L2*L3);
  cosK = clampd(cosK, -1.0, 1.0);
  double theta_k = acos(cosK);

  // theta_b
  double s1 = clampd(y/d, -1.0, 1.0);
  double cos2 = ((double)L2*L2 + d*d - (double)L3*L3) / (2.0*L2*d);
  cos2 = clampd(cos2, -1.0, 1.0);
  double theta_b = M_PI/2.0 - (asin(s1) + acos(cos2));

  // theta_a
  double ca = clampd(L1 / l, -1.0, 1.0);
  double theta_a = atan2(x, fabs(z)) + acos(ca) - M_PI/2.0;

  // theta_c (your linkage)
  double A = 2.0*r*(L4*cos(theta_k) + L2);
  double B = 6.0*r;
  double C = (double)r*r + pow((double)L2 + L4*cos(theta_k), 2)
           + 9.0 + pow(L4*sin(theta_k) - h, 2) - (double)L3*L3;

  double disc = (A*B)*(A*B) - (A*A - C*C)*(B*B - C*C);
  double sq = sqrt(fmax(disc, 0.0));

  double Y1 = -A*B + sq;
  double Y2 = -A*B - sq;
  double X  = A*A - C*C;
  double c1 = wrapPi(atan2(Y1, X));
  double c2 = wrapPi(atan2(Y2, X));

  double chosen = 0.0;
  if(!inited_c){
    chosen = c1;
    inited_c = true;
  }else{
    double err1 = fabs(angDiff(c1, last_c_local));
    double err2 = fabs(angDiff(c2, last_c_local));
    chosen = (err1 <= err2) ? c1 : c2;
  }
  last_c_local = chosen;

  angle1 = wrapPi(theta_a);
  angle2 = wrapPi(theta_b);
  angle3 = chosen;
}

// -----------------------------
//  Globals
// -----------------------------
IntervalTimer controlTimer;
volatile bool doUpdate = false;
double tt = 0;

Leg leg_controller;

// -----------------------------
//  Motor packet (SYNC WRITE 12 motors)
//  - a1..a12 in RAD (internal)
// -----------------------------
void sendMotorPacketRad(double a1, double a2, double a3,
                        double a4, double a5, double a6,
                        double a7, double a8, double a9,
                        double a10,double a11,double a12)
{
  // safety clamp
  auto clampRad = [](double r){
    return clampd(r, deg2rad(-LIMIT_DEG), deg2rad(LIMIT_DEG));
  };

  const uint8_t header[] = {0xFF, 0xFF, 0xFE, 0x58, 0x83, 0x2A, 0x06};
  uint8_t packet[150];
  int idx = 0;

  memcpy(packet, header, sizeof(header));
  idx += sizeof(header);

  const uint8_t ids[12] = {1,2,3,4,5,6,7,8,9,10,11,12};

  double rad[12] = {
    clampRad(a1), clampRad(a2), clampRad(a3),
    clampRad(a4), clampRad(a5), clampRad(a6),
    clampRad(a7), clampRad(a8), clampRad(a9),
    clampRad(a10),clampRad(a11),clampRad(a12)
  };

  uint16_t goal[12];
  for(int i=0;i<12;i++){
    goal[i] = centerPlusRad(rad[i]);
  }

  for(int i=0;i<12;i++){
    packet[idx++] = ids[i];

    // position 0x2A
    packet[idx++] = (goal[i] >> 0) & 0xFF;
    packet[idx++] = (goal[i] >> 8) & 0xFF;

    packet[idx++] = 0x00;                // 資料長度低位
    packet[idx++] = 0x01;                // 資料長度高位
    packet[idx++] = 0xE8;    // 位置值低位
    packet[idx++] = 0x03; // 位置值高位
  }

  // checksum: sum from 0xFE
  uint8_t sum = 0;
  for (int i = 2; i < idx; i++) {
    sum += packet[i];
  }
  uint8_t checksum = ~sum;

  packet[idx++] = checksum;


  Serial4.write(packet, idx);
}

// Convenience: send in DEG (more human-friendly)
void sendMotorPacketDeg(double d1, double d2, double d3,
                        double d4, double d5, double d6,
                        double d7, double d8, double d9,
                        double d10,double d11,double d12)
{
  sendMotorPacketRad(deg2rad(d1), deg2rad(d2), deg2rad(d3),
                     deg2rad(d4), deg2rad(d5), deg2rad(d6),
                     deg2rad(d7), deg2rad(d8), deg2rad(d9),
                     deg2rad(d10),deg2rad(d11),deg2rad(d12));
}

// -----------------------------
//  Trajectory (safe small ellipse)
// -----------------------------
/*void generate_trajectory(Leg& leg, double t){
  const double x  = 26.086;
  const double y0 = 0.0;
  const double z0 = -145.0;
  const double Ay = 20.0;
  const double Az = 10.0;

  double y = y0 + Ay * sin(t);
  double z = z0 + Az * cos(t);

  leg.setTargetPosition(x, y, z);

  debug_x = x; debug_y = y; debug_z = z;
}*/

// -----------------------------
//  Timer ISR: only set a flag
// -----------------------------
void updateControlISR(){
  doUpdate = true;
}

// -----------------------------
//  setup / loop
// -----------------------------
void setup(){
  Serial4.begin(BAUD_SERVO);
  Serial.begin(BAUD_USB);
  delay(200);

  // 先送一次固定姿勢：用「度數」寫最直覺
  // 你原本那串數字我視為「度」
  sendMotorPacketDeg(
    0.0,  -75.0, -30.0,
   -0.0,   75.0, 35.0,
    0.0,   75.0,  30.0,
   -0.0,  -75.0, -30.0
  );
  delay(1000);

  if(ENABLE_TIMER){
    controlTimer.begin(updateControlISR, CTRL_US);
  }
}

void loop(){
  // 控制更新（非ISR）
  if(ENABLE_TIMER && doUpdate){
    doUpdate = false;

    tt += (CTRL_US / 1000000.0);
    if(tt > 2.0*M_PI) tt -= 2.0*M_PI;

    if(ENABLE_TRAJ){
     // generate_trajectory(leg_controller, tt);
      leg_controller.computeIK();

      debug_a1 = leg_controller.angle1;
      debug_a2 = leg_controller.angle2;
      debug_a3 = leg_controller.angle3;

      // 目前先不做 12 馬達 gait：你要先驗單腿時，這裡可以只送那 3 顆有用的
      // 這裡先示範：把單腿 IK 角度送到某三顆（例如 RF: 7,8,9），其他保持 0 deg
      // 你可以改成你要測的那隻腿對應的三顆 ID
      sendMotorPacketRad(
        0,0,0,
        0,0,0,
        leg_controller.angle1, leg_controller.angle2, leg_controller.angle3,
        0,0,0
      );
    }
  }

  // debug print every 200ms
  unsigned long now = millis();
  if(now - lastPrintTime >= 200){
    lastPrintTime = now;

    Serial.print("x: "); Serial.print((double)debug_x, 3);
    Serial.print(", y: "); Serial.print((double)debug_y, 3);
    Serial.print(", z: "); Serial.println((double)debug_z, 3);

    Serial.print("a1(rad): "); Serial.print((double)debug_a1, 6);
    Serial.print(", a2(rad): "); Serial.print((double)debug_a2, 6);
    Serial.print(", a3(rad): "); Serial.println((double)debug_a3, 6);
  }
}
