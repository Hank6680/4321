#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ---------------- BLE config ----------------
const char* targetDeviceName = "ESP32_BLE_Server";
BLEUUID serviceUUID("12345678-1234-1234-1234-123456789ABC");
BLEUUID charUUID   ("ABCDEF12-3456-7890-1234-567890ABCDEF");

BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
bool deviceConnected = false;
String serverAddress;

// ---------------- Joystick & control params ----------------
static const int JOY_MIN = 0;
static const int JOY_MAX = 4095;
static const int JOY_MID = 2048;

// 阈值
static const int Y_HIGH = 3800;
static const int Y_LOW  = 300;
static const int X_LOW  = 300;
static const int X_HIGH = 3800;

// 未收到数据超时刹车（ms）
static const uint32_t RX_TIMEOUT_MS = 500;

// ---------------- L298N pins（按接线改） ----------------
// Motor A
static const int ENA = 1;   // PWM
static const int IN1 = 2;
static const int IN2 = 3;
// Motor B
static const int ENB = 5;   // PWM
static const int IN3 = 6;
static const int IN4 = 7;

// --------- Motor C pins（你实际接线后改成对应 IO）---------
static const int ENC  = 8;   // PWM for Motor C
static const int INC1 = 9;
static const int INC2 = 10;

bool button_state = 0;      // 0 = 正转, 1 = 反转
uint8_t lastBtn   = 1;      // 约定：1 = 松开, 0 = 按下

// ---------------- PWM config (analogWrite) ----------------
static const int PWM_MAX = 255;

// ---------------- runtime state ----------------
volatile int16_t gJoyX = JOY_MID;
volatile int16_t gJoyY = JOY_MID;
volatile uint8_t gBtnState = 1;      // 0=按下, 1=松开
volatile uint32_t gLastRxMs = 0;

// 从 Service 端下发过来的三个电机 PWM 命令（0~255）
volatile uint8_t gPwmA = 0;
volatile uint8_t gPwmB = 0;
volatile uint8_t gPwmC = 0;

// ---------------- utils ----------------
static inline int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void motorsBrakeAB() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void motorBrakeC() {
  digitalWrite(INC1, LOW);
  digitalWrite(INC2, LOW);
  analogWrite(ENC, 0);
}

void motorsBrakeAll() {
  motorsBrakeAB();
  motorBrakeC();
}

void setMotorPWM(int pwmA, int pwmB) {
  pwmA = clampInt(pwmA, 0, PWM_MAX);
  pwmB = clampInt(pwmB, 0, PWM_MAX);
  analogWrite(ENA, pwmA);
  analogWrite(ENB, pwmB);
}

void setMotorCPWM(int pwmC) {
  pwmC = clampInt(pwmC, 0, PWM_MAX);
  analogWrite(ENC, pwmC);
}

// ---------------- BLE callbacks ----------------
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    deviceConnected = true;
    Serial.println("Connected");
    pclient->setMTU(100);
  }
  void onDisconnect(BLEClient* pclient) override {
    deviceConnected = false;
    Serial.println("Disconnected");
    serverAddress = "";
    motorsBrakeAll();
  }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveName()) {
      String devName = String(advertisedDevice.getName().c_str());
      if (devName == targetDeviceName) {
        String macStr = String(advertisedDevice.getAddress().toString().c_str());
        Serial.print("Found: "); Serial.println(macStr);
        serverAddress = macStr;
        BLEDevice::getScan()->stop();
      }
    }
  }
};

// 接收 8 字节：X(int16 LE), Y(int16 LE), Btn(uint8), PWM_A(uint8), PWM_B(uint8), PWM_C(uint8)
void notifyCB(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length != 8) {
    Serial.printf("Len=%u (expect 8)\n", (unsigned)length);
    return;
  }

  int16_t x = (int16_t)(pData[0] | (pData[1] << 8));
  int16_t y = (int16_t)(pData[2] | (pData[3] << 8));
  uint8_t btn  = pData[4];
  uint8_t pwmA = pData[5];
  uint8_t pwmB = pData[6];
  uint8_t pwmC = pData[7];

  Serial.print("X="); Serial.print(x);
  Serial.print("  Y="); Serial.print(y);
  Serial.print("  BTN="); Serial.print(btn);
  Serial.print("  PWM(A,B,C)=(");
  Serial.print(pwmA); Serial.print(",");
  Serial.print(pwmB); Serial.print(",");
  Serial.print(pwmC); Serial.println(")");

  gJoyX = x;
  gJoyY = y;
  gBtnState = btn;
  gPwmA = pwmA;
  gPwmB = pwmB;
  gPwmC = pwmC;
  gLastRxMs = millis();
}

bool connectToServer() {
  if (serverAddress.length() == 0) return false;

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  Serial.print("Connecting to ");
  Serial.println(serverAddress.c_str());

  if (!pClient->connect(BLEAddress(serverAddress.c_str()))) {
    Serial.println("Connect failed");
    return false;
  }

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (!pRemoteService) {
    Serial.println("Service not found");
    pClient->disconnect();
    return false;
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (!pRemoteCharacteristic) {
    Serial.println("Char not found");
    pClient->disconnect();
    return false;
  }

  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCB);
    Serial.println("Subscribed");
    return true;
  }
  return false;
}

// ---------------- control loop ----------------
// 规则：
// 1) 超时/断线 → 全部刹车
// 2) 如果按钮按下（btn==0）：三个电机一起跑，用 button_state 切前进/反转，PWM 用 A/B/C 命令
// 3) 否则：用摇杆决定方向，PWM 仍然用 A/B/C 命令
void controlByJoystick() {
  uint32_t now = millis();
  if (!deviceConnected || (now - gLastRxMs > RX_TIMEOUT_MS)) {
    motorsBrakeAll();
    return;
  }

  int x = gJoyX;
  int y = gJoyY;
  uint8_t btn = gBtnState;   // 0 = 按下, 1 = 松开
  uint8_t pwmA = gPwmA;
  uint8_t pwmB = gPwmB;
  uint8_t pwmC = gPwmC;

  // ---------------- 按钮边沿检测：只在“按下瞬间”切换一次 ----------------
  if (btn == 0 && lastBtn == 1) {   // 上一次松开，这次变成按下
    button_state = !button_state;   // 0→1 或 1→0 切换方向
    Serial.print("Toggle button_state = ");
    Serial.println(button_state ? "REVERSE" : "FORWARD");
  }
  // 记住这次的按钮状态，给下一次用
  lastBtn = btn;

  // ---------------- 按钮优先：按下时三个电机一起转 ----------------
  if (btn == 0) {   // 按住期间，只按当前 button_state 跑
    if (button_state == 0) {
      // ★ 模式0：三个电机同向“前进”
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // A forward
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // B forward（按你原来写法）
      pwmA = 255;
      pwmB = 255;
      pwmC = 255;
      setMotorPWM(pwmA, pwmB);

      digitalWrite(INC1, HIGH); digitalWrite(INC2, LOW); // C forward
      setMotorCPWM(pwmC);
    } else {
      // ★ 模式1：三个电机同向“反转”
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // A reverse
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // B reverse
      pwmA = pwmA;
      pwmB = pwmB;
      pwmC = pwmC;
      setMotorPWM(pwmA, pwmB);

      digitalWrite(INC1, LOW); digitalWrite(INC2, HIGH); // C reverse
      setMotorCPWM(pwmC);
    }
    return;  // 按钮模式优先，直接结束
  }

  // ---------------- 按钮未按下：摇杆控制 A/B，C 刹车或跟随 ----------------
  motorBrakeC();  // 默认 C 停止，下面转向时再启动

  // 这里沿用你原来的逻辑（只是把固定 255/200 换成下发的 pwmA/pwmB/pwmC）
  if (x > X_HIGH) {
    // 前进：A/B 同向
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    pwmA = pwmA;
    pwmB = pwmB;
    setMotorPWM(pwmA, pwmB);
  }
  else if (x < X_LOW) {
    // 后退：A/B 同向反向
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    pwmA = pwmA;
    pwmB = pwmB;
    setMotorPWM(pwmA, pwmB);
  }
  else {
    // Y 在中间：再看 X 左右
    if (y < Y_LOW) {
      // 左转：一个前一个后（自己按车结构调）
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      pwmA = pwmA - 55;
      pwmB = 255;
      pwmC = pwmC - 55;
      
      setMotorPWM(pwmA, pwmB);

      digitalWrite(INC1, HIGH); digitalWrite(INC2, LOW); // C forward
      setMotorCPWM(pwmC);
    }
    else if (y > Y_HIGH) {
      // 右转：反过来
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
      pwmA = 255;
      pwmB = pwmB - 55;
      pwmC = pwmC - 55;
      setMotorPWM(pwmA, pwmB);

      digitalWrite(INC1, LOW); digitalWrite(INC2, HIGH); // C forward（你可以按需求改方向）
      setMotorCPWM(pwmC);
    }
    else {
      // 中立：A/B 刹车
      motorsBrakeAB();
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("BLE Joystick Client -> L298 (A/B) + Button->3 motors + PWM over BLE");

  // L298 IO
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // Motor C IO
  pinMode(INC1, OUTPUT);
  pinMode(INC2, OUTPUT);
  pinMode(ENC,  OUTPUT);

  motorsBrakeAll();

  // BLE 初始化与扫描
  BLEDevice::init("ESP32_BLE_Client");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(90);
  Serial.println("Scanning...");
  pBLEScan->start(10);
}

void loop() {
  if (!deviceConnected) {
    if (serverAddress.length() > 0) {
      if (connectToServer()) {
        Serial.println("Connected and subscribed.");
        deviceConnected = true;
        gLastRxMs = millis();
      } else {
        Serial.println("Retry in 3s...");
        delay(3000);
      }
    } else {
      BLEScan* pBLEScan = BLEDevice::getScan();
      Serial.println("Rescan 5s...");
      pBLEScan->start(5);
      delay(500);
    }
  }

  controlByJoystick();
  delay(10);
}
