// ====================== Joystick BLE Server (手柄板) ======================
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ---- Device name & UUIDs (和 Client 必须一致) ----
static const char* DEVICE_NAME = "ESP32_BLE_Server";
static BLEUUID serviceUUID("12345678-1234-1234-1234-123456789ABC");
static BLEUUID charUUID   ("ABCDEF12-3456-7890-1234-567890ABCDEF");

// ---- Joystick pins (classic ESP32 ADC1) ----
static const int JOY_X_PIN = 34;
static const int JOY_Y_PIN = 35;

// ---- Button pin（改成你实际用的 IO）----
static const int BTN_PIN   = 33;   // 按钮，一端接 GND，使用 INPUT_PULLUP

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
volatile bool deviceConnected = false;

// ======== 三个电机的 PWM 命令（0~255）========
uint8_t pwmA = 255;
uint8_t pwmB = 255;
uint8_t pwmC = 255;

// 小端写 int16
static inline void put_i16_le(uint8_t* dst, int16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    Serial.println("[Server] Client connected");
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    Serial.println("[Server] Client disconnected, restart adv");
    s->getAdvertising()->start(); // 允许重连
  }
};

// ========= 串口解析：一行命令形如 A255 / B120 / C80 =========
void handleSerialPWMInput() {
  static String line = "";

  while (Serial.available() > 0) {
    char c = Serial.read();

    // 换行（串口监视器点一次“发送”通常会带 \r 或 \n）→ 一条命令结束
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        char motor = toupper(line[0]);   // A / B / C
        String numStr = line.substring(1);
        numStr.trim();

        int val = numStr.toInt();
        if (val < 0)   val = 0;
        if (val > 255) val = 255;

        switch (motor) {
          case 'A': pwmA = (uint8_t)val; break;
          case 'B': pwmB = (uint8_t)val; break;
          case 'C': pwmC = (uint8_t)val; break;
          default:
            Serial.print("[PWM] Unknown motor: ");
            Serial.println(motor);
            break;
        }

        Serial.printf("[PWM CMD] %c = %d (A=%u, B=%u, C=%u)\n",
                      motor, val, pwmA, pwmB, pwmC);

        line = "";
      }
    } else {
      // 只收数字和字母，避免奇怪字符
      if (isAlphaNumeric(c)) {
        line += c;
      }
    }

    // 行太长就丢弃，防止乱码卡死
    if (line.length() > 10) {
      line = "";
    }
  }
}

void sendJoystick2() {
  // 读取摇杆
  uint16_t x = (uint16_t)analogRead(JOY_X_PIN); // 0..4095
  uint16_t y = (uint16_t)analogRead(JOY_Y_PIN);

  // 读取按钮：按下=LOW，松开=HIGH
  int raw = digitalRead(BTN_PIN);
  // 保持原来的约定：0 = 按下, 1 = 松开
  uint8_t btn = (raw == LOW) ? 0 : 1;

  // 打包：
  // X(int16 LE) + Y(int16 LE) + Btn(1 byte) + PWM_A + PWM_B + PWM_C  共 8 字节
  uint8_t pkt[8];
  put_i16_le(&pkt[0], (int16_t)x);
  put_i16_le(&pkt[2], (int16_t)y);
  pkt[4] = btn;
  pkt[5] = pwmA;
  pkt[6] = pwmB;
  pkt[7] = pwmC;

  pCharacteristic->setValue(pkt, sizeof(pkt));
  pCharacteristic->notify();

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last > 500) {
    last = now;
    Serial.printf("[Srv] X=%u Y=%u BTN=%u  PWM(A,B,C)=(%u,%u,%u)\n",
                  x, y, btn, pwmA, pwmB, pwmC);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== Joystick BLE Server with A/B/C PWM ===");
  Serial.println("在串口输入一行，例如: A255 或 B120 或 C80，然后点发送。");

  // ADC 设置
  analogReadResolution(12);
  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);

  // 按钮：内置上拉，按下时接地
  pinMode(BTN_PIN, INPUT_PULLUP);

  // BLE 初始化
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(100);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* service = pServer->createService(serviceUUID);
  pCharacteristic = service->createCharacteristic(
      charUUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  auto* cccd = new BLE2902();
  cccd->setNotifications(true);
  pCharacteristic->addDescriptor(cccd);

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(serviceUUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[Server] Advertising as ESP32_BLE_Server");
}

void loop() {
  // 处理串口 PWM 命令
  handleSerialPWMInput();

  static uint32_t last = 0;
  uint32_t now = millis();

  // ~50 Hz 发数据
  if (deviceConnected && (now - last >= 20)) {
    last = now;
    sendJoystick2();
  }
}
