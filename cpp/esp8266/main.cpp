#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <U8g2lib.h>
#include <time.h>
#include <Servo.h>

/* ================= OLED ================= */
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
  U8G2_R0, 5, 4, U8X8_PIN_NONE
);

/* ================= WiFi ================= */
const char* ssid = "hadlinks_ss";
const char* password = "hadlinks.com";

/* ================= AP ================= */
const char* ap_ssid = "ESP8266_AP";
const char* ap_pass = "1119836254";

/* ================= NTP ================= */
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 8 * 3600, 60000);

/* ================= UDP 监听 ================= */
WiFiUDP udp;
const unsigned int udpPort = 4210;
char udpBuffer[50];

/* ================= 时间变量 ================= */
int hh = 0, mm = 0, ss = 0;
bool timeSynced = false;

/* ================= 定时 ================= */
unsigned long lastTick = 0;
unsigned long lastNTP  = 0;

/* ================= 显示布局 ================= */
#define DATE_Y 14         // 日期基线
#define TIME_BASE_Y 52    // 时间基线

/* ================= 星期（中文） ================= */
const char* weekdays[] = {"日","一","二","三","四","五","六"};

/* ================= 舵机 ================= */
Servo myServo;
const int servoPin = 14; // 舵机信号接 D5

/* ================= 工具函数 ================= */
int toSeconds(int h, int m, int s) {
  return h * 3600 + m * 60 + s;
}

/* ================= UDP 处理 ================= */
void handleUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(udpBuffer, 50);
    if (len > 0) udpBuffer[len] = 0;
    String msg = String(udpBuffer);
    msg.trim();

    // 判断指令是否以 "push:" 开头
    if (msg.startsWith("push:")) {
      String angleStr = msg.substring(5); // 获取冒号后的角度部分
      angleStr.trim();
      int angle = angleStr.toInt();       // 转成整数

      // 限制舵机角度在 0~180
      if (angle < 0) angle = 0;
      if (angle > 180) angle = 180;

      Serial.print("Received push! Angle: ");
      Serial.println(angle);

      // 控制舵机旋转
      myServo.write(angle);
      delay(500);           // 等待舵机到位
      myServo.write(0);     // 回到 0°
    }
  }
}


void setupAP() {
  WiFi.mode(WIFI_AP_STA); // 保持 STA + AP 模式
  WiFi.softAP(ap_ssid, ap_pass);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(udpPort);
  Serial.print("UDP Listening on port: ");
  Serial.println(udpPort);

  myServo.attach(servoPin);
}

void setup() {
  Serial.begin(9600);

  /* ===== OLED 初始化 ===== */
  u8g2.begin();
  u8g2.enableUTF8Print();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub20_tf);
  u8g2.setCursor(0, TIME_BASE_Y);
  u8g2.print("--:--:--");
  u8g2.sendBuffer();

  /* ===== WiFi STA 连接 ===== */
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  timeClient.begin();

  /* ===== 必须等 NTP 成功 ===== */
  while (!timeSynced) {
    if (timeClient.update()) {
      time_t epoch = timeClient.getEpochTime();
      struct tm* t = localtime(&epoch);

      hh = t->tm_hour;
      mm = t->tm_min;
      ss = t->tm_sec;

      // 首次绘制
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_wqy16_t_gb2312b);
      char dateBuf[24];
      sprintf(dateBuf, "%04d-%02d-%02d 周%s",
              t->tm_year + 1900,
              t->tm_mon + 1,
              t->tm_mday,
              weekdays[t->tm_wday]);
      u8g2.setCursor(0, DATE_Y);
      u8g2.print(dateBuf);

      u8g2.setFont(u8g2_font_fub20_tf);
      char timeBuf[9];
      sprintf(timeBuf, "%02d:%02d:%02d", hh, mm, ss);
      u8g2.setCursor(0, TIME_BASE_Y);
      u8g2.print(timeBuf);

      u8g2.sendBuffer();
      timeSynced = true;
    } else {
      delay(500);
    }
  }

  lastTick = millis();
  lastNTP  = millis();

  /* ===== 启动 AP + UDP + 舵机 ===== */
  setupAP();
}

void loop() {
  if (!timeSynced) return;

  unsigned long now = millis();

  /* ===== 每秒走时（只刷新时间区域） ===== */
  if (now - lastTick >= 1000) {
    lastTick = now;

    ss++;
    if (ss >= 60) { ss = 0; mm++; }
    if (mm >= 60) { mm = 0; hh++; }
    if (hh >= 24) { hh = 0; }

    char timeBuf[9];
    sprintf(timeBuf, "%02d:%02d:%02d", hh, mm, ss);

    // 擦除时间区域
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, 32, 128, 24);
    u8g2.setDrawColor(1);

    // 重绘时间
    u8g2.setFont(u8g2_font_fub20_tf);
    u8g2.setCursor(0, TIME_BASE_Y);
    u8g2.print(timeBuf);
    u8g2.sendBuffer();
  }

  /* ===== 120 秒 NTP 校准 ===== */
  if (now - lastNTP >= 120000) {
    lastNTP = now;

    if (timeClient.update()) {
      time_t epoch = timeClient.getEpochTime();
      struct tm* t = localtime(&epoch);

      int ntpSec   = toSeconds(t->tm_hour, t->tm_min, t->tm_sec);
      int localSec = toSeconds(hh, mm, ss);

      if (abs(ntpSec - localSec) > 2) {
        hh = t->tm_hour;
        mm = t->tm_min;
        ss = t->tm_sec;
      }

      // 刷新日期 + 星期
      u8g2.setDrawColor(0);
      u8g2.drawBox(0, 0, 128, 20);
      u8g2.setDrawColor(1);

      u8g2.setFont(u8g2_font_wqy16_t_gb2312b);
      char dateBuf[24];
      sprintf(dateBuf, "%04d-%02d-%02d 周%s",
              t->tm_year + 1900,
              t->tm_mon + 1,
              t->tm_mday,
              weekdays[t->tm_wday]);
      u8g2.setCursor(0, DATE_Y);
      u8g2.print(dateBuf);
      u8g2.sendBuffer();
    }
  }

  /* ===== UDP 检测 ===== */
  handleUDP();
}
