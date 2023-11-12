#include "mcp2515_can.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

#define DISPLAY_INTERVAL_RAW 500
#define DISPLAY_INTERVAL_FAST 40
#define DISPLAY_INTERVAL_SLOW 1000
#define MINPRESSURE 100
#define MAXPRESSURE 1000
#define MAX_DATA_SIZE 8
#define DEBUG false
#define DEBUG_SKIP_CAN false

#define CAR_Y 160

// RGB565 https://github.com/newdigate/rgb565_colors#gray
#define WHITE 0xFFFF
#define RED   0xF800
#define BLUE  0x001F
#define GREEN 0x07E0
#define BLACK 0x0000

const int SPI_CS_PIN = 53;
const int CAN_INT_PIN = 21;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
MCUFRIEND_kbv tft;

const int XP = 8, XM = A2, YP = A3, YM = 9;
const int TS_LEFT = 957, TS_RT = 203, TS_TOP = 902, TS_BOT = 212;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Adafruit_GFX_Button logo_btn, car_btn;

uint16_t readID(void) {
    uint16_t ID = tft.readID();
    if (ID == 0xD3D3) ID = 0x9486;
    return ID;
}

int pixel_x, pixel_y;
bool Touch_getXY(void)
{
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
    if (pressed) {
        pixel_x = map(p.y, TS_LEFT, TS_RT, 0, tft.width()); //.kbv makes sense to me
        pixel_y = map(p.x, TS_TOP, TS_BOT, 0, tft.height());
    }
    return pressed;
}

// void initButton(Adafruit_GFX *gfx, int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t outline, uint16_t fill, uint16_t textcolor, char *label, uint8_t textsize);
// void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
// void drawFastVLine(uint16_t x0, uint16_t y0, uint16_t length, uint16_t color);
// void drawFastHLine(uint8_t x0, uint8_t y0, uint8_t length, uint16_t color);
// void drawRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t color);
// void fillRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t color);
// void drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
// void fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);

void setup() {
  SERIAL_PORT_MONITOR.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  while (!SERIAL_PORT_MONITOR) {}
  digitalWrite(LED_BUILTIN, LOW);
  uint16_t ID = readID();
  tft.begin(ID);
  tft.fillScreen(BLACK);
  tft.setRotation(1);

  int count = 0;
  if (!DEBUG_SKIP_CAN) {
    while (CAN_OK != CAN.begin(CAN_1000KBPS)) {
      SERIAL_PORT_MONITOR.println(F("CAN init fail, retry..."));
      tft.println("CAN init fail, retry...");
      delay(100);
      count++;
      if (count == 40) {
        count = 0;
        tft.fillScreen(BLACK);
        tft.setCursor(0, 0);
      }
    }
  }
  SERIAL_PORT_MONITOR.println(F("CAN init ok!"));
  tft.println("CAN init ok!");
  tft.fillScreen(BLACK);

  pinMode(CAN_INT_PIN, INPUT); // Setting pin 2, MCP2515 /INT, to input mode  NB!!!!  

  char empty = ' ';
  logo_btn.initButton(&tft, 240, 25, 88, 44, BLACK, BLACK, BLACK, &empty, 1);
  logo_btn.drawButton(false);
  car_btn.initButton(&tft, 208, CAR_Y, 64, 145, BLACK, BLACK, BLACK, &empty, 1);
  car_btn.drawButton(false);

  tft.setTextColor(WHITE, BLACK);
  init_rpm_graph();
  init_kmh_unit();
  init_fuel_indicator();
  init_coolant_indicator();
  draw_car();
  draw_mini_logo();
  digitalWrite(LED_BUILTIN, HIGH);
}

uint32_t id;
uint8_t len;
byte cdata[MAX_DATA_SIZE] = {0};
uint8_t cdata_size = sizeof(cdata);
unsigned long time_now_raw = 0;
unsigned long time_now_fast = 0;
unsigned long time_now_slow = 0;

// https://www.ms4x.net/index.php?title=Siemens_MS43_CAN_Bus
byte cdataASC1[MAX_DATA_SIZE] = {0}; // 0x153 ASC1 Traction Control - 20ms -> kmh
byte cdataASC2[MAX_DATA_SIZE] = {0}; // 0x1F0 ASC2 Traction Control - 20ms -> wheel kmh
byte cdataASC4[MAX_DATA_SIZE] = {0}; // 0x1F8 ASC4 Traction Control - 20ms
byte cdataDME1[MAX_DATA_SIZE] = {0}; // 0x316 DME1 Engine Control Unit - 10ms -> rpm
byte cdataDME2[MAX_DATA_SIZE] = {0}; // 0x329 DME2 Engine Control Unit - 10ms -> coolant
byte cdataDME4[MAX_DATA_SIZE] = {0}; // 0x545 DME4 Engine Control Unit - 10ms
byte cdataICL2[MAX_DATA_SIZE] = {0}; // 0x613 ICL2 Instrument Cluster - 200ms -> fuel
byte cdataICL3[MAX_DATA_SIZE] = {0}; // 0x615 ICL3 Instrument Cluster - 200ms
byte cdataEGS1[MAX_DATA_SIZE] = {0}; // 0x43F EGS1 Automatic Transmission - 10ms

byte cdataICLA[MAX_DATA_SIZE] = {0}; // 0x61A Instrument Cluster Stuff
byte cdataICLB[MAX_DATA_SIZE] = {0}; // 0x61F Instrument Cluster Stuff
byte cdataIDK0[MAX_DATA_SIZE] = {0}; // 0x336 IDK

uint8_t kmh = 0;
uint8_t fuel = 0;
uint8_t coolant = 0;
uint16_t rpm = 0;

uint8_t kmh_wheel1 = 0;
uint8_t kmh_wheel2 = 0;
uint8_t kmh_wheel3 = 0;
uint8_t kmh_wheel4 = 0;

void loop() {
  bool down = Touch_getXY();
  logo_btn.press(down && logo_btn.contains(pixel_x, pixel_y));
  if (logo_btn.justReleased())
    toggle_raw_data();
    
  if (car_btn.justReleased())
    toggle_detailed_data();
    
#if !DEBUG_SKIP_CAN
  // check if data coming
  // if(!digitalRead(2)) { // If pin 2 is low, read receive buffer
  if (CAN_MSGAVAIL != CAN.checkReceive()) {
    return;
  }

  //unsigned long t = millis();
  // read data, len: data length, buf: data buf
  CAN.readMsgBuf(&len, cdata);

  if (len > 8) {
    return;
  }
    
  if (((CAN.isExtendedFrame() << 0) |
      (CAN.isRemoteRequest() << 1)) != 0) {
    return;
  }
    
  id = CAN.getCanId();
  
  switch (id) {
    case 339: // 00000153
      memcpy(cdataASC1, cdata, cdata_size);
      break;
    case 496: // 000001F0
      memcpy(cdataASC2, cdata, cdata_size);
      break;
    case 504: // 000001F8
      memcpy(cdataASC4, cdata, cdata_size);
      break;
    case 790: // 00000316
      memcpy(cdataDME1, cdata, cdata_size);
      break;
    case 809: // 00000329
      memcpy(cdataDME2, cdata, cdata_size);
      break;
    case 822: // 00000336
      memcpy(cdataIDK0, cdata, cdata_size);
      break;
    case 1087: // 0000043F
      memcpy(cdataEGS1, cdata, cdata_size);
      break;
    case 1349: // 00000545
      memcpy(cdataDME4, cdata, cdata_size);
      break;
    case 1555: // 00000613
      memcpy(cdataICL2, cdata, cdata_size);
      break;
    case 1557: // 00000615
      memcpy(cdataICL3, cdata, cdata_size);
      break;
    case 1562: // 0000061A
      memcpy(cdataICLA, cdata, cdata_size);
      break;
    case 1567: // 0000061F
      memcpy(cdataICLB, cdata, cdata_size);
      break;
    default:
      return;
      break;
  }
#endif

#if DEBUG
  char prbuf[32];
  sprintf(prbuf, "%08lX %02X %02X %02X %02X %02X %02X %02X %02X", (unsigned long)id, cdata[0], cdata[1], cdata[2], cdata[3], cdata[4], cdata[5], cdata[6], cdata[7]);
  SERIAL_PORT_MONITOR.println(prbuf);
#endif

  if ((unsigned long)(millis() - time_now_raw) > DISPLAY_INTERVAL_RAW)
  {
    display_raw_can();
    update_wheel_kmh();
    time_now_raw = millis();
  }
  if ((unsigned long)(millis() - time_now_fast) > DISPLAY_INTERVAL_FAST)
  {
    update_kmh();
    update_rpm();
    time_now_fast = millis();
  }
  if ((unsigned long)(millis() - time_now_slow) > DISPLAY_INTERVAL_SLOW)
  {
    update_fuel();
    update_coolant();
    time_now_slow = millis();
  }
}

void update_kmh()
{
  kmh = (uint8_t)(((cdataASC1[2] * 256) + cdataASC1[1]) / 128);
  tft.setTextSize(3);
  tft.setCursor(395, 110);
  if (kmh > 130) 
    tft.setTextColor(RED, BLACK);
  char display_speed[3];
  sprintf(display_speed, "%3d", kmh);
  tft.println(display_speed);
  tft.setTextColor(WHITE, BLACK);
}

void init_kmh_unit()
{
  tft.setTextSize(3);
  tft.setCursor(395, 110);
  tft.println("  0");
  tft.setTextSize(1);
  tft.setCursor(450, 124);
  tft.println("km/h");
}

void update_rpm()
{
  rpm = (uint16_t)((cdataDME1[2] + (cdataDME1[3] * 256)) / 6.4);
  tft.setTextSize(2);
  tft.setCursor(16, 70);
  char display_rpm[4];
  sprintf(display_rpm, "%d", rpm);
  tft.print(display_rpm);
  if (rpm < 1000)
    tft.print(" ");
  if (rpm < 100)
    tft.print(" ");
  if (rpm < 10)
    tft.print(" ");
  color_rpm_graph(rpm);
}

void init_rpm_graph()
{
  tft.setTextSize(2);
  tft.setCursor(16, 70);
  tft.println("0");
  for (int i = 18; i <= 312; i+=14)
  {
    tft.drawRect(i, rpm_ellipse_calc_y(i), 10, 20, WHITE);
  }
  for (int i = 326; i <= 452; i+=14)
  {
    tft.drawRect(i, rpm_ellipse_calc_y(i), 10, 20, RED);
  }
  tft.setTextSize(1);
  for (int i = 1; i <= 8; i++) {
    int x = (((i * 4) - 1) * 14) + 18;
    tft.setCursor(x + 1, rpm_ellipse_calc_y(x) + 25);
    tft.println(i);
  }
  tft.setCursor(16, 86);
  tft.println("rpm");
}

//  (x - 464) ^ 2     (y - 120) ^ 2
// --------------- + --------------- = 1
//     200704             3600
int rpm_ellipse_calc_y(int x)
{
  return (120 - sqrt((1- (pow(x - 464, 2) / 200704)) * 3600));
}

uint8_t last_rect = 0;
void color_rpm_graph(uint16_t rpm)
{
  uint8_t rect = rpm / 250;
  if (rect > last_rect) 
  {
    for (int i = last_rect; i < rect; i++)
    {
      int x = (i * 14) + 18;
      tft.fillRect(x + 1, rpm_ellipse_calc_y(x) + 1, 8, 18, (i >= 22) ? RED : WHITE);
    }
  }
  if (rect < last_rect) 
  {
    for (int i = last_rect; i >= rect; i--)
    {
      int x = (i * 14) + 18;
      tft.fillRect(x + 1, rpm_ellipse_calc_y(x) + 1, 8, 18, BLACK);
    }
  }
  last_rect = rect;
}

void update_fuel()
{
  fuel = (uint8_t)((cdataICL2[2] > 128) ? cdataICL2[2] - 128 : cdataICL2[2]);
  tft.setTextSize(2);
  tft.setCursor(428, 288);
  if (fuel <= 7) 
    tft.setTextColor(RED, BLACK);
  char display_fuel[3];
  sprintf(display_fuel, "%2dL", fuel);
  tft.println(display_fuel);
  tft.setTextColor(WHITE, BLACK);
}

void init_fuel_indicator()
{
  tft.setTextSize(2);
  tft.setCursor(428, 288);
  tft.println(" 0L");
  tft.setTextSize(1);
  tft.setCursor(440, 304);
  tft.println("fuel");
}

void update_coolant()
{
  coolant = (uint8_t)((cdataDME2[1] * 0.75) - 48.373);
  tft.setTextSize(2);
  tft.setCursor(16, 288);
  if (coolant >= 110) 
    tft.setTextColor(RED, BLACK);
  char display_coolant[3];
  sprintf(display_coolant, "%d", coolant);
  tft.print(coolant);
  tft.write(0xF7);
  tft.print(" ");
  tft.setTextColor(WHITE, BLACK);
}

void init_coolant_indicator()
{
  tft.setTextSize(2);
  tft.setCursor(16, 288);
  tft.print("0");
  tft.write(0xF7);
  tft.setTextSize(1);
  tft.setCursor(16, 304);
  tft.println("coolant");
}

bool show_detailed_data = false;
void toggle_detailed_data()
{
  show_detailed_data = !show_detailed_data;
}

void update_wheel_kmh()
{
  if (!show_detailed_data) return;
  kmh_wheel1 = (uint8_t)((cdataASC2[0] + (cdataASC2[1] * 256)) / 16); // LF
  kmh_wheel2 = (uint8_t)((cdataASC2[2] + (cdataASC2[3] * 256)) / 16); // RF
  kmh_wheel3 = (uint8_t)((cdataASC2[4] + (cdataASC2[5] * 256)) / 16); // LR
  kmh_wheel4 = (uint8_t)((cdataASC2[6] + (cdataASC2[7] * 256)) / 16); // RR
}

bool show_raw_data = false;
uint8_t raw_data_iteration = 0;
void toggle_raw_data()
{
  show_raw_data = !show_raw_data;
  if (show_raw_data) {
    init_raw_can_caption();
    raw_data_iteration = 0;
  } else {
    tft.fillRect(2, 2, 174, 50, BLACK);
    tft.fillRect(304, 2, 174, 50, BLACK);
  }
}

void init_raw_can_caption()
{
  tft.setTextSize(1);
  tft.setCursor(2, 2);
  tft.println("0x153 00 00 00 00 00 00 00 00");
  tft.setCursor(2, 10);
  tft.println("0x1F0 00 00 00 00 00 00 00 00");
  tft.setCursor(2, 18);
  tft.println("0x1F8 00 00 00 00 00 00 00 00");
  tft.setCursor(2, 26);
  tft.println("0x316 00 00 00 00 00 00 00 00");
  tft.setCursor(2, 34);
  tft.println("0x329 00 00 00 00 00 00 00 00");
  tft.setCursor(2, 42);
  tft.println("0x545 00 00 00 00 00 00 00 00");
  
  tft.setCursor(304, 2);
  tft.println("0x613 00 00 00 00 00 00 00 00");
  tft.setCursor(304, 10);
  tft.println("0x615 00 00 00 00 00 00 00 00");
  tft.setCursor(304, 18);
  tft.println("0x43F 00 00 00 00 00 00 00 00");
  tft.setCursor(304, 26);
  tft.println("0x61A 00 00 00 00 00 00 00 00");
  tft.setCursor(304, 34);
  tft.println("0x61F 00 00 00 00 00 00 00 00");
  tft.setCursor(304, 42);
  tft.println("0x336 00 00 00 00 00 00 00 00");
}

void display_raw_can()
{
  if (!show_raw_data) return;
  char prbuf[23];
  tft.setTextSize(1);
  switch (raw_data_iteration)
  {
    case 0:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataASC1[0], cdataASC1[1], cdataASC1[2], cdataASC1[3], cdataASC1[4], cdataASC1[5], cdataASC1[6], cdataASC1[7]);
      tft.setCursor(38, 2);
      break;
    case 1:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataASC2[0], cdataASC2[1], cdataASC2[2], cdataASC2[3], cdataASC2[4], cdataASC2[5], cdataASC2[6], cdataASC2[7]);
      tft.setCursor(38, 10);
      break;
    case 2:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataASC4[0], cdataASC4[1], cdataASC4[2], cdataASC4[3], cdataASC4[4], cdataASC4[5], cdataASC4[6], cdataASC4[7]);
      tft.setCursor(38, 18);
      break;
    case 3:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataDME1[0], cdataDME1[1], cdataDME1[2], cdataDME1[3], cdataDME1[4], cdataDME1[5], cdataDME1[6], cdataDME1[7]);
      tft.setCursor(38, 26);
      break;
    case 4:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataDME2[0], cdataDME2[1], cdataDME2[2], cdataDME2[3], cdataDME2[4], cdataDME2[5], cdataDME2[6], cdataDME2[7]);
      tft.setCursor(38, 34);
      break;
    case 5:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataDME4[0], cdataDME4[1], cdataDME4[2], cdataDME4[3], cdataDME4[4], cdataDME4[5], cdataDME4[6], cdataDME4[7]);
      tft.setCursor(38, 42);
      break;
    case 6:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataICL2[0], cdataICL2[1], cdataICL2[2], cdataICL2[3], cdataICL2[4], cdataICL2[5], cdataICL2[6], cdataICL2[7]);
      tft.setCursor(340, 2);
      break;
    case 7:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataICL3[0], cdataICL3[1], cdataICL3[2], cdataICL3[3], cdataICL3[4], cdataICL3[5], cdataICL3[6], cdataICL3[7]);
      tft.setCursor(340, 10);
      break;
    case 8:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataEGS1[0], cdataEGS1[1], cdataEGS1[2], cdataEGS1[3], cdataEGS1[4], cdataEGS1[5], cdataEGS1[6], cdataEGS1[7]);
      tft.setCursor(340, 18);
      break;
    case 9:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataICLA[0], cdataICLA[1], cdataICLA[2], cdataICLA[3], cdataICLA[4], cdataICLA[5], cdataICLA[6], cdataICLA[7]);
      tft.setCursor(340, 26);
      break;
    case 10:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataICLB[0], cdataICLB[1], cdataICLB[2], cdataICLB[3], cdataICLB[4], cdataICLB[5], cdataICLB[6], cdataICLB[7]);
      tft.setCursor(340, 34);
      break;
    case 11:
      sprintf(prbuf, "%02X %02X %02X %02X %02X %02X %02X %02X", cdataIDK0[0], cdataIDK0[1], cdataIDK0[2], cdataIDK0[3], cdataIDK0[4], cdataIDK0[5], cdataIDK0[6], cdataIDK0[7]);
      tft.setCursor(340, 42);
      break;
    default:
      break;
  }
  tft.println(prbuf);

  raw_data_iteration++;
  if (raw_data_iteration > 11)
    raw_data_iteration = 0;
}

void draw_mini_logo()
{
  tft.fillCircle(240, 25, 20, WHITE);
  tft.fillCircle(240, 25, 17, BLACK);
  tft.setCursor(229, 22);
  tft.setTextSize(1);
  tft.println("MINI");

  tft.drawFastHLine(196, 15, 27, WHITE); tft.drawFastHLine(257, 15, 27, WHITE);
  tft.drawFastHLine(197, 16, 25, WHITE); tft.drawFastHLine(258, 16, 25, WHITE);
  tft.drawFastHLine(198, 17, 24, WHITE); tft.drawFastHLine(258, 17, 24, WHITE);

  tft.drawFastHLine(202, 21, 18, WHITE); tft.drawFastHLine(260, 21, 18, WHITE);
  tft.drawFastHLine(203, 22, 17, WHITE); tft.drawFastHLine(260, 22, 17, WHITE);
  tft.drawFastHLine(204, 23, 16, WHITE); tft.drawFastHLine(260, 23, 16, WHITE);

  tft.drawFastHLine(208, 27, 12, WHITE); tft.drawFastHLine(260, 27, 12, WHITE);
  tft.drawFastHLine(209, 28, 11, WHITE); tft.drawFastHLine(260, 28, 11, WHITE);
  tft.drawFastHLine(210, 29, 10, WHITE); tft.drawFastHLine(260, 29, 10, WHITE);
  
  tft.drawFastHLine(214, 33, 8, WHITE); tft.drawFastHLine(258, 33, 8, WHITE);
  tft.drawFastHLine(215, 34, 7, WHITE); tft.drawFastHLine(258, 34, 7, WHITE);
  tft.drawFastHLine(216, 35, 7, WHITE); tft.drawFastHLine(257, 35, 7, WHITE);
}

void draw_car()
{
  tft.fillRoundRect(208, CAR_Y, 64, 145, 20, WHITE);
  tft.fillRoundRect(212, CAR_Y + 40, 56, 100, 20, BLACK);
  tft.drawRoundRect(218, CAR_Y + 60, 44, 70, 16, WHITE);
  tft.drawRoundRect(219, CAR_Y + 61, 42, 68, 15, WHITE);
  tft.fillRect(230, CAR_Y + 2, 8, 36, BLACK);
  tft.fillRect(242, CAR_Y + 2, 8, 36, BLACK);
  tft.drawCircle(219, CAR_Y + 10, 4, BLACK);
  tft.drawCircle(219, CAR_Y + 10, 5, BLACK);
  tft.drawCircle(260, CAR_Y + 10, 4, BLACK);
  tft.drawCircle(260, CAR_Y + 10, 5, BLACK);
  tft.drawLine(214, CAR_Y + 134, 221, CAR_Y + 139, RED);
  tft.drawLine(214, CAR_Y + 135, 221, CAR_Y + 140, RED);
  tft.drawLine(214, CAR_Y + 136, 221, CAR_Y + 141, RED);
  tft.drawLine(266, CAR_Y + 134, 259, CAR_Y + 139, RED);
  tft.drawLine(266, CAR_Y + 135, 259, CAR_Y + 140, RED);
  tft.drawLine(266, CAR_Y + 136, 259, CAR_Y + 141, RED);
  tft.fillTriangle(207, CAR_Y + 40, 207, CAR_Y + 43, 203, CAR_Y + 43, WHITE);
  tft.fillTriangle(272, CAR_Y + 40, 272, CAR_Y + 43, 276, CAR_Y + 43, WHITE);
}
