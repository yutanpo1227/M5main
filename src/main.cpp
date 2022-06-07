#define M5STACK_MPU6886
#define CALIBCOUNT 10000
#define SPEED 92
#define PLUS 35
#define PIN 26
#define NUMPIXELS 3
#define PaHub_I2C_ADDRESS	0x70
#define LGFX_AUTODETECT
#define LGFX_USE_V1

#include <M5Stack.h>
#include <Adafruit_NeoPixel.h>
#include "BaseX.h"
#include "Adafruit_TCS34725.h"
#include "ClosedCube_TCA9548A.h"
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include <MadgwickAHRS.h>

Madgwick MadgwickFilter;

static LGFX lcd;
static LGFX_Sprite canvas(&lcd);

ClosedCube::Wired::TCA9548A tca9548a;

BASE_X base_x = BASE_X();
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float yaw   = 0.0F;

float gyroOffsetZ = 1.021;

float preTime = 0.0F;
float dt = 0.0F;

float pregz = 0.0F;
float GYdegree = 0;
float IRdegree = 0;

int cnt = 0;
int cnt1 = 0;
int GrovalCam = 0;

int num = 5;

int ave_motor_power[4][3] = {0};
int ave_mpPlus = 0;

int prevIR, count;
int dirIR = 0;

static uint16_t color16(uint16_t r, uint16_t g, uint16_t b) {
	uint16_t _color;
	_color = (uint16_t)(r & 0xF8) << 8;
	_color |= (uint16_t)(g & 0xFC) << 3;
	_color |= (uint16_t)(b & 0xF8) >> 3;
  return _color;
}

void calibration()
{
  delay(1000);
  M5.Lcd.printf("...");
  float gyroSumZ = 0;
  int count = CALIBCOUNT;
  for (int i = 0; i < count; i++) {
    M5.update();

    float gyroZ;
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);

    gyroSumZ += gyroZ;
    if (M5.BtnB.wasPressed())
    {
      M5.Lcd.clear();
      M5.Lcd.setCursor(140, 120);
      M5.Lcd.printf("Exit");
      delay(500);
      return;
    }
  }
  gyroOffsetZ = gyroSumZ / count - 0.02;
  M5.Lcd.clear();
  M5.Lcd.setCursor(140, 120);
  M5.Lcd.printf("Done");
	M5.update();
  delay(500);
}

// int GetGyro()
// {
//   M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
//   M5.IMU.getAccelData(&accX, &accY, &accZ);
//
//   gyroZ -= gyroOffsetZ;
//
//   dt = (micros() - preTime) / 1000000;
//   preTime = micros();
//
//   yaw -= (pregz + gyroZ) * dt / 2;
//   pregz = gyroZ;
//
//   if (yaw > 180)
//   {
//     yaw -= 360;
//   }
//   else if (yaw < -180)
//   {
//     yaw += 360;
//   }
//   return (int)yaw;
//   delay(10);
// }

int GetGyro()
{
	M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
	M5.IMU.getAccelData(&accX, &accY, &accZ);
	MadgwickFilter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);
	yaw = MadgwickFilter.getYaw();
	if (yaw > 180)
	{
	  yaw -= 360;
	}
	else if (yaw < -180)
	{
	  yaw += 360;
	}
	return (int)yaw;
}

int GetIRval(int f) {
  byte val = 0;
  Wire.beginTransmission(0x0E);
  Wire.write(f);
  Wire.endTransmission();
  Wire.requestFrom(0x0E, 1);
  while (Wire.available()) {
    val = Wire.read();
  }
  return (int)val;
}

int GetIRdir(int i) {
  int a = GetIRval(0x04);
  int b = GetIRval(0x05);
  int c = GetIRval(0x06);
  int d = GetIRval(0x07);
  int re_angle;
  int re_strength;

  if (d < 10) {
    re_angle = a;
    re_strength = b;
  }
  else {
    re_angle = c;
    re_strength = d;
  }

  if (i != 1) {
    return re_strength;
  }
  else {
    return re_angle * 5;
  }
}

int aveIR() {
  int a = 0;
  int num = 5;
  for (int i = 0; i < num; i++) {
    dirIR = GetIRdir(1);
    if (abs(prevIR - dirIR) > 100) {
      count++;
      if (count == 5) {
        count = 0;
        prevIR = dirIR;
      }
      else {
        dirIR = prevIR;
      }
    }
    else {
      count = 0;
      prevIR = dirIR;
    }
    a += dirIR;
  }

  return a / num - 45;
}

void Button()
{
  M5.update();
  if (M5.BtnA.wasPressed())
  {
    cnt--;
    M5.Lcd.clear();
  }

  if (M5.BtnC.wasPressed())
  {
    cnt++;
    M5.Lcd.clear();
  }
}

void Button1()
{
  M5.update();
  if (M5.BtnA.wasPressed())
  {
    cnt1--;
    M5.Lcd.clear();
  }

  if (M5.BtnC.wasPressed())
  {
    cnt1++;
    M5.Lcd.clear();
  }
}

void ResetGyro()
{
	M5.update();
  gyroZ = 0.0;
  pregz = 0.0;
  yaw = 0.0;
  M5.Lcd.clear();
  M5.Lcd.setCursor(120, 120);
  M5.Lcd.printf("RESET");
  delay(500);
  M5.Lcd.clear();
	M5.update();
}

void GetCam()
{
	int re = 0;
	// int a = tca9548a.selectChannel(0);
	// re = a;
	if(Serial2.available())
	{
    int a = Serial2.read();
    if (a >= 0 && a <= 70) {
      GrovalCam = a - 35;
    }
	}
}

void DrawCam()
{
  M5.Lcd.clear();
  while (true)
  {
    M5.update();
    M5.Lcd.setCursor(100, 120);
		GetCam();
    M5.Lcd.printf("x = %4d", GrovalCam);
    if (M5.BtnB.wasPressed())
    {
      M5.Lcd.clear();
      break;
    }
  }
}

void DrawGyro()
{
  int gyro = 0;
  M5.Lcd.clear();
  while (true)
  {
		M5.update();
  	canvas.fillScreen(BLACK);
		gyro = GetGyro();
		GYdegree = (gyro - 90 - 135) / (180 / PI);
		canvas.drawCircle(160, 120, 80, WHITE);
		canvas.fillCircle(160 + 80 * cos(GYdegree), 120 + 80 * sin(GYdegree), 10, GREEN);
		canvas.setCursor(160, 0);
		canvas.printf("%4d", gyro);
		canvas.pushSprite(0,0);
    if (M5.BtnB.wasPressed())
    {
      M5.Lcd.clear();
      break;
    }
  }
}

void TurnRight(int power)
{
  base_x.SetMotorSpeed(1, power);
  base_x.SetMotorSpeed(2, power);
  base_x.SetMotorSpeed(3, power);
  base_x.SetMotorSpeed(4, power);
  delay(1);
}

void TurnLeft(int power)
{
  base_x.SetMotorSpeed(1, -power);
  base_x.SetMotorSpeed(2, -power);
  base_x.SetMotorSpeed(3, -power);
  base_x.SetMotorSpeed(4, -power);
  delay(1);
}

void motor_stop()
{
  base_x.SetMotorSpeed(1, 0);
  base_x.SetMotorSpeed(2, 0);
  base_x.SetMotorSpeed(3, 0);
  base_x.SetMotorSpeed(4, 0);
  delay(1);
}

void motor(int angle)
{
  int gyro = GetGyro();
	GetCam();
  int plus_power = 0;
  int diff = 10;

  if (gyro >= 0 && gyro <= 35)
  {
    plus_power = -GrovalCam;
  }
  else if (gyro <= 0 && gyro >= -35)
  {
    plus_power = -GrovalCam;
  }
  else
  {
    plus_power = PLUS;
  }

  double motor_power[4];
  double max_power;
	angle -= 90;
  motor_power[0] = cos((45 - angle) / 180.0 * PI);
  motor_power[1] = cos((135 - angle) / 180.0 * PI);
  motor_power[2] = cos((-135 - angle) / 180.0 * PI);
  motor_power[3] = cos((-45 - angle) / 180.0 * PI);

  for (int i = 0; i < 4; i++)
  {
    if (abs(motor_power[i]) > max_power)
    {
      max_power = abs(motor_power[i]);
    }
  }

  for (int i = 0; i < 4; i++)
  {
    motor_power[i] = SPEED * motor_power[i] / max_power + plus_power;
    // for (int j = (num - 1); j > 0; j--)
    // {
    //   ave_motor_power[i][j] = ave_motor_power[i][j - 1];
    // }
    // ave_motor_power[i][0] = motor_power[i];
    // ave_mpPlus = 0;
    // for (int k = 0; k < num; k++)
    // {
    //   ave_mpPlus = ave_mpPlus + ave_motor_power[i][k];
    // }
    // motor_power[i] = ave_mpPlus / num;
  }

  base_x.SetMotorSpeed(1, (int)motor_power[0]);
  base_x.SetMotorSpeed(2, (int)motor_power[1]);
  base_x.SetMotorSpeed(3, (int)motor_power[2]);
  base_x.SetMotorSpeed(4, (int)motor_power[3]);
  delay(1);
}

void DrawIR()
{
  int IRval = 0;
  M5.Lcd.clear();
  while (true)
  {
    M5.update();
		canvas.fillScreen(BLACK);
		IRval = aveIR();
		IRdegree = (IRval - 90 - 135) / (180 / PI);
		canvas.drawCircle(160, 120, 80, WHITE);
		canvas.fillCircle(160 + 80 * cos(IRdegree), 120 + 80 * sin(IRdegree), 10, GREEN);
		canvas.setCursor(160, 0);
		canvas.printf("    ");
		canvas.printf("%4d", IRval);
		canvas.pushSprite(0,0);
    if (M5.BtnB.wasPressed())
    {
      M5.Lcd.clear();
      break;
    }
  }
}
int Line()
{
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);
  return clear;
}

void DrawLine()
{
  M5.Lcd.clear();
  while(true)
  {
    M5.update();
    M5.lcd.setCursor(140,120);
    M5.Lcd.printf("%4d",Line());
    if (M5.BtnB.wasPressed())
    {
      M5.Lcd.clear();
      break;
    }
  }
}

void Main()
{
	M5.lcd.clear();
  while(true)
  {
		M5.update();
  	canvas.fillScreen(BLACK);
		int gyro = GetGyro();
		GYdegree = (gyro - 90 - 135) / (180 / PI);
		canvas.drawCircle(160, 120, 80, WHITE);
		canvas.fillCircle(160 + 80 * cos(GYdegree), 120 + 80 * sin(GYdegree), 10, GREEN);
		canvas.setCursor(160, 0);
		canvas.printf("%4d", gyro);
		canvas.pushSprite(0,0);
    if (gyro >= 90)
    {
      while(gyro > 3)
      {
        TurnLeft(112 * gyro / 180 + 15);

		  	canvas.fillScreen(BLACK);
				int gyro = GetGyro();
				GYdegree = (gyro - 90 - 135) / (180 / PI);
				canvas.drawCircle(160, 120, 80, WHITE);
				canvas.fillCircle(160 + 80 * cos(GYdegree), 120 + 80 * sin(GYdegree), 10, GREEN);
				canvas.setCursor(160, 0);
				canvas.printf("%4d", gyro);
				canvas.pushSprite(0,0);
      }
    }
    if (gyro <= -90)
    {
      while(gyro < -3)
      {
        TurnRight(112 * gyro / -180 + 15);

		  	canvas.fillScreen(BLACK);
				int gyro = GetGyro();
				GYdegree = (gyro - 90 - 135) / (180 / PI);
				canvas.drawCircle(160, 120, 80, WHITE);
				canvas.fillCircle(160 + 80 * cos(GYdegree), 120 + 80 * sin(GYdegree), 10, GREEN);
				canvas.setCursor(160, 0);
				canvas.printf("%4d", gyro);
				canvas.pushSprite(0,0);
      }
    }
    else
    {
       int IR = aveIR();
			 if(IR == 0 || IR == 5 || IR == 355)
			 {
	       motor(0);
	     }
	     else
			 {
	        if (IR <= 90)
					{
	           motor(IR + 30);
	        }
	        else if(IR > 90 && IR <= 180)
					{
	           motor(IR + 60);
	        }
	        else if(IR > 180 && IR <= 270)
					{
	           motor(IR - 60);
	        }
	        else if(IR > 270)
					{
	           motor(IR - 30);
	        }
	     }
     }
  }
}


void setup() {

	M5.begin();
	Serial2.begin(115200);
  Wire.begin();
	tca9548a.address(PaHub_I2C_ADDRESS);
  M5.Power.begin();
  tcs.begin();
  tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_2_4MS);
  tcs.setGain(TCS34725_GAIN_16X);
  M5.IMU.Init();
	MadgwickFilter.begin(100);
  base_x.SetMode(1, NORMAL_MODE);
  base_x.SetMode(2, NORMAL_MODE);
  base_x.SetMode(3, NORMAL_MODE);
  base_x.SetMode(4, NORMAL_MODE);

  pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  pixels.setPixelColor(1, pixels.Color(255, 255, 255));
  pixels.setPixelColor(2, pixels.Color(255, 255, 255));
	pixels.show();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE , BLACK);
  M5.Lcd.setTextSize(2);
	lcd.init();
	lcd.setRotation(1);
	canvas.setColorDepth(8);
	canvas.setTextWrap(false);
	canvas.setTextSize(2);
	canvas.createSprite(lcd.width(), lcd.height());
  delay(1);
}


void loop() {
exit1:
	canvas.fillScreen(BLACK);
	canvas.setTextColor(WHITE);
  Button();

  switch (cnt)
  {
    case -1:
      cnt = 3;
      break;
    case 0:
      M5.Lcd.setCursor(140, 120);
      M5.Lcd.printf("Main");
      if (M5.BtnB.wasPressed())
      {
        while(true)
        {
          Main();
        }
      }
      break;
    case 1:
      M5.Lcd.setCursor(140, 120);
      M5.Lcd.printf("Gyro");
      if (M5.BtnB.wasPressed())
      {
        cnt1 = 0;
        M5.Lcd.clear();
        while (true)
        {
          Button1();
          switch (cnt1)
          {
            case -2:
              cnt1 = 0;
            case -1:

              M5.Lcd.setCursor(140, 120);
              M5.Lcd.printf("Exit");
              if (M5.BtnB.wasPressed())
              {
                goto exit1;
              }
              break;
            case 0:

              M5.Lcd.setCursor(100, 120);
              M5.Lcd.printf("DrawGyro");
              if (M5.BtnB.wasPressed())
              {
                DrawGyro();
              }
              break;
            case 1:

              M5.Lcd.setCursor(90, 120);
              M5.Lcd.printf("Calibration");
              if (M5.BtnB.wasPressed())
              {
                calibration();
              }
              break;
            case 2:

              M5.Lcd.setCursor(100, 120);
              M5.Lcd.printf("ResetGyro");
              if (M5.BtnB.wasPressed())
              {
                ResetGyro();
              }
              break;
            case 3:
              cnt1 = 0;
              break;
          }
        }
      }
      break;
    case 2:
      M5.Lcd.setCursor(135, 120);
      M5.Lcd.printf("IR");
      if (M5.BtnB.wasPressed())
      {
        DrawIR();
      }
      break;
    case 3:
      M5.Lcd.setCursor(130, 120);
      M5.Lcd.printf("Cam");
      if (M5.BtnB.wasPressed())
      {
        DrawCam();
      }
      break;
    case 4:
      M5.Lcd.setCursor(130, 120);
      M5.Lcd.printf("Line");
      if (M5.BtnB.wasPressed())
      {
        DrawLine();
      }
      break;
    default:
      cnt = 0;
      break;
  }
}