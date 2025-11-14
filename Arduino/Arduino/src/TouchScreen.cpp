// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// (c) ladyada / adafruit
// Code under MIT License

// #include "pins_arduino.h"
// #include "wiring_private.h"
// #include <avr/pgmspace.h>
#include "TouchScreen.h"

// increase or decrease the touchscreen oversampling. This is a little different than you make think:
// 1 is no oversampling, whatever data we get is immediately returned
// 2 is double-sampling and we only return valid data if both points are the same
// 3+ uses insert sort to get the median value.
// We found 2 is precise yet not too slow so we suggest sticking with it!

#define NUMSAMPLES 64

TSPoint::TSPoint(void)
{
  x = y = 0;
}

TSPoint::TSPoint(int16_t x0, int16_t y0, int16_t z0)
{
  x = x0;
  y = y0;
  z = z0;
}

bool TSPoint::operator==(TSPoint p1)
{
  return ((p1.x == x) && (p1.y == y) && (p1.z == z));
}

bool TSPoint::operator!=(TSPoint p1)
{
  return ((p1.x != x) || (p1.y != y) || (p1.z != z));
}

#if (NUMSAMPLES > 2)
static void insert_sort(int array[], uint8_t size)
{
  uint8_t j;
  int save;

  for (int i = 1; i < size; i++)
  {
    save = array[i];
    for (j = i; j >= 1 && save < array[j - 1]; j--)
      array[j] = array[j - 1];
    array[j] = save;
  }
}
#endif

TSPoint TouchScreen::calPoint(TSPoint pRaw, TSPoint *pRawRef, TSPoint *pCalRef, TSPoint pMaxRef)
{
  uint16_t x_raw_pos = pRaw.x;
  uint16_t x_raw_min = (pRawRef[0].x + pRawRef[2].x) / 2;
  uint16_t x_raw_max = (pRawRef[1].x + pRawRef[3].x) / 2;

  uint16_t x_cal_min = (pCalRef[0].x + pCalRef[2].x) / 2;
  uint16_t x_cal_max = (pCalRef[1].x + pCalRef[3].x) / 2;

  int16_t x_cal_pos = map(x_raw_pos, x_raw_min, x_raw_max, x_cal_min, x_cal_max);

  if (x_cal_pos > pMaxRef.x)
  {
    x_cal_pos = pMaxRef.x;
  }
  else if (x_cal_pos < 0)
  {
    x_cal_pos = 0;
  }

  uint16_t y_raw_pos = pRaw.y;
  uint16_t y_raw_min = (pRawRef[0].y + pRawRef[1].y) / 2;
  uint16_t y_raw_max = (pRawRef[2].y + pRawRef[3].y) / 2;

  uint16_t y_cal_min = (pCalRef[0].y + pCalRef[1].y) / 2;
  uint16_t y_cal_max = (pCalRef[2].y + pCalRef[3].y) / 2;

  int16_t y_cal_pos = map(y_raw_pos, y_raw_min, y_raw_max, y_cal_min, y_cal_max);

  if (y_cal_pos > pMaxRef.y)
  {
    y_cal_pos = pMaxRef.y;
  }
  else if (y_cal_pos < 0)
  {
    y_cal_pos = 0;
  }

  return TSPoint(x_cal_pos, y_cal_pos, 0);
}

bool TouchScreen::penFound(void)
{
  // Disable XR to VCC
  digitalWrite(_p_XR, HIGH);

  // Set XL to ground
  digitalWrite(_p_XL, HIGH);

  // Disable YU to ground
  digitalWrite(_p_YU, LOW);

  // Disable YD to VCC
  digitalWrite(_p_YD, HIGH);

  // Activate the T_HOa pin
  pinMode(_p_THOa, OUTPUT);
  digitalWrite(_p_THOa, HIGH);

  if (analogRead(_yu) > 512)
  {
    return false;
  }

  // Disable the T_HOa pin
  pinMode(_p_THOa, INPUT);
  digitalWrite(_p_THOa, LOW);

  return true;
}

TSPoint TouchScreen::getPoint(void)
{
  int x, y, z;
  int samples[NUMSAMPLES];
  uint8_t i;
  uint8_t valid = 1;

  // Set XL to ground
  digitalWrite(_p_XL, HIGH);

  // Set XR to VCC
  digitalWrite(_p_XR, LOW);

  // Disable YU to ground
  digitalWrite(_p_YU, LOW);

  // Disable YD to VCC
  digitalWrite(_p_YD, HIGH);

  // uint16_t adc1 = 0;

  for (i = 0; i < NUMSAMPLES; i++)
  {
    samples[i] = analogRead(_yu);

    // adc1 += analogRead(_yu);
  }

  // adc1 /= NUMSAMPLES;

  // Disable XL to ground
  digitalWrite(_p_XL, LOW);

  // Disable XR to VCC
  digitalWrite(_p_XR, HIGH);

#if NUMSAMPLES > 2
  insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
  if (samples[0] != samples[1])
  {
    valid = 0;
  }
#endif

  for (i = (NUMSAMPLES / 4); i < ((NUMSAMPLES / 4) * 3); i++)
  {
    if (samples[i] == samples[i + 1])
    {
      x = samples[i];
      break;
    }
  }

  // adc1 += analogRead(_yu);
  // x = (1023 - samples[NUMSAMPLES / 2]);
  // x = samples[NUMSAMPLES / 2];

  // x = adc1;

  // Set YU to ground
  digitalWrite(_p_YU, HIGH);

  // Set YD to VCC
  digitalWrite(_p_YD, LOW);

  // Disable XL to ground
  digitalWrite(_p_XL, LOW);

  // Disable XR to VCC
  digitalWrite(_p_XR, HIGH);

  // adc1 = 0;

  for (i = 0; i < NUMSAMPLES; i++)
  {
    samples[i] = analogRead(_xr);
    // adc1 += analogRead(_xr);
  }

  // adc1 /= NUMSAMPLES;

  // Disable YU to ground
  digitalWrite(_p_YU, LOW);

  // Disable YD to VCC
  digitalWrite(_p_YD, HIGH);

#if NUMSAMPLES > 2
  insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
  if (samples[0] != samples[1])
  {
    valid = 0;
  }
#endif

  for (i = (NUMSAMPLES / 4); i < ((NUMSAMPLES / 4) * 3); i++)
  {
    if (samples[i] == samples[i + 1])
    {
      y = samples[i];
      break;
    }
  }

  // y = (1023 - samples[NUMSAMPLES / 2]);
  // y = samples[NUMSAMPLES / 2];

  // y = adc1;

  // Set XL to ground
  digitalWrite(_p_XL, HIGH);

  // Set YD to VCC
  digitalWrite(_p_YD, LOW);

  // Disable XR to VCC
  digitalWrite(_p_XR, HIGH);

  // Disable YU to ground
  digitalWrite(_p_YU, LOW);

  delay(1);

  uint16_t z1 = 0;
  uint16_t z2 = 0;

  // for (i = 0; i < NUMSAMPLES; i++)
  // {
  z1 += analogRead(_xr);
  z2 += analogRead(_yu);
  // }

  // z1 /= NUMSAMPLES;
  // z2 /= NUMSAMPLES;

  // Disable XL to ground
  digitalWrite(_p_XL, LOW);

  // Disable YD to VCC
  digitalWrite(_p_YD, HIGH);

  if (_rxplate != 0)
  {
    // now read the x
    float rtouch;
    rtouch = z2;
    rtouch /= z1;
    rtouch -= 1;
    rtouch *= x;
    rtouch *= _rxplate;
    rtouch /= 1024;

    z = rtouch;
  }
  else
  {
    z = (1023 - (z2 - z1));
  }

  if (!valid)
  {
    z = 0;
  }

  return TSPoint(x, y, z);
}

TouchScreen::TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym)
{
  _yu = yp;
  _xr = xm;
  _p_YD = ym;
  _p_XL = xp;
  _rxplate = 0;
  pressureThreshhold = 10;
}

TouchScreen::TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym,
                         uint16_t rxplate)
{
  _yu = yp;
  _xr = xm;
  _p_YD = ym;
  _p_XL = xp;
  _rxplate = rxplate;

  pressureThreshhold = 10;
}

TouchScreen::TouchScreen(uint8_t xl, uint8_t xr, uint8_t yd, uint8_t yu,
                         uint16_t rxplate,
                         uint8_t p_XL, uint8_t p_XR, uint8_t p_YD, uint8_t p_YU,
                         uint8_t p_THOa)
{
  _xl = xl;
  _xr = xr;
  _yd = yd;
  _yu = yu;

  _rxplate = rxplate;

  _p_XL = p_XL;
  _p_XR = p_XR;
  _p_YD = p_YD;
  _p_YU = p_YU;

  _p_THOa = p_THOa;

  pressureThreshhold = 10;
}

void TouchScreen::begin(void)
{
  pinMode(_xr, INPUT);
  pinMode(_xl, INPUT);
  pinMode(_yu, INPUT);
  pinMode(_yd, INPUT);

  pinMode(_p_XR, OUTPUT);
  pinMode(_p_XL, OUTPUT);
  pinMode(_p_YU, OUTPUT);
  pinMode(_p_YD, OUTPUT);

  digitalWrite(_p_XR, HIGH);
  digitalWrite(_p_XL, HIGH);
  digitalWrite(_p_YU, LOW);
  digitalWrite(_p_YD, HIGH);

  pinMode(_p_THOa, OUTPUT);
  digitalWrite(_p_THOa, HIGH); // Activate the T_HOa pin
}

int TouchScreen::readTouchX(void)
{
  digitalWrite(_p_XL, HIGH);
  digitalWrite(_p_XR, LOW);

  int adc1 = analogRead(_yu);

  digitalWrite(_p_XL, LOW);
  digitalWrite(_p_XR, HIGH);

  // return (1023 - adc1);
  return adc1;
}

int TouchScreen::readTouchY(void)
{
  digitalWrite(_p_YU, HIGH);
  digitalWrite(_p_YD, LOW);

  int adc1 = analogRead(_yu);

  digitalWrite(_p_YU, LOW);
  digitalWrite(_p_YD, HIGH);

  // return (1023 - adc1);
  return adc1;
}

uint16_t TouchScreen::pressure(void)
{
  // Set X+ to ground
  digitalWrite(_p_XL, HIGH);

  // Set Y- to VCC
  digitalWrite(_p_YD, LOW);

  // Disable XR to VCC
  digitalWrite(_p_XR, HIGH);

  // Disable YU to ground
  digitalWrite(_p_YU, LOW);

  int z1 = 0;
  int z2 = 0;

  for (uint8_t i = 0; i < 8; i++)
  {
    z1 += analogRead(_xr);
    z2 += analogRead(_yu);
  }

  z1 /= 8;
  z2 /= 8;

  // Disable X+ to ground
  digitalWrite(_p_XL, LOW);

  // Disable Y- to VCC
  digitalWrite(_p_YD, HIGH);

  if (_rxplate != 0)
  {
    // now read the x
    float rtouch;
    rtouch = z2;
    rtouch /= z1;
    rtouch -= 1;
    rtouch *= readTouchX();
    rtouch *= _rxplate;
    rtouch /= 1024;

    return rtouch;
  }
  else
  {
    return (1023 - (z2 - z1));
  }
}
