// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// (c) ladyada / adafruit
// Code under MIT License

#ifndef _ADAFRUIT_TOUCHSCREEN_H_
#define _ADAFRUIT_TOUCHSCREEN_H_
#include <stdint.h>
#include <Arduino.h>

class TSPoint
{
public:
  TSPoint(void);
  TSPoint(int16_t x, int16_t y, int16_t z);

  bool operator==(TSPoint);
  bool operator!=(TSPoint);

  int16_t x, y, z;
};

class TouchScreen
{
public:
  TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym);
  TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym, uint16_t rx);

  TouchScreen(uint8_t xl, uint8_t xr, uint8_t yd, uint8_t yu, uint16_t rxplate,
              uint8_t p_XL, uint8_t p_XR, uint8_t p_YD, uint8_t p_YU, uint8_t p_THOa);
  void begin(void);

  bool penFound(void);

  TSPoint calPoint(TSPoint pRaw, TSPoint *pRawRef, TSPoint *pCalRef, TSPoint pMaxRef);

  uint16_t pressure(void);
  int readTouchY();
  int readTouchX();
  TSPoint getPoint();
  int16_t pressureThreshhold;

private:
  uint8_t _xl, _xr, _yd, _yu, _p_XL, _p_XR, _p_YD, _p_YU, _p_THOa;
  uint16_t _rxplate;
};

#endif
