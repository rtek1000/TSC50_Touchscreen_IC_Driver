/*

  Touchscreen driver with serial communication (TSC50 based)

  - VScode (PlatformIO)

  Features:

  - Power On:
  After it is turned on or reset, enters the Initialization mode immediately.
  In this mode, no setting is made and each mode shall be set.
  To transition to the available state, select the coordinate output rate and
  coordinate output mode and set the operation mode.

  - Initialization mode
  By either power supply ON or hardware reset/software reset,
  the internal initial setting is performed. Inthe serial scheme,
  communication with host is available. In USB scheme,
  device recognition process ends. In this mode,
  receive a coordinate output rate command and transition to the idle mode.
  After hardware reset, it takes 500ms to transition to the initialization mode.
  In using the EEPROM, EEPROM data is read in this mode.

  - Idle mode
  After the coordinate output rate setting command is received,
  enters this operation waiting mode.
  Transition to each mode takes place in this mode

  - Calibration data setup mode
  In this mode, calibration data used in “ calibration data mode“ is set.
  Transition to this mode takes place when the setup mode start command is received in the idle mode.
  Select an appropriate calibration point among:
  - X=2, Y=2 (four point calibration)
  - X=3, Y=3 (nine point calibration) and
  - X=2, Y=2 (four point calibration) plus one center position (five point calibration)
  When this mode ends, calibration data is entered to EEPROM.

  - Calibration data read mode
  In this mode, setup data is read out from EEPROM if in using EEPROM,
  data set in the ” calibration data setup mode“ is stored to EEPROM.
  It is used if setting data is to be confirmed. Transition to this
  mode takes place and calibration data is output to the host if in the idle mode,
  calibration data read command is received.

  - Calibration data mode
  In this mode, individual difference and loss in the touch screen are corrected and coordinate value
  output is performed. Using calibration data set in the ” calibration data setup mode“, touch screen
  input point and indicator’ s cursor display position can be matched.
  In using this mode, the host driver needs no calibration function.
  Using “ calibration data setup mode“ setting value to set the coordinate data maximum value,
  maximum value can be limited to 10bits or less.
  This mode starts when in the idle mode, any ” calibration data send start command“ is received, and
  returns to the idle mode when receiving the calibration data send end command. When receiving the
  reset command, this mode transitions to the initialization mode.

  - STOP mode
  This mode is enabled only in the serial communication. This mode starts when in the idle mode, STOP
  mode start command is received. This mode stops ceramic vibrator’ s vibration and enters the power
  saving mode where the operation stops. By hardware reset, this mode transitions to the initialization
  mode. By receiving [00h] (null command) from host, the power saving mode ends and transitions to
  the idle mode.

  - Power-save mode
  This mode is only enabled in the serial communication. If in the coordinate data mode transitioned
  from [01h] or the calibration data mode transitioned from [0Ah], there are no inputs on the touch
  screen for 20 samplings, the ceramic vibrator stops its vibration and enters the power saving mode
  where no operations are taken. By inputting the touch screen or receiving a command from host,
  power saving mode is released and the transition to “ coordinate data mode “ or ” calibration data
  mode“ takes place.

 */

#include <Arduino.h>
#include <EEPROM.h>

#include <stdint.h>
#include "TouchScreen.h"

// These are the pins for the shield!
#define YD_pin A0 // must be an analog pin, use "An" notation!
#define YU_pin A1 // can be a digital pin
#define XL_pin A2 // must be an analog pin, use "An" notation!
#define XR_pin A3 // can be a digital pin

#define p_XR_pin 6 // digital pin active low
#define p_XL_pin 7 // digital pin active high
#define p_YU_pin 8 // digital pin active high
#define p_YD_pin 9 // digital pin active low

#define PANEL_THOa_pin 5 // digital pin active high

#define RECALL_pin 3 // digital pin active low
#define LED_pin 13   // digital pin active high

#define EXT_pin 2 // external interrupt pin (RESET)

#define MINPRESSURE 100
#define MAXPRESSURE 2000
#define R_PLATE_X 695 // Resistance across X plate in ohms
#define R_PLATE_Y 412 // Resistance across Y plate in ohms

#define BUFFER_SIZE 38

#define CMD_NULL 0x00
#define CMD_COOR_OUTPUT_RATE 0x05
#define CMD_GOTO_COOR_DAT_SEND_START1 0x01
#define CMD_GOTO_COOR_DAT_SEND_START2 0x21
#define CMD_GOTO_COOR_DAT_SEND_START3 0x31
#define CMD_COOR_DATA_END 0x02
#define CMD_GOTO_CAL_DAT_SEND_START1 0x0A
#define CMD_GOTO_CAL_DAT_SEND_START2 0x2A
#define CMD_GOTO_CAL_DAT_SEND_START3 0x3A
#define CMD_CAL_DATA_SEND_END 0x0B
#define CMD_GOTO_CAL_DATA_SETUP1 0x0D
#define CMD_GOTO_CAL_DATA_SETUP2 0x0E
#define CMD_GOTO_CAL_DATA_READ 0x1D
#define CMD_GOTO_STOP 0x0F
#define CMD_WRITE_EEPROM 0x19
#define CMD_RESET 0x55
#define CMD_PANEL_ID 0x15

#define RESPONSE_ACK 0x06
#define RESPONSE_NACK 0x15
#define RESPONSE_PEN_UP 0x10

#define CAL_POINTS_MAX 4

uint8_t cal_points = CAL_POINTS_MAX;

enum MODES
{
  MODE_POWER_SAVE1 = 0,
  MODE_COOR_DATA1,
  MODE_INIT1,
  MODE_CAL_DATA_SEND1,
  MODE_CAL_DATA_SEND2,
  MODE_CAL_DATA_SEND3,
  MODE_IDLE1,
  MODE_CAL_DATA_SETUP1,
  MODE_STOP1,
  MODE_CAL_DATA_READ1,
};

enum OUTPUT_RATES
{
  OUTPUT_RATE_30CPS = 0x40,
  OUTPUT_RATE_50CPS,
  OUTPUT_RATE_80CPS,
  OUTPUT_RATE_100CPS,
  OUTPUT_RATE_130CPS,
  OUTPUT_RATE_150CPS,
  OUTPUT_RATE_ONCE = 0x50
};

enum EEPROM_ADDR
{
  ADDR_P00_X_RAW_H = 0x00, // 0
  ADDR_P00_X_RAW_L,        // 1
  ADDR_P00_L_RAW_H,        // 2
  ADDR_P00_L_RAW_L,        // 3

  ADDR_P00_X_CAL_H, // 4
  ADDR_P00_X_CAL_L, // 5
  ADDR_P00_L_CAL_H, // 6
  ADDR_P00_L_CAL_L, // 7

  ADDR_P01_X_RAW_H, // 8
  ADDR_P01_X_RAW_L, // 9
  ADDR_P01_L_RAW_H, // 10
  ADDR_P01_L_RAW_L, // 11

  ADDR_P01_X_CAL_H, // 12
  ADDR_P01_X_CAL_L, // 13
  ADDR_P01_L_CAL_H, // 14
  ADDR_P01_L_CAL_L, // 15

  ADDR_P10_X_RAW_H, // 16
  ADDR_P10_X_RAW_L, // 17
  ADDR_P10_L_RAW_H, // 18
  ADDR_P10_L_RAW_L, // 19

  ADDR_P10_X_CAL_H, // 20
  ADDR_P10_X_CAL_L, // 21
  ADDR_P10_L_CAL_H, // 22
  ADDR_P10_L_CAL_L, // 23

  ADDR_P11_X_RAW_H, // 24
  ADDR_P11_X_RAW_L, // 25
  ADDR_P11_L_RAW_H, // 26
  ADDR_P11_L_RAW_L, // 27

  ADDR_P11_X_CAL_H, // 28
  ADDR_P11_X_CAL_L, // 29
  ADDR_P11_L_CAL_H, // 30
  ADDR_P11_L_CAL_L, // 31
};

uint8_t buffer_index = 0;
uint8_t input_buffer[BUFFER_SIZE];
bool input_error = false;
bool send_ack = false;
bool send_nack = false;
bool pen_down1 = false;

uint32_t millis_pen_timeout = 0;

#define DEBUG 0 // Comment this line to disable debug mode

#if (DEBUG == 1)
uint8_t operation_mode = MODE_CAL_DATA_SEND3; // MODE_INIT1; // MODE_CAL_DATA_SEND3
#else
uint8_t operation_mode = MODE_INIT1; // MODE_INIT1; // MODE_CAL_DATA_SEND3
#endif

bool set_coord_output_rate = false;
uint8_t coord_output_rate_ms = 33; // 33ms for 30 cps

TSPoint point_raw[CAL_POINTS_MAX] = {
    {0x0028, 0x0032, 0},
    {0x03D4, 0x0028, 0},
    {0x001E, 0x03D4, 0},
    {0x03CA, 0x03CA, 0}};

TSPoint point_cal[CAL_POINTS_MAX] = {
    {0x0000, 0x0000, 0},
    {0x0320, 0x0000, 0},
    {0x0000, 0x0258, 0},
    {0x0320, 0x0258, 0}};

TSPoint point_raw_recall[CAL_POINTS_MAX] = {
    {0x0028, 0x0032, 0},
    {0x03D4, 0x0028, 0},
    {0x001E, 0x03D4, 0},
    {0x03CA, 0x03CA, 0}};

TSPoint point_cal_recall[CAL_POINTS_MAX] = {
    {0x0000, 0x0000, 0},
    {0x0320, 0x0000, 0},
    {0x0000, 0x0258, 0},
    {0x0320, 0x0258, 0}};

TSPoint point_max_val = {0x03FF, 0x03FF, 0};

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XL_pin, XR_pin, YD_pin, YU_pin, R_PLATE_X, p_XL_pin, p_XR_pin, p_YD_pin, p_YU_pin, PANEL_THOa_pin);

uint32_t millis_output_rate = 0;

bool recall_data_flag = false;
uint32_t millis_recall_data = 0;
bool send_0x12_signal = false;

// put function declarations here:
void send_data(void);
void save_data(void);
void load_data(void);

void ext_reset_ISR(void)
{
  // External RESET Interrupt Service Routine
  operation_mode = MODE_INIT1;
  send_0x12_signal = true;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Serial.println("Start");

  // pinMode(PANEL_THOa_pin, OUTPUT);
  // digitalWrite(PANEL_THOa_pin, LOW);
  pinMode(PANEL_THOa_pin, INPUT);

  pinMode(LED_pin, OUTPUT);
  digitalWrite(LED_pin, HIGH);

  pinMode(RECALL_pin, INPUT_PULLUP);

  if (digitalRead(RECALL_pin) == HIGH)
  {
    recall_data_flag = true;
    load_data();
  }

  digitalWrite(LED_pin, LOW);

  // while (millis() < 40)
  //   ;

  // Serial.write(0x12);

  for (size_t i = 0; i < BUFFER_SIZE; i++)
  {
    input_buffer[i] = 0;
  }

  ts.begin();

  while (millis() < 30)
    ;

  attachInterrupt(digitalPinToInterrupt(EXT_pin), ext_reset_ISR, FALLING); // More Recommended

  digitalWrite(LED_pin, HIGH);

  Serial.write(0x12);

  // while (1)
  // {
  //   TSPoint p = ts.getPoint();
  //   Serial.println("X: " + String(p.x) + " Y: " + String(p.y) + " Z: " + String(p.z));
  //   delay(500);
  // }
}

void loop()
{
  // put your main code here, to run repeatedly:

  if (recall_data_flag == true)
  {
    if (digitalRead(RECALL_pin) == LOW)
    {
      if ((millis() - millis_recall_data) > 10000)
      {
        millis_recall_data = millis();

        for (uint8_t i = 0; i < cal_points; i++)
        {
          point_raw[i] = point_raw_recall[i];
          point_cal[i] = point_cal_recall[i];
        }

        save_data();

        for (uint8_t i = 0; i < cal_points; i++)
        {
          digitalWrite(LED_pin, HIGH);
          delay(500);
          digitalWrite(LED_pin, LOW);
          delay(500);
        }
      }
    }
    else
    {
      millis_recall_data = millis();
    }
  }

  if ((operation_mode == MODE_COOR_DATA1) || (operation_mode == MODE_CAL_DATA_SEND3))
  {
    send_data();
  }

  if (send_0x12_signal == true)
  {
    Serial.write(0x12);
    send_0x12_signal = false;
  }

  if (Serial.available())
  {
    uint8_t _data = Serial.read();

    digitalWrite(LED_pin, _data & 0x01);

    if (operation_mode == MODE_INIT1)
    {
      if (set_coord_output_rate == false)
      {
        if (_data == CMD_COOR_OUTPUT_RATE)
        {
          set_coord_output_rate = true;
        }
      }
      else
      {
        switch (_data)
        {
        case OUTPUT_RATE_30CPS:
          coord_output_rate_ms = 33; // 30 cps
          send_ack = true;
          break;
        case OUTPUT_RATE_50CPS:
          coord_output_rate_ms = 20; // 50 cps
          send_ack = true;
          break;
        case OUTPUT_RATE_80CPS:
          coord_output_rate_ms = 12; // 80 cps
          send_ack = true;
          break;
        case OUTPUT_RATE_100CPS:
          coord_output_rate_ms = 10; // 100 cps
          send_ack = true;
          break;
        case OUTPUT_RATE_130CPS:
          coord_output_rate_ms = 8; // 130 cps
          send_ack = true;
          break;
        case OUTPUT_RATE_150CPS:
          coord_output_rate_ms = 7; // 150 cps
          send_ack = true;
          break;
        case OUTPUT_RATE_ONCE:
          coord_output_rate_ms = 0; // Once when touched
          send_ack = true;
          break;
        default:
          send_nack = true;
          break;
        }

        if (send_ack == true)
        {
          operation_mode = MODE_IDLE1;
        }

        set_coord_output_rate = false;
      }
    }
    else if (operation_mode == MODE_COOR_DATA1)
    {
      if (_data == CMD_COOR_DATA_END)
      {
        operation_mode = MODE_IDLE1;
        send_ack = true;
      }
    }
    else if ((operation_mode == MODE_CAL_DATA_SEND1) ||
             (operation_mode == MODE_CAL_DATA_SEND2) ||
             (operation_mode == MODE_CAL_DATA_SEND3))
    {
      if (_data == CMD_CAL_DATA_SEND_END)
      {
        operation_mode = MODE_IDLE1;
        send_ack = true;
      }
    }
    else if (operation_mode == MODE_IDLE1)
    {
      switch (_data)
      {
      case CMD_GOTO_CAL_DAT_SEND_START3:
        operation_mode = MODE_CAL_DATA_SEND3;
        break;
      case CMD_GOTO_COOR_DAT_SEND_START3:
        operation_mode = MODE_COOR_DATA1;
        send_ack = true;
        break;
      case CMD_GOTO_CAL_DATA_SETUP1:
        operation_mode = MODE_CAL_DATA_SETUP1;
        buffer_index = 0;
        break;
      case CMD_NULL:
        break;
      default:
        send_nack = true;
        break;
      }
    }
    else if (operation_mode == MODE_CAL_DATA_SETUP1)
    {
      if (buffer_index < BUFFER_SIZE)
      {
        input_buffer[buffer_index++] = _data;

        if (buffer_index == BUFFER_SIZE)
        {
          operation_mode = MODE_IDLE1;

          if ((input_buffer[0] == 0x02) && (input_buffer[1] == 0x02) &&
              (input_buffer[34] == 0x03) && (input_buffer[35] == 0xFF) &&
              (input_buffer[36] == 0x03) && (input_buffer[37] == 0xFF))
          {
            point_raw[0].x = (input_buffer[2] << 8) | input_buffer[3];
            point_raw[0].y = (input_buffer[4] << 8) | input_buffer[5];
            point_cal[0].x = (input_buffer[6] << 8) | input_buffer[7];
            point_cal[0].y = (input_buffer[8] << 8) | input_buffer[9];

            point_raw[1].x = (input_buffer[10] << 8) | input_buffer[11];
            point_raw[1].y = (input_buffer[12] << 8) | input_buffer[13];
            point_cal[1].x = (input_buffer[14] << 8) | input_buffer[15];
            point_cal[1].y = (input_buffer[16] << 8) | input_buffer[17];

            point_raw[2].x = (input_buffer[18] << 8) | input_buffer[19];
            point_raw[2].y = (input_buffer[20] << 8) | input_buffer[21];
            point_cal[2].x = (input_buffer[22] << 8) | input_buffer[23];
            point_cal[2].y = (input_buffer[24] << 8) | input_buffer[25];

            point_raw[3].x = (input_buffer[26] << 8) | input_buffer[27];
            point_raw[3].y = (input_buffer[28] << 8) | input_buffer[29];
            point_cal[3].x = (input_buffer[30] << 8) | input_buffer[31];
            point_cal[3].y = (input_buffer[32] << 8) | input_buffer[33];

            save_data();

            send_ack = true;
          }
          else
          {
            send_nack = true;
          }
        }
      }
    }
  }

  if (send_nack == true)
  {
    Serial.write(RESPONSE_NACK); // NACK
    send_nack = false;
  }

  if (send_ack == true)
  {
    Serial.write(RESPONSE_ACK); // ACK
    send_ack = false;
  }
}

void send_data(void)
{
  if (coord_output_rate_ms != 0)
  {
    if (ts.penFound() == true)
    {
      if ((millis() - millis_output_rate) > coord_output_rate_ms)
      {
        millis_output_rate = millis();

        // a point object holds x y and z coordinates
        TSPoint p = ts.getPoint();

        // we have some minimum pressure we consider 'valid'
        // pressure of 0 means no pressing!
        if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
        {
          // digitalWrite(PANEL_THOa_pin, HIGH);
          digitalWrite(LED_pin, HIGH);

          pen_down1 = true;

#if (DEBUG == 1)
          Serial.print("X: " + String(p.x) + " Y: " + String(p.y) + " Z: " + String(p.z));
#endif
          if (operation_mode == MODE_CAL_DATA_SEND3)
          {
            p = ts.calPoint(p, point_raw, point_cal, point_max_val);
          }

#if (DEBUG == 1)
          Serial.println(" - X cal: " + String(p.x) + " Y cal: " + String(p.y) + " Z cal: " + String(p.z));
#endif
          uint8_t pen_ID = 0x11;

          uint8_t pen_XH = (p.x >> 8) & 0xFF;
          uint8_t pen_XL = p.x & 0xFF;
          uint8_t pen_YH = (p.y >> 8) & 0xFF;
          uint8_t pen_YL = p.y & 0xFF;

          pen_ID |= (p.y >> 6) & 0x01;

#if (DEBUG == 0)
          Serial.write(pen_ID);
          Serial.write(pen_XH);
          Serial.write(pen_XL);
          Serial.write(pen_YH);
          Serial.write(pen_YL);
#endif
          // delay(33);

          // Serial.write(pen_ID);
          // Serial.write(pen_XH);
          // Serial.write(pen_XL);
          // Serial.write(pen_YH);
          // Serial.write(pen_YL);

          // delay(33);

          // Serial.write(pen_ID);
          // Serial.write(pen_XH);
          // Serial.write(pen_XL);
          // Serial.write(pen_YH);
          // Serial.write(pen_YL);

          // millis_output_rate = millis();

          millis_pen_timeout = millis();
        }
      }
    }
    else if (pen_down1 == true)
    {
      if ((millis() - millis_pen_timeout) > (coord_output_rate_ms * 2))
      {
        // digitalWrite(PANEL_THOa_pin, LOW);
        digitalWrite(LED_pin, LOW);

        pen_down1 = false;

        Serial.write(RESPONSE_PEN_UP);
      }
    }
  }
}

void save_data(void)
{
  for (uint8_t i = 0; i < cal_points; i++)
  {
    EEPROM.update(i * 8, (point_raw[0].x >> 8) & 0xFF);
    EEPROM.update((i * 8) + 1, point_raw[0].x & 0xFF);

    EEPROM.update((i * 8) + 2, (point_raw[0].y >> 8) & 0xFF);
    EEPROM.update((i * 8) + 3, point_raw[0].y & 0xFF);

    EEPROM.update((i * 8) + 4, (point_cal[0].x >> 8) & 0xFF);
    EEPROM.update((i * 8) + 5, point_cal[0].x & 0xFF);

    EEPROM.update((i * 8) + 6, (point_cal[0].y >> 8) & 0xFF);
    EEPROM.update((i * 8) + 7, point_cal[0].y & 0xFF);
  }
}

void load_data(void)
{
  TSPoint temp_point_raw[CAL_POINTS_MAX];
  TSPoint temp_point_cal[CAL_POINTS_MAX];

  for (uint8_t i = 0; i < cal_points; i++)
  {
    temp_point_raw[0].x = EEPROM.read(i * 8) << 8;
    temp_point_raw[0].x |= EEPROM.read((i * 8) + 1);

    temp_point_raw[0].y = EEPROM.read((i * 8) + 2) << 8;
    temp_point_raw[0].y |= EEPROM.read((i * 8) + 3);

    temp_point_cal[0].x = EEPROM.read((i * 8) + 4) << 8;
    temp_point_cal[0].x |= EEPROM.read((i * 8) + 5);

    temp_point_cal[0].y = EEPROM.read((i * 8) + 6) << 8;
    temp_point_cal[0].y |= EEPROM.read((i * 8) + 7);
  }

  if (((temp_point_raw[0].x <= 0xFF) && (temp_point_raw[0].y <= 0xFF)) &&
      ((temp_point_raw[1].x >= 0x300) && (temp_point_raw[1].y <= 0xFF)) &&
      ((temp_point_raw[2].x <= 0xFF) && (temp_point_raw[2].y >= 0x300)) &&
      ((temp_point_raw[4].x >= 0x300) && (temp_point_raw[4].y >= 0x300)) &&
      ((temp_point_cal[0].x <= 0xFF) && (temp_point_cal[0].y <= 0xFF)) &&
      ((temp_point_cal[1].x >= 0x300) && (temp_point_cal[1].y <= 0xFF)) &&
      ((temp_point_cal[2].x <= 0xFF) && (temp_point_cal[2].y >= 0x300)) &&
      ((temp_point_cal[4].x >= 0x300) && (temp_point_cal[4].y >= 0x300)))
  {
    for (uint8_t i = 0; i < cal_points; i++)
    {
      point_raw[i] = temp_point_raw[i];
      point_cal[i] = temp_point_cal[i];
    }
  }
}
