#define PWMCHANGED

#include <Arduino.h>
#define PWMADJUSTER 32
#ifdef PWMCHANGED 
#define millis() millis()/PWMADJUSTER
#define micros() micros()/PWMADJUSTER
#define delay( x ) delay( (x) * PWMADJUSTER)
// #if PWMADJUSTER == 64
// #define delay(x) {delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);}
// #endif
// #if PWMADJUSTER == 32
// #define delay(x) {delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);}
// #endif
// #if PWMADJUSTER == 8
// #define delay(x) {delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);delay(x);}
// #endif
// #if PWMADJUSTER == 4
// #define delay(x) {delay(x);delay(x);delay(x);delay(x);}
// #endif
//#define delayMicroseconds(x) delayMicroseconds(x/64)
#endif

#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include "encodedMotor.h"
#include <PID_v1.h>
#include <Encoder.h>

//Print message priority
//#define DEBUG_INFO_ERROR
//#define DEBUG_INFO_HIGH
//#define DEBUG_INFO_MID
//#define DEBUG_INFO_LOW
// #define DEBUG_INFO

//Common parameter configuration
#define DEFAULT_I2C_ADDR 0x09 // default is 0x09, alternative is 0x0A
//#define DEFAULT_I2C_ADDR 0xA0
// #define PULSE_PER_C 8
// #define MOTOR_BOTH 2

// #define DONT_USE_METER_PER_SECOND_AS_INPUT

//EEPROM CONFIG
#define EEPROM_DEVID_ADDR 3

//GPIO configuration
#define INT_1_PIN 3
#define DIR_1_PIN A2
#define MOTOR_1_PWM 6
#define MOTOR_1_H1 7

#define INT_2_PIN 2
#define DIR_2_PIN A3
#define MOTOR_2_PWM 5
#define MOTOR_2_H1 8

/*******************
 * ENCODER CONFIGURATION
 *******************/ 
#define ENCODER_PULSES_PER_ROTATION 700.0
Encoder encoder1(INT_1_PIN, DIR_1_PIN);
Encoder encoder2(INT_2_PIN, DIR_2_PIN);

//PID config
//Define Variables we'll be connecting to
double motor1Setpoint, motor1Input, motor1Output;
double motor2Setpoint, motor2Input, motor2Output;
boolean motor1Released = false;
boolean motor2Released = false;

// Motor command variables
float motor1TargetSpeed = 0;
float motor2TargetSpeed = 0;

// PWM RANGE CONFIG
// this is some weird hack to skip setpoint values that won't make the motor move.
// This should hopefully make the PID more responsive. Especially when suddenly changing direction.
// We basically skip the lower parts of the setpoint in order to quicker reach actual movement of the motor.
#define PWM_NO_MOTION_THRESHOLD 2.0
#define PWM_MOTION_THRESHOLD 100.0

//Specify the links and initial tuning parameters
// double Kp = 80, Ki = 40, Kd = 20;
//double Kp = 16, Ki = 8, Kd = 4;
double Kp = 20, Ki = 130, Kd = 0;
PID motor1PID(&motor1Input, &motor1Output, &motor1Setpoint, Kp, Ki, Kd, REVERSE);
PID motor2PID(&motor2Input, &motor2Output, &motor2Setpoint, Kp, Ki, Kd, REVERSE);

/****************************************************************************************************
 * I2C slave code
 * These codes used to start the i2c slave service and its command processing, If it is not
 * necessary, Do not modify this part of the code
****************************************************************************************************/
union {
  byte byteVal[16];
  int16_t intVal[8];
  float floatVal[4];
  long longVal[4];
} val;

char bufI2C[32];
int16_t rd, wr;

/**
 * \par Function
 *   I2C_init
 * \par Description
 *   I2C initialization function, set the baudrate, device address, and configuration some 
 *   registers
 * \param[in]
 *   address - device address 
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void I2C_init(uint8_t address)
{
  // load address into TWI address register
  TWAR = address;

  TWCR = 0x00;  //Disable TWI
  TWBR = 0x02;  //Set the baudrate: CPU Clock/16+2(TWBR)
  TWSR |= 0x00; //Set Divider factor

  //Set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
  TWCR = (1 << TWIE) | (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
}

/**
 * \par Function
 *   ISR(TWI_vect)
 * \par Description
 *   This function is used to process data from i2c host
 * \param[in]
 *   None 
 * \par Output
 *   bufI2C[]
 * \return
 *   None
 * \par Others
 *   None
 */
ISR(TWI_vect)
{
  // temporary stores the received data
  uint8_t data;

  // own address has been acknowledged
  if ((TWSR & 0xF8) == TW_SR_SLA_ACK)
  {
    rd = 0;
    wr = 0;
    // clear TWI interrupt flag, prepare to receive next byte and acknowledge
    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
  }
  else if ((TWSR & 0xF8) == TW_SR_DATA_ACK)
  {
    // data has been received in slave receiver mode
    // save the received byte inside data
    data = TWDR;
    bufI2C[rd++] = data;
    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
  }
  else if ((TWSR & 0xF8) == TW_ST_SLA_ACK)
  {
    // the start of i2c read
    TWDR = bufI2C[wr++]; // todo: errata of avr, to insert delay between twdr and twcr?
    // clear TWI interrupt flag, prepare to send next byte and receive acknowledge
    TWCR |= (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
  }
  else if ((TWSR & 0xF8) == TW_ST_DATA_ACK)
  {
    // device has been addressed to be a transmitter
    // copy the specified buffer address into the TWDR register for transmission
    TWDR = bufI2C[wr++];
    // clear TWI interrupt flag, prepare to send next byte and receive acknowledge
    TWCR |= (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
  }
  else if ((TWSR & 0xF8) == TW_SR_STOP)
  {
    ParseI2cCmd(bufI2C);
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
  }
  else
  {
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);
    while (TWCR & _BV(TWSTO))
    {
      continue;
    }
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
  }
}

/**
 * \par Function
 *   ParseI2cCmd
 * \par Description
 *   This function is used to process command from i2c host
 * \param[in]
 *   (bufI2C[])byte 0:   motor index
 *   (bufI2C[])byte 1:   cmd
 *   (bufI2C[])byte 2-n: parameters
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void ParseI2cCmd(char *c)
{
  int rpm;
  long angle;
  int power;
  uint8_t cmd = (c[1] & 0x3f);
  uint8_t slot = c[0];
  memcpy(&val, c + 2, 16);
#ifdef DEBUG_INFO
  Serial.print("cmd:");
  Serial.print(cmd);
#endif
  switch (cmd)
  {
  // move state and function
  case CMD_RESET:
    // resetMotor();
    break;
  case CMD_MOVE_TO:
#ifdef DEBUG_INFO
    Serial.print("  ,slot:");
    Serial.print(slot);
    Serial.print("  ,flag:");
    Serial.print((uint8_t)val.floatVal[0]);
    Serial.print("  ,pos:");
    Serial.print(val.longVal[1]);
    Serial.print("  speed:");
    Serial.println(val.floatVal[2]);
#endif
    // moveTo(slot, (uint8_t)val.floatVal[0], val.longVal[1], val.floatVal[2]);
    break;
  case CMD_MOVE:
    // move(slot, (uint8_t)val.floatVal[0], val.longVal[1], val.floatVal[2]);
    break;
  case CMD_MOVE_SPD:
    // By default we expect m/s as unit here.
    // But it's possible to uncomment the macro DONT_USE_METERS_PER_SECOND in order
    // to use values between -250 and 250
    // Parameter order is slot, lock_state, speed
    move_speed(slot, (uint8_t)val.floatVal[0], val.floatVal[1]);
    break;
  case CMD_SET_SPEED_PID:
    // setSpeedPid(slot, val.floatVal[0], val.floatVal[1], val.floatVal[2]);
    break;
  case CMD_SET_POS_PID:
    // setPosPid(slot, val.floatVal[0], val.floatVal[1], val.floatVal[2]);
    break;
  case CMD_SET_MODE:
    // setMode(slot, val.byteVal[0]);
    break;
  case CMD_SET_PWM:
    // setMode(slot, PWM_I2C_PWM);
    // set_pwm(slot, val.intVal[0]);
    break;
  case CMD_SET_RATIO:
    // setRatio(slot, val.floatVal[0]);
    break;
  case CMD_SET_CUR_POS:
    // setCurPos(slot, val.longVal[0]);
    break;
  case CMD_SET_PULSE:
    // setPulse(slot, val.intVal[0]);
    break;
  case CMD_SET_DEVID:
    setDevid(val.byteVal[0]);
    break;
  case CMD_GET_SPEED_PID:
    // if (slot == MOTOR_1)
    // {
    //   memcpy(&bufI2C[0], &encoder_eeprom.speed1_p, 4);
    //   memcpy(&bufI2C[4], &encoder_eeprom.speed1_i, 4);
    //   memcpy(&bufI2C[8], &encoder_eeprom.speed1_d, 4);
    // }
    // else
    // {
    //   memcpy(&bufI2C[0], &encoder_eeprom.speed0_p, 4);
    //   memcpy(&bufI2C[4], &encoder_eeprom.speed0_i, 4);
    //   memcpy(&bufI2C[8], &encoder_eeprom.speed0_d, 4);
    // }
    break;
  case CMD_GET_POS_PID:
    // if (slot == MOTOR_1)
    // {
    //   memcpy(&bufI2C[0], &encoder_eeprom.pos1_p, 4);
    //   memcpy(&bufI2C[4], &encoder_eeprom.pos1_i, 4);
    //   memcpy(&bufI2C[8], &encoder_eeprom.pos1_d, 4);
    // }
    // else
    // {
    //   memcpy(&bufI2C[0], &encoder_eeprom.pos0_p, 4);
    //   memcpy(&bufI2C[4], &encoder_eeprom.pos0_i, 4);
    //   memcpy(&bufI2C[8], &encoder_eeprom.pos0_d, 4);
    // }
    break;
  case CMD_GET_SPEED:
    // memcpy(&bufI2C[0], &encoder_data[slot].current_speed, 4);
    break;
  case CMD_GET_POS:
    // memcpy(&bufI2C[0], &encoder_data[slot].pulse, 4);
    break;
  case CMD_GET_LOCK_STATE:
    // memcpy(&bufI2C[0], &lock_flag[slot], 2);
    break;
  case CMD_GET_RATIO:
    // if (slot == MOTOR_1)
    // {
    //   memcpy(&bufI2C[0], &encoder_eeprom.ratio1, 4);
    // }
    // else
    // {
    //   memcpy(&bufI2C[0], &encoder_eeprom.ratio0, 4);
    // }
    break;
  case CMD_GET_PULSE:
    // if (slot == MOTOR_1)
    // {
    //   memcpy(&bufI2C[0], &encoder_eeprom.pulse1, 2);
    // }
    // else
    // {
    //   memcpy(&bufI2C[0], &encoder_eeprom.pulse0, 2);
    // }
    break;
  }
}

/******************************
 * EEPROM FUNCTIONS
*******************************/

void FloatToEEPROM(int address, int num, float *f)
{
  int i, j, bakupaddress;
  bakupaddress = address + EEPROM.length() / 2;
  for (i = 0; i < 2; i++)
  {
    for (j = 0; j < num; j++)
    {
      EEPROM.put(address, f[j]);
      address = address + sizeof(float);
    }
    address = bakupaddress;
  }
}

void IntToEEPROM(int address, int num, uint16_t *data)
{
  int i, j, bakupaddress;
  bakupaddress = address + EEPROM.length() / 2;
  for (i = 0; i < 2; i++)
  {
    for (j = 0; j < num; j++)
    {
      EEPROM.put(address, data[j]);
      address = address + sizeof(uint16_t);
    }
    address = bakupaddress;
  }
}

void ByteToEEPROM(int address, int num, uint8_t *data)
{
  int i, j, bakupaddress;
  bakupaddress = address + EEPROM.length() / 2;
  for (i = 0; i < 2; i++)
  {
    for (j = 0; j < num; j++)
    {
      EEPROM.put(address, data[j]);
      address = address + sizeof(uint8_t);
    }
    address = bakupaddress;
  }
}

// void writetoEEPROM(void)
// {
//   int16_t i, address, length;
//   length = EEPROM.length();
//   for (i = 0; i < 2; i++)
//   {
//     EEPROM.write(EEPROM_START_POS, EEPROM_IF_HAVEPID_CHECK1);
//     EEPROM.write(EEPROM_START_POS + 1, EEPROM_IF_HAVEPID_CHECK2);
//     EEPROM.put(STORE_START_ADDR, encoder_eeprom);
//     address = length / 2;
//   }
// }

// void updatefromBackuparea(void)
// {
//   int16_t address, length;
//   length = EEPROM.length();
//   address = length / 2;
//   if ((EEPROM.read(address) == EEPROM_IF_HAVEPID_CHECK1) & (EEPROM.read(address + 1) == EEPROM_IF_HAVEPID_CHECK2))
//   {
//     address = length / 2 + 2;
//     EEPROM.get(address, encoder_eeprom);
//     if ((encoder_eeprom.start_data == EEPROM_PID_START) & (encoder_eeprom.mid_data == EEPROM_PID_MID) & (encoder_eeprom.end_data == EEPROM_PID_END))
//     {
//       EEPROM.write(EEPROM_START_POS, EEPROM_IF_HAVEPID_CHECK1);
//       EEPROM.write(EEPROM_START_POS + 1, EEPROM_IF_HAVEPID_CHECK2);
//       EEPROM.put(STORE_START_ADDR, encoder_eeprom);
//     }
//     else
//     {
//       setDefaultvalue();
//       writetoEEPROM();
//     }
//   }
// }

// boolean readEEPROM(void)
// {
//   int16_t length = EEPROM.length();
//   Serial.println("Read data from EEPROM ");
//   if ((EEPROM.read(EEPROM_START_POS) == EEPROM_IF_HAVEPID_CHECK1) & (EEPROM.read(EEPROM_START_POS + 1) == EEPROM_IF_HAVEPID_CHECK2))
//   {
//     EEPROM.get(STORE_START_ADDR, encoder_eeprom);
//     if ((encoder_eeprom.start_data == EEPROM_PID_START) & (encoder_eeprom.mid_data == EEPROM_PID_MID) & (encoder_eeprom.end_data == EEPROM_PID_END))
//     {
//       return true;
//     }
//     else
//     {
//       Serial.println("updatefromBackuparea");
//       updatefromBackuparea();
//       return true;
//     }
//   }
//   else if ((EEPROM.read(length / 2) == EEPROM_IF_HAVEPID_CHECK1) & (EEPROM.read(length / 2 + 1) == EEPROM_IF_HAVEPID_CHECK2))
//   {
//     Serial.println("updatefromBackuparea 2");
//     updatefromBackuparea();
//     return true;
//   }
//   else
//   {
//     setDefaultvalue();
//     writetoEEPROM();
//     return false;
//   }
// }

/****************************************************************************************************
 * Motor Control Functions
****************************************************************************************************/
// ISR(MOTOR_1_IRQ)
// {
//   if (MOTOR_1_PIN & (1 << MOTOR_1_DIR))
//   {
//     encoder_data[MOTOR_0].pulse++;
//   }
//   else
//   {
//     encoder_data[MOTOR_0].pulse--;
//   }
// }

// ISR(MOTOR_2_IRQ)
// {
//   if (MOTOR_2_PIN & (1 << MOTOR_2_DIR))
//   {
//     encoder_data[MOTOR_1].pulse++;
//   }
//   else
//   {
//     encoder_data[MOTOR_1].pulse--;
//   }
// }

void setMotorPwm(uint8_t slot, int16_t setpoint)
{
  //  Serial.print("mode:");
  //  Serial.print(encoder_data[slot].mode);
  //  Serial.print(" ,slot:");
  //  Serial.print(slot);
  //  Serial.print(" ,setpoint:");
  //  Serial.println(setpoint);
  if (slot == 1)
  {
    setMotor2Pwm(setpoint);
  }
  else
  {
    setMotor1Pwm(setpoint);
  }
}

void setMotor1Pwm(int16_t setpoint)
{
  if(motor1Released)
  {
    analogWrite(MOTOR_1_PWM, 0);
    return;
  }

  setpoint = constrain(setpoint, -250, 250);
  if (setpoint < 0)
  {
    digitalWrite(MOTOR_1_H1, HIGH);
    analogWrite(MOTOR_1_PWM, abs(setpoint));
  }
  else
  {
    digitalWrite(MOTOR_1_H1, LOW);
    analogWrite(MOTOR_1_PWM, abs(setpoint));
  }
}

void setMotor2Pwm(int16_t setpoint)
{
  if(motor2Released)
  {
    analogWrite(MOTOR_2_PWM, 0);
    return;
  }

  setpoint = constrain(setpoint, -250, 250);
  if (setpoint < 0)
  {
    digitalWrite(MOTOR_2_H1, HIGH);
    analogWrite(MOTOR_2_PWM, abs(setpoint));
  }
  else
  {
    digitalWrite(MOTOR_2_H1, LOW);
    analogWrite(MOTOR_2_PWM, abs(setpoint));
  }
}

/****************************************************************************************************
 * processing function
****************************************************************************************************/
// void updateSpeed(void)
// {
//   if ((millis() - measurement_speed_time) > 20)
//   {
//     uint16_t dt = millis() - measurement_speed_time;
//     long cur_pos_0 = encoder_data[MOTOR_0].pulse;
//     long cur_pos_1 = encoder_data[MOTOR_1].pulse;
//     encoder_data[MOTOR_0].current_speed = ((cur_pos_0 - last_pulse_pos_encoder0) / (encoder_data[MOTOR_0].encoder_pulse * encoder_data[MOTOR_0].ratio)) * (1000 / dt) * 60;
//     encoder_data[MOTOR_1].current_speed = ((cur_pos_1 - last_pulse_pos_encoder1) / (encoder_data[MOTOR_1].encoder_pulse * encoder_data[MOTOR_1].ratio)) * (1000 / dt) * 60;
//     last_pulse_pos_encoder0 = cur_pos_0;
//     last_pulse_pos_encoder1 = cur_pos_1;
//     measurement_speed_time = millis();
//   }
// }

// void encoder_move(uint8_t slot)
// {
//   if (millis() - encoder_move_time[slot] > 40)
//   {
//     int16_t pwm_encoder = 0;
//     encoder_move_time[slot] = millis();
//     if ((encoder_data[slot].motion_state == MOTION_WITH_POS_LOCK) ||
//         (encoder_data[slot].motion_state == MOTION_WITH_POS_RELEASE))
//     {
//       pwm_encoder = pid_position_to_pwm(slot);
//     }
//     else if ((encoder_data[slot].motion_state == MOTION_WITHOUT_POS_LOCK) ||
//              (encoder_data[slot].motion_state == MOTION_WITHOUT_POS_RELEASE))
//     {
//       pwm_encoder = speed_without_pos(slot);
//     }
//     encoder_data[slot].cur_pwm = pwm_encoder;
//   }
// }

// void set_pwm(uint8_t slot, int setpoint)
// {
//   encoder_data[slot].tar_pwm = setpoint;
// }

// void pwm_move(uint8_t slot)
// {
//   if (millis() - encoder_move_time[slot] > 40)
//   {
//     encoder_move_time[slot] = millis();
//     encoder_data[slot].cur_pwm = 0.8 * encoder_data[slot].cur_pwm + 0.2 * encoder_data[slot].tar_pwm;
//     if ((abs(encoder_data[slot].cur_pwm) <= 20) && (encoder_data[slot].tar_pwm == 0))
//     {
//       lock_flag[slot] == true;
//       encoder_data[slot].cur_pwm = 0;
//     }
//     else
//     {
//       lock_flag[slot] = false;
//     }
//   }
// }

// void updateCurPos(void)
// {
//   encoder_data[MOTOR_0].current_pos = (long)((encoder_data[MOTOR_0].pulse / (encoder_data[MOTOR_0].encoder_pulse * encoder_data[MOTOR_0].ratio)) * 360);
//   encoder_data[MOTOR_1].current_pos = (long)((encoder_data[MOTOR_1].pulse / (encoder_data[MOTOR_1].encoder_pulse * encoder_data[MOTOR_1].ratio)) * 360);
// }

// long encoder_distance_togo(uint8_t slot)
// {
//   return encoder_data[slot].target_pos - encoder_data[slot].current_pos;
// }

// int16_t pid_position_to_pwm(uint8_t slot)
// {
//   float cur_pos = 0;
//   float seek_speed = 0;
//   float seek_temp = 0;
//   float pos_error = 0;
//   float speed_error = 0;
//   float d_component = 0;
//   float out_put_offset = 0;

//   pos_error = encoder_distance_togo(slot);

//   if ((lock_flag[slot] == false) && (dir_lock_flag[slot] == true) && (pos_error < 0))
//   {
//     d_component = encoder_data[slot].current_speed;
//     out_put_offset = encoder_data[slot].PID_pos.D * d_component;
//     encoder_data[slot].PID_pos.Output = -out_put_offset;
//     encoder_output[slot] = encoder_data[slot].PID_pos.Output;
//     lock_flag[slot] = true;
//     encoder_data[slot].cur_pwm = encoder_output[slot];
//     return encoder_output[slot];
//   }
//   else if ((lock_flag[slot] == false) && (dir_lock_flag[slot] == false) && (pos_error > 0))
//   {
//     d_component = encoder_data[slot].current_speed;
//     out_put_offset = encoder_data[slot].PID_pos.D * d_component;
//     encoder_data[slot].PID_pos.Output = -out_put_offset;
//     encoder_output[slot] = encoder_data[slot].PID_pos.Output;
//     lock_flag[slot] = true;
//     encoder_data[slot].cur_pwm = encoder_output[slot];
//     return encoder_output[slot];
//   }

//   //speed pid;
//   if ((lock_flag[slot] == false) && (abs(pos_error) >= encoder_data[slot].target_speed * DECELERATION_DISTANCE_PITCH))
//   {
//     speed_error = encoder_data[slot].current_speed - encoder_data[slot].target_speed * (pos_error / abs(pos_error));
//     out_put_offset = encoder_data[slot].PID_speed.P * speed_error;
//     out_put_offset = constrain(out_put_offset, -25, 25);
//     encoder_data[slot].PID_speed.Output = encoder_output[slot];
//     encoder_data[slot].PID_speed.Output -= out_put_offset;
//     encoder_data[slot].PID_speed.Output = constrain(encoder_data[slot].PID_speed.Output, -255, 255);
//     encoder_output[slot] = encoder_data[slot].PID_speed.Output;
//   }

//   //position pid;
//   else
//   {
//     if ((lock_flag[slot] == false) && (abs(pos_error) > ENCODER_POS_DEADBAND))
//     {
//       seek_speed = sqrt(abs(encoder_data[slot].target_speed * DECELERATION_DISTANCE_PITCH * (pos_error - ENCODER_POS_DEADBAND))) / DECELERATION_DISTANCE_PITCH;
//       d_component = encoder_data[slot].current_speed - seek_speed * (pos_error / abs(pos_error));
//       out_put_offset = encoder_data[slot].PID_pos.D * d_component;
//       out_put_offset = constrain(out_put_offset, -20, 20);

//       encoder_data[slot].PID_pos.Output = encoder_output[slot];
//       encoder_data[slot].PID_pos.Output -= out_put_offset;
//       if (pos_error >= 0)
//       {
//         encoder_data[slot].PID_pos.Output = constrain(encoder_data[slot].PID_pos.Output, PWM_MIN_OFFSET, 255);
//       }
//       else
//       {
//         encoder_data[slot].PID_pos.Output = constrain(encoder_data[slot].PID_pos.Output, -255, -PWM_MIN_OFFSET);
//       }
//       encoder_output[slot] = encoder_data[slot].PID_pos.Output;
//     }
//     else
//     {
//       lock_flag[slot] = true;
//       if (encoder_data[slot].motion_state == MOTION_WITH_POS_LOCK)
//       {
//         d_component = encoder_data[slot].current_speed;
//         out_put_offset = encoder_data[slot].PID_pos.D * d_component;
//         out_put_offset = constrain(out_put_offset, -20, 20);
//         encoder_data[slot].PID_pos.Output = pos_error * encoder_data[slot].PID_pos.P;
//         encoder_data[slot].PID_pos.Output -= out_put_offset;
//         encoder_data[slot].PID_pos.Output = constrain(encoder_data[slot].PID_pos.Output, -255, 255);
//         encoder_output[slot] = encoder_data[slot].PID_pos.Output;
//       }
//       else
//       {
//         encoder_output[slot] = 0;
//       }
//     }
//   }
// #ifdef DEBUG_INFO
//   Serial.print("tar1:");
//   Serial.print(encoder_data[slot].target_pos);
//   Serial.print(" ,current_speed:");
//   Serial.print(encoder_data[slot].current_speed);
//   Serial.print(" ,target_speed:");
//   Serial.print(encoder_data[slot].target_speed);
//   Serial.print(" ,cur1:");
//   Serial.print(encoder_data[slot].current_pos);
//   Serial.print(" ,pos_error1:");
//   Serial.print(pos_error);
//   Serial.print(" ,d_component1:");
//   Serial.print(d_component);
//   Serial.print(" ,motion_state1:");
//   Serial.print(encoder_data[slot].motion_state);
//   Serial.print(" ,out1:");
//   Serial.println(encoder_output[slot]);
// #endif
//   return encoder_output[slot];
// }

// int16_t speed_without_pos(uint8_t slot)
// {
//   float speed_error;
//   float out_put_offset;
//   speed_error = encoder_data[slot].current_speed - encoder_data[slot].target_speed;
//   out_put_offset = encoder_data[slot].PID_speed.P * speed_error;

//   out_put_offset = constrain(out_put_offset, -25, 25);
//   encoder_data[slot].PID_speed.Output = encoder_output[slot];
//   encoder_data[slot].PID_speed.Output -= out_put_offset;

//   if ((lock_flag[slot] == true) && (encoder_data[slot].motion_state == MOTION_WITHOUT_POS_LOCK))
//   {
//     encoder_data[slot].PID_speed.Integral += speed_error;
//     out_put_offset = encoder_data[slot].PID_speed.I * encoder_data[slot].PID_speed.Integral;
//     encoder_data[slot].PID_speed.Output = -out_put_offset;
//   }
//   else if ((lock_flag[slot] == true) && (encoder_data[slot].motion_state == MOTION_WITHOUT_POS_RELEASE))
//   {
//     encoder_data[slot].PID_speed.Output = 0;
//     lock_flag[slot] = true;
//   }

//   if ((lock_flag[slot] == false) && (encoder_data[slot].target_speed == 0) && (abs(out_put_offset) < 15))
//   {
//     encoder_data[slot].PID_speed.Output = 0;
//     lock_flag[slot] = true;
//   }

//   encoder_data[slot].PID_speed.Output = constrain(encoder_data[slot].PID_speed.Output, -255, 255);
//   encoder_output[slot] = encoder_data[slot].PID_speed.Output;
// #ifdef DEBUG_INFO
//   Serial.print("Mode:");
//   Serial.print(encoder_data[slot].mode);
//   Serial.print("slot:");
//   Serial.print(slot);
//   Serial.print(" ,tar:");
//   Serial.print(encoder_data[slot].target_speed);
//   Serial.print(" ,cur:");
//   Serial.print(encoder_data[slot].current_speed);
//   Serial.print(" ,speed_error:");
//   Serial.print(speed_error);
//   Serial.print(" ,out:");
//   Serial.println(encoder_output[slot]);
// #endif
//   return encoder_output[slot];
// }

// void initMotor()
// {
//   memset(&encoder_data[MOTOR_0], 0, sizeof(encoder_data_type));
//   memset(&encoder_data[MOTOR_1], 0, sizeof(encoder_data_type));
//   dev_id = encoder_eeprom.devid;
//   encoder_data[MOTOR_0].slot = 0;
//   encoder_data[MOTOR_0].mode = I2C_MODE;
//   encoder_data[MOTOR_0].motion_state = 0;
//   encoder_data[MOTOR_0].tar_pwm = 0;
//   encoder_data[MOTOR_0].cur_pwm = 0;
//   encoder_data[MOTOR_0].pulse = 0;
//   encoder_data[MOTOR_0].current_speed = 0;
//   encoder_data[MOTOR_0].target_speed = 0;
//   encoder_data[MOTOR_0].current_pos = 0;
//   encoder_data[MOTOR_0].previous_pos = 0;
//   encoder_data[MOTOR_0].target_pos = 0;
//   encoder_data[MOTOR_0].PID_speed.P = encoder_eeprom.speed0_p;
//   encoder_data[MOTOR_0].PID_speed.I = encoder_eeprom.speed0_i;
//   encoder_data[MOTOR_0].PID_speed.D = encoder_eeprom.speed0_d;
//   encoder_data[MOTOR_0].PID_pos.P = encoder_eeprom.pos0_p;
//   encoder_data[MOTOR_0].PID_pos.I = encoder_eeprom.pos0_i;
//   encoder_data[MOTOR_0].PID_pos.D = encoder_eeprom.pos0_d;
//   //  encoder_data[MOTOR_0].PID_speed.P = 0.18;
//   //  encoder_data[MOTOR_0].PID_speed.I = 0.64;
//   //  encoder_data[MOTOR_0].PID_speed.D = 0;
//   //  encoder_data[MOTOR_0].PID_pos.P = 1.8;
//   //  encoder_data[MOTOR_0].PID_pos.I = 0;
//   //  encoder_data[MOTOR_0].PID_pos.D = 1.2;
//   encoder_data[MOTOR_0].ratio = encoder_eeprom.ratio0;
//   encoder_data[MOTOR_0].encoder_pulse = encoder_eeprom.pulse0;

//   encoder_data[MOTOR_1].slot = 1;
//   encoder_data[MOTOR_1].mode = I2C_MODE;
//   encoder_data[MOTOR_1].motion_state = 0;
//   encoder_data[MOTOR_1].tar_pwm = 0;
//   encoder_data[MOTOR_1].cur_pwm = 0;
//   encoder_data[MOTOR_1].pulse = 0;
//   encoder_data[MOTOR_1].current_speed = 0;
//   encoder_data[MOTOR_1].target_speed = 0;
//   encoder_data[MOTOR_1].current_pos = 0;
//   encoder_data[MOTOR_1].previous_pos = 0;
//   encoder_data[MOTOR_1].target_pos = 0;
//   encoder_data[MOTOR_1].PID_speed.P = encoder_eeprom.speed1_p;
//   encoder_data[MOTOR_1].PID_speed.I = encoder_eeprom.speed1_i;
//   encoder_data[MOTOR_1].PID_speed.D = encoder_eeprom.speed1_d;
//   encoder_data[MOTOR_1].PID_pos.P = encoder_eeprom.pos1_p;
//   encoder_data[MOTOR_1].PID_pos.I = encoder_eeprom.pos1_i;
//   encoder_data[MOTOR_1].PID_pos.D = encoder_eeprom.pos1_d;
//   //  encoder_data[MOTOR_1].PID_speed.P = 0.18;
//   //  encoder_data[MOTOR_1].PID_speed.I = 0.64;
//   //  encoder_data[MOTOR_1].PID_speed.D = 0;
//   //  encoder_data[MOTOR_1].PID_pos.P = 1.8;
//   //  encoder_data[MOTOR_1].PID_pos.I = 0;
//   //  encoder_data[MOTOR_1].PID_pos.D = 1.2;
//   encoder_data[MOTOR_1].ratio = encoder_eeprom.ratio1;
//   encoder_data[MOTOR_1].encoder_pulse = encoder_eeprom.pulse1;
// }

// void resetMotor(void)
// {
//   setMotor1Pwm(0);
//   setMotor2Pwm(0);
//   delay(10);
//   readEEPROM();
//   delay(10);
//   initMotor();
//   I2C_init((dev_id) << 1);
//   encoder_move_time[MOTOR_0] = encoder_move_time[MOTOR_1] = measurement_speed_time = millis();
// }

// void moveTo(uint8_t slot, uint8_t state, long turns, float speed)
// {
//   encoder_data[slot].mode = I2C_MODE;
//   lock_flag[slot] = false;
//   if (LOCK_STATE == state)
//   {
//     encoder_data[slot].motion_state = MOTION_WITH_POS_LOCK;
//   }
//   else
//   {
//     encoder_data[slot].motion_state = MOTION_WITH_POS_RELEASE;
//   }

//   encoder_data[slot].target_pos = turns;
//   float speed_temp = constrain(speed, MIN_ENCODER_SPEED, MAX_ENCODER_SPEED);
//   encoder_data[slot].target_speed = abs(speed_temp);
//   if (encoder_distance_togo(slot) > 0)
//   {
//     dir_lock_flag[slot] = true;
//   }
//   else
//   {
//     dir_lock_flag[slot] = false;
//   }
// }

// void move(uint8_t slot, uint8_t state, long turns, float speed)
// {
//   moveTo(slot, state, (encoder_data[slot].current_pos + turns), speed);
// }

// void move_speed(uint8_t slot, uint8_t state, float speed)
// {
//   encoder_data[slot].mode = I2C_MODE;
//   lock_flag[slot] = false;
//   if (LOCK_STATE == state)
//   {
//     encoder_data[slot].motion_state = MOTION_WITHOUT_POS_LOCK;
//   }
//   else
//   {
//     encoder_data[slot].motion_state = MOTION_WITHOUT_POS_RELEASE;
//   }
//   float speed_temp = constrain(speed, MIN_ENCODER_SPEED, MAX_ENCODER_SPEED);
//   encoder_data[slot].target_speed = speed_temp;
// }

/**********************'
 * slot is which motor. State is whether the motor should be free to move or not. Speed is from -255 to 255.
************************/ 
void move_speed(uint8_t slot, uint8_t state, float speed)
{
  // TODO Maybe need to add a constrain to make sure speed is within bounds?
  if(slot == 0)
  {
    motor1TargetSpeed = speed;
    if(state == LOCK_STATE)
    {
      motor1Released = false;
    }
    else
    {
      motor1Released = true;
    }
  }
  if(slot == 1)
  {
    motor2TargetSpeed = speed;
    if(state == LOCK_STATE)
    {
      motor2Released = false;
    }
    else
    {
      motor2Released = true;
    }
  }
}

void setDevid(uint8_t devid)
{
  // encoder_eeprom.devid = devid;
  ByteToEEPROM(EEPROM_DEVID_ADDR, 1, &devid);
}

uint8_t getDevid(void){
  uint8_t dev_id = 0;
  return EEPROM.get(EEPROM_DEVID_ADDR, dev_id);

}

void pwm_frequency_init(void)
{
  // TCCR1A = _BV(WGM10);
  // TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

  // TCCR2A = _BV(WGM21) | _BV(WGM20);
  // TCCR2B = _BV(CS22);
#ifdef PWMCHANGED
  //WGM2:0 = 1 //Phase correct mode = half frequency.
  //TCCR0A : COM0A1 | COM0A0 | COM0B1 | COM0B0 | ------ | ------ | WGM01 | WGM00
  // TCCR0A = TCCR0A & B11111100 | B00000001;
  
  // TCCR0B : FOC0A | FOC0B | ---- | ---- | WGM02 | CS02 | CS01 | CS00
  #if PWMADJUSTER == 64
  TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz
  #elif PWMADJUSTER == 32
  TCCR0B = TCCR0B & B11110000 | B00000001; // for PWM frequency of 31250.00 Hz
  TCCR0A = TCCR0A & B11111100 | B00000001;
  #elif PWMADJUSTER == 8
  TCCR0B = TCCR0B & B11111000 | B00000010; // 7812.5 Hz 
  #elif PWMADJUSTER == 4
  TCCR0B = TCCR0B & B11110000 | B00000010;
  TCCR0A = TCCR0A & B11111100 | B00000001;
  #endif
#endif
}
/****************************************************************************************************
 * Arduino main function
****************************************************************************************************/


// char Uart_Buf[64];
// char bufindex;

#define WHEELCIRCUMFERENCEMM 314.159265359
#define ENCODERVALUESARRAYLENGTH 100

struct encoderData {
  int position = 0;
  int deltaPosition = 0;
  float filteredDeltaPosition = 0;
  int deltaPositions[ENCODERVALUESARRAYLENGTH];
  int deltaPositionsIndex = 0;
  int deltaPositionSum = 0;
  float deltaSpeed = 0;
  float ticksPerMillis = 0;
  float RPM = 0;
  float metersPerSecond = 0;
};

struct encoderData enc[2];

// int encoder1Pos = 0;
// int encoder1Delta = 0;
// int16_t encoder1Deltas[ENCODERVALUESARRAYLENGTH];
// int encoder1DeltasIndex = 0;
// int encoder1DeltasSum = 0;
// float encoder1MovingAvg = 0;
// float encoder1Speed = 0;

// int encoder2Pos = 0;
// int encoder2Delta = 0;
// int16_t encoder2Deltas[ENCODERVALUESARRAYLENGTH];
// int encoder2DeltasIndex = 0;
// int encoder2DeltasSum = 0;
// float encoder2MovingAvg = 0;
// float encoder2Speed = 0;


const unsigned long encoderUpdateInterval = 5;
unsigned long encoderUpdateStamp = 0;
float encoderFilterC = 0.1;
unsigned long encoderWrapDuration = 0;
unsigned long encoderWrapStamp = 0;

void updateEncoders(unsigned long now){
  if (now - encoderUpdateStamp > encoderUpdateInterval){

    //Rather than setting the stamp to equal *now* we increment with interval. This is to have the interval more exact on average.
    //The error introduced in one interval should get corrected for in the next interval, if you get what I mean.
    //This might not work if the if statement is called to rarely. In that case the main loop will outrun the timing check.
    //The interval will then run each time the if-statement is called, but I guess that would happen with normal timestamping aswell.
    encoderUpdateStamp += encoderUpdateInterval;

    //If due, calculate wraparound duration.
    //We use the duration for the wraparound instead of the duration for each interval.
    //It seems this give better accuracy.
    if(enc[1].deltaPositionsIndex == 0){
      encoderWrapDuration = now - encoderWrapStamp;
      encoderWrapStamp = now;
    }

    long newEncValues[2] = {encoder1.read(), encoder2.read()};
    for(int i = 0; i < 2; i++){
      enc[i].deltaPosition = newEncValues[i] - enc[i].position;
      enc[i].position = newEncValues[i];

      // accumulating part
      {
        enc[i].deltaPositionSum -= enc[i].deltaPositions[enc[i].deltaPositionsIndex];

        // enc[i].deltaSpeed = (float)enc[i].deltaPosition / (float)deltaTime * 1000.0;

        enc[i].deltaPositions[enc[i].deltaPositionsIndex] = enc[i].deltaPosition;

        enc[i].deltaPositionSum += enc[i].deltaPositions[enc[i].deltaPositionsIndex];

        enc[i].deltaPositionsIndex++;
        enc[i].deltaPositionsIndex %= ENCODERVALUESARRAYLENGTH;
      }

      enc[i].ticksPerMillis = (float) enc[i].deltaPositionSum / (float) encoderWrapDuration;
      enc[i].RPM = enc[i].ticksPerMillis * 1000.0 * 60.0 / ENCODER_PULSES_PER_ROTATION;
      enc[i].metersPerSecond = (enc[i].RPM / 60.0) * (WHEELCIRCUMFERENCEMM / 1000.0);

      // Serial.print(enc[i].metersPerSecond);
      // Serial.print(", \t");
    }
    // Serial.println();
  }  
}
const unsigned long pidUpdateInterval = 10;
unsigned long pidUpdateStamp = 0;


unsigned long now = 0;
unsigned long loopTime = 0;
bool sampleTimeIsSet = false;


unsigned long randomSetpointStamp = 0;


void setup()
{
  delay(10);
  Serial.begin(115200);
  pwm_frequency_init();

  // These pins sets the direction of the motor circuit.
  pinMode(MOTOR_1_H1, OUTPUT);
  pinMode(MOTOR_2_H1, OUTPUT);

  // These lines enable interrupt on falling edge of INT0 (PD2)(arduino pin 2) and INT1 (PD3)(arduino pin 3)
  // Commenting out in order to use the encoder library, which automatically will change pin mode and attach interrupts where applicable.
  // EICRA = (1 << ISC11) | (1 << ISC01);
  // EIMSK = (1 << INT0) | (1 << INT1);

  delay(10);
  // readEEPROM();
  // delay(10);
  // initMotor();

  motor1Setpoint = 0;
  motor2Setpoint = 0;
  motor1PID.SetMode(AUTOMATIC);
  //This is pretty weird. But we have a precalculation inside the PID that offsets values that wouldn't make the motor move.
  motor1PID.SetOutputLimits(-250 + PWM_MOTION_THRESHOLD, 250 - PWM_MOTION_THRESHOLD);
  
  motor2PID.SetMode(AUTOMATIC);
  motor2PID.SetOutputLimits(-250 + PWM_MOTION_THRESHOLD, 250 - PWM_MOTION_THRESHOLD);

  motor1PID.SetSampleTime(pidUpdateInterval);
  motor2PID.SetSampleTime(pidUpdateInterval);

  I2C_init(getDevid() << 1);
  // I2C_init((DEFAULT_I2C_ADDR) << 1);
  // Serial.println("Driver board says heeellloooo!!!! Much power! Such high current! Wow!");
}

void loop()
{
  loopTime = millis() - now;
  now = millis();

  if(loopTime > encoderUpdateInterval)
  {
     Serial.println("WARNING long looptime (encoder)");
  }
  if(loopTime > pidUpdateInterval)
  {
    Serial.println("WARNING long looptime (pid)");
  }

  updateEncoders(now);

  // if(now - randomSetpointStamp > 2000){
  //   randomSetpointStamp = now;
  // #ifdef DONT_USE_METER_PER_SECOND_AS_INPUT
  //   pickRandomTargetSpeed(0, -250.0f, 250.0f);
  //   pickRandomTargetSpeed(1, -250.0f, 250.0f);
  // #else
  //   pickRandomTargetSpeed(0, -2.0f, 2.0f);
  //   pickRandomTargetSpeed(1, -2.0f, 2.0f);
  // #endif
  // }

  

  if (now - pidUpdateStamp > pidUpdateInterval)
  {
    //Rather than setting the stamp to now we increment with interval. This is to have the interval more exact on average.
    //The error introduced in one interval should get corrected for in the next interval, if you get what I mean.
    //This might not work if the if statement is called to rarely. In that case the main loop will outrun the timing check.
    //The interval will then run each time the if-statement is called.
    pidUpdateStamp += pidUpdateInterval;

  #ifdef DONT_USE_METER_PER_SECOND_AS_INPUT
    motor1Setpoint = pwmValueToMpS(motor1TargetSpeed);
    motor2Setpoint = pwmValueToMpS(motor2TargetSpeed);
  #else
    motor1Setpoint = motor1TargetSpeed;
    motor2Setpoint = motor2TargetSpeed;
  #endif

    motor1Input = enc[0].metersPerSecond;
    motor2Input = enc[1].metersPerSecond;

    motor1PID.Compute();
    motor2PID.Compute();

    int16_t motor1PwmValue = rescalePwmValue(motor1Output);
    int16_t motor2PwmValue = rescalePwmValue(motor2Output);

    setMotorPwm(0, motor1PwmValue);
    setMotorPwm(1, motor2PwmValue);

    Serial.print(motor2Setpoint);
    Serial.print(", ");
    Serial.print(motor2Input);
    Serial.print(", ");
    // Serial.print(motor2Output);
    // Serial.print(", ");
    // Serial.print(motor2PwmValue);
    
    Serial.println(",");
  }
  //Serial.println(loopTime);
  
  
  
  

  // Serial.print("PID setpoint: ");
  // Serial.print(motor1Setpoint);
  // Serial.print(", PID input: ");
  // Serial.print(motor1Input);
  // Serial.print(", PID output: ");
  // Serial.print(motor1Output);
  // Serial.println();

  // Serial.println(String(motor1Setpoint) + "," + String(motor1Input) + "," + String(motor1Output));

  //Serial.println(String(motor1Setpoint) + "," + String(motor1Input));
  //Serial.println(String((millis()/100)%60) + ", " + String(motor2Setpoint) + ", " + String(motor2Input) + ", " + String(motor2Output/10.0));
  // Serial.println(String(motor2Setpoint) + ", " + String(motor2Input) + ", " + String(motor2Output));
}

int16_t rescalePwmValue(float pwmValue){
  if(abs(pwmValue) < PWM_NO_MOTION_THRESHOLD)
    pwmValue = 0;
  else{
    if(pwmValue > 0){
      pwmValue += PWM_MOTION_THRESHOLD;
    }else{
      pwmValue -= PWM_MOTION_THRESHOLD;
    }
  }
  return pwmValue;
}

float pwmValueToMpS(int16_t pwmValue){
  // TODO: double check if map function can handle floats!!!!
  float rescaledValue = map(pwmValue, -250, 250, -1800, 1800); 
  return rescaledValue/1000.0f;
}

void pickRandomTargetSpeed(int slot, float min, float max){
  float randomValue = ((float) random(min*1000, max*1000))/1000.0;
  if(slot == 0){
    motor1TargetSpeed = randomValue;
  }else{
    motor2TargetSpeed = randomValue;
  }
}
