/*
  spindle_control.c - Modified spindle controller to control a servo
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

static float pwm_gradient;

#define PEN_SERVO_DOWN     16      
#define PEN_SERVO_UP      31  

void spindle_init()
{
  SPINDLE_PWM_DDR |= (1 << SPINDLE_PWM_BIT); // Configure as PWM output pin.
  SPINDLE_TCCRA_REGISTER = (1 << COM2A1) | ((1 << WGM20) | (1 << WGM21));
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  SPINDLE_DIRECTION_DDR |= (1 << SPINDLE_DIRECTION_BIT);
  pwm_gradient = SPINDLE_PWM_RANGE / (settings.rpm_max - settings.rpm_min);
  spindle_stop();
}

uint8_t spindle_get_state()
{
  if (SPINDLE_TCCRA_REGISTER & (1 << SPINDLE_COMB_BIT))
  { // Check if PWM is enabled.
    if (SPINDLE_DIRECTION_PORT & (1 << SPINDLE_DIRECTION_BIT))
    {
      return (SPINDLE_STATE_CCW);
    }
    else
    {
      return (SPINDLE_STATE_CW);
    }
  }
  return (SPINDLE_STATE_DISABLE);
}

void spindle_stop()
{
  SPINDLE_OCR_REGISTER = PEN_SERVO_UP;
}

void spindle_set_speed(uint8_t pwm_value)
{
  if (pwm_value == SPINDLE_PWM_OFF_VALUE)
  {
    spindle_stop();
  }
  else
  { // not off
    SPINDLE_OCR_REGISTER = PEN_SERVO_DOWN;
  }
}

void spindle_set_state(uint8_t state, float rpm)
{
  if (sys.abort)
  {
    return;
  }
  if (state == SPINDLE_DISABLE)
  {
    sys.spindle_speed = 0.0;
    spindle_stop();
  }
  else
  {
    if (state == SPINDLE_ENABLE_CW)
    {
      SPINDLE_DIRECTION_PORT &= ~(1 << SPINDLE_DIRECTION_BIT);
    }
    else
    {
      SPINDLE_DIRECTION_PORT |= (1 << SPINDLE_DIRECTION_BIT);
    }
    // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
    if (settings.flags & BITFLAG_LASER_MODE)
    {
      if (state == SPINDLE_ENABLE_CCW)
      {
        rpm = 0.0;
      } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
    }
    spindle_set_speed(spindle_compute_pwm_value(rpm));
  }
  sys.report_ovr_counter = 0;
}

void spindle_sync(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE)
  {
    return;
  }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
  spindle_set_state(state, rpm);
}

uint8_t spindle_compute_pwm_value(float rpm)
{
  uint8_t pwm_value;
  rpm *= (0.010 * sys.spindle_speed_ovr);

  if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max))
  {
    sys.spindle_speed = settings.rpm_max;
    pwm_value = 255;
  }
  else if (rpm <= settings.rpm_min)
  {
    if (rpm == 0.0)
    {
      sys.spindle_speed = 0.0;
      pwm_value = 0;
    }
    else
    {
      sys.spindle_speed = settings.rpm_min;
      pwm_value = 1;
    }
  }
  else
  {
    sys.spindle_speed = rpm;
    pwm_value = floor((rpm - settings.rpm_min) * pwm_gradient) + 1;
  }
  return (pwm_value);
}