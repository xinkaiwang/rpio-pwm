/*
 * This file is part of RPIO-PWM.
 *
 * Copyright
 *
 *     Copyright (C) 2020 Xinkai Wang <xinkaiwang1017@gmail.com>
 *
 * License
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU Lesser General Public License as published
 *     by the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU Lesser General Public License for more details at
 *     <http://www.gnu.org/licenses/lgpl-3.0-standalone.html>
 *
 * Documentation
 *
 *     https://github.com/xinkaiwang/rpio-pwm
 */

// #define DELAY_VIA_PWM   0
// #define DELAY_VIA_PCM   1

#ifdef __cplusplus 
extern "C" {
#endif
// void pwm_shutdown(void);

// channel: suggest 14 (for pi2/3/zero), suggest 7 for pi4
// delay_hw: 0=PWM, 1=PCM
// cycle_time_us: typical value 20000=20ms each cycle
// step_time_us: typical value 10=10us each step 
//               (total steps = 20000/10=2000 steps)
// invert: invert HIGH/LOW output, default 0
int pwm_channel_init(int channel, int cycle_time_us, int step_time_us, int delay_hw, int invert);

// shutdown channel before exit is always a good thing to do
int pwm_channel_shutdown(int channel);

int pwm_gpio_add(int channel, int gpio, int width);
int pwm_gpio_set_width(int gpio, int width);
int pwm_gpio_release(int gpio);
int pwm_host_is_model_pi4(); // return true/false

int pwm_set_log_level(int level);

#ifdef __cplusplus
}
#endif
