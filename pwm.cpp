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
 *
 *
*/


#include "pwm.h"

#include <stdio.h>
#include <unordered_map>
// #include <stdlib.h>
#include <unistd.h>
// #include <string.h>
// #include <errno.h>
// #include <stdarg.h>
// #include <stdint.h>
// #include <signal.h>
// #include <time.h>
// #include <sys/time.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <sys/mman.h>
// #include "pwm.h"
// #include "mailbox.h"
// #include <sys/sysmacros.h>

#include "dma.h"

using namespace wpp;
namespace {

std::unordered_map<int, std::shared_ptr<DmaChannel>> channels;

std::unordered_map<int, std::shared_ptr<PwmPin>> pins;

} // anonymous namespace

// channel: suggest 14 (for pi2/3/zero), suggest 7 for pi4
// delay_hw: 0=PWM, 1=PCM
// cycle_time_us: typical value 20000=20ms each cycle
// step_time_us: typical value 10=10us each step 
//               (total steps = 20000/10=2000 steps)
// invert: invert HIGH/LOW output, default 0
int pwm_channel_init(int ch, int cycle_time_us, int step_time_us, int delay_hw, int invert) {
  if (channels.find(ch) != channels.end()) {
    // channel already exist?
    return 1;
  }
  DmaChannelConfig config{};
  config.chNum = ch;
  config.cycleTimeUs = cycle_time_us;
  config.stepTimeUs = step_time_us;
  config.delay_hw = delay_hw?DelayHardware::DELAY_VIA_PCM:DelayHardware::DELAY_VIA_PWM;
  config.invert = invert != 0;
  auto channel = DmaChannel::CreateInstance(config);
  channels[ch] = channel;
  return 0;
}

// shutdown channel before exit is always a good thing to do
int pwm_channel_shutdown(int ch) {
  auto it = channels.find(ch);
  if (it == channels.end()) {
    // channel don't exist?
    return 1;
  }
  it->second->DeactivateChannel();
  channels.erase(it);
  return 0;
}

int pwm_gpio_add(int ch, int gpio, int width) {
  auto channel = channels.find(ch);
  if (channel == channels.end()) {
    // channel don't exist?
    return 1;
  }
  if (pins.find(gpio) != pins.end()) {
    // pin already exist?
    return 1;
  }
  DmaPwmPinConfig config{};
  config.gpioPinNum = gpio;
  config.widthInSteps = width;
  auto pin = channel->second->CreatePin(config);
  pins[gpio] = pin;
  return 0;
}

int pwm_gpio_set_width(int gpio, int width) {
  auto pin = pins.find(gpio);
  if (pin == pins.end()) {
    // pin don't exist?
    return 1;
  }
  pin->second->SetByWidth(width);
  return 0;
}

int pwm_gpio_release(int gpio) {
  auto pin = pins.find(gpio);
  if (pin == pins.end()) {
    // pin don't exist?
    return 1;
  }
  pin->second->DeactivatePin();
  pins.erase(pin);
  return 0;
}

// return true/false
int pwm_host_is_model_pi4() {
  return DmaHardware::GetInstance().host_is_model_pi4;
}

int pwm_set_log_level(int log_level) {
  DmaHardware::GetInstance().current_log_level = static_cast<LogLevel>(log_level);
  return 0;
}

int main() {
  printf("init\n");
  int ch = 14;
  int pin = 21; // GPIO_21 = Phys_40 = wiringPi_29
  unsigned int sleepUs = 1*1000*1000;
  pwm_channel_init(ch, 20000, 10, 1 /*delay_hw*/, 0 /*invert*/);
  pwm_gpio_add(ch, pin, 2000);
  usleep(sleepUs);
  pwm_gpio_set_width(pin, 800);
  usleep(sleepUs);
  pwm_gpio_set_width(pin, 400);
  usleep(sleepUs);
  pwm_gpio_set_width(pin, 200);
  usleep(sleepUs);
  pwm_gpio_set_width(pin, 100);
  usleep(sleepUs);
  pwm_gpio_set_width(pin, 50);
  usleep(sleepUs);
  pwm_gpio_set_width(pin, 20);
  usleep(sleepUs);
  pwm_gpio_set_width(pin, 10);
  usleep(sleepUs);
  pwm_gpio_set_width(pin, 2000);
  usleep(sleepUs);
  printf("Done\n");
  pwm_channel_shutdown(ch);
}
