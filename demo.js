#!/usr/bin/env node

const pwm = require('./index');

// DMA channel: (avoid those DMA(s) already occupied by others)
// for example, GPU uses 1, 3, 6, 7
// frame buffer uses 0 and the SD card uses 2.
const chNum = pwm.host_is_model_pi4() ? 7 : 14; // DMA channel 14 for pi2/3/zero, channel 7 for pi4
const pinNum = 21; // GPIO 21

pwm.set_log_level(pwm.logLevel.info);

const ch = pwm.create_dma_channel(chNum, {
  cycle_time_us: 20000, // default 20000us (20ms)
  step_time_us: 10, // default 10us
  delay_hw: 0, // 0=PWM, 1=PCM, (default 0)
  invert: 0, // invert high/low? (default no invert)
});

const pin = ch.create_pwm(pinNum);

pin.set_width(1000);
setTimeout(function () {
  // Note: it's good idea to cleanup before exit.
  ch.shutdown();
}, 2000);
