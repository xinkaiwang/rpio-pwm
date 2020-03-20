# rpio-pwm
High performance soft PWM for raspberry pi. Unlike hardware PWM (which limited to those special 4 hardware PWM ports), rpio-pwm allow you to use any GPIO pins. Unlike other soft PWM solutions (which use lot's CPU cycles to switch on/off based on timers), rpio-pwm use DMA, so literily 0 CPU usage at run time.

Note: This current version (1.0.0) is NOT compatible with privous versions (0.2.x). This is a total re-write, the new API is much simpler and cleaner. And the new code is only depend on servoblaster (no python rpio-pwm anymore).

# install

```
npm install rpio-pwm --save
```

# Quick Start 
## Simple 
``` js
var pwm = require('rpio-pwm');

var chNum = 14; // DMA channel 14
var pinNum = 21; // GPIO 21
var ch = pwm.create_dma_channel(chNum);
var pin = ch.create_pwm(pinNum);
pin.set_width(100); // 100 * 10us=1000us
```

## More config

``` js
var pwm = require('rpio-pwm');
var chNum = 14; // DMA channel 14
var pinNum = 21; // GPIO 21

var cfg = {
  cycle_time_us: 20000, // default 20000us (20ms)
  step_time_us: 10, // default 10us
  delay_hw: 0, // 0=PWM, 1=PCM, (default 0)
  invert: 0, // invert high/low (default 0=no invert)
};
var ch = pwm.create_dma_channel(chNum, cfg);
var pin = ch.create_pwm(pinNum);
pin.set_width(1000);
setTimeout(function () {
  // it's good idea to cleanup before exit.
  ch.shutdown();
}, 2000);
```

# Notes

`channel`: rpi have 15 DMA channels available (0-14). But you need to know which one might already in use by other services (I found this thread is helpful https://www.raspberrypi.org/forums/viewtopic.php?f=32&t=86339).


# Thanks to servoblaster
This rpio-pwm is based on the awesome work of servoblaster (git://github.com/richardghirst/PiBits.git).

# license

MIT
