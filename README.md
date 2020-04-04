# rpio-pwm
High performance soft PWM for raspberry pi. Unlike hardware PWM (which limited to those special 4 hardware PWM ports), rpio-pwm allow you to use any GPIO pins. Unlike other soft PWM solutions (which use lot's CPU cycles to switch on/off based on timers), rpio-pwm use DMA, so literily 0 CPU usage at run time.

Note: This current version (1.0.0) is NOT compatible with privous version (0.2.x). This is a total re-write, the new API is much simpler and also more powerful. The new code is only depend on servoblaster.

Pi4 support: Yes, rpio-pwm >1.0.0 supports Pi4, remember to use channel 7 (on Pi4) in stead of 14 (for pi3/2/zero). I tested, it works out of the box.

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

## More control

``` js
var pwm = require('rpio-pwm');
// DMA channel 14 for pi2/3/zero, channel 7 for pi4
var chNum = pwm.host_is_model_pi4() ? 7 : 14;
var pinNum = 21; // GPIO 21

pwm.set_log_level(pwm.logLevel.debug); // by default info

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

`gpio`: multiple GPIO pin can be created under 1 channel (max 32 pin per channel, is it enough?).

`delay_hw`: You may choice to use 0=PWM or 1=PCM as delay hardware source. This can be useful in case if you need 1) setup multiple DMA channel with different step_time_us. 2) one of the hardware is alreay be used by something else. 

`shutdown`: you can shutdown 1 gpio pin by `pin.release()`, or shutdown the whole DMA channel with `ch.shutdown()`. Typically you want to register a signal handler and shutdown DMA channal before exit.

# Thanks to servoblaster
This rpio-pwm is based on the awesome work of servoblaster (git://github.com/richardghirst/PiBits.git).

# license

MIT
