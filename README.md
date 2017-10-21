# rpio-pwm
High performance soft PWM for raspberry pi. Unlike hardware PWM (which limited to those special 4 hardware PWM ports), rpio-pwm allow you to use any GPIO pins. Unlike other soft PWM solutions (which use lot's CPU cycles to switch on/off based on timers), rpio-pwm use DMA, so literily 0 CPU usage at run time.

# install

```
npm install rpio-pwm --save
```

# Quick Start 
## Simple 
``` js
var pwm = require('rpio-pwm');

pwm.setup(1); // 1 us resolution
pwm.init_channel(14, 3000); // DMA channel 14; loop interval 3000us
pwm.add_channel_pulse(14, 26, 0, 1500); // GPIO 26, output 'ON' start from position 0, last for 1500 samples

setTimeout(function() {
    console.log('cleanup');
    pwm.clear_channel_gpio(14, 26); // stop output when you exit
    pwm.clear_channel(14);
    pwm.cleanup();
}, 2000);
```

## multiple time slots and multiple pins
Once a chnnel has been initialized, you can call add_channel_pulse() multiple time (either on different pins, or different time slots).
for instance:
``` js
pwm.add_channel_pulse(14, 26, 0, 1500);
pwm.add_channel_pulse(14, 26, 1500, 1500);
```
is equavlent to
``` js
pwm.add_channel_pulse(14, 26, 0, 3000);
```

Of course you can do this on different pins as well.
``` js
pwm.add_channel_pulse(14, 26, 0, 1500);
pwm.add_channel_pulse(14, 27, 0, 1500);
```

# API

#### `setup(resolutionUs)`
Init hardware registers, etc.. `resolutionUs` is the time resolution (in us) 1 sample will represent. 
##### Note: max resolution is 1us (resolutionUs=1)

#### `init_channel(channel, timeIntervalUs)`
`channel` is DMA channel you want to use, rpi have 15 DMA channels available (0-14). But you need to know which one might already in use by other services (I found this thread is helpful https://www.raspberrypi.org/forums/viewtopic.php?f=32&t=86339).

`timeIntervalUs` is the loop time interval in us. For example, typical servo control signal you need 20ms loop interval so you want `timeIntervalUs=20000`. 
##### Note: timeIntervalUs muse be multiplications of resolutionUs (for obvious reason).
##### Note: samplesInLoop = timeIntervalUs/resolutionUs;
##### Note: I found samplesInLoop need to be >= 1000 (don't know why). And also you don't want samplesInLoop to be too large (wasting memory).

#### `add_channel_pulse(channel, gpio_port, width_start, width)`
`gpio_port` is GPIO port number (int). for instance, 26 means GPIO26.
`width_start`/`width` in samples count, where to start and what is the width
##### Note `add_channel_pulse()` can be called multiple times, on different pins; or different time slot on same pin.


#### `clear_channel_gpio(channel, gpio_port)`
shutdown a GPIO port
#### `clear_channel(channel)`
shutdown a channel
#### `cleanup()`
shutdown



# Thanks to (python) rpio and servoblaster
I steal-and-modify pwm.c/h from python rpio (https://github.com/metachris/RPIO/tree/v2), which (soft PWM part) is based on the awesome work of servoblaster (git://github.com/richardghirst/PiBits.git). I steal mailbox.c/h from servoblaster as well :).


# license

MIT
