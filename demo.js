var pwm = require('./index');

// console.log(pwm.hello());

// DMA channel: (avoid those DMA(s) already occupied by others)
// for example, GPU uses 1, 3, 6, 7
// frame buffer uses 0 and the SD card uses 2.
var channel = 14; // DMA channel 14

var pin = 21; // GPIO 21
var pin2 = 26; // GPIO 26
pwm.setup(10); // 10us resolution
pwm.init_channel(channel, 20000); // DMA channel 14; loop interval 20ms

// when resolution = 10us, cycle=20000 us, which means we can set 
// pulse-wide 0 (all low) - 2000 (all high)
pwm.add_channel_pulse(channel, pin, 0 /*start*/, 500 /*duration*/); // GPIO 21, start from 0

// one DMA channel can support multiple pins
pwm.add_channel_pulse(channel, pin2, 1000 /*start*/, 1000 /*duration*/); // GPIO 26, start from 0

// one DMA channel can support multiple pulse on the same pin
pwm.add_channel_pulse(channel, pin, 1000/*start*/, 500/*duration*/); // GPIO 21, start from 1000

setTimeout(function () {
    // Note: it's important to do cleanup before exit.
    // Otherwise you may run into problem when next time init mailbox.
    console.log('cleanup');
    pwm.clear_channel_gpio(channel, pin);
    pwm.clear_channel_gpio(channel, pin2);
    pwm.clear_channel(channel);
    pwm.cleanup();
}, 2000);

