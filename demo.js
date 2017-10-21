var pwm = require('./index');

// console.log(pwm.hello());
var channel = 14;

pwm.setup(10); // 10us resolution
pwm.init_channel(channel, 20000); // DMA channel 14; loop interval 20ms
pwm.add_channel_pulse(channel, 26, 0, 500); // GPIO 26, start from 0
pwm.add_channel_pulse(channel, 26, 1000, 500); // GPIO 26, start from 0

setTimeout(function() {
    console.log('cleanup');
    pwm.clear_channel_gpio(channel, 26);
    pwm.clear_channel(channel); // DMA channel 14
    pwm.cleanup();
}, 2000);

