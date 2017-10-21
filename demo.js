var pwm = require('./index');

// console.log(pwm.hello());
var channel = 13;

pwm.setup(1); // 10us resolution
pwm.init_channel(channel, 3000); // DMA channel 14; loop length 3000us

pwm.add_channel_pulse(channel, 26, 0, 1500);

setTimeout(function() {
    console.log('cleanup');
    pwm.clear_channel(channel); // DMA channel 14
    pwm.cleanup();
}, 2000);

