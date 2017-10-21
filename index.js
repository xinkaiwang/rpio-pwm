
var pwm = require('./build/Release/rpiopwm.node');

var exp = {
    hello: function() { return pwm.hello(); },
    setup: function(minResolutionUs) { pwm.setup(minResolutionUs); },
    init_channel: function(dma_channel, subcycle_time_us) { pwm.init_channel(dma_channel, subcycle_time_us); },
    add_channel_pulse: function(dma_channel, gpio_port, width_start, width) { pwm.add_channel_pulse(dma_channel, gpio_port, width_start, width); },
    clear_channel: function(dma_channel) { pwm.clear_channel(dma_channel); },
    clear_channel_gpio: function(dma_channel, gpio_port) { pwm.clear_channel_gpio(dma_channel, gpio_port); },
    cleanup: function() { return pwm.cleanup(); },
};

module.exports = exp;