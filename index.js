
var pwm = require('./build/Release/rpiopwm.node');

function create_dma_channel(ch /*int*/, cfg /*json obj*/) {
  var cycle_time_us = (cfg&&cfg.cycle_time_us)?cfg.cycle_time_us:20000; // by default 20ms
  var step_time_us = (cfg&&cfg.step_time_us)?cfg.step_time_us:10; // by default 10us
  var delay_hw = (cfg&&cfg.delay_hw)?cfg.delay_hw:0; // by default 0 (PWM)
  var invert = (cfg&&cfg.invert)?cfg.invert:0; // by default 0 (don't invert)
  pwm.init_channel(ch, cycle_time_us, step_time_us, delay_hw, invert);

  function create_pwm(gpio) {
    pwm.add_gpio(ch, gpio, 0);

    function set_width(newWidth) {
      pwm.set_width(gpio, newWidth);
    }
    function release() {
      pwm.release_gpio(gpio);
    }

    return {
      set_width: set_width,
      release: release
    };
  }
  
  function shutdown() {
    pwm.shutdown_channel(ch);
  }

  return {
    create_pwm: create_pwm,
    shutdown: shutdown
  };
}

function host_is_model_pi4() {
  return pwm.host_is_model_pi4();
}

// need keep sync with LogLevel in dma.h
var logLevel = {
  fatal: 0,
  error: 1,
  warning: 2,
  info: 3,
  debug: 4,
}

function set_log_level(logLevel) {
  pwm.set_log_level(logLevel);
}

var exp = {
    hello: function() { return pwm.hello(); },
    // setup: function(minResolutionUs) { pwm.setup(minResolutionUs); },
    create_dma_channel: create_dma_channel,
    host_is_model_pi4: host_is_model_pi4,
    logLevel: logLevel,
    set_log_level: set_log_level,
};

module.exports = exp;