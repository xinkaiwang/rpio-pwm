const pwm = require('./build/Release/rpiopwm.node');

function create_dma_channel(ch /*int*/, cfg /*json obj*/) {
  cfg = cfg || {};
  const cycle_time_us = cfg.cycle_time_us || 20000; // by default 20ms
  const step_time_us = cfg.step_time_us || 10; // by default 10us
  const delay_hw = cfg.delay_hw || 0; // by default 0 (PWM)
  const invert = cfg.invert || 0; // by default 0 (don't invert)
  pwm.init_channel(ch, cycle_time_us, step_time_us, delay_hw, invert);

  function create_pwm(gpio) { // GPIO pin number
    pwm.add_gpio(ch, gpio, 0);

    return {
      set_width: (newWidth) => pwm.set_width(gpio, newWidth),
      release: () => pwm.release_gpio(gpio),
    };
  }

  return {
    create_pwm,
    shutdown: () => pwm.shutdown_channel(ch),  
  };
}

// need keep sync with LogLevel in dma.h
const logLevel = {
  fatal: 0,
  error: 1,
  warning: 2,
  info: 3,
  debug: 4,
}

function set_log_level(logLevel) {
  pwm.set_log_level(logLevel);
}

const exp = {
  // setup: function(minResolutionUs) { pwm.setup(minResolutionUs); },
  create_dma_channel,
  host_is_model_pi4: () => pwm.host_is_model_pi4(),
  logLevel,
  set_log_level,
};

module.exports = exp;