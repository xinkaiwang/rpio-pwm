
var pwm = require('./build/Release/rpiopwm.node');

function init() {
    return {
        hello: function() { return pwm.hello(); },
    };
}

module.exports = init();