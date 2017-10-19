
var pwm = require('./build/Release/pwm.node');

function init() {
    return {
        hello: function() { return pwm.hello(); },
    };
}

module.exports = init();