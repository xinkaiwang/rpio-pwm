# rpio-pwm
* The most simple way to use rpio-pwm is just `npm install rpio-pwm -save`. I tested it on Pi4 + nodejs 18.12.1
* Or you can also use perf-gpio instead (`npm install perf-gpio -save`). perd-gpio provided a simpler higher-level API and make it easy to use.

# build from source
you need to install nan and gyp if you don't already have them.
```
node-gyp configure
node-gyp build
```
On build is suceessful you will see `build/Release/rpiopwm.node` got created.

# publish to npm
```
<update version number first>
npm publish
```
