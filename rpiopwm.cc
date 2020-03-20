#include <nan.h>
#include "pwm.h"

namespace {

void Method(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  info.GetReturnValue().Set(Nan::New("world").ToLocalChecked());
}

// pwm.setup(1); // 1 = resolution 1us
// void Setup(const Nan::FunctionCallbackInfo<v8::Value>& info) {
//   if (info.Length() < 1) {
//     Nan::ThrowTypeError("Wrong number of arguments, expected 1");
//     return;
//   }

//   if (!info[0]->IsNumber()) {
//     Nan::ThrowTypeError("Wrong arguments");
//     return;
//   }

//   double arg0 = info[0]->NumberValue();
//   int incrementInUs = (int)arg0;

//   setup(incrementInUs, DELAY_VIA_PWM);
// }

// pwm.init_channel(14, 3000); // 14=DMA channel 14;  3000=full cycle time is 3000us
void PwmChannelInit(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 5) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 5");
    return;
  }

  if (!info[0]->IsNumber() || !info[1]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  double arg0 = info[0]->NumberValue();
  double arg1 = info[1]->NumberValue();
  double arg2 = info[2]->NumberValue();
  double arg3 = info[3]->NumberValue();
  double arg4 = info[4]->NumberValue();
  int dma_channel = (int)arg0;
  int cycle_time_us = (int)arg1;
  int step_time_us = (int)arg2;
  int delay_hw = (int)arg3;
  int invert = (int)arg4;

  pwm_channel_init(dma_channel, cycle_time_us, step_time_us, delay_hw, invert);
}

// pwm.clear_channel(14); // DMA channel 14
void PwmChannelShutdown(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 1) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 1");
    return;
  }

  if (!info[0]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  double arg0 = info[0]->NumberValue();
  int dma_channel = (int)arg0;

  pwm_channel_shutdown(dma_channel);
}

// pwm.add_channel_pulse(14, 17, 0, 50); // DMA channel 14; GPIO 17; start at 0us, width 50us
void PwmGpioAdd(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 3) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 3");
    return;
  }

  if (!info[0]->IsNumber() || !info[1]->IsNumber() || !info[2]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  double arg0 = info[0]->NumberValue();
  int dma_channel = (int)arg0; // 14 = DMA channel 14
  double arg1 = info[1]->NumberValue();
  int gpio_port = (int)arg1;  // 17 = GPIO 17
  double arg2 = info[2]->NumberValue();
  int width = (int)arg2; // 100 = 1000 us (assume resolution is 10us)

  pwm_gpio_add(dma_channel, gpio_port, width);
}


void PwmGpioSetWidth(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 2) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 2");
    return;
  }

  if (!info[0]->IsNumber() || !info[1]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  double arg0 = info[0]->NumberValue();
  double arg1 = info[1]->NumberValue();
  int gpio_port = (int)arg0;
  int width = (int)arg1;

  pwm_gpio_set_width(gpio_port, width);
}

void PwmGpioRelease(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 1) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 1");
    return;
  }

  if (!info[0]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  double arg0 = info[0]->NumberValue();
  int gpio_port = (int)arg0;

  pwm_gpio_release(gpio_port);
}

// void Shutdown(const Nan::FunctionCallbackInfo<v8::Value>& info) {
//   pwm_shutdown();
// }

} // namespace

void Init(v8::Local<v8::Object> exports) {
  exports->Set(Nan::New("hello").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(Method)->GetFunction());

  exports->Set(Nan::New("init_channel").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(PwmChannelInit)->GetFunction());

  exports->Set(Nan::New("shutdown_channel").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(PwmChannelShutdown)->GetFunction());

  exports->Set(Nan::New("add_gpio").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(PwmGpioAdd)->GetFunction());

  exports->Set(Nan::New("set_width").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(PwmGpioSetWidth)->GetFunction());

  exports->Set(Nan::New("release_gpio").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(PwmGpioRelease)->GetFunction());

  // exports->Set(Nan::New("cleanup").ToLocalChecked(),
  //              Nan::New<v8::FunctionTemplate>(Shutdown)->GetFunction());
}

NODE_MODULE(rpiopwm, Init)