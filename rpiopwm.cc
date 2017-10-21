#include <nan.h>
#include "pwm.h"

namespace {

void Method(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  info.GetReturnValue().Set(Nan::New("world").ToLocalChecked());
}

// pwm.setup(1); // 1 = resolution 1us
void Setup(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 1) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 1");
    return;
  }

  if (!info[0]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  double arg0 = info[0]->NumberValue();
  int incrementInUs = (int)arg0;

  setup(incrementInUs, DELAY_VIA_PWM);
}

// pwm.init_channel(14, 3000); // 14=DMA channel 14;  3000=full cycle time is 3000us
void InitChannel(const Nan::FunctionCallbackInfo<v8::Value>& info) {
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
  int dma_channel = (int)arg0;
  int subcycle_time_us = (int)arg1;

  init_channel(dma_channel, subcycle_time_us);
}

// pwm.add_channel_pulse(14, 17, 0, 50); // DMA channel 14; GPIO 17; start at 0us, width 50us
void addChannelPulse(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 4) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 4");
    return;
  }

  if (!info[0]->IsNumber() || !info[1]->IsNumber() || !info[2]->IsNumber() || !info[3]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  double arg0 = info[0]->NumberValue();
  int dma_channel = (int)arg0; // 14 = DMA channel 14
  double arg1 = info[1]->NumberValue();
  int gpio_port = (int)arg1;  // 17 = GPIO 17
  double arg2 = info[2]->NumberValue();
  int width_start = (int)arg2; // 100 = start from 100th us (assume resolution is 1us)
  double arg3 = info[3]->NumberValue();
  int width = (int)arg3; // 50 = last for 50 us (assume resolution is 1us)

  add_channel_pulse(dma_channel, gpio_port, width_start, width);
}

// pwm.clear_channel(14); // DMA channel 14
void ClearChannel(const Nan::FunctionCallbackInfo<v8::Value>& info) {
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

  clear_channel(dma_channel);
}

void ClearChannelGpio(const Nan::FunctionCallbackInfo<v8::Value>& info) {
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
  int dma_channel = (int)arg0;
  int gpio_port = (int)arg1;

  clear_channel_gpio(dma_channel, gpio_port);
}

void Shutdown(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  pwm_shutdown();
}

} // namespace

void Init(v8::Local<v8::Object> exports) {
  exports->Set(Nan::New("hello").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(Method)->GetFunction());

  exports->Set(Nan::New("setup").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(Setup)->GetFunction());

  exports->Set(Nan::New("init_channel").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(InitChannel)->GetFunction());

  exports->Set(Nan::New("add_channel_pulse").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(addChannelPulse)->GetFunction());

  exports->Set(Nan::New("clear_channel").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(ClearChannel)->GetFunction());

  exports->Set(Nan::New("clear_channel_gpio").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(ClearChannelGpio)->GetFunction());

  exports->Set(Nan::New("cleanup").ToLocalChecked(),
               Nan::New<v8::FunctionTemplate>(Shutdown)->GetFunction());
}

NODE_MODULE(rpiopwm, Init)