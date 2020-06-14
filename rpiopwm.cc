#include <nan.h>
#include "pwm.h"

namespace {

using v8::Context;
using v8::Isolate;
using v8::Local;

NAN_METHOD(Method)
// void Method(const Nan::FunctionCallbackInfo<v8::Value>& info) 
{
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

//  Isolate* isolate = info.GetIsolate();
//  Local<Context> context = isolate->GetCurrentContext();

//   double arg0 = info[0]->NumberValue(context).FromMaybe(0);
//   int incrementInUs = (int)arg0;

//   setup(incrementInUs, DELAY_VIA_PWM);
// }

// pwm.init_channel(14, 3000); // 14=DMA channel 14;  3000=full cycle time is 3000us
NAN_METHOD(PwmChannelInit) {
// void PwmChannelInit(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 5) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 5");
    return;
  }

  if (!info[0]->IsNumber() || !info[1]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  Isolate* isolate = info.GetIsolate();
  Local<Context> context = isolate->GetCurrentContext();

  double arg0 = info[0]->NumberValue(context).FromMaybe(0);
  double arg1 = info[1]->NumberValue(context).FromMaybe(0);
  double arg2 = info[2]->NumberValue(context).FromMaybe(0);
  double arg3 = info[3]->NumberValue(context).FromMaybe(0);
  double arg4 = info[4]->NumberValue(context).FromMaybe(0);
  int dma_channel = (int)arg0;
  int cycle_time_us = (int)arg1;
  int step_time_us = (int)arg2;
  int delay_hw = (int)arg3;
  int invert = (int)arg4;

  pwm_channel_init(dma_channel, cycle_time_us, step_time_us, delay_hw, invert);
}

// pwm.clear_channel(14); // DMA channel 14
NAN_METHOD(PwmChannelShutdown) {
//void PwmChannelShutdown(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 1) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 1");
    return;
  }

  if (!info[0]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  Isolate* isolate = info.GetIsolate();
  Local<Context> context = isolate->GetCurrentContext();

  double arg0 = info[0]->NumberValue(context).FromMaybe(0);
  int dma_channel = (int)arg0;

  pwm_channel_shutdown(dma_channel);
}

// pwm.add_channel_pulse(14, 17, 0, 50); // DMA channel 14; GPIO 17; start at 0us, width 50us
NAN_METHOD(PwmGpioAdd) {
//void PwmGpioAdd(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 3) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 3");
    return;
  }

  if (!info[0]->IsNumber() || !info[1]->IsNumber() || !info[2]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  Isolate* isolate = info.GetIsolate();
  Local<Context> context = isolate->GetCurrentContext();

  double arg0 = info[0]->NumberValue(context).FromMaybe(0);
  int dma_channel = (int)arg0; // 14 = DMA channel 14
  double arg1 = info[1]->NumberValue(context).FromMaybe(0);
  int gpio_port = (int)arg1;  // 17 = GPIO 17
  double arg2 = info[2]->NumberValue(context).FromMaybe(0);
  int width = (int)arg2; // 100 = 1000 us (assume resolution is 10us)

  pwm_gpio_add(dma_channel, gpio_port, width);
}

NAN_METHOD(PwmGpioSetWidth) {
// void PwmGpioSetWidth(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 2) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 2");
    return;
  }

  if (!info[0]->IsNumber() || !info[1]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  Isolate* isolate = info.GetIsolate();
  Local<Context> context = isolate->GetCurrentContext();

  double arg0 = info[0]->NumberValue(context).FromMaybe(0);
  double arg1 = info[1]->NumberValue(context).FromMaybe(0);
  int gpio_port = (int)arg0;
  int width = (int)arg1;

  pwm_gpio_set_width(gpio_port, width);
}

NAN_METHOD(PwmGpioRelease) {
// void PwmGpioRelease(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 1) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 1");
    return;
  }

  if (!info[0]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  Isolate* isolate = info.GetIsolate();
  Local<Context> context = isolate->GetCurrentContext();

  double arg0 = info[0]->NumberValue(context).FromMaybe(0);
  int gpio_port = (int)arg0;

  pwm_gpio_release(gpio_port);
}

NAN_METHOD(PwmHostIsPi4) {
// void PwmHostIsPi4(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  auto is_pi4 = pwm_host_is_model_pi4();
  info.GetReturnValue().Set(Nan::New(is_pi4));
}

NAN_METHOD(PwmSetLogLevel) {
// void PwmSetLogLevel(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.Length() < 1) {
    Nan::ThrowTypeError("Wrong number of arguments, expected 1");
    return;
  }

  if (!info[0]->IsNumber()) {
    Nan::ThrowTypeError("Wrong arguments");
    return;
  }

  Isolate* isolate = info.GetIsolate();
  Local<Context> context = isolate->GetCurrentContext();

  double arg0 = info[0]->NumberValue(context).FromMaybe(0);
  int logLevel = (int)arg0;
  pwm_set_log_level(logLevel);
}

} // namespace

using namespace v8;

NAN_MODULE_INIT(Init) {

  Nan::Set(target, Nan::New("hello").ToLocalChecked(),
               Nan::GetFunction(Nan::New<FunctionTemplate>(Method)).ToLocalChecked());

  Nan::Set(target, Nan::New("init_channel").ToLocalChecked(),
               Nan::GetFunction(Nan::New<FunctionTemplate>(PwmChannelInit)).ToLocalChecked());

  Nan::Set(target, Nan::New("shutdown_channel").ToLocalChecked(),
               Nan::GetFunction(Nan::New<FunctionTemplate>(PwmChannelShutdown)).ToLocalChecked());

  Nan::Set(target, Nan::New("add_gpio").ToLocalChecked(),
               Nan::GetFunction(Nan::New<FunctionTemplate>(PwmGpioAdd)).ToLocalChecked());

  Nan::Set(target, Nan::New("set_width").ToLocalChecked(),
               Nan::GetFunction(Nan::New<FunctionTemplate>(PwmGpioSetWidth)).ToLocalChecked());

  Nan::Set(target, Nan::New("release_gpio").ToLocalChecked(),
               Nan::GetFunction(Nan::New<FunctionTemplate>(PwmGpioRelease)).ToLocalChecked());

  Nan::Set(target, Nan::New("host_is_model_pi4").ToLocalChecked(),
               Nan::GetFunction(Nan::New<FunctionTemplate>(PwmHostIsPi4)).ToLocalChecked());

  Nan::Set(target, Nan::New("set_log_level").ToLocalChecked(),
               Nan::GetFunction(Nan::New<FunctionTemplate>(PwmSetLogLevel)).ToLocalChecked());
}

NODE_MODULE(rpiopwm, Init)