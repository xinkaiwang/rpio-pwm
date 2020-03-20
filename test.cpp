#include <stdio.h>
#include <unistd.h>
#include "dma.h"

using namespace wpp;

int main() {
    printf("init\n");
    DmaChannelConfig config{};
    config.delay_hw = DelayHardware::DELAY_VIA_PCM;
    config.chNum = 14;
    auto ch = DmaChannel::CreateInstance(config);
    // printf("hello Step 2\n");
    DmaPwmPinConfig pinConfig{};
    auto pin = ch->CreatePin(pinConfig);
    printf("started\n");
    // pin->SetByPercentage(100);
    // usleep(0.5*1000000);
    // pin->SetByPercentage(80);
    // usleep(0.5*1000000);
    // pin->SetByPercentage(60);
    // usleep(0.5*1000000);
    // pin->SetByPercentage(40);
    // usleep(0.5*1000000);
    // pin->SetByPercentage(20);
    // usleep(0.5*1000000);
    // pin->SetByPercentage(10);
    // usleep(0.5*1000000);
    pin->SetByPercentage(10);
    usleep(0.5*1000000);
    pin->SetByPercentage(0);
    usleep(0.5*1000000);
    pin->SetByPercentage(100);
    usleep(0.5*1000000);
    printf("Done\n");
    ch->DeactivateChannel();
}
