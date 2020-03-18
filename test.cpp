#include <stdio.h>
#include <unistd.h>
#include "dma.h"

using namespace wpp;

int main() {
    printf("hello Step 1\n");
    // TestDma();
    DmaChannelConfig config{};
    config.chNum = 14;
    auto ch = DmaChannel::CreateInstance(config);
    // printf("hello Step 2\n");
    DmaPwmPinConfig pinConfig{};
    auto pin = ch->CreatePin(pinConfig);
    usleep(1*1000000);
    pin->SetByPercentage(100.);
    usleep(1*1000000);
    pin->SetByPercentage(50.);

    usleep(5*1000000);
    printf("hello world\n");
    ch->DeactivateChannel();
}
