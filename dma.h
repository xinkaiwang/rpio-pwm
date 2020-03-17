#include <memory>
#include <vector>

namespace wpp {

const int maxServoCount = 12; // max to 32

enum class DelayHardware { DELAY_VIA_PWM = 0, DELAY_VIA_PCM = 1 };

class DmaChannel;
class PwmPin;

typedef struct {
  uint32_t info, src, dst, length, stride, next, pad[2];
} dma_cb_t;

//************************** DmaHardware ****************************
struct DmaHardware {
  DelayHardware delay_hw{DelayHardware::DELAY_VIA_PWM};

  // struct timeval *servo_kill_time; // for idle_timeout feature.

  uint32_t plldfreq_mhz;
  // int dma_chan;
  // int idle_timeout;
  // int servo_min_ticks;
  // int servo_max_ticks;

  int board_model;
  int gpio_cfg;

  // init by get_model_and_revision() at init time (detect hardware)
  uint32_t periph_phys_base;
  uint32_t periph_virt_base;
  uint32_t dram_phys_base;
  uint32_t mem_flag;

  std::vector<std::weak_ptr<DmaChannel>> channels{};
};

//************************** DmaChannel ****************************
struct DmaChannelConfig {
  int chNum = 14; // default 14 (for pi2/3/zero), suggest use 7 for pi4
  int cycleTimeUs = 20000; // 20ms cycle
  int stepTimeUs = 10; // 10us each step
  bool invert = false; // default HIGH active
};

struct DmaPwmPinConfig {
  int gpioPinNum = 21;
  int widthInSteps = 20;
  bool restoreOnExit = true; // restore gpio mode when exit
};

class DmaChannel : public std::enable_shared_from_this<DmaChannel> {
public:
  // for Pi2/3 ch can use 14 or 13 etc.
  // for Pi4 suggest use 7.
  static std::shared_ptr<DmaChannel>
  CreateInstance(const DmaChannelConfig &config);

public:
  DmaChannel(DmaHardware &hw, const DmaChannelConfig &config);

  DmaChannel(DmaChannel const&) = delete;
  DmaChannel& operator = (DmaChannel const&) = delete;

public:
  // gpioPinNum (for example gpio_21 = P1_40 = wiringPi_29)
  std::shared_ptr<PwmPin> CreatePin(const DmaPwmPinConfig &pinConfig);

public:
  void Init();

  void Uninit();

public:
  DmaHardware &hw;
  const int chNum;
  int seqCount{0};

  // cycle_time_us is the pulse cycle time per servo, in microseconds.
  // Typically it should be 20ms, or 20000us.

  // step_time_us is the pulse width increment granularity, again in
  // microseconds. Setting step_time_us too low will likely cause problems as
  // the DMA controller will use too much memory bandwidth.  10us is a good
  // value, though you might be ok setting it as low as 2us.

  const int cycle_time_us;
  const int step_time_us;
  bool invert{false};
  // bool restore_gpio_modes{true};

  volatile uint32_t *pwm_reg{};
  volatile uint32_t *pcm_reg{};
  volatile uint32_t *clk_reg{};
  volatile uint32_t *dma_reg{};
  volatile uint32_t *gpio_reg{};

  int num_samples;
  int num_cbs;
  int num_pages;
  uint32_t *turnoff_mask;
  uint32_t *turnon_mask;
  dma_cb_t *cb_base;

public:
  std::vector<std::shared_ptr<PwmPin>> pins = 
    std::vector<std::shared_ptr<PwmPin>>(maxServoCount);
  std::vector<int> servostart = std::vector<int>(maxServoCount);
};

//************************** PwmPin ****************************
class PwmPin : public std::enable_shared_from_this<PwmPin> {
public:
  PwmPin(std::shared_ptr<DmaChannel> dmaChannel, const DmaPwmPinConfig &pinConfig);

  PwmPin(PwmPin const&) = delete;
  PwmPin& operator = (PwmPin const&) = delete;

public:
  void SetByPercentage(const float pct);

  void SetByActiveTimeUs(const int timeInUs);

  void Deactivate();

public:
  void Init(int slotIndex);
  void Uninit();

public:
  std::shared_ptr<DmaChannel> ch;
  const int gpioPinNum;
  uint32_t
      gpiomode; // when we exit, we can restore privious mode for this pin.

public:
  int slotIndex{-1}; // index in array ch.pins (start with 0)
  int servowidth{0};
  bool restoreOnExit{true};
};

void TestDma();

} // namespace wpp
