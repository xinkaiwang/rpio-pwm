/*
 * This file is part of RPIO-PWM.
 *
 * Copyright
 *
 *     Copyright (C) 2020 Xinkai Wang <xinkaiwang1017@gmail.com>
 *
 * License
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU Lesser General Public License as published
 *     by the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU Lesser General Public License for more details at
 *     <http://www.gnu.org/licenses/lgpl-3.0-standalone.html>
 *
 * Documentation
 *
 *     https://github.com/xinkaiwang/rpio-pwm
 *
 * dma.c, based on the excellent servod.c by Richard Hirst, provides flexible
 * PWM via DMA for the Raspberry Pi, supporting a resolution of up to 1us,
 * all 15 DMA channels, multiple GPIOs per channel, timing by PWM (default)
 * or PCM, a Python wrapper, and more.
 *
 * Feedback is much appreciated.
*/

#include "dma.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <bcm_host.h>
#include <iostream>

#include "mailbox.h"

#define DMY 255 // Used to represent an invalid P1 pin, or unmapped servo

#define NUM_P1PINS 40
#define NUM_P5PINS 8

#define MAX_MEMORY_USAGE                                                       \
  (16 * 1024 * 1024) /* Somewhat arbitrary limit of 16MB */

#define DEFAULT_CYCLE_TIME_US 20000
#define DEFAULT_STEP_TIME_US 10
#define DEFAULT_SERVO_MIN_US 500
#define DEFAULT_SERVO_MAX_US 2500

#define DEVFILE "/dev/servoblaster"
#define CFGFILE "/dev/servoblaster-cfg"

#define PAGE_SIZE 4096
#define PAGE_SHIFT 12

#define DMA_CHAN_SIZE 0x100
#define DMA_CHAN_MIN 0
#define DMA_CHAN_MAX 14
#define DMA_CHAN_DEFAULT 14
#define DMA_CHAN_PI4 7

#define DMA_BASE_OFFSET 0x00007000
#define DMA_LEN DMA_CHAN_SIZE *(DMA_CHAN_MAX + 1)
#define PWM_BASE_OFFSET 0x0020C000
#define PWM_LEN 0x28
#define CLK_BASE_OFFSET 0x00101000
#define CLK_LEN 0xA8
#define GPIO_BASE_OFFSET 0x00200000
#define GPIO_LEN 0x100
#define PCM_BASE_OFFSET 0x00203000
#define PCM_LEN 0x24

#define DMA_VIRT_BASE(hw) ((hw).periph_virt_base + DMA_BASE_OFFSET)
#define PWM_VIRT_BASE(hw) ((hw).periph_virt_base + PWM_BASE_OFFSET)
#define CLK_VIRT_BASE(hw) ((hw).periph_virt_base + CLK_BASE_OFFSET)
#define GPIO_VIRT_BASE(hw) ((hw).periph_virt_base + GPIO_BASE_OFFSET)
#define PCM_VIRT_BASE(hw) ((hw).periph_virt_base + PCM_BASE_OFFSET)

#define PWM_PHYS_BASE(hw) ((hw).periph_phys_base + PWM_BASE_OFFSET)
#define PCM_PHYS_BASE(hw) ((hw).periph_phys_base + PCM_BASE_OFFSET)
#define GPIO_PHYS_BASE(hw) ((hw).periph_phys_base + GPIO_BASE_OFFSET)

#define DMA_NO_WIDE_BURSTS (1 << 26)
#define DMA_WAIT_RESP (1 << 3)
#define DMA_D_DREQ (1 << 6)
#define DMA_PER_MAP(x) ((x) << 16)
#define DMA_END (1 << 1)
#define DMA_RESET (1U << 31)
#define DMA_INT (1 << 2)

#define DMA_CS (0x00 / 4)
#define DMA_CONBLK_AD (0x04 / 4)
#define DMA_SOURCE_AD (0x0c / 4)
#define DMA_DEBUG (0x20 / 4)

#define GPIO_FSEL0 (0x00 / 4)
#define GPIO_SET0 (0x1c / 4)
#define GPIO_CLR0 (0x28 / 4)
#define GPIO_LEV0 (0x34 / 4)
#define GPIO_PULLEN (0x94 / 4)
#define GPIO_PULLCLK (0x98 / 4)

#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1

#define PWM_CTL (0x00 / 4)
#define PWM_DMAC (0x08 / 4)
#define PWM_RNG1 (0x10 / 4)
#define PWM_FIFO (0x18 / 4)

#define PWMCLK_CNTL 40
#define PWMCLK_DIV 41

#define PWMCTL_MODE1 (1 << 1)
#define PWMCTL_PWEN1 (1 << 0)
#define PWMCTL_CLRF (1 << 6)
#define PWMCTL_USEF1 (1 << 5)

#define PWMDMAC_ENAB (1U << 31)
#define PWMDMAC_THRSHLD ((15 << 8) | (15 << 0))

#define PCM_CS_A (0x00 / 4)
#define PCM_FIFO_A (0x04 / 4)
#define PCM_MODE_A (0x08 / 4)
#define PCM_RXC_A (0x0c / 4)
#define PCM_TXC_A (0x10 / 4)
#define PCM_DREQ_A (0x14 / 4)
#define PCM_INTEN_A (0x18 / 4)
#define PCM_INT_STC_A (0x1c / 4)
#define PCM_GRAY (0x20 / 4)

#define PCMCLK_CNTL 38
#define PCMCLK_DIV 39

#define PLLDFREQ_MHZ_DEFAULT 500
#define PLLDFREQ_MHZ_PI4 750

// #define DELAY_VIA_PWM		0
// #define DELAY_VIA_PCM		1

#define ROUNDUP(val, blksz) (((val) + ((blksz)-1)) & ~(blksz - 1))

#define BUS_TO_PHYS(x) ((x) & ~0xC0000000)

using namespace wpp;

namespace {

DmaHardware hardware{};

// L362
static struct {
  int handle;         /* From mbox_open() */
  uint32_t size;      /* Required size */
  unsigned mem_ref;   /* From mem_alloc() */
  unsigned bus_addr;  /* From mem_lock() */
  uint8_t *virt_addr; /* From mapmem() */
} mbox;

void set_servo(DmaChannel &ch, int servo, int width);

// L375
static void udelay(int us) {
  struct timespec ts = {0, us * 1000};

  nanosleep(&ts, NULL);
}

// L384
// void terminateChannel(DmaChannel &ch) {
//   if (ch.dma_reg && mbox.virt_addr) {
//   	for (int i = 0; i < maxServoCount; i++) {
//   		if (ch.pins[i]) {
//   			set_servo(ch, i, 0);
//       }
//   	}
//   	udelay(ch.cycle_time_us);
//   	ch.dma_reg[DMA_CS] = DMA_RESET;
//   	udelay(10);
//   }
//   // if (restore_gpio_modes) {
//   // 	for (i = 0; i < MAX_SERVOS; i++) {
//   // 		if (servo2gpio[i] != DMY)
//   // 			gpio_set_mode(servo2gpio[i], gpiomode[i]);
//   // 	}
//   // }
// }

void terminate(int dummy) {
  printf("terminate() %d\n", dummy);
  for (auto &it : hardware.channels) {
    if (auto locked = it.lock()) {
      locked->DeactivateChannel();
    }
  }

  if (mbox.virt_addr != NULL) {
  	unmapmem(mbox.virt_addr, mbox.size);
  	mem_unlock(mbox.handle, mbox.mem_ref);
  	mem_free(mbox.handle, mbox.mem_ref);
  	if (mbox.handle >= 0)
  		mbox_close(mbox.handle);
  }

  exit(1);
}

// L416
static void fatal(const char *fmt, ...) {
  printf("fatal() %s\n", fmt);
  va_list ap;

  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  terminate(0);
}

// L480
static uint32_t gpio_get_mode(DmaChannel& ch, uint32_t gpio)
{
	uint32_t fsel = ch.gpio_reg[GPIO_FSEL0 + gpio/10];

	return (fsel >> ((gpio % 10) * 3)) & 7;
}

// L488
static void
gpio_set_mode(DmaChannel &ch, uint32_t gpio, uint32_t mode)
{
	uint32_t fsel = ch.gpio_reg[GPIO_FSEL0 + gpio/10];

	fsel &= ~(7 << ((gpio % 10) * 3));
	fsel |= mode << ((gpio % 10) * 3);
	ch.gpio_reg[GPIO_FSEL0 + gpio/10] = fsel;
}

// L498
static void
gpio_set(DmaChannel &ch, int gpio, int level)
{
	if (level)
		ch.gpio_reg[GPIO_SET0] = 1 << gpio;
	else
		ch.gpio_reg[GPIO_CLR0] = 1 << gpio;
}

// L506
static uint32_t
mem_virt_to_phys(void *virt)
{
	uint32_t offset = (uint8_t *)virt - mbox.virt_addr;

	return mbox.bus_addr + offset;
}

// L515
static void *map_peripheral(uint32_t base, uint32_t len) {
  int fd = open("/dev/mem", O_RDWR | O_SYNC);
  void *vaddr;

  if (fd < 0)
    fatal("servod: Failed to open /dev/mem: %m\n");
  vaddr = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, base);
  if (vaddr == MAP_FAILED)
    fatal("servod: Failed to map peripheral at 0x%08x: %m\n", base);
  close(fd);

  return vaddr;
}

/* Carefully add or remove bits from the turnoff_mask such that regardless
 * of where the DMA controller is in its cycle, and whether we are increasing
 * or decreasing the pulse width, the generated pulse will only ever be the
 * old width or the new width.  If we don't take such care then there could be
 * a cycle with some pulse width between the two requested ones.  That doesn't
 * really matter for servos, but when driving LEDs some odd intensity for one
 * cycle can be noticeable.  It may be that the servo output has been turned
 * off via the inactivity timer, which is handled by always setting the turnon
 * mask appropriately at the end of this function.
 */
// L556
void set_servo(DmaChannel &ch, int servo, int newWidth)
{
	volatile uint32_t *dp;
	int i;
	uint32_t mask = 1 << ch.pins[servo]->gpioPinNum;

  int oldWidth = ch.pins[servo]->servowidth;
  printf("set_servo oldWidth=%d new=%d\n", oldWidth, newWidth);

	if (newWidth > oldWidth) {
		dp = ch.turnoff_mask + ch.servostart[servo] + newWidth;
		if (dp >= ch.turnoff_mask + ch.num_samples)
			dp -= ch.num_samples;

		for (i = newWidth; i > oldWidth; i--) {
			dp--;
			if (dp < ch.turnoff_mask)
				dp = ch.turnoff_mask + ch.num_samples - 1;
			//printf("%5d, clearing at %p\n", dp - ctl->turnoff, dp);
			*dp &= ~mask;
		}
	} else if (newWidth < oldWidth) {
		dp = ch.turnoff_mask + ch.servostart[servo] + newWidth;
		if (dp >= ch.turnoff_mask + ch.num_samples)
			dp -= ch.num_samples;

		for (i = newWidth; i < oldWidth; i++) {
			//printf("%5d, setting at %p\n", dp - ctl->turnoff, dp);
			*dp++ |= mask;
			if (dp >= ch.turnoff_mask + ch.num_samples)
				dp = ch.turnoff_mask;
		}
	}
	ch.pins[servo]->servowidth = newWidth;
	if (newWidth == 0) {
		ch.turnon_mask[servo] = 0;
	} else {
		ch.turnon_mask[servo] = mask;
	}
}

// L596
// static void setup_sighandlers(void) {
//   int i;

//   // Catch all signals possible - it is vital we kill the DMA engine
//   // on process exit!
//   for (i = 0; i < 64; i++) {
//     struct sigaction sa;

//     memset(&sa, 0, sizeof(sa));
//     sa.sa_handler = terminate;
//     sigaction(i, &sa, NULL);
//   }
// }

// L612
void init_ctrl_data(DmaHardware &hw, DmaChannel& ch) {
  dma_cb_t *cbp = ch.cb_base;
  uint32_t phys_fifo_addr, cbinfo;
  uint32_t phys_gpclr0;
  uint32_t phys_gpset0;
  int curstart = 0;
  uint32_t maskall = 0;

  if (ch.invert) {
    phys_gpclr0 = GPIO_PHYS_BASE(hw) + 0x1c;
    phys_gpset0 = GPIO_PHYS_BASE(hw) + 0x28;
  } else {
    phys_gpclr0 = GPIO_PHYS_BASE(hw) + 0x28;
    phys_gpset0 = GPIO_PHYS_BASE(hw) + 0x1c;
  }

  if (ch.delay_hw == DelayHardware::DELAY_VIA_PWM) {
    phys_fifo_addr = PWM_PHYS_BASE(hw) + 0x18;
    cbinfo = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
  } else {
    phys_fifo_addr = PCM_PHYS_BASE(hw) + 0x04;
    cbinfo = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
  }

  memset(ch.turnon_mask, 0, maxServoCount * sizeof(*(ch.turnon_mask)));

  // for (servo = 0 ; servo < MAX_SERVOS; servo++) {
  // 	servowidth[servo] = 0;
  // 	if (servo2gpio[servo] != DMY) {
  // 		numservos++;
  // 		maskall |= 1 << servo2gpio[servo];
  // 	}
  // }

  for (int i = 0; i < ch.num_samples; i++)
  	ch.turnoff_mask[i] = maskall;

  for (uint32_t servo = 0; servo < ch.pins.size(); servo++) {
    ch.servostart[servo] = curstart;
    curstart += ch.num_samples / ch.pins.size();
  }

  for (int i = 0, servo = 0; i < ch.num_samples; i++) {
  	cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
  	cbp->src = mem_virt_to_phys(ch.turnoff_mask + i);
  	cbp->dst = phys_gpclr0;
  	cbp->length = 4;
  	cbp->stride = 0;
  	cbp->next = mem_virt_to_phys(cbp + 1);
  	cbp++;
  	if (servo < maxServoCount && i == ch.servostart[servo]) {
  		cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
  		cbp->src = mem_virt_to_phys(ch.turnon_mask + servo);
  		cbp->dst = phys_gpset0;
  		cbp->length = 4;
  		cbp->stride = 0;
  		cbp->next = mem_virt_to_phys(cbp + 1);
  		cbp++;
  		servo++;
  	}
  	// Delay
  	cbp->info = cbinfo;
  	cbp->src = mem_virt_to_phys(ch.turnoff_mask);	// Any data will do
  	cbp->dst = phys_fifo_addr;
  	cbp->length = 4;
  	cbp->stride = 0;
  	cbp->next = mem_virt_to_phys(cbp + 1);
  	cbp++;
  }
  cbp--;
  cbp->next = mem_virt_to_phys(ch.cb_base);
}

// L695
void init_hardware(DmaHardware &hw, DmaChannel &ch)
{
	if (ch.delay_hw == DelayHardware::DELAY_VIA_PWM) {
		// Initialise PWM
		ch.pwm_reg[PWM_CTL] = 0;
		udelay(10);
		ch.clk_reg[PWMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz or 750MHz on Pi4)
		udelay(100);
		ch.clk_reg[PWMCLK_DIV] = 0x5A000000 | (hw.plldfreq_mhz<<12);	// set pwm div to give 1MHz
		udelay(100);
		ch.clk_reg[PWMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		ch.pwm_reg[PWM_RNG1] = ch.step_time_us;
		udelay(10);
		ch.pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
		udelay(10);
		ch.pwm_reg[PWM_CTL] = PWMCTL_CLRF;
		udelay(10);
		ch.pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
		udelay(10);
	} else {
		// Initialise PCM
		ch.pcm_reg[PCM_CS_A] = 1;				// Disable Rx+Tx, Enable PCM block
		udelay(100);
		ch.clk_reg[PCMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz or 750MHz on Pi4)
		udelay(100);
		ch.clk_reg[PCMCLK_DIV] = 0x5A000000 | (hw.plldfreq_mhz<<12);	// Set pcm div to give 1MHz
		udelay(100);
		ch.clk_reg[PCMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		ch.pcm_reg[PCM_TXC_A] = 0U<<31 | 1<<30 | 0<<20 | 0<<16; // 1 channel, 8 bits
		udelay(100);
		ch.pcm_reg[PCM_MODE_A] = (ch.step_time_us - 1) << 10;
		udelay(100);
		ch.pcm_reg[PCM_CS_A] |= 1<<4 | 1<<3;		// Clear FIFOs
		udelay(100);
		ch.pcm_reg[PCM_DREQ_A] = 64<<24 | 64<<8;		// DMA Req when one slot is free?
		udelay(100);
		ch.pcm_reg[PCM_CS_A] |= 1<<9;			// Enable DMA
		udelay(100);
	}

	// Initialise the DMA
	ch.dma_reg[DMA_CS] = DMA_RESET;
	udelay(10);
	ch.dma_reg[DMA_CS] = DMA_INT | DMA_END;
	ch.dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ch.cb_base);
	ch.dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	ch.dma_reg[DMA_CS] = 0x10880001;	// go, mid priority, wait for outstanding writes

	if (ch.delay_hw == DelayHardware::DELAY_VIA_PCM) {
		ch.pcm_reg[PCM_CS_A] |= 1<<2;			// Enable Tx
	}
}

/* Determining the board revision is a lot more complicated than it should be
 * (see comments in wiringPi for details).  We will just look at the last two
 * digits of the Revision string and treat '00' and '01' as errors, '02' and
 * '03' as rev 1, and any other hex value as rev 2.  'Pi1 and Pi2 are
 * differentiated by the Hardware being BCM2708 or BCM2709.
 *
 * NOTE: These days we should just use bcm_host_get_model_type().
 */
// L945
void get_model_and_revision(DmaHardware &hw) {
  char buf[128], revstr[128], modelstr[128];
  char *ptr, *end, *res;
  int board_revision;
  FILE *fp;

  revstr[0] = modelstr[0] = '\0';

  fp = fopen("/proc/cpuinfo", "r");

  if (!fp)
    fatal("Unable to open /proc/cpuinfo: %m\n");

  while ((res = fgets(buf, 128, fp))) {
    if (!strncasecmp("hardware", buf, 8))
      memcpy(modelstr, buf, 128);
    else if (!strncasecmp(buf, "revision", 8))
      memcpy(revstr, buf, 128);
  }
  fclose(fp);

  if (modelstr[0] == '\0')
    fatal("servod: No 'Hardware' record in /proc/cpuinfo\n");
  if (revstr[0] == '\0')
    fatal("servod: No 'Revision' record in /proc/cpuinfo\n");

  if (strstr(modelstr, "BCM2708"))
    hw.board_model = 1;
  else if (strstr(modelstr, "BCM2709") || strstr(modelstr, "BCM2835"))
    hw.board_model = 2;
  else
    fatal("servod: Cannot parse the hardware name string\n");

  /* Revisions documented at http://elinux.org/RPi_HardwareHistory */
  ptr = revstr + strlen(revstr) - 3;
  board_revision = strtol(ptr, &end, 16);
  if (end != ptr + 2)
    fatal("servod: Failed to parse Revision string\n");
  if (board_revision < 1)
    fatal("servod: Invalid board Revision\n");
  else if (board_revision < 4)
    hw.gpio_cfg = 1;
  else if (board_revision < 16)
    hw.gpio_cfg = 2;
  else
    hw.gpio_cfg = 3;

  if (bcm_host_is_model_pi4()) {
    hw.plldfreq_mhz = PLLDFREQ_MHZ_PI4;
    // hw.dma_chan = DMA_CHAN_PI4;
  } else {
    hw.plldfreq_mhz = PLLDFREQ_MHZ_DEFAULT;
    // hw.dma_chan = DMA_CHAN_DEFAULT;
  }

  hw.periph_virt_base = bcm_host_get_peripheral_address();
  hw.dram_phys_base = bcm_host_get_sdram_address();
  hw.periph_phys_base = 0x7e000000;

  /*
   * See https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
   *
   * 1:  MEM_FLAG_DISCARDABLE = 1 << 0	// can be resized to 0 at any time. Use
   * for cached data
   *     MEM_FLAG_NORMAL = 0 << 2		// normal allocating alias. Don't use
   * from ARM 4:  MEM_FLAG_DIRECT = 1 << 2		// 0xC alias uncached 8:
   * MEM_FLAG_COHERENT = 2 << 2	// 0x8 alias. Non-allocating in L2 but coherent
   *     MEM_FLAG_L1_NONALLOCATING =	// Allocating in L2
   *       (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)
   * 16: MEM_FLAG_ZERO = 1 << 4		// initialise buffer to all zeros
   * 32: MEM_FLAG_NO_INIT = 1 << 5	// don't initialise (default is initialise
   * to all ones 64: MEM_FLAG_HINT_PERMALOCK = 1 << 6	// Likely to be locked
   * for long periods of time
   *
   */
  if (hw.board_model == 1) {
    hw.mem_flag = 0x0c; /* MEM_FLAG_DIRECT | MEM_FLAG_COHERENT */
  } else {
    hw.mem_flag = 0x04; /* MEM_FLAG_DIRECT */
  }
}

// L1184 main()
bool setup_hardware(DmaHardware &hw) {
  get_model_and_revision(hw);

  // init_idle_timers(hw);
  // setup_sighandlers();

  return true;
}

} // anonymous namespace

namespace wpp {

//************************** DmaChannel ****************************
// static
std::shared_ptr<DmaChannel> DmaChannel::CreateInstance(const DmaChannelConfig &config) {
  static bool inited = false;
  if (!inited) {
    inited = setup_hardware(hardware);
  }
  auto ch =
      std::make_shared<DmaChannel>(hardware, config);
  ch->Init();
  hardware.channels.push_back(ch);
  return ch;
}

DmaChannel::DmaChannel(DmaHardware &hw, const DmaChannelConfig &config)
    : hw{hw}, delay_hw{config.delay_hw}, chNum{config.chNum}, cycle_time_us{config.cycleTimeUs}, step_time_us{config.stepTimeUs}, invert{config.invert} {
  if (step_time_us < 2 || step_time_us > 1000) {
    throw std::runtime_error("Invalid step-size specified");
  }
  if (cycle_time_us < 1000 || cycle_time_us > 1000000) {
    throw std::runtime_error("Invalid cycle-time specified");
  }
	if (cycle_time_us % step_time_us) {
		throw std::runtime_error("cycle-time is not a multiple of step-size");
	}
	if (cycle_time_us / step_time_us < 100) {
		throw std::runtime_error("cycle-time must be at least 100 * step-size");
	}
}

void DmaChannel::Init() {
  num_samples = cycle_time_us / step_time_us;
  num_cbs = num_samples * 2 + maxServoCount;
  num_pages = (num_cbs * sizeof(dma_cb_t) + num_samples * 4 + maxServoCount * 4 +
               PAGE_SIZE - 1) >>
              PAGE_SHIFT;

  auto &ch = *this;
  ch.dma_reg =
      static_cast<uint32_t *>(map_peripheral(DMA_VIRT_BASE(hw), DMA_LEN));
  ch.dma_reg += ch.chNum * DMA_CHAN_SIZE / sizeof(uint32_t);
  ch.pwm_reg =
      static_cast<uint32_t *>(map_peripheral(PWM_VIRT_BASE(hw), PWM_LEN));
  ch.pcm_reg =
      static_cast<uint32_t *>(map_peripheral(PCM_VIRT_BASE(hw), PCM_LEN));
  ch.clk_reg =
      static_cast<uint32_t *>(map_peripheral(CLK_VIRT_BASE(hw), CLK_LEN));
  ch.gpio_reg =
      static_cast<uint32_t *>(map_peripheral(GPIO_VIRT_BASE(hw), GPIO_LEN));
  /* Use the mailbox interface to the VC to ask for physical memory */
  // Use the mailbox interface to request memory from the VideoCore
  // We specifiy (-1) for the handle rather than calling mbox_open()
  // so multiple users can share the resource.
  mbox.handle = -1; // mbox_open();
  mbox.size = ch.num_pages * 4096;
  mbox.mem_ref = mem_alloc(mbox.handle, mbox.size, 4096, hw.mem_flag);
  if (mbox.mem_ref == 0) {
    fatal("Failed to alloc memory from VideoCore\n");
  }
  mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref);
  if (mbox.bus_addr == ~0U) {
    mem_free(mbox.handle, mbox.size);
    fatal("Failed to lock memory\n");
  }
  mbox.virt_addr =
      static_cast<uint8_t *>(mapmem(BUS_TO_PHYS(mbox.bus_addr), mbox.size));

  ch.turnoff_mask = (uint32_t *)mbox.virt_addr;
  ch.turnon_mask =
      (uint32_t *)(mbox.virt_addr + ch.num_samples * sizeof(uint32_t));
  ch.cb_base =
      (dma_cb_t *)(mbox.virt_addr +
                   ROUNDUP(ch.num_samples + maxServoCount, 8) * sizeof(uint32_t));

  init_ctrl_data(hw, ch);
  init_hardware(hw, ch);
  ch.isActive = true;
}

void DmaChannel::DeactivateChannel() {
  auto &ch = *this;
  if (!ch.IsActive()) return;
  printf("DmaChannel::DeactivateChannel() ch=%d\n", ch.chNum);
  ch.isActive = false;

  for (int i = 0; i < maxServoCount; i++) {
      if (ch.pins[i]) {
          ch.pins[i]->DeactivatePin();
      }
  }
  udelay(ch.cycle_time_us);
  ch.dma_reg[DMA_CS] = DMA_RESET;
  udelay(10);
}

std::shared_ptr<PwmPin> DmaChannel::CreatePin(const DmaPwmPinConfig &pinConfig) {
  auto pin = std::make_shared<PwmPin>(this->shared_from_this(), pinConfig);
  
  // find next available slot in pins
  int servo=0;
  while(servo < maxServoCount && pins[servo] != nullptr) {
    servo++;
  }
  if (servo == maxServoCount) {
    throw std::runtime_error("run out of availabel slots");
  }
  pins[servo] = pin;
  pin->Init(servo);
  set_servo(*this, servo, pinConfig.widthInSteps);

  return pin;
}

//************************** PwmPin ****************************
PwmPin::PwmPin(std::shared_ptr<DmaChannel> dmaChannel, const DmaPwmPinConfig &pinConfig):
  ch{dmaChannel}, gpioPinNum{pinConfig.gpioPinNum}, restoreOnExit{pinConfig.restoreOnExit} {
    //
  }

void PwmPin::Init(int slotIndex) {
  this->slotIndex = slotIndex;
  this->gpiomode = gpio_get_mode(*ch, gpioPinNum);
  gpio_set(*ch, gpioPinNum, ch->invert ? 1 : 0);
  gpio_set_mode(*ch, gpioPinNum, GPIO_MODE_OUT);
}

void PwmPin::SetByWidth(const int width) {
  if (width < 0 || width > ch->num_samples) {
    throw std::runtime_error("width out of range");
  }
  printf("setByWidth %d\n", width);
  set_servo(*ch, slotIndex, width);
}

void PwmPin::SetByPercentage(const float pct) {
  if (slotIndex < 0) {
    throw std::runtime_error("already deactivated?");
  }
  if (pct < 0.0f || pct > 100.0f) {
    throw std::runtime_error("pct out of range");
  }
  int newWidth = static_cast<int>(pct * ch->num_samples / 100.0f);
  set_servo(*ch, slotIndex, newWidth);
}

void PwmPin::SetByActiveTimeUs(const int timeInUs) {
  if (slotIndex < 0) {
    throw std::runtime_error("already deactivated?");
  }
  if (timeInUs < 0 || timeInUs > ch->cycle_time_us) {
    throw std::runtime_error("timeInUs out of range");
  }
  int newWidth = timeInUs / ch->step_time_us;
  set_servo(*ch, slotIndex, newWidth);
}

void PwmPin::DeactivatePin() {
  if (slotIndex < 0) {
    // already deactivated?
    return;
  }
  // to avoid destruct before exit scope
  std::shared_ptr<PwmPin> self = shared_from_this();
  set_servo(*ch, slotIndex, 0 /*newWidth*/);
  ch->pins[slotIndex] = nullptr;
  slotIndex = -1;
}

//************************** TestDma ****************************
void TestDma() {
  // setup_hardware(hardware);
  // auto ch = DmaChannel::CreateInstance(14, 20000,10);
  std::cout << "TestDma";
}

} // namespace wpp