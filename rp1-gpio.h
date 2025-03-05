/*
 rp1-gpio.h - Raspberry Pi RP1 GPIO Driver
 
  Copyright (c) 2024 Sasapea's Lab. All right reserved.
 
  This program is free software: you can redistribute it and/or
  modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once
 
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
 
class RP1_GPIO
{
  public:
 
    typedef enum
    {
      CHIP_UNKNOWN = -1,
      CHIP_BCM2835,
      CHIP_BCM2711,
      CHIP_RP1,
    } ChipType;
 
    static
    ChipType getGpioChipType(uint32_t *lines = nullptr)
    {
      int fd = open("/dev/gpiochip0", 0);
      if (fd != -1)
      {
        struct gpiochip_info cinfo;
        if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &cinfo) != -1)
        {
          close(fd);
          if (strcmp("pinctrl-rp1", cinfo.label) == 0)
          {
            if (lines)
              *lines = cinfo.lines;
            return CHIP_RP1;
          }
          else if (strcmp("pinctrl-bcm2711", cinfo.label) == 0)
          {
            if (lines)
              *lines = cinfo.lines;
            return CHIP_BCM2711;
          }
          else if (strcmp("pinctrl-bcm2835", cinfo.label) == 0)
          {
            if (lines)
              *lines = cinfo.lines;
            return CHIP_BCM2835;
          }
        }
        close(fd);
      }
      return CHIP_UNKNOWN;
    }
 
    typedef enum
    {
      IRQ_NORMAL,
      IRQ_INVERT,
      IRQ_DRIVE_LOW,
      IRQ_DRIVE_HIGH,
    } PinInterrupt;
 
    typedef enum
    {
      DVDD_3V3,   // voltage to 3.3V (DVDD >= 2V5)
      DVDD_1V8,   // voltage to 1.8V (DVDD <= 1V8)
    } PinVoltage;
 
    typedef enum
    {
      DRIVE_2MA,   // Drive strength. 2mA
      DRIVE_4MA,   // Drive strength. 4mA
      DRIVE_8MA,   // Drive strength. 8mA
      DRIVE_12MA,  // Drive strength. 12mA
    } PinDrive;
 
    typedef enum
    {
      INPUT,
      OUTPUT,
      OPEN_DRAIN,
      OPEN_SOURCE,
      DISABLE,
    } PinMode;
 
    typedef enum
    {
      PULL_DISABLE,
      PULL_UP,
      PULL_DOWN,
    } PinPull;
 
    typedef enum
    {
      LOW,
      HIGH,
      TOGGLE,     // only digitalWrite()
    } PinStatus;
 
    typedef enum
    {
      PinFunc_A0, // SPI[0/1], PWM[0/1/2/3], GPCLK[0/1/2]
      PinFunc_A1, // DPI
      PinFunc_A2, // UART[1/2/3/4], MIPI[0/1], I2S[0]
      PinFunc_A3, // I2C[0/1/2/3], PWM[2/3], GPCLK[0/1]
      PinFunc_A4, // UART[0], AUDIO[0], I2S[1]
      PinFunc_A5, // SYS_RIO,
      PinFunc_A6, // PROC_RIO,
      PinFunc_A7, // PIO,
      PinFunc_A8, // SPI[2/3/4/5], GPCLK[1],
      PinFunc_RIO  = PinFunc_A5,
      PinFunc_NONE = 0x1F,
    } PinFunc;
 
    typedef enum
    {
      INMODE_PERI,
      INMODE_INVPERI,
      INMODE_LOW,
      INMODE_HIGH,
    } PinInput;
 
    typedef enum
    {
      OEMODE_PERI,
      OEMODE_INVPERI,
      OEMODE_DISABLE,
      OEMODE_ENABLE,
    } PinEnable;
 
    typedef enum
    {
      OUTMODE_PERI,
      OUTMODE_INVPERI,
      OUTMODE_LOW,
      OUTMODE_HIGH,
    } PinOutput;
 
    typedef uint32_t pin_size_t;
 
  private:
 
    static constexpr size_t PERIPHERALS_BASE = 0x1F00000000;
    static constexpr size_t PWM0_BASE        = PERIPHERALS_BASE + 0x98000;
    static constexpr size_t PWM1_BASE        = PERIPHERALS_BASE + 0x9C000;
    static constexpr size_t IO_BANK0_BASE    = PERIPHERALS_BASE + 0xD0000;
    static constexpr size_t SYS_RIO0_BASE    = PERIPHERALS_BASE + 0xE0000;
    static constexpr size_t PADS_BANK0_BASE  = PERIPHERALS_BASE + 0xF0000;
 
    typedef volatile uint32_t rw_reg_t;
    typedef const volatile uint32_t ro_reg_t;
 
    enum
    {
      ATOM_RW ,
      ATOM_XOR,
      ATOM_SET,
      ATOM_CLR,
    };
 
  public:
 
    //
    // IO_BANK0: GPIO_STATUS Register
    //
    typedef union
    {
      struct
      {
        ro_reg_t _rsv1_               : 8; // Reserved.
        ro_reg_t OUTFROMPERI          : 1; // [0x00] output signal from selected peripheral, before register overide is applied
        ro_reg_t OUTTOPAD             : 1; // [0x00] output signal to pad after register overide is applied
        ro_reg_t _rsv2_               : 2; // Reserved.
        ro_reg_t OEFROMPERI           : 1; // [0x00] output enable from selected peripheral, before register overide is applied
        ro_reg_t OETOPAD              : 1; // [0x00] output enable to pad after register overide is applied
        ro_reg_t _rsv3_               : 2; // Reserved.
        ro_reg_t INISDIRECT           : 1; // [0x00] input signal from pad, goes directly to the selected peripheral without filtering or override
        ro_reg_t INFROMPAD            : 1; // [0x00] input signal from pad, before filtering and override are applied
        ro_reg_t INFILTERED           : 1; // [0x00] input signal from pad, after filtering is applied but before override, not valid if inisdirect=1
        ro_reg_t INTOPERI             : 1; // [0x00] input signal to peripheral, after filtering and override are applied, not valid if inisdirect=1
        ro_reg_t EVENT_EDGE_LOW       : 1; // [0x00] Input pin has seen falling edge. Clear with ctrl_irqreset
        ro_reg_t EVENT_EDGE_HIGH      : 1; // [0x00] Input pin has seen rising edge. Clear with ctrl_irqreset
        ro_reg_t EVENT_LEVEL_LOW      : 1; // [0x00] Input pin is Low
        ro_reg_t EVENT_LEVEL_HIGH     : 1; // [0x00] Input pin is high
        ro_reg_t EVENT_F_EDGE_LOW     : 1; // [0x00] Input pin has seen a filtered falling edge. Clear with ctrl_irqreset
        ro_reg_t EVENT_F_EDGE_HIGH    : 1; // [0x00] Input pin has seen a filtered rising edge. Clear with ctrl_irqreset
        ro_reg_t EVENT_DB_LEVEL_LOW   : 1; // [0x00] Debounced input pin is low
        ro_reg_t EVENT_DB_LEVEL_HIGH  : 1; // [0x00] Debounced input pin is high
        ro_reg_t IRQCOMBINED          : 1; // [0x00] interrupt to processors, after masking
        ro_reg_t IRQTOPROC            : 1; // [0x00] interrupt to processors, after mask and override is applied
        ro_reg_t _rsv4_               : 2; // Reserved.
      } bit;
      ro_reg_t reg;
    } GPIO_STATUS;
 
    //
    // IO_BANK0: GPIO_CTRL Register
    //
    typedef union
    {
      struct
      {
        rw_reg_t FUNCSEL              : 5; // [0x1f] Function select. 31 == NULL. See GPIO function table for available functions.
        rw_reg_t F_M                  : 7; // [0x04] Filter/debounce time constant M
        rw_reg_t OUTOVER              : 2; // [0x00] 0x0=drive output from peripheral signal selected by funcsel
                                           //        0x1=drive output from inverse of peripheral signal selected by funcsel
                                           //        0x2=drive output low
                                           //        0x3=drive output high
        rw_reg_t OEOVER               : 2; // [0x00] 0x0=drive output enable from peripheral signal selected by funcsel
                                           //        0x1=drive output enable from inverse of peripheral signal selected by funcsel
                                           //        0x2=disable output
                                           //        0x3=enable output
        rw_reg_t INOVER               : 2; // [0x00] 0x0=don’t invert the peri input
                                           //        0x1=invert the peri input
                                           //        0x2=drive peri input low
                                           //        0x3=drive peri input high
        ro_reg_t _rsv1_               : 2; // Reserved.
        rw_reg_t IRQMASK_EDGE_LOW     : 1; // [0x00] Masks the edge low interrupt into the interrupt output
        rw_reg_t IRQMASK_EDGE_HIGH    : 1; // [0x00] Masks the edge high interrupt into the interrupt output
        rw_reg_t IRQMASK_LEVEL_LOW    : 1; // [0x00] Masks the level low interrupt into the interrupt output
        rw_reg_t IRQMASK_LEVEL_HIGH   : 1; // [0x00] Masks the level high interrupt into the interrupt output
        rw_reg_t IRQMASK_F_EDGE_LOW   : 1; // [0x00] Masks the filtered edge low interrupt into the interrupt output
        rw_reg_t IRQMASK_F_EDGE_HIGH  : 1; // [0x00] Masks the filtered edge high interrupt into the interrupt output
        rw_reg_t IRQMASK_DB_LEVEL_LOW : 1; // [0x00] Masks the debounced level low interrupt into the interrupt output
        rw_reg_t IRQMASK_DB_LEVEL_HIGH: 1; // [0x00] Masks the debounced level high interrupt into the interrupt output
        rw_reg_t IRQRESET             : 1; // [0x00] 0x0=do nothing, 0x1=reset the interrupt edge detector
        ro_reg_t _rsv2_               : 1; // Reserved.
        rw_reg_t IRQOVER              : 2; // [0x00] 0x0=don’t invert the interrupt
                                           //        0x1=invert the interrupt
                                           //        0x2=drive interrupt low
                                           //        0x3=drive interrupt high
      } bit;
      rw_reg_t reg;
    } GPIO_CTRL;
 
  private:
 
    //
    // IO_BANK0: GPIO Register
    //
    typedef struct
    {
      GPIO_STATUS STATUS;
      GPIO_CTRL   CTRL;
    } gpio_reg_t;
 
    //
    // IO_BANK0: PROCx/PCIE Interrupt Register
    //
    typedef struct
    {
      rw_reg_t INTE;                       // Interrupt Enable
      rw_reg_t INTF;                       // Interrupt Force
      ro_reg_t INTS;                       // Interrupt status after masking & forcing
    } gpio_irq_t;
 
    //
    // IO_BANK0: Registers
    //
    typedef struct
    {
      union
      {
        struct
        {
          gpio_reg_t PINS[28];             // GPIO[0-27]
          gpio_reg_t _rsv_[4];             // Reserved.
          ro_reg_t   INTR;                 // Raw Interrupts
          gpio_irq_t PROC[2];              // Interrupt for PROC[0,1]
          gpio_irq_t PCIE;                 // Interrupt for PCIE
        } REG;
        uint8_t _rsv_[0x1000];
      } ATOM[4];
    } IO_BANK0;
 
    //
    // PADS_BANK0: GPIO0, GPIO1, …, GPIO26, GPIO27 Registers
    //
    typedef union
    {
      struct
      {
        rw_reg_t SLEWFAST:  1;             // [0x00] Slew rate control. 1 = Fast, 0 = Slow
        rw_reg_t SCHMITT :  1;             // [0x01] Enable schmitt trigger
        rw_reg_t PDE     :  1;             // [varies] Pull down enable
        rw_reg_t PUE     :  1;             // [varies] Pull up enable
        rw_reg_t DRIVE   :  2;             // [0x01] Drive strength. 0=2mA, 1=4mA, 2=8mA, 3=12mA
        rw_reg_t IE      :  1;             // [0x00] Input enable
        rw_reg_t OD      :  1;             // [0x01] Output disable. Has priority over output enable from peripherals
        ro_reg_t _rsv1_  : 24;             // Reserved.
      } bit;
      rw_reg_t reg;
    } PADS_CTRL;
 
    //
    // PADS_BANK0: Registers.
    //
    typedef struct
    {
      union
      {
        struct
        {
          //
          // VOLTAGE_SELECT Register
          //
          union
          {
            struct
            {
              rw_reg_t VOLTAGE_SELECT:  1; // [0x00] Voltage select. Per bank control
                                           //        0x0=Set voltage to 3.3V (DVDD >= 2V5)
                                           //        0x1=Set voltage to 1.8V (DVDD <= 1V8)
              ro_reg_t _rsv1_        : 31; // Reserved.
            } bit;
            rw_reg_t reg;
          } VOLT;
          //
          // GPIO0, GPIO1, …, GPIO26, GPIO27 Registers
          //
          PADS_CTRL PADS[28];
        } REG;
        uint8_t _rsv_[0x1000];
      } ATOM[4];
    } PADS_BANK0;
 
    //
    // SYS_RIO0: Registers.
    //
    typedef struct
    {
      union
      {
        struct
        {
          rw_reg_t OUT;                    // controls the GPIO output drive
          rw_reg_t OE;                     // controls the GPIO output drive enable
          ro_reg_t IN;                     // samples the GPIO inputs directly
          ro_reg_t SYNC_IN;                // samples the GPIO inputs, each synchronised with a 2-stage synchroniser to clk_sys
        } REG;
        uint8_t _rsv_[0x1000];
      } ATOM[4];
    } SYS_RIO0;
 
    //
    // PWM MODES
    //
    typedef enum
    {
      PWM_MODE_ZERO,
      PWM_MODE_TRAILING_EDGE,
      PWM_MODE_DOUBLE_EDGE,
      PWM_MODE_PDM,
      PWM_MODE_SERIALISER_MSB,
      PWM_MODE_PPM,
      PWM_MODE_LEADING_EDGE,
      PWM_MODE_SERIALISER_LSB,
    } PWM_MODE;
 
    //
    // PWM: Registers.
    //
    typedef struct
    {
      //
      // GLOBAL_CTRL Register. Global control bits
      //
      union
      {
        struct
        {
          rw_reg_t CHAN0_EN  :  1;         // [0x00] 1=Enable the respective PWM channel in the mode set by the CHAN0_CTRL registers, 0=Channel disabled
          rw_reg_t CHAN1_EN  :  1;         // [0x00] 1=Enable the respective PWM channel in the mode set by the CHAN1_CTRL registers, 0=Channel disabled
          rw_reg_t CHAN2_EN  :  1;         // [0x00] 1=Enable the respective PWM channel in the mode set by the CHAN2_CTRL registers, 0=Channel disabled
          rw_reg_t CHAN3_EN  :  1;         // [0x00] 1=Enable the respective PWM channel in the mode set by the CHAN3_CTRL registers, 0=Channel disabled
          ro_reg_t _rsv_     : 27;         // Reserved.
          rw_reg_t SET_UPDATE:  1;         // [0x00] To prevent mis-sampling of multi-bit bus signals in the PWM clock domain,
                                           //        this bit should be used to trigger a settings update.
                                           //        This ensures that all PWM channel settings update on the same PWM clock cycle.
                                           //        Write 1 to trigger a settings update to the block. Self clears to 0.
                                           //        This bit affects the chan*_en bits, chan*_phase, chan*_ctrl and common_range registers.
                                           //        Writes to the *_duty and *_range registers have an integral update strobe
                                           //        and writes take effect on the next counter overflow of the respective PWM channel.
        } bit;
        rw_reg_t reg;
      } GLOBAL_CTRL;
      //
      // FIFO_CTRL Register. FIFO thresholding and status
      //
      union
      {
        struct
        {
          ro_reg_t LEVEL     :  5;         // [0x00] Number of available words in the FIFO
          rw_reg_t FLUSH     :  1;         // [0x00] Assert to flush FIFO
          ro_reg_t FLUSH_DONE:  1;         // [0x00] FIFO flush completed in the PWM clock domain
          ro_reg_t _rsv1_    :  4;         // Reserved.
          rw_reg_t THRESHOLD :  5;         // [0x00] Threshold for the comparator. DREQ is asserted when level <= threshold.
          rw_reg_t DWELL_TIME:  5;         // [0x02] Delay in number of bus cycles before successive DREQs are generated.
                                           //        Used to account for system bus latency in write data arriving at the FIFO.
          ro_reg_t _rsv2_    : 10;         // Reserved.
          rw_reg_t DREQ_EN   :  1;         // [0x00] 1=Generate DMA request signals to the DMA controller
                                           //        0=Don’t generate request signals - the dreq_active interrupt is unaffected.
        } bit;
        rw_reg_t reg;
      } FIFO_CTRL;
      //
      // COMMON_RANGE Register.
      //
      rw_reg_t COMMON_RANGE;               // [0x00] Counter range register for channels that are set to use channel binding
      //
      // COMMON_DUTY Register.
      //
      rw_reg_t COMMON_DUTY;                // [0x00] Counter compare register for channels that are set to use channel binding and are not set to use the common FIFO
      //
      // DUTY_FIFO Register.
      //
      rw_reg_t DUTY_FIFO;                  // [0x00] 32-bit interface to a 128-bit backed duty cycle FIFO
                                           //        In round-robin fashion, 32-bit writes to this address are sequentially packed
                                           //        as 32*n-bit words that are pushed into the duty cycle FIFO. N varies as per the
                                           //        number of enabled channels set to use the FIFO. A distributor checks which channels
                                           //        are enabled and using the FIFO, and writes the 32-bit words accordingly
      //
      // CHANx Registers.
      //
      struct
      {
        //
        // CHANx_CTRL Register
        //
        union
        {
          struct
          {
            rw_reg_t MODE         :  3;    // [0x00] PWM generation mode
                                           //        0x0=Generates 0
                                           //        0x1=Trailing-edge mark-space PWM modulation
                                           //        0x2=Phase-correct mark-space PWM modulation
                                           //        0x3=Pulse-density encoded output
                                           //        0x4=MSB Serialiser output.
                                           //        0x5=Pulse position modulated output - a single high pulse is transmitted per cycle.
                                           //        0x6=Leading-edge mark-space PWM modulation
                                           //        0x7=LSB Serialiser output.
            rw_reg_t INVERT       :  1;    // [0x00] 1=Invert the output bit
            rw_reg_t BIND         :  1;    // [0x00] 1=Bind Channel 0 to the common_range and common_duty/duty_fifo registers
                                           //        0=Channel 0 uses chan0_range and chan0_duty
            rw_reg_t USEFIFO      :  1;    // [0x00] 1=Use the duty_fifo. Note: setting bind=0 and usefifo=1 will lead to unpredictable operation
                                           //        0=Use the duty cycle register common_duty/chan0_duty
            rw_reg_t SDM          :  1;    // [0x00] 1=Use sigma-delta noise shaping modulator. In conjunction with sdm_bitwidth,
                                           //          treat the duty cycle as a 16-bit signed truncation of the 32 bit duty cycle value and quantise
                                           //          to (sdm_bitwidth+1)-bits. The resulting quantisation noise is filtered using a 2nd-order loop.
                                           //        0=Bypass modulator
            rw_reg_t DITHER       :  1;    // [0x00] 1=When SDM mode is used, add a 1-bit LSB dither inside the noise shaping loop to suppress idle tones
                                           //        0=No dither applied
            rw_reg_t FIFO_POP_MASK:  1;    // [0x01] 0=Counter overflow events do not generate FIFO pop events
                                           //        1=Counter overflow events generate FIFO pop events
            ro_reg_t _rsv1_       :  3;    // Reserved.
            rw_reg_t SDM_BITWIDTH :  4;    // [0x00] Quantise the 16-bit input to a (sdm_bitwidth+1)-bit output. 0=1-bit output.
            rw_reg_t SDM_BIAS     : 16;    // [0x00] Unsigned offset to be added to the output PWM code generated by the sigma-delta modulator.
          } bit;
          rw_reg_t reg;
        } CTRL;
        //
        // CHANx_RANGE Register.
        //
        rw_reg_t RANGE;                    // [0x00] Channel 0 counter range
        //
        // CHANx_PHASE Register.
        //
        rw_reg_t PHASE;                    // [0x00] Channel 0 counter phase offset register
                                           //        This register preloads the internal counter such that phase offsets between
                                           //        channels can be introduced. Do not set higher than the respective range register.
        //
        // CHANx_DUTY Register.
        //
        rw_reg_t DUTY;                     // [0x00] Channel 0 counter compare register
      } CHAN[4];
      //
      // INTR Register. Raw Interrupts
      //
      union
      {
        struct
        {
          rw_reg_t FIFO_UNDERFLOW:  1;     // [0x00]
          rw_reg_t FIFO_OVERFLOW :  1;     // [0x00]
          ro_reg_t FIFO_EMPTY    :  1;     // [0x00]
          ro_reg_t FIFO_FULL     :  1;     // [0x00]
          ro_reg_t DREQ_ACTIVE   :  1;     // [0x00]
          rw_reg_t CHAN0_RELOAD  :  1;     // [0x00]
          rw_reg_t CHAN1_RELOAD  :  1;     // [0x00]
          rw_reg_t CHAN2_RELOAD  :  1;     // [0x00]
          rw_reg_t CHAN3_RELOAD  :  1;     // [0x00]
          ro_reg_t _rsv_         : 23;
        } bit;
        rw_reg_t reg;
      } INTR;
      //
      // INTE Register. Interrupt Enable
      //
      union
      {
        struct
        {
          rw_reg_t FIFO_UNDERFLOW:  1;     // [0x00]
          rw_reg_t FIFO_OVERFLOW :  1;     // [0x00]
          rw_reg_t FIFO_EMPTY    :  1;     // [0x00]
          rw_reg_t FIFO_FULL     :  1;     // [0x00]
          rw_reg_t DREQ_ACTIVE   :  1;     // [0x00]
          rw_reg_t CHAN0_RELOAD  :  1;     // [0x00]
          rw_reg_t CHAN1_RELOAD  :  1;     // [0x00]
          rw_reg_t CHAN2_RELOAD  :  1;     // [0x00]
          rw_reg_t CHAN3_RELOAD  :  1;     // [0x00]
          ro_reg_t _rsv_         : 23;
        } bit;
        rw_reg_t reg;
      } INTE;
      //
      // INTF Register. Interrupt Force
      //
      union
      {
        struct
        {
          rw_reg_t FIFO_UNDERFLOW:  1;     // [0x00]
          rw_reg_t FIFO_OVERFLOW :  1;     // [0x00]
          rw_reg_t FIFO_EMPTY    :  1;     // [0x00]
          rw_reg_t FIFO_FULL     :  1;     // [0x00]
          rw_reg_t DREQ_ACTIVE   :  1;     // [0x00]
          rw_reg_t CHAN0_RELOAD  :  1;     // [0x00]
          rw_reg_t CHAN1_RELOAD  :  1;     // [0x00]
          rw_reg_t CHAN2_RELOAD  :  1;     // [0x00]
          rw_reg_t CHAN3_RELOAD  :  1;     // [0x00]
          ro_reg_t _rsv_         : 23;
        } bit;
        rw_reg_t reg;
      } INTF;
      //
      // INTS Register. Interrupt status after masking & forcing
      //
      union
      {
        struct
        {
          ro_reg_t FIFO_UNDERFLOW:  1;     // [0x00]
          ro_reg_t FIFO_OVERFLOW :  1;     // [0x00]
          ro_reg_t FIFO_EMPTY    :  1;     // [0x00]
          ro_reg_t FIFO_FULL     :  1;     // [0x00]
          ro_reg_t DREQ_ACTIVE   :  1;     // [0x00]
          ro_reg_t CHAN0_RELOAD  :  1;     // [0x00]
          ro_reg_t CHAN1_RELOAD  :  1;     // [0x00]
          ro_reg_t CHAN2_RELOAD  :  1;     // [0x00]
          ro_reg_t CHAN3_RELOAD  :  1;     // [0x00]
          ro_reg_t _rsv_         : 23;
        } bit;
        rw_reg_t reg;
      } INTS;
    } PWM;
 
    PWM        *_pwm[2]     = { nullptr, nullptr };
    IO_BANK0   *_io_bank0   = nullptr;
    SYS_RIO0   *_sys_rio0   = nullptr;
    PADS_BANK0 *_pads_bank0 = nullptr;
 
    PinMode _pin_mode[28] = {};
 
    bool isValidPin(pin_size_t pin)
    {
      return pin < sizeof(_pin_mode) / sizeof(_pin_mode[0]);
    }
 
  public:
 
    RP1_GPIO(void)
    {
    }
 
    virtual ~RP1_GPIO(void)
    {
      end();
    }
 
    ChipType begin(void)
    {
      ChipType chip = getGpioChipType();
      if (chip == CHIP_RP1)
      {
        end();
        int fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (fd != -1)
        {
          size_t page = sysconf(_SC_PAGESIZE);
          _pwm[0]     = (PWM        *)mmap(0, (sizeof(*_pwm[0]    ) + page - 1) / page * page, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, PWM0_BASE      );
          _pwm[1]     = (PWM        *)mmap(0, (sizeof(*_pwm[1]    ) + page - 1) / page * page, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, PWM1_BASE      );
          _io_bank0   = (IO_BANK0   *)mmap(0, (sizeof(*_io_bank0  ) + page - 1) / page * page, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, IO_BANK0_BASE  );
          _sys_rio0   = (SYS_RIO0   *)mmap(0, (sizeof(*_sys_rio0  ) + page - 1) / page * page, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, SYS_RIO0_BASE  );
          _pads_bank0 = (PADS_BANK0 *)mmap(0, (sizeof(*_pads_bank0) + page - 1) / page * page, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, PADS_BANK0_BASE);
          close(fd);
          return chip;
        }
      }
      return CHIP_UNKNOWN;
    }
 
    void end(void)
    {
      if (_pwm[0])
      {
        munmap(_pwm[0], sizeof(*_pwm[0]));
        _pwm[0] = nullptr;
      }
      if (_pwm[1])
      {
        munmap(_pwm[1], sizeof(*_pwm[1]));
        _pwm[1] = nullptr;
      }
      if (_io_bank0)
      {
        munmap(_io_bank0, sizeof(*_io_bank0));
        _io_bank0 = nullptr;
      }
      if (_sys_rio0)
      {
        munmap(_sys_rio0, sizeof(*_sys_rio0));
        _sys_rio0 = nullptr;
      }
      if (_pads_bank0)
      {
        munmap(_pads_bank0, sizeof(*_pads_bank0));
        _pads_bank0 = nullptr;
      }
    }
 
    void setPadsVoltage(PinVoltage mode = DVDD_3V3)
    {
      _pads_bank0->ATOM[ATOM_RW].REG.VOLT.bit.VOLTAGE_SELECT = mode;
    }
 
    void setSlewFast(pin_size_t pin, bool enable = false)
    {
      if (isValidPin(pin))
        _pads_bank0->ATOM[ATOM_RW].REG.PADS[pin].bit.SLEWFAST = enable;
    }
 
    void setSchmittTrigger(pin_size_t pin, bool enable = true)
    {
      if (isValidPin(pin))
        _pads_bank0->ATOM[ATOM_RW].REG.PADS[pin].bit.SCHMITT = enable;
    }
 
    void setDriveStrength(pin_size_t pin, PinDrive mode = DRIVE_4MA)
    {
      if (isValidPin(pin))
        _pads_bank0->ATOM[ATOM_RW].REG.PADS[pin].bit.DRIVE = mode;
    }
 
    void setPullUpDown(pin_size_t pin, PinPull mode = PULL_DISABLE)
    {
      if (isValidPin(pin))
      {
        PADS_CTRL &pads = _pads_bank0->ATOM[ATOM_RW].REG.PADS[pin];
        switch (mode)
        {
          default:
          case PULL_DISABLE:
            pads.bit.PDE = 0;
            pads.bit.PUE = 0;
            break;
          case PULL_UP:
            pads.bit.PDE = 0;
            pads.bit.PUE = 1;
            break;
          case PULL_DOWN:
            pads.bit.PDE = 1;
            pads.bit.PUE = 0;
            break;
        }
      }
    }
 
    void setInterrupt(pin_size_t pin, PinInterrupt mode = IRQ_NORMAL)
    {
      if (isValidPin(pin))
        _io_bank0->ATOM[ATOM_RW].REG.PINS[pin].CTRL.bit.IRQOVER = mode;
    }
 
    void resetEvent(pin_size_t pin)
    {
      if (isValidPin(pin))
        _io_bank0->ATOM[ATOM_RW].REG.PINS[pin].CTRL.bit.IRQRESET = 1;
    }
 
    void setEventMask(pin_size_t pin, GPIO_CTRL mask)
    {
      if (isValidPin(pin))
      {
        GPIO_CTRL &gpio = _io_bank0->ATOM[ATOM_RW].REG.PINS[pin].CTRL;
        gpio.bit.IRQMASK_EDGE_LOW      = mask.bit.IRQMASK_EDGE_LOW;
        gpio.bit.IRQMASK_EDGE_HIGH     = mask.bit.IRQMASK_EDGE_HIGH;
        gpio.bit.IRQMASK_LEVEL_LOW     = mask.bit.IRQMASK_LEVEL_LOW;
        gpio.bit.IRQMASK_LEVEL_HIGH    = mask.bit.IRQMASK_LEVEL_HIGH;
        gpio.bit.IRQMASK_F_EDGE_LOW    = mask.bit.IRQMASK_F_EDGE_LOW;
        gpio.bit.IRQMASK_F_EDGE_HIGH   = mask.bit.IRQMASK_F_EDGE_HIGH;
        gpio.bit.IRQMASK_DB_LEVEL_LOW  = mask.bit.IRQMASK_DB_LEVEL_LOW;
        gpio.bit.IRQMASK_DB_LEVEL_HIGH = mask.bit.IRQMASK_DB_LEVEL_HIGH;
      }
    }
 
    GPIO_STATUS getPinStatus(pin_size_t pin)
    {
      static const GPIO_STATUS EMPTY = {};
      return isValidPin(pin) ? _io_bank0->ATOM[ATOM_RW].REG.PINS[pin].STATUS : EMPTY;
    }
 
    void setFuncSel(pin_size_t pin, PinFunc func = PinFunc_NONE)
    {
      if (isValidPin(pin))
        _io_bank0->ATOM[ATOM_RW].REG.PINS[pin].CTRL.bit.FUNCSEL = func;
    }
 
    void setFilterDebounceTime(pin_size_t pin, uint8_t m = 4)
    {
      if (isValidPin(pin))
        _io_bank0->ATOM[ATOM_RW].REG.PINS[pin].CTRL.bit.F_M = m;
    }
 
    void setInputMode(pin_size_t pin, PinInput mode = INMODE_PERI)
    {
      if (isValidPin(pin))
        _io_bank0->ATOM[ATOM_RW].REG.PINS[pin].CTRL.bit.INOVER = mode;
    }
 
    void setOutputEnable(pin_size_t pin, PinEnable mode = OEMODE_PERI)
    {
      if (isValidPin(pin))
        _io_bank0->ATOM[ATOM_RW].REG.PINS[pin].CTRL.bit.OEOVER = mode;
    }
 
    void setOutputMode(pin_size_t pin, PinOutput mode = OUTMODE_PERI)
    {
      if (isValidPin(pin))
        _io_bank0->ATOM[ATOM_RW].REG.PINS[pin].CTRL.bit.OUTOVER = mode;
    }
 
    void pinMode(pin_size_t pin, PinMode mode)
    {
      if (isValidPin(pin))
      {
        GPIO_CTRL &gpio = _io_bank0->ATOM[ATOM_RW].REG.PINS[pin].CTRL;
        PADS_CTRL &pads = _pads_bank0->ATOM[ATOM_RW].REG.PADS[pin];
        pads.bit.IE = 0;
        pads.bit.OD = 1;
        switch (_pin_mode[pin] = mode)
        {
          default:
          case INPUT:
            _sys_rio0->ATOM[ATOM_CLR].REG.OE = (1 << pin);
            gpio.bit.FUNCSEL = PinFunc_RIO;
            pads.bit.IE      = 1;
            break;
          case OUTPUT:
            _sys_rio0->ATOM[ATOM_SET].REG.OE = (1 << pin);
            gpio.bit.FUNCSEL = PinFunc_RIO;
            pads.bit.IE      = 1;
            pads.bit.OD      = 0;
            break;
          case OPEN_DRAIN:
            digitalWrite(pin, _sys_rio0->ATOM[ATOM_RW].REG.OUT & (1 << pin) ? HIGH : LOW);
            _sys_rio0->ATOM[ATOM_CLR].REG.OUT = (1 << pin);
            gpio.bit.FUNCSEL = PinFunc_RIO;
            pads.bit.IE      = 1;
            pads.bit.OD      = 0;
            break;
          case OPEN_SOURCE:
            digitalWrite(pin, _sys_rio0->ATOM[ATOM_RW].REG.OUT & (1 << pin) ? HIGH : LOW);
            _sys_rio0->ATOM[ATOM_SET].REG.OUT = (1 << pin);
            gpio.bit.FUNCSEL = PinFunc_RIO;
            pads.bit.IE      = 1;
            pads.bit.OD      = 0;
            break;
          case DISABLE:
            gpio.bit.FUNCSEL = PinFunc_NONE;
            break;
        }
      }
    }
 
    void digitalWrite(pin_size_t pin, PinStatus value)
    {
      if (isValidPin(pin))
      {
        int mode;
        switch (_pin_mode[pin])
        {
          default:
            if (value == TOGGLE)
              mode = ATOM_XOR;
            else if (value)
              mode = ATOM_SET;
            else
              mode = ATOM_CLR;
            _sys_rio0->ATOM[mode].REG.OUT = (1 << pin);
            break;
          case OPEN_DRAIN:
            if (value == TOGGLE)
              mode = ATOM_XOR;
            else if (value)
              mode = ATOM_CLR;
            else
              mode = ATOM_SET;
            _sys_rio0->ATOM[mode].REG.OE = (1 << pin);
            break;
          case OPEN_SOURCE:
            if (value == TOGGLE)
              mode = ATOM_XOR;
            else if (value)
              mode = ATOM_SET;
            else
              mode = ATOM_CLR;
            _sys_rio0->ATOM[mode].REG.OE = (1 << pin);
            break;
        }
      }
    }
 
    PinStatus digitalRead(pin_size_t pin, bool sync = false)
    {
      rw_reg_t val = 0;
      if (isValidPin(pin))
        val = (sync ? _sys_rio0->ATOM[ATOM_RW].REG.SYNC_IN : _sys_rio0->ATOM[ATOM_RW].REG.IN) & (1 << pin);
      return val ? HIGH : LOW;
    }
};
