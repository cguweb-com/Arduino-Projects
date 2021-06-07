/* This library code is placed under the MIT license
 * Copyright (c) 2020 Frank Bösing
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "T4_PowerButton.h"


#if defined(ARDUINO_TEENSY40)
  static const unsigned DTCM_START = 0x20000000UL;
  static const unsigned OCRAM_START = 0x20200000UL;
  static const unsigned OCRAM_SIZE = 512;
  static const unsigned FLASH_SIZE = 1984;
#elif defined(ARDUINO_TEENSY41)
  static const unsigned DTCM_START = 0x20000000UL;
  static const unsigned OCRAM_START = 0x20200000UL;
  static const unsigned OCRAM_SIZE = 512;
  static const unsigned FLASH_SIZE = 7936;
#if TEENSYDUINO>151
  extern "C" uint8_t external_psram_size;
#endif
#endif

static void (*__user_power_button_callback)(void);
static callback_ex_action (*__user_power_button_callback_ex)(void);

FLASHMEM __attribute__((noreturn))
void arm_power_down() {
  SNVS_LPCR |= SNVS_LPCR_TOP; //Switch off now
  asm volatile ("dsb");
  while (1) asm ("wfi");
}

/*
 * To be used combined with callback_ex_action_poweroff_keeparmed
 * If this function is not called, the normal program will not
 * continue. IntervalTimers, however, will. See the example
 */
FLASHMEM
void rearm_power_button_callback(void)
{
  if(__user_power_button_callback != nullptr || __user_power_button_callback_ex != nullptr)
    SNVS_LPSR |= (1 << 18) | (1 << 17);
}


// I think bit 18 should be checked instead bit 17
bool arm_power_button_pressed(void) {
  return (SNVS_LPSR >> 18) & 0x01;
}

/*
 * Checks whether one or both possible callbacks has been installed.
 * Each installed callback will be called.
 * The possible result of __user_power_button_callback_ex will be
 * evaluated and the action will be accordingly
 */
FLASHMEM
void __int_power_button(void) {
  if (SNVS_HPSR & 0x40) {
    SNVS_HPCOMR |= (1 << 31) ;//| (1 << 4);
    // This would prevent the callback from being called again
    // while the poweroff line is low. We will postpone this decision
    // till after the callbacks are called;
    // SNVS_LPSR |= (1 << 18) | (1 << 17);

    if (__user_power_button_callback != nullptr) __user_power_button_callback();

    callback_ex_action _action = __user_power_button_callback_ex == nullptr ? callback_ex_action_poweroff : callback_ex_action_poweroff_cancel;
    if (__user_power_button_callback_ex != nullptr)
        _action = __user_power_button_callback_ex();

    if(_action == callback_ex_action_poweroff) {
      SNVS_LPSR |= (1 << 18) | (1 << 17);
      __disable_irq();
      NVIC_CLEAR_PENDING(IRQ_SNVS_ONOFF);
      arm_power_down();
    } else {
        if(_action == callback_ex_action_poweroff_cancel)
          SNVS_LPSR |= (1 << 18) | (1 << 17);
        // Else keeparmed
    }
  }
}

/*
 * This installs a callback that does not return a value.
 * When there is no callback installed using
 * set_arm_power_button_callback_ex, a poweroff will be
 * performed after the callback has returned (once the poweroff line has been brought low)
 */
FLASHMEM
void set_arm_power_button_callback(void (*fun_ptr)(void)) {
  SNVS_LPSR |= (1 << 18) | (1 << 17);
  __user_power_button_callback = fun_ptr;
  if (fun_ptr != nullptr) {
    NVIC_CLEAR_PENDING(IRQ_SNVS_ONOFF);
    attachInterruptVector(IRQ_SNVS_ONOFF, &__int_power_button);
    NVIC_SET_PRIORITY(IRQ_SNVS_ONOFF, 255); //lowest priority
    asm volatile ("dsb"); //make sure to write before interrupt-enable
    NVIC_ENABLE_IRQ(IRQ_SNVS_ONOFF);
  } else {
    if (__user_power_button_callback_ex == nullptr)
      NVIC_DISABLE_IRQ(IRQ_SNVS_ONOFF);
  }
  asm volatile ("dsb":::"memory");
}

/*
 * This installs a callback that returns an int.
 * The return value (enum callback_ex_action) of the callback will
 * determine, whether a poweroff is performed,
 * or the poweroff is canceled or the callback function
 * should be called back, while the poweroff line is low.
 */
FLASHMEM
void set_arm_power_button_callback_ex(callback_ex_action (*fun_ptr)(void)) {
  SNVS_LPSR |= (1 << 18) | (1 << 17);
  __user_power_button_callback_ex = fun_ptr;
  if (fun_ptr != nullptr) {
    NVIC_CLEAR_PENDING(IRQ_SNVS_ONOFF);
    attachInterruptVector(IRQ_SNVS_ONOFF, &__int_power_button);
    NVIC_SET_PRIORITY(IRQ_SNVS_ONOFF, 255); //lowest priority
    asm volatile ("dsb"); //make sure to write before interrupt-enable
    NVIC_ENABLE_IRQ(IRQ_SNVS_ONOFF);
  } else {
    if (__user_power_button_callback == nullptr)
      NVIC_DISABLE_IRQ(IRQ_SNVS_ONOFF);
  }
  asm volatile ("dsb":::"memory");
}

void set_arm_power_button_callback(void (*fun_ptr)(void));
void set_arm_power_button_debounce(arm_power_button_debounce debounce) { SNVS_LPCR = (SNVS_LPCR & ~(3 << 18)) | (debounce << 18); }
void set_arm_power_button_press_time_emergency(arm_power_button_press_time_emergency emg) { SNVS_LPCR = (SNVS_LPCR & ~(3 << 16)) | (emg << 16); }
void set_arm_power_button_press_on_time(arm_power_button_press_on_time ontime) { SNVS_LPCR = (SNVS_LPCR & ~(3 << 20)) | (ontime << 20); }
void arm_enable_nvram(void) { SNVS_LPCR |= (1 << 24); }

FLASHMEM __attribute__((noreturn))
void arm_reset(void) {
#if TEENSYDUINO < 150
  IOMUXC_GPR_GPR16 = 0x00200007;
  asm volatile ("dsb":::"memory");
#endif
  SCB_AIRCR = 0x05FA0004;
  while (1) asm ("wfi");
}

unsigned memfree(void) {
  extern unsigned long _ebss;
  extern unsigned long _sdata;
  extern unsigned long _estack;
  const unsigned DTCM_START = 0x20000000UL;
  unsigned dtcm = (unsigned)&_estack - DTCM_START;
  unsigned stackinuse = (unsigned) &_estack -  (unsigned) __builtin_frame_address(0);
  unsigned varsinuse = (unsigned)&_ebss - (unsigned)&_sdata;
  unsigned freemem = dtcm - (stackinuse + varsinuse);
  return freemem;
}

unsigned heapfree(void) {
// https://forum.pjrc.com/threads/33443-How-to-display-free-ram?p=99128&viewfull=1#post99128
  void* hTop = malloc(1);// current position of heap.
  unsigned heapTop = (unsigned) hTop;
  free(hTop);
  unsigned freeheap = (OCRAM_START + (OCRAM_SIZE * 1024)) - heapTop;
  return freeheap;
}

extern "C" {
void startup_early_hook(void) {
  extern unsigned long _ebss;
  uint32_t e = (uintptr_t)&_ebss;
  uint32_t * p = (uint32_t*)e + 32;
  size_t size = (size_t)(uint8_t*)__builtin_frame_address(0) - ((uintptr_t) &_ebss + 32) - 1024;
  memset((void*)p, 0, size);
}
}

unsigned long maxstack(void) {
  extern unsigned long _ebss;
  extern unsigned long _estack;
  uint32_t e = (uintptr_t)&_ebss;
  uint32_t * p = (uint32_t*)e + 32;
  while (*p == 0) p++;
  return (unsigned) &_estack - (unsigned) p;
}

FLASHMEM
void progInfo(void) {
  Serial.println(__FILE__ " " __DATE__ " " __TIME__ );
  Serial.print("Teensyduino version ");
  Serial.println(TEENSYDUINO / 100.0f);
  Serial.println();
}


FLASHMEM
void flexRamInfo(void) {

  //extern unsigned long _stext;
  extern unsigned long _etext;
  extern unsigned long _sdata;
  extern unsigned long _ebss;
  extern unsigned long _flashimagelen;
  extern unsigned long _heap_start;
  extern unsigned long _estack;
  extern unsigned long _itcm_block_count;

  int itcm = (unsigned long)&_itcm_block_count;
  int dtcm = 0;
  int ocram = 0;
  uint32_t gpr17 = IOMUXC_GPR_GPR17;

  char __attribute__((unused)) dispstr[17] = {0};
  dispstr[16] = 0;

  for (int i = 15; i >= 0; i--) {
    switch ((gpr17 >> (i * 2)) & 0b11) {
      default: dispstr[15 - i] = '.'; break;
      case 0b01: dispstr[15 - i] = 'O'; ocram++; break;
      case 0b10: dispstr[15 - i] = 'D'; dtcm++; break;
      case 0b11: dispstr[15 - i] = 'I'; break;
    }
  }

  const char* fmtstr = "%-6s%7d %5.02f%% of %4dkB (%7d Bytes free) %s\n";

  Serial.printf(fmtstr, "FLASH:",
                (unsigned)&_flashimagelen,
                (double)((unsigned)&_flashimagelen) / (FLASH_SIZE * 1024) * 100,
                FLASH_SIZE,
                FLASH_SIZE * 1024 - ((unsigned)&_flashimagelen), "FLASHMEM, PROGMEM");

  unsigned long szITCM = itcm>0?(unsigned long)&_etext:0;
  Serial.printf(fmtstr, "ITCM:",
                szITCM,
                (double)(itcm>0?(((double)szITCM / (itcm * 32768) * 100)):0),
                itcm * 32,
                itcm * 32768 - szITCM, "(RAM1) FASTRUN");

  void* hTop = malloc(8);// current position of heap.
  unsigned heapTop = (unsigned) hTop;
  free(hTop);
  unsigned freeheap = (OCRAM_START + (OCRAM_SIZE * 1024)) - heapTop;
#if defined(ARDUINO_TEENSY41) && TEENSYDUINO>151
  if (external_psram_size > 0) {
	Serial.printf("PSRAM: %d MB\n", external_psram_size);
  } else {
	Serial.printf("PSRAM: none\n", external_psram_size);
  }
#endif
  Serial.printf("OCRAM:\n  %7d Bytes (%d kB)\n", OCRAM_SIZE * 1024, OCRAM_SIZE);
  Serial.printf("- %7d Bytes (%d kB) DMAMEM\n", ((unsigned)&_heap_start - OCRAM_START), ((unsigned)&_heap_start - OCRAM_START) / 1024);
  Serial.printf("- %7d Bytes (%d kB) Heap\n", (heapTop - (unsigned)&_heap_start ), (heapTop - (unsigned)&_heap_start ) / 1024);
  Serial.printf("  %7d Bytes heap free (%d kB), %d Bytes OCRAM in use (%d kB).\n",
                freeheap, freeheap / 1024,
                heapTop - OCRAM_START, (heapTop - OCRAM_START) / 1024);

  unsigned _dtcm = (unsigned)&_estack - DTCM_START; //or, one could use dtcm * 32768 here.
  //unsigned stackinuse = (unsigned) &_estack -  (unsigned) __builtin_frame_address(0);
  unsigned stackinuse = maxstack();
  unsigned varsinuse = (unsigned)&_ebss - (unsigned)&_sdata;
  //unsigned freemem = _dtcm - stackinuse - varsinuse;
  Serial.printf("DTCM:\n  %7d Bytes (%d kB)\n", _dtcm, _dtcm / 1024);
  Serial.printf("- %7d Bytes (%d kB) global variables\n", varsinuse, varsinuse / 1024);
  Serial.printf("- %7d Bytes (%d kB) max. stack so far\n", stackinuse, stackinuse / 1024);
  Serial.println("=========");
  Serial.printf("  %7d Bytes free (%d kB), %d Bytes in use (%d kB).\n",
                _dtcm - (varsinuse + stackinuse), (_dtcm - (varsinuse + stackinuse)) / 1024,
                varsinuse + stackinuse, (varsinuse + stackinuse) / 1024
               );
}




/* -------------------------------------------------------------------------------------------------- */
/* -----------------------Hardfaults----------------------------------------------------------------- */
/* -------------------------------------------------------------------------------------------------- */

#if defined(SHOW_HARDFAULTS)

static const uint32_t _marker = 0xfbfb;

typedef struct __attribute__((packed)) ContextStateFrame {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t return_address;
  uint32_t xpsr;
} sContextStateFrame;

struct __attribute__((packed)) RegInfo
{
  uint32_t marker;
  uint32_t ipsr;
  uint32_t cfsr;
  uint32_t hfsr;
  //uint32_t dfsr;
  uint32_t mmar;
  uint32_t bfar;
  //uint32_t afsr;
  uint32_t return_address;
  uint32_t xpsr;
  bool temperature;
};


DMAMEM RegInfo sRegInfo;

extern "C" {

  FASTRUN __attribute__((used, noreturn, optimize("O0"))) static
  void my_fault_handler_c(ContextStateFrame *frame) {

    //Read these both first:
    sRegInfo.mmar = (*((volatile uint32_t *)(0xE000ED34)));     // MemManage Fault Address Register
    sRegInfo.bfar = (*((volatile uint32_t *)(0xE000ED38)));     // Bus Fault Address Register
    asm volatile("mrs %0, ipsr\n" : "=r" (sRegInfo.ipsr)::);
    volatile uint32_t *cfsr = (volatile uint32_t *)0xE000ED28;  // Configurable Fault Status Register

    //sRegInfo.sContextStateFrame.r0 = frame->r0;
    //sRegInfo.sContextStateFrame.r1 = frame->r1;
    //sRegInfo.sContextStateFrame.r2 = frame->r2;
    //sRegInfo.sContextStateFrame.r12 = frame->r12;
    //sRegInfo.sContextStateFrame.lr = frame->lr;
    sRegInfo.return_address = frame->return_address;
    sRegInfo.xpsr = frame->xpsr;

    sRegInfo.hfsr = (*((volatile uint32_t *)(0xE000ED2C)));     // Hard Fault Status Register
    //sRegInfo.dfsr = (*((volatile uint32_t *)(0xE000ED30)));     // Debug Fault Status Register
    //sRegInfo.afsr = (*((volatile uint32_t *)(0xE000ED3C)));     // Auxiliary Fault Status Register
    sRegInfo.cfsr = *cfsr;
    *cfsr |= *cfsr;

    sRegInfo.temperature = 0;
    sRegInfo.marker = _marker;
    arm_dcache_flush((void*)&sRegInfo, sizeof(RegInfo));

    //Reset:
    volatile uint32_t *aircr = (volatile uint32_t *)0xE000ED0C;
    *aircr = (0x05FA << 16) | 0x1 << 2;
    asm volatile("dsb");
    while (1) asm ("WFI");

  }

  FLASHMEM
  static void fault_temp_isr(void) {
    sRegInfo.temperature = true;
    sRegInfo.marker = _marker;
    arm_dcache_flush((void*)&sRegInfo, sizeof(RegInfo));
    volatile uint32_t *aircr = (volatile uint32_t *)0xE000ED0C;
    *aircr = (0x05FA << 16) | 0x1 << 2;
    while (1) asm ("WFI");
  }

  FASTRUN __attribute__((naked)) static
  void interrupt_vector(void)
  {
    __asm( ".syntax unified\n"
           "MOVS R0, #4 \n"
           "MOV R1, LR \n"
           "TST R0, R1 \n"
           "BEQ _MSP \n"
           "MRS R0, PSP \n"
           "B my_fault_handler_c \n"
           "_MSP: \n"
           "MRS R0, MSP \n"
           "B my_fault_handler_c \n"
           ".syntax divided\n") ;
  }

} //extern "c"

FLASHMEM
bool show_callstack(void)
{

  arm_dcache_delete((void*)&sRegInfo, sizeof(RegInfo));
  bool found = sRegInfo.marker == _marker;
  if (!found) return false;

#if (HARDFAULTSOUT==Serial)
  while(!Serial && millis() < 10000){}
#endif

  if (sRegInfo.temperature) {
	HARDFAULTSOUT.println("Temperature Panic.\n Power down.\n");
	HARDFAULTSOUT.flush();
	delay(5);
	IOMUXC_GPR_GPR16 = 0x00000007;
	SNVS_LPCR |= SNVS_LPCR_TOP; //Switch off now
	while (1) asm ("wfi");
  }


  HARDFAULTSOUT.print("Hardfault.\nReturn Address: 0x");
  HARDFAULTSOUT.println(sRegInfo.return_address, HEX);

  //const bool non_usage_fault_occurred = (sRegInfo.cfsr & ~0xffff0000) != 0;
  const bool faulted_from_exception = ((sRegInfo.xpsr & 0xFF) != 0);
  //if (non_usage_fault_occurred) HARDFAULTSOUT.println("non usage fault");
  if (faulted_from_exception) HARDFAULTSOUT.printf("Faulted from exception.\n");

  // the bottom 8 bits of the xpsr hold the exception number of the
  // executing exception or 0 if the processor is in Thread mode
  // const bool faulted_from_exception = ((frame->xpsr & 0xFF) != 0);

  //taken from startup.c :

  uint32_t _CFSR = sRegInfo.cfsr;
  if (_CFSR > 0) {

    if ((_CFSR & 0xff) > 0) {
      //Memory Management Faults
      if ((_CFSR & 1) == 1) {
        HARDFAULTSOUT.println("\t(IACCVIOL) Instruction Access Violation");
      } else  if (((_CFSR & (0x02)) >> 1) == 1) {
        HARDFAULTSOUT.println("\t(DACCVIOL) Data Access Violation");
      } else if (((_CFSR & (0x08)) >> 3) == 1) {
        HARDFAULTSOUT.println("\t(MUNSTKERR) MemMange Fault on Unstacking");
      } else if (((_CFSR & (0x10)) >> 4) == 1) {
        HARDFAULTSOUT.println("\t(MSTKERR) MemMange Fault on stacking");
      } else if (((_CFSR & (0x20)) >> 5) == 1) {
        HARDFAULTSOUT.println("\t(MLSPERR) MemMange Fault on FP Lazy State");
      }
      if (((_CFSR & (0x80)) >> 7) == 1) {
        HARDFAULTSOUT.print("\t(MMARVALID) Accessed Address: 0x");
	HARDFAULTSOUT.print(sRegInfo.mmar, HEX);
	if (sRegInfo.mmar < 32) HARDFAULTSOUT.print(" (nullptr)");
	extern unsigned long _ebss;
	if ((sRegInfo.mmar >= (uint32_t)&_ebss) && (sRegInfo.mmar < (uint32_t)&_ebss + 32))
		HARDFAULTSOUT.print(" (Stack problem)\n\tCheck for stack overflows, array bounds, etc.");
	HARDFAULTSOUT.println();
      }
    }

    //Bus Fault Status Register BFSR
    if (((_CFSR & 0x100) >> 8) == 1) {
      HARDFAULTSOUT.println("\t(IBUSERR) Instruction Bus Error");
    } else  if (((_CFSR & (0x200)) >> 9) == 1) {
      HARDFAULTSOUT.println("\t(PRECISERR) Data bus error(address in BFAR)");
    } else if (((_CFSR & (0x400)) >> 10) == 1) {
      HARDFAULTSOUT.println("\t(IMPRECISERR) Data bus error but address not related to instruction");
    } else if (((_CFSR & (0x800)) >> 11) == 1) {
      HARDFAULTSOUT.println("\t(UNSTKERR) Bus Fault on unstacking for a return from exception");
    } else if (((_CFSR & (0x1000)) >> 12) == 1) {
      HARDFAULTSOUT.println("\t(STKERR) Bus Fault on stacking for exception entry");
    } else if (((_CFSR & (0x2000)) >> 13) == 1) {
      HARDFAULTSOUT.println("\t(LSPERR) Bus Fault on FP lazy state preservation");
    }
    if (((_CFSR & (0x8000)) >> 15) == 1) {
      HARDFAULTSOUT.print("\t(BFARVALID) Accessed Address: 0x");
      HARDFAULTSOUT.println(sRegInfo.bfar, HEX);
    }


    //Usage Fault Status Register UFSR
    if (((_CFSR & 0x10000) >> 16) == 1) {
      HARDFAULTSOUT.println("\t(UNDEFINSTR) Undefined instruction");
    } else  if (((_CFSR & (0x20000)) >> 17) == 1) {
      HARDFAULTSOUT.println("\t(INVSTATE) Instruction makes illegal use of EPSR)");
    } else if (((_CFSR & (0x40000)) >> 18) == 1) {
      HARDFAULTSOUT.println("\t(INVPC) Usage fault: invalid EXC_RETURN");
    } else if (((_CFSR & (0x80000)) >> 19) == 1) {
      HARDFAULTSOUT.println("\t(NOCP) No Coprocessor");
    } else if (((_CFSR & (0x1000000)) >> 24) == 1) {
      HARDFAULTSOUT.println("\t(UNALIGNED) Unaligned access UsageFault");
    } else if (((_CFSR & (0x2000000)) >> 25) == 1) {
      HARDFAULTSOUT.println("\t(DIVBYZERO) Divide by zero");
    }
  }

  uint32_t _HFSR = sRegInfo.hfsr;
  if (_HFSR > 0) {
    //Memory Management Faults
    if (((_HFSR & (0x02)) >> 1) == 1) {
      HARDFAULTSOUT.println("\t(VECTTBL) Bus Fault on Vec Table Read");
    } else if (((_HFSR & (0x40000000)) >> 30) == 1) {
      // HARDFAULTSOUT.println("\t(FORCED) Forced Hard Fault");
    } else if (((_HFSR & (0x80000000)) >> 31) == 31) {
      // HARDFAULTSOUT.println("\t(DEBUGEVT) Reserved for Debug");
    }
  }

#if 0
  HARDFAULTSOUT.printf("\nDEBUG DUMP:\n");
  HARDFAULTSOUT.printf("ipsr:0x%X ", sRegInfo.ipsr);
  HARDFAULTSOUT.printf("cfsr:0x%08X ", sRegInfo.cfsr);
  HARDFAULTSOUT.printf("hfsr:0x%08X ", sRegInfo.hfsr);
  HARDFAULTSOUT.printf("dfsr:0x%08X ", sRegInfo.dfsr);
  HARDFAULTSOUT.printf("mmar:0x%08X ", sRegInfo.mmar);
  HARDFAULTSOUT.printf("bfar:0x%08X ", sRegInfo.bfar);
  HARDFAULTSOUT.printf("afsr:0x%08X\n", sRegInfo.afsr);
  HARDFAULTSOUT.printf("r0:0x%X ", sRegInfo.sContextStateFrame.r0);
  HARDFAULTSOUT.printf("r1:0x%X ", sRegInfo.sContextStateFrame.r1);
  HARDFAULTSOUT.printf("r2:0x%X ", sRegInfo.sContextStateFrame.r2);
  HARDFAULTSOUT.printf("r12:0x%X ", sRegInfo.sContextStateFrame.r12);
  HARDFAULTSOUT.printf("lr:0x%X ", sRegInfo.sContextStateFrame.lr);
  HARDFAULTSOUT.printf("return_address:0x%X ", sRegInfo.sContextStateFrame.return_address);
  HARDFAULTSOUT.printf("xpsr:0x%X\n", sRegInfo.sContextStateFrame.xpsr);
#endif

  sRegInfo.marker = 0;
  arm_dcache_flush_delete((void*)&sRegInfo, 32);
  return true;
}

extern "C" {

 FLASHMEM void startup_late_hook()
 {

   attachInterruptVector(IRQ_TEMPERATURE_PANIC, &fault_temp_isr);
  _VectorsRam[3] = interrupt_vector;
   SCB_CCR = 0x10; //Enable "Div By Zero" exceptions

   if (show_callstack()) delay(10000);

 }

} //extern c

#endif
