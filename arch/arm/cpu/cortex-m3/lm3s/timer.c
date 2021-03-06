#include <common.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>

void reset_timer(void)
{

}

ulong get_timer(ulong base)
{
  return 0;
}

void __udelay(unsigned long usec)
{
  __asm__ __volatile__(
      "1:\n"
      "\tsubs  %0, #1\n"
#if SYSCLK_FREQUENCY == 50000000
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
#else
#  error "Unknown SYSCLK_FREQUENCY value."
#endif
      "\tbne   1b\n"
  :
      "=r"(usec)
  :
      "r"(usec));
}

int timer_init(void)
{
	return 0;
}

