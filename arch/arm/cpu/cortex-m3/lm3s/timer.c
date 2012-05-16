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
  unsigned long loops;
  loops = usec * (SYSCLK_FREQUENCY / 1000000 / 2);

  __asm__ __volatile__("1:\n"
                       "\tsubs  %0, #1\n"
                       "\tbne   1b\n"
                       : "=r"(loops) : "r"(loops));
}

int timer_init(void)
{

	return 0;
}

