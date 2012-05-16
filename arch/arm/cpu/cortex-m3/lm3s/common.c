#include <common.h>

#ifdef CONFIG_DISPLAY_CPUINFO
/* Print CPU information */
int print_cpuinfo(void)
{
	puts("TI LM3S1D21 Cortex-M3\n");
	return 0;
}
#endif	/* CONFIG_DISPLAY_CPUINFO */

