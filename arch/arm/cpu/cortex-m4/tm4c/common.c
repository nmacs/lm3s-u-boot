#include <common.h>

#ifdef CONFIG_DISPLAY_CPUINFO
/* Print CPU information */
int print_cpuinfo(void)
{
	puts("TI LM4C Cortex-M4\n");
	return 0;
}
#endif	/* CONFIG_DISPLAY_CPUINFO */

