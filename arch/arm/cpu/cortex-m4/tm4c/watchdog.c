#include <common.h>

#define CURRENT_WDT 0

#define WDT_MAX_TIME     ((0xFFFFFFFF / CONFIG_SYSCLK_FREQUENCY) * 2) /* seconds */

#if CONFIG_WATCHDOG_TIMEOUT > WDT_MAX_TIME
# error "CONFIG_WATCHDOG_TIMEOUT > WDT_MAX_TIME"
#endif

static int wd_ready = 0;

static inline void _wdt_unlock(void)
{
	putreg32(WATCHDOG_WDTLOCK_MAGIC, LM3S_WATCHDOG_WDTLOCK(CURRENT_WDT));
}

/*
 * Reload the watchdog timer.  (ie, pat the watchdog)
 */
static inline void lm3s_wdt_reload(void)
{
	putreg32(1, LM3S_WATCHDOG_WDTICR(CURRENT_WDT));
}

/*
 * Enable and reset the watchdog.
 */
static inline void lm3s_wdt_start(void)
{
	uint32_t regval;
	uint32_t tval = CONFIG_WATCHDOG_TIMEOUT * (CONFIG_SYSCLK_FREQUENCY / 2);

	watchdog_clock_ctrl(CURRENT_WDT, SYS_ENABLE_CLOCK);

	_wdt_unlock();

	putreg32(tval, LM3S_WATCHDOG_WDTLOAD(CURRENT_WDT));
	lm3s_wdt_reload();

	regval = getreg32(LM3S_WATCHDOG_WDTCTL(CURRENT_WDT));
	regval |= WATCHDOG_WDTCTL_RESEN_MASK;
	putreg32(regval, LM3S_WATCHDOG_WDTCTL(CURRENT_WDT));
	regval |= WATCHDOG_WDTCTL_INTEN_MASK;
	putreg32(regval, LM3S_WATCHDOG_WDTCTL(CURRENT_WDT));

	wd_ready = 1;
}

void watchdog_reset(void)
{
	if( wd_ready )
		lm3s_wdt_reload();
}

int watchdog_init(void)
{
	lm3s_wdt_start();
	return 0;
}