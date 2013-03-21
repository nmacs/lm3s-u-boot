#include <common.h>

#define CURRENT_WDT 0

#define WDT_DEFAULT_TIME	60	/* seconds */
#define WDT_MAX_TIME		(0xFFFFFFFF / SYSCLK_FREQUENCY)	/* seconds */

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
	uint32_t tval = WDT_DEFAULT_TIME * (SYSCLK_FREQUENCY / 2);

	regval = getreg32(LM3S_SYSCON_RCGC0);
	regval |= SYSCON_RCGC0_WDT;
	/* NOTE: put LM3S_SYSCON_RCGC0 twice to workaround LM3S bug */
	putreg32(regval, LM3S_SYSCON_RCGC0);
	putreg32(regval, LM3S_SYSCON_RCGC0);

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