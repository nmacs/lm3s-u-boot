#include <common.h>
#include <asm/arch/hardware.h>

#ifndef CONFIG_SYSCLK_FREQUENCY
#  error "CONFIG_SYSCLK_FREQUENCY is undefined"
#endif

#ifndef CONFIG_XTAL_FREQUENCY
#  error "CONFIG_XTAL_FREQUENCY is undefined"
#endif

#if CONFIG_XTAL_FREQUENCY != 12000000
#  error "Unknown value of CONFIG_XTAL_FREQUENCY"
#endif

#if CONFIG_SYSCLK_FREQUENCY == 60000000
#  define PLLFREQ0  (SYSCON_PLLFREQ0_MINT_SET(40) | \
                     SYSCON_PLLFREQ0_MFRAC_SET(0) | \
                     SYSCON_PLLFREQ0_PLLPWR)
#  define PLLFREQ1  (SYSCON_PLLFREQ1_N_SET(0) | \
                     SYSCON_PLLFREQ1_Q_SET(0))
#  define RSCLKCFG  (SYSCON_RSCLKCFG_PSYSDIV_SET(7) | \
                     SYSCON_RSCLKCFG_USEPLL | \
                     SYSCON_RSCLKCFG_MEMTIMU | \
                     SYSCON_RSCLKCFG_PLLSRC_MOSC | SYSCON_RSCLKCFG_NEWFREQ)
#  define MEMTIME0  (SYSCON_MEMTIM0_FBCHT_2_0  | \
                     SYSCON_MEMTIM0_FWS_SET(3) | \
                     SYSCON_MEMTIM0_FBCE_FALL  | \
                     SYSCON_MEMTIM0_EBCHT_2_0  | \
                     SYSCON_MEMTIM0_EWS_SET(3) | \
                     SYSCON_MEMTIM0_EBCE_FALL)
#  define MEMTIME0_MASK (SYSCON_MEMTIM0_FBCHT_MASK | \
                         SYSCON_MEMTIM0_FWS_MASK | \
                         SYSCON_MEMTIM0_FBCE_MASK | \
                         SYSCON_MEMTIM0_EBCHT_MASK | \
                         SYSCON_MEMTIM0_EWS_MASK | \
                         SYSCON_MEMTIM0_EBCE_MASK)
#elif CONFIG_SYSCLK_FREQUENCY == 120000000
#  define PLLFREQ0  (SYSCON_PLLFREQ0_MINT_SET(40) | \
                     SYSCON_PLLFREQ0_MFRAC_SET(0) | \
                     SYSCON_PLLFREQ0_PLLPWR)
#  define PLLFREQ1  (SYSCON_PLLFREQ1_N_SET(0) | \
                     SYSCON_PLLFREQ1_Q_SET(0))
#  define RSCLKCFG  (SYSCON_RSCLKCFG_PSYSDIV_SET(3) | \
                     SYSCON_RSCLKCFG_USEPLL | \
                     SYSCON_RSCLKCFG_MEMTIMU | \
                     SYSCON_RSCLKCFG_PLLSRC_MOSC | SYSCON_RSCLKCFG_NEWFREQ)
#  define MEMTIME0  (SYSCON_MEMTIM0_FBCHT_3_5  | \
                     SYSCON_MEMTIM0_FWS_SET(6) | \
                     SYSCON_MEMTIM0_FBCE_FALL  | \
                     SYSCON_MEMTIM0_EBCHT_3_5  | \
                     SYSCON_MEMTIM0_EWS_SET(3) | \
                     SYSCON_MEMTIM0_EBCE_FALL)
#  define MEMTIME0_MASK (SYSCON_MEMTIM0_FBCHT_MASK | \
                         SYSCON_MEMTIM0_FWS_MASK | \
                         SYSCON_MEMTIM0_FBCE_MASK | \
                         SYSCON_MEMTIM0_EBCHT_MASK | \
                         SYSCON_MEMTIM0_EWS_MASK | \
                         SYSCON_MEMTIM0_EBCE_MASK)
#else
#  error "Unknown value of CONFIG_SYSCLK_FREQUENCY"
#endif

#define MOSC_DELAY    (512*1024)
#define PLLLOCK_DELAY (512*1024)

static inline void delay(uint32_t delay)
{
  __asm__ __volatile__("1:\n"
                       "\tsubs  %0, #1\n"
                       "\tbne   1b\n"
                       : "=r"(delay) : "r"(delay));
}

static inline void plllock(void)
{
	uint32_t delay;
	/* Loop until the lock is achieved or until a timeout occurs */
	for (delay = PLLLOCK_DELAY; delay > 0; delay--) {
		/* Check if the PLL is locked on */
		if (getreg32(LM3S_SYSCON_PLLSTAT) != 0)
			return;
	}
}

void clockconfig(void)
{
	uint32_t regval;
	
	/* Enable Main Oscillator */
	regval = getreg32(LM3S_SYSCON_MOSCCTL);
	regval &= ~(SYSCON_MOSCCTL_NOXTAL | SYSCON_MOSCCTL_PWRDN);
	putreg32(regval, LM3S_SYSCON_MOSCCTL);
	
	delay(MOSC_DELAY);
	
	/* Setup PLL */
	putreg32(PLLFREQ0, LM3S_SYSCON_PLLFREQ0);
	putreg32(PLLFREQ1, LM3S_SYSCON_PLLFREQ1);
	
	regval = getreg32(LM3S_SYSCON_MEMTIM0);
	regval &= ~MEMTIME0_MASK;
	regval |= MEMTIME0;
	putreg32(regval, LM3S_SYSCON_MEMTIM0);
	
	/* Wait for PLL */
	plllock();
	
	regval = getreg32(LM3S_SYSCON_RSCLKCFG);
	regval |= RSCLKCFG;
	putreg32(regval, LM3S_SYSCON_RSCLKCFG);
	
	delay(6);
}