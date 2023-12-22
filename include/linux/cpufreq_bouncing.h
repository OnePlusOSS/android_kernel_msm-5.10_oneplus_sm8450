#ifndef __CPUFREQ_BOUNCING_H__
#define __CPUFREQ_BOUNCING_H__

#include <linux/cpufreq.h>

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GKI_CPUFREQ_BOUNCING)
void cb_update(struct cpufreq_policy *pol, u64 time);
void cb_reset(int cpu, u64 time);
unsigned int cb_cap(struct cpufreq_policy *pol, unsigned int freq);
void cb_stuff_init(struct cpufreq_policy *policy);
#else
static inline void cb_update(struct cpufreq_policy *pol, u64 time)
{
}

static inline void cb_reset(int cpu, u64 time)
{
}

static inline unsigned int cb_cap(struct cpufreq_policy *pol, unsigned int freq)
{
}
static inline void cb_stuff_init(struct cpufreq_policy *policy)
{
}
#endif

#endif /* __CPUFREQ_BOUNCING_H__ */
