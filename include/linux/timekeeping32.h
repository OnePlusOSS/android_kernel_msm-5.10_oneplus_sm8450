#ifndef _LINUX_TIMEKEEPING32_H
#define _LINUX_TIMEKEEPING32_H
/*
 * These interfaces are all based on the old timespec type
 * and should get replaced with the timespec64 based versions
 * over time so we can remove the file here.
 */

static inline unsigned long get_seconds(void)
{
	return ktime_get_real_seconds();
}

//add for dx1 bringup
static inline void getnstimeofday(struct timespec *ts)
{
	struct timespec64 ts64;

	ktime_get_real_ts64(&ts64);
	*ts = timespec64_to_timespec(ts64);
}

#endif
