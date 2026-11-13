#ifndef WATCHDOG_H
#define WATCHDOG_H

/* Hardware watchdog (SAMD51 WDT, ~16 s period). Implemented in
 * firmware.ino; declared here so flight_log_dump_serial() can keep the
 * dog alive during its ~40 s record printout. */

void watchdog_init(void);
void watchdog_feed(void);

#endif /* WATCHDOG_H */
