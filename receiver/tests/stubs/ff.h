/**
 * @file ff.h
 * @brief Minimal FatFs ff.h stub for host tests.
 *
 * The real header lives in the untracked STM32CubeF4 checkout. Host tests
 * only need the config macros that ff_gen_drv.h / sd_diskio.h read; the
 * values come from the project's real tracked config (inc/ffconf.h).
 */

#ifndef __STUB_FF_H
#define __STUB_FF_H

#include "ffconf.h"   /* project config: FF_FS_READONLY, FF_USE_IOCTL, FF_VOLUMES */

#ifndef FF_FS_READONLY
#define FF_FS_READONLY 0
#endif
#ifndef FF_USE_IOCTL
#define FF_USE_IOCTL   1
#endif
#ifndef FF_VOLUMES
#define FF_VOLUMES     1
#endif

#endif /* __STUB_FF_H */
