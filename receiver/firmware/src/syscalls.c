/**
 ******************************************************************************
 * @file           : syscalls.c
 * @brief          : Minimal newlib syscalls for bare-metal.
 *
 * Why this exists: previously LDFLAGS used -specs=nosys.specs, which links
 * libnosys.a - whose stubs every-INVOCATION emits "not implemented and will
 * always fail" linker warnings. Adding these seven no-op implementations
 * silences the warnings at the source.
 *
 * Heap: _sbrk is sized against the linker's _end and the stack guard.
 * Everything else returns "not supported" - we don't stdio anywhere anyway.
 ******************************************************************************
 */

#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

/* Linker symbols - declared as byte arrays so &sym IS the address and gcc
 * doesn't think we're indexing a scalar. */
extern uint8_t end[];       /* end of .bss / start of heap */
extern uint8_t _estack[];   /* top of stack */

int _close(int fd)                    { (void)fd; return -1; }
int _fstat(int fd, struct stat *st)   { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)                   { (void)fd; return 1; }
int _lseek(int fd, int ptr, int dir)  { (void)fd; (void)ptr; (void)dir; return 0; }
int _read(int fd, char *ptr, int len) { (void)fd; (void)ptr; (void)len; return 0; }
int _write(int fd, char *ptr, int len){ (void)fd; (void)ptr; return len; }
int _kill(int pid, int sig)           { (void)pid; (void)sig; errno = EINVAL; return -1; }
int _getpid(void)                     { return 1; }
void _exit(int status)                { (void)status; while (1) {} }

caddr_t _sbrk(ptrdiff_t incr)
{
  static uint8_t *heap_end = NULL;
  uint8_t *prev;
  if (heap_end == NULL) heap_end = end;
  prev = heap_end;
  /* Leave 256 bytes of headroom against the stack. uintptr arithmetic:
   * comparing `_estack - 256` as a pointer irks -Warray-bounds on LTO-free
   * builds since _estack is an extern array of unknown size. */
  if ((uintptr_t)(heap_end + incr) > (uintptr_t)_estack - 256u) {
    errno = ENOMEM;
    return (caddr_t)-1;
  }
  heap_end += incr;
  return (caddr_t)prev;
}
