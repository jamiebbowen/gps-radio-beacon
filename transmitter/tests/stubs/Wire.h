/* Minimal Wire (I2C) stub for host-side transmitter tests. */
#ifndef WIRE_STUB_H
#define WIRE_STUB_H

#include <stdint.h>

class WireClass {
public:
    void begin() {}
    void begin(uint8_t) {}
    void end() {}
    void setClock(uint32_t hz) { (void)hz; }
};

extern WireClass Wire;

#endif /* WIRE_STUB_H */
