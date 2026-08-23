/* Minimal Wire (I2C) stub for host-side transmitter tests. */
#ifndef WIRE_STUB_H
#define WIRE_STUB_H

class WireClass {
public:
    void begin() {}
};

extern WireClass Wire;

#endif /* WIRE_STUB_H */
