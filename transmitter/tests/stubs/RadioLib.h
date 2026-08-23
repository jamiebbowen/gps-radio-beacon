/*
 * Minimal RadioLib stub for host-side testing of transmitter/radio.cpp.
 * The test executable defines the knobs below to script begin/transmit
 * outcomes; pin/bit-banging is irrelevant at this level.
 */
#ifndef RADIOLIB_STUB_H
#define RADIOLIB_STUB_H

#include <stdint.h>
#include <stddef.h>

#define RADIOLIB_ERR_NONE            0
#define RADIOLIB_ERR_CHIP_NOT_FOUND  -2
#define RADIOLIB_ERR_INVALID_ENCODING -11

class Module {
public:
    Module(int cs, int dio1, int nrst, int busy)
        : cs_(cs), dio1_(dio1), nrst_(nrst), busy_(busy) {}
    int cs_, dio1_, nrst_, busy_;
};

/* Test knobs (defined by the test executable) */
extern int     radiolib_begin_result;   /* return value of begin()      */
extern float   radiolib_begin_freq;     /* captured freq argument       */
extern int     radiolib_begin_calls;
extern int     radiolib_standby_calls;
extern int     radiolib_transmit_result;
extern int     radiolib_transmit_calls;
extern uint8_t radiolib_last_tx[256];
extern size_t  radiolib_last_tx_len;

class SX1268 {
public:
    SX1268(Module *mod) : mod_(mod) {}

    int begin(float freq, float bw, int sf, int cr, int syncWord,
              int power, int preamble, float tcxo)
    {
        (void)bw; (void)sf; (void)cr; (void)syncWord;
        (void)power; (void)preamble; (void)tcxo;
        radiolib_begin_calls++;
        radiolib_begin_freq = freq;
        return radiolib_begin_result;
    }

    int standby() { radiolib_standby_calls++; return RADIOLIB_ERR_NONE; }

    int transmit(uint8_t *data, size_t len)
    {
        radiolib_transmit_calls++;
        if (len > sizeof(radiolib_last_tx)) len = sizeof(radiolib_last_tx);
        memcpy(radiolib_last_tx, data, len);
        radiolib_last_tx_len = len;
        return radiolib_transmit_result;
    }

private:
    Module *mod_;
};

#endif /* RADIOLIB_STUB_H */
