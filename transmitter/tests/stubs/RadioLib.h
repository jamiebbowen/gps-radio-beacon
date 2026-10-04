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
#define RADIOLIB_SX126X_IRQ_RX_DONE  0x0002u
#define RADIOLIB_SX126X_IRQ_CRC_ERR  0x0040u
#define RADIOLIB_SX126X_IRQ_TIMEOUT  0x0200u

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
extern int     radiolib_set_cl_calls;   /* setCurrentLimit() call count */
extern float   radiolib_current_limit;  /* captured mA argument         */
extern int     radiolib_set_cl_result;  /* scripted return value        */
extern uint32_t radiolib_time_on_air_us; /* scripted getTimeOnAir() us  */
extern int     radiolib_start_receive_result; /* startReceive() retval  */
extern uint32_t radiolib_irq_flags;     /* getIrqFlags() scripted value */
extern uint8_t radiolib_rx_buf[256];    /* scripted inbound packet      */
extern size_t  radiolib_rx_len;         /* scripted packet length       */

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

    int setCurrentLimit(float ma)
    {
        radiolib_set_cl_calls++;
        radiolib_current_limit = ma;
        return radiolib_set_cl_result;
    }

    uint32_t getTimeOnAir(uint8_t len)
    {
        (void)len;
        return radiolib_time_on_air_us;
    }

    /* Two-way-survey scaffolding: radio_poll_rx drives these knobs so the
     * harness injects a scripted inbound frame (or none). */
    int startReceive()
    {
        return radiolib_start_receive_result;
    }

    uint32_t getIrqFlags()
    {
        return radiolib_irq_flags;
    }

    size_t getPacketLength()
    {
        return radiolib_rx_len;
    }

    int readData(uint8_t *data, size_t len)
    {
        if (len > radiolib_rx_len) len = radiolib_rx_len;
        memcpy(data, radiolib_rx_buf, len);
        return RADIOLIB_ERR_NONE;
    }

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
