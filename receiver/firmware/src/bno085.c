/**
 * @file bno085.c
 * @brief Minimal SHTP-over-I2C driver for the BNO085 (see bno085.h).
 *
 * SHTP framing: each packet is a 16-bit little-endian length (including
 * the 4-byte header; bit 15 = continuation), then channel number, then a
 * per-channel sequence number, then the cargo. On I2C every master read
 * transaction starts at the packet's first byte: the length is learned
 * from a 4-byte header read, and the cargo read that follows returns the
 * header again ahead of the cargo (Adafruit's i2chal_read and SparkFun's
 * getData both discard it). When the hub has nothing queued it answers
 * with length 0x0000 or all-0xFF. Input-report cargo is a run of SH-2
 * records: a 0xFB base-timestamp reference, then one or more batched
 * sensor reports.
 */

#include "bno085.h"
#include <string.h>

/* SHTP channels */
#define CH_SHTP_COMMAND   0
#define CH_EXECUTABLE     1   /* device control (reset) */
#define CH_CONTROL        2   /* SH-2 control reports (SetFeature etc.) */
#define CH_INPUT_REPORTS  3   /* sensor reports */
#define CH_WAKE_REPORTS   4
#define CH_GYRO_RV        5

/* Executable channel commands */
#define EXE_CMD_RESET     0x01
#define EXE_RESP_RESET    0x01  /* payload[0] == 0x01: reset complete */

/* SH-2 control report IDs (channel 2) */
#define SH2_SET_FEATURE   0xFD

/* SH-2 sensor report IDs */
#define SH2_ACCELEROMETER             0x01
#define SH2_GYROSCOPE_CALIBRATED      0x02
#define SH2_MAGNETIC_FIELD_CALIBRATED 0x03
#define SH2_ROTATION_VECTOR           0x05
#define SH2_GAME_ROTATION_VECTOR      0x08
#define SH2_TIMESTAMP_REBASE          0xFA
#define SH2_BASE_TIMESTAMP_REF        0xFB

#define BNO085_I2C_TIMEOUT_MS  10u   /* matches the old BNO055 budget */
#define BNO085_HDR_LEN          4u
#define BNO085_MAX_PACKET     128u   /* cargo buffer: a batched input cargo
                                       * (0xFB + RV + 3 raw reports) is 49 B */
#define BNO085_CH_CONTINUATION 0xFFu /* read_packet: tail of a long packet */

static I2C_HandleTypeDef *s_hi2c = NULL;
static uint8_t s_addr8     = 0;     /* 8-bit (shifted) I2C address */
static uint8_t s_seq_control = 0;   /* host->hub seq on the control channel */
static uint8_t s_seq_exe     = 0;   /* host->hub seq on the exe channel */

/* ------------------------------------------------------------------ */
/* Transport                                                           */
/* ------------------------------------------------------------------ */

static uint8_t bno085_write(uint8_t channel, uint8_t seq,
                            const uint8_t *payload, uint8_t plen)
{
  uint8_t buf[BNO085_HDR_LEN + BNO085_MAX_PACKET];
  uint16_t len = BNO085_HDR_LEN + plen;
  if (plen > BNO085_MAX_PACKET) return BNO085_ERROR;
  buf[0] = (uint8_t)(len & 0xFF);
  buf[1] = (uint8_t)(len >> 8);
  buf[2] = channel;
  buf[3] = seq;
  memcpy(&buf[BNO085_HDR_LEN], payload, plen);
  return (HAL_I2C_Master_Transmit(s_hi2c, s_addr8, buf, len,
                                  BNO085_I2C_TIMEOUT_MS) == HAL_OK)
         ? BNO085_OK : BNO085_ERROR;
}

/**
 * @brief Read one SHTP packet.
 * @param[out] channel  packet channel (valid only on BNO085_OK)
 * @param[out] payload  destination buffer
 * @param      max_plen payload buffer capacity
 * @param[out] plen     actual payload length
 * @retval BNO085_OK packet read; BNO085_TIMEOUT nothing queued;
 *         BNO085_ERROR bus fault
 */
static uint8_t bno085_read_packet(uint8_t *channel, uint8_t *payload,
                                  uint8_t max_plen, uint8_t *plen)
{
  uint8_t hdr[BNO085_HDR_LEN];
  if (HAL_I2C_Master_Receive(s_hi2c, s_addr8, hdr, BNO085_HDR_LEN,
                             BNO085_I2C_TIMEOUT_MS) != HAL_OK) {
    return BNO085_ERROR;
  }

  uint16_t len = ((uint16_t)hdr[1] << 8) | hdr[0];
  if (len == 0xFFFF || len == 0) {
    return BNO085_TIMEOUT;  /* hub has nothing queued */
  }
  uint8_t continuation = (len & 0x8000u) ? 1 : 0;
  len &= 0x7FFF;            /* bit 15 = continuation flag */
  if (len < BNO085_HDR_LEN) {
    return BNO085_TIMEOUT;  /* malformed/short - treat as empty */
  }

  /* The cargo read restarts at the packet's first byte: header again,
   * then cargo. Reading less than a long packet is fine - the hub offers
   * the rest as a continuation packet, which is drained and ignored. */
  uint8_t pkt[BNO085_HDR_LEN + BNO085_MAX_PACKET];
  uint16_t want = len;
  if (want > sizeof(pkt)) want = sizeof(pkt);
  if (HAL_I2C_Master_Receive(s_hi2c, s_addr8, pkt, want,
                             BNO085_I2C_TIMEOUT_MS) != HAL_OK) {
    return BNO085_ERROR;
  }

  /* pkt[2] = channel, pkt[3] = channel sequence (we do not track it) */
  *channel = continuation ? (uint8_t)BNO085_CH_CONTINUATION : pkt[2];
  uint16_t out = (uint16_t)(want - BNO085_HDR_LEN);
  if (out > max_plen) out = max_plen;
  memcpy(payload, &pkt[BNO085_HDR_LEN], out);
  *plen = (uint8_t)out;
  return BNO085_OK;
}

/** Drain any queued packets (post-boot advertisement etc.). */
static void bno085_flush(uint32_t budget_ms)
{
  uint32_t start = HAL_GetTick();
  uint8_t dummy[BNO085_MAX_PACKET];
  uint8_t ch, pl;
  while (HAL_GetTick() - start < budget_ms) {
    uint8_t r = bno085_read_packet(&ch, dummy, sizeof(dummy), &pl);
    if (r == BNO085_TIMEOUT) break;
    if (r == BNO085_ERROR)   break;  /* caller decides about the bus */
  }
}

static uint8_t bno085_set_feature(uint8_t report_id, uint32_t interval_us)
{
  uint8_t p[17];
  p[0] = SH2_SET_FEATURE;
  p[1] = report_id;
  p[2] = 0;                 /* feature flags: no wake/batch sensitivities */
  p[3] = 0; p[4] = 0;       /* change sensitivity */
  p[5] = (uint8_t)(interval_us);
  p[6] = (uint8_t)(interval_us >> 8);
  p[7] = (uint8_t)(interval_us >> 16);
  p[8] = (uint8_t)(interval_us >> 24);
  p[9]  = 0; p[10] = 0; p[11] = 0; p[12] = 0;   /* batch interval: none */
  p[13] = 0; p[14] = 0; p[15] = 0; p[16] = 0;   /* sensor specific */
  return bno085_write(CH_CONTROL, s_seq_control++, p, sizeof(p));
}

/* ------------------------------------------------------------------ */
/* Report parsing                                                      */
/* ------------------------------------------------------------------ */

static int16_t rd_i16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void bno085_parse_report(BNO085_State *st,
                                const uint8_t *payload, uint8_t plen)
{
  if (plen < 4 || payload == NULL) return;
  uint8_t id     = payload[0];
  uint8_t status = payload[2];   /* bits[1:0] = accuracy 0..3 */
  const uint8_t *d = &payload[4];
  uint8_t dlen   = (uint8_t)(plen - 4);

  switch (id) {
    case SH2_ROTATION_VECTOR:
    case SH2_GAME_ROTATION_VECTOR:
      if (dlen < 8) return;   /* i, j, k, real (RV adds an accuracy est.) */
      st->quat_i    = (float)rd_i16(&d[0]) / BNO085_Q_QUAT;
      st->quat_j    = (float)rd_i16(&d[2]) / BNO085_Q_QUAT;
      st->quat_k    = (float)rd_i16(&d[4]) / BNO085_Q_QUAT;
      st->quat_real = (float)rd_i16(&d[6]) / BNO085_Q_QUAT;
      st->accuracy_rv = status & 0x03;
      st->rv_ms = HAL_GetTick();
      st->rv_count++;
      break;
    case SH2_ACCELEROMETER:
      if (dlen < 6) return;
      for (int i = 0; i < 3; i++) {
        st->accel_raw[i] = rd_i16(&d[2 * i]);
        st->accel_ms2[i] = (float)st->accel_raw[i] / BNO085_Q_ACCEL;
      }
      st->accuracy_accel = status & 0x03;
      st->accel_ms = HAL_GetTick();
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      if (dlen < 6) return;
      for (int i = 0; i < 3; i++) {
        st->gyro_raw[i] = rd_i16(&d[2 * i]);
        st->gyro_rads[i] = (float)st->gyro_raw[i] / BNO085_Q_GYRO;
      }
      st->accuracy_gyro = status & 0x03;
      st->gyro_ms = HAL_GetTick();
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      if (dlen < 6) return;
      for (int i = 0; i < 3; i++) {
        st->mag_raw[i] = rd_i16(&d[2 * i]);
        st->mag_ut[i] = (float)st->mag_raw[i] / BNO085_Q_MAG;
      }
      st->accuracy_mag = status & 0x03;
      st->mag_ms = HAL_GetTick();
      break;
    default:
      break;  /* timestamp records, unasked-for reports */
  }
}

/* Length of one SH-2 record in an input cargo, by report ID (the values
 * the hub advertises; SH-2 reference manual). 0 = unknown, after which
 * the rest of the cargo cannot be framed. */
static uint8_t bno085_record_len(uint8_t id)
{
  switch (id) {
    case SH2_BASE_TIMESTAMP_REF:
    case SH2_TIMESTAMP_REBASE:          return 5;
    case SH2_ACCELEROMETER:
    case SH2_GYROSCOPE_CALIBRATED:
    case SH2_MAGNETIC_FIELD_CALIBRATED: return 10;
    case SH2_GAME_ROTATION_VECTOR:      return 12;
    case SH2_ROTATION_VECTOR:           return 14;
    default:                            return 0;
  }
}

/* Walk an input cargo record by record: the leading 0xFB base timestamp,
 * then every batched sensor report. (Parsing cargo[0] as the report ID
 * only ever saw the timestamp record.) */
static void bno085_parse_cargo(BNO085_State *st, const uint8_t *cargo,
                               uint8_t len)
{
  uint8_t cursor = 0;
  while (cursor < len) {
    uint8_t rlen = bno085_record_len(cargo[cursor]);
    if (rlen == 0 || rlen > (uint8_t)(len - cursor)) return;  /* unknown / cut */
    bno085_parse_report(st, &cargo[cursor], rlen);
    cursor = (uint8_t)(cursor + rlen);
  }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

static uint8_t bno085_configure(void)
{
  /* Rotation vector carries the fused 9-axis quaternion (mag-dependent
   * absolute heading). The three raw channels exist for the diagnostics
   * screens and for their per-sensor accuracy (cal) bits. */
  const struct { uint8_t id; uint32_t interval_us; } feat[] = {
    { SH2_ROTATION_VECTOR,           20000u },  /*  50 Hz fused */
    { SH2_ACCELEROMETER,            100000u },  /*  10 Hz raw */
    { SH2_GYROSCOPE_CALIBRATED,     100000u },
    { SH2_MAGNETIC_FIELD_CALIBRATED, 100000u },
  };
  for (unsigned i = 0; i < sizeof(feat) / sizeof(feat[0]); i++) {
    if (bno085_set_feature(feat[i].id, feat[i].interval_us) != BNO085_OK) {
      return BNO085_ERROR;
    }
  }
  return BNO085_OK;
}

uint8_t BNO085_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
  s_hi2c = hi2c;
  s_addr8 = (uint8_t)(addr7 << 1);
  s_seq_control = 0;
  s_seq_exe = 0;

  /* Soft reset (RST pad is unwired): tell the hub, then wait for the
   * reset-complete on the executable channel, draining the boot
   * advertisement burst that follows. */
  bno085_flush(50);
  uint8_t rst = EXE_CMD_RESET;
  if (bno085_write(CH_EXECUTABLE, s_seq_exe++, &rst, 1) != BNO085_OK) {
    return BNO085_ERROR;
  }
  HAL_Delay(10);

  uint32_t start = HAL_GetTick();
  uint8_t reset_seen = 0;
  while (HAL_GetTick() - start < 500) {
    uint8_t buf[BNO085_MAX_PACKET], ch, pl;
    uint8_t r = bno085_read_packet(&ch, buf, sizeof(buf), &pl);
    if (r == BNO085_OK) {
      /* Reset-complete on the executable channel: payload[0] == 0x01
       * (matches the Adafruit driver check). */
      if (ch == CH_EXECUTABLE && pl >= 1 && buf[0] == EXE_RESP_RESET) {
        reset_seen = 1;
        break;
      }
    } else {
      if (r == BNO085_ERROR) return BNO085_ERROR;
      HAL_Delay(5);        /* empty - hub still booting */
    }
  }
  if (!reset_seen) return BNO085_TIMEOUT;  /* hub up but never confirmed */

  bno085_flush(100);

  if (bno085_configure() != BNO085_OK) return BNO085_ERROR;

  /* Fusion needs a beat to spin up; confirm reports actually arrive. */
  BNO085_State probe = {0};
  start = HAL_GetTick();
  while (HAL_GetTick() - start < 1500) {
    if (BNO085_Poll(&probe) == BNO085_OK && probe.rv_count > 0) {
      return BNO085_OK;
    }
    HAL_Delay(10);
  }
  return BNO085_TIMEOUT;
}

uint8_t BNO085_Poll(BNO085_State *st)
{
  uint8_t buf[BNO085_MAX_PACKET], ch, pl;
  uint8_t result = BNO085_OK;
  /* Bounded loop: hub queues at most a couple dozen packets at our poll
   * rate; 32 iterations is generous headroom without risking a hang. */
  for (unsigned i = 0; i < 32; i++) {
    uint8_t r = bno085_read_packet(&ch, buf, sizeof(buf), &pl);
    if (r == BNO085_TIMEOUT) break;
    if (r == BNO085_ERROR)   return BNO085_ERROR;
    if (ch == CH_INPUT_REPORTS || ch == CH_WAKE_REPORTS || ch == CH_GYRO_RV) {
      bno085_parse_cargo(st, buf, pl);
    }
  }
  return result;
}

uint8_t BNO085_IsAlive(void)
{
  return (HAL_I2C_IsDeviceReady(s_hi2c, s_addr8, 1,
                                BNO085_I2C_TIMEOUT_MS) == HAL_OK)
         ? BNO085_OK : BNO085_ERROR;
}

uint8_t BNO085_Recover(void)
{
  if (BNO085_IsAlive() != BNO085_OK) return BNO085_ERROR;
  return BNO085_Init(s_hi2c, (uint8_t)(s_addr8 >> 1));
}
