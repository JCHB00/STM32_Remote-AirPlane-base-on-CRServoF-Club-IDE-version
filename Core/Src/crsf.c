#include "crsf.h"
#include "app.h"
#include "main.h"

#define FRAME_HDR 0xC8      /* 飞控地址;只处理发往它的帧 [cite 同原库] */
#define TYPE_CH   0x16      /* RC_CHANNELS_PACKED */

static int32_t  s_ch[16];
static uint32_t s_lastMs;
static bool     s_up;

static uint8_t crc8(const uint8_t *p, uint8_t len) {  /* DVB-S2,poly 0xD5,初值0 */
    uint8_t crc = 0;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* 11bit×16 解包 + 191→1000µs / 1792→2000µs(与原库 map 一致) */
static void unpack(const uint8_t *p) {
    uint32_t scratch = 0; int bits = 0;
    for (int c = 0; c < 16; c++) {
        while (bits < 11) { scratch |= (uint32_t)(*p++) << bits; bits += 8; }
        uint32_t raw = scratch & 0x7FF; scratch >>= 11; bits -= 11;
        s_ch[c] = (int32_t)(raw - 191) * 1000 / 1601 + 1000;
    }
}

static uint8_t s_buf[64], s_pos, s_len;   /* 状态机 */
static bool s_active;

void crsf_rx_byte(uint8_t b) {            /* 主循环里逐字节喂进来 */
    if (!s_active) {
        if (b == FRAME_HDR) { s_buf[0] = b; s_pos = 1; s_active = true; }
        return;
    }
    if (s_pos >= sizeof(s_buf)) { s_active = false; return; }
    s_buf[s_pos++] = b;
    if (s_pos == 2) {                     /* 长度字节 */
        s_len = b;
        if (s_len < 3 || s_len > 62) s_active = false;   /* 非法 → 重新找帧头 */
        return;
    }
    if (s_pos == (uint8_t)(s_len + 2)) {  /* 收齐 [0xC8][len][type][payload][crc] */
        if (crc8(&s_buf[2], s_len - 1) == s_buf[s_len + 1] &&
            s_buf[2] == TYPE_CH) {
            unpack(&s_buf[3]);
            s_lastMs = HAL_GetTick();
            if (!s_up) { s_up = true; led_set(true); }
            packet_channels();            /* → 立即写 8 路 PWM */
        }
        s_active = false;
    }
}

void crsf_check_link(void) {              /* 主循环里周期调用 */
    if (s_up && (HAL_GetTick() - s_lastMs > 300)) {   /* 原版超时 = 300ms [cite] */
        s_up = false;
        led_set(false);
        failsafe_apply();                 /* 触发一次失控保护 */
    }
}

int32_t crsf_get_channel(uint8_t ch1) { return s_ch[ch1 - 1]; }  /**/


void crsf_init(void)
{
  s_pos    = 0;
  s_len    = 0;
  s_active = false;   /* 状态机回到"等帧头 0xC8" */
  s_up     = false;   /* 还没收到过通道帧(链路视为未建立) */
  s_lastMs = 0;
  for (int i = 0; i < 16; ++i)
    s_ch[i] = 1500;   /* 防御性默认值:中位;实际输出仍要等第一帧数据才会启动 */
}
/* ---- 调试用:链路是否在线 ---- */
bool crsf_is_link_up(void)
{
  return s_up;
}
