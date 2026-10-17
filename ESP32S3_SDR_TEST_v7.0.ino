

// ======================================================================================

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"

#if __has_include("esp_rom_crc.h")
  #include "esp_rom_crc.h"
#elif __has_include("rom/crc.h")
  #include "rom/crc.h"
#else
extern "C" uint32_t crc32_le(uint32_t crc, const uint8_t *buf, uint32_t len);
#define esp_rom_crc32_le crc32_le
#endif

/* Forward types */
enum ReceiverMode : uint8_t {
  MODE_IDLE = 0,
  MODE_STREAM = 1,
  MODE_SWEEP = 2,
  MODE_DETECT = 3,
  MODE_IQSTREAM = 4
};

struct CommandMsg {
  ReceiverMode mode;
  uint32_t f_start_mhz, f_stop_mhz, step_mhz;
  uint16_t fft_size;
  uint8_t fps;
};

struct RiceChoice {
  unsigned k;
  unsigned bits;
};

/* IPv6 hook workaround */
extern "C" __attribute__((weak))
int probe_ip6_hook(void *p, void *inp) __asm__("lwip_hook_ip6_input");
extern "C"
int probe_ip6_hook(void *p, void *inp) { (void)p; (void)inp; return 0; }

#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "dsps_fft2r.h"
#include "dsps_wind.h"

#if __has_include("heap_memory_layout.h")
  #include "heap_memory_layout.h"
#endif

#ifndef SOC_RESERVE_MEMORY_REGION
typedef struct { intptr_t start, end; } probe_region_t;
#define SOC_RESERVE_MEMORY_REGION(S, E, N) \
  __attribute__((section(".reserved_memory_address"), used)) \
  static probe_region_t probe_res_##N = { S, E };
#endif

#ifndef CONFIG_IDF_TARGET_ESP32S3
  #error "Only ESP32-S3"
#endif
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif




#define AUTO_DETECT      0      // 0 = обычный режим с ПК
#define AUTO_DET_FREQ    2440
#define AUTO_DET_FFT     2048
#define AUTO_DET_FPS     10



#define HB_N 128                      // бинов карты на кадр
static float   hb_floor[HB_N];        // пол каждого бина
static bool    hb_rdy[HB_N];
static uint8_t hb_hit[HB_N];          // частота срабатываний
static uint8_t hb_marg[HB_N];         // макс. запас над полом, дБ

static uint32_t hb_decay_ms = 0;
static float   hb_simul = 0;          // бинов за кадр (сглажено)
static uint32_t spectrum_tx_errors = 0;



// ======================================================================================
// WS2812 status LED
// ======================================================================================
#define RGB_LED_PIN 21
#define WS_BIT (1UL << RGB_LED_PIN)

static bool led_auto = true;
static bool hb_state = false;
static unsigned long hb_last = 0;

static void IRAM_ATTR ws2812_send_byte(uint8_t data, uint32_t mhz)
{
  for (int bit = 7; bit >= 0; --bit) {
    bool one = ((data >> bit) & 1) != 0;
    uint32_t start = ESP.getCycleCount();
    REG_WRITE(GPIO_OUT_W1TS_REG, WS_BIT);
    uint32_t wh = one ? (mhz * 700 / 1000) : (mhz * 350 / 1000);
    if (wh > 25) wh -= 15;
    while ((uint32_t)(ESP.getCycleCount() - start) < wh) {}
    REG_WRITE(GPIO_OUT_W1TC_REG, WS_BIT);
    uint32_t total = mhz * 1250 / 1000;
    while ((uint32_t)(ESP.getCycleCount() - start) < total) {}
  }
}

static void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t mhz = ESP.getCpuFreqMHz();
  if (mhz < 80) mhz = 240;
  noInterrupts();
  ws2812_send_byte(g, mhz);
  ws2812_send_byte(r, mhz);
  ws2812_send_byte(b, mhz);
  interrupts();
  delayMicroseconds(80);
}
static void rgb_off() { rgb_set(0, 0, 0); }
static void rgb_blink(uint8_t r, uint8_t g, uint8_t b, int n, int on_ms, int off_ms)
{
  for (int i = 0; i < n; ++i) {
    rgb_set(r, g, b); delay(on_ms); rgb_off();
    if (i + 1 < n) delay(off_ms);
  }
}

// ======================================================================================
// Radio registers
// ======================================================================================
#define RES_START 0x3FCD0000u
#define RES_END   0x3FCE0000u
#define IQ_ADDR   0x3FCD0000u
SOC_RESERVE_MEMORY_REGION(RES_START, RES_END, probe_rf_dump);

#define DUMP_CTRL       0x60033D5Cu
#define DUMP_PACK       0x60033D90u
#define SRAM_OWNER_REG  0x600C101Cu
#define GAIN_REG        0x6001C02Cu
#define FE_WIDTH_REG    0x60006100u
#define BB_ENABLE_REG   0x6002600Cu
#define PBUS_CTRL_REG   0x60006104u
#define PBUS_MODE_REG   0x6000610Cu
#define PBUS_STATUS_REG 0x60006110u
#define PBUS_BB_GAIN_REG 0x60006118u
#define PBUS_RF_GAIN_REG 0x6000611Cu
#define IQ_CORRECTION_REG 0x6000607Cu
#define RFPLL_OWNER_REG 0x6000E0C4u
#define AGC_CTRL_REG    0x6001C01Cu
#define AGC_DISABLE_REG 0x6001C034u
#define AGC_RX_FORCE_REG 0x6001C080u

#define BIT1(n) (1u << (n))

#define I2C_RFPLL     0x62
#define I2C_SDM       0x63
#define I2C_CKGEN     0x65
#define I2C_BB_FILTER 0x67
#define BB_I2C_HOST   0    // FW1: обратно как в v9

#define CKGEN_5_6_BIT 0x10
#define IQ_FIELDS 0x1FFF0000u
#define IQ_MANUAL 0x08000000u

#define ESP32S3_PLL_MIN_HZ    2210000000u
#define ESP32S3_PLL_MAX_HZ    2790000000u
#define ESP32S3_LO_MIN_HZ     1841666667u
#define ESP32S3_LO_5_6_MAX_HZ 2325000000u

#define MAX_SAMPLES    16380u
#define MAX_FFT        4096u
#define MAX_SWEEP_BINS 16384u
#define CAP_SLOTS      60u
#define CAP_BASE_HZ    2210000000u

static const uint32_t FILL          = 0xA5A0055Au;
static const uint32_t TAIL_SENTINEL = 0x5A5AA5A5u;
static uint32_t *const IQ = (uint32_t *)IQ_ADDR;

#if __has_include("esp_private/regi2c_ctrl.h")
  #include "esp_private/regi2c_ctrl.h"
  #define S3_I2C_READ(b, h, r)  regi2c_ctrl_read_reg((uint8_t)(b), (uint8_t)(h), (uint8_t)(r))
  #define S3_I2C_WRITE(b, h, r, v) regi2c_ctrl_write_reg((uint8_t)(b), (uint8_t)(h), (uint8_t)(r), (uint8_t)(v))
#else
extern "C" {
  unsigned rom_chip_i2c_readReg(unsigned, unsigned, unsigned);
  void rom_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
}
  #define S3_I2C_READ(b, h, r)  rom_chip_i2c_readReg((unsigned)(b), (unsigned)(h), (unsigned)(r))
  #define S3_I2C_WRITE(b, h, r, v) rom_chip_i2c_writeReg((unsigned)(b), (unsigned)(h), (unsigned)(r), (unsigned)(v))
#endif

extern "C" {
  void rom_pbus_workmode(void);
  void rom_pbus_xpd_tx_off(void);
  void rom_pbus_xpd_rx_on(unsigned);
  void rom_set_rxclk_en(unsigned);
  void stop_tx_tone(unsigned);
  void set_chanfreq(unsigned mhz, unsigned mode);
  unsigned rom_pbus_rd(unsigned block, unsigned index);
}

// ======================================================================================
// Global state
// ======================================================================================
static bool reserve_ok = false;
static esp_err_t wifi_err = ESP_OK;

static unsigned gain_max = 0, gain_code = 60;
static bool hw_agc = false;

static unsigned settings_width = 40;
static int rx_filt0 = -1, rx_filt1 = -1, rx_filter = -1;

// неиспользуемые после FW1, оставлены чтобы не трогать setup
static unsigned rx_filter_boot_20[2] = {0, 0};
static unsigned rx_filter_boot_40[2] = {0, 0};
static bool rx_filter_manual_written[2] = {false, false};

static unsigned sample_rate_mhz = 80;
static uint32_t rate_bits = 0;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t iqLock = NULL;
static SemaphoreHandle_t serialMux = NULL;
static QueueHandle_t cmdQueue = NULL;
static TaskHandle_t dspTaskHandle = NULL;

static volatile ReceiverMode currentMode = MODE_IDLE;
static CommandMsg requested = {MODE_IDLE, 2440, 2440, 60, 1024, 10};

static int manual_rf = -1, manual_bb = -1, manual_iq = -1;
static int manual_dc[4] = {-1, -1, -1, -1};
static unsigned cur_rf = 0, cur_bb = 0, cur_dc[4] = {0, 0, 0, 0};
static unsigned cur_iq_a = 0, cur_iq_p = 0;

static bool pbus_owned = false;
static uint32_t saved_pbus_ctrl = 0, saved_pbus_mode = 0, saved_iq_reg = 0;
static unsigned saved_dc_hw[4] = {0, 0, 0, 0};
static unsigned saved_rf_hw = 0, saved_bb_hw = 0; // unused после FW2

static uint32_t agc_ctrl_saved = 0, bb_enable_saved = 0;
static bool agc_regs_saved = false; // unused после FW2

static uint32_t g_lo_hz = 2412000000u, g_pll_hz = 2412000000u, g_sdm = 0;
static uint8_t g_lo_mode = 1;
static unsigned g_pll_cap = 0, g_pll_first = 0, g_pll_len = 0;
static unsigned last_mhz = 2412;
static uint32_t last_good_lo = 2412000000u;
static uint8_t last_good_mode = 1;

struct CapCache { uint32_t pll_hz; int16_t cap; };
static CapCache cap_cache[CAP_SLOTS];

static const uint8_t dc_block[4] = {3, 3, 2, 2};
static const uint8_t dc_index[4] = {1, 2, 1, 2};

static const struct { unsigned mhz, code; } bw_cal[] = {
  {69, 0}, {51, 4}, {45, 8}, {33, 16}, {25, 24}, {21, 32}, {16, 48}, {13, 60}
};
#define BW_CAL_N (sizeof(bw_cal) / sizeof(bw_cal[0]))




static unsigned fhss_idx = 0, fhss_cnt = 0;
static uint32_t fhss_last_report_ms = 0;
static float det_thr_db = 12.0f;
static uint32_t det_next_id = 1, det_frame_no = 0;
static unsigned det_hist[241];
static bool conv_en = false;
static uint32_t conv_lo_mhz = 0;
static int conv_sign = 1;

// ======================================================================================
// Helpers
// ======================================================================================
static unsigned bw_to_code(unsigned mhz)
{
  if (mhz >= 69) return 0;
  if (mhz <= 13) return 60;
  for (unsigned i = 1; i < BW_CAL_N; ++i)
    if (mhz >= bw_cal[i].mhz) {
      unsigned s = bw_cal[i-1].mhz - bw_cal[i].mhz;
      return bw_cal[i-1].code + ((bw_cal[i].code - bw_cal[i-1].code) * (bw_cal[i-1].mhz - mhz) + s/2) / s;
    }
  return 60;
}
static void sync_rx_filter_mirror() { rx_filter = (rx_filt0 == rx_filt1) ? rx_filt0 : -2; }
static void set_filt_both(int v) { rx_filt0 = rx_filt1 = rx_filter = v; }

static void set_width_hw(unsigned mhz)
{
  bool w = (mhz == 40);
  REG_WRITE(FE_WIDTH_REG, (REG_READ(FE_WIDTH_REG) & ~0x003F0000u) | (w ? 0x00120000u : 0));
  REG_WRITE(BB_ENABLE_REG, (REG_READ(BB_ENABLE_REG) & ~0xCu) | (w ? 0x4u : 0));
}
static void apply_width_setting()
{
  REG_WRITE(BB_ENABLE_REG, REG_READ(BB_ENABLE_REG) & ~2u);
  set_width_hw(settings_width);
  delayMicroseconds(2);
  REG_WRITE(BB_ENABLE_REG, REG_READ(BB_ENABLE_REG) | 2u);
}

// FW1: v9 фильтр, host 0
static void rx_filter_apply()
{
  unsigned base = (settings_width == 40) ? 6u : 4u;
  if (rx_filt0 >= 0) {
    unsigned s = S3_I2C_READ(I2C_BB_FILTER, 0, base);
    S3_I2C_WRITE(I2C_BB_FILTER, 0, base, (s & ~63u) | (unsigned)rx_filt0);
  }
  if (rx_filt1 >= 0) {
    unsigned s = S3_I2C_READ(I2C_BB_FILTER, 0, base + 1);
    S3_I2C_WRITE(I2C_BB_FILTER, 0, base + 1, (s & ~63u) | (unsigned)rx_filt1);
  }
}

// ======================================================================================
// PBUS
// ======================================================================================
static bool pbus_write(unsigned blk, unsigned idx, unsigned val)
{
  uint32_t f = ((val & 511u) << 6) | ((blk & 15u) << 2) | ((idx & 3u) << 15);
  REG_WRITE(PBUS_CTRL_REG, (REG_READ(PBUS_CTRL_REG) & 0xFFFE0001u) | (f & 0x1FFFCu) | 2u);
  uint32_t st = ESP.getCycleCount();
  while (REG_READ(PBUS_STATUS_REG) & 0x80000000u) {
    if ((uint32_t)(ESP.getCycleCount() - st) > 24000u) {
      REG_WRITE(PBUS_CTRL_REG, REG_READ(PBUS_CTRL_REG) & ~2u);
      return false;
    }
  }
  REG_WRITE(PBUS_CTRL_REG, REG_READ(PBUS_CTRL_REG) & ~2u);
  return true;
}

static void update_cur_stages()
{
  cur_rf = (REG_READ(PBUS_RF_GAIN_REG) >> 18) & 511u;
  cur_bb = (REG_READ(PBUS_BB_GAIN_REG) >> 9) & 511u;
  uint32_t iq = REG_READ(IQ_CORRECTION_REG);
  cur_iq_a = (iq >> 16) & 31u;
  cur_iq_p = (iq >> 21) & 63u;
  for (unsigned r = 0; r < 4; ++r)
    cur_dc[r] = rom_pbus_rd(dc_block[r], dc_index[r]) & 511u;
}

static void pbus_release_all()
{
  if (!pbus_owned) return;
  for (unsigned r = 0; r < 4; ++r) {
    if (manual_dc[r] >= 0) pbus_write(dc_block[r], dc_index[r], saved_dc_hw[r]);
  }
  pbus_write(0, 1, 0); pbus_write(1, 1, 0); pbus_write(1, 2, 0);
  REG_WRITE(PBUS_CTRL_REG, saved_pbus_ctrl);
  REG_WRITE(PBUS_MODE_REG, saved_pbus_mode);
  REG_WRITE(IQ_CORRECTION_REG, saved_iq_reg);
  pbus_owned = false;
}

static bool apply_manual_stages(bool force_ownership)
{
  (void)force_ownership; // FW2: игнор, поведение v9
  update_cur_stages();
  bool need = (manual_rf >= 0 || manual_bb >= 0 || manual_iq >= 0 ||
               manual_dc[0] >= 0 || manual_dc[1] >= 0 || manual_dc[2] >= 0 || manual_dc[3] >= 0);
  if (!need) { pbus_release_all(); return true; }
  if (!pbus_owned) {
    saved_pbus_ctrl = REG_READ(PBUS_CTRL_REG);
    saved_pbus_mode = REG_READ(PBUS_MODE_REG);
    saved_iq_reg = REG_READ(IQ_CORRECTION_REG);
    for (unsigned r = 0; r < 4; ++r) saved_dc_hw[r] = cur_dc[r];
    REG_WRITE(PBUS_MODE_REG, REG_READ(PBUS_MODE_REG) & ~0x08000000u);
    REG_WRITE(PBUS_CTRL_REG, REG_READ(PBUS_CTRL_REG) | 1u);
    pbus_owned = true;
  }
  bool ok = true;
  if (manual_rf >= 0) { ok &= pbus_write(1, 2, (unsigned)manual_rf); cur_rf = (unsigned)manual_rf; }
  if (manual_bb >= 0) { unsigned v = 0x180 | ((unsigned)manual_bb & 127u); ok &= pbus_write(0, 1, v); cur_bb = v; }
  for (unsigned r = 0; r < 4; ++r) {
    if (manual_dc[r] >= 0) { ok &= pbus_write(dc_block[r], dc_index[r], (unsigned)manual_dc[r]); cur_dc[r] = (unsigned)manual_dc[r]; }
  }
  if (manual_iq >= 0) {
    uint32_t v = (saved_iq_reg & ~IQ_FIELDS) | IQ_MANUAL | (((uint32_t)manual_iq & 31u) << 16) | ((((uint32_t)manual_iq >> 8) & 63u) << 21);
    REG_WRITE(IQ_CORRECTION_REG, v);
    cur_iq_a = (manual_iq & 31u);
    cur_iq_p = ((manual_iq >> 8) & 63u);
  }
  return ok;
}

// ======================================================================================
// WiFi / PHY
// ======================================================================================
static bool check_reserved()
{
  void *head = NULL; bool ok = true;
  for (int i = 0; i < 64; ++i) {
    size_t sz = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (sz < 64) break;
    void **p = (void **)heap_caps_malloc(sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!p) break;
    *p = head; head = p;
    uintptr_t a = (uintptr_t)p;
    if (a < RES_END && a + sz > RES_START) ok = false;
  }
  while (head) { void *nx = *(void**)head; heap_caps_free(head); head = nx; }
  return ok;
}

static esp_err_t wifi_bringup()
{
  esp_err_t e = nvs_flash_init();
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); e = nvs_flash_init(); }
  if (e) return e;
  e = esp_netif_init(); if (e && e != ESP_ERR_INVALID_STATE) return e;
  e = esp_event_loop_create_default(); if (e && e != ESP_ERR_INVALID_STATE) return e;
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  e = esp_wifi_init(&cfg); if (e) return e;
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_start();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  return ESP_OK;
}

static void prepare_rx()
{
  stop_tx_tone(1);
  rom_pbus_workmode();
  rom_pbus_xpd_tx_off();
  rom_pbus_xpd_rx_on(1);
  rom_set_rxclk_en(1);
}
static void read_gain_max()
{
  unsigned m = (REG_READ(GAIN_REG) >> 8) & 127u;
  gain_max = (m <= 82u) ? m : 0u;
}

// ======================================================================================
// PLL — FW2: v9 1:1 без rollback
// ======================================================================================
static bool plan_lo(uint32_t hz, uint32_t *sdm, uint32_t *pll, uint32_t *loe, uint8_t *mode)
{
  uint8_t m; uint32_t step;
  if (hz < ESP32S3_PLL_MIN_HZ) {
    m = 2; step = 25000000u;
    if (hz < ESP32S3_LO_MIN_HZ || hz > ESP32S3_LO_5_6_MAX_HZ) return false;
  } else {
    m = 1; step = 30000000u;
    if (hz > ESP32S3_PLL_MAX_HZ) return false;
  }
  uint64_t sc = ((uint64_t)hz * 65536u + step / 2u) / step;
  *sdm = (uint32_t)(sc - 32u * 65536u);
  *pll = (uint32_t)((30000000ull * sc + 32768u) / 65536u);
  *loe = (uint32_t)(((uint64_t)step * sc + 32768u) / 65536u);
  *mode = m;
  return true;
}
static void set_pll_cap(unsigned c)
{
  S3_I2C_WRITE(I2C_RFPLL, 1, 1, c & 0xFFu);
  uint8_t h = S3_I2C_READ(I2C_RFPLL, 1, 2);
  S3_I2C_WRITE(I2C_RFPLL, 1, 2, (h & ~0x10u) | ((c >> 8) << 4));
}
static void set_pll_manual(bool m)
{
  uint8_t v = S3_I2C_READ(I2C_RFPLL, 1, 11);
  S3_I2C_WRITE(I2C_RFPLL, 1, 11, m ? (v | 0x40u) : (v & ~0x40u));
}
static bool scan_caps(unsigned f, unsigned l, unsigned *bs, unsigned *bl)
{
  unsigned rs = 0, rl = 0, bbs = 0, bbl = 0;
  for (unsigned c = f; c <= l; ++c) {
    set_pll_cap(c); delayMicroseconds(20);
    bool lk = ((S3_I2C_READ(I2C_RFPLL, 1, 12) >> 2) & 3u) == 0;
    if (!lk) { rl = 0; continue; }
    if (rl++ == 0) rs = c;
    if (rl > bbl) { bbs = rs; bbl = rl; }
  }
  *bs = bbs; *bl = bbl;
  return bbl != 0;
}
static int predict_cap(uint32_t phz)
{
  int bi = -1; uint32_t bd = 0xFFFFFFFFu;
  for (unsigned i = 0; i < CAP_SLOTS; ++i)
    if (cap_cache[i].pll_hz) {
      uint32_t d = (cap_cache[i].pll_hz > phz) ? cap_cache[i].pll_hz - phz : phz - cap_cache[i].pll_hz;
      if (d < bd) { bd = d; bi = (int)i; }
    }
  if (bi < 0 || bd > 100000000u) return -1;
  double dm = ((double)((int64_t)phz - (int64_t)cap_cache[bi].pll_hz)) / 1e6;
  return cap_cache[bi].cap + (int)lround(-0.73 * dm);
}

static bool tune_pll_direct(uint32_t hz_req)
{
  uint32_t sdm, pll, lo_eff; uint8_t mode;
  if (!plan_lo(hz_req, &sdm, &pll, &lo_eff, &mode)) return false;
  g_sdm = sdm; g_pll_hz = pll; g_lo_hz = lo_eff; g_lo_mode = mode;
  S3_I2C_WRITE(I2C_CKGEN, 1, 0, S3_I2C_READ(I2C_CKGEN, 1, 0) & ~CKGEN_5_6_BIT);
  REG_WRITE(RFPLL_OWNER_REG, REG_READ(RFPLL_OWNER_REG) | (1u << 25));
  set_pll_manual(false);
  S3_I2C_WRITE(I2C_SDM, 1, 0, 0x07);
  S3_I2C_WRITE(I2C_SDM, 1, 3, (sdm >> 16) & 0xFF);
  S3_I2C_WRITE(I2C_SDM, 1, 4, (sdm >> 8) & 0xFF);
  S3_I2C_WRITE(I2C_SDM, 1, 5, sdm & 0xFF);
  S3_I2C_WRITE(I2C_SDM, 1, 0, 0x17);
  S3_I2C_WRITE(I2C_RFPLL, 1, 0, S3_I2C_READ(I2C_RFPLL, 1, 0) | 0x40);
  S3_I2C_WRITE(I2C_RFPLL, 1, 0, S3_I2C_READ(I2C_RFPLL, 1, 0) & ~0x20);
  S3_I2C_WRITE(I2C_RFPLL, 1, 0, S3_I2C_READ(I2C_RFPLL, 1, 0) | 0x20);
  S3_I2C_WRITE(I2C_RFPLL, 1, 0, S3_I2C_READ(I2C_RFPLL, 1, 0) & ~0x40);
  bool cal = false;
  for (unsigned i = 0; i < 100 && !cal; i++) {
    delayMicroseconds(20);
    cal = (S3_I2C_READ(I2C_RFPLL, 1, 7) & 2) != 0;
  }
  if (!cal) return false;
  delayMicroseconds(5);
  uint8_t sl = S3_I2C_READ(I2C_RFPLL, 1, 1), sh = S3_I2C_READ(I2C_RFPLL, 1, 2), sm = S3_I2C_READ(I2C_RFPLL, 1, 11);
  set_pll_manual(true);
  unsigned bs = 0, bl = 0; bool found = false;
  int pred = predict_cap(pll);
  if (pred >= 0) {
    unsigned f = (pred > 24) ? pred - 24 : 0, l = (pred + 24 < 511) ? pred + 24 : 511;
    if (scan_caps(f, l, &bs, &bl)) {
      bool cutL = (bs == f && f != 0), cutR = (bs + bl - 1 == l && l != 511);
      if (!cutL && !cutR) found = true;
    }
  }
  if (!found) found = scan_caps(0, 511, &bs, &bl);
  S3_I2C_WRITE(I2C_RFPLL, 1, 1, sl);
  S3_I2C_WRITE(I2C_RFPLL, 1, 2, (S3_I2C_READ(I2C_RFPLL, 1, 2) & ~0x10) | (sh & 0x10));
  S3_I2C_WRITE(I2C_RFPLL, 1, 11, (S3_I2C_READ(I2C_RFPLL, 1, 11) & ~0x40) | (sm & 0x40));
  if (!found) return false;
  g_pll_first = bs; g_pll_len = bl; g_pll_cap = bs + (bl - 1) / 2;
  set_pll_cap(g_pll_cap); set_pll_manual(true);
  if (pll >= CAP_BASE_HZ) {
    unsigned slot = (pll - CAP_BASE_HZ) / 10000000u;
    if (slot < CAP_SLOTS) { cap_cache[slot].pll_hz = pll; cap_cache[slot].cap = (int16_t)g_pll_cap; }
  }
  return true;
}
static bool tune_rx_new(unsigned mhz) {
  uint32_t hz = mhz * 1000000u;
  if (hz >= ESP32S3_LO_MIN_HZ && hz <= ESP32S3_PLL_MAX_HZ) {
    if (!tune_pll_direct(hz)) return false;
    uint8_t ck = S3_I2C_READ(I2C_CKGEN, 1, 0);
    if (g_lo_mode == 2) ck |= CKGEN_5_6_BIT; else ck &= ~CKGEN_5_6_BIT;
    S3_I2C_WRITE(I2C_CKGEN, 1, 0, ck);
    delayMicroseconds(3000);
    bool is56 = (S3_I2C_READ(I2C_CKGEN, 1, 0) & CKGEN_5_6_BIT) != 0;
    if (is56 != (g_lo_mode == 2)) return false;
    return true;
  }
  return false;
}
static bool tune_rx_hz(uint32_t hz) { return tune_pll_direct(hz); }

// ======================================================================================
// RX configuration — FW2:
// ======================================================================================
static void park_receiver()
{
  REG_WRITE(DUMP_CTRL, 0);
  REG_WRITE(SRAM_OWNER_REG, REG_READ(SRAM_OWNER_REG) & ~15u);
  delayMicroseconds(20);
}
static void full_park_dump()
{
  park_receiver();
  REG_WRITE(AGC_RX_FORCE_REG, REG_READ(AGC_RX_FORCE_REG) & ~0xC1u);
  REG_WRITE(PBUS_STATUS_REG, REG_READ(PBUS_STATUS_REG) & ~0xCF00u);
  REG_WRITE(BB_ENABLE_REG, REG_READ(BB_ENABLE_REG) & ~2u);
  delayMicroseconds(5);
}

static bool configure_rx_path()
{
  if (hw_agc) {
    pbus_release_all();
    apply_width_setting();
    REG_WRITE(GAIN_REG, REG_READ(GAIN_REG) & ~0x00800000u);
    REG_WRITE(AGC_DISABLE_REG, REG_READ(AGC_DISABLE_REG) & ~0x80u);
    REG_WRITE(AGC_RX_FORCE_REG, REG_READ(AGC_RX_FORCE_REG) & ~0x01u);
    REG_WRITE(PBUS_STATUS_REG, REG_READ(PBUS_STATUS_REG) & ~0xC000u);
    rx_filter_apply();
    update_cur_stages();
    REG_WRITE(DUMP_PACK, (1u << 6) | (2u << 12) | (3u << 18));
    delayMicroseconds(100);
    REG_WRITE(DUMP_CTRL, 0);
    return true;
  }
  set_width_hw(settings_width);
  REG_WRITE(BB_ENABLE_REG, REG_READ(BB_ENABLE_REG) | 0x10000000u);
  REG_WRITE(BB_ENABLE_REG, REG_READ(BB_ENABLE_REG) & ~2u);
  delayMicroseconds(1);
  REG_WRITE(BB_ENABLE_REG, REG_READ(BB_ENABLE_REG) | 2u);
  REG_WRITE(AGC_CTRL_REG, (REG_READ(AGC_CTRL_REG) & 0xFF00FFFFu) | 0x007F0000u);
  REG_WRITE(AGC_DISABLE_REG, REG_READ(AGC_DISABLE_REG) | 0x80u);
  REG_WRITE(AGC_RX_FORCE_REG, REG_READ(AGC_RX_FORCE_REG) | 1u);
  if (gain_code > gain_max) gain_code = gain_max;
  REG_WRITE(GAIN_REG, (REG_READ(GAIN_REG) & 0x007FFFFFu) | ((uint32_t)gain_code << 24) | 0x00800000u);
  REG_WRITE(PBUS_STATUS_REG, REG_READ(PBUS_STATUS_REG) | 0xC000u);
  rx_filter_apply();
  delayMicroseconds(100);
  unsigned bb_hw = (REG_READ(PBUS_BB_GAIN_REG) >> 9) & 511u;
  unsigned rf_hw = (REG_READ(PBUS_RF_GAIN_REG) >> 18) & 511u;
  unsigned rf_use = (manual_rf >= 0) ? (unsigned)manual_rf : rf_hw;
  unsigned bb_use = (manual_bb >= 0) ? (0x180 | ((unsigned)manual_bb & 127u)) : bb_hw;
  if (!pbus_owned) {
    saved_pbus_ctrl = REG_READ(PBUS_CTRL_REG);
    saved_pbus_mode = REG_READ(PBUS_MODE_REG);
    saved_iq_reg = REG_READ(IQ_CORRECTION_REG);
    for (int r = 0; r < 4; r++) saved_dc_hw[r] = rom_pbus_rd(dc_block[r], dc_index[r]) & 511u;
    REG_WRITE(PBUS_MODE_REG, REG_READ(PBUS_MODE_REG) & ~0x08000000u);
    REG_WRITE(PBUS_CTRL_REG, REG_READ(PBUS_CTRL_REG) | 1u);
    pbus_owned = true;
  }
  REG_WRITE(BB_ENABLE_REG, REG_READ(BB_ENABLE_REG) & ~2u);
  bool ok = pbus_write(4, 1, 0) && pbus_write(5, 1, 0) && pbus_write(0, 1, 0x184) &&
            pbus_write(1, 1, 0x189) && pbus_write(1, 2, rf_use) && pbus_write(0, 1, bb_use);
  cur_rf = rf_use; cur_bb = bb_use;
  for (int r = 0; r < 4 && ok; r++) {
    unsigned hw = rom_pbus_rd(dc_block[r], dc_index[r]) & 511u;
    if (manual_dc[r] >= 0) {
      ok &= pbus_write(dc_block[r], dc_index[r], (unsigned)manual_dc[r]);
      cur_dc[r] = (unsigned)manual_dc[r];
    } else cur_dc[r] = hw;
  }
  if (manual_iq >= 0) {
    uint32_t v = (saved_iq_reg & ~IQ_FIELDS) | IQ_MANUAL | (((uint32_t)manual_iq & 31u) << 16) | ((((uint32_t)manual_iq >> 8) & 63u) << 21);
    REG_WRITE(IQ_CORRECTION_REG, v);
    cur_iq_a = (manual_iq & 31u);
    cur_iq_p = ((manual_iq >> 8) & 63u);
  } else {
    uint32_t iq = REG_READ(IQ_CORRECTION_REG);
    cur_iq_a = (iq >> 16) & 31u;
    cur_iq_p = (iq >> 21) & 63u;
  }
  if (!ok) return false;
  REG_WRITE(DUMP_PACK, (1u << 6) | (2u << 12) | (3u << 18));
  delayMicroseconds(100);
  REG_WRITE(DUMP_CTRL, 0);
  return true;
}

static bool reconfig_current_no_pll()
{
  if (wifi_err) return false;
  pbus_release_all();
  full_park_dump();
  prepare_rx();
  read_gain_max();
  if (gain_code > gain_max) gain_code = gain_max;
  return configure_rx_path();
}
static bool retune(unsigned mhz)
{
  if (wifi_err) return false;
  pbus_release_all();
  full_park_dump();
  if (!tune_rx_new(mhz)) return false;
  last_mhz = mhz;
  prepare_rx();
  read_gain_max();
  if (gain_code > gain_max) gain_code = gain_max;
  return configure_rx_path();
}

// ======================================================================================
// Snapshot (v10: iqLock + короткий critical)
// ======================================================================================
static const char *snapshot(unsigned n, uint32_t *capture_us)
{
  if (!reserve_ok) return "ERR reserve";
  if (wifi_err)    return "ERR wifi";
  if (!iqLock)     return "ERR lock";
  if (n < 256 || n > MAX_SAMPLES) return "ERR args";

  xSemaphoreTake(iqLock, portMAX_DELAY);
  const char *error = NULL;

  for (unsigned j = 0; j < n; ++j) IQ[j] = FILL;
  for (unsigned j = 0; j < 4; ++j) IQ[n + j] = TAIL_SENTINEL ^ j;

  rx_filter_apply();

  uint32_t owner;
  portENTER_CRITICAL(&mux);
  owner = REG_READ(SRAM_OWNER_REG);
  REG_WRITE(DUMP_CTRL, 0);
  REG_WRITE(DUMP_PACK, (1u << 6) | (2u << 12) | (3u << 18));
  REG_WRITE(SRAM_OWNER_REG, (owner & ~15u) | 4u);
  __asm__ volatile("memw" ::: "memory");
  uint32_t ctrl = 0x80000000u | rate_bits | n;
  REG_WRITE(DUMP_CTRL, ctrl);
  REG_WRITE(DUMP_CTRL, ctrl | BIT1(19));
  REG_WRITE(DUMP_CTRL, ctrl);
  portEXIT_CRITICAL(&mux);

  int64_t t0 = esp_timer_get_time();
  while (!(REG_READ(DUMP_CTRL) & BIT1(18))) {
    if (esp_timer_get_time() - t0 >= 20000) { error = "ERR capture_timeout"; break; }
  }
  uint32_t el = (uint32_t)(esp_timer_get_time() - t0);
  if (capture_us) *capture_us = el;

  portENTER_CRITICAL(&mux);
  REG_WRITE(DUMP_CTRL, 0);
  REG_WRITE(SRAM_OWNER_REG, owner);
  __asm__ volatile("memw" ::: "memory");
  portEXIT_CRITICAL(&mux);

  if (!error) for (unsigned j = 0; j < n; ++j) if (IQ[j] == FILL) { error = "ERR capture_incomplete"; break; }
  if (!error) for (unsigned j = 0; j < 4; ++j) if (IQ[n+j] != (TAIL_SENTINEL ^ j)) { error = "ERR capture_overrun"; break; }

  xSemaphoreGive(iqLock);
  return error;
}

// ======================================================================================
// FFT
// ======================================================================================
static float fft_buf[2 * MAX_FFT] __attribute__((aligned(16)));
static float fft_win[MAX_FFT], acc_pow[MAX_FFT], db_tmp[MAX_FFT];
static uint16_t reversed_bins[MAX_FFT];
static uint8_t spectrumBytes[MAX_SWEEP_BINS];
static unsigned win_N = 0; static float win_sum = 1.0f; static unsigned acc_ffts = 0;

static inline float fast_db(float x)
{
  union { float f; uint32_t u; } v = { .f = x };
  float e = (float)((int)((v.u >> 23) & 255u) - 127);
  v.u = (v.u & 0x7FFFFFu) | 0x3F800000u;
  float m = v.f;
  float l = ((((-0.07914958f * m + 0.62880993f) * m - 2.08104467f) * m + 4.02835512f) * m - 2.49676657f);
  return 3.01029996f * (e + l);
}
static void prepare_window(unsigned n)
{
  if (n == win_N) return;
  dsps_wind_blackman_harris_f32(fft_win, n);
  win_sum = 0; for (unsigned i = 0; i < n; ++i) win_sum += fft_win[i];
  unsigned b = 0; while ((1u << b) < n) ++b;
  for (unsigned k = 0; k < n; ++k) {
    unsigned r = 0; for (unsigned i = 0; i < b; ++i) r |= ((k >> i) & 1u) << (b - 1 - i);
    reversed_bins[k] = (uint16_t)r;
  }
  win_N = n;
}
static void acc_clear(unsigned n) { memset(acc_pow, 0, n * sizeof(float)); acc_ffts = 0; }
static bool acc_add_current(unsigned n)
{
  const unsigned SKIP = 512u;
  if (!n || n > MAX_FFT || (n & (n - 1u))) return false;
  unsigned mb = (MAX_SAMPLES - SKIP) / n; if (!mb) return false;
  unsigned tb = (8192u + n - 1u) / n;
  if (tb < 2) tb = 2; if (tb > 12) tb = 12; if (tb > mb) tb = mb;
  for (unsigned bl = 0; bl < tb; ++bl) {
    unsigned off = SKIP + bl * n;
    float mi = 0, mq = 0;
    for (unsigned i = 0; i < n; ++i) {
      uint32_t w = IQ[off + i];
      int32_t iv = (int32_t)(w & 0x3FFu); if (iv & 0x200) iv -= 1024;
      int32_t qv = (int32_t)((w >> 10) & 0x3FFu); if (qv & 0x200) qv -= 1024;
      mi += iv; mq += qv;
    }
    mi /= n; mq /= n;
    for (unsigned i = 0; i < n; ++i) {
      uint32_t w = IQ[off + i];
      int32_t iv = (int32_t)(w & 0x3FFu); if (iv & 0x200) iv -= 1024;
      int32_t qv = (int32_t)((w >> 10) & 0x3FFu); if (qv & 0x200) qv -= 1024;
      fft_buf[2*i] = (iv - mi) * fft_win[i];
      fft_buf[2*i+1] = (qv - mq) * fft_win[i];
    }
    dsps_fft2r_fc32(fft_buf, n);
    for (unsigned i = 0; i < n; ++i) {
      float re = fft_buf[2*i], im = fft_buf[2*i+1];
      acc_pow[i] += re * re + im * im;
    }
    ++acc_ffts;
  }
  return acc_ffts != 0;
}
static void acc_to_db(unsigned n, float *db)
{
  if (!acc_ffts) { for (unsigned i = 0; i < n; ++i) db[i] = -150; return; }
  float inv = 1.0f / (float)acc_ffts;
  float sc = 1.0f / (512.0f * 512.0f * win_sum * win_sum);
  float off = fast_db(sc * inv);
  unsigned m = n - 1u;
  for (unsigned j = 0; j < n; ++j) {
    unsigned src = reversed_bins[(n/2u - j) & m];
    float p = acc_pow[src];
    db[j] = (p > 1e-12f) ? (fast_db(p) + off) : -150.0f;
  }
  if (n >= 16) {
    double bh = ((double)sample_rate_mhz * 1e6) / n;
    unsigned h = (unsigned)((150000.0 / bh) + 0.5);
    if (h < 1) h = 1; if (h > 16) h = 16;
    unsigned c = n / 2u, lo = c > h+1 ? c-h-1 : 0, hi = c+h+1;
    if (hi >= n) hi = n - 1;
    float ed = db[lo] < db[hi] ? db[lo] : db[hi];
    for (unsigned i = lo+1; i < hi; ++i) db[i] = ed;
  }
}
static bool capture_spectrum(unsigned n, float *db)
{
  if (!n || n > MAX_FFT || (n & (n - 1u))) return false;
  if (snapshot(MAX_SAMPLES, NULL)) return false;
  acc_clear(n);
  if (!acc_add_current(n)) return false;
  acc_to_db(n, db);
  return true;
}
static bool capture_spectrum_multi(unsigned n, float *db, int ms)
{
  if (ms < 1) ms = 1; if (ms > 4) ms = 4;
  acc_clear(n);
  for (int s = 0; s < ms; ++s) {
    if (snapshot(MAX_SAMPLES, NULL)) { if (!acc_ffts) return false; break; }
    if (!acc_add_current(n)) { if (!acc_ffts) return false; break; }
  }
  acc_to_db(n, db);
  return true;
}

// ======================================================================================
// IQR v2 codec
// ======================================================================================
#define IQR_MAGIC         0x31534949u
#define IQR_VERSION       2u
#define IQR_RAW           0u
#define IQR_MIN           1u
#define IQR_CENTER        2u
#define IQR_DELTA         3u
#define IQR_CHUNK_PAIRS   1024u
#define IQR_HEADER_BYTES  32u
#define IQR_BLOCK_MAXBYTES 2592u

static uint8_t iqr_buf[IQR_BLOCK_MAXBYTES];

static inline int wrap10(int x) { x &= 1023; if (x >= 512) x -= 1024; return x; }
static inline unsigned zigzag(int x) { return (x < 0) ? (unsigned)(-2*x-1) : (unsigned)(2*x); }
static inline unsigned bit_width(unsigned x) { return x ? 32u - __builtin_clz(x) : 0u; }
static inline unsigned rice_bits(unsigned z, unsigned k) { unsigned q = z >> k; return (q >= 15u) ? 26u : q + 1u + k; }

static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void put_i16(uint8_t *p, int16_t v) { put_u16(p, (uint16_t)v); }
static inline void put_u32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v>>8; p[2] = v>>16; p[3] = v>>24; }
static inline void put_u64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; ++i) p[i] = (uint8_t)(v >> (8u*i)); }

struct BitWriter {
  uint8_t *out; uint64_t acc; unsigned used;
  BitWriter(uint8_t *p) : out(p), acc(0), used(0) {}
  void put(unsigned v, unsigned b) {
    if (!b) return;
    acc |= (uint64_t)v << used; used += b;
    while (used >= 8) { *out++ = (uint8_t)acc; acc >>= 8; used -= 8; }
  }
  void rice(unsigned z, unsigned k) {
    unsigned q = z >> k;
    if (q >= 15u) { put((z << 16u) | (1u << 15u), 26u); }
    else { put((1u << q) | ((z & ((1u << k) - 1u)) << (q + 1u)), q + 1u + k); }
  }
  void flush() { if (used) *out = (uint8_t)acc; }
};

static RiceChoice best_rice(const uint32_t *pr, size_t cnt, unsigned ci, int base, bool dm)
{
  unsigned cost[10] = {}; int prev = base;
  for (size_t j = 0; j < cnt; ++j) {
    uint32_t w = pr[j] & 0xFFFFFu;
    int val = (ci == 0) ? wrap10(w & 0x3FFu) : wrap10((w >> 10) & 0x3FFu);
    unsigned enc = zigzag(wrap10(val - prev));
    for (unsigned k = 0; k < 10u; ++k) cost[k] += rice_bits(enc, k);
    if (dm) prev = val;
  }
  unsigned bk = 0;
  for (unsigned k = 1; k < 10u; ++k) if (cost[k] < cost[bk]) bk = k;
  RiceChoice rc; rc.k = bk; rc.bits = cost[bk]; return rc;
}
static uint32_t crc_pairs(const uint32_t *pr, size_t cnt)
{
  uint32_t c = 0;
  for (size_t j = 0; j < cnt; ++j) {
    uint32_t w = pr[j] & 0xFFFFFu;
    c = esp_rom_crc32_le(c, (const uint8_t *)&w, 4);
  }
  return c;
}

static size_t compress_iq_block(const uint32_t *pr, size_t cnt, uint64_t idx, uint8_t *out, size_t cap)
{
  if (!pr || !out || !cnt || cnt > IQR_CHUNK_PAIRS || cap < IQR_HEADER_BYTES) return 0;
  int mn[2] = {511, 511}, mx[2] = {-512, -512}, sm[2] = {0, 0};
  for (size_t j = 0; j < cnt; ++j) {
    uint32_t w = pr[j] & 0xFFFFFu;
    int iv = wrap10(w & 0x3FFu), qv = wrap10((w >> 10) & 0x3FFu);
    if (iv < mn[0]) mn[0] = iv; if (iv > mx[0]) mx[0] = iv; sm[0] += iv;
    if (qv < mn[1]) mn[1] = qv; if (qv > mx[1]) mx[1] = qv; sm[1] += qv;
  }
  unsigned mb = 0, cb = 0, db = 0;
  uint8_t mk[2] = {0}, ck[2] = {0}, dk[2] = {0};
  int cbase[2] = {(int)(sm[0]/(int)cnt), (int)(sm[1]/(int)cnt)};
  uint32_t w0 = pr[0] & 0xFFFFFu;
  int dbase[2] = {wrap10(w0 & 0x3FFu), wrap10((w0 >> 10) & 0x3FFu)};
  for (int c = 0; c < 2; ++c) {
    mk[c] = (uint8_t)bit_width((unsigned)(mx[c] - mn[c]));
    mb += (unsigned)cnt * mk[c];
    RiceChoice rc = best_rice(pr, cnt, c, cbase[c], false);
    RiceChoice rd = best_rice(pr, cnt, c, dbase[c], true);
    ck[c] = rc.k; dk[c] = rd.k; cb += rc.bits; db += rd.bits;
  }
  uint8_t mode = IQR_RAW;
  unsigned bb = (unsigned)cnt * 20u;
  if ((mb+15)/16 < (bb+15)/16) { mode = IQR_MIN; bb = mb; }
  if ((cb+15)/16 < (bb+15)/16) { mode = IQR_CENTER; bb = cb; }
  if ((db+15)/16 < (bb+15)/16) { mode = IQR_DELTA; bb = db; }
  if (bb > 0xFFFFu) return 0;
  size_t pb = 2u * ((bb + 15u) / 16u);
  size_t tb = IQR_HEADER_BYTES + pb;
  if (tb > cap) return 0;
  memset(out, 0, tb);
  put_u32(out, IQR_MAGIC);
  out[4] = IQR_VERSION; out[5] = mode;
  int16_t bi = 0, bq = 0;
  if (mode == IQR_RAW) { out[6] = out[7] = 0; }
  else if (mode == IQR_MIN) { out[6] = mk[0]; out[7] = mk[1]; bi = (int16_t)mn[0]; bq = (int16_t)mn[1]; }
  else if (mode == IQR_CENTER) { out[6] = ck[0]; out[7] = ck[1]; bi = (int16_t)cbase[0]; bq = (int16_t)cbase[1]; }
  else { out[6] = dk[0]; out[7] = dk[1]; bi = (int16_t)dbase[0]; bq = (int16_t)dbase[1]; }
  put_u16(out+8, (uint16_t)cnt);
  put_u16(out+10, (uint16_t)bb);
  put_i16(out+12, bi); put_i16(out+14, bq);
  put_u64(out+16, idx);
  uint32_t dc = crc_pairs(pr, cnt);
  put_u32(out+24, dc);
  uint32_t hc = esp_rom_crc32_le(0, out, 28);
  put_u32(out+28, hc);
  BitWriter w(out + IQR_HEADER_BYTES);
  int pv[2] = {bi, bq};
  for (size_t j = 0; j < cnt; ++j) {
    uint32_t ww = pr[j] & 0xFFFFFu;
    int v[2] = {wrap10(ww & 0x3FFu), wrap10((ww >> 10) & 0x3FFu)};
    for (int c = 0; c < 2; ++c) {
      if (mode == IQR_RAW) w.put((unsigned)v[c] & 1023u, 10);
      else if (mode == IQR_MIN) { int bs = (c == 0) ? bi : bq; w.put((unsigned)(v[c] - bs), out[6 + c]); }
      else if (mode == IQR_CENTER) { int bs = (c == 0) ? bi : bq; w.rice(zigzag(wrap10(v[c] - bs)), out[6 + c]); }
      else { w.rice(zigzag(wrap10(v[c] - pv[c])), out[6 + c]); pv[c] = v[c]; }
    }
  }
  w.flush();
  return tb;
}

// ======================================================================================
// IQSTREAM
// ======================================================================================
static bool serial_write_all(const uint8_t *buf, size_t len);

#define IQSTREAM_DECIM_SAFE 1024u
#define IQSTREAM_DECIM_FAST 256u
#define IQSTREAM_MAX_OUT ((MAX_SAMPLES + IQSTREAM_DECIM_FAST - 1u) / IQSTREAM_DECIM_FAST)

static volatile bool iqstream_run = false;
static unsigned iqstream_decim = IQSTREAM_DECIM_SAFE;
static uint32_t iqstream_sequence = 0;
static int64_t iqstream_epoch_us = 0;
static uint64_t iqstream_last_source_index = 0;
static uint32_t iqstream_sent_blocks = 0, iqstream_capture_errors = 0, iqstream_tx_errors = 0;
static uint32_t iqstream_pairs[IQSTREAM_MAX_OUT];
static uint8_t iqstream_record[IQR_BLOCK_MAXBYTES];

static uint64_t iqstream_source_index_now()
{
  if (!iqstream_epoch_us) return 0;
  int64_t el = esp_timer_get_time() - iqstream_epoch_us;
  if (el < 0) el = 0;
  return ((uint64_t)el * (uint64_t)sample_rate_mhz * 1000000ull) / 1000000ull;
}
static bool iqstream_start_safe(unsigned d)
{
  if (d != IQSTREAM_DECIM_SAFE && d != IQSTREAM_DECIM_FAST) return false;
  if (!reserve_ok || wifi_err || !iqLock) return false;
  iqstream_decim = d; iqstream_sequence = 0;
  iqstream_epoch_us = esp_timer_get_time();
  iqstream_last_source_index = 0;
  iqstream_sent_blocks = iqstream_capture_errors = iqstream_tx_errors = 0;
  iqstream_run = true;
  return true;
}
static void iqstream_stop_safe() { iqstream_run = false; }
static bool iqstream_capture_once()
{
  if (!iqstream_run) return false;
  uint64_t src = iqstream_source_index_now();
  if (src <= iqstream_last_source_index) src = iqstream_last_source_index + 1u;
  uint32_t cus = 0;
  const char *err = snapshot(MAX_SAMPLES, &cus);
  if (err) {
    ++iqstream_capture_errors;
    if (serialMux && xSemaphoreTake(serialMux, pdMS_TO_TICKS(50)) == pdTRUE) {
      Serial.printf("#IQSTREAM_CAPTURE_ERR seq=%lu %s\n", (unsigned long)iqstream_sequence, err);
      xSemaphoreGive(serialMux);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    return false;
  }
  unsigned oc = 0;
  for (unsigned at = 0; at < MAX_SAMPLES && oc < IQSTREAM_MAX_OUT; at += iqstream_decim)
    iqstream_pairs[oc++] = IQ[at] & 0xFFFFFu;
  if (!oc) return false;
  size_t bytes = compress_iq_block(iqstream_pairs, oc, src, iqstream_record, sizeof(iqstream_record));
  if (!bytes) {
    ++iqstream_tx_errors;
    if (serialMux && xSemaphoreTake(serialMux, pdMS_TO_TICKS(50)) == pdTRUE) {
      Serial.printf("#IQSTREAM_COMPRESS_ERR seq=%lu\n", (unsigned long)iqstream_sequence);
      xSemaphoreGive(serialMux);
    }
    return false;
  }
  bool sent = false;
  if (serialMux && xSemaphoreTake(serialMux, pdMS_TO_TICKS(250)) == pdTRUE) {
    uint32_t sl = (uint32_t)src, sh = (uint32_t)(src >> 32);
    Serial.printf("IQB %lu %lu %lu %u %u %u %lu\n",
      (unsigned long)iqstream_sequence, (unsigned long)sl, (unsigned long)sh,
      oc, iqstream_decim, (unsigned)bytes, (unsigned long)cus);
    sent = serial_write_all(iqstream_record, bytes);
    xSemaphoreGive(serialMux);
  }
  if (!sent) { ++iqstream_tx_errors; return false; }
  iqstream_last_source_index = src;
  ++iqstream_sequence; ++iqstream_sent_blocks;
  return true;
}

// ======================================================================================
// Spectrum protocol
// ======================================================================================
struct __attribute__((packed)) SpectrumHeader {
  uint32_t magic; uint8_t frame_type, reserved;
  double start_hz, stop_hz;
  uint16_t bins_count; float min_db, db_step;
  uint32_t timestamp_ms;
};




static bool serial_write_all(const uint8_t *buf, size_t len)
{
  if (!buf && len)
    return false;

  size_t sent = 0;
  uint32_t last_progress = millis();

  while (sent < len) {
    size_t written = Serial.write(buf + sent, len - sent);

    if (written > 0) {
      sent += written;
      last_progress = millis();
      continue;
    }

    // Ошибка только если 5 секунд вообще не было прогресса.
    if ((uint32_t)(millis() - last_progress) >= 5000u)
      return false;

    vTaskDelay(pdMS_TO_TICKS(1));
  }

  return true;
}






static bool sendFrame(uint8_t t,
                      double sh,
                      double eh,
                      uint16_t bc,
                      float md,
                      float ds)
{
  if (bc == 0 || bc > sizeof(spectrumBytes))
    return false;

  SpectrumHeader h = {
    0x5AA55AA5u,
    t,
    0,
    sh,
    eh,
    bc,
    md,
    ds,
    millis()
  };

  bool ok;

  if (serialMux)
    xSemaphoreTake(serialMux, portMAX_DELAY);

  ok = serial_write_all(
    reinterpret_cast<const uint8_t *>(&h),
    sizeof(h));

  if (ok)
    ok = serial_write_all(spectrumBytes, bc);

  if (serialMux)
    xSemaphoreGive(serialMux);

  if (!ok)
    ++spectrum_tx_errors;

  return ok;
}


static bool valid_fft(unsigned n) { return n==512||n==1024||n==2048||n==4096; }





//-------------------------------------------------------------------------------


//-------------------------------------------------------------------------------



//-------------------------------------------------------------------------------




//-------------------------------------------------------------------------------

static void hop_map_add(float f_mhz, float snr, double lo_hz, double span_hz)
{
  const double start = (lo_hz - span_hz / 2) / 1e6;
  int c = (int)((f_mhz - start) * HB_N / (span_hz / 1e6));
  if (c < 0 || c >= HB_N) return;
  if (hb_hit[c] < 240) hb_hit[c] += 16;
  uint8_t m = snr > 60 ? 60 : (uint8_t)snr;
  if (m > hb_marg[c]) hb_marg[c] = m;
}

static void hop_map_decay(uint32_t now)
{
  if (now - hb_decay_ms < 1000) return;
  hb_decay_ms = now;
  for (unsigned c = 0; c < HB_N; ++c) {
    if (hb_hit[c])  hb_hit[c]  -= (hb_hit[c] >> 2) + 1;   // быстрее забываем
    if (hb_marg[c]) hb_marg[c] -= 1;
  }
}
//-------------------------------------------------------------------------------


//-------------------------------------------------------------------------------

// DETSTART 2440 4096 10

// ======================================================================================
// DSP worker — FW3: v9 прием + v9 sweep + DETECT
// ======================================================================================
static void responsive_wait(uint32_t ms) {
  while (ms && uxQueueMessagesWaiting(cmdQueue) == 0) {
    uint32_t s = ms > 20u ? 20u : ms; vTaskDelay(pdMS_TO_TICKS(s)); ms -= s;
  }
}
static void worker_print(const char *t) {
  if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
  Serial.print(t);
  if (serialMux) xSemaphoreGive(serialMux);
}

static void dspWorkerTask(void *arg) {
  (void)arg;
  CommandMsg act = {MODE_IDLE, 2440, 2440, 60, 1024, 10};
  for (;;) {
    CommandMsg cmd;
    if (xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) {
      if (currentMode == MODE_IQSTREAM) iqstream_stop_safe();
      park_receiver(); act = cmd; currentMode = cmd.mode; requested = cmd;
      if (cmd.mode == MODE_IDLE) {
        worker_print("#IDLE\n");
      }
      else if (cmd.mode == MODE_STREAM) {
        prepare_window(cmd.fft_size);
        if (retune(cmd.f_start_mhz)) {
          if (serialMux) { xSemaphoreTake(serialMux, portMAX_DELAY);
            Serial.printf("#STREAM_READY %lu\n", (unsigned long)g_lo_hz);
            xSemaphoreGive(serialMux); }
          vTaskDelay(pdMS_TO_TICKS(5)); uint32_t du; snapshot(1024, &du);
        } else {
          currentMode = MODE_IDLE; act.mode = MODE_IDLE; requested.mode = MODE_IDLE;
          if (serialMux) { xSemaphoreTake(serialMux, portMAX_DELAY);
            Serial.print("#STREAM_ERR\n#IDLE\n"); xSemaphoreGive(serialMux); }
        }
      }
      //else if (cmd.mode == MODE_SWEEP) {
     //   prepare_window(cmd.fft_size);
     // }
      else if (cmd.mode == MODE_SWEEP) {
        sample_rate_mhz = 80; rate_bits = 0; settings_width = 40; set_filt_both(0);
        prepare_window(cmd.fft_size);
      }

      

      else if (cmd.mode == MODE_IQSTREAM) {
        if (retune(cmd.f_start_mhz) && iqstream_start_safe(cmd.step_mhz)) {
          if (serialMux) { xSemaphoreTake(serialMux, portMAX_DELAY);
            Serial.printf("#IQSTREAM_READY lo=%lu decim=%u\n", (unsigned long)g_lo_hz, iqstream_decim);
            xSemaphoreGive(serialMux); }
        } else {
          currentMode = MODE_IDLE; act.mode = MODE_IDLE; requested.mode = MODE_IDLE;
          if (serialMux) { xSemaphoreTake(serialMux, portMAX_DELAY);
            Serial.print("#IQSTREAM_ERR\n#IDLE\n"); xSemaphoreGive(serialMux); }
        }
      }
    }

    if (currentMode == MODE_IDLE) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

    if (currentMode == MODE_IQSTREAM) {
      if (!iqstream_run) {
        currentMode = MODE_IDLE; requested.mode = MODE_IDLE;
        worker_print("#IDLE\n");
        continue;
      }
      iqstream_capture_once();
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    if (currentMode == MODE_STREAM) {
      uint32_t t0 = millis();
      if (uxQueueMessagesWaiting(cmdQueue) > 0) continue;
      int sn = 2; uint32_t per = 1000u / (act.fps ? act.fps : 10u);
      if (act.fft_size <= 1024u) sn = per >= 90 ? 3 : 2;
      else if (act.fft_size >= 4096u) sn = 1;
      bool ok = sn > 1 ? capture_spectrum_multi(act.fft_size, db_tmp, sn) : capture_spectrum(act.fft_size, db_tmp);
      if (!ok) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
      for (unsigned i = 0; i < act.fft_size; ++i) {
        float c = (db_tmp[i] + 120) * 2 + .5f;
        if (c < 0) c = 0; if (c > 255) c = 255;
        spectrumBytes[i] = (uint8_t)c;
      }
      double lo = (double)act.f_start_mhz * 1e6, sp = (double)sample_rate_mhz * 1e6;
      sendFrame(1, lo - sp/2, lo + sp/2, act.fft_size, -120, .5f);
      uint32_t el = millis() - t0;
      responsive_wait(el < per ? per - el : 1);
      continue;
    }

    if (currentMode == MODE_SWEEP) {
      uint32_t fs = act.f_start_mhz, fe = act.f_stop_mhz, stp = act.step_mhz;
      unsigned N = act.fft_size;
      double bin_hz = ((double)sample_rate_mhz * 1e6) / (double)N;
      uint16_t kept = (uint16_t)(((double)stp * 1e6) / bin_hz + 0.5);
      kept = (kept + 1u) & ~1u;
      uint16_t max_kept = (N * 3u) / 4u;
      if (kept > max_kept) kept = max_kept;
      if (kept < 16u) kept = 16u;
      uint16_t sb = N / 2u - kept / 2u;
      uint16_t nsteps = (uint16_t)(((fe > fs ? fe - fs : stp) + stp - 1u) / stp);
      if (!nsteps) nsteps = 1u;
      uint16_t total = nsteps * kept;
      if (total > sizeof(spectrumBytes)) {
        nsteps = sizeof(spectrumBytes) / kept;
        total = nsteps * kept;
      }
      bool ab = false;
      for (uint16_t s = 0; s < nsteps; ++s) {
        if (uxQueueMessagesWaiting(cmdQueue) > 0) { ab = true; break; }
        uint32_t step_lo = fs + s * stp + stp / 2u;
        if (!retune(step_lo)) {
          for (uint16_t k = 0; k < kept; ++k) spectrumBytes[(size_t)s * kept + k] = 0;
          continue;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
        if (uxQueueMessagesWaiting(cmdQueue) > 0) { ab = true; break; }
        if (!capture_spectrum(N, db_tmp)) {
          for (uint16_t k = 0; k < kept; ++k) spectrumBytes[(size_t)s * kept + k] = 0;
          continue;
        }
        for (uint16_t k = 0; k < kept; ++k) {
          float c = (db_tmp[sb + k] + 120.0f) * 2.0f + 0.5f;
          if (c < 0) c = 0; if (c > 255) c = 255;
          spectrumBytes[(size_t)s * kept + k] = (uint8_t)c;
        }
        vTaskDelay(1);
      }
      if (!ab) sendFrame(2, (double)fs * 1e6, (double)(fs + nsteps * stp) * 1e6, total, -120.0f, 0.5f);
      vTaskDelay(pdMS_TO_TICKS(4));
      continue;
    }


  }
}

// ======================================================================================
// Capture commands
// ======================================================================================
static size_t pack_iq8(unsigned n) {
  uint8_t *p = (uint8_t *)IQ;
  for (unsigned j = 0; j < n; ++j) {
    uint32_t w = IQ[j];
    p[2*j] = (w >> 2) & 255; p[2*j+1] = (w >> 12) & 255;
  }
  return 2u * n;
}
static void reply_text(const char *t) {
  if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
  Serial.print(t);
  if (serialMux) xSemaphoreGive(serialMux);
}
static void reply_ok() { reply_text("OK\n"); }
static void reply_error(const char *t) {
  if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
  Serial.print("ERR "); Serial.print(t); Serial.print('\n');
  if (serialMux) xSemaphoreGive(serialMux);
}
static void cmd_capture(unsigned n) {
  uint32_t us = 0;
  const char *e = snapshot(n, &us);
  if (e) { reply_text(e); reply_text("\n"); return; }
  size_t b = pack_iq8(n);
  uint8_t *p = (uint8_t *)IQ;
  uint32_t s = 0;
  for (size_t i = 0; i < b; ++i) s += p[i];
  if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
  Serial.printf("DATA %u %08lx %lu\n", n, (unsigned long)s, (unsigned long)us);
  serial_write_all(p, b);
  if (serialMux) xSemaphoreGive(serialMux);
}
static void cmd_capture_iqr(unsigned n) {
  if (n < 256 || n > MAX_SAMPLES) { reply_error("args"); return; }
  uint32_t us = 0;
  const char *e = snapshot(n, &us);
  if (e) { reply_text(e); reply_text("\n"); return; }
  unsigned nblocks = (n + IQR_CHUNK_PAIRS - 1) / IQR_CHUNK_PAIRS;
  if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
  Serial.printf("DATA_IQR %u %u %lu\n", n, nblocks, (unsigned long)us);
  if (serialMux) xSemaphoreGive(serialMux);
  for (unsigned at = 0; at < n; at += IQR_CHUNK_PAIRS) {
    unsigned cnt = (n - at < IQR_CHUNK_PAIRS) ? (n - at) : IQR_CHUNK_PAIRS;
    size_t sz = compress_iq_block(IQ + at, cnt, at, iqr_buf, sizeof(iqr_buf));
    if (!sz) { reply_error("compress"); return; }
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    serial_write_all(iqr_buf, sz);
    if (serialMux) xSemaphoreGive(serialMux);
  }
}
static bool queue_command(const CommandMsg &c) {
  if (!cmdQueue) return false;
  xQueueReset(cmdQueue);
  return xQueueSend(cmdQueue, &c, 0) == pdTRUE;
}
static bool radio_busy() {
  return currentMode != MODE_IDLE || requested.mode != MODE_IDLE;
}

// ======================================================================================
// Command handler — FW4
// ======================================================================================
static void handle(char *line) {
  unsigned n, a, b, r, g, bl; int ai, pi; char ex;
  if (!line || !line[0]) return;
  if (led_auto && currentMode != MODE_DETECT) rgb_set(0, 0, 40);

  if (!strcmp(line, "PING")) { reply_text("PONG\n"); return; }
  if (!strcmp(line, "INFO")) {
    const char *m = currentMode==MODE_STREAM?"STREAM":currentMode==MODE_SWEEP?"SWEEP":
      currentMode==MODE_DETECT?"DETECT":currentMode==MODE_IQSTREAM?"IQSTREAM":"IDLE";
    unsigned sm = dspTaskHandle ? (unsigned)uxTaskGetStackHighWaterMark(dspTaskHandle) : 0;
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("ESPSDR 10 PATCHED mode=%s rate=%uM W%u res=%s wifi=%d gmax=%u lpf=%d,%d "
      "fft=%u lo=%lu pll=%lu m=%u cap=%u[%u+%u] hold=%d det=%.0f "
      "iqb=%lu cerr=%lu terr=%lu stack=%u iqr_v=%u host=%d\n",
      m, sample_rate_mhz, settings_width, reserve_ok?"ok":"FAIL", (int)wifi_err, gain_max,
      rx_filt0, rx_filt1, requested.fft_size, (unsigned long)g_lo_hz, (unsigned long)g_pll_hz,
      g_lo_mode, g_pll_cap, g_pll_first, g_pll_len, pbus_owned, det_thr_db,
      (unsigned long)iqstream_sent_blocks, (unsigned long)iqstream_capture_errors,
      (unsigned long)iqstream_tx_errors, sm, IQR_VERSION, BB_I2C_HOST);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (!strcmp(line, "STOP") || !strcmp(line, "IQSTOP")) {
    iqstream_stop_safe();
    CommandMsg c = {MODE_IDLE, 0, 0, 0, 0, 0};
    if (!queue_command(c)) { reply_error("stop_queue"); return; }
    requested.mode = MODE_IDLE;
    reply_ok();
    return;
  }
  if (!strncmp(line, "STREAM ", 7)) {
    unsigned f, ff, fp;
    if (sscanf(line, "STREAM %u %u %u %c", &f, &ff, &fp, &ex) == 3 &&
        f >= 1842 && f <= 2790 && valid_fft(ff) && fp >= 1 && fp <= 30) {
      CommandMsg c = {MODE_STREAM, f, f, 0, (uint16_t)ff, (uint8_t)fp};
      if (!queue_command(c)) { reply_error("queue"); return; }
      requested = c; reply_ok();
    } else reply_error("args");
    return;
  }
  if (!strncmp(line, "IQSTREAM ", 9)) {
    unsigned f, d;
    if (sscanf(line, "IQSTREAM %u %u %c", &f, &d, &ex) == 2 &&
        f >= 1842 && f <= 2790 && (d == 1024 || d == 256)) {
      CommandMsg c = {MODE_IQSTREAM, f, f, d, 0, 0};
      if (!queue_command(c)) { reply_error("queue"); return; }
      requested = c; reply_ok();
    } else reply_error("args");
    return;
  }
  if (!strncmp(line, "SWEEP ", 6)) {
    unsigned fs, fe, st, ff;
    if (sscanf(line, "SWEEP %u %u %u %u %c", &fs, &fe, &st, &ff, &ex) == 4 &&
        fs >= 1842 && fe <= 2790 && fe > fs && st >= 1 && st <= 60 && valid_fft(ff)) {
     // sample_rate_mhz = 80; rate_bits = 0; settings_width = 40; set_filt_both(0);
      CommandMsg c = {MODE_SWEEP, fs, fe, st, (uint16_t)ff, 0};
      if (!queue_command(c)) { reply_error("queue"); return; }
      requested = c; reply_ok();
    } else reply_error("args");
    return;
  }

  if (!strcmp(line, "CONV?")) {
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("CONV %s %lu %c\n", conv_en ? "ON" : "OFF", (unsigned long)conv_lo_mhz, conv_sign >= 0 ? '+' : '-');
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (!strcmp(line, "CONVOFF")) { conv_en = false; reply_ok(); return; }
  { unsigned cl; char cs;
    if (sscanf(line, "CONV %u %c %c", &cl, &cs, &ex) == 2 && cl >= 100 && cl <= 6000 && (cs == '+' || cs == '-')) {
      conv_en = true; conv_lo_mhz = cl; conv_sign = cs == '+' ? 1 : -1; reply_ok(); return;
    }
  }
  bool rc = !strncmp(line, "FREQ ", 5) || !strncmp(line, "RATE ", 5) || !strncmp(line, "WIDTH", 5) ||
    !strncmp(line, "FILTER", 6) || !strncmp(line, "GAIN", 4) || !strncmp(line, "CAP16 ", 6) ||
    !strncmp(line, "CAPIQC", 6) || !strncmp(line, "DIAG ", 5) || !strncmp(line, "LPF", 3) ||
    !strncmp(line, "BANDWIDTH", 9) || !strncmp(line, "RF", 2) || !strncmp(line, "BB", 2) ||
    !strncmp(line, "DC", 2) || !strncmp(line, "IQ", 2) || !strncmp(line, "STAGES", 6) ||
    !strcmp(line, "DCCAL");
  if (rc && radio_busy()) { reply_error("busy (STOP first; wait #IDLE)"); return; }

  if (sscanf(line, "CAPIQC %u %c", &n, &ex) == 1 && n >= 256 && n <= MAX_SAMPLES) { cmd_capture_iqr(n); return; }
  if (sscanf(line, "CAP16 %u %c", &n, &ex) == 1 && n >= 256 && n <= MAX_SAMPLES) { cmd_capture(n); return; }

  if (!strcmp(line, "LED AUTO")) { led_auto = true; reply_ok(); return; }
  if (!strcmp(line, "LEDTEST")) {
    reply_ok(); bool old = led_auto; led_auto = false;
    rgb_set(255,0,0); delay(300); rgb_set(0,255,0); delay(300); rgb_set(0,0,255); delay(300);
    rgb_off(); led_auto = old; return;
  }
  if (sscanf(line, "LED %u %u %u %c", &r, &g, &bl, &ex) == 3 && r<=255 && g<=255 && bl<=255) {
    led_auto = false; rgb_set(r, g, bl); reply_ok(); return;
  }
  if (sscanf(line, "FREQ %u %c", &n, &ex) == 1 && n >= 1842 && n <= 2790) {
    if (retune(n)) reply_ok(); else reply_error("pll"); return;
  }
  if (!strcmp(line, "WIDTH?")) {
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("WIDTH %u\n", settings_width);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (sscanf(line, "WIDTH %u %c", &n, &ex) == 1 && (n == 20 || n == 40)) {
    settings_width = n;
    if (reconfig_current_no_pll()) reply_ok(); else reply_error("cfg");
    return;
  }
  // FW4: RATE без reconfig
  if (sscanf(line, "RATE %u %c", &n, &ex) == 1 && (n == 80 || n == 40 || n == 16)) {
    sample_rate_mhz = n;
    rate_bits = n == 40 ? BIT1(15) : n == 16 ? BIT1(16) : 0;
    if (n == 16) set_filt_both(54); else if (n == 40) set_filt_both(16); else set_filt_both(0);
    reply_ok(); return;
  }
  if (!strcmp(line, "FILTER?")) {
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("FILTER %d,%d W%u\n", rx_filt0, rx_filt1, settings_width);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (sscanf(line, "FILTER %u,%u %c", &a, &b, &ex) == 2 && a <= 63 && b <= 63) {
    rx_filt0 = (int)a; rx_filt1 = (int)b; sync_rx_filter_mirror(); reply_ok(); return;
  }
  if (sscanf(line, "FILTER %u %c", &a, &ex) == 1 && a <= 63) {
    set_filt_both((int)a); reply_ok(); return;
  }
  if (!strcmp(line, "GAIN HARDWARE")) {
    hw_agc = true;
    if (reconfig_current_no_pll()) reply_ok(); else reply_error("cfg");
    return;
  }
  if (sscanf(line, "GAIN MANUAL %u %c", &n, &ex) == 1 && n <= 82) {
    hw_agc = false; gain_code = n;
    if (reconfig_current_no_pll()) reply_ok(); else reply_error("cfg");
    return;
  }
  if (!strcmp(line, "GAIN?")) {
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("GAIN %s %u max=%u\n", hw_agc ? "HARDWARE" : "MANUAL", gain_code, gain_max);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (!strcmp(line, "RF?")) {
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("RF %d cur=%u\n", manual_rf, cur_rf);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (!strcmp(line, "RF AUTO")) {
    manual_rf = -1;
    if (apply_manual_stages(!hw_agc)) reply_ok(); else reply_error("pbus");
    return;
  }
  if (sscanf(line, "RF %u %c", &n, &ex) == 1 && n <= 511) {
    if (hw_agc) { reply_error("agc"); return; }
    manual_rf = (int)n;
    if (apply_manual_stages(false)) reply_ok(); else reply_error("pbus");
    return;
  }
  if (!strcmp(line, "BB?")) {
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("BB %d cur=%u\n", manual_bb, cur_bb & 127u);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (!strcmp(line, "BB AUTO")) {
    manual_bb = -1;
    if (apply_manual_stages(!hw_agc)) reply_ok(); else reply_error("pbus");
    return;
  }
  if (sscanf(line, "BB %u %c", &n, &ex) == 1 && n <= 127) {
    if (hw_agc) { reply_error("agc"); return; }
    manual_bb = (int)n;
    if (apply_manual_stages(false)) reply_ok(); else reply_error("pbus");
    return;
  }
  if (!strcmp(line, "DC?")) {
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("DC %d,%d,%d,%d cur=%u,%u,%u,%u\n",
      manual_dc[0],manual_dc[1],manual_dc[2],manual_dc[3],cur_dc[0],cur_dc[1],cur_dc[2],cur_dc[3]);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (line[0]=='D' && line[1]=='C' && line[2]>='0' && line[2]<='3') {
    int ch = line[2]-'0'; const char *arg = line+3;
    while (*arg == ' ') ++arg;
    if (!strcmp(arg, "AUTO")) {
      manual_dc[ch] = -1;
      if (apply_manual_stages(!hw_agc)) reply_ok(); else reply_error("pbus");
      return;
    }
    if (!strcmp(arg, "?")) {
      if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
      Serial.printf("DC%d %d cur=%u\n", ch, manual_dc[ch], cur_dc[ch]);
      if (serialMux) xSemaphoreGive(serialMux);
      return;
    }
    if (sscanf(arg, "%u %c", &n, &ex) == 1 && n <= 511) {
      if (hw_agc) { reply_error("agc"); return; }
      manual_dc[ch] = (int)n;
      if (apply_manual_stages(false)) reply_ok(); else reply_error("pbus");
      return;
    }
    reply_error("args"); return;
  }
  if (!strcmp(line, "IQ?")) {
    int am = (cur_iq_a & 16) ? (int)cur_iq_a-32 : (int)cur_iq_a;
    int ph = (cur_iq_p & 32) ? (int)cur_iq_p-64 : (int)cur_iq_p;
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("IQ %d cur=%d,%d\n", manual_iq, am, ph);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (!strcmp(line, "IQ AUTO")) {
    manual_iq = -1;
    if (apply_manual_stages(!hw_agc)) reply_ok(); else reply_error("pbus");
    return;
  }
  if (sscanf(line, "IQ %d,%d %c", &ai, &pi, &ex) == 2 && ai>=-16 && ai<=15 && pi>=-32 && pi<=31) {
    if (hw_agc) { reply_error("agc"); return; }
    manual_iq = (ai & 31) | ((pi & 63) << 8);
    if (apply_manual_stages(false)) reply_ok(); else reply_error("pbus");
    return;
  }
  if (!strcmp(line, "STAGES?")) {
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("STAGES rf=%u bb=%u dc=%u,%u,%u,%u iq=%u,%u pll=%u[%u+%u] lo=%lu(%u) hold=%d\n",
      cur_rf,cur_bb&127u,cur_dc[0],cur_dc[1],cur_dc[2],cur_dc[3],cur_iq_a,cur_iq_p,
      g_pll_cap,g_pll_first,g_pll_len,(unsigned long)g_lo_hz,g_lo_mode,pbus_owned);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (sscanf(line, "DIAG %u %c", &n, &ex) == 1 && n >= 256 && n <= MAX_SAMPLES) {
    uint32_t us = 0; const char *e = snapshot(n, &us);
    if (e) { reply_text(e); reply_text("\n"); return; }
    int32_t imn=9999,imx=-9999,is=0,qmn=9999,qmx=-9999,qs=0;
    for (unsigned j=0;j<n;j++) {
      uint32_t w=IQ[j];
      int32_t iv=(int32_t)(w&0x3FFu);if(iv&0x200)iv-=1024;
      int32_t qv=(int32_t)((w>>10)&0x3FFu);if(qv&0x200)qv-=1024;
      if(iv<imn)imn=iv;if(iv>imx)imx=iv;is+=iv;
      if(qv<qmn)qmn=qv;if(qv>qmx)qmx=qv;qs+=qv;
    }
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("DIAG n=%u us=%lu I[%ld..%ld] avg=%ld Q[%ld..%ld] avg=%ld PLL %u[%u+%u] LO %lu M%u\n",
      n,(unsigned long)us,(long)imn,(long)imx,(long)(is/(int32_t)n),(long)qmn,(long)qmx,(long)(qs/(int32_t)n),
      g_pll_cap,g_pll_first,g_pll_len,(unsigned long)g_lo_hz,g_lo_mode);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  // FW4: LPF/BANDWIDTH без reconfig
  if (!strcmp(line, "LPF AUTO")) { set_filt_both(-1); reply_ok(); return; }
  if (sscanf(line, "LPF %u %c", &n, &ex) == 1 && n <= 63) { set_filt_both((int)n); reply_ok(); return; }
  if (!strcmp(line, "LPF?")) {
    unsigned base = settings_width == 40 ? 6u : 4u;
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("LPF %d,%d W%u HW %u %u\n", rx_filt0, rx_filt1, settings_width,
      S3_I2C_READ(I2C_BB_FILTER, BB_I2C_HOST, base) & 63u,
      S3_I2C_READ(I2C_BB_FILTER, BB_I2C_HOST, base + 1) & 63u);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  if (sscanf(line, "BANDWIDTH %u %c", &n, &ex) == 1 && (!n || (n >= 13 && n <= 69))) {
    if (!n) set_filt_both(-1); else set_filt_both((int)bw_to_code(n)); reply_ok(); return;
  }
  if (!strcmp(line, "DCCAL")) {
    if (hw_agc) { reply_error("agc (DCCAL requires GAIN MANUAL first)"); return; }
    for (unsigned iter = 0; iter < 6; ++iter) {
      const char *e = snapshot(4096, NULL);
      if (e) { reply_error("capture"); return; }
      int64_t si = 0, sq = 0;
      for (unsigned j = 0; j < 4096; ++j) {
        uint32_t w = IQ[j];
        si += wrap10(w & 0x3FFu);
        sq += wrap10((w >> 10) & 0x3FFu);
      }
      int ai2 = (int)(si / 4096), aq = (int)(sq / 4096);
      if (abs(ai2) <= 1 && abs(aq) <= 1) break;
      if (manual_dc[0] < 0) manual_dc[0] = (int)cur_dc[0];
      if (manual_dc[2] < 0) manual_dc[2] = (int)cur_dc[2];
      manual_dc[0] -= ai2; manual_dc[2] -= aq;
      if (manual_dc[0] < 0) manual_dc[0] = 0; if (manual_dc[0] > 511) manual_dc[0] = 511;
      if (manual_dc[2] < 0) manual_dc[2] = 0; if (manual_dc[2] > 511) manual_dc[2] = 511;
      if (!apply_manual_stages(false)) { reply_error("pbus"); return; }
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (serialMux) xSemaphoreTake(serialMux, portMAX_DELAY);
    Serial.printf("DCCAL done dc=%d,%d,%d,%d\n", manual_dc[0],manual_dc[1],manual_dc[2],manual_dc[3]);
    if (serialMux) xSemaphoreGive(serialMux);
    return;
  }
  reply_error("command");
}

// ======================================================================================
// Setup & Loop
// ======================================================================================
void setup() {
  pinMode(RGB_LED_PIN, OUTPUT); digitalWrite(RGB_LED_PIN, LOW); rgb_set(80, 0, 80);
  serialMux = xSemaphoreCreateMutex();
  iqLock = xSemaphoreCreateMutex();
  cmdQueue = xQueueCreate(4, sizeof(CommandMsg));
  Serial.begin(115200);    
  delay(1500);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  Serial.println("# ESPSDR v10 PATCHED boot"); Serial.flush();
  if (!serialMux || !iqLock || !cmdQueue) {
    Serial.println("# FATAL: FreeRTOS alloc failed"); Serial.flush();
    for (;;) { rgb_set(80,0,0); delay(150); rgb_off(); delay(150); }
  }

  esp_log_level_set("*", ESP_LOG_NONE);
  reserve_ok = check_reserved();
  {
    const uintptr_t starts[] = {
      (uintptr_t)fft_buf, (uintptr_t)fft_win, (uintptr_t)db_tmp,
      (uintptr_t)acc_pow, (uintptr_t)reversed_bins, (uintptr_t)spectrumBytes,
      (uintptr_t)iqr_buf, (uintptr_t)iqstream_pairs, (uintptr_t)iqstream_record,
      (uintptr_t)hb_floor, (uintptr_t)hb_hit, (uintptr_t)hb_marg, (uintptr_t)hb_rdy
    };
    const size_t sizes[] = {
      sizeof(fft_buf), sizeof(fft_win), sizeof(db_tmp),
      sizeof(acc_pow), sizeof(reversed_bins), sizeof(spectrumBytes),
      sizeof(iqr_buf), sizeof(iqstream_pairs), sizeof(iqstream_record),
      sizeof(hb_floor), sizeof(hb_hit), sizeof(hb_marg), sizeof(hb_rdy)
    };
    for (unsigned i = 0; i < sizeof(starts) / sizeof(starts[0]); ++i) {
      bool bad = starts[i] < RES_END && starts[i] + sizes[i] > RES_START;
      if (bad) reserve_ok = false;
      Serial.printf("# buf[%u] %08X..%08X %s\n", i,
                    (unsigned)starts[i], (unsigned)(starts[i] + sizes[i]),
                    bad ? "<-- IN DUMP AREA" : "");
    }
  }
  Serial.printf("# reserve=%s\n", reserve_ok ? "ok" : "FAIL"); //Serial.flush();

  


  Serial.printf("# fft_buf=%p spectrum=%p iqr_buf=%p RES=%08X-%08X\n",
                fft_buf, spectrumBytes, iqr_buf, RES_START, RES_END);
  
  Serial.flush();
  rgb_blink(0, 0, 80, 2, 120, 120); rgb_set(0, 40, 40);
  wifi_err = wifi_bringup();
  Serial.printf("# wifi_err=%d\n", (int)wifi_err); Serial.flush();
  dsps_fft2r_init_fc32(NULL, MAX_FFT);
  rx_filter_boot_20[0] = S3_I2C_READ(I2C_BB_FILTER, BB_I2C_HOST, 4) & 63u;
  rx_filter_boot_20[1] = S3_I2C_READ(I2C_BB_FILTER, BB_I2C_HOST, 5) & 63u;
  rx_filter_boot_40[0] = S3_I2C_READ(I2C_BB_FILTER, BB_I2C_HOST, 6) & 63u;
  rx_filter_boot_40[1] = S3_I2C_READ(I2C_BB_FILTER, BB_I2C_HOST, 7) & 63u;
  set_chanfreq(2412, 0);
  if (!retune(2412))
    Serial.println("# initial PLL tune failed");
  read_gain_max();
  Serial.printf("# gain_max=%u\n", gain_max); Serial.flush();
  // FW-FIX P4: НЕ пересохранять AGC здесь
  BaseType_t ok = xTaskCreatePinnedToCore(dspWorkerTask, "DSP", 12288, NULL, 3, &dspTaskHandle, 0);
  if (ok != pdPASS || !dspTaskHandle) {
    Serial.println("# DSP task failed"); Serial.flush();
    for (;;) { rgb_set(80,0,0); delay(250); rgb_off(); delay(250); }
  }
  Serial.println("READY"); Serial.flush();




  
  rgb_off(); hb_last = millis();
}

void loop() {
  if (currentMode != MODE_DETECT && led_auto) {
    unsigned long iv = currentMode==MODE_IDLE ? 700u : currentMode==MODE_STREAM ? 200u :
      currentMode==MODE_IQSTREAM ? 250u : 350u;
    if (millis() - hb_last > iv) {
      hb_last = millis(); hb_state = !hb_state;
      uint8_t r=0, g=0, b=0;
      if (!reserve_ok || wifi_err) r = 80;
      else if (currentMode == MODE_STREAM) b = 80;
      else if (currentMode == MODE_SWEEP) { r = 60; b = 60; }
      else if (currentMode == MODE_IQSTREAM) { g = 50; b = 50; }
      else g = 40;
      if (hb_state) rgb_set(r, g, b); else rgb_off();
    }
  }
  static char line[128];
  static size_t used = 0;
  static bool drop = false;
  if (!Serial.available()) { delay(1); return; }
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (drop) continue;
      if (used < sizeof(line) - 1u) line[used++] = c;
      else { used = 0; drop = true; }
      continue;
    }
    if (drop) { drop = false; used = 0; continue; }
    line[used] = '\0'; used = 0;
    if (line[0]) handle(line);
  }
}
// ==================== END ====================
