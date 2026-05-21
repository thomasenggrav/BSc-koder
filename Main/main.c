/*----------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include "app_filex.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_gpio.h"

/*----------------------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

RTC_HandleTypeDef hrtc;
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
ADC_HandleTypeDef hadc1;
SD_HandleTypeDef hsd1;

FX_MEDIA sd_media;
FX_FILE  sd_file;
UCHAR    media_cache[4096];


/*----------------------------------------------------------------------------*/
/* Defines -------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

// Buffersizes
#define LINE_BUF_SIZE             256u   /* 255 symbols + '\0' */
#define MODEM_LINE_BUF_SIZE       128u

// Seabird ringbuffer / statistics
#define WINDOW_SIZE               10u    /* only the last 10 */

// PFF-format
#define PFF_DATA_BYTES            5u                // Data-bytes in PFF message, 40 bit
#define PFF_TOTAL_BYTES           10u               // Total bytes in PFF message, 80 bit
#define PFF_WORDS                 5u                // 16-bit words in PFF message, 80 bit / 16 = 5

// PFF Message IDs
#define PFF_MSGID_SEABIRD_RAW         0x01u
#define PFF_MSGID_SEABIRD_PRES_STATS  0x06u
#define PFF_MSGID_SIG_PNORS           0x10u
#define PFF_MSGID_SIG_PNORC_12        0x11u
#define PFF_MSGID_SIG_PNORC_22        0x12u

// Modem timing / line
#define MODEM_WORD_INTERVAL_MS    1650u             // Time between words when sending to modem, 1.65 seconds
#define MODEM_Q_MAX               26u               // Max number of frames in modem queue

// App timeouts
#define SENSOR_ACQUIRE_TIMEOUT_MS  15000u           // 15 seconds to acquire sensor data
#define MODEM_REPLY_WINDOW_MS     12000u            // 12 seconds to wait for a reply
#define MODEM_SEND_TIMEOUT_MS     60000u            // 60 seconds to send everything in modem queue

// Seabird polling
#define SEABIRD_POLL_CMD          "TPS\r\n"
#define SEABIRD_POLL_TX_TIMEOUT_MS  200u

// Signature500 polling
#define SIG500_POLL_CMD           "START\r\n"
#define SIG500_POLL_TX_TIMEOUT_MS   200u

// Signature500
#define PNORC_BURST_COUNT           3u              // 3 PNORC bursts
#define PNORC_MID_INDEX             2u              // The "middle" burst
#define SIG500_WAKE_TX_TIMEOUT_MS   700u			
#define SIG500_WAKE_DELAY_1_MS      150u
#define SIG500_WAKE_DELAY_2_MS      400u
#define SIG500_WAKE_ACK_TIMEOUT_MS  7000u


// ------------------------------------------------------------
// Single UART sensor test mode
// ------------------------------------------------------------
// 1 = Seabird and Signature500 use the same UART for simulation test
// 0 = normal setup: Seabird on USART1, Signature500 on USART2
#define SENSOR_SINGLE_UART_TEST  0u

#if SENSOR_SINGLE_UART_TEST
  #define SEABIRD_UART_HANDLE      (&huart2)
  #define SIG500_UART_HANDLE       (&huart2)
  #define SENSOR_TEST_UART_HANDLE  (&huart2)
#else
  #define SEABIRD_UART_HANDLE      (&huart1)
  #define SIG500_UART_HANDLE       (&huart2)
#endif

// Sleep / RTC
#define DEFAULT_SLEEP_INTERVAL_S  10u             // Default sleep interval in seconds
#define MIN_SLEEP_INTERVAL_S       5u             // minimum sleep interval in seconds
#define MAX_SLEEP_INTERVAL_S     21600u           // maximum sleep interval in seconds (6 hours)


// PFF Command Message IDs
#define PFF_MSGID_CMD_SHELL         0x20u
#define PFF_MSGID_CMD_EXTENDED      0x21u

// Shell command IDs
#define CMD_SET_SLEEP_INTERVAL_S    0x01u

//  Reply RX
#define MODEM_REPLY_FRAME_BYTES     PFF_TOTAL_BYTES          // Expecting a full PFF frame as reply, so 10 bytes
#define MODEM_RX_GUARD_MS           30u

// SD card / FileX
#define FILEX_MEDIA_CACHE_SIZE      4096u
#define SD_FILENAME_SEABIRD         "SEABIRD.CSV"
#define SD_HEADER_SEABIRD           "Timestamp,Temperature,Conductivity,Pressure,Salinity,SpecificConductivity\r\n"
#define SD_FILENAME_SIG500          "SIGNATURE.CSV"
#define SD_HEADER_SIG500            "Timestamp,C1_CP,C1_S,C1_DIR,C2_CP,C2_S,C2_DIR,C3_CP,C3_S,C3_DIR\r\n"

// ADC / Temperature measurement
#define ADC_MAX_VALUE           16383.0f               // ADC-resolution: 14-bit => 0..16383
#define ADC_VREF                3.3f                   // Reference voltage for ADC in volts
#define TMP35_SCALE_VOLT_PER_C  0.010f                 // TMP35 scale factor: 10 mV per degree Celsius
#define ADC_TIMEOUT_MS          100U                   // Timeout for ADC-reading

// Time - must be set to code upload time
#define RTC_INIT_HOURS 0x17
#define RTC_INIT_MINUTES 0x20
#define RTC_INIT_SECONDS 0x00

#define RTC_INIT_WEEKDAY RTC_WEEKDAY_WEDNESDAY
#define RTC_INIT_MONTH RTC_MONTH_MAY
#define RTC_INIT_DATE 0x20
#define RTC_INIT_YEAR 0x26


/*----------------------------------------------------------------------------*/
/* Typedefs ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

// General helper types
typedef struct {
  float min;
  float max;
} scale_t;

// Seabird
typedef struct {
  uint8_t t;
  uint8_t c;
  uint8_t p;
  uint8_t s;
  uint8_t sc;
} seabird_u8_t;

// Signature500 parsed data
typedef struct {
  bool  valid;
  float battery;
  float sound_speed;
  float heading;
  float pitch;
  float roll;
} sig_pnors_t;

typedef struct {
  bool  valid;
  float cell_pos;
  float speed;
  float direction;
} sig_pnorc_t;

// Signature500 raw collection
typedef struct {
  bool active;
  bool complete;

  char pnorh[LINE_BUF_SIZE];
  char pnors[LINE_BUF_SIZE];
  char pnorc[PNORC_BURST_COUNT][LINE_BUF_SIZE];

  uint8_t pnorc_count;
} sig500_raw_packet_t;

typedef enum {
  SIG500_COLLECT_IGNORED = 0,
  SIG500_COLLECT_IN_PROGRESS,
  SIG500_COLLECT_COMPLETE,
  SIG500_COLLECT_ERROR
} sig500_collect_result_t;

// Modem / PFF
typedef enum {
  FRAME_SEABIRD_RAW = 1,
  FRAME_SEABIRD_PRES_STATS = 2,
  FRAME_SIG500 = 3
} frame_src_t;

typedef struct {
  uint16_t    words[PFF_WORDS];
  frame_src_t src;
} modem_frame_t;

typedef struct {
  bool     valid;
  uint8_t  version;
  uint8_t  msg_id;
  uint32_t time22;
  uint8_t  data[PFF_DATA_BYTES];
} pff_rx_packet_t;

// App state machine
typedef enum {
  ST_BOOT = 0,
  ST_SLEEP,
  ST_WAKE_SIG500,
  ST_SENSOR_ACQUIRE,
  ST_PROCESS_AND_STORE,
  ST_MODEM_SEND,
  ST_WAIT_REPLY,
  ST_HANDLE_REPLY,
  ST_SD_STORE,
  ST_ERROR
} app_state_t;

typedef struct {
  app_state_t state;
  uint32_t    state_enter_ms;
  uint32_t    deadline_ms;
  uint32_t    cycle_no;
  uint32_t modem_rx_enable_ms;
  bool        state_started;
  bool        modem_reply_ready;
  bool        modem_reply_valid;
  bool        sig500_wakeup_ok;
} app_ctx_t;

// Cycle data
typedef enum {
  CYCLE_SENSOR_NONE = 0,
  CYCLE_SENSOR_SEABIRD,
  CYCLE_SENSOR_SIG500
} cycle_sensor_t;

typedef struct
{
  /* -------- Seabird -------- */
  bool seabird_raw_ready;
  bool seabird_valid;
  char seabird_raw_line[LINE_BUF_SIZE];

  float temp;
  float cond;
  float pres;
  float sal;
  float spcond;

  /* -------- Signature500 -------- */
  bool sig500_raw_ready;
  bool sig500_valid;
  sig500_raw_packet_t sig500_raw;

  /* -------- Shared / extra -------- */
  float chamber_temp_c;

  bool reply_ready;
  char reply_line[MODEM_LINE_BUF_SIZE];
} cycle_data_t;


// Time
typedef struct
{
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
} rtc_datetime_t;


/*----------------------------------------------------------------------------*/
/* Static variables ----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

// App state / cycle
static app_ctx_t app;
static cycle_data_t cycle_data;
static uint32_t sleep_interval_s = DEFAULT_SLEEP_INTERVAL_S;

// Seabird RX (USART1)
static uint8_t  seabirdRxByte;
static uint8_t  seabirdLineBuf[LINE_BUF_SIZE];
static uint16_t seabirdLineLen = 0;
static char     seabirdLine[LINE_BUF_SIZE];

// Signature500 RX (USART2)
static uint8_t  sig500RxByte;
static uint8_t  sig500LineBuf[LINE_BUF_SIZE];
static uint16_t sig500LineLen = 0;
static char     sig500Line[LINE_BUF_SIZE];

#if SENSOR_SINGLE_UART_TEST
static uint8_t  sensorRxByte;
static uint8_t  sensorLineBuf[LINE_BUF_SIZE];
static uint16_t sensorLineLen = 0;
static char     sensorLine[LINE_BUF_SIZE];
#endif

// Modem RX (LPUART1)
static uint8_t  modemRxByte;

// Seabird ring buffer / statistics
static float temperatur[WINDOW_SIZE];
static float konduktivitet[WINDOW_SIZE];
static float trykk[WINDOW_SIZE];
static float salinitet[WINDOW_SIZE];
static float spes_konduktivitet[WINDOW_SIZE];

static uint32_t total_samples = 0;
static uint16_t window_count = 0;
static uint16_t write_pos = 0;
static uint32_t last_stats_sent_total = 0;
static sig500_raw_packet_t sig500_collect_pkt;

// Signature500 parsed values
static sig_pnors_t sig_pnors       = {0};
static sig_pnorc_t sig_pnorc_first = {0};
static sig_pnorc_t sig_pnorc_mid   = {0};
static sig_pnorc_t sig_pnorc_last  = {0};

// Modem TX / PFF-line
static modem_frame_t modem_q[MODEM_Q_MAX];
static uint16_t modem_q_head = 0;
static uint16_t modem_q_tail = 0;
static uint16_t modem_q_count = 0;

static uint16_t tx_words[PFF_WORDS];
static uint8_t  tx_word_idx = 0;
static bool     tx_active = false;
static uint32_t next_tx_ms = 0;

// Modem reply RX (PFF frame)
static uint8_t         modem_reply_frame[MODEM_REPLY_FRAME_BYTES];
static uint8_t         modem_rx_window[MODEM_REPLY_FRAME_BYTES];
static uint8_t         modem_rx_window_count = 0;
static bool            modem_frame_ready = false;
static pff_rx_packet_t modem_rx_pkt = {0};

// SD card / FileX
static bool sd_ready = false;



/*----------------------------------------------------------------------------*/
/* Scale factors -------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

// Seabird37 scales (8-bit)
static const scale_t temp_scale   = { .min = -2.0f,  .max = 25.0f  };   // °C
static const scale_t cond_scale   = { .min = 3.0f,   .max = 5.5f   };   // S/m
static const scale_t pres_scale   = { .min = 0.0f,   .max = 30.0f };   // dbar ~ 1 dbar per meter
static const scale_t sal_scale    = { .min = 32.0f,  .max = 37.0f  };   // PSU
static const scale_t spcond_scale = { .min = 3.0f,  .max = 5.5f  };   // S/m

// Signature500 8-bit scales
static const scale_t sig_battery_scale     = { .min = 12.0f,    .max = 48.0f  };   // V
static const scale_t sig_sound_speed_scale = { .min = 1400.0f, .max = 1600.0f };  // m/s
static const scale_t sig_heading_scale     = { .min = 0.0f,    .max = 360.0f };   // deg
static const scale_t sig_pitch_scale       = { .min = -90.0f,  .max = 90.0f  };   // deg
static const scale_t sig_roll_scale        = { .min = -180.0f, .max = 180.0f };   // deg

// Scales for PNORC-values (Signature500)
static const scale_t sig_cellpos_scale     = { .min = 0.0f,    .max = 30.0f  };   // "cell position"
static const scale_t sig_speed_scale       = { .min = 0.0f,    .max = 5.0f   };   // m/s
static const scale_t sig_direction_scale   = { .min = 0.0f,    .max = 360.0f };   // deg

// Scale for standard deviation on print
static const scale_t pres_std_scale = { .min = 0.0f, .max = 10.0f };  // dbar

// Scale for chamber temperature
static const scale_t chamber_temp_scale = { .min = -10.0f, .max = 50.0f };  // °C


/*----------------------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/*----------------------------------------------------------------------------*/

// POWER/CLOCK/INIT
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void Activate_GPIO_Pins(void);
static void Deactivate_GPIO_Pins(void);
static void Blink_Led(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static bool MX_SDMMC1_SD_Init(void);
static void MX_ICACHE_Init(void);
static void MX_ADC1_Init(void);


// App / state machine
static void app_init(void);
static void app_set_state(app_state_t s);
static void app_step(void);

// Sleep / RTC
static void MX_RTC_Init(void);
static void EnterSleep(uint32_t seconds);

// Sensor I/O
static void sensor_uart_service(void);
static void sensor_capture_reset(void);
static bool seabird_poll_start(void);
static bool seabird_send_cmd(const char *cmd);
static bool sig500_poll_start(void);
static bool sig500_send_cmd(const char *cmd);
static bool wakeup_sig500_break(void);
static void sensor_acquire_kick_timeout(void);

// Modem I/O
static void modem_uart_service(void);
static void modem_reply_reset(void);
static void modem_send_step(void);
static bool modem_q_push(const uint16_t words5[PFF_WORDS], frame_src_t src);
static bool modem_q_pop(uint16_t out_words5[PFF_WORDS], frame_src_t *src);
static void modem_reply_frame_reset(void);
static void uart_clear_rx_errors_and_drain(UART_HandleTypeDef *huart);
static bool handle_shell_command(const uint8_t data5[PFF_DATA_BYTES]);
static bool handle_extended_command(const uint8_t data5[PFF_DATA_BYTES]);

// Cycle data / parsing
static void cycle_data_reset(void);
static bool parse_seabird_line_values(const char *line, float *temp, float *cond, float *pres, float *sal, float *spcond);
static void store_seabird_measurement(float temp, float cond, float pres, float sal, float spcond);
static void sig500_raw_reset(sig500_raw_packet_t *pkt);
static sig500_collect_result_t sig500_collect_line(sig500_raw_packet_t *pkt, const char *line);
static bool sig500_parse_raw_packet(const sig500_raw_packet_t *pkt);
static bool parse_floats_after_prefix(const char *line, const char *prefix,
                                      float *out, int n);
static bool parse_sig_pnors(const char *line, sig_pnors_t *dst);
static bool parse_sig_pnorc(const char *line, sig_pnorc_t *dst);

// PFF / packing
static uint8_t scale_to_u8(float x, float in_min, float in_max);
static float clampf(float x, float lo, float hi);
static void pff_write_bits(uint8_t *dst, uint16_t *bitpos, uint32_t value, uint8_t nbits);
static uint8_t pff_checksum_xor(const uint8_t *bytes, uint8_t len);
static void pff_build(uint8_t version, uint8_t msg_id, uint32_t time22,
                      const uint8_t data5[PFF_DATA_BYTES],uint8_t out10[PFF_TOTAL_BYTES]);
static void pff_to_words_be(const uint8_t msg10[PFF_TOTAL_BYTES], uint16_t words[PFF_WORDS]);
static bool pff_parse(const uint8_t in10[PFF_TOTAL_BYTES],
                      pff_rx_packet_t *out);
static uint32_t pff_read_bits(const uint8_t *src, uint16_t *bitpos, uint8_t nbits);
static bool seabird_pack_latest(seabird_u8_t *out);
static bool sig500_pack_send1(uint8_t data5[5]);
static bool sig500_pack_send2(uint8_t data5[5]);
static bool sig500_pack_send3(uint8_t data5[5]);
static bool sig500_queue_frames(void);
static void seabird_queue_pressure_stats_frame(void);

// Stats / debug
static uint16_t ring_start_index(uint16_t write_pos, uint16_t count);
static float mean_ring(const float *buf, uint16_t count, uint16_t write_pos);
static void min_max_ring(const float *buf, uint16_t count, uint16_t write_pos,
                         float *min_out, float *max_out);
static float std_avvik_ring(const float *buf, uint16_t count, uint16_t write_pos);

// SD / FileX
static bool MX_SDMMC1_SD_Init(void);
static bool SD_init_and_open(void);
static bool SD_create_file_and_header_if_needed(const CHAR *filename, const char *header);
static bool SD_write_seabird_record(void);
static bool SD_write_sig500_record(void);
static bool SD_append_line(const CHAR *filename, const char *line);
static bool SD_power_on(void);
static void SD_power_down(void);

// ADC / Temperature measurement
static uint32_t TMP35_ReadAdcRaw(void);
static float TMP35_AdcToVoltage(uint32_t adc_raw);
static float TMP35_VoltageToCelsius(float voltage);
static float TMP35_ReadTemperatureC(void);

// Time
static bool is_leap_year(uint16_t year);
static uint8_t days_in_month(uint16_t year, uint8_t month);
static bool rtc_get_datetime(rtc_datetime_t *dt);
static uint32_t rtc_get_pff_time22(void);
static void rtc_format_timestamp(char *out, size_t out_size);



/*----------------------------------------------------------------------------*/
/* Main function -------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

int main(void)
{
  HAL_Init();

  SystemPower_Config();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  // MX_SDMMC1_SD_Init();
  MX_FileX_Init();
  MX_ICACHE_Init();

  sd_ready = false;

  app_init();

  while (1)
  {
    if (app.state == ST_WAKE_SIG500 || app.state == ST_SENSOR_ACQUIRE)
    {
      sensor_uart_service();
    }

    if (app.state == ST_WAIT_REPLY || app.state == ST_HANDLE_REPLY)
    {
      modem_uart_service();
    }

    app_step();
  }
}



/*----------------------------------------------------------------------------*/
/* Function implementations --------------------------------------------------*/
/*----------------------------------------------------------------------------*/

// Initilizes app state and variables
static void app_init(void)
{
  memset(&app, 0, sizeof(app));
  app_set_state(ST_BOOT);
}


// Set state and initialize state variables
static void app_set_state(app_state_t s)
{
  app.state = s;
  app.state_enter_ms = HAL_GetTick();
  app.deadline_ms = 0;
  app.state_started = false;
}

// Main state machine for the application. One "step" each time the function is called.
// States: Sleep, Sensor acquire, Process/store, Modem send, Wait reply, Handle reply, SD store, Error
static void app_step(void)
{
  uint32_t now = HAL_GetTick();

  switch (app.state)
  {
    // Boot state, goes directly to sensor acquire to start first cycle.
    case ST_BOOT:
    {
      if (!app.state_started)
      {
        app.state_started = true;

        app.cycle_no = 1;                         	// Start cycle number at 1

        Blink_Led();								// Blink Led to confirm program started

        app_set_state(ST_WAKE_SIG500);
      }
      break;
    }

    // Sleep state, low power mode to save battery
    case ST_SLEEP:
    {
      if (!app.state_started)
      {
        app.state_started = true;

        // Reset everything before we sleep, to ensure a clean state for the next cycle and to save power
        sensor_capture_reset();
        modem_reply_reset();
        SD_power_down();
        Deactivate_GPIO_Pins();

        // Enter Sleep for set interval
        EnterSleep(sleep_interval_s);

        // Add one to cycle number and go to wake sig500 to start next cycle
        app.cycle_no++;

        app_set_state(ST_WAKE_SIG500);          // Go to wake sig500 to start next cycle
      }
      break;
    }

    // Wake Signature500 state, send break and wakeup, wait for OK, go to acquire if ok, sleep if not
    case ST_WAKE_SIG500:
    {
      if (!app.state_started)
      {
        app.state_started = true;

        // Activate all GPIO pins
        Activate_GPIO_Pins();

        // Give time for all periferals to wake
        HAL_Delay(500);

        // Turn on SD card
        if (!sd_ready)
        {
          (void)SD_power_on();
        }

        // Reset sensor capture state to be sure we're in a clean state
        sensor_capture_reset();
        // Reset Signature500 raw collection state
        sig500_raw_reset(&sig500_collect_pkt);
        app.sig500_wakeup_ok = false;

        // Clear RX buffer for "OK" from Signature500
        uart_clear_rx_errors_and_drain(SIG500_UART_HANDLE);
        HAL_Delay(100);

        // Send break and wakeup command to Signature500
        if (!wakeup_sig500_break())
        {
          app_set_state(ST_SLEEP);
          break;
        }

        app.deadline_ms = HAL_GetTick() + SIG500_WAKE_ACK_TIMEOUT_MS;
      }

      // Check for Signature500 wakeup response, if we get it, move on to sensor acquire, if we timeout, go to sleep
      if (app.sig500_wakeup_ok)
      {
        app_set_state(ST_SENSOR_ACQUIRE);
      }
      else if ((int32_t)(now - app.deadline_ms) >= 0)
      {
        app_set_state(ST_SLEEP);
      }
      break;
    }

    // Sensor acquire state, poll sensor data, go to process and store if data is acquired, sleep if not.
    case ST_SENSOR_ACQUIRE:
    {
      if (!app.state_started)
      {
        app.state_started = true;

        // Small delay
        HAL_Delay(100);

        // Reset cycle data and sensor capture state to be sure we're in a clean state for the acquire step
        cycle_data_reset();
        sensor_capture_reset();
        sig500_raw_reset(&sig500_collect_pkt);

        // Start polling sensors, if we fail to start polling for any reason, go to sleep and start next cycle
        if (!seabird_poll_start())
        {
          app_set_state(ST_SLEEP);
          break;
        }

        if (!sig500_poll_start())
        {
          app_set_state(ST_SLEEP);
          break;
        }

        sensor_acquire_kick_timeout();
      }

      bool seabird_done = cycle_data.seabird_raw_ready;
      bool sig500_done = cycle_data.sig500_raw_ready;

      // If both datasets are ready, move on to process and store, if we timeout while waiting for data, go to sleep and start next cycle
      if (seabird_done && sig500_done)
      {
        app_set_state(ST_PROCESS_AND_STORE);
      }

      else if ((int32_t)(now - app.deadline_ms) >= 0)
      {
        app_set_state(ST_SLEEP);
      }

      break;
    }

    // Process and store state, parse sensor data, read chamber temperature,
    // and queue data for modem. Go to modem send if valid data exists,
    // otherwise go to sleep.
    case ST_PROCESS_AND_STORE:
    {
      if (!app.state_started)
      {
        app.state_started = true;

        // If there is no raw data to process, go to sleep and start next cycle.
        if (!cycle_data.seabird_raw_ready && !cycle_data.sig500_raw_ready)
        {
          app_set_state(ST_SLEEP);
          break;
        }

        // Read chamber temperature once per cycle if there is sensor data.
        cycle_data.chamber_temp_c = TMP35_ReadTemperatureC();

        // -------------------- Seabird -------------------- //
        if (cycle_data.seabird_raw_ready)
        {
          if (parse_seabird_line_values(cycle_data.seabird_raw_line,
                                        &cycle_data.temp,
                                        &cycle_data.cond,
                                        &cycle_data.pres,
                                        &cycle_data.sal,
                                        &cycle_data.spcond))
          {
            cycle_data.seabird_valid = true;

            // Store measurement in ring buffer for statistics.
            store_seabird_measurement(cycle_data.temp,
                                      cycle_data.cond,
                                      cycle_data.pres,
                                      cycle_data.sal,
                                      cycle_data.spcond);

            seabird_u8_t packed;

            // Pack the latest Seabird data.
            if (seabird_pack_latest(&packed))
            {
              // Build PFF data bytes from packed Seabird data.
              uint8_t data5[PFF_DATA_BYTES] = {
                packed.t, packed.c, packed.p, packed.s, packed.sc
              };

              // Prepare buffers for PFF message and modem queue.
              uint8_t  pff_msg[PFF_TOTAL_BYTES];
              // Buffer for PFF message converted to 16-bit words for modem queue.
              uint16_t pff_words[PFF_WORDS];

              //PFF version, message ID and time for Seabird raw data frame.
              uint8_t  version = 1;
              uint8_t  msg_id  = PFF_MSGID_SEABIRD_RAW;
              uint32_t time22  = rtc_get_pff_time22();

              // Build PFF message and convert to words for modem queue.
              pff_build(version, msg_id, time22, data5, pff_msg);
              // Convert PFF message bytes to 16-bit words
              pff_to_words_be(pff_msg, pff_words);

              // Queue the PFF message words for modem transmission.
              (void)modem_q_push(pff_words, FRAME_SEABIRD_RAW);
            }

            // Queue pressure statistics every WINDOW_SIZE measurements.
            if (window_count == WINDOW_SIZE &&
                (total_samples - last_stats_sent_total) >= WINDOW_SIZE)
            {
              // Queue pressure statistics for transmission.
              last_stats_sent_total = total_samples;
              seabird_queue_pressure_stats_frame();
            }
          }
        }

        // -------------------- Signature500 -------------------- //
        if (cycle_data.sig500_raw_ready)
        {
          // Try to parse the raw Signature500 data
          if (sig500_parse_raw_packet(&cycle_data.sig500_raw))
          {
            cycle_data.sig500_valid = true;

            // If parsing is successful, pack and queue the three PNORC bursts for modem transmission.
            (void)sig500_queue_frames();
          }
        }

        // If neither dataset parsed successfully, go to sleep.
        if (!cycle_data.seabird_valid && !cycle_data.sig500_valid)
        {
          app_set_state(ST_SLEEP);
          break;
        }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_SET);    // Modem power on
        HAL_Delay(500);
        app_set_state(ST_MODEM_SEND);
      }
      break;
    }

    // Modem send state, send everything in the line, 16 bits at a time.
    case ST_MODEM_SEND:
    {
      // If we haven't started sending yet, set deadline for send timeout
      if (!app.state_started)
      {
        // Set state started to true and set deadline for send timeout
        app.state_started = true;
        app.deadline_ms = now + MODEM_SEND_TIMEOUT_MS;
      }

      // Send one step every time we get here, when everything is sent tx_active=false and modem_q_count=0
      modem_send_step();

      if (!tx_active && modem_q_count == 0)
      {
        // Reset BEFORE we enter wait-reply
        modem_reply_reset();
        app.modem_rx_enable_ms = HAL_GetTick() + MODEM_RX_GUARD_MS;
        app_set_state(ST_WAIT_REPLY);
      }
       // We've waited too long to send everything to modem, so move on to waiting for interval for next cycle
      else if ((int32_t)(now - app.deadline_ms) >= 0)
      {
        app_set_state(ST_SLEEP);
      }

      break;
    }

    // Wait reply state, waits for a reply from modem, handle reply if it comes, go to sd store if not
    case ST_WAIT_REPLY:
    {
      // Enable modem RX after guard time to avoid picking up any remaining bits from our send
      if (!app.state_started)
      {
        app.state_started = true;
        app.deadline_ms = HAL_GetTick() + MODEM_REPLY_WINDOW_MS;
      }

      // If we get a modem frame, move on to handle reply, if we timeout while waiting for reply, move on to SD store and start next cycle
      if (modem_frame_ready)
      {
        modem_frame_ready = false;
        app_set_state(ST_HANDLE_REPLY);
      }
      else if ((int32_t)(now - app.deadline_ms) >= 0)
      {
        app_set_state(ST_SD_STORE);
      }

      break;
    }

    // Handle reply state, handles reply from modem with updated parameters
    case ST_HANDLE_REPLY:
    {
      if (!app.state_started)
      {
        app.state_started = true;

        // Try to parse modem reply as PFF frame.
        // If parsing fails, ignore the reply and continue to SD store.
        if (!pff_parse(modem_reply_frame, &modem_rx_pkt))
        {
          app_set_state(ST_SD_STORE);
          break;
        }

        // Handle different PFF reply message IDs.
        switch (modem_rx_pkt.msg_id)
        {
          case PFF_MSGID_CMD_SHELL:
          {
            // Shell command received. Handle it accordingly.
            (void)handle_shell_command(modem_rx_pkt.data);
            break;
          }

          case PFF_MSGID_CMD_EXTENDED:
          {
            // Extended command received. Handle it accordingly.
            (void)handle_extended_command(modem_rx_pkt.data);
            break;
          }

          default:
          {
            // Unknown message ID. Ignore and continue.
            break;
          }
        }

        app_set_state(ST_SD_STORE);
      }
      break;
    }

    // SD store state, saves acquired sensor data to SD card
    case ST_SD_STORE:
    {
      if (!app.state_started)
      {
        app.state_started = true;

        // Store Seabird data if valid.
        if (cycle_data.seabird_valid)
        {
          (void)SD_write_seabird_record();
        }

        // Store Signature500 data if valid.
        if (cycle_data.sig500_valid)
        {
          (void)SD_write_sig500_record();
        }

        app_set_state(ST_SLEEP);
      }
      break;
    }

    // Error state, if at any time there's an error, set controller to sleep.
    case ST_ERROR:
    default:
    {
      if (!app.state_started)
      {
        app.state_started = true;
        app_set_state(ST_SLEEP);
      }
      break;
    }
  }
}

/*------------------------------------------------------------------------*/
/*Sleep / RTC-------------------------------------------------------------*/
/*------------------------------------------------------------------------*/


// Enter STOP3 sleep for a given number of seconds using RTC wake-up.
static void EnterSleep(uint32_t seconds)
{
  if (seconds == 0)
    seconds = 1;

  // Deactivate wakeup timer in case it's still active from before
  if (HAL_RTCEx_DeactivateWakeUpTimer(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  uint32_t counter = seconds - 1;

  /*
   * Set RTC wakeup timer while HAL tick still runs.
   */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
                                  counter,
                                  RTC_WAKEUPCLOCK_CK_SPRE_16BITS,
                                  0) != HAL_OK)
  {
    Error_Handler();
  }

  // HAL tick will be suspended, and will automatically resume on wakeup.
  HAL_SuspendTick();

  // Enter STOP3 mode
  HAL_PWREx_EnterSTOP3Mode(PWR_STOPENTRY_WFI);

  // Resume HAL tick after wakeup
  HAL_ResumeTick();

  SystemClock_Config();

  HAL_Delay(1);

  if (HAL_RTCEx_DeactivateWakeUpTimer(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  // Re-initialize peripherals that we use after sleep
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
}

/*------------------------------------------------------------------------*/
/*Sensor I/O--------------------------------------------------------------*/
/*------------------------------------------------------------------------*/

// Check if we have the needed data for pack 1 (PNORS) and fill data5 if yes
#if !SENSOR_SINGLE_UART_TEST
static void sensor_uart_service(void)
{
  // ---------------- USART1 : Seabird ---------------- //
  if (app.state == ST_SENSOR_ACQUIRE)
  {
    while (HAL_UART_Receive(&huart1, &seabirdRxByte, 1, 0) == HAL_OK)
    {
      sensor_acquire_kick_timeout();

      // If we get a newline, try to parse the line, if we get a full line, check if it starts with #, if it does, store it as raw line for SD and print, if not, ignore.
      if (seabirdRxByte == '\r' || seabirdRxByte == '\n')
      {
        if (seabirdLineLen > 0)
        {
          // Null-terminate the line buffer to safely use it as a string.
          seabirdLineBuf[seabirdLineLen] = '\0';

          // Copy the line buffer to seabirdLine for processing and printing, ensuring null-termination.
          strncpy(seabirdLine, (const char *)seabirdLineBuf, LINE_BUF_SIZE - 1);
          // Ensure null-termination in case of overflow in strncpy
          seabirdLine[LINE_BUF_SIZE - 1] = '\0';

          // Reset line buffer for next line
          seabirdLineLen = 0;
          seabirdLineBuf[0] = '\0';

          if (seabirdLine[0] == '#')
          {
            if (!cycle_data.seabird_raw_ready)
            {
              // Store the raw line in cycle data for SD card storage and potential debugging, ensuring null-termination.
              strncpy(cycle_data.seabird_raw_line, seabirdLine, LINE_BUF_SIZE - 1);
              cycle_data.seabird_raw_line[LINE_BUF_SIZE - 1] = '\0';
              cycle_data.seabird_raw_ready = true;
            }
          }
        }
      }
      // If we get any other character, add to line buffer until we get a full line, if we overflow the buffer, reset it and print a warning.
      else
      {
        if (seabirdLineLen < (LINE_BUF_SIZE - 1))
        {
          seabirdLineBuf[seabirdLineLen++] = seabirdRxByte;
        }
        else
        {
          seabirdLineLen = 0;
          seabirdLineBuf[0] = '\0';
        }
      }
    }
  }

  // ---------------- USART2 : Signature500 ---------------- //
  while (HAL_UART_Receive(&huart2, &sig500RxByte, 1, 0) == HAL_OK)
  {
    if (app.state == ST_SENSOR_ACQUIRE)
    {
      sensor_acquire_kick_timeout();
    }

    // If we get a newline, try to parse the line, if we get a full line, check if it starts with $, if it does, try to collect it as part of the raw packet, if not, ignore.
    if (sig500RxByte == '\r' || sig500RxByte == '\n')
    {
      if (sig500LineLen > 0)
      {
        // Null-terminate the line buffer to safely use it as a string.
        sig500LineBuf[sig500LineLen] = '\0';

        // Copy the line buffer to sig500Line for processing, ensuring null-termination.
        strncpy(sig500Line, (const char *)sig500LineBuf, LINE_BUF_SIZE - 1);
        // Ensure null-termination in case of overflow in strncpy
        sig500Line[LINE_BUF_SIZE - 1] = '\0';

        // Reset line buffer for next line
        sig500LineLen = 0;
        sig500LineBuf[0] = '\0';

        // Check for OK response from Signature500, which indicates it has woken up and is ready. We only care about this in the wakeup state, in acquire state we care about the data lines that start with $.
        if (strcmp(sig500Line, "OK") == 0)
        {
          if (app.state == ST_WAKE_SIG500)
          {
            app.sig500_wakeup_ok = true;
          }
          continue;
        }

        // Under wake-state we only care aboutq OK //
        if (app.state != ST_SENSOR_ACQUIRE)
        {
          continue;
        }

        // For acquire state, we care about all lines that start with $ because that's where the data is, and we need to collect multiple lines to get the complete packet.
        if (sig500Line[0] == '$')
        {
          // Try to collect the line as part of the raw packet. If we get a complete packet, store it in cycle data for SD card storage
          sig500_collect_result_t r = sig500_collect_line(&sig500_collect_pkt, sig500Line);

          // If we got a complete packet, store it in cycle data if we don't already have one, and reset the collector for the next packet
          if (r == SIG500_COLLECT_COMPLETE)
          {
            // If we don't already have a raw packet ready, store the collected packet in cycle data for SD card storage. If we already have a packet ready, we ignore this one since we only have storage for one raw packet per cycle.
            if (!cycle_data.sig500_raw_ready)
            {
              // Store the collected raw packet in cycle data for SD card storage.
              cycle_data.sig500_raw = sig500_collect_pkt;
              cycle_data.sig500_raw_ready = true;
            }

            // Reset the collector for the next packet.
            sig500_raw_reset(&sig500_collect_pkt);
          }
          // If we got an error while collecting, reset the collector to start fresh on the next line.
          else if (r == SIG500_COLLECT_ERROR)
          {
            sig500_raw_reset(&sig500_collect_pkt);
          }
        }
      }
    }
    // If we get any other character, add to line buffer until we get a full line, if we overflow the buffer, reset it and print a warning.
    else
    {
      // We add all lines to the buffer, even if they don't start with $, because we need to collect multiple lines to get the complete packet, and not all lines start with $.
      if (sig500LineLen < (LINE_BUF_SIZE - 1))
      {
        // Add character to line buffer and increment length
        sig500LineBuf[sig500LineLen++] = sig500RxByte;
      }
      else
      {
        // If we overflow the buffer, reset it.
        sig500LineLen = 0;
        sig500LineBuf[0] = '\0';
      }
    }
  }
}

#endif

#if SENSOR_SINGLE_UART_TEST

static void sensor_uart_service(void)
{
  while (HAL_UART_Receive(SENSOR_TEST_UART_HANDLE, &sensorRxByte, 1, 0) == HAL_OK)
  {
    // Keep sensor acquire timeout alive while data is arriving
    if (app.state == ST_SENSOR_ACQUIRE)
    {
      sensor_acquire_kick_timeout();
    }

    // End of line received
    if (sensorRxByte == '\r' || sensorRxByte == '\n')
    {
      if (sensorLineLen > 0)
      {
        // Null-terminate line
        sensorLineBuf[sensorLineLen] = '\0';

        strncpy(sensorLine, (const char *)sensorLineBuf, LINE_BUF_SIZE - 1);
        sensorLine[LINE_BUF_SIZE - 1] = '\0';

        // Reset line buffer for next line
        sensorLineLen = 0;
        sensorLineBuf[0] = '\0';

        // ------------------------------------------------------------
        // Signature500 wake OK
        // ------------------------------------------------------------
        if (strcmp(sensorLine, "OK") == 0)
        {
          if (app.state == ST_WAKE_SIG500)
          {
            app.sig500_wakeup_ok = true;
          }

          // OK during ST_SENSOR_ACQUIRE can be ignored
          continue;
        }

        // During wake state, only OK is relevant
        if (app.state != ST_SENSOR_ACQUIRE)
        {
          continue;
        }

        // ------------------------------------------------------------
        // Seabird line: starts with '#'
        // ------------------------------------------------------------
        if (sensorLine[0] == '#')
        {
          // If we don't already have a raw line ready, store the line in cycle data for SD card storage and potential debugging.
          if (!cycle_data.seabird_raw_ready)
          {
            strncpy(cycle_data.seabird_raw_line, sensorLine, LINE_BUF_SIZE - 1);
            cycle_data.seabird_raw_line[LINE_BUF_SIZE - 1] = '\0';
            cycle_data.seabird_raw_ready = true;
          }

          continue;
        }

        // ------------------------------------------------------------
        // Signature500 lines: starts with '$'
        // ------------------------------------------------------------
        if (sensorLine[0] == '$')
        {
          sig500_collect_result_t r =
              sig500_collect_line(&sig500_collect_pkt, sensorLine);

              // If we got a complete packet, store it in cycle data if we don't already have one, and reset the collector for the next packet
          if (r == SIG500_COLLECT_COMPLETE)
          {
            if (!cycle_data.sig500_raw_ready)
            {
              cycle_data.sig500_raw = sig500_collect_pkt;
              cycle_data.sig500_raw_ready = true;
            }

            sig500_raw_reset(&sig500_collect_pkt);
          }
          // If we got an error while collecting, reset the collector to start fresh on the next line.
          else if (r == SIG500_COLLECT_ERROR)
          {
            sig500_raw_reset(&sig500_collect_pkt);
          }

          continue;
        }

        // Other lines are ignored
      }
    }
    else
    {
      // Add character to line buffer
      if (sensorLineLen < (LINE_BUF_SIZE - 1))
      {
        sensorLineBuf[sensorLineLen++] = sensorRxByte;
      }
      else
      {
        // Overflow protection
        sensorLineLen = 0;
        sensorLineBuf[0] = '\0';
      }
    }
  }
}

#endif

// Reset variables related to sensor line collection, so we are ready for a new cycle.
static void sensor_capture_reset(void)
{
  seabirdLineLen = 0;
  seabirdLineBuf[0] = '\0';
  seabirdLine[0] = '\0';

  sig500LineLen = 0;
  sig500LineBuf[0] = '\0';
  sig500Line[0] = '\0';

#if SENSOR_SINGLE_UART_TEST
  sensorLineLen = 0;
  sensorLineBuf[0] = '\0';
  sensorLine[0] = '\0';
#endif
}

// Send poll command to seabird to start measurement and data output. Returns true if sent successfully, false if error.
static bool seabird_poll_start(void)
{
  return seabird_send_cmd(SEABIRD_POLL_CMD);
}

// Send a command to Seabird. Returns true if sent successfully, false if error.
static bool seabird_send_cmd(const char *cmd)
{
  if (!cmd) return false;

  //Transmit command to Seabird over UART
  HAL_StatusTypeDef st = HAL_UART_Transmit(SEABIRD_UART_HANDLE,
                                           (uint8_t *)cmd,
                                           (uint16_t)strlen(cmd),
                                           SEABIRD_POLL_TX_TIMEOUT_MS);

  return (st == HAL_OK);
}

// Send poll command to Signature500 to start measurement and data output. Returns true if successfull, false if error.
static bool sig500_poll_start(void)
{
  return sig500_send_cmd(SIG500_POLL_CMD);
}

// Send a command to Signature500. Returns true if sent successfully, false if error.
static bool sig500_send_cmd(const char *cmd)
{
  if (!cmd) return false;

  // Transmit command to Signature500 over UART
  HAL_StatusTypeDef st = HAL_UART_Transmit(SIG500_UART_HANDLE,
                                           (uint8_t *)cmd,
                                           (uint16_t)strlen(cmd),
                                           SIG500_POLL_TX_TIMEOUT_MS);

  return (st == HAL_OK);
}

// Send the break/wakeup sequence to Signature500. Returns true if all sent successfully, false if error.
static bool wakeup_sig500_break(void)
{
  // The wake-up sequence of Signature500. Plus set delays.
  static const char *wakeup_string_1 = "@@@@@@";
  static const char *wakeup_string_2 = "K1W%!Q";
  static const char *wakeup_string_3 = "K1W%!Q";

  if (HAL_UART_Transmit(SIG500_UART_HANDLE,
                        (uint8_t *)wakeup_string_1,
                        (uint16_t)strlen(wakeup_string_1),
                        SIG500_WAKE_TX_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  HAL_Delay(SIG500_WAKE_DELAY_1_MS);

  if (HAL_UART_Transmit(SIG500_UART_HANDLE,
                        (uint8_t *)wakeup_string_2,
                        (uint16_t)strlen(wakeup_string_2),
                        SIG500_WAKE_TX_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  HAL_Delay(SIG500_WAKE_DELAY_2_MS);

  if (HAL_UART_Transmit(SIG500_UART_HANDLE,
                        (uint8_t *)wakeup_string_3,
                        (uint16_t)strlen(wakeup_string_3),
                        SIG500_WAKE_TX_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  return true;
}

// Call this to kick the sensor acquire timeout, so we don't timeout while waiting for data as long as data is coming in.
static void sensor_acquire_kick_timeout(void)
{
  app.deadline_ms = HAL_GetTick() + SENSOR_ACQUIRE_TIMEOUT_MS;
}

/*------------------------------------------------------------------------*/
/*Modem I/O---------------------------------------------------------------*/
/*------------------------------------------------------------------------*/

// Check if a newline has arrived from the modem, and if so, handle it. We are only interested in this when we are waiting for or handling a modem response, so this function is only called in those states.
static void modem_uart_service(void)
{
  while (HAL_UART_Receive(&huart3, &modemRxByte, 1, 0) == HAL_OK)
  {
    uint32_t now = HAL_GetTick();

    // Discard bytes in guard period right after TX //
    if ((int32_t)(now - app.modem_rx_enable_ms) < 0)
    {
      continue;
    }

    // Fill window until full, then slide 1 byte at a time.
    if (modem_rx_window_count < MODEM_REPLY_FRAME_BYTES)
    {
      modem_rx_window[modem_rx_window_count++] = modemRxByte;
    }
    // Window is full, slide and add new byte at the end.
    else
    {
      memmove(&modem_rx_window[0],
              &modem_rx_window[1],
              MODEM_REPLY_FRAME_BYTES - 1);
      modem_rx_window[MODEM_REPLY_FRAME_BYTES - 1] = modemRxByte;
    }

    // Try parse only when 10 bytes are available //
    if (modem_rx_window_count >= MODEM_REPLY_FRAME_BYTES)
    {
      pff_rx_packet_t tmp;

      // Try to parse the current window as a PFF packet. If parsing is successful, copy the frame to modem_reply_frame for later processing, store the parsed packet in modem_rx_pkt, and set modem_frame_ready to true so that we can handle it in the main loop.
      if (pff_parse(modem_rx_window, &tmp))
      {
        memcpy(modem_reply_frame, modem_rx_window, MODEM_REPLY_FRAME_BYTES);
        modem_rx_pkt = tmp;
        modem_frame_ready = true;

        return;
      }
    }
  }
}

// Reset variables related to collecting and handling modem responses, so we are ready for a new cycle.
static void modem_reply_reset(void)
{
  modem_reply_frame_reset();

  app.modem_reply_ready = false;
  app.modem_reply_valid = false;

  cycle_data.reply_ready = false;
  cycle_data.reply_line[0] = '\0';
}

// Call this often (in the main loop) to handle timing of modem transmissions.
static void modem_send_step(void)
{
  uint32_t now = HAL_GetTick();

  // Pace: do not send before the time is up
  if ((int32_t)(now - next_tx_ms) < 0) return;

  // If we don't have an active frame, get the next one from the queue
  if (!tx_active)
  {
    frame_src_t src;
    if (!modem_q_pop(tx_words, &src))
    {
      return; // nothing to send
    }

    tx_word_idx = 0;
    tx_active = true;

  }

  // Send 1 word (16-bit) as 2 bytes, HI first
  uint16_t w = tx_words[tx_word_idx++];

  // Convert the 16-bit word into two bytes in big-endian order
  uint8_t b[2] = { (uint8_t)(w >> 8), (uint8_t)(w & 0xFF) };
  // Transmit the two bytes over UART to the modem
  (void)HAL_UART_Transmit(&huart3, b, 2, 50);


  // Next word in 1.65sec
  next_tx_ms = now + MODEM_WORD_INTERVAL_MS;

  // Done with this frame?
  if (tx_word_idx >= PFF_WORDS)
  {
    tx_active = false;
  }
}

// Push one modem frame into the circular transmit queue.
// Returns false if the queue is full.
static bool modem_q_push(const uint16_t words5[PFF_WORDS], frame_src_t src)
{
  // Do not overwrite existing frames if the queue is full.
  if (modem_q_count >= MODEM_Q_MAX) return false;

  // Copy the five 16-bit PFF words into the next free queue position.
  for (int i = 0; i < PFF_WORDS; i++)
    modem_q[modem_q_tail].words[i] = words5[i];

  // Store the frame source/type.
  modem_q[modem_q_tail].src = src;

  // Move the tail index to the next position in the ring buffer, wrapping around if necessary.
  modem_q_tail = (uint16_t)((modem_q_tail + 1u) % MODEM_Q_MAX);

  // Increase the number of frames currently stored in the queue.
  modem_q_count++;
  return true;
}


// Pop the oldest frame from the queue. Returns false if the queue is empty. If src!=NULL, write the source to *src.
static bool modem_q_pop(uint16_t out_words5[PFF_WORDS], frame_src_t *src)
{
  if (modem_q_count == 0) return false;

  // Copy the five 16-bit words of the oldest frame in the queue to the output buffer.
  for (int i = 0; i < PFF_WORDS; i++)
    out_words5[i] = modem_q[modem_q_head].words[i];

  // If the caller provided a non-NULL pointer for the source, write the source/type of the frame to that location.
  if (src) *src = modem_q[modem_q_head].src;

  // Move the head index to the next position in the ring buffer, wrapping around if necessary.
  modem_q_head = (uint16_t)((modem_q_head + 1u) % MODEM_Q_MAX);

  // Decrease the number of frames currently stored in the queue.
  modem_q_count--;
  return true;
}

// Reset the modem reply frame assembly state and clear any partially received packet data.
static void modem_reply_frame_reset(void)
{
  memset(modem_reply_frame, 0, sizeof(modem_reply_frame));
  memset(modem_rx_window, 0, sizeof(modem_rx_window));
  modem_rx_window_count = 0;
  modem_frame_ready = false;
  memset(&modem_rx_pkt, 0, sizeof(modem_rx_pkt));
}

// This stopped OK from signature500 from getting parsed, RX wasn't "clean" after wake-up
static void uart_clear_rx_errors_and_drain(UART_HandleTypeDef *huart)
{
  if (!huart) return;

// Clears UART flags so it wakes up OK.
#if defined(UART_CLEAR_OREF)
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
#endif

#if defined(UART_CLEAR_FEF)
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
#endif

#if defined(UART_CLEAR_NEF)
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);
#endif

#if defined(UART_CLEAR_PEF)
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);
#endif

#if defined(UART_CLEAR_IDLEF)
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
#endif

  // Resets internal HAL error
  huart->ErrorCode = HAL_UART_ERROR_NONE;


  //Drain old RX data
#if defined(UART_FLAG_RXNE_RXFNE)
  while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE_RXFNE) != RESET)
#else
  while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
#endif
  {
    volatile uint8_t dummy = (uint8_t)huart->Instance->RDR;
    (void)dummy;
  }

  // Resets interval HAL error once more.
  huart->ErrorCode = HAL_UART_ERROR_NONE;
}


// Handle a shell command contained in a 5-byte payload. Returns true if the command
// was recognized and processed successfully, otherwise false.
static bool handle_shell_command(const uint8_t data5[PFF_DATA_BYTES])
{
  // Check that data5 is not NULL before accessing it
  if (!data5) return false;

  // The first byte of data5 is the command ID, the remaining bytes are parameters.
  uint8_t cmd = data5[0];

  // Handle different command IDs using a switch statement.
  switch (cmd)
  {
    // Example command: Set sleep interval in seconds.
    case CMD_SET_SLEEP_INTERVAL_S:
    {
      // The new sleep interval is contained in data5[1] and data5[2] as a big-endian 16-bit unsigned integer.
      // Check that the new sleep interval is within allowed bounds before setting it. If it's out of bounds, return false to indicate an error.
      uint16_t new_sleep = ((uint16_t)data5[1] << 8) | data5[2];

      if (new_sleep < MIN_SLEEP_INTERVAL_S || new_sleep > MAX_SLEEP_INTERVAL_S)
      {
        return false;
      }

      // Set the new sleep interval for the next cycles.
      sleep_interval_s = (uint32_t)new_sleep;
      return true;
    }
	  
    default:
    {
      return false;
    }
  }
}


// No extended commands are implemented in the current version.
static bool handle_extended_command(const uint8_t data5[PFF_DATA_BYTES])
{
  if (!data5) return false;

  return false;
}


/*------------------------------------------------------------------------*/
/*Cycle data / parsing----------------------------------------------------*/
/*------------------------------------------------------------------------*/

// Reset variables related to a cycle of sensor line collection and modem response handling, so we are ready for a new cycle.
static void cycle_data_reset(void)
{
  memset(&cycle_data, 0, sizeof(cycle_data));
}

// Handle a line received from the sensors. If it is a Seabird37 line, parse and store the values. If it is a Signature500 line, try to collect it into pkt for later parsing.
/*
Mapping:
  v0 = temperature
  v1 = conductivity
  v2 = pressure
  v3 = salinity
  v4 = specific conductivity
*/
static bool parse_seabird_line_values(const char *line,
                                      float *temp,
                                      float *cond,
                                      float *pres,
                                      float *sal,
                                      float *spcond)
{
  // Check that the line is not NULL and starts with '#'
  if (!line || line[0] != '#') return false;

  // The line should have 5 comma-separated values after the initial '#'.
  const char *p = line + 1;
  // We will parse the 5 values into this array before assigning them to the output parameters.
  float v[5];

  // Parse the 5 comma-separated values into the array v. If parsing fails at any point, return false.
  for (int i = 0; i < 5; i++)
  {
    // Use strtof to parse a float from the string, and get the end pointer to check if parsing was successful.
    char *end = NULL;
    // Parse the next float value from the string starting at p, and store it in v[i].
    v[i] = strtof(p, &end);
    // If end is equal to p, it means that no valid float was parsed, so we return false.
    if (end == p) return false;

    // If we haven't parsed all 5 values yet, we expect a comma after the number.
    if (i < 4)
    {
      // Look for the next comma in the string starting from end.
      const char *comma = strchr(end, ',');
      // If we don't find a comma where we expect one, the format is invalid, so we return false.
      if (!comma) return false;
      // Move p to the character after the comma for the next iteration of parsing.
      p = comma + 1;
    }
  }

  // If we successfully parsed all 5 values, assign them to the output parameters.
  *temp   = v[0];
  *cond   = v[1];
  *pres   = v[2];
  *sal    = v[3];
  *spcond = v[4];

  return true;
}

// Store a measurement from Seabird in the ring buffer. When we reach the end of the buffer, we will start overwriting the oldest measurements. Also update total_samples and window_count.
static void store_seabird_measurement(float temp,
                                      float cond,
                                      float pres,
                                      float sal,
                                      float spcond)
{
  // Store the measurement values in the current write position of the ring buffer.
  temperatur[write_pos]         = temp;
  konduktivitet[write_pos]      = cond;
  trykk[write_pos]              = pres;
  salinitet[write_pos]          = sal;
  spes_konduktivitet[write_pos] = spcond;

  // Move the write position forward by one, wrapping around to the beginning of the buffer if we reach the end.
  write_pos = (uint16_t)((write_pos + 1) % WINDOW_SIZE);

  total_samples++;
  if (window_count < WINDOW_SIZE)
    window_count++;
}


// Reset a sig500_raw_packet_t to initial value (ready for new collection)
static void sig500_raw_reset(sig500_raw_packet_t *pkt)
{
  if (!pkt) return;
  memset(pkt, 0, sizeof(*pkt));
}


// Handle a line starting with '$' and try to collect it into pkt if it is part of a Signature500 package. Return the result of the collection.
static sig500_collect_result_t sig500_collect_line(sig500_raw_packet_t *pkt, const char *line)
{
  // Check that pkt and line are not NULL, and that line starts with '$'
  if (!pkt || !line || line[0] != '$') return SIG500_COLLECT_IGNORED;

  // If the line starts with "$PNORH", this is the start of a new packet, so we reset pkt and start collecting
  if (strncmp(line, "$PNORH", 6) == 0)
  {
    // Start of new packet, reset pkt and start collecting
    sig500_raw_reset(pkt);

    // Save the PNORH line in pkt for later parsing and SD storage, ensuring null-termination.
    strncpy(pkt->pnorh, line, LINE_BUF_SIZE - 1);

    pkt->active = true;                                           // start collection
    pkt->complete = false;                                        // reset PNORC-counter for new pack
    pkt->pnorc_count = 0;

    return SIG500_COLLECT_IN_PROGRESS;
  }

  // If we are not active, we ignore all lines until we get a new "$PNORH"
  if (!pkt->active)
  {
    return SIG500_COLLECT_IGNORED;
  }

  // If the line starts with "$PNORS", save it in pkt for later parsing and SD storage, ensuring null-termination.
  if (strncmp(line, "$PNORS", 6) == 0)
  {
    // Save the PNORS line in pkt for later parsing and SD storage, ensuring null-termination.
    strncpy(pkt->pnors, line, LINE_BUF_SIZE - 1);
    pkt->pnors[LINE_BUF_SIZE - 1] = '\0';
    return SIG500_COLLECT_IN_PROGRESS;
  }

  // If the line starts with "$PNORC", save it in the next available PNORC slot in pkt for later parsing and SD storage, ensuring null-termination.
  if (strncmp(line, "$PNORC", 6) == 0)
  {
    // If we have already collected the maximum number of PNORC lines, this is an error in the packet format, so we return an error result and reset the collector to start fresh on the next line.
    if (pkt->pnorc_count >= PNORC_BURST_COUNT)
    {
      return SIG500_COLLECT_ERROR;
    }

    // Save the PNORC line in the next available PNORC slot in pkt for later parsing and SD storage, ensuring null-termination.
    strncpy(pkt->pnorc[pkt->pnorc_count], line, LINE_BUF_SIZE - 1);
    pkt->pnorc[pkt->pnorc_count][LINE_BUF_SIZE - 1] = '\0';
    pkt->pnorc_count++;

    // If we have collected at least one PNORC line and we have a PNORS line, we consider the packet complete.
    if (pkt->pnors[0] != '\0' && pkt->pnorc_count >= PNORC_BURST_COUNT)
    {
      pkt->complete = true;
      return SIG500_COLLECT_COMPLETE;
    }

    // We are still collecting the packet, so we return the in-progress result.
    return SIG500_COLLECT_IN_PROGRESS;
  }

  // If the line does not match any of the expected types, we ignore it.
  return SIG500_COLLECT_IGNORED;
}


// Check that we have a complete and valid Signature500 package in pkt, and parse the values ​​into global variables.
static bool sig500_parse_raw_packet(const sig500_raw_packet_t *pkt)
{
  // Check that pkt is not NULL, that we have a complete packet, that the PNORS line is not empty, and that we have at least the expected number of PNORC lines.
  // If any of these conditions are not met, return false to indicate that parsing failed.
  if (!pkt) return false;
  if (!pkt->complete) return false;
  if (pkt->pnors[0] == '\0') return false;
  if (pkt->pnorc_count < PNORC_BURST_COUNT) return false;

  sig_pnors.valid = false;
  sig_pnorc_first.valid = false;
  sig_pnorc_mid.valid   = false;
  sig_pnorc_last.valid  = false;

  // Parse PNORS line into sig_pnors, return false if parsing fails
  if (!parse_sig_pnors(pkt->pnors, &sig_pnors))
    return false;

  // Parse PNORC lines into sig_pnorc_first, sig_pnorc_mid, sig_pnorc_last based on their order in the packet, return false if parsing fails
  for (uint8_t i = 0; i < pkt->pnorc_count; i++)
  {
    sig_pnorc_t tmp = {0};

    // Parse the PNORC line into a temporary sig_pnorc_t struct. We will later check that we got the expected number of PNORC lines and that the first, middle, and last ones are valid.
    if (!parse_sig_pnorc(pkt->pnorc[i], &tmp))
      return false;

    // Store the parsed PNORC line in the appropriate global variable based on its order in the packet. First line is 0 index
    if (i == 0)
      sig_pnorc_first = tmp;

    // For the middle PNORC, we take the one at index PNORC_MID_INDEX-1 because the index is 0-based. We expect this to be the middle line in the burst of PNORC lines.
    if (i == (PNORC_MID_INDEX - 1))
      sig_pnorc_mid = tmp;

    // For the last PNORC, we take the one at index PNORC_BURST_COUNT-1 because the index is 0-based. We expect this to be the last line in the burst of PNORC lines.
    if (i == (PNORC_BURST_COUNT - 1))
      sig_pnorc_last = tmp;
  }

  return sig_pnors.valid &&
         sig_pnorc_first.valid &&
         sig_pnorc_mid.valid &&
         sig_pnorc_last.valid;
}


// Parse N floats after a prefix (ex. "$PNORS" or "$PNORC").
static bool parse_floats_after_prefix(const char *line, const char *prefix,
                                      float *out, int n)
{
  // Check that the input parameters are valid
  if (!line || !prefix || !out || n <= 0) return false;

  // Check that the line starts with the expected prefix. If not, return false. We use strncmp to compare the beginning of the line with the prefix, and we check only the length of the prefix.
  size_t prelen = strlen(prefix);
  if (strncmp(line, prefix, prelen) != 0) return false;

  // Move the pointer to the position right after the prefix. We will start parsing floats from this position.
  const char *p = line + prelen;
  if (*p == ',') p++;   // if "$PNORS,4,..."
  // if "$PNORS4,..." then p is right on '4' and that's also OK

  // Parse n floats from the string, separated by commas. For each float, we use strtof to parse it and get the end pointer to check if parsing was successful.
  for (int i = 0; i < n; i++)
  {
    char *end = NULL;
    // Parse the next float value from the string starting at p, and store it in out[i].
    out[i] = strtof(p, &end);
    // If end is equal to p, it means that no valid float was parsed, so we return false.
    if (end == p) return false;

    // If we haven't parsed all n floats yet, we expect a comma after the number. We look for the next comma in the string starting from end.
    // If we don't find a comma where we expect one, the format is invalid, so we return false.
    // If we do find a comma, we move p to the character after the comma for the next iteration of parsing.
    if (i < (n - 1))
    {
      const char *comma = strchr(end, ',');
      if (!comma) return false;
      p = comma + 1;
    }
  }
  return true;
}



// PNORS: keep first 5 (Battery, Sound Speed, Heading, Pitch, Roll),
// discard index 6 (pressure) and 7 (temperature).
static bool parse_sig_pnors(const char *line, sig_pnors_t *dst)
{
  // Check that dst and line are not NULL
  if (!dst || !line) return false;

  /* Variant A: $PNORS4,.... -> read 8 floats: [id] + 7 values
     Variant B: $PNORS,... -> read 7 floats: 7 values ​​*/
  bool has_id = (line[6] != '\0' && line[6] != ',');

  if (has_id)
  {
    float f[8]; // id + battery..roll + pressure + temp
    // Parse the line to extract 8 floats
    if (!parse_floats_after_prefix(line, "$PNORS", f, 8)) return false;

    // Map the parsed floats to the appropriate fields in dst, skipping the id (f[0]) and the pressure and temperature (f[6] and f[7])
    dst->battery     = f[1];
    dst->sound_speed = f[2];
    dst->heading     = f[3];
    dst->pitch       = f[4];
    dst->roll        = f[5];
    dst->valid       = true;
    return true;
  }
  else
  {
    float f[7]; // battery..roll + pressure + temp
    // Parse the line to extract 7 floats
    if (!parse_floats_after_prefix(line, "$PNORS", f, 7)) return false;

    // Map the parsed floats to the appropriate fields in dst, taking the first 5 values as battery, sound speed, heading, pitch, and roll, and ignoring the pressure and temperature (f[5] and f[6])
    dst->battery     = f[0];
    dst->sound_speed = f[1];
    dst->heading     = f[2];
    dst->pitch       = f[3];
    dst->roll        = f[4];
    dst->valid       = true;
    return true;
  }
}

// PNORC: keep index 1-3 (Cell position, speed, direction).
static bool parse_sig_pnorc(const char *line, sig_pnorc_t *dst)
{
  if (!dst || !line) return false;

  bool has_id = (line[6] != '\0' && line[6] != ',');

  if (has_id)
  {
    float f[4]; // id + cellpos + speed + direction
    if (!parse_floats_after_prefix(line, "$PNORC", f, 4)) return false;

    // Map the parsed floats to the appropriate fields in dst, skipping the id (f[0])
    dst->cell_pos  = f[1];
    dst->speed     = f[2];
    dst->direction = f[3];
    dst->valid     = true;
    return true;
  }
  else
  {
    float f[3]; // cellpos + speed + direction
    if (!parse_floats_after_prefix(line, "$PNORC", f, 3)) return false;

    // Map the parsed floats to the appropriate fields in dst
    dst->cell_pos  = f[0];
    dst->speed     = f[1];
    dst->direction = f[2];
    dst->valid     = true;
    return true;
  }
}

/*------------------------------------------------------------------------*/
/*PFF / Packing-----------------------------------------------------------*/
/*------------------------------------------------------------------------*/

// Scales a float x in the interval [in_min, in_max] to a uint8_t in the interval [0, 255].
static uint8_t scale_to_u8(float x, float in_min, float in_max)
{
  // If the input range is invalid (max less or equal to min), we cannot scale, so we return 0 as a default value.
  if (in_max <= in_min) return 0;

  // Clamp x to the input range [in_min, in_max] to ensure that the output is within [0, 255].
  x = clampf(x, in_min, in_max);

  // Scale x from the input range [in_min, in_max] to the output range [0, 255]. We first normalize x to a value between 0 and 1, and then scale it to 0-255.
  float norm = (x - in_min) / (in_max - in_min);  // 0..1
  float y = norm * 255.0f;

  // Round to nearest integer and clamp to [0, 255]
  if (y < 0.0f) y = 0.0f;
  if (y > 255.0f) y = 255.0f;

  return (uint8_t)(y + 0.5f);
}

// Helper function: clamps x to the interval [lo, hi]
static float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Write a selected number of bits from value into dst.
// The bits are written from most significant to least significant bit.
// bitpos points to the next free bit position in dst and is updated after each written bit.
static void pff_write_bits(uint8_t *dst, uint16_t *bitpos, uint32_t value, uint8_t nbits)
{
  for (int i = (int)nbits - 1; i >= 0; i--)
  {
    // Extract the current bit from value.
    uint8_t bit = (value >> i) & 1u;

    // Get the current absolute bit position in the byte array.
    uint16_t pos = *bitpos;

    // Find which byte this bit belongs to.
    uint16_t byte_index = pos / 8;

    // Find the bit position inside the byte.
    // MSB-first means that bit position 0 maps to bit 7 in dst[0].
    uint8_t bit_in_byte = 7 - (pos % 8);

    // If the bit is 1, set the corresponding bit in dst.
    // If the bit is 0, nothing is done because dst was cleared before use.
    if (bit)
    {
      dst[byte_index] |= (uint8_t)(1u << bit_in_byte);
    }

    // Move to the next bit position.
    (*bitpos)++;
  }
}

// Simple checksum: XOR of all bytes in the message (without the checksum bit itself)
static uint8_t pff_checksum_xor(const uint8_t *bytes, uint8_t len)
{
  uint8_t c = 0;
  for (uint8_t i = 0; i < len; i++) c ^= bytes[i];
  return c;
}

// Build a PFF message (10 bytes) from the components.
static void pff_build(uint8_t version,
                      uint8_t msg_id,
                      uint32_t time22,
                      const uint8_t data5[PFF_DATA_BYTES],
                      uint8_t out10[PFF_TOTAL_BYTES])
{

  /* ---- PFF format ----
   version: 2 bits
   msg_id : 8 bits
   time   : 22 bits
   data   : 40 bits (5 bytes)
   csum   : 8 bits
   Total  : 80 bits = 10 bytes = 5 * 16-bit
  */
  memset(out10, 0, PFF_TOTAL_BYTES);

  // We will write bits into out10 using pff_write_bits, which takes care of packing bits into bytes. bitpos keeps track of the next free bit position in out10.
  uint16_t bitpos = 0;

  // Version is 2 bits
  version &= 0x03;
  // time is 22 bits
  time22  &= 0x003FFFFFu; // 22 bits

  // Header: 2 + 8 + 22 bits = 32 bits = 4 bytes
  pff_write_bits(out10, &bitpos, version, 2);
  pff_write_bits(out10, &bitpos, msg_id, 8);
  pff_write_bits(out10, &bitpos, time22, 22);

  // Data: 5 bytes = 40 bits
  for (int i = 0; i < PFF_DATA_BYTES; i++)
  {
    pff_write_bits(out10, &bitpos, data5[i], 8);
  }

  // Now we are at 72 bits = 9 bytes
  // (2 + 8 + 22 + 40 = 72)
  // Checksum will be the last byte:
  uint8_t csum = pff_checksum_xor(out10, 9);
  out10[9] = csum;
}


// Split 10 bytes into 5x 16-bit words (big-endian).
static void pff_to_words_be(const uint8_t msg10[PFF_TOTAL_BYTES], uint16_t words[PFF_WORDS])
{
  for (int i = 0; i < PFF_WORDS; i++)
  {
    // Big-endian: HI byte first, then LO byte
    uint8_t hi = msg10[i * 2 + 0];
    uint8_t lo = msg10[i * 2 + 1];

    words[i] = (uint16_t)(((uint16_t)hi << 8) | lo);
  }
}

// Parse a PFF frame. Returns false on invalid input or checksum failure.
static bool pff_parse(const uint8_t in10[PFF_TOTAL_BYTES], pff_rx_packet_t *out)
{
  if (!in10 || !out) return false;

  // Check checksum first.
  uint8_t csum = pff_checksum_xor(in10, 9);

  if (csum != in10[9])
  {
    return false;
  }

  uint16_t bitpos = 0;

  memset(out, 0, sizeof(*out));

  // Header: 2 + 8 + 22 bits = 32 bits = 4 bytes
  out->version = (uint8_t)pff_read_bits(in10, &bitpos, 2);
  out->msg_id  = (uint8_t)pff_read_bits(in10, &bitpos, 8);
  out->time22  = (uint32_t)pff_read_bits(in10, &bitpos, 22);

  // Data: 5 bytes = 40 bits
  for (int i = 0; i < PFF_DATA_BYTES; i++)
  {
    out->data[i] = (uint8_t)pff_read_bits(in10, &bitpos, 8);
  }

  out->valid = true;
  return true;
}

// Read nbits from src at *bitpos. Advances *bitpos and returns the value.
static uint32_t pff_read_bits(const uint8_t *src, uint16_t *bitpos, uint8_t nbits)
{
  uint32_t value = 0;

  // Reads MSB-first from the byte array.
  for (uint8_t i = 0; i < nbits; i++)
  {
    // Calculate byte index and bit index within the byte
    uint16_t pos = *bitpos;
    // For MSB-first, byte index is pos / 8
    uint16_t byte_index = pos / 8;
    // For MSB-first, bit index in byte is 7 - (pos % 8)
    uint8_t  bit_in_byte = 7 - (pos % 8);

    // Extract the bit and add to value
    uint8_t bit = (src[byte_index] >> bit_in_byte) & 0x01u;
    value = (value << 1) | bit;

    (*bitpos)++;
  }

  return value;
}

// Calls the “last sample” in the ring buffer and scales to 8 bits
static bool seabird_pack_latest(seabird_u8_t *out)
{
  if (!out) return false;
  if (window_count == 0) return false;

  // Get the index of the last sample (the one before write_pos, wrapping around)
  uint16_t last = (write_pos == 0) ? (WINDOW_SIZE - 1) : (write_pos - 1);

  // Scale and pack to 8-bit. Adjust the scales according to the expected range for each variable.
  out->t  = scale_to_u8(temperatur[last],         temp_scale.min,   temp_scale.max);
  out->c  = scale_to_u8(konduktivitet[last],      cond_scale.min,   cond_scale.max);
  out->p  = scale_to_u8(trykk[last],              pres_scale.min,   pres_scale.max);
  out->s  = scale_to_u8(salinitet[last],          sal_scale.min,    sal_scale.max);
  out->sc = scale_to_u8(spes_konduktivitet[last], spcond_scale.min, spcond_scale.max);

  return true;
}

// Sending 1: PNORS -> 5 values (battery, sound_speed, heading, pitch, roll)
static bool sig500_pack_send1(uint8_t data5[5])
{
  if (!sig_pnors.valid) return false;

  // Scale and pack to 8-bit. Adjust the scales according to the expected range for each variable.
  data5[0] = scale_to_u8(sig_pnors.battery,     sig_battery_scale.min,     sig_battery_scale.max);
  data5[1] = scale_to_u8(sig_pnors.sound_speed, sig_sound_speed_scale.min, sig_sound_speed_scale.max);
  data5[2] = scale_to_u8(sig_pnors.heading,     sig_heading_scale.min,     sig_heading_scale.max);
  data5[3] = scale_to_u8(sig_pnors.pitch,       sig_pitch_scale.min,       sig_pitch_scale.max);
  data5[4] = scale_to_u8(sig_pnors.roll,        sig_roll_scale.min,        sig_roll_scale.max);

  return true;
}

//   Sending 2: PNORC 1/2 -> 5 values
//   3 from the first cell: (cell_pos, speed, direction)
//   2 from middle cell: (cell_pos, speed)
// + chamber temperature from cycle_data.chamber_temp_c
static bool sig500_pack_send2(uint8_t data5[5])
{
  if (!sig_pnorc_first.valid) return false;
  if (!sig_pnorc_mid.valid)   return false;

  // Scale and pack to 8-bit.
  data5[0] = scale_to_u8(sig_pnorc_first.cell_pos,  sig_cellpos_scale.min,   sig_cellpos_scale.max);
  data5[1] = scale_to_u8(sig_pnorc_first.speed,     sig_speed_scale.min,     sig_speed_scale.max);
  data5[2] = scale_to_u8(sig_pnorc_first.direction, sig_direction_scale.min, sig_direction_scale.max);

  data5[3] = scale_to_u8(sig_pnorc_mid.cell_pos,    sig_cellpos_scale.min,   sig_cellpos_scale.max);
  data5[4] = scale_to_u8(sig_pnorc_mid.speed,       sig_speed_scale.min,     sig_speed_scale.max);

  return true;
}

//   Sending 3: PNORC 2/2 -> 4 values ​​+ chamber temp
//   1 from middle cell: (direction)
//   3 from last cell: (cell_pos, speed, direction)
//   + chamber temperature from cycle_data.chamber_temp_c

static bool sig500_pack_send3(uint8_t data5[5])
{
  if (!sig_pnorc_mid.valid)  return false;
  if (!sig_pnorc_last.valid) return false;

  // Scale and pack to 8-bit.
  data5[0] = scale_to_u8(sig_pnorc_mid.direction,  sig_direction_scale.min, sig_direction_scale.max);
  data5[1] = scale_to_u8(sig_pnorc_last.cell_pos,  sig_cellpos_scale.min,   sig_cellpos_scale.max);
  data5[2] = scale_to_u8(sig_pnorc_last.speed,     sig_speed_scale.min,     sig_speed_scale.max);
  data5[3] = scale_to_u8(sig_pnorc_last.direction, sig_direction_scale.min, sig_direction_scale.max);
  data5[4] = scale_to_u8(cycle_data.chamber_temp_c,chamber_temp_scale.min,  chamber_temp_scale.max);

  return true;
}


// Build and queue the necessary PFF frames for PNORS, the middle of the PNORC burst,
// and the last PNORC, based on the parsed values in sig_pnors and sig_pnorc_*.
static bool sig500_queue_frames(void)
{
  uint8_t  version = 1;
  uint32_t time22 = rtc_get_pff_time22();

  uint8_t  data5[PFF_DATA_BYTES];
  uint8_t  msg10[PFF_TOTAL_BYTES];
  uint16_t words5[PFF_WORDS];
  bool any = false;

  // Package one: PNORS -> battery, sound speed, heading, pitch, roll.
  if (sig500_pack_send1(data5))
  {
    pff_build(version, PFF_MSGID_SIG_PNORS, time22, data5, msg10);
    pff_to_words_be(msg10, words5);

    if (modem_q_push(words5, FRAME_SIG500))
    {
      any = true;
    }
  }

  // Package two: first PNORC cell + part of middle PNORC cell.
  if (sig500_pack_send2(data5))
  {
    pff_build(version, PFF_MSGID_SIG_PNORC_12, time22, data5, msg10);
    pff_to_words_be(msg10, words5);

    if (modem_q_push(words5, FRAME_SIG500))
    {
      any = true;
    }
  }

  // Package three: rest of middle PNORC cell + last PNORC cell + chamber temperature.
  if (sig500_pack_send3(data5))
  {
    pff_build(version, PFF_MSGID_SIG_PNORC_22, time22, data5, msg10);
    pff_to_words_be(msg10, words5);

    if (modem_q_push(words5, FRAME_SIG500))
    {
      any = true;
    }
  }

  return any;
}

// Calculate statistics (mean, min, max, std) over the pressure measurements
// in the ring buffer and queue a PFF frame with these stats.
static void seabird_queue_pressure_stats_frame(void)
{
  if (window_count == 0) return;

  // Calculate mean
  float p_mean = mean_ring(trykk, window_count, write_pos);

  // Calculate min and max
  float p_min, p_max;
  min_max_ring(trykk, window_count, write_pos, &p_min, &p_max);

  // Calculate standard deviation
  float p_std = std_avvik_ring(trykk, window_count, write_pos);

  // data5 = 5 bytes (40 bit)
  // [0] = mean, [1] = min, [2] = max, [3] = std, [4] = count
  uint8_t data5[PFF_DATA_BYTES];
  data5[0] = scale_to_u8(p_mean, pres_scale.min, pres_scale.max);
  data5[1] = scale_to_u8(p_min,  pres_scale.min, pres_scale.max);
  data5[2] = scale_to_u8(p_max,  pres_scale.min, pres_scale.max);
  data5[3] = scale_to_u8(p_std,  pres_std_scale.min, pres_std_scale.max);
  data5[4] = (uint8_t)window_count;

  // Build PFF frame
  uint8_t  msg10[PFF_TOTAL_BYTES];
  uint16_t words5[PFF_WORDS];

  uint8_t  version = 1;
  uint32_t time22 = rtc_get_pff_time22();

  // Build PFF with msg_id for Seabird pressure statistics.
  pff_build(version, PFF_MSGID_SEABIRD_PRES_STATS, time22, data5, msg10);
  pff_to_words_be(msg10, words5);

  // Queue for sending. If the queue is full, the frame is dropped.
  (void)modem_q_push(words5, FRAME_SEABIRD_PRES_STATS);
}

/*------------------------------------------------------------------------*/
/*PFF / Packing-----------------------------------------------------------*/
/*------------------------------------------------------------------------*/

// Calculate the start index of the ring buffer window, given the current write position and the count of valid samples.
static uint16_t ring_start_index(uint16_t write_pos, uint16_t count)
{
  // write_pos points to "next place to write"
  // oldest is write_pos - count (mod WINDOW_SIZE)
  int32_t start = (int32_t)write_pos - (int32_t)count;
  while (start < 0) start += WINDOW_SIZE;
  return (uint16_t)start;
}

// Mean over ring buffer-window
static float mean_ring(const float *buf, uint16_t count, uint16_t write_pos)
{
  if (count == 0) return 0.0f;

  // Start index of the window is write_pos - count (wrapping around)
  uint16_t idx = ring_start_index(write_pos, count);

  float sum = 0.0f;
  // Loop through the count of valid samples, starting from idx and wrapping around the buffer
  for (uint16_t i = 0; i < count; i++)
  {
    sum += buf[idx];
    idx++;
    if (idx >= WINDOW_SIZE) idx = 0;
  }
  // Return the mean
  return sum / (float)count;
}

// Min/Max over ring buffer-window
static void min_max_ring(const float *buf, uint16_t count, uint16_t write_pos,
                         float *min_out, float *max_out)
{
  if (count == 0) { *min_out = 0.0f; *max_out = 0.0f; return; }

  // Start index of the window is write_pos - count (wrapping around)
  uint16_t idx = ring_start_index(write_pos, count);

  float min_v = buf[idx];
  float max_v = buf[idx];

  // Loop through the count of valid samples, starting from idx and wrapping around the buffer
  for (uint16_t i = 0; i < count; i++)
  {
    float x = buf[idx];
    if (x < min_v) min_v = x;
    if (x > max_v) max_v = x;

    idx++;
    if (idx >= WINDOW_SIZE) idx = 0;
  }

  // Write results to output parameters
  *min_out = min_v;
  *max_out = max_v;
}

// Sample standard deviation (N-1) over ring buffer-window
static float std_avvik_ring(const float *buf, uint16_t count, uint16_t write_pos)
{
  if (count < 2) return 0.0f;

  // First calculate the mean
  float m = mean_ring(buf, count, write_pos);

  // Then calculate the sum of squared deviations from the mean
  uint16_t idx = ring_start_index(write_pos, count);

  float sum_sq = 0.0f;
  // Loop through the count of valid samples, starting from idx and wrapping around the buffer
  for (uint16_t i = 0; i < count; i++)
  {
    float d = buf[idx] - m;
    sum_sq += d * d;

    idx++;
    if (idx >= WINDOW_SIZE) idx = 0;
  }

  // Sample standard deviation is sqrt of variance
  float var = sum_sq / (float)(count - 1);
  return sqrtf(var);
}

/*----------------------------------------------------------------------------*/
/* SD card / FileX -----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

// Initialize and open SD media. Returns false on failure.
static bool SD_init_and_open(void)
{
  UINT status;

  // Initialize the SD card media driver and open the media with FileX.
  // A cache buffer is provided for FileX.
  status = fx_media_open(&sd_media,
                         "SD_DISK",
                         fx_stm32_sd_driver,
                         0,
                         media_cache,
                         sizeof(media_cache));

  return (status == FX_SUCCESS);
}

// Ensure file exists; create and write header if missing. Returns false on error.
static bool SD_create_file_and_header_if_needed(const CHAR *filename, const char *header)
{
  UINT status;

  // Try to open the file for reading. If it exists, close it and return true.
  status = fx_file_open(&sd_media, &sd_file, (CHAR *)filename, FX_OPEN_FOR_READ);
  if (status == FX_SUCCESS)
  {
    (void)fx_file_close(&sd_file);
    return true;
  }

  // If the file was not missing, another error occurred.
  if (status != FX_NOT_FOUND)
  {
    return false;
  }

  // Create file if it does not exist.
  status = fx_file_create(&sd_media, (CHAR *)filename);
  if (status != FX_SUCCESS)
  {
    return false;
  }

  // Open the newly created file for writing.
  status = fx_file_open(&sd_media, &sd_file, (CHAR *)filename, FX_OPEN_FOR_WRITE);
  if (status != FX_SUCCESS)
  {
    return false;
  }

  // Write CSV header.
  status = fx_file_write(&sd_file, header, strlen(header));
  if (status != FX_SUCCESS)
  {
    (void)fx_file_close(&sd_file);
    return false;
  }

  // Flush to ensure header is written to media, then close file.
  (void)fx_media_flush(&sd_media);
  (void)fx_file_close(&sd_file);

  return true;
}

// Append a line to a file. Returns false on failure.
static bool SD_append_line(const CHAR *filename, const char *line)
{
  UINT status;

  // Open the file for writing.
  status = fx_file_open(&sd_media, &sd_file, (CHAR *)filename, FX_OPEN_FOR_WRITE);
  if (status != FX_SUCCESS)
  {
    return false;
  }

  // Move write pointer to end of file.
  status = fx_file_seek(&sd_file, sd_file.fx_file_current_file_size);
  if (status != FX_SUCCESS)
  {
    (void)fx_file_close(&sd_file);
    return false;
  }

  // Write line to file.
  status = fx_file_write(&sd_file, line, strlen(line));
  if (status != FX_SUCCESS)
  {
    (void)fx_file_close(&sd_file);
    return false;
  }

  // Flush media to ensure data is written.
  status = fx_media_flush(&sd_media);
  if (status != FX_SUCCESS)
  {
    (void)fx_file_close(&sd_file);
    return false;
  }

  // Close file.
  status = fx_file_close(&sd_file);
  if (status != FX_SUCCESS)
  {
    return false;
  }

  return true;
}

// Write a Seabird record to SD as a CSV line. Returns false on failure.
static bool SD_write_seabird_record(void)
{
  char timestamp_string[32];
  char line_buffer[128];

  if (!sd_ready)
  {
    return false;
  }

  if (!cycle_data.seabird_valid)
  {
    return false;
  }

  rtc_format_timestamp(timestamp_string, sizeof(timestamp_string));

  // Format: timestamp, temp, cond, pres, sal, spcond
  snprintf(line_buffer, sizeof(line_buffer),
           "%s,%.4f,%.4f,%.3f,%.4f,%.4f\r\n",
           timestamp_string,
           cycle_data.temp,
           cycle_data.cond,
           cycle_data.pres,
           cycle_data.sal,
           cycle_data.spcond);

  // Append the line to the Seabird CSV file on SD.
  return SD_append_line(SD_FILENAME_SEABIRD, line_buffer);
}

// Write a Signature500 record to SD as a CSV line. Returns false on failure.
static bool SD_write_sig500_record(void)
{
  char timestamp_string[32];
  char line_buffer[192];

  if (!sd_ready)
  {
    return false;
  }

  if (!cycle_data.sig500_valid)
  {
    return false;
  }

  rtc_format_timestamp(timestamp_string, sizeof(timestamp_string));

  // Format: timestamp, then 3 values from PNORC first cell,
  // 3 from middle cell, and 3 from last cell.
  snprintf(line_buffer, sizeof(line_buffer),
           "%s,%.1f,%.3f,%.2f,%.1f,%.3f,%.2f,%.1f,%.3f,%.2f\r\n",
           timestamp_string,

           sig_pnorc_first.cell_pos,
           sig_pnorc_first.speed,
           sig_pnorc_first.direction,

           sig_pnorc_mid.cell_pos,
           sig_pnorc_mid.speed,
           sig_pnorc_mid.direction,

           sig_pnorc_last.cell_pos,
           sig_pnorc_last.speed,
           sig_pnorc_last.direction);

  return SD_append_line(SD_FILENAME_SIG500, line_buffer);
}

// Power on sequence for SD card: initialize hardware, open media,
// ensure files and headers exist. Returns true if SD is ready for writing.
static bool SD_power_on(void)
{
  if (sd_ready)
  {
    return true;
  }

  HAL_Delay(50);

  /*
   * No card-detect pin is used on this PCB.
   * Actual detection is done by trying to initialize and open the card.
   */
  if (!MX_SDMMC1_SD_Init())
  {
    sd_ready = false;
    return false;
  }

  // Initialize and open SD media with FileX. This will return false if there is no card or if initialization fails.
  if (!SD_init_and_open())
  {
    sd_ready = false;
    return false;
  }

  // For the Seabird file, we always want to ensure it has the correct header, so we create it with header if it does not exist, or overwrite the header if it already exists.
  if (!SD_create_file_and_header_if_needed(SD_FILENAME_SEABIRD, SD_HEADER_SEABIRD))
  {
    (void)fx_media_close(&sd_media);
    sd_ready = false;
    return false;
  }

  // For the Signature500 file, we only write a header if the file does not already exist. If it exists, we assume it already has a header and we do not want to overwrite it.
  if (!SD_create_file_and_header_if_needed(SD_FILENAME_SIG500, SD_HEADER_SIG500))
  {
    (void)fx_media_close(&sd_media);
    sd_ready = false;
    return false;
  }

  sd_ready = true;
  return true;
}

// Power down sequence for SD card: flush and close media if it was ready.
static void SD_power_down(void)
{
  if (sd_ready)
  {
    // Flush any pending writes and close the media. We set sd_ready to false to indicate that SD is not ready until we power on again.
    (void)fx_media_flush(&sd_media);
    (void)fx_media_close(&sd_media);
    sd_ready = false;
  }
}
/*----------------------------------------------------------------------------*/
/* ADC / Temp measurement ----------------------------------------------------*/
/*----------------------------------------------------------------------------*/

// Reads an ADC value from the TMP35 sensor on ADC1 on PC0. Returns the raw ADC value, or 0 on error.
static uint32_t TMP35_ReadAdcRaw(void)
{
  uint32_t adc_raw = 0;

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return 0;
  }

  // Wait for conversion to complete.
  if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) == HAL_OK)
  {
    adc_raw = HAL_ADC_GetValue(&hadc1);
  }
  else
  {
    (void)HAL_ADC_Stop(&hadc1);
    return 0;
  }

  (void)HAL_ADC_Stop(&hadc1);

  return adc_raw;
}

// Converts raw ADC-value to voltage
static float TMP35_AdcToVoltage(uint32_t adc_raw)
{
    return ((float)adc_raw * ADC_VREF) / ADC_MAX_VALUE;
}

// Converts voltage to temperature in °C, using the TMP35 characteristics.
static float TMP35_VoltageToCelsius(float voltage)
{
    return voltage / TMP35_SCALE_VOLT_PER_C;
}

// Reads temperature directly in °C
static float TMP35_ReadTemperatureC(void)
{
    uint32_t adc_raw = TMP35_ReadAdcRaw();
    float voltage = TMP35_AdcToVoltage(adc_raw);
    float temperature_c = TMP35_VoltageToCelsius(voltage);
    return temperature_c;
}

/*----------------------------------------------------------------------------*/
/* Time------------------ ----------------------------------------------------*/
/*----------------------------------------------------------------------------*/

// Helper functions to get current date/time from RTC, convert to Unix seconds, and format timestamps. Also includes leap year and days in month calculations.
static bool is_leap_year(uint16_t year)
{
  if ((year % 400u) == 0u) return true;
  if ((year % 100u) == 0u) return false;
  return ((year % 4u) == 0u);
}

// Returns the number of days in the given month of the given year (accounts for leap years).
static uint8_t days_in_month(uint16_t year, uint8_t month)
{
  static const uint8_t days[12] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
  };

  if (month == 2u && is_leap_year(year))
    return 29u;

  return days[month - 1u];
}

// Get current date and time from RTC and fill rtc_datetime_t struct. Returns false on error.
static bool rtc_get_datetime(rtc_datetime_t *dt)
{
  RTC_TimeTypeDef gTime;
  RTC_DateTypeDef gDate;

  if (!dt) return false;

  if (HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN) != HAL_OK)
    return false;

  if (HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN) != HAL_OK)
    return false;

  dt->year   = (uint16_t)(2000u + gDate.Year);
  dt->month  = gDate.Month;
  dt->day    = gDate.Date;
  dt->hour   = gTime.Hours;
  dt->minute = gTime.Minutes;
  dt->second = gTime.Seconds;

  return true;
}

// Convert rtc_datetime_t to Unix timestamp (seconds since 1970-01-01 00:00:00 UTC). Returns 0 on error.
static uint32_t rtc_datetime_to_unix_seconds(const rtc_datetime_t *dt)
{
  uint32_t days = 0u;

  if (!dt) return 0u;

  for (uint16_t y = 1970u; y < dt->year; y++)
    days += is_leap_year(y) ? 366u : 365u;

  for (uint8_t m = 1u; m < dt->month; m++)
    days += days_in_month(dt->year, m);

  days += (uint32_t)(dt->day - 1u);

  return days * 86400u
       + (uint32_t)dt->hour   * 3600u
       + (uint32_t)dt->minute * 60u
       + (uint32_t)dt->second;
}

// Get a 22-bit time value for PFF messages, representing the current time in 10-minute intervals since Unix epoch. Returns 0 on error.
static uint32_t rtc_get_pff_time22(void)
{
  rtc_datetime_t dt;

  if (!rtc_get_datetime(&dt))
    return 0u;

    // Convert to Unix seconds and then to 10-minute intervals.
  uint32_t unix_seconds = rtc_datetime_to_unix_seconds(&dt);
  uint32_t time_10min   = unix_seconds / 600u;

  return time_10min & 0x003FFFFFu;
}

// Format the current RTC date and time into a readable string ("2024-06-01 12:34:56"). Returns empty string on error.
static void rtc_format_timestamp(char *out, size_t out_size)
{
  rtc_datetime_t dt;

  if (!out || out_size == 0u) return;

  if (!rtc_get_datetime(&dt))
  {
    snprintf(out, out_size, "2000-01-01 00:00:00");
    return;
  }

  snprintf(out, out_size,
           "%04u-%02u-%02u %02u:%02u:%02u",
           dt.year, dt.month, dt.day,
           dt.hour, dt.minute, dt.second);
}



/*----------------------------------------------------------------------------*/
/* CLOCK / POWER / INIT ------------------------------------------------------*/
/*----------------------------------------------------------------------------*/


// Configure the system clock
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE3) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

// Activate GPIO pins to power on connected peripherals (SD card, modem, sensors). This is called at startup and after waking from standby.
static void Activate_GPIO_Pins(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);    // SD-card power on
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,  GPIO_PIN_SET);    // Sensor 1 power on
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  GPIO_PIN_SET);    // Sensor 2 power on
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,  GPIO_PIN_SET);    // Sensor 3 power on
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_SET);    // Tempsensor power on
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);    // 12V enable
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  GPIO_PIN_SET);    // 5V enable
}

// Deactivate GPIO pins to power off connected peripherals. This is called before entering standby.
static void Deactivate_GPIO_Pins(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);    // SD-card power off
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_RESET);    // Modem power off
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,  GPIO_PIN_RESET);    // Sensor 1 power off
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  GPIO_PIN_RESET);    // Sensor 2 power off
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,  GPIO_PIN_RESET);    // Sensor 3 power off
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_RESET);    // Tempsensor power off
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);    // 12V disable
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  GPIO_PIN_RESET);    // 5V disable
}

// Led blink to indicate that the system is alive and has passed initial setup.
static void Blink_Led(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,  GPIO_PIN_SET);    	// LED on
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);     	// LED off
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,  GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,  GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
}

// System power configuration
static void SystemPower_Config(void)
{
  HAL_PWREx_DisableUCPDDeadBattery();

  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }

}

// Initialize ADC1
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // Set PC0 as ADC input channel 1, single-ended, with a sampling time of 391.5 cycles. No offset.
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_391CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

// Initialize the instruction cache in 1-way associative mode and enable it.
static void MX_ICACHE_Init(void)
{
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }

}

// Initialize the RTC peripheral with specified parameters, set privilege mode, and configure initial time and date.
static void MX_RTC_Init(void)
{

  RTC_PrivilegeStateTypeDef privilegeState = {0};
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }

  sTime.Hours = RTC_INIT_HOURS;
  sTime.Minutes = RTC_INIT_MINUTES;
  sTime.Seconds = RTC_INIT_SECONDS;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_INIT_WEEKDAY;
  sDate.Month = RTC_INIT_MONTH;
  sDate.Date = RTC_INIT_DATE;
  sDate.Year = RTC_INIT_YEAR;


  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
}

// Initialize the SDMMC1 peripheral for SD card communication with specified parameters.
static bool MX_SDMMC1_SD_Init(void)
{
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 60;

  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    return false;
  }

  return true;
}

// Initialize the LPUART1 peripheral for UART communication with specified parameters. This is used for sensor 3
static void MX_LPUART1_UART_Init(void)
{
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
}

// Initialize the USART1 peripheral for UART communication with specified parameters. This is used for Seabird
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

// Initialize the USART2 peripheral for UART communication with specified parameters. This is used for Signature500
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

// Initialize the USART3 peripheral for UART communication with specified parameters. This is used for Modem
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

//  Initialize GPIO pins: set modes, pull-up/down, and initial output levels for all used pins. This includes powering on/off peripherals and setting unused pins to analog mode to reduce power consumption.
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC1
                           PC2 PC3 PC6 PC7
                           PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA6 PA8
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB14
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB13 PB15
                           PB4 PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// Error handler
void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
