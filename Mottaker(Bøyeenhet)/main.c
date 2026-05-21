#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include "stm32u575xx.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_uart.h"
#include "stm32u5xx_nucleo.h"
 
// Handle for using COM1
COM_InitTypeDef BspCOMInit;

// Handle for usig USART2 to receive data from M16
UART_HandleTypeDef huart2;

// Handle for using VCOM inputs 
extern UART_HandleTypeDef hcom_uart[COMn];

// PFF frame format, in our case 10 bytes (version 1):
#define RX_FRAME_SIZE 10

// Variables for sending down data, buffer and flag
static uint8_t pending_tx_frame[RX_FRAME_SIZE];
static volatile uint8_t pending_tx_ready = 0;

// section of PFF format in bits
typedef struct {
  uint8_t  prot_ver;     // 2 bit
  uint8_t  msg_id;       // 8 bit
  uint32_t time;         // 22 bit
  uint8_t  data[5];      // 40 bit 
  uint8_t  checksum;     // 8 bit
} packet_t;

// Struct used for setting sensor values 
typedef struct {
  const char *name;
  float min;
  float max;
  const char *unit;  
} sensor_range_t;

// All sensors struct are mapped with min, max and unit
// Struct for seabird sensor
static const sensor_range_t seabird_sensors[5] =
{
  { "Temperature",        -2.0f,  25.0f,   "C"     },
  { "Conductivity",        3.0f,  5.5f,    "mS/cm" },
  { "Pressure",            0.0f,  30.0f, "dbar"    },
  { "Salinity",            32.0f, 37.0f,  "PSU"    },
  { "Specific Conductivity",   3.0f,  5.5f,  "mS/cm"   }
};

// Struct for midling seabird pressure data
static const sensor_range_t midling_seabird_pressure[5] =
{
  { "Mean",      0.0f,   30.0f,  "dBar" },
  { "Min",       0.0f,   30.0f,  "dBar" },
  { "Maks",      00.0f,  30.0f,  "dBar" },
  { "Std avvik", 0.0f,   10.0f,  "dBar" },
  { "EMPTY",     0.0f,   0.0f,   "NULL" }
};

// Struct for PNORS values from signature 500
static const sensor_range_t signature500_PNORS[5] =
{ 
  { "Battery",       12.0f,  48.0f,      "V"   },
  { "Sound speed",   1400.0f, 1600.0f,  "m/s" },
  { "Heading",       0.0f, 360.0f,      "deg" },
  { "Pitch",        -90.0f,  90.0f,     "deg" },
  { "Roll",         -180.0f,  180.0f,   "deg" }
};

// Struct for PNORC_1 values from signature_500
static const sensor_range_t signature500_PNORC_1[5] =
{ 
  { "C1: Cell position",  0.0f,  30.0f,  "m"   },
  { "C1: Speed",          0.0f,   5.0f,  "m/s" },
  { "C1: Direction",      0.0f, 360.0f,  "deg" },
  { "C2: Cell position",  0.0f,  30.0f,  "m"   },
  { "C2: Speed",          0.0f,   5.0f,  "m/s" }
};

// Struct for PNORC_2 values from signature_500
static const sensor_range_t signature500_PNORC_2[5] =
{ 
  { "C2: Direction",      0.0f, 360.0f,   "deg"  },
  { "C3: Cell position",  0.0f,  30.0f,   "m"    },
  { "C3: Speed",          0.0f,    5.0f,  "m/s"  },
  { "C3: Direction",      0.0f, 360.0f,   "deg"  },
  { "Chamber temp",      -10.0f,  50.0f,  "deg" }
};

// Variables used for the system. 
// Static variables keep their value between function calls.
// Volatile variables can be changed by interrupts.

static uint8_t rx_byte;
static uint8_t rx_frame[RX_FRAME_SIZE];
static volatile uint8_t rx_count = 0;
static volatile uint8_t rx_frame_ready = 0;

static inline float map_u8_to_value(uint8_t x, float min, float max);
static void USART2_StartRxTx(void);
static void USART1_StartRx(void);
static void process_seabird(const packet_t *p);
static void process_signature500_PNORS(const packet_t *p);
static void process_signature500_PNORC_1(const packet_t *p);
static void process_signature_500_PNORC_2(const packet_t *p);
static void process_seabird_avg_pressure(const packet_t *p);
static packet_t parse_packet_msb(const uint8_t f[RX_FRAME_SIZE]);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
static void process_packet(const packet_t *p);

// Variables and defines used for sending command from buoy
static volatile uint8_t  vcom_rx_byte;
static char              vcom_line[32];
static volatile uint8_t  vcom_idx = 0;
static volatile uint8_t  vcom_line_ready = 0;

static uint8_t calc_checksum_simple(const uint8_t *buf, uint8_t len);
static void prepare_packet(uint8_t prot_ver, uint8_t msg_id, uint32_t time, const uint8_t data[5]);
static void send_pending_packet_if_any(void);
static void handle_vcom_command(char *line);
static void read_time(uint32_t time_raw);
static bool is_leap_year(uint16_t year);
static uint8_t days_in_month(uint16_t year, uint8_t month);

// Default system functions genereted by MX
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART2_UART_Init(void);
void vcom_init(void);

int main(void)
{
  // initialization of the system, clocks, GPIO, UART and VCOM
  HAL_Init();
  SystemPower_Config();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_USART2_UART_Init();

  // Activate VCOM for debugging
  vcom_init();

  // Enable USART1 interrupt for receiving commands from terminal 
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  
  // Activate LED on board for debugging
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  // Flush the RX on USART2
  uint8_t dummy = 0;
  while (HAL_UART_Receive(&huart2, &dummy, 1, 10) == HAL_OK) {}
  // when no more data to read -> breaks out of loop, and buffer empty 
  HAL_Delay(100);

  // USART2 (M16) recieving side ready 
  USART2_StartRxTx();
  printf("USART2 RX/TX ready, receive and send data to observatory\r\n");
  HAL_Delay(100);

  // USART1 receiving side ready (VCOM commands)
  USART1_StartRx();
  printf("USART1 RX ready (Receiving side VCOM commands) \r\n");

  while (1)
  {
    // If we recieve data on RX line (USART2)
    if (rx_frame_ready)
    {
      // Reset flag for next incoming frame
      rx_frame_ready = 0;

      // Parse the received frame
      packet_t p = parse_packet_msb(rx_frame);

      // Print parsed packet
      printf("\r\n");
      printf("Parsed headers: prot_ver=%u, msg_id=%u, checksum=%u \r\n",(unsigned)p.prot_ver, (unsigned)p.msg_id, (unsigned)p.checksum);

      // Print incoming timestamp
      printf("Timestamp: %u \r\n", (unsigned)p.time);
      read_time(p.time);

      // Print raw data bytes for debugging
      printf("Data[0]: %u, Data[1]: %u, Data[2]: %u, Data[3]: %u, Data[4]: %u\r\n", (unsigned)p.data[0],(unsigned)p.data[1],(unsigned)p.data[2],(unsigned)p.data[3],(unsigned)p.data[4]);

      // Process packet based on msg_id
      process_packet(&p);

      // if we have received the last expected packet, send down new interval
      if (p.msg_id == 18) 
      {
        // Send down new interval  if one is ready
        send_pending_packet_if_any();
        
        // Stop interrupt-based RX before flushing
        HAL_UART_AbortReceive_IT(&huart2);

        // Flush any remaining bytes in RX buffer
        uint8_t dummy;
        while (HAL_UART_Receive(&huart2, &dummy, 1, 1) == HAL_OK)
        {
            // discard bytes in RX buffer
        }

        // Reset software RX state
        rx_count = 0;
        rx_frame_ready = 0;
        memset(rx_frame, 0, RX_FRAME_SIZE);

        // Restart interrupt RX for next measurement cycle
        USART2_StartRxTx();
    }
  }

    // Check if we are receiving commands in terminal
    if (vcom_line_ready)
    {
      // Reset flag
      vcom_line_ready = 0;
      // Handle the incomng command from terminal
      handle_vcom_command(vcom_line);
    }
  }
}

// Function for checking if a year is a leap year
static bool is_leap_year(uint16_t year)
{
  if ((year % 400u) == 0u) return true; // Leap year if dividiable by 400
  if ((year % 100u) == 0u) return false; // Is not leap year if dividiable by 100
  return ((year % 4u) == 0u); // Is a leap year if dividiable by 4
}

// Function for calculating number of days in a given month
static uint8_t days_in_month(uint16_t year, uint8_t month)
{
  // Hard coded number of days in each month
  static const uint8_t days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  // If the month is february and its a leap year, update to 29 days
  if (month == 2u && is_leap_year(year))
    return 29u;
  // Return the number of days for month, subtracting to get 0 based index
  return days[month - 1u];
}

// Function for converting raw time to readable format
static void read_time(uint32_t time_raw)
{
  // Starting from 01.01.1970 00:00
  // If time_raw = 1 means it has passed 10 minutes since 01.01.1970 00:00

  uint32_t unix_seconds = time_raw * 600u; // 10 min step resolution 
  uint32_t days = unix_seconds / 86400u;   // Number of full days since 01.01.1970
  uint32_t rem  = unix_seconds % 86400u;   // Find remanding seconds after calcutlating days

  uint32_t hour   = rem / 3600u; // Number of full hours since 00:00
  rem %= 3600u;                  // Find remainding seconds after hours
  uint32_t minute = rem / 60u;   // Number of full minutes since last full hour
  uint32_t second = rem % 60u;   // Find reaminding seconds after minutes

  uint16_t year = 1970u;   // Start year is 1970

  while (1)
  {
    uint32_t days_in_year = 365u;

    if (is_leap_year(year))
    {
      // Update number of days to 366 if its a leap year
      days_in_year = 366u;
    }

    // Check if the number of days left is less than days in a year
    if (days < days_in_year)
    {
      break;
    }

    // Subtract the number of days in a year from total days (removing a year)
    days -= days_in_year;
    // Add one year since we removed it from days
    year++;
  }

  // starting from january
  uint8_t month = 1u;
  while (1)
  {
    // Now knows the year, and starting month
    uint8_t days_in_current_month = days_in_month(year, month);

    // Checks if remainding days is less than number of days in a month
    if (days < days_in_current_month)
    {
      break;
    }

    // Remove one full month from days
    days -= days_in_current_month;
    // Goes to the next month
    month++;
  }

  // Add one since days is 0 based in this, and we want to start from day 1
  uint8_t day = (uint8_t)days + 1u;

  // Print out time in readable format
  printf("Time: %04u-%02u-%02u %02lu:%02lu:%02lu\r\n",
         year, month, day,
         (unsigned long)hour,
         (unsigned long)minute,
         (unsigned long)second);
}

// FUnction for handling incoming commands from the terminal
static void handle_vcom_command(char *line)
{
  // Start with interval value = 0
  unsigned long interval = 0;

  // Scaning incoming line, and extracting numbers after "PFF,"
  if (sscanf(line, "PFF,%lu", &interval) == 1)
  {
    if (interval > 65535UL)
    {
      // In case the interval is to large for 2 bytes, print error
      printf("Interval too large. Max is 65535 seconds.\r\n");
      return;
    }

    // Create data array, and fill it with 0 to begin with
    uint8_t data[5] = {0};

    // Data format is as following:
    data[0] = 0x01; // Important: this is the message ID down from bouy, 0x01-> change interval
    data[1] = (uint8_t)((interval >> 8) & 0xFF); // Contains the interval 1/2
    data[2] = (uint8_t)(interval & 0xFF);        // Contains the interval 2/2
    data[3] = 0x00; // Empty
    data[4] = 0x00; // Empty

    // Sent with prot=1, msg_id=0x20, time is empty (no need as it wont be processed), data
    // Prepare PFF. Prot_ver=1, msg_id=0x20 (not used as data[0] contains msg_id for downwards packet)
    // Time is set to 0 since we will not use it. Then the data packets
    prepare_packet(1, 0x20, 0, data);

    // Print out for debugging 
    printf("Prepared PFF interval packet: %lu s\r\n", interval);
    printf("Data bytes: %02X %02X %02X %02X %02X\r\n", data[0], data[1], data[2], data[3], data[4]);
  }
  else
  {
    // Print error if command is not in correct format
    printf("Unknown command: %s\r\n", line);
    printf("Use format: PFF,20\r\n");
  }
}

// Function for sending down packet if any is ready
static void send_pending_packet_if_any(void)
{
  // Checks flag
  if (pending_tx_ready)
  {
    printf("Sending packet in 2-byte chunks:\r\n");

    // Send PFF in 2 - byte packets
    for (int i = 0; i < RX_FRAME_SIZE; i += 2)
    {
      HAL_StatusTypeDef st = HAL_UART_Transmit(&huart2, &pending_tx_frame[i], 2, HAL_MAX_DELAY);

      // If 2 byte is sent, print out this
      if (st == HAL_OK)
      {
        printf("Sent: %02X %02X\r\n", pending_tx_frame[i], pending_tx_frame[i + 1]);
      }
      // If something went wrong, print in which chunk it failed
      else
      {
        printf("Transmit failed at chunk %d, status=%d\r\n", i / 2, (int)st);
        break;
      }

      // Checks if we are not in the last packet transmission
      if (i < RX_FRAME_SIZE - 2)
      {
        // Delay between packets, in order to fit the M16 modem
        HAL_Delay(1650);
      }
    }

    // Reset flag
    pending_tx_ready = 0;
  }
}

// Function for calculating checksum of the PFF packet
static uint8_t calc_checksum_simple(const uint8_t *buf, uint8_t len)
{
  // simple XOR checksum
  uint8_t cs = 0;
  // Loop through each byte in the buffer
  for (uint8_t i = 0; i < len; i++)
  {
    // XOR each byte in the buffer
    cs ^= buf[i];
  }
  return cs;
}

// Function for preparing the PFF packet
static void prepare_packet(uint8_t prot_ver, uint8_t msg_id, uint32_t time, const uint8_t data[5])
{
  // Sett the protprotocol version (2 bits, value of 1) and 6 bits of msg_id the first byte
  pending_tx_frame[0] = (uint8_t)(((prot_ver & 0x03) << 6) | ((msg_id >> 2) & 0x3F));
  // Set the remainng 2 bits of msg_id and the first 6 bits of time
  pending_tx_frame[1] = (uint8_t)(((msg_id & 0x03) << 6) | ((time >> 16) & 0x3F));
  // Set the remaindng 16 bits of time in byte 2 and 3
  pending_tx_frame[2] = (uint8_t)((time >> 8) & 0xFF);
  pending_tx_frame[3] = (uint8_t)(time & 0xFF);
  // Set the data bytes
  pending_tx_frame[4] = (data[0]);
  pending_tx_frame[5] = (data[1]);
  pending_tx_frame[6] = (data[2]);
  pending_tx_frame[7] = (data[3]);
  pending_tx_frame[8] = (data[4]);

  // Calculate checksum for packet going down
  pending_tx_frame[9] = calc_checksum_simple(pending_tx_frame, 9);
  // Update flag to indicate packet is ready
  pending_tx_ready = 1;
}

// Function for Setting up the VCOM, side for receiving commands from terminal 
void vcom_init(void)
{
  BspCOMInit.BaudRate   = 115200;             // 115 200 baud rate
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;  // 8 data bits
  BspCOMInit.StopBits   = COM_STOPBITS_1;     // 1 stop bit
  BspCOMInit.Parity     = COM_PARITY_NONE;    // No parity
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE; // No hardware flow control 
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
}

// Function for starting RX side on USART1
static void USART1_StartRx(void)
{
  HAL_StatusTypeDef st;

  // Reset VCOM flags
  vcom_idx = 0;
  vcom_line_ready = 0;

  
  st = HAL_UART_Receive_IT(&hcom_uart[COM1], (uint8_t *)&vcom_rx_byte, 1);
  if (st != HAL_OK)
  {
    printf("Failed to set up RX on USART1, status = %d\r\n", (int)st);
  }

  // Check status code (DEBUGGING)
  //printf("USART1_StartRx status = %d\r\n", (int)st);
}

// Inline function for mapping a uin8_t to float value based on min and max
static inline float map_u8_to_value(uint8_t x, float min, float max)
{ 
  // Formula: y = ymin + ((x * (ymax- ymin)) / 255)
  return min + ((float)x * (max - min) / 255.0f);
}

// Function for starting RX/TX side on USART2
static void USART2_StartRxTx(void)
{
  // Reseet RX state
  rx_count = 0;
  rx_frame_ready = 0;
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

// Process function for handling packets based on msg_id
void process_packet(const packet_t *p)
{
  printf("\r\n");
  printf("msg_id received: %u", (unsigned)p->msg_id);
  switch (p->msg_id)
  // Case for each of the MSG_ID used 
  {
    // msg_id 1 is seabird
    case 1:
      process_seabird(p);
      break;

    // msg_id 6 is seabird averaging pressure
    case 6:
      process_seabird_avg_pressure(p);
      break;

    // msg_id is signature PNORS
    case 16:
      process_signature500_PNORS(p);
      break;
    
    // msg_id is signature PNORC_1
    case 17:
      process_signature500_PNORC_1(p);
      break;
    
    // msg_id is signature PNORC_2
    case 18:
      process_signature_500_PNORC_2(p);
      break;

    // Deafult case for msg_id that has not been implmeented
    default:
      printf("Unknown msg_id: %u\r\n", (unsigned)p->msg_id);
      break;
  }
}

// Callback function for when a byte is received on either USARt2 or USART1(VCOM)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // Check if USART2 has received a byte
  if (huart->Instance == USART2)
  {
    // Check if we have the full frame, if not add the byte to the buffer
    if (!rx_frame_ready)
    {
      rx_frame[rx_count++] = rx_byte;

      // Reached full frame, reset flags
      if (rx_count >= RX_FRAME_SIZE)
      {
        rx_count = 0;
        rx_frame_ready = 1;
      }
    }
    // Wait to receive 1 byte, and store it in rx_byte and trigger an interrupt
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }

  // Check if the VCOM side has received a byte 
  else if (huart == &hcom_uart[COM1])
  {
    // Expect char for terminal, convert from vcom_rx_byte to char 
    char c = (char)vcom_rx_byte;

    // Check if we recieve commands from terminal (DEBUGGING)
    //printf("USART1 got char: %c\r\n", c);

    // Check if full line has been received
    if (!vcom_line_ready)
    {
      // If we recieve a \r\ or \n we know the user is done writing
      if (c == '\r' || c == '\n')
      {
        // Only set the line ready if we have content in the line
        if (vcom_idx > 0)
        {
          // Terminate the line, and reset the index for the next line
          vcom_line[vcom_idx] = '\0';
          vcom_idx = 0;
          vcom_line_ready = 1;
        }
      }
      // if we receive characters, add them to the line buffer
      else
      {
        // Check if there is space in the buffer before adding
        if (vcom_idx < sizeof(vcom_line) - 1)
        {
          vcom_line[vcom_idx++] = c;
        }
        else
        {
          vcom_idx = 0;
        }
      }
    }
    // Wait for the next byte to be received on VCOM
    HAL_UART_Receive_IT(&hcom_uart[COM1], (uint8_t *)&vcom_rx_byte, 1);
  }
}

// Function for parsing the received PFF packet
static packet_t parse_packet_msb(const uint8_t f[RX_FRAME_SIZE])
{
  packet_t p = {0};

  // Extract first the prot_version from the first 2 bits in the first byte
  p.prot_ver = (f[0] >> 6) & 0x03;

  // Extracting the msg_id from the last 6 bits in first byte, and the first 2 bits in second byte
  p.msg_id   = (uint8_t)(((f[0] & 0x3F) << 2) | (f[1] >> 6));

  // Extracting the time from the last 6 bits in second byte, and entire third and fourth byte
  p.time     = ((uint32_t)(f[1] & 0x3F) << 16) |
               ((uint32_t) f[2] << 8) |
               ((uint32_t) f[3]);

  // Extracting data bytes from byte 5 to byte 9
  p.data[0] = f[4];
  p.data[1] = f[5];
  p.data[2] = f[6];
  p.data[3] = f[7];
  p.data[4] = f[8];

  // Extracting checksum from the last byte
  p.checksum = f[9];

  return p;
}

// Functions for processing the different packets based on msg_id
// Printing out the data in given formats
void process_seabird(const packet_t *p)
{
  printf("\r\n");
  printf("SEABIRD VALUES:\r\n");

  float val = map_u8_to_value(p->data[0], seabird_sensors[0].min, seabird_sensors[0].max);
  printf("%s: %.2f %s\r\n",seabird_sensors[0].name, val, seabird_sensors[0].unit);

  float val2 = map_u8_to_value(p->data[1], seabird_sensors[1].min, seabird_sensors[1].max);
  printf("%s: %.2f %s\r\n",seabird_sensors[1].name, val2, seabird_sensors[1].unit);

  float val3 = map_u8_to_value(p->data[2], seabird_sensors[2].min, seabird_sensors[2].max);
  printf("%s: %.2f %s\r\n",seabird_sensors[2].name, val3, seabird_sensors[2].unit); 

  float val4 = map_u8_to_value(p->data[3], seabird_sensors[3].min, seabird_sensors[3].max);   
  printf("%s: %.2f %s\r\n",seabird_sensors[3].name, val4, seabird_sensors[3].unit);

  float val5 = map_u8_to_value(p->data[4], seabird_sensors[4].min, seabird_sensors[4].max);   
  printf("%s: %.2f %s\r\n",seabird_sensors[4].name, val5, seabird_sensors[4].unit);
  printf("\r\n");
}

void process_signature500_PNORS(const packet_t *p)
{
  printf("\r\n");
  printf("SIGNATURE 500 PNORS VALUES:\r\n");

  float val = map_u8_to_value(p->data[0], signature500_PNORS[0].min, signature500_PNORS[0].max);
  printf("%s: %.2f %s\r\n",signature500_PNORS[0].name, val, signature500_PNORS[0].unit);

  float val2 = map_u8_to_value(p->data[1], signature500_PNORS[1].min, signature500_PNORS[1].max);
  printf("%s: %.2f %s\r\n",signature500_PNORS[1].name, val2, signature500_PNORS[1].unit);

  float val3 = map_u8_to_value(p->data[2], signature500_PNORS[2].min, signature500_PNORS[2].max);
  printf("%s:%.2f %s\r\n",signature500_PNORS[2].name, val3, signature500_PNORS[2].unit); 

  float val4 = map_u8_to_value(p->data[3], signature500_PNORS[3].min, signature500_PNORS[3].max);   
  printf("%s: %.2f %s\r\n",signature500_PNORS[3].name, val4, signature500_PNORS[3].unit);

  float val5 = map_u8_to_value(p->data[4], signature500_PNORS[4].min, signature500_PNORS[4].max);   
  printf("%s: %.2f %s\r\n",signature500_PNORS[4].name, val5, signature500_PNORS[4].unit);
  printf("\r\n");
}

void process_signature500_PNORC_1(const packet_t *p)
{
  printf("\r\n");
  printf("SIGNATURE 500 PNORC_1 VALUES:\r\n");

  float val = map_u8_to_value(p->data[0], signature500_PNORC_1[0].min, signature500_PNORC_1[0].max);
  printf("%s: %.2f %s\r\n",signature500_PNORC_1[0].name, val, signature500_PNORC_1[0].unit);

  float val2 = map_u8_to_value(p->data[1], signature500_PNORC_1[1].min, signature500_PNORC_1[1].max);
  printf("%s: %.2f %s\r\n",signature500_PNORC_1[1].name, val2, signature500_PNORC_1[1].unit);

  float val3 = map_u8_to_value(p->data[2], signature500_PNORC_1[2].min, signature500_PNORC_1[2].max);
  printf("%s: %.2f %s\r\n",signature500_PNORC_1[2].name, val3, signature500_PNORC_1[2].unit); 

  float val4 = map_u8_to_value(p->data[3], signature500_PNORC_1[3].min, signature500_PNORC_1[3].max);   
  printf("%s: %.2f %s\r\n",signature500_PNORC_1[3].name, val4, signature500_PNORC_1[3].unit);

  float val5 = map_u8_to_value(p->data[4], signature500_PNORC_1[4].min, signature500_PNORC_1[4].max);   
  printf("%s: %.2f %s\r\n",signature500_PNORC_1[4].name, val5, signature500_PNORC_1[4].unit);
  printf("\r\n");
}

void process_signature_500_PNORC_2(const packet_t *p)
{
  printf("\r\n");
  printf("SIGNATURE 500 PNORC_2 VALUES:\r\n");

  float val = map_u8_to_value(p->data[0], signature500_PNORC_2[0].min, signature500_PNORC_2[0].max);
  printf("%s: %.2f %s\r\n",signature500_PNORC_2[0].name, val, signature500_PNORC_2[0].unit);

  float val2 = map_u8_to_value(p->data[1], signature500_PNORC_2[1].min, signature500_PNORC_2[1].max);
  printf("%s: %.2f %s\r\n",signature500_PNORC_2[1].name, val2, signature500_PNORC_2[1].unit);

  float val3 = map_u8_to_value(p->data[2], signature500_PNORC_2[2].min, signature500_PNORC_2[2].max);
  printf("%s: %.2f %s\r\n",signature500_PNORC_2[2].name, val3, signature500_PNORC_2[2].unit); 

  float val4 = map_u8_to_value(p->data[3], signature500_PNORC_2[3].min, signature500_PNORC_2[3].max);   
  printf("%s: %.2f %s\r\n",signature500_PNORC_2[3].name, val4, signature500_PNORC_2[3].unit);

  float val5 = map_u8_to_value(p->data[4], signature500_PNORC_2[4].min, signature500_PNORC_2[4].max);   
  printf("%s: %.2f %s\r\n",signature500_PNORC_2[4].name, val5, signature500_PNORC_2[4].unit);
  printf("\r\n");
}

void process_seabird_avg_pressure(const packet_t *p)
{
  printf("\r\n");
  printf("SEABIRD AVERAGE PRESSURE VALUES:\r\n");

  float val = map_u8_to_value(p->data[0], midling_seabird_pressure[0].min, midling_seabird_pressure[0].max);
  printf("%s: %.2f %s\r\n",midling_seabird_pressure[0].name, val, midling_seabird_pressure[0].unit);

  float val2 = map_u8_to_value(p->data[1], midling_seabird_pressure[1].min, midling_seabird_pressure[1].max);
  printf("%s: %.2f %s\r\n",midling_seabird_pressure[1].name, val2, midling_seabird_pressure[1].unit);

  float val3 = map_u8_to_value(p->data[2], midling_seabird_pressure[2].min, midling_seabird_pressure[2].max);
  printf("%s: %.2f %s\r\n",midling_seabird_pressure[2].name, val3, midling_seabird_pressure[2].unit);

  float val4 = map_u8_to_value(p->data[3], midling_seabird_pressure[3].min, midling_seabird_pressure[3].max);   
  printf("%s: %.2f %s\r\n",midling_seabird_pressure[3].name, val4, midling_seabird_pressure[3].unit); 

  // Expect nothing
  printf("%s: %.2f %s\r\n",midling_seabird_pressure[4].name, 0.0f, midling_seabird_pressure[4].unit);
  printf("\r\n");
}

// System clock configuration generated by MX
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

// System power configuration with SMPS supply, generated by MX
static void SystemPower_Config(void)
{
  HAL_PWREx_DisableUCPDDeadBattery();
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
}

// Function for initializing the instruction cache, generated by MX
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

// Function for initializing USART2, generated by MX
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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

// Function for initializing GPIO, generated by MX
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

// Callback function for timer interrupt, generated by MX
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
}

// Function for handling errors, generated by MX
void Error_Handler(void)
{
  printf("Landed in error handler");
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
