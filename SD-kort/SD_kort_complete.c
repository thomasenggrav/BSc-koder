#include "main.h"
#include "app_filex.h"
#include <string.h>

COM_InitTypeDef BspCOMInit;

SD_HandleTypeDef hsd1;
RTC_HandleTypeDef hrtc;

void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void vcom_init(void);
static void MX_RTC_Init(void);

RTC_TimeTypeDef gTime;
RTC_DateTypeDef gDate;

// Dato og klokkeslett for å sette RTC 
#define RTC_INIT_HOURS   13
#define RTC_INIT_MINUTES 30
#define RTC_INIT_SECONDS 0

#define RTC_INIT_WEEKDAY RTC_WEEKDAY_MONDAY
#define RTC_INIT_MONTH   RTC_MONTH_MAY
#define RTC_INIT_DATE    05
#define RTC_INIT_YEAR    25

#define FILEX_MEDIA_CACHE_SIZE 4096

// Filnavn og tilhørende header for seabird 37
#define FILENAME_SEABIRD "SEABIRD.CSV"
#define HEADER_STRING_SEABIRD "Timestamp,Temperature,Conductivity,Pressure,Salinity,Specific Conductivity\r\n"

// Filnavn og tihørende header for signature 500
#define FILENAME_SIGNATURE "SIGNATURE.CSV"
#define HEADER_STRING_SIGNATURE "C1_CP, C1_S, C1_DIR, C2_CP, C2_S, C2_DIR, C3_CP, C3_S, C3_DIR, CH_TEMP \r\n"

// Filnavn og tilhørende header for bruk av testing av funksjoner
#define FILENAME_TEST "TEST_1.CSV"
#define HEADER_STRING_TEST "Row 1, Row 2, Row 3, Row 4, Row 5\r\n"

FX_MEDIA sd_media;
FX_FILE sd_file;
UCHAR media_cache[FILEX_MEDIA_CACHE_SIZE];
UINT status;

static void SD_open_media(void);
static void SD_close_media(void);
static void SD_CreateFileAndHeader(const CHAR *filename, const char *header);
static void SD_WriteToFile(const CHAR *filename, const uint8_t *data_array, uint8_t array_size);
static void SD_read_file(const CHAR *filename);

typedef struct {
    uint32_t timestamp_seabird;
    uint8_t temperature_rec;
    uint8_t conductivity_rec;
    uint8_t pressure_rec;
    uint8_t salinity_rec;
    uint8_t specific_conductivity_rec;
} Seabird_rec_data_t;

int main(void)
{
    HAL_Init();
    SystemPower_Config();
    SystemClock_Config();
    vcom_init();
    MX_GPIO_Init();
    MX_ICACHE_Init();
    MX_SDMMC1_SD_Init();
    MX_FileX_Init();
    MX_RTC_Init();

    // FLush ut "start dato", hvis ikke starter den på 2000 tallet 
    HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
    HAL_Delay(100);
    HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
    HAL_Delay(500);

    // Åpne SD kort
    SD_open_media();

    // Kommandoer for å opprette nye filer med headere
    //SD_CreateFileAndHeader(FILENAME_SEABIRD, HEADER_STRING_SEABIRD);
    //HAL_Delay(500);
    //SD_CreateFileAndHeader(FILENAME_SIGNATURE, HEADER_STRING_SIGNATURE);
    //HAL_Delay(500);
    //SD_CreateFileAndHeader(FILENAME_TEST, HEADER_STRING_TEST);
    //HAL_Delay(500);

    // Sekvens for å skrive test verdier til test fil
    // Test skriving med temp array, erstatt med faktiske datapunkter senere
    //uint8_t temp_array[5] = {10, 20, 30, 40, 50};
    //SD_WriteToFile(FILENAME_TEST, temp_array, 5);
    //HAL_Delay(500);

    // Lese av innholdet i filen for å verifisere at alt er skrevet riktig
    //SD_read_file(FILENAME_SEABIRD);
    //HAL_Delay(500);
    //SD_read_file(FILENAME_SIGNATURE);
    SD_read_file(FILENAME_TEST);
    HAL_Delay(200);
    SD_read_file(FILENAME_SIGNATURE);
    HAL_Delay(200);
    SD_read_file(FILENAME_SEABIRD);
    HAL_Delay(200);

    // Denne kan som oftest sløyfes da mediet skal være åpent gjennom hele programmet
    //SD_close_media();

    while (1)
    { 
    }
}

// Funksjon for å lese av innhold i gitt fil
void SD_read_file(const CHAR *filename)
{
  UINT status;
  CHAR read_buffer[500];
  ULONG bytes_read;

  // Åpne filen man skal lese av 
  status = fx_file_open(&sd_media, &sd_file, (CHAR *)filename, FX_OPEN_FOR_READ);
  if (status != FX_SUCCESS)
  {
    printf("fx_file_open for read feilet! status=%u\r\n", status);
    return;
  }

  // Lese av filen man har åpnet
  status = fx_file_read(&sd_file, read_buffer, sizeof(read_buffer) - 1, &bytes_read);
  if (status != FX_SUCCESS)
  {
    printf("fx_file_read feilet! status=%u\r\n", status);
    fx_file_close(&sd_file);
    return;
  }

  // Lese helt til en plass har verdien ¨\0¨: slutten av linjen
  read_buffer[bytes_read] = '\0'; 
  printf("Innhold i filen %s:\r\n%s\r\n", filename, read_buffer);

  // Lukke filen etter avlesning
  fx_file_close(&sd_file);
}

// Funksjon for å initialisere SD kortet
void SD_open_media(void)
{
  // Initialisering av filex, oppsett av SD - kort
  UINT status;
  status = fx_media_open(&sd_media,
                         "SD_DISK",
                         fx_stm32_sd_driver,
                         0,
                         media_cache,
                         sizeof(media_cache));

  if (status != FX_SUCCESS)
  {
    printf("fx_media_open feilet! status=%u\r\n", status);
  }
  else
  {
    printf("SD-kort aapnet.\r\n");
  }
}

// Funksjon for å avslutte/ lukke SD kort initaliseringen
void SD_close_media(void)
{
  UINT status;
  status = fx_media_close(&sd_media);
  if (status != FX_SUCCESS)
  {
    printf("fx_media_close feilet! status=%u\r\n", status);
  }
  else
  {
    printf("SD-kort lukket.\r\n");
  }
}

// Funksjon for å lage en dedikert fil med tihørende header 
void SD_CreateFileAndHeader(const CHAR *filename, const char *header)
{
  UINT status;
  UINT status_write;
  UINT status_ok;

  // Sjekke først om filen eksisterer, dersom dette er tilfellet lukk filen
  status = fx_file_open(&sd_media, &sd_file, (CHAR *)filename, FX_OPEN_FOR_READ);
  if (status == FX_SUCCESS)
  {
    printf("Filnavn %s finnes allerede.\r\n", filename);
    fx_file_close(&sd_file);
  }

  // Dersom den ikke finnes, lage den og skrive inn ønsket header til den
  else if (status == FX_NOT_FOUND)
  {
    printf("Filnavn %s finnes ikke. Oppretter filen.\r\n", filename);

    status = fx_file_create(&sd_media, (CHAR *)filename);
    if (status != FX_SUCCESS)
    {
      printf("fx_file_create feilet! status=%u\r\n", status);
      return;
    }
    else if(status == FX_SUCCESS)
    {
      printf("Filen %s ble opprettet.\r\n", filename);

      // Skrive inn ønsket "header" i filen
      status_write = fx_file_open(&sd_media, &sd_file, (CHAR *)filename, FX_OPEN_FOR_WRITE);
      if (status_write == FX_SUCCESS)
      {
        status_ok = fx_file_write(&sd_file, header, strlen(header));
        if (status_ok == FX_SUCCESS)
        {
          printf("Header skrevet til filen %s.\r\n", filename);
          printf("Header ble folgende: \r\n");
          printf("%s\r\n", header);
          fx_media_flush(&sd_media);
        }
        else 
        {
          printf("Skriving av header til fil feilet! status=%u\r\n", status_ok);
        }
      } 
      else
      {
        printf("fx_file_open for write feilet! status=%u\r\n", status);
        return;
      }
      // Lukke filen etter opprettelse
      fx_file_close(&sd_file);
    }
  }
}

// Funksjon for å skrive inn et array på en bestemt størrelse til dedikert fil
void SD_WriteToFile(const CHAR *filename, const uint8_t *data_array, uint8_t array_size)
{

  UINT status;
  CHAR timestamp_string[32];
  CHAR line_buffer[96];

  // Forventer at array skal være 5 i lengde (Data til PFF som skal skrives)
  if (array_size != 5)
  {
    printf("Feil: forventer 5 verdier, fikk %u.\r\n", array_size);
    return;
  }

  // Les RTC i riktig rekkefølge
  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

  // Lage en tidsstempling 
  snprintf(timestamp_string, sizeof(timestamp_string),"20%02d-%02d-%02d %02d:%02d:%02d", gDate.Year, gDate.Month, gDate.Date, gTime.Hours, gTime.Minutes, gTime.Seconds);

  // Lage en komplett CSV-linje, med tidsstemplingen i første index som standard
  snprintf(line_buffer, sizeof(line_buffer), "%s,%u,%u,%u,%u,%u\r\n", timestamp_string, data_array[0], data_array[1], data_array[2], data_array[3], data_array[4]);

  printf("Data skrives til SD kort: %s", line_buffer);

  // Åpne fila for skriving
  status = fx_file_open(&sd_media, &sd_file, (CHAR *)filename, FX_OPEN_FOR_WRITE);
  if (status != FX_SUCCESS)
  {
      printf("fx_file_open for write feilet! status=%u\r\n", status);
      return;
  }

  // Gå til slutten av fila lokalisert med byte_offset
  status = fx_file_seek(&sd_file, sd_file.fx_file_current_file_size);
  if (status != FX_SUCCESS)
  {
    // byte_offset vil endre verdi for hver gang, noe som resulterer i at man skriver alltid "bak" data
    printf("fx_file_seek feilet! status=%u\r\n", status);
    fx_file_close(&sd_file);
    return;
  }

  // Skriv linjen "line_buffer" på denne lokasjon
  status = fx_file_write(&sd_file, line_buffer, strlen(line_buffer));
  if (status != FX_SUCCESS)
  {
    printf("fx_file_write feilet! status=%u\r\n", status);
    fx_file_close(&sd_file);
    return;
  }

  // Flush til SD-kort, "flytte" data fra lokal RAM e.l og inn på SD kort
  status = fx_media_flush(&sd_media);
  if (status != FX_SUCCESS)
  {
    printf("fx_media_flush feilet! status=%u\r\n", status);
  }

  // Lukk fila
  status = fx_file_close(&sd_file);
  if (status != FX_SUCCESS)
  {
    printf("fx_file_close feilet! status=%u\r\n", status);
    return;
  }
  printf("Data skrevet til filen %s.\r\n", filename);
}

// RTC klokke initialiserings funksjon (MX)
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

  // Sjekk om RTC allerede er satt tidligere
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2)
  {
    // Sett tid og dato kun første gang
    sTime.Hours = RTC_INIT_HOURS;
    sTime.Minutes = RTC_INIT_MINUTES;
    sTime.Seconds = RTC_INIT_SECONDS;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
       Error_Handler();
    }

    sDate.WeekDay = RTC_INIT_WEEKDAY;
    sDate.Month = RTC_INIT_MONTH;
    sDate.Date = RTC_INIT_DATE;
    sDate.Year = RTC_INIT_YEAR;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
      Error_Handler();
    }
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
    }
}

// Funksjon for å initialisere VCOM for print av status/ resultat (MX)
void vcom_init(void)
{
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
}

// Funksjon for å initalisere systemklokken (MX)
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

// Funskjon for initalisering av power (MX)
static void SystemPower_Config(void)
{
  HAL_PWREx_DisableUCPDDeadBattery();

  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
}

// Initialisering av CACHE minnet (MX)
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

// Initialisering av SDMMC1 (MX)
static void MX_SDMMC1_SD_Init(void)
{
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 60;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
}

// Initialisering av GPIO pins (MX)
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

// Initalisering av ticks (MX)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
}

// Feilhåndtering (MX)
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */


