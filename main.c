/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Unified TX/RX firmware
  *          PB12=HIGH  → TRANSMITTER  (boot-time + 200ms polling)
  *          PB12=LOW   → RECEIVER     (boot-time + 200ms polling)
  *
  *          CAN role override (PCAN-View remote frame or data frame):
  *          ID 0x100 → force TRANSMITTER (until reset)
  *          ID 0x200 → force RECEIVER    (until reset)
  *          Both PB12 and CAN override coexist — CAN wins once received.
  ******************************************************************************
  */
//Timers:
//  TIM2 → CH2 PWM output  → PA1
//  TIM3 → CH1 Input Capture → PA6
//         Base interrupt ON
//         IC interrupt ON
//
//UART:
//  USART1 → 9600 baud → PA9(TX) PA10(RX)
//  USART2 → 9600 baud → PA2(TX) PA3(RX)
//
//GPIO outputs:
//  PB0  → E32 M0+M1 shorted (LOW=Normal, HIGH=Configure)
//  PB12 → Input pull-down → HIGH=TX / LOW=RX
//  PC13 → Output → LED
//
//I2C1:
//  PB6 SCL, PB7 SDA
//  Speed = 100kHz
//
//CAN (debug + role control — PCAN-View):
//  PA11 → CAN_RX
//  PA12 → CAN_TX
//  250 kbps
//
//  CAN IDs used:
//   0x100  →  Set role = TRANSMITTER  (send any frame with this ID from PCAN-View)
//   0x200  →  Set role = RECEIVER     (send any frame with this ID from PCAN-View)
//   0x101  →  TX/RX debug data output [FREQ(4B big-endian)][RPM(4B big-endian)]
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
typedef enum
{
    ROLE_IDLE        = 0,
    ROLE_TRANSMITTER = 1,
    ROLE_RECEIVER    = 2
} DeviceRole;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
/* ── Signal loss timeout ── */
#define SIGNAL_TIMEOUT_MS   2000   /* show "NO SIGNAL" after 2s of no capture */
/* ── Role pin ── */
#define TX_SELECT_PORT      GPIOB
#define TX_SELECT_PIN       GPIO_PIN_12

/* ── E32 mode pin ── */
#define E32_M0_PORT         GPIOB
#define E32_M0_PIN          GPIO_PIN_0

/* ── E32 operating mode ── */
#define NORMAL              0
#define CONFIGURE           1
#define E32_MODE            NORMAL

#if E32_MODE == CONFIGURE
    #define E32_ApplyMode() HAL_GPIO_WritePin(E32_M0_PORT, E32_M0_PIN, GPIO_PIN_SET)
#else
    #define E32_ApplyMode() HAL_GPIO_WritePin(E32_M0_PORT, E32_M0_PIN, GPIO_PIN_RESET)
#endif

/* ── UART aliases ── */
#define FTDI_UART           huart1    /* USART1 PA9/PA10 → FTDI/PC  */
#define E32_UART            huart2    /* USART2 PA2/PA3  → E32 LoRa */

/* ── OLED addresses ── */
#define OLED_ADDR_PRIMARY   0x78
#define OLED_ADDR_FALLBACK  0x7A

/* ── RX buffer ── */
#define RX_BUF_SIZE         80

/* ── Timer clock: HSI 8MHz, PSC=79 → 100kHz ── */
#define TIMER_CLK           100000.0f
#define PPR                 1

/* ── Averaging ── */
#define AVG_LOW             4
#define AVG_MID             16
#define AVG_HIGH            64
#define AVG_MAX             64
#define MA_WINDOW           10

/* ── TX timing ── */
#define TX_INTERVAL_MS      500
#define E32_TX_DELAY_MS     200

/* ── CAN IDs ── */
#define CAN_DEBUG_ID        0x101   /* outbound data frames (FREQ + RPM)  */
#define CAN_CMD_TX_ID       0x100   /* inbound: set role = TRANSMITTER    */
#define CAN_CMD_RX_ID       0x200   /* inbound: set role = RECEIVER       */
/* USER CODE END PD */

/* USER CODE BEGIN PV */
/* ── Role ── */
static DeviceRole g_role = ROLE_IDLE;

/* ── CAN role override ──────────────────────────────────────────────────────
 * Set to 1 by the CAN RX ISR when a 0x100 or 0x200 frame arrives.
 * Once set, the 200ms PB12 polling loop will NO LONGER trigger a reset
 * based on the pin — the CAN command is "sticky" until the board resets.
 * Both mechanisms stay active; CAN simply wins while this flag is set.
 * ────────────────────────────────────────────────────────────────────────── */
static volatile uint8_t    g_can_override_active = 0;
static volatile DeviceRole g_can_requested_role  = ROLE_IDLE;

/* ── OLED presence flag ── */
static uint8_t oled_present = 0;

/* ── TX variables ── */
static volatile uint32_t overflow_count = 0;
static uint32_t cap_buf[AVG_MAX + 1];
static uint8_t  cap_idx    = 0;
static uint8_t  cur_avg    = AVG_MID;
static uint8_t  meas_ready = 0;
volatile float  g_Frequency = 0.0f;

/* ── Moving average ── */
static float   ma_buf[MA_WINDOW] = {0};
static uint8_t ma_idx   = 0;
static uint8_t ma_count = 0;

/* ── RX variables ── */
static char    rx_buf[RX_BUF_SIZE];
static uint8_t rx_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float   moving_average(float v);
static void    E32_SetMode(void);
static void    E32_Transmit(uint32_t freq, uint32_t rpm);
static void    TX_OLED_Display(uint32_t freq, uint32_t rpm);
static uint8_t OLED_Verify(void);
static void    Parse_And_Display(char *line);
static void    TX_Init(void);
static void    TX_Loop(void);
static void    RX_Init(void);
static void    RX_Loop(void);
static void    IDLE_Loop(void);
static void    CAN_Config(void);
static void    CAN_SendData(uint32_t freq, uint32_t rpm);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ══════════════════════════════════════════════
 * MOVING AVERAGE
 * ══════════════════════════════════════════════ */
static float moving_average(float new_freq)
{
    if (ma_count > 0)
    {
        float last = ma_buf[(ma_idx == 0) ?
                    (MA_WINDOW - 1) : (ma_idx - 1)];
        if (last > 0.0f)
        {
            float ratio = new_freq / last;
            if (ratio < 0.5f || ratio > 2.0f)
            {
                ma_count = 0;
                ma_idx   = 0;
                for (uint8_t i = 0; i < MA_WINDOW; i++)
                    ma_buf[i] = 0.0f;
            }
        }
    }
    ma_buf[ma_idx] = new_freq;
    ma_idx = (ma_idx + 1) % MA_WINDOW;
    if (ma_count < MA_WINDOW) ma_count++;

    float sum = 0.0f;
    for (uint8_t i = 0; i < ma_count; i++)
        sum += ma_buf[i];
    return sum / (float)ma_count;
}

/* ══════════════════════════════════════════════
 * E32 MODE
 * ══════════════════════════════════════════════ */
static void E32_SetMode(void)
{
    E32_ApplyMode();
    HAL_Delay(100);
}

/* ══════════════════════════════════════════════
 * E32 TRANSMIT
 * ══════════════════════════════════════════════ */
static void E32_Transmit(uint32_t freq, uint32_t rpm)
{
    char buf[64];
    int  len = snprintf(buf, sizeof(buf),
                        "FREQ:%lu RPM:%lu\r\n",
                        freq, rpm);

    HAL_UART_Transmit(&E32_UART,
                      (uint8_t *)buf, (uint16_t)len, 200);
    HAL_Delay(E32_TX_DELAY_MS);

    HAL_UART_Transmit(&FTDI_UART,
                      (uint8_t *)buf, (uint16_t)len, 200);
}

/* ══════════════════════════════════════════════
 * CAN CONFIG
 * Filter bank 0  — accept ALL IDs (mask = 0x000 / 0x000)
 * This lets us receive 0x100 and 0x200 role commands as well as
 * any other frames without needing multiple filter banks.
 * ══════════════════════════════════════════════ */
static void CAN_Config(void)
{
    /* Skip filter + start if CAN already running */
    if (HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_LISTENING)
    {
        CAN_FilterTypeDef filterConfig;
        filterConfig.FilterBank           = 0;
        filterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
        filterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
        filterConfig.FilterIdHigh         = 0x0000;
        filterConfig.FilterIdLow          = 0x0000;
        filterConfig.FilterMaskIdHigh     = 0x0000;
        filterConfig.FilterMaskIdLow      = 0x0000;
        filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        filterConfig.FilterActivation     = ENABLE;
        filterConfig.SlaveStartFilterBank = 14;

        if (HAL_CAN_ConfigFilter(&hcan, &filterConfig) != HAL_OK)
            Error_Handler();

        if (HAL_CAN_Start(&hcan) != HAL_OK)
            Error_Handler();
    }

    /* ALWAYS re-enable notification — gets cleared after role switch */
    if (HAL_CAN_ActivateNotification(&hcan,
            CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();
}
/* ══════════════════════════════════════════════
 * CAN TX — debug data output
 * ID 0x101, 8-byte big-endian [FREQ(4B)][RPM(4B)]
 * ══════════════════════════════════════════════ */
static void CAN_SendData(uint32_t freq, uint32_t rpm)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t  txData[8];
    uint32_t txMailbox;

    txHeader.StdId              = CAN_DEBUG_ID;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    txData[0] = (uint8_t)(freq >> 24);
    txData[1] = (uint8_t)(freq >> 16);
    txData[2] = (uint8_t)(freq >>  8);
    txData[3] = (uint8_t)(freq);
    txData[4] = (uint8_t)(rpm  >> 24);
    txData[5] = (uint8_t)(rpm  >> 16);
    txData[6] = (uint8_t)(rpm  >>  8);
    txData[7] = (uint8_t)(rpm);

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
        HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
}

/* ══════════════════════════════════════════════
 * CAN RX FIFO0 INTERRUPT CALLBACK
 *
 * Called by HAL when a CAN frame arrives in FIFO0.
 *
 * ID 0x100 → request switch to TRANSMITTER
 * ID 0x200 → request switch to RECEIVER
 *
 * We set g_can_override_active and g_can_requested_role, then
 * let the main loop apply the switch via NVIC_SystemReset().
 * Doing the reset here (inside ISR) is safe but we avoid it to
 * keep the ISR short and not lose the FTDI "switching…" message.
 * ══════════════════════════════════════════════ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0,
                             &rxHeader, rxData) != HAL_OK)
        return;

    /* Only act on standard (11-bit) IDs */
    if (rxHeader.IDE != CAN_ID_STD)
        return;

    if (rxHeader.StdId == CAN_CMD_TX_ID)          /* 0x100 → TX */
    {
        g_can_requested_role  = ROLE_TRANSMITTER;
        g_can_override_active = 1;
    }
    else if (rxHeader.StdId == CAN_CMD_RX_ID)     /* 0x200 → RX */
    {
        g_can_requested_role  = ROLE_RECEIVER;
        g_can_override_active = 1;
    }
    /* All other IDs (e.g. 0x101 echoes) are silently ignored */
}

/* ══════════════════════════════════════════════
 * TIMER CALLBACKS — guarded by g_role
 * ══════════════════════════════════════════════ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (g_role == ROLE_TRANSMITTER && htim->Instance == TIM3)
        overflow_count++;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (g_role != ROLE_TRANSMITTER)                 return;
    if (htim->Instance != TIM3)                     return;
    if (htim->Channel  != HAL_TIM_ACTIVE_CHANNEL_1) return;

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    uint16_t captured = (uint16_t)HAL_TIM_ReadCapturedValue(
                                   htim, TIM_CHANNEL_1);
    uint32_t ovf = overflow_count;

    if ((TIM3->SR & TIM_SR_UIF) && (captured < 0x8000))
        ovf--;

    uint32_t timestamp = (ovf * 65536UL) + (uint32_t)captured;
    cap_buf[cap_idx]   = timestamp;
    cap_idx++;

    if (cap_idx <= cur_avg) return;

    uint32_t total_ticks = cap_buf[cur_avg] - cap_buf[0];
    if (total_ticks > 0)
    {
        float avg_period = (float)total_ticks / (float)cur_avg;
        float new_freq   = TIMER_CLK / avg_period;

        if      (new_freq <  100.0f) cur_avg = AVG_LOW;
        else if (new_freq < 1000.0f) cur_avg = AVG_MID;
        else                         cur_avg = AVG_HIGH;

        g_Frequency = new_freq;
        meas_ready  = 1;
    }
    cap_idx = 0;
}

/* ══════════════════════════════════════════════
 * OLED HELPERS
 * ══════════════════════════════════════════════ */
static uint8_t OLED_Verify(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1,
        OLED_ADDR_PRIMARY,  3, 100) == HAL_OK) return 1;
    if (HAL_I2C_IsDeviceReady(&hi2c1,
        OLED_ADDR_FALLBACK, 3, 100) == HAL_OK) return 1;
    return 0;
}

/* ══════════════════════════════════════════════
 * TX OLED DISPLAY
 * Row 0  : "Transmitter"
 * Row 16 : "FREQ:xxx Hz"
 * Row 35 : "RPM :xxx"
 * ══════════════════════════════════════════════ */
static void TX_OLED_Display(uint32_t freq, uint32_t rpm)
{
    if (!oled_present) return;

    char buf[22];
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Transmitter", &Font_7x10, SSD1306_COLOR_WHITE);

    snprintf(buf, sizeof(buf), "FREQ:%lu Hz", freq);
    SSD1306_GotoXY(0, 16);
    SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

    snprintf(buf, sizeof(buf), "RPM :%lu", rpm);
    SSD1306_GotoXY(0, 35);
    SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_UpdateScreen();
}

/* ══════════════════════════════════════════════
 * PARSE AND DISPLAY (RX side)
 * ══════════════════════════════════════════════ */
static void Parse_And_Display(char *line)
{
    int freq     = 0;
    int rval     = 0;
    int gen_freq = 0;

    char *p = strstr(line, "FREQ:");
    if (p) sscanf(p, "FREQ:%d", &freq);

    if      (strstr(line, "RPM:"))
        sscanf(strstr(line, "RPM:"), "RPM:%d", &rval);
    else if (strstr(line, "RM:"))
        sscanf(strstr(line, "RM:"),  "RM:%d",  &rval);
    else if (strstr(line, "RP:"))
        sscanf(strstr(line, "RP:"),  "RP:%d",  &rval);
    else if (strstr(line, "R:"))
        sscanf(strstr(line, "R:"),   "R:%d",   &rval);
    else if (strstr(line, "M:"))
        sscanf(strstr(line, "M:"),   "M:%d",   &rval);

    if (freq > 0)
    {
        uint32_t timer_clk = 8000000UL;
        uint32_t total_div = timer_clk / (uint32_t)freq;

        uint32_t arr = 999;
        uint32_t psc = total_div / (arr + 1);
        if (psc == 0) psc = 1;
        arr = (total_div / psc) - 1;
        if (arr > 65535) arr = 65535;
        if (psc > 65535) psc = 65535;

        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        htim2.Instance->PSC  = psc - 1;
        htim2.Instance->ARR  = arr;
        htim2.Instance->CCR2 = (arr + 1) / 2;
        htim2.Instance->CNT  = 0;
        htim2.Instance->EGR  = TIM_EGR_UG;
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

        gen_freq = (int)(timer_clk / (psc * (arr + 1)));
    }

    if (freq > 0)
        CAN_SendData((uint32_t)freq, (uint32_t)rval);

    if (!oled_present) return;

    char l1[22], l2[22], l3[22];
    snprintf(l1, sizeof(l1), "FREQ=%d Hz", freq);
    snprintf(l2, sizeof(l2), "RPM =%d",    rval);
    snprintf(l3, sizeof(l3), "GEN =%d Hz", gen_freq);

    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_GotoXY(0,  0); SSD1306_Puts("Receiver", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 16); SSD1306_Puts(l1, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 32); SSD1306_Puts(l2, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 48); SSD1306_Puts(l3, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
}

/* ══════════════════════════════════════════════
 * TX INIT + LOOP
 * ══════════════════════════════════════════════ */
static void TX_Init(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(800);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(200);

    E32_SetMode();
    CAN_Config();

    const char *msg = "=== TRANSMITTER READY (HSI 8MHz) ===\r\n";
    HAL_UART_Transmit(&FTDI_UART,
                      (uint8_t *)msg, strlen(msg), 200);

    oled_present = OLED_Verify();
    if (!oled_present) { HAL_Delay(50); oled_present = OLED_Verify(); }

    if (oled_present)
    {
        SSD1306_Init();
        SSD1306_Fill(SSD1306_COLOR_BLACK);
        SSD1306_GotoXY(0,  0); SSD1306_Puts("Transmitter", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 16); SSD1306_Puts("FREQ: ----",  &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 35); SSD1306_Puts("RPM:  ----",  &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_UpdateScreen();
    }
    else
    {
        const char *no_oled = "OLED not detected — continuing without display\r\n";
        HAL_UART_Transmit(&FTDI_UART, (uint8_t *)no_oled, strlen(no_oled), 200);
    }

    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
}
static void Apply_Role_Switch(DeviceRole new_role)
{
    char sw_msg[60];
    snprintf(sw_msg, sizeof(sw_msg),
             "Switching to %s via %s...\r\n",
             (new_role == ROLE_TRANSMITTER) ? "TRANSMITTER" : "RECEIVER",
             g_can_override_active ? "CAN" : "PB12");
    HAL_UART_Transmit(&FTDI_UART, (uint8_t *)sw_msg, strlen(sw_msg), 200);

    /* ── Stop everything cleanly ── */
    HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

    /* ── Reset ALL stale state ── */
    overflow_count = 0;
    cap_idx        = 0;
    cur_avg        = AVG_MID;
    meas_ready     = 0;
    g_Frequency    = 0.0f;
    ma_idx         = 0;
    ma_count       = 0;
    for (uint8_t i = 0; i < MA_WINDOW; i++) ma_buf[i] = 0.0f;
    rx_idx = 0;
    memset(rx_buf, 0, sizeof(rx_buf));

    /* ── Force OLED re-init ── */
    oled_present = 0;

    /* ── Clear CAN override so PB12 works again after this switch ── */
  //  g_can_override_active = 0;

    g_role = new_role;

    HAL_Delay(100);   /* let peripherals settle */

    if (new_role == ROLE_TRANSMITTER)
    {
        TX_Init();   /* restarts TIM3 IC + base, re-probes OLED */
        TX_Loop();
    }
    else
    {
        RX_Init();   /* restarts PWM TIM2, re-probes OLED */
        RX_Loop();
    }
}
/* ── TX_Loop: role-switch logic handles BOTH PB12 and CAN override ── */
static void TX_Loop(void)
{
    uint32_t last_tx_tick  = 0;
    uint32_t last_pin_tick = 0;
    uint32_t last_meas_tick = 0;    /* ← ADD THIS */
    uint8_t  signal_lost    = 0;    /* ← ADD THIS */

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ── Role-switch check every 200ms ────────────────────────────────
         * Priority:
         *   1. CAN override (g_can_override_active set by ISR)
         *   2. PB12 pin (original behaviour, active only when no CAN cmd)
         * ──────────────────────────────────────────────────────────────── */
        if ((now - last_pin_tick) >= 200)
        {
            last_pin_tick = now;

            DeviceRole requested_role = ROLE_IDLE;

            if (g_can_override_active)
            {
                /* CAN command wins */
                requested_role = g_can_requested_role;
            }
            else
            {
                /* PB12 original logic */
                uint8_t tx_pin = HAL_GPIO_ReadPin(TX_SELECT_PORT, TX_SELECT_PIN);
                requested_role = tx_pin ? ROLE_TRANSMITTER : ROLE_RECEIVER;
            }

            if (requested_role != g_role)
            {
                char sw_msg[60];
                snprintf(sw_msg, sizeof(sw_msg),
                         "Switching to %s via %s...\r\n",
                         (requested_role == ROLE_TRANSMITTER) ? "TRANSMITTER" : "RECEIVER",
                         g_can_override_active ? "CAN" : "PB12");
                HAL_UART_Transmit(&FTDI_UART,
                                  (uint8_t *)sw_msg, strlen(sw_msg), 200);
                HAL_Delay(50);
                Apply_Role_Switch(requested_role);
            }
        }

        /* ── Normal TX measurement + transmit ── */
        /* ── Normal TX measurement + transmit ── */
                if (meas_ready)
                {
                    meas_ready      = 0;
                    last_meas_tick  = now;    // ← record when last valid capture arrived

                    /* Clear signal-lost state if it was set */
                    if (signal_lost)
                    {
                        signal_lost = 0;
                        /* Immediately redraw OLED with live data on recovery */
                    }

                    float freq_smooth = moving_average(g_Frequency);

                    if ((now - last_tx_tick) >= TX_INTERVAL_MS)
                    {
                        last_tx_tick      = now;
                        uint32_t freq_int = (uint32_t)(freq_smooth + 0.5f);
                        uint32_t rpm_int  = (freq_int * 60U) / PPR;

                        E32_Transmit(freq_int, rpm_int);
                        CAN_SendData(freq_int, rpm_int);
                        TX_OLED_Display(freq_int, rpm_int);
                    }
                }

                /* ── Signal loss detection ── */
                if ((now - last_meas_tick) >= SIGNAL_TIMEOUT_MS && !signal_lost)
                {
                    signal_lost = 1;

                    /* UART warning */
                    const char *lost_msg = "SIGNAL LOST\r\n";
                    HAL_UART_Transmit(&FTDI_UART,
                                      (uint8_t *)lost_msg, strlen(lost_msg), 200);

                    /* OLED — show NO SIGNAL */
                    if (oled_present)
                    {
                        SSD1306_Fill(SSD1306_COLOR_BLACK);
                        SSD1306_GotoXY(0,  0);
                        SSD1306_Puts("Transmitter",  &Font_7x10, SSD1306_COLOR_WHITE);
                        SSD1306_GotoXY(0, 16);
                        SSD1306_Puts("NO SIGNAL",    &Font_7x10, SSD1306_COLOR_WHITE);
                        SSD1306_GotoXY(0, 35);
                        SSD1306_Puts("FREQ: ----",   &Font_7x10, SSD1306_COLOR_WHITE);
                        SSD1306_GotoXY(0, 48);
                        SSD1306_Puts("RPM : ----",   &Font_7x10, SSD1306_COLOR_WHITE);
                        SSD1306_UpdateScreen();
                    }
                }
    }
}

/* ══════════════════════════════════════════════
 * RX INIT + LOOP
 * ══════════════════════════════════════════════ */
static void RX_Init(void)
{
    HAL_TIM_IC_Stop_IT(&htim3,  TIM_CHANNEL_1);
    HAL_TIM_Base_Stop_IT(&htim3);

    for (int i = 0; i < 3; i++)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(150);
    }

    E32_SetMode();
    CAN_Config();

    oled_present = OLED_Verify();
    if (!oled_present) { HAL_Delay(50); oled_present = OLED_Verify(); }

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    if (oled_present)
    {
        SSD1306_Init();
        SSD1306_Fill(SSD1306_COLOR_BLACK);
        SSD1306_GotoXY(0,  0); SSD1306_Puts("Receiver",   &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 16); SSD1306_Puts("FREQ= ----", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 35); SSD1306_Puts("RPM = ----", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_UpdateScreen();
    }

    const char *msg = "=== RECEIVER READY (HSI 8MHz) ===\r\n";
    HAL_UART_Transmit(&FTDI_UART, (uint8_t *)msg, strlen(msg), 200);

    if (!oled_present)
    {
        const char *no_oled = "OLED not detected — continuing without display\r\n";
        HAL_UART_Transmit(&FTDI_UART, (uint8_t *)no_oled, strlen(no_oled), 200);
    }
}

/* ── RX_Loop: role-switch logic handles BOTH PB12 and CAN override ── */
static void RX_Loop(void)
{
    uint32_t last_pin_tick = 0;
    uint32_t last_rx_tick  = HAL_GetTick();   /* grace period from role entry */
    uint8_t  rx_signal_lost = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ── Role-switch check every 200ms ── (unchanged) */
        if ((now - last_pin_tick) >= 200)
        {
            last_pin_tick = now;

            DeviceRole requested_role = ROLE_IDLE;

            if (g_can_override_active)
            {
                requested_role = g_can_requested_role;
            }
            else
            {
                uint8_t tx_pin = HAL_GPIO_ReadPin(TX_SELECT_PORT, TX_SELECT_PIN);
                requested_role = tx_pin ? ROLE_TRANSMITTER : ROLE_RECEIVER;
            }

            if (requested_role != g_role)
            {
                char sw_msg[60];
                snprintf(sw_msg, sizeof(sw_msg),
                         "Switching to %s via %s...\r\n",
                         (requested_role == ROLE_TRANSMITTER) ? "TRANSMITTER" : "RECEIVER",
                         g_can_override_active ? "CAN" : "PB12");
                HAL_UART_Transmit(&FTDI_UART,
                                  (uint8_t *)sw_msg, strlen(sw_msg), 200);
                HAL_Delay(50);
                Apply_Role_Switch(requested_role);
            }
        }

        /* ── Receive from E32 on USART2 ── */
        uint8_t byte;
        if (HAL_UART_Receive(&huart2, &byte, 1, 50) == HAL_OK)
        {
            if (byte == '\n')
            {
                rx_buf[rx_idx] = '\0';

                /* Valid line arrived → refresh timeout + clear lost flag */
                last_rx_tick = now;
                rx_signal_lost = 0;

                Parse_And_Display(rx_buf);

                HAL_UART_Transmit(&huart1,
                                  (uint8_t *)rx_buf, rx_idx, 200);
                HAL_UART_Transmit(&huart1,
                                  (uint8_t *)"\r\n", 2, 20);

                HAL_UART_Transmit(&huart3,
                                  (uint8_t *)rx_buf, rx_idx, 200);
                HAL_UART_Transmit(&huart3,
                                  (uint8_t *)"\r\n", 2, 20);

                rx_idx = 0;
                memset(rx_buf, 0, sizeof(rx_buf));
            }
            else if (byte != '\r' && rx_idx < RX_BUF_SIZE - 1)
            {
                rx_buf[rx_idx++] = (char)byte;
            }
        }

        /* ── Signal loss detection (RX side) ──────────────────────────── */
        if ((now - last_rx_tick) >= SIGNAL_TIMEOUT_MS && !rx_signal_lost)
        {
            rx_signal_lost = 1;

            /* Freeze PA1 low instead of holding last frequency forever */
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

            const char *lost_msg = "SIGNAL LOST\r\n";
            HAL_UART_Transmit(&FTDI_UART,
                              (uint8_t *)lost_msg, strlen(lost_msg), 200);

            if (oled_present)
            {
                SSD1306_Fill(SSD1306_COLOR_BLACK);
                SSD1306_GotoXY(0,  0);
                SSD1306_Puts("Receiver",   &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(0, 16);
                SSD1306_Puts("NO SIGNAL",  &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(0, 32);
                SSD1306_Puts("FREQ= ----", &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(0, 48);
                SSD1306_Puts("RPM = ----", &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_UpdateScreen();
            }
        }
    }
}
/* ══════════════════════════════════════════════
 * IDLE LOOP
 * ══════════════════════════════════════════════ */
static void IDLE_Loop(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500);
        NVIC_SystemReset();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_TIM2_Init();
    MX_CAN_Init();

    /* USER CODE BEGIN 2 */

    /* Debounce delay before reading pin */
    HAL_Delay(100);

    /* ── Boot-time role resolution ─────────────────────────────────────────
     * CAN interrupt is not yet active here (CAN_Config is called inside
     * TX_Init / RX_Init), so boot role always comes from PB12.
     * After CAN_Config runs, the ISR can set g_can_override_active and the
     * main loop will reset into the new role within 200ms.
     * ────────────────────────────────────────────────────────────────────── */
    uint8_t tx_pin = (HAL_GPIO_ReadPin(TX_SELECT_PORT, TX_SELECT_PIN)
                      == GPIO_PIN_SET) ? 1 : 0;

    g_role = tx_pin ? ROLE_TRANSMITTER : ROLE_RECEIVER;

    char role_msg[60];
    snprintf(role_msg, sizeof(role_msg),
             "PB12=%d ROLE=%d (1=TX,2=RX) | CAN 0x100=TX 0x200=RX\r\n",
             tx_pin, (int)g_role);
    HAL_UART_Transmit(&FTDI_UART,
                      (uint8_t *)role_msg, strlen(role_msg), 300);

    switch (g_role)
    {
        case ROLE_TRANSMITTER:
            TX_Init();
            TX_Loop();
            break;
        case ROLE_RECEIVER:
            RX_Init();
            RX_Loop();
            break;
        default:
            IDLE_Loop();
            break;
    }

    /* USER CODE END 2 */

    while (1) {}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
