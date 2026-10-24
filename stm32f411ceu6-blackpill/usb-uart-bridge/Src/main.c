/**
 * @file    main.c
 * @brief   USB-UART bridge for the Blackpill (STM32F411CEU6).
 *
 * The Blackpill's USB-C port appears on the PC as a serial port (USB CDC,
 * /dev/ttyACM* or COMx), and everything is passed through to USART1 on PA9
 * (TX) and PA10 (RX), at 3.3 V logic. The baud rate, parity and stop bits
 * follow whatever the terminal program sets (115200 8N1 until it does).
 *
 * Used to reach the PCB's terminal on J7 without a separate USB-serial
 * adapter. See README.md for the wiring.
 *
 * Clock: 25 MHz crystal -> PLL -> 96 MHz core, 48 MHz USB, PCLK2 96 MHz
 * (same as robot). The HAL and ST's USB device library do the USB; USART1
 * and its DMA are set up at register level in uart.c.
 */
#include "main.h"
#include "usb_device.h"
#include "bridge.h"
#include "bridge_port.h"

#define LED_PIN     13u     /* PC13, blue LED, lit when low */

static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;
    GPIOC->BSRR = 1u << LED_PIN;                    /* off */
    GPIOC->MODER = (GPIOC->MODER & ~(3u << (2u * LED_PIN))) | (1u << (2u * LED_PIN));
}

void board_led(bool on)
{
    GPIOC->BSRR = on ? 1u << (LED_PIN + 16u) : 1u << LED_PIN;
}

uint32_t board_millis(void)
{
    return HAL_GetTick();
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    led_init();
    bridge_init();
    uart_init();
    MX_USB_DEVICE_Init();

    for (;;)
        bridge_poll();
}

/* Called by HAL_Init(). */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/* HSE 25 MHz, PLL M 25, N 192, P 2 -> 96 MHz, Q 4 -> 48 MHz USB. */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 25;
    osc.PLL.PLLN = 192;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
        Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK)
        Error_Handler();
}

/* Something went badly wrong (clock didn't start, hard fault, ...): flash
 * the LED very fast forever. Doesn't need interrupts or the clock setup. */
void Error_Handler(void)
{
    __disable_irq();
    led_init();
    for (;;) {
        GPIOC->ODR ^= 1u << LED_PIN;
        for (volatile uint32_t i = 0; i < 100000u; i++) {}
    }
}
