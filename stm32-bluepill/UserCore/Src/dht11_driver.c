#include "dht11_driver.h"
#include "cmsis_os.h"

void DHT11_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint8_t DHT11_ReadData(uint16_t GPIO_Pin, uint8_t *temperature, uint8_t *humidity)
{
    uint8_t data[5] = {0};
    uint8_t bits_read = 0;

    /* Start signal — drive low then release */
    HAL_GPIO_WritePin(GPIOA, GPIO_Pin, GPIO_PIN_RESET);
    osDelay(20);
    HAL_GPIO_WritePin(GPIOA, GPIO_Pin, GPIO_PIN_SET);  /* release — pull-up takes it high */

    /* Wait for sensor response */
    uint32_t start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin) == GPIO_PIN_SET) {
        if (DWT->CYCCNT - start > 8000) return 0;  /* timeout */
    }
    start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin) == GPIO_PIN_RESET) {
        if (DWT->CYCCNT - start > 8000) return 0;
    }
    start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin) == GPIO_PIN_SET) {
        if (DWT->CYCCNT - start > 8000) return 0;
    }

    /* Read 40 bits */
    __disable_irq();
    for (uint8_t byte_idx = 0; byte_idx < 5; byte_idx++) {
        for (uint8_t bit_idx = 0; bit_idx < 8; bit_idx++) {
            uint32_t timeout = 50000;
            while (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin) == GPIO_PIN_RESET && timeout--);
            if (timeout == 0) { __enable_irq(); return 0; }

            start = DWT->CYCCNT;
            timeout = 50000;
            while (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin) == GPIO_PIN_SET && timeout--);
            uint32_t cycles = DWT->CYCCNT - start;

            data[byte_idx] <<= 1;
            if (cycles > 350) data[byte_idx] |= 1;
            bits_read++;
        }
    }
    __enable_irq();

    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (bits_read == 40 && data[4] == checksum) {
        *temperature = data[2];
        *humidity    = data[0];
        return 1;
    }

    return 0;
}
