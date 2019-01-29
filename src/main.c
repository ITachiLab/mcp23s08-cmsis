/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f10x.h"
#include "stm32f1xx_nucleo.h"
			
#define WRITE_ADDR	0b01000000
#define READ_ADDR	0b01000001

void spi_write(uint8_t *data, uint32_t size) {
	// Select slave
	GPIOA->BRR = GPIO_Pin_1;

	// Transmission
	for (uint32_t i = 0; i < size; i++) {
		SPI1->DR = data[i];

		while (!(SPI1->SR & SPI_SR_TXE));
	}

	while (SPI1->SR & SPI_SR_BSY);

	// Deselect SPI slave
	GPIOA->BSRR = GPIO_Pin_1;
}

void delay_ms(uint16_t delay) {
	TIM2->CNT = 0;
	TIM2->EGR |= TIM_EGR_UG;

	uint16_t clk_value = TIM2->CNT;
	while (TIM2->CNT < (clk_value + delay));
}

int main(void)
{
	// Enable GPIOA and SPI1 clocks
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN);

	// Enable TIM2 clock
	RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN);

	// Set prescaler on TIM2 to generate 1 ms ticks
	TIM2->PSC = 63999;
	TIM2->CR1 |= TIM_CR1_CEN;

	GPIOA->CRL |= (
			GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5_1 | GPIO_CRL_MODE5_0 | // PA5 as AF, 50 MHz, push-pull
			GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0 | // PA7 as AF, 50 MHz, push-pull
			GPIO_CRL_MODE1_1 // PA1 as GP, 2 MHz, push-pull
			);

	GPIOA->CRL &= ~(
			GPIO_CRL_CNF5_0 | // PA5 as AF, 50 MHz, push-pull
			GPIO_CRL_CNF7_0 | // PA7 as AF, 50 MHz, push-pull
			GPIO_CRL_CNF1_0 | GPIO_CRL_CNF1_1); // PA1 as GP, 2 MHz, push-pull

	// Deselect SPI slave
	GPIOA->BSRR = GPIO_Pin_1;

	// Enable SPI, frequency = fclk / 8, master mode
	SPI1->CR1 |= (SPI_CR1_BR_1 | SPI_CR1_MSTR);
	SPI1->CR1 |= SPI_CR1_SPE;

	// Set output direction on GP0
	// Writing to IODIR (0x00) register
	uint8_t set_direction[] = {
		WRITE_ADDR,
		0,
		0b11111110
	};

	// Set high output on GP0
	// Writing to OLAT (0x0A) register
	uint8_t set_output_1[] = {
		WRITE_ADDR,
		0x0A,
		0b00000001
	};

	// Set low output on GP0
	// Writing to OLAT (0x0A) register
	uint8_t set_output_0[] = {
		WRITE_ADDR,
		0x0A,		// Select register OLAT
		0b00000000  // Set output
	};

	spi_write(set_direction, sizeof(set_direction));

	for(;;) {
		spi_write(set_output_1, sizeof(set_output_1));
		delay_ms(1000);
		spi_write(set_output_0, sizeof(set_output_0));
		delay_ms(1000);
	}
}
