#include "main.h"
#include <stdio.h>
#include <math.h>

# define TOTAL_DATA_COUNT 2000
# define RESET 0

//system clock is 24 Mhz

void SystemClock_Config(void);
void UART_init(void);
void UART_print(char *str);
void TIM1_CC_IRQHandler(void);
void ADC_init(void);
void TIM_init(void);
void Calculate_Frequency(void);
void UART_print_int(char *string, uint16_t value);
void reset(void);
void add_to_RMS(uint16_t value);
void UART_print_float(char *string, float value);
void print_hashes(uint8_t num);
float calculate_RMS(void);


uint16_t timer_length;
uint8_t conversion_ready = RESET;
uint16_t reading_data = RESET;
uint16_t values[TOTAL_DATA_COUNT] = {RESET};
uint16_t value_count = RESET;
uint32_t average = RESET;
uint16_t cross_count = RESET;
uint16_t max = RESET;
uint16_t min = 50000;
uint16_t hertz = 0;
uint8_t finished = 0;
uint8_t DC_voltage = 0;
float RMS_value = 0.0;
float voltage = 0.0;

//add .015 to dc voltage
// minus .03 to vpp unless 1vpp


int main(void)
{
	HAL_Init();
	SystemClock_Config();
	ADC_init();
	UART_init();
	TIM_init();



	while (1) {

		Calculate_Frequency();
		if(finished)
		{
			UART_print("\033[2J\033[H");
			float voltage = (((float)average / 4096.0) * 3300.0);

			if(DC_voltage)
			{
			  UART_print("|-------------DC--------------|\r\n");
			  UART_print("\r\n");
			  UART_print_float("Voltage: ", voltage / 1000.0);
			  print_hashes((int)(voltage / 100.0));
			}
			else
			{
			  UART_print("|-------------Freq------------|\r\n\r\n");
			  UART_print_int("Hertz: %d Hz\r\n\r\n", hertz);
			  UART_print("|-------------AC--------------|\r\n\r\n");




//			  UART_print_int("max: %d ", max);
//			  UART_print_int("min: %d \r\n", min);
//			  float vpp =  (((((1353.0 * ((float)(max - min))) - 1600.0) / 1000000.0) * 0.625) - 0.1);
			  float vpp = (((float)(max - min)) /4096.9) * 3.3;
//			  UART_print_int("regular vpp: %d \r\n", max - min);
//			  UART_print_int("calibrated vpp: %d \r\n", ((1353 * (max - min)) - 1600) / 10000);
			  UART_print_float("Vpp: ", vpp);





			  float RMS_num = calculate_RMS();
			  UART_print_float("RMS Voltage: ", RMS_num );
			  print_hashes((int)(RMS_num * 10));
			}
			UART_print("|----|----|----|----|----|----|\r\n");
			UART_print("0   0.5  1.0  1.5  2.0  2.5  3.0");
			reset();
			finished = 0;
		}
	}
}


float calculate_RMS()
{
	float result = 0.0;
	result = (((float)max / 4096.0) * 3.3);
//	UART_print_float("max to volts: ", result);                     // should be 3 volts
	result = (result / 1.414);
//	UART_print_float("divided by 1.414: ", result);                // 2.12
	result *= result;
//	UART_print_float("squared: ", result);                         //4.5
//	UART_print_float("voltage: ",(((float)average / 4096.0) * 3.3));		//1.5
	float voltageRMS = (((float)average / 4096.0) * 3.3);
//	UART_print_float("voltage: ",voltageRMS);
	result += (voltageRMS * voltageRMS);
//	UART_print_float("plus voltage squared: ", result);           // 6.75
	result = sqrt(result);
//	UART_print_float("root: ", result);                           // 2.59
	return result;
}



void print_hashes(uint8_t num)
{
	for(uint8_t i = 0; i < num + 1; i++)
	{
		UART_print("#");
	}
	UART_print("\r\n");
}





void UART_print_float(char *string, float value) {
    char buffer[50];
    int integer_part = (int)value;
    int decimal_part = (int)((value - integer_part) * 1000); // Keep 3 decimal places
    sprintf(buffer, "%s%d.%03d V\r\n\r\n", string, integer_part, decimal_part);
    UART_print(buffer);
}



void UART_print_int(char *string, uint16_t value)
{
	char stringify[100] = {0};
	sprintf(stringify,string ,value);
	UART_print(stringify);
}


void Calculate_Frequency()
{
	if((value_count < TOTAL_DATA_COUNT) & conversion_ready) //for when collecting data
	{
	  values[value_count++] = reading_data;
	  conversion_ready = 0;
	}
	if(value_count >= TOTAL_DATA_COUNT) // process data and reset
	{
	  for(uint32_t i = 0; i < TOTAL_DATA_COUNT; i++)
	  {
		  if(values[i] < min)
		  {
			  min = values[i];
		  }
		  if(values[i] > max)
		  {
			  max = values[i];
		  }
		  average += values[i];

	  }
	  average /= TOTAL_DATA_COUNT;




	  uint8_t above_average = 0;

	  if(values[0] > average)
	  {
		  above_average = 1;
	  }
	  else
	  {
		  above_average = 0;
	  }



	  for(uint32_t i = 0; i < TOTAL_DATA_COUNT; i++)
	  {
		  if(above_average & (values[i] < average))
		  {
			  above_average = 0;
			  cross_count++;
		  }
		  if(!above_average & (values[i] > average))
		  {
			  above_average = 1;
			  cross_count++;
		  }
	  }

	  hertz = (cross_count / 2);
	  finished = 1;

	}
}

void reset()
{
	average = 0;
	conversion_ready = 0;
	value_count = 0;
	cross_count = 0;
	min = 50000;
	max = 0;
	hertz = 0;
	RMS_value = 0.0;
	voltage = 0.0;
	for(uint16_t i = 0; i < 20000; i++) {}
}



void TIM2_IRQHandler(void) {

  if (TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~(TIM_SR_UIF);
    //Start a conversion
    ADC1->CR |= ADC_CR_ADSTART;
  }

}

void TIM_init(void) {

  // Turn on timer TIM2 RCC
  RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
  // ARR Timer Length Set to Sample Speed 5 - 1
  TIM2->ARR = 12000 - 1;

  // Update Interrupt Enable
  TIM2->DIER |= (TIM_DIER_UIE);
  // Clear interrupt status register for update event
  TIM2->SR &= ~(TIM_SR_UIF);
  // Start the timer / counter
  TIM2->CR1 |= TIM_CR1_CEN;
  // Count-Up Mode for TIM2
  TIM2->CR1 &= ~(TIM_CR1_DIR);
  // Enable interrupts for TIM2 in NVIC
  NVIC->ISER[0] = (1 << TIM2_IRQn);
  // Enable interrupts globally
  __enable_irq();

}






void ADC1_2_IRQHandler(void)
{
	//check to see if conversion is done
	if(ADC1->ISR & ADC_ISR_EOC)
	{
		//clear flag, set global variable and read into another global variable
		ADC1->ISR &= ~ADC_ISR_EOC;
		conversion_ready = 1;
		reading_data = ADC1->DR;
	}
}


void ADC_init(void) {
    // Enable the ADC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    // Set ADC clock to synchronous HCLK / 1
    ADC123_COMMON->CCR = (1 << ADC_CCR_CKMODE_Pos);

    // Power up the ADC and voltage regulator
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;

    // Wait for 20 µs for the voltage regulator to stabilize
    for (uint32_t i = 0; i < 50000; i++);

    // Configure PA0 as analog input
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER |= (GPIO_MODER_MODE0);  // PA0 in analog mode
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0);
    GPIOA->ASCR |= GPIO_ASCR_ASC0;  // Enable analog switch for PA0

    // Single-ended mode for channel 5 (PA0)
    ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);

    // Calibrate the ADC
    ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF); // Ensure ADC is disabled and single-ended calibration
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL); // Wait for calibration to complete

    // Enable the ADC
    ADC1->ISR |= (ADC_ISR_ADRDY); // Clear ready bit
    ADC1->CR |= (ADC_CR_ADEN);
    while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Wait for ADC to be ready

    // Configure sequence for single channel (channel 5)
    ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);

    // Configure for single conversion, 12-bit resolution, right-aligned data
    ADC1->CFGR = 0;

    // Configure sample time to 2.5 ADC clock cycles
    ADC1->SMPR1 &= ~(ADC_SMPR1_SMP5);
    ADC1->SMPR1 |= (0 << ADC_SMPR1_SMP5_Pos); // 2.5 cycles for channel 5

    // Enable end-of-conversion interrupt
    ADC1->IER |= (ADC_IER_EOC);
    // Enable ADC interrupt in NVIC
    NVIC_EnableIRQ(ADC1_2_IRQn);
    __enable_irq();
    ADC1->CR |= ADC_CR_ADSTART;

}



void UART_init(void)
{
	 // Enable clocks for GPIOA and USART2
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIOA clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // Enable USART2 clock

	// Configure PA2 as USART2_TX and PA3 as USART2_RX
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3); // Clear mode bits
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1); // Set alternate function mode

//	GPIOA->AFR[0] |= ~(GPIO_AFRL_AFSEL2_3);  // Set AF7 (USART2) for PA2
//	GPIOA->AFR[0] |= ~(GPIO_AFRL_AFSEL3_3); // Set AF7 (USART2) for PA3

	GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL2_Pos); // Set AF7 (USART2) for PA2
	GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL3_Pos); // Set AF7 (USART2) for PA3

	// Configure USART2 for 115200 baud rate (assuming 4 MHz clock)
	USART2->BRR = 208; // Set baud rate divisor for 115200 baud (4 MHz / (16 * 115200) ≈ 35)

	// Enable USART2, transmitter, and receiver and receive data register
	USART2->CR1 = (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE);
	//enable interrupts
	NVIC_EnableIRQ(USART2_IRQn);
}

void UART_print(char *str)
{
	while (*str != '\0') 												// Loop until the end of the string
	{
		while (!(USART2->ISR & USART_ISR_TXE)) {}
		USART2->TDR = *str++;
	}

}

void USART2_IRQHandler(void)
{

	//if receiving data interrupt
	if(USART2->ISR & USART_ISR_RXNE)
	{
		char single[2];
		//put input anded with the read register into a variable
		sprintf(single,"%c", (char)(USART2->RDR & USART_RDR_RDR));

		if(single[0] == 's')
		{
			DC_voltage = DC_voltage ? 0 : 1;
		}
		USART2->ISR &= ~(USART_ISR_RXNE);
	}

}





















/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
