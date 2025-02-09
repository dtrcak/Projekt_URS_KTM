/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "BMP180.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint8_t debounce_flag = 0;  // Zastavica za debounce
volatile uint8_t pwm_manual = 0; // Zastavica za manualni mod PWM-a
volatile uint8_t pwm_konst = 0; // Zastavica za prebacivanje PWM-a na konstantnu vrijednost
uint16_t rawValue;
uint16_t pwmValue;
float temperature;
uint16_t value;
uint16_t dutyCycle;
float speed;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/******* FUNKCIJE ZA LCD ********/
void lcd_naredba(char podatak, uint8_t adresa)
{
	char podaci[4], left, right;  // definiranje podataka koji  će biti poslani

	left = podatak & 0xF0;  // 4 viša bita (MSB)
	right = (podatak << 4) & 0xF0;  // 4 niža bita (LSB)

	podaci[0] = left | 0x0C; // naredba za zapisivanje (RS = 0, EN = 1)
	podaci[1] = left | 0x08;  // naredba za otpuštanje enable linije (EN = 0)
	podaci[2] = right | 0x0C;
	podaci[3] = right | 0x08;

	HAL_I2C_Master_Transmit(&hi2c1, adresa<<1, podaci, 4, 100);	  // slanje svih podataka na LCD
}

void lcd_podatak(char podatak, uint8_t adresa)
{
	char podaci[4], left, right;  // definiranje podatak koji  će biti poslani

	left = podatak & 0xF0;  // 4 viša bita (MSB)
	right = (podatak << 4) & 0xF0;  // 4 niža bita (LSB)

	podaci[0] = left | 0x0D;  // naredba za zapisivanje podataka (RS = 1, EN = 1)
	podaci[1] = left | 0x09;  // naredba za otpuštanje enable linije (EN = 0)
	podaci[2] = right | 0x0D;
	podaci[3] = right | 0x09;

	HAL_I2C_Master_Transmit(&hi2c1, adresa<<1, podaci, 4, 100);
}

void lcd_set_cursor(uint8_t line, uint8_t addr) {
    if (line == 0) {
        lcd_naredba(0x80, addr); // postavlja kursor na početak prve linije
    } else if (line == 1) {
        lcd_naredba(0xC0, addr); // postavlja kursor na početak druge linije
    }
}
/******** FUNKCIJE ZA LCD *******/




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /********* LCD ADRESE *********/
  	uint8_t addr = 0x27;  // adresa LCD-a
  	char sucelje = 0x38;  // naredba koja definira tip LCD-a (8-bitni, dvije linije)
    char initial0 = 0x03; // naredba za reesetiranje
    char bit_4 = 0x02;  // naredba za 4-bitni režim rada
    char ekran0 = 0x00;  // naredba za brisanje ekrana
    char ekran1 = 0x01;  // naredba za pomicanje kursora
    char kursor = 0x04;  // naredba za kontrolu kursora
    char ukljuci = 0x0F;  // naredba za uključenje LCD-a
    char pocetak = 0x02;  // naredba za postavljanje kursora na početak (1. red, 1. pozicija)

    char prva = 0x80;  // postavljanje kursora na prvu liniju
    char druga = 0xC0;  // postavljanje kursora na drugu liniju
    char str0[16] = "";  // polje za prvi red
  	char str1[16] = "";  // polje za drugi red
    uint8_t i;
    /******** LCD ADRESE *******/


    /************ LCD INICIJALIZACIJA *************************/
       HAL_Delay(15);
       lcd_naredba(initial0,addr);  // slanje 0x03 kako bi se LCD resetirao u 8-bitni mod
       HAL_Delay(5);
       lcd_naredba(initial0,addr);  // ponovo slanje
       HAL_Delay(1);
       lcd_naredba(initial0,addr);  // ponovo slanje
       HAL_Delay(1);

       lcd_naredba(bit_4,addr);  // slanje 0x02 za prelazak u 4-bitni mod
       lcd_naredba(sucelje,addr);  // postavljanje sučelja
       lcd_naredba(ekran1,addr);
       lcd_naredba(ekran0,addr);
       lcd_naredba(ekran0,addr);
       lcd_naredba(ekran1,addr);	// proces brisanja zaslona
       lcd_naredba(kursor,addr);	// namještanje kursora
       lcd_naredba(ukljuci,addr);	// uključivanje ekrana, kursora i treptanja kursora

       HAL_Delay(5);

       lcd_naredba(pocetak,addr);	// postavljanje kursora na početnu poziciju

       HAL_Delay(2);
       /**************************** LCD INICIJALIZACIJA *****************/


       lcd_naredba(ekran1,addr);	// šalje 0x01 kako bi se zaslon očistio
       HAL_Delay(2);
       lcd_naredba(prva,addr);	// šalje 0x80 za postavljanje kursora na početak prve linije
       HAL_Delay(2);

       for(i = 0; i<sizeof(str0); i++){			// ispisivanje teksta iz polja za prvu liniju
            lcd_podatak(str0[i], addr);			// slanje jednog po jednog podatka na LCD
       }

       HAL_Delay(5);
       lcd_naredba(druga,addr);		// šalje 0xC0 za postavljanje kursora na početak druge linije

       for(i = 0; i<sizeof(str1); i++){			// ispisivanje teksta iz polja na drugu liniju
            lcd_podatak(str1[i], addr);			// slanje jednoga po jednog podatka na LCD
       }



       /************************ BMP180 ********************/
       HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  //pokretanje PWM-a na timeru 2
       BMP180_Start();  // Inicijalizacija senzora

       /*********************** BMP180 ***********************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(pwm_manual % 2 == 0 && pwm_konst % 2 == 0){
		  temperature = BMP180_GetTemp();  // očitavanje vrijednosti temperature sa senzora
		  value = (((temperature + 40) * 4095) / (85 + 40));  // izračun vrijednosti za PWM
		  dutyCycle = ((value * 1000) / 4095);
		  speed = ((value * 1600) / 4095);  // izračun brzine za prikaz na LCD-u

		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, dutyCycle);  // postavljanje PWM vrije

		  char temp_str[16];
		  sprintf(temp_str, "TEMP: %.2f C   ", temperature);	// prikaz temperature na LCD-u
		  lcd_set_cursor(0, addr);       // Postavi kursor na prvu liniju
		  for (uint8_t i = 0; i < 16; i++) {
			  	 lcd_podatak(temp_str[i], addr); // Ispisivanje jednog po jednog karaktera na LCD
		  }

		  char speed_str[16];
		  sprintf(speed_str, "BRZINA: %.0f      ", speed);		// prikaz brzine ma LCD-u
		  lcd_set_cursor(1, addr);       // Postavi kursor na drugu liniju
		  for (uint8_t i = 0; i < 16; i++) {
			  lcd_podatak(speed_str[i], addr); // Ispisivanje jednog po jednog karaktera na LCD
		  }

		  HAL_Delay(2000);

	  	  }else if(pwm_manual % 2 == 1){

	  		  HAL_ADC_Start(&hadc1);
	  		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  // pokretanje ADC-a
	  		  rawValue = HAL_ADC_GetValue(&hadc1);		// čitanje vrijednosti s analognog ulaza
	  		  pwmValue = (rawValue * __HAL_TIM_GET_AUTORELOAD(&htim2)) / 4095;	// proračun pwm vrijednosti iz vrijednosti analognog ulaza
	  		  speed = ((rawValue * 1600) / 4095);

	  		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwmValue);	// postavljanje PWm vrijednosti

	  		  char text_str[16];
	  		  sprintf(text_str, "Manualni mod    ", speed);
	  		  lcd_set_cursor(0, addr);		// postavi kursor na prvu liniju
	  		  for (uint8_t i = 0; i < 16; i++) {
	  			  lcd_podatak(text_str[i], addr); // ispis jednog po jednog karaktera na LCD
	  		  }

	  		  char speed_str[16];
	  		  sprintf(speed_str, "Brzina: %.0f    ", speed);		// prikaz brzine na LCD-u
	  		  lcd_set_cursor(1, addr);       // postavi kursor na drugu liniju
	  		  for (uint8_t i = 0; i < 16; i++) {
	  			  lcd_podatak(speed_str[i], addr); 	// ispis jednog po jednog karaktera na LC
	  		  }

	  		  HAL_Delay(2000);

	  	  	  }else if(pwm_konst % 2 == 1){

	  	  		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);	// postavljanje fiksne vrijednosti PWM-a
	  	  		  temperature = BMP180_GetTemp();		// očitanje temperature sa senzora
	  	  		  char text_str[16];
	  	  		  sprintf(text_str, "Konstantni mod    ", speed);
	  	  		  lcd_set_cursor(0, addr);
	  	  		  for (uint8_t i = 0; i < 16; i++) {
	  	  			  lcd_podatak(text_str[i], addr);
	  	  		  }
	  	  		  char temp_str[16];
	  	  		  sprintf(temp_str, "TEMP: %.2f C    ", temperature);	// ispis temperature na LCD-u
	  	  		  lcd_set_cursor(1, addr);
	  	  		  for (uint8_t i = 0; i < 16; i++) {
	  	  			  lcd_podatak(temp_str[i], addr);
	  	  		  }

	  	  		  HAL_Delay(2000);
	  	  	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }



}
  /* USER CODE END 3 */


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_8 && debounce_flag == 0) {  // provjera je li pritisnuto tipkalo na pinu PC8
        debounce_flag = 1; 		// postavlja zastavicu da se spriječi višestruko aktiviranje
        HAL_TIM_Base_Start_IT(&htim6); // pokreni TIM6 za otklanjanje bouncinga
    }
    if (GPIO_Pin == GPIO_PIN_4 && debounce_flag == 0) {		// provjera je li pritisnuto tipkalo na pinu PA4
    	debounce_flag = 1; 		// postavlja zastavicu da se spriječi višestruko aktiviranje
    	HAL_TIM_Base_Start_IT(&htim6); 		// pokreni TIM6 za otklanjanje bouncinga
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) { // provjerite je li TIM6 generirao prekid
        HAL_TIM_Base_Stop_IT(&htim6); // zaustavi TIM6
        debounce_flag = 0; // resetiraj zastavicu za debounce kako bi se omogućilo ponovno aktiviranje prekida

        // provjeri je li tipkalo još uvijek pritisnuto
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) { // Ako je tipkalo pritisnuto
            pwm_manual++;		// povećaj brojač za manualni mod
        }

        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET){
        	pwm_konst++;		// povečaj brojač za konstantni mod
        }
    }
}

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
  while (1){
	  }
  }

  /* USER CODE END Error_Handler_Debug */


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
