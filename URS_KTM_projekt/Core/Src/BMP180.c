#include "stm32f4xx_hal.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c2;
#define BMP180_I2C &hi2c2

#define BMP180_ADDRESS 0xEE  // adresa senzora

// podaci za kalibraciju senzora
short AC1 = 0;
short AC2 = 0;
short AC3 = 0;
unsigned short AC4 = 0;
unsigned short AC5 = 0;
unsigned short AC6 = 0;
short B1 = 0;
short B2 = 0;
short MB = 0;
short MC = 0;
short MD = 0;

// varijable za osnovne podatke senzora
long UT = 0;
short oss = 0;
long UP = 0;
long X1 = 0;
long X2 = 0;
long X3 = 0;
long B3 = 0;
long B5 = 0;
unsigned long B4 = 0;
long B6 = 0;
unsigned long B7 = 0;


long Temp = 0;



void read_calliberation_data (void)
{
	uint8_t Callib_Data[22] = {0}; // polje za pohranu kalibracijskih podataka
	uint16_t Callib_Start = 0xAA;  // početna adresa kalibracijskih podataka
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, Callib_Start, 1, Callib_Data,22, HAL_MAX_DELAY);

	// spajanje podataka u 16-bitne vrijednosti

	AC1 = ((Callib_Data[0] << 8) | Callib_Data[1]);
	AC2 = ((Callib_Data[2] << 8) | Callib_Data[3]);
	AC3 = ((Callib_Data[4] << 8) | Callib_Data[5]);
	AC4 = ((Callib_Data[6] << 8) | Callib_Data[7]);
	AC5 = ((Callib_Data[8] << 8) | Callib_Data[9]);
	AC6 = ((Callib_Data[10] << 8) | Callib_Data[11]);
	B1 = ((Callib_Data[12] << 8) | Callib_Data[13]);
	B2 = ((Callib_Data[14] << 8) | Callib_Data[15]);
	MB = ((Callib_Data[16] << 8) | Callib_Data[17]);
	MC = ((Callib_Data[18] << 8) | Callib_Data[19]);
	MD = ((Callib_Data[20] << 8) | Callib_Data[21]);

}


uint16_t Get_UTemp (void)
{
	uint8_t datatowrite = 0x2E;    // naredba za mjerenje temperature
	uint8_t Temp_RAW[2] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	HAL_Delay (5);  // pričekati 4.5 ms za mjerenje
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Temp_RAW, 2, 1000);
	return ((Temp_RAW[0]<<8) + Temp_RAW[1]);  // spajanje podataka u 16-bitne vrijednosti
}

float BMP180_GetTemp (void)
{
	UT = Get_UTemp();  // dohvaćanje neobrađenih podataka o temperaturi
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));  // obrada tih podataka
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	Temp = (B5+8)/(pow(2,4));
	return Temp/10.0;
}

// inicijalizacija
void BMP180_Start (void)
{
	read_calliberation_data();  // učitavanje kalibracijskih podataka
}

