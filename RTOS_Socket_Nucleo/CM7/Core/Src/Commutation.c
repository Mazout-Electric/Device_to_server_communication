/*
 * Commutation.c
 *
 *  Created on: Jan 10, 2025
 *      Author: pirda
 */

#include "Commutation.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

#define CCRValue_BufferSize     37
#define CCRValue_NSize			72
volatile uint16_t _index = 0;
volatile uint32_t _duty = 0;
volatile uint8_t step = 0;
volatile uint64_t counter_ = 0;
volatile uint32_t arr4_ = 10000;
volatile uint32_t arr1_ = 30000;

ALIGN_32BYTES (uint32_t DiscontinuousSineCCRValue_Buffer[CCRValue_BufferSize]) =
{
  14999, 17603, 20128, 22498, 24640, 26488, 27988, 29093, 29770,
  29998, 29770, 29093, 27988, 26488, 24640, 22498, 20128, 17603,
  14999, 12394, 9869, 7499, 5357, 3509, 2009, 904, 227, 1, 227,
  904, 2009, 3509, 5357, 7499, 9869, 12394, 14999
};

uint32_t CCRValue_30000[CCRValue_BufferSize] =
{
	14999, 17603, 20128, 22498, 24640, 26488, 27988, 29093, 29770,
	29998, 29770, 29093, 27988, 26488, 24640, 22498, 20128, 17603,
	14999, 12394, 9869, 7499, 5357, 3509, 2009, 904, 227, 1, 227,
	904, 2009, 3509, 5357, 7499, 9869, 12394, 14999
};

uint32_t CCRValue_7500[CCRValue_BufferSize] =
{
		3749,4400,5031,5623,6158,6620,6995,7271,7441,7498,7441,7271,6995
		,6620,6158,5623,5031,4400,3749,3097,2466,1874,1339,877,502,226,56,0
		,56,226,502,877,1339,1874,2466,3097,3749
};

uint32_t CCRValue_3750[CCRValue_BufferSize] =
{
		1874,2199,2514,2811,3078,3309,3496,3634,3719,3748,3719,3634,3496
		,3309,3078,2811,2514,2199,1874,1548,1233,937,669,438,251,113
		,28,0,28,113,251,438,669,936,1233,1548,1874
};

uint32_t CCRValue_937_5[CCRValue_BufferSize] =
{
		467,548,627,701,768,826,872,907,928,935,928,907,872,826,768,701
		,627,548,467,386,307,233,167,109,62,28,7,0,7,28,62,109,167,233
		,307,386,467
};

uint32_t CCRValue_468_75[CCRValue_BufferSize] =
{
		233,273,313,350,383,412,435,452,463,466,463,452,435,412,383,350
		,313,273,233,192,153,116,83,54,31,14,3,0,3,14,31,54,83,116,153
		,192,233
};

uint32_t CCRValue_937_5_72[CCRValue_NSize] =
{
		467,508,548,588,627,665,701,736,768,798,826,850,872,891,907,919
		,928,933,935,933,928,919,907,891,872,850,826,798,768,736,701,665
		,627,588,548,508,467,426,386,346,307,270,233,199,167,137,109,84
		,62,43,28,15,7,1,1,7,15,28,43,62,84,109,137,167,199,233,270,307
		,346,386,426,467
};

uint32_t CCRValue_468_75_72[CCRValue_NSize] =
{
		233,253,273,293,313,332,350,367,383,398,412,424,435,444,452,458
		,463,465,466,465,463,458,452,444,435,424,412,398,383,367,350,332
		,313,293,273,253,233,213,192,172,153,134,116,99,83,68,54,42,31,21
		,14,7,3,0,0,3,7,14,21,31,42,54,68,83,99,116,134,153,172,192,213,233
};



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM6) {
	    HAL_IncTick();
	  }
	//_duty = CCRValue_937_5_72[_index];
	//SetCommutationStep(step,_duty);
	threeSine(_index);
	++_index;
	if(_index == CCRValue_NSize)
	{
		_index = 0;
		/*if (step < 5)
		{
			++step;
		}
		else if (step == 5) {
				step = 0;
		}*/
	}
	__HAL_TIM_SET_AUTORELOAD( &htim4, arr4_);
	__HAL_TIM_SET_AUTORELOAD( &htim1, arr1_);
}

void SetCommutationStep(uint8_t step, uint16_t duty) {
    switch (step) {
        case 0:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, 0 );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;
        case 1:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, 0 );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
            break;
        case 3:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, 0 );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;
        case 2:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, 0 );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
            break;
        case 4:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, 0 );
        	  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
        	  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
            break;
        case 5:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, 0 );
        	  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
        	  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );

            break;
        default:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;
    }
}

void threeSine(uint16_t degree)
{
	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, CCRValue_937_5_72[degree] );
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );

	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, CCRValue_937_5_72[(degree + 24) % CCRValue_NSize] );
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );

	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, CCRValue_937_5_72[(degree + 48) % CCRValue_NSize] );
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
}
