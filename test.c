#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx.h"
#include "lpc17xx_clkpwr.h"

/*
 *     	ADC: medimos 6 potenciometros y un sensor de distancia
        DAC: señal de audio variable en frecuencia
        DMA: transferencia de datos M2P a DAC de la señal de audio
        Timers: interrupción para largar conversión ADC
        PWM: servomotores (5)
        Hw de comunicación: Ethernet
 */

/*
 * 1. ADC: tendremos que configurar AD0 a AD5 como entrada analogica (Quedara inhabilitado el uso del PIN AOUT - AD3)
 * 			    - tendremos que utilizar un rate que permita realizar la conversion (200 kHZ no funciona)
 * 			    - mediante el timer extraeremos la muestra
 */

/*
    EINT0 - controlamos modos
   Modo 0 controlado manualmente
   Modo 1 mediante EINT1 ejecuta el movimiento y por DMA manda al DAC una señal de audio por 10 seg

*/

/*
    FALTA MODO 1 y probar MODO 0

*/

void configPin(void);
void configPWM(void);
void configADC(void);
void servo_write(uint8_t servo_number, float value);
void configEINT0(void);

// Variables globales para ver las conversiones de ADC
uint32_t AD0Value = 0;
uint32_t AD1Value = 0;
uint32_t AD2Value = 0;
uint32_t AD4Value = 0;
uint32_t AD5Value = 0;
uint32_t AD6Value = 0;

int main(void)
{
//    SystemInit();
    configPin();
    configPWM();
    configADC();

    while (1)
    {

        // LPC_PWM1->LER = (1<<1) | (1<<0) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6);
    }
    return 0;
}

void configPin(void)
{
    // Configuracion (P2.0 - PWM1.1 - P2.5 - PWM1.6)
    PINSEL_CFG_Type PWM_pins_config;
    PWM_pins_config.Portnum = 2;
    PWM_pins_config.Pinmode = PINSEL_PINMODE_TRISTATE;
    PWM_pins_config.Funcnum = 1;
    PWM_pins_config.OpenDrain = PINSEL_PINMODE_NORMAL;
    for (uint8_t i = 0; i < 6; i++)
    {
        PWM_pins_config.Pinnum = i;
        PINSEL_ConfigPin(&PWM_pins_config);
    }
}

void configPWM(void)
{
    // Configuracion
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_1);
    LPC_PWM1->PCR = 0x0; // Single edge PWM para 6 CH
    LPC_PWM1->PR = 99;

    LPC_PWM1->MR0 = 20000; //

    // LPC_PWM1->MR1 = 1000; //1ms - default pulse duration - servo at 0 degrees
    LPC_PWM1->MR1 = 1300; // 1ms - default pulse duration - servo at 0 degrees
    LPC_PWM1->MR2 = 1500; // 1ms - default pulse duration - servo at 0 degrees
    LPC_PWM1->MR3 = 1400; // 1ms - default pulse duration - servo at 0 degrees
    LPC_PWM1->MR4 = 1500; // 1ms - default pulse duration - servo at 0 degrees
    LPC_PWM1->MR5 = 1500; // 1ms - default pulse duration - servo at 0 degrees
    LPC_PWM1->MR6 = 1500;
    LPC_PWM1->MCR = (1 << 1);                                                                   // Reset PWM TC on PWM1MR0 match
    LPC_PWM1->LER = (1 << 1) | (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6); // update values in MR0 and MR1
    LPC_PWM1->PCR = (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14);       // enable PWM outputs
    LPC_PWM1->TCR = 3;                                                                          // Reset PWM TC & PR
    LPC_PWM1->TCR &= ~(1 << 1);                                                                 // libera la cuenta
    LPC_PWM1->TCR |= (1 << 3);                                                                  // enable counters and PWM Mode
    // LPC_PWM1 -> MR1 = 2000;
}

void configADC(void)
{
    // CONNFIGURACION DE PINES P0.23, 24, 25, 26, P1.30 y P1.31 como pines ANALOGICOS
    /*
     * P0.23 -> AD0
     * P0.24 -> AD1
     * P0.25 -> AD2
     * P1.30 -> AD4
     * P1.31 -> AD5
     * P0.2  -> AD6
     */
    PINSEL_CFG_Type potenciometros;

    // configuring port 0 pins, 23 24 25 y 2 - AD0, AD1, AD2 y AD6
    potenciometros.Portnum = 0;
    potenciometros.Pinnum = 23;
    potenciometros.Funcnum = 1;
    potenciometros.Pinmode = PINSEL_PINMODE_TRISTATE;
    potenciometros.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&potenciometros);
    potenciometros.Pinnum = 24;
    PINSEL_ConfigPin(&potenciometros);
    potenciometros.Pinnum = 25;
    PINSEL_ConfigPin(&potenciometros);
    potenciometros.Pinnum = 26;
    PINSEL_ConfigPin(&potenciometros);
    potenciometros.Funcnum = 2;
    potenciometros.Pinnum = 3;
    PINSEL_ConfigPin(&potenciometros);

    // configuring port 1 pins, 30 y 31 - AD4 y AD5
    potenciometros.Portnum = 1;
    potenciometros.Pinnum = 30;
    potenciometros.Funcnum = 3;
    potenciometros.Pinmode = PINSEL_PINMODE_TRISTATE;
    potenciometros.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&potenciometros);
    potenciometros.Pinnum = 31;
    PINSEL_ConfigPin(&potenciometros);

    // ---------------- CONFIGURACION DE ADC  ---------------------
    //      Channels a utilizar son los 0, 1, 2, 4, 5 y 6

    // Configuracion del CLOCK del ADC
    LPC_SC->PCLKSEL0 |= (3 << 24); // PCLK_ADC = CCLK/8

    // fs = 120 Hz , Ts = 8.33 mseg -> 6 Channels cada 50 ms, sampling rate -> 1/20 (20 muestras por segundo)
    ADC_Init(LPC_ADC, 120);

    // ADC interrupt configuration
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN2, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN4, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN5, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN6, ENABLE);

    // Enable ADC channel number
    ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
    ADC_ChannelCmd(LPC_ADC, 1, ENABLE);
    ADC_ChannelCmd(LPC_ADC, 2, ENABLE);
    ADC_ChannelCmd(LPC_ADC, 4, ENABLE);
    ADC_ChannelCmd(LPC_ADC, 5, ENABLE);
    ADC_ChannelCmd(LPC_ADC, 6, ENABLE);

    // ADC Burst mode setting
    ADC_BurstCmd(LPC_ADC, 1);

    // Enable interupts for ADC
    NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(void)
{
    float value_volt;
    // Verifico estado de DONE de cada canal
    if (ADC_ChannelGetStatus(LPC_ADC, 0, 1))
    {
        AD0Value = ADC_ChannelGetData(LPC_ADC, 0);
        value_volt = (AD0Value / 4096) * 3.3;
        //servo_write(1, value_volt);
        //uint32_t constante = 303;
        //uint32_t convertido = (uint32_t)(value_volt * constante + 1000);
        //LPC_PWM1->MR1 = 0;
        LPC_PWM1 -> MR1 = (uint32_t)((AD0Value/1.7)+700);
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 1, 1))
    {
        AD1Value = ADC_ChannelGetData(LPC_ADC, 1);
        value_volt = (AD1Value / 4096) * 3.3;
        LPC_PWM1 -> MR2 = (uint32_t)((AD1Value/1.9)+800);
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 2, 1))
    {
        AD2Value = ADC_ChannelGetData(LPC_ADC, 2);
        value_volt = (AD2Value / 4096) * 3.3;
        LPC_PWM1 -> MR3 = (uint32_t)((AD2Value/1.6)+900);
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 4, 1))
    {
        AD4Value = ADC_ChannelGetData(LPC_ADC, 4);
        value_volt = (AD4Value / 4096) * 3.3;
        LPC_PWM1 -> MR4 = 1600;
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 5, 1))
    {
        AD5Value = ADC_ChannelGetData(LPC_ADC, 5);
        value_volt = (AD5Value / 4096) * 3.3;
        LPC_PWM1 -> MR5 = (uint32_t)((AD5Value/2.4)+740);
    }
    else if (ADC_ChannelGetStatus(LPC_ADC, 6, 1))
    {
        AD6Value = ADC_ChannelGetData(LPC_ADC, 6);
        value_volt = (AD6Value / 4096) * 3.3;
        LPC_PWM1 -> MR6 = (uint32_t)((AD6Value/2.04)+1800);
    }

    /*if (LPC_ADC->ADSTAT & (1 << 0))
    {
        uint16_t adcValue = ((LPC_ADC->ADDR0)>>4) & 0xFFF;
        volt = (ADC0Value/4096)*3.3;
        servo_write(0, volt);
    }
    else if (LPC_ADC->ADSTAT & (1 << 1))
    {
        uint16_t adcValue = ((LPC_ADC->ADDR1)>>4) & 0xFFF;
        volt = (ADC0Value/4096)*3.3;
        servo_write(0, volt);
    }
    else if (LPC_ADC->ADSTAT & (1 << 2))
    {
        uint16_t adcValue = ((LPC_ADC->ADDR2)>>4) & 0xFFF;
        volt = (ADC0Value/4096)*3.3;
        servo_write(0, volt);
    }
    ADC0Value = ((LPC_ADC->ADDR0) >> 4) & 0xFFF; // Variable auxiliar para observar el valor del registro de captura
    */

    LPC_PWM1->LER = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);
    return;
}

void servo_write(uint8_t servo_number, float value)
{
    // Conversion volt a PWM
    float constante = 303.03;
    uint32_t convertido = (uint32_t)(value * constante + 1000);
    switch (servo_number)
    {
    case 1:
        LPC_PWM1->MR1 = convertido;
        break;
    case 2:
        LPC_PWM1->MR2 = convertido;
        break;
    case 3:
        LPC_PWM1->MR3 = convertido;
        break;
    case 4:
        LPC_PWM1->MR4 = convertido;
        break;
    case 5:
        LPC_PWM1->MR5 = convertido;
        break;
    case 6:
        LPC_PWM1->MR6 = convertido;
        break;
    default:
        break;
    }
    LPC_PWM1->LER = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);

    return;
}
