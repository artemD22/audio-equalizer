#include "coefficients.h"
#include "conbits.h"
#include <p33FJ16GP304.h>

#include <libpic30.h>

#define FP 40000000L
#define BAUDRATE 9600
#define BRGVAL ((FP/BAUDRATE)/16)-1
#define RES (5.0/1023.0)
register int dsp_accA asm("A");

volatile uint8_t gain_index = 0;
fractional gain[5] = {Q15(0), Q15(0.0), Q15(1.0), Q15(0.0), Q15(0.0)};
        
void filter_init()
{
    FIRStructInit(&FIR1S, numCoeffs, coeff1S, COEFFS_IN_DATA , delays1S);
    FIRDelayInit(&FIR1S);
    
    FIRStructInit(&FIR2S, numCoeffs, coeff2S, COEFFS_IN_DATA , delays2S);
FIRDelayInit(&FIR2S);

    FIRStructInit(&FIR3S, numCoeffs, coeff3S, COEFFS_IN_DATA , delays3S);
FIRDelayInit(&FIR3S);

    FIRStructInit(&FIR4S, numCoeffs, coeff4S, COEFFS_IN_DATA , delays4S);
FIRDelayInit(&FIR4S);
    FIRStructInit(&FIR5S, numCoeffs, coeff5S, COEFFS_IN_DATA , delays5S);
FIRDelayInit(&FIR5S);

}
        

void uart_init()
{
    // Unlock Registers
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    // Assign U1Rx To Pin RP5
    RPINR18bits.U1RXR = 5;
    // Assign U1CTS To Pin RP1
    RPINR18bits.U1CTSR = 1;
    // Assign U1Tx To Pin RP2
    RPOR1bits.RP2R = 3;
    // Assign U1RTS To Pin RP3
    RPOR1bits.RP3R = 4;
    
    __builtin_write_OSCCONL(OSCCON | (1<<6));
    
    U1MODEbits.STSEL = 0; // 1-Stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-Data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud disabled
    U1MODEbits.BRGH = 0; // Standard-Speed mode

    U1BRG = BRGVAL;

    IPC2bits.U1RXIP = 4; //UART2 RX interrupt priority, mid-range
    IPC16bits.U1EIP = 5; //UART2 Error Priority set higher

    IFS4bits.U1EIF = 0;
    U1STAbits.URXISEL = 0x00; // Interrupt after one RX character is received;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
    IEC4bits.U1EIE = 1;
    U1MODEbits.UARTEN = 0x01; // Enable UART
    U1STAbits.UTXEN = 0x01; // Enable UART TX    
}


void adc_init(void)
{
    TRISBbits.TRISB4 = 0;
    AD1CON1bits.FORM = 0; // Data Output Format: INteger
    AD1CON1bits.SSRC = 7; // Internal Counter (SAMC) ends sampling and starts conversion
    AD1CON1bits.ASAM = 1; // ADC Sample Control: Sampling begins immediately after// conversion
    AD1CON1bits.AD12B = 0; // 10-bit ADC operation
    AD1CON1bits.SIMSAM = 0; // Sequential sampling of channels

    AD1CON2bits.CHPS = 0; // Converts channels CH0

    AD1CON3bits.ADRC = 0; // ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC = 0; // Auto Sample Time = 0 * TAD
    AD1CON3bits.ADCS = 2; // ADC Conversion Clock TAD = TCY * (ADCS + 1) = (1/40M) * 3 =
     // 75 ns (13.3 MHz)
     // ADC Conversion Time for 10-bit Tconv = 12 * TAD = 900 ns (1.1 MHz)
    //AD1CON1bits.ADDMABM = 1; // DMA buffers are built in conversion order mode
    AD1CON2bits.SMPI = 0; // SMPI must be 0

    TRISAbits.TRISA0 = 1;
    //AD1CHS0/AD1CHS123: Analog-to-Digital Input Select Register
    AD1CHS0bits.CH0SA = 0; // MUXA +ve input selection (AIN0) for CH0
    AD1CHS0bits.CH0NA = 0; // MUXA -ve input selection (VREF-) for CH0
    /*
    AD1CHS123bits.CH123SA = 0; // MUXA +ve input selection (AIN0) for CH1
    AD1CHS123bits.CH123NA = 0; // MUXA -ve input selection (VREF-) for CH1
    */
    //AD1PCFGH/AD1PCFGL: Port Configuration Register
    AD1PCFGL = 0xFFFF;
    AD1PCFGLbits.PCFG0 = 0; // AN0 as Analog Input
    IFS0bits.AD1IF = 0; // Clear the Analog-to-Digital interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable Analog-to-Digital interrupt
    AD1CON1bits.ADON = 1; // Turn on the ADC
}


void i2c_init()
{
    I2C1CONbits.I2CEN = 1; //enable i2c
    I2C1CONbits.IPMIEN = 0; //disable  IPMI Support mode
    I2C1BRG = 42;
}

void i2c_start()
{
    I2C1CONbits.SEN = 1;    //start transmittion
    while(!I2C1STATbits.S);
    
    
    I2C1TRN = 0x70;//address and write bit
    while(I2C1STATbits.TRSTAT);  

}


void i2c_send(uint8_t data)
{
    while(I2C1STATbits.TRSTAT);
    I2C1TRN = data;//address and write bit 
}

void i2c_stop()
{
    while(I2C1STATbits.TRSTAT);
    I2C1CONbits.SEN = 0;
    I2C1CONbits.RSEN = 0;
    I2C1CONbits.PEN = 0;
    I2C1CONbits.RCEN = 0;
    I2C1CONbits.ACKEN = 0;
    I2C1CONbits.PEN = 1;    //start transmittion
    
    while(I2C1CONbits.PEN); //wait for star completion
}

void uart_send(uint8_t *c)
{
    U1TXREG = *c;
    while(U1STAbits.TRMT == 0);
}

void uart_send_array(uint8_t *c, uint16_t len)
{
    uint8_t i;
    for(i = 0; i < len; i++)
    {
        uart_send(&c[i]);
    }
}

void uart_send_string(uint8_t *c)
{
    uint8_t i = 0;
    while(c[i] != '\0')
    {
        uart_send(&c[i]);
        i++;
    }
}


void FIRStage1()
{   
    input_value1S[0] = (uint16_t)ADC1BUF0;
    input_value2S[0] = input_value1S[0];
    input_value3S[0] = input_value1S[0];
    input_value4S[0] = input_value1S[0];
    input_value5S[0] = input_value1S[0];

    
    dsp_accA = __builtin_mpy(input_value1S[0], gain[0], NULL, NULL, 0, NULL, NULL, 0);
    input_value1S[0] = __builtin_sac(dsp_accA, 0);
    
    FIR(1, output_value1S, input_value1S, &FIR1S);
}

void FIRStage2()
{
    dsp_accA = __builtin_mpy(input_value2S[0], gain[1], NULL, NULL, 0, NULL, NULL, 0);
    input_value2S[0] = __builtin_sac(dsp_accA, 0);
    FIR(1, output_value2S, input_value2S, &FIR2S);
}

void FIRStage3()
{
    dsp_accA = __builtin_mpy(input_value3S[0], gain[2], NULL, NULL, 0, NULL, NULL, 0);
    input_value3S[0] = __builtin_sac(dsp_accA, 0);
    FIR(1, output_value3S, input_value3S, &FIR3S);
}


void FIRStage4()
{
    dsp_accA = __builtin_mpy(input_value4S[0], gain[3], NULL, NULL, 0, NULL, NULL, 0);
    input_value4S[0] = __builtin_sac(dsp_accA, 0);
    FIR(1, output_value4S, input_value4S, &FIR4S);
}

void FIRStage5()
{
    dsp_accA = __builtin_mpy(input_value5S[0], gain[4], NULL, NULL, 0, NULL, NULL, 0);
    input_value5S[0] = __builtin_sac(dsp_accA, 0);
    FIR(1, output_value5S, input_value5S, &FIR5S);
}




int main(int argc, char** argv) {

    
    PLLFBD=18; // M=20
    CLKDIVbits.PLLPOST=0; // N2=2
    CLKDIVbits.PLLPRE=0; // N1=2
    // Initiate Clock Switch to Primary Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC!= 0b011);
    // Wait for PLL to lock
    while (OSCCONbits.LOCK!= 1);
    
    TRISBbits.TRISB4 = 0;
    
    uart_init();
    adc_init();
    i2c_init();
    filter_init();
            
    i2c_start();
    i2c_send(0xF0);
    i2c_send(0xFC);
    i2c_stop();

    signed short filtered_value;
    uint16_t positive_filtered_value;
    uint8_t first_byte;
    uint8_t second_byte;
    while (1) 
    {
        FIRStage1();
        FIRStage2();
        FIRStage3();
        FIRStage4();
        FIRStage5();

        filtered_value = output_value1S[0]+output_value2S[0]+output_value3S[0]+output_value4S[0]+output_value5S[0];

        filtered_value += 511;

        positive_filtered_value = (uint16_t)filtered_value;
        if(positive_filtered_value > 1023)
            positive_filtered_value = 1023;
         
        first_byte = (uint8_t)(positive_filtered_value>>6);
        second_byte = (uint8_t)(positive_filtered_value<<2);
      
        
        i2c_start();
        i2c_send(first_byte);
        i2c_send(second_byte);
        i2c_stop();
    }

    return (EXIT_SUCCESS);
}

void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void) 
{    
    if (U1STAbits.PERR || U1STAbits.FERR || U1STAbits.OERR) {
        U1STAbits.PERR = U1STAbits.FERR = U1STAbits.OERR = 0;
    }
    

    if(U1STAbits.URXDA = 1)
    {
        if(gain_index != 5)
        {
            if(U1RXREG >= 48 && U1RXREG <= 57)
            {
                gain[gain_index] = Float2Fract((U1RXREG-48)*0.1);
            }
            else
                gain[gain_index] = Q15(1.0);
        }
    }
    
    gain_index = (gain_index+1)%6;
    IFS0bits.U1RXIF = 0;
    
    
}
