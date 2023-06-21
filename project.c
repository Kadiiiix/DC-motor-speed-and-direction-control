
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#include "common/include/adc.h"
#include "common/include/clk.h"
#include "common/include/flash.h"
#include "common/include/gpio.h"
#include "common/include/pie.h"
#include "common/include/pll.h"
#include "common/include/timer.h"
#include "common/include/wdog.h"
#include "common/include/pwm.h"

//
// Function Prototypes
//
__interrupt void cpu_timer0_isr(void);
void InitEPwm2(void);
void Gpio_setup1(void);
void Gpio_setup2(void);
void loop_delay(void);
void toggle(void);

//
//Handlers
//
ADC_Handle myAdc;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
TIMER_Handle myTimer;
CPU_Handle myCpu;
PLL_Handle myPll;
WDOG_Handle myWDog;
PWM_Handle myPwm; //ADC PWM
PWM_Handle myPwm2; //Motor speed PWM

//
//Variables
//
uint16_t ConversionCount; //used to reset the Voltage readings
uint16_t Voltage1[10]; //used to store voltage readings
uint16_t avV1;  //average voltage value
uint16_t k;  //loop counter
uint16_t j;  //loop counter
uint16_t interruptCount = 0; //used to count the number of interrupts
uint16_t button;  //button reading
uint16_t b=0;  //semaphore variable
uint16_t a=1;  //semaphore variable

void main(void)
{

    //
    //Peripherals
    myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    myTimer = TIMER_init((void *)TIMER0_BASE_ADDR, sizeof(TIMER_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
    myPwm = PWM_init((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
    myPwm2 = PWM_init((void *)PWM_ePWM2_BASE_ADDR, sizeof(PWM_Obj));

    WDOG_disable(myWDog);                      // Disable the watchdog timer
    CLK_enableAdcClock(myClk);                 // Enable the clock for the ADC module
    (*Device_cal)();                           // Execute the device-specific calibration routine

    CLK_setOscSrc(myClk, CLK_OscSrc_Internal); // Set the internal oscillator as the clock source
    PLL_setup(myPll, PLL_Multiplier_10, PLL_DivideSelect_ClkIn_by_2); // Configure the PLL with a multiplier of 10 and divide select of ClkIn/2
    PIE_disable(myPie);                        // Disable the PIE (Peripheral Interrupt Expansion) module
    PIE_disableAllInts(myPie);                 // Disable all interrupts in the PIE module
    CPU_disableGlobalInts(myCpu);              // Disable global interrupts
    CPU_clearIntFlags(myCpu);                   // Clear any pending interrupt flags


#ifdef _FLASH
    // Copy the contents of the memory block from RamfuncsLoadStart to RamfuncsRunStart
    // The size of the memory block is determined by RamfuncsLoadSize
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

        //
        // Make GPIO0 an input for button
        //
        GPIO_setPullUp(myGpio, GPIO_Number_0, GPIO_PullUp_Disable);      // Disable pull-up resistor for GPIO pin 0
        GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose); // Set GPIO pin 0 to General Purpose mode
        GPIO_setDirection(myGpio, GPIO_Number_0, GPIO_Direction_Input);  // Set GPIO pin 0 as an input


        PIE_setDebugIntVectorTable(myPie);    // Set the Debug Interrupt Vector Table for the PIE module
        PIE_enable(myPie);                    // Enable the PIE module
        PIE_registerPieIntHandler(myPie, PIE_GroupNumber_1, PIE_SubGroupNumber_7,
                                  (intVec_t)&cpu_timer0_isr);   // Register the PIE interrupt handler for Group 1, Subgroup 7, using the cpu_timer0_isr function



    //setup for ADC input
    ADC_enableBandGap(myAdc);                                 // Enable Band Gap reference for ADC
    ADC_enableRefBuffers(myAdc);                              // Enable reference buffers for ADC
    ADC_powerUp(myAdc);                                       // Power up the ADC module
    ADC_enable(myAdc);                                        // Enable the ADC module
    ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);           // Set the voltage reference source to internal

    PIE_enableAdcInt(myPie, ADC_IntNumber_1);                  // Enable ADC interrupt for ADC interrupt number 1
    CPU_enableInt(myCpu, CPU_IntNumber_10);                    // Enable CPU interrupt for CPU interrupt number 10
    CPU_enableGlobalInts(myCpu);                               // Enable global interrupts on the CPU
    CPU_enableDebugInt(myCpu);                                 // Enable debug interrupts on the CPU

    ADC_setIntPulseGenMode(myAdc, ADC_IntPulseGenMode_Prior);  // Set ADC interrupt pulse generation mode to priority
    ADC_enableInt(myAdc, ADC_IntNumber_1);                     // Enable ADC interrupt for ADC interrupt number 1
    ADC_setIntMode(myAdc, ADC_IntNumber_1, ADC_IntMode_ClearFlag); // Set ADC interrupt mode to clear the interrupt flag
    ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC2);    // Set ADC interrupt source to End of Conversion 2

    ADC_setSocChanNumber(myAdc, ADC_SocNumber_1, ADC_SocChanNumber_B4);   // Set ADC SOC channel number for SOC number 1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_1, ADC_SocTrigSrc_EPWM1_ADCSOCA);  // Set ADC SOC trigger source for SOC number 1
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_14_cycles);  // Set ADC SOC sample window for SOC number 1

    CLK_enablePwmClock(myClk, PWM_Number_1);                   // Enable PWM clock for PWM module 1


      //ADC PWM signal setup
      PWM_enableSocAPulse(myPwm);                          // Enable Start of Conversion (SOC) pulse on A-compare event
      PWM_setSocAPulseSrc(myPwm, PWM_SocPulseSrc_CounterEqualCmpAIncr);  // Set SOC pulse source to Counter=Compare A, increment
      PWM_setSocAPeriod(myPwm, PWM_SocPeriod_FirstEvent);   // Set SOC period to first event
      PWM_setCmpA(myPwm, 0x0080);                      // Set compare A value
      PWM_setPeriod(myPwm, 0xFFFF);                    // Set period for ePWM1
      PWM_setCounterMode(myPwm, PWM_CounterMode_Up);   // count up and start


      InitEPwm2();                       // Initialize ePWM2 module
      CLK_enableTbClockSync(myClk);       // Enable time base (TB) clock synchronization


      CPU_enableInt(myCpu, CPU_IntNumber_2);               // Enable CPU interrupt for CPU interrupt number 2
      PIE_enablePwmTzInt(myPie, PWM_Number_2);             // Enable PWM Trip Zone interrupt for PWM module 2

    //Global timer
      TIMER_stop(myTimer); // Stop the timer

      TIMER_setPeriod(myTimer, 50 * 500000); // Set the timer period to 50 * 500000

      TIMER_setPreScaler(myTimer, 0); // Set the timer prescaler to 0

      TIMER_reload(myTimer); // Reload the timer

      TIMER_setEmulationMode(myTimer, TIMER_EmulationMode_StopAfterNextDecrement); // Set the timer emulation mode to stop after the next decrement

      TIMER_enableInt(myTimer); // Enable interrupts for the timer

      TIMER_start(myTimer); // Start the timer


    ConversionCount = 0;  //initialize variable

    CPU_enableInt(myCpu, CPU_IntNumber_1); // Enable Interrupt Number 1 on the CPU



    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PIE_enableTimer0Int(myPie);


    //
    // Enable global Interrupts and higher priority real-time debug events
    //
    CPU_enableGlobalInts(myCpu);
    CPU_enableDebugInt(myCpu);


    for(;;)
    {
        // Read the status of GPIO pin GPIO0 into the 'button' variable
        button = GpioDataRegs.GPADAT.bit.GPIO0;

        // Check if button is pressed (GPIO0 is high)
        if (button == 1) {
            b = 1;
        }

        // Check the conditions for executing specific code blocks
        if (a == 1 && b == 1) {
            // Perform setup 1
            Gpio_setup1();
            b = 0;
            a = 0;
        } else if (b == 1 && a == 0) {
            // Perform setup 2
            Gpio_setup2();
            b = 0;
            a = 1;
        }

        // Read the ADC result into the 'Voltage1' array at index 'ConversionCount'
        Voltage1[ConversionCount] = ADC_readResult(myAdc, ADC_ResultNumber_1);

        // Update the 'ConversionCount' for the next ADC conversion
        if (ConversionCount == 9) {
            ConversionCount = 0;
        } else {
            ConversionCount++;
        }

        // Calculate the average value of 'Voltage1'
        uint16_t sum1 = 0;
        for (k = 0; k < 10; k++) {
            sum1 = sum1 + Voltage1[k];
        }
        avV1 = sum1 / 10;

        // Check the condition based on the calculated average value
        if (avV1 < 1500) {
            // Set the Compare A value of PWM2 to 300
            PWM_setCmpA(myPwm2, 300);
            PIE_clearInt(myPie, PIE_GroupNumber_2);
        } else {
            // Set the Compare A value of PWM2 to 500
            PWM_setCmpA(myPwm2, 500);
            PIE_clearInt(myPie, PIE_GroupNumber_2);
        }

        // Clear ADC interrupt flag and PIE interrupt flag
        ADC_clearIntFlag(myAdc, ADC_IntNumber_1);
        PIE_clearInt(myPie, PIE_GroupNumber_10);

    }
}

void InitEPwm2()
  {
    // Setup TBCLK
    PWM_setPeriod(myPwm2, 1000);   // Set timer period 801 TBCLKs
    PWM_setPhase(myPwm2, 0x0000);               // Phase is 0
    PWM_setCount(myPwm2, 0x0000);               // Clear counter


    // Setup counter mode
    PWM_setCounterMode(myPwm2, PWM_CounterMode_UpDown); // Count up
    PWM_disableCounterLoad(myPwm2);             // Disable phase loading

    // Clock ratio to SYSCLKOUT
    PWM_setHighSpeedClkDiv(myPwm2, PWM_HspClkDiv_by_1);
    PWM_setClkDiv(myPwm2, PWM_ClkDiv_by_1);

    // Setup shadowing
    PWM_setShadowMode_CmpA(myPwm2, PWM_ShadowMode_Shadow);
    PWM_setShadowMode_CmpB(myPwm2, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpA(myPwm2, PWM_LoadMode_Zero);
    PWM_setLoadMode_CmpB(myPwm2, PWM_LoadMode_Zero);

    // Set compare values
    PWM_setCmpA(myPwm2, 1000);
    PWM_setCmpB(myPwm2, 1000);

    // Set PWM2A on event A, up count
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm2, PWM_ActionQual_Set);

    // Clear PWM2A on event B, down count
    PWM_setActionQual_CntDown_CmpB_PwmA(myPwm2, PWM_ActionQual_Clear);

    // Clear PWM2B on zero
    PWM_setActionQual_Zero_PwmB(myPwm2, PWM_ActionQual_Set);

    // Set PWM2B on period
    PWM_setActionQual_Period_PwmB(myPwm2, PWM_ActionQual_Clear);

    // Interrupt where we will change the Compare Values

    // Select INT on Zero event
    PWM_setIntMode(myPwm2, PWM_IntMode_CounterEqualZero);

    PWM_enableInt(myPwm2);                  // Enable INT

    // Generate INT on 3rd event
    PWM_setIntPeriod(myPwm2, PWM_IntPeriod_ThirdEvent);

  }



__interrupt void
cpu_timer0_isr(void)
{
    interruptCount++;


    PWM_clearIntFlag(myPwm2);
    PIE_clearInt(myPie, PIE_GroupNumber_3);
    PIE_clearInt(myPie, PIE_GroupNumber_2);
    PIE_clearInt(myPie, PIE_GroupNumber_1);
    return;

}
void loop_delay(void)
{

    for(j=0;j<10000; j++)
    {}
} 
//
//first motor direction
//
void Gpio_setup1(void)
{
    EALLOW;
    GPIO_setPullUp(myGpio, GPIO_Number_3, GPIO_PullUp_Disable);      // Disable pull-up on GPIO 3
    GPIO_setMode(myGpio, GPIO_Number_3, GPIO_2_Mode_EPWM2A);         // Set GPIO 3 mode as EPWM2A
    loop_delay();                                                   // Delay for stabilization
    GPIO_setMode(myGpio, GPIO_Number_2, GPIO_0_Mode_GeneralPurpose); // Set GPIO 2 mode as general purpose
    GPIO_setDirection(myGpio, GPIO_Number_2, GPIO_Direction_Output); // Set GPIO 2 as an output
    GPIO_setLow(myGpio, GPIO_Number_2);                              // Set GPIO 2 to low
    EDIS;
}
//
//second motor direction
//
void Gpio_setup2(void)
{
    EALLOW;
    GPIO_setPullUp(myGpio, GPIO_Number_2, GPIO_PullUp_Enable);       // Enable pull-up on GPIO 2
    GPIO_setMode(myGpio, GPIO_Number_2, GPIO_2_Mode_EPWM2A);         // Set GPIO 2 mode as EPWM2A
    loop_delay();                                                   // Delay for stabilization
    GPIO_setMode(myGpio, GPIO_Number_3, GPIO_0_Mode_GeneralPurpose); // Set GPIO 3 mode as general purpose
    GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Output); // Set GPIO 3 as an output
    GPIO_setLow(myGpio, GPIO_Number_3);                              // Set GPIO 3 to low
    EDIS;
}



