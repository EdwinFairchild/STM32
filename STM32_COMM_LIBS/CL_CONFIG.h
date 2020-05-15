#ifndef CL_CONFIG_H_
#define CL_CONFIG_H_


/********************|        MICROCONTROLER  SELECTION      |********************
 *
 * Every C and H file depends on the proper microcontroller slection.
 * Uncomment to select the mcu in use.
 */

//------ Select mcu
#define CL_USING_G4
//#define CL_USING_F1
//#define CL_USING_L0
//#define CL_USING_G0



/*********************| CL_systemClockUpdate.h SETTINGS |************************
 *
 * if CL_systemClockUpdate_USE_LL is not defined the Code will use register access
 * 3 different Systm clock speed are currently supported SLOW MEDIUM and MAX
 * all of which use the HIGH SPEED EXTERNAL and PLL . 
 * The function and prescaler values can be adjusted to suite application needs
 */


//------ Select whether to use LL or not
#define CL_SYSTEMCLOCKUPDATE_USE_LL







/*********************|      CL_delay.h SETTINGS        |************************
 *
 * Select the timer to use by uncommenting
 * currently only supports General Purpose timers
 * Option to include micro second delay
 */

//------ Select which timer to use by uncommenting it
//#define CL_DELAY_USE_TIM2
//#define CL_DELAY_USE_TIM3
#define CL_DELAY_USE_TIM4

//------ Select uses LL to init timer
//#define CL_DELAY_USE_LL

//------ Select to add micro second support
//#define CL_DELAY_US_ENABLE




#endif
