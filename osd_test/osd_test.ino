/*
Author: Marko Hoepken


This code is just to check if your OSD board is basically working
with this code.

I tried to reduce as much as possible.

Compile and download it.

You may try PAL and NTSC (see Setup section)

The simple test case that shows a endless counter (0..1000) on the screen:

Output:

Test: xxx

*/

/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 


/* **********************************************/
/* ***************** INCLUDES *******************/


// AVR Includes
#include <FastSerial.h> // better steam

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif

// Include the MAX stuff

#include "ArduCam_Max7456.h"
#include "Spi.h"


/* *************************************************/
/* ***************** DEFINITIONS *******************/

//OSD Hardware 
//#define ArduCAM328
#define MinimOSD

// most screens to "Overscan" so offset may be required.
#define OSD_H_OFFSET 40 // adjust to your screen (0...63, 31 = center)
#define OSD_V_OFFSET 25 // adjust to your screen (0...31, 15 = center)

OSD osd; //OSD object 

/**********************************************/
/*                   SETUP()                  */
/**********************************************/
void setup() 
{
//    osd.setMode(NTSC);
    osd.setMode(PAL);
    osd.init();
    osd.set_h_offset(OSD_H_OFFSET);
    osd.set_v_offset(OSD_V_OFFSET);

} // END of setup();


/************************************************/
/*                 MAIN LOOP                    */
/************************************************/

void loop() 
{

    uint16_t count=0;

    while(1)
    {
        count++;
        osd.setPanel(5,5);  
        osd.openPanel();
        osd.printf("Test: %i   ",count); 
        osd.closePanel();         
        if(count==1000)
        {
            count=0;
        }
        delay(10);
    }
}


