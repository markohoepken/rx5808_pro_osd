/*

Copyright (c) 2015.  All rights reserved.
An Open Source Arduino based OSD project for controlling 5.8 GHz Boscam rx5808 modules.
These are used as FPV ground stations / FPV Goolges

This project has been realsized alread without OSD based on the TVlib.
Please check this project as reference:

http://code.google.com/p/rx5808-pro/

The "look" and feel is the same, but the code has been fully rewriten.

The rewrite took place since the GUI is differently build with the MAX7456.
Beside that the  rx5808-pro implementation had to work with smaller memory footprint since the
frame buffer has been held in the Adruino.

Since no Ardunino frame buffer is required for the MAX7456, the ram
can be used for "cleaner" data and channel handling inside data structures.

Thanks for the great starting point with from the arducam-osd project.

http://code.google.com/p/arducam-osd/wiki/minimosd


Program  : rx5808_pro_osd (MinimOSD [and variants] Firmware)
Version  : V1.0, Sept 1th 2015
Author(s): Marko Hoepken
Coauthor: CHECK

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>

*/

/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */



// AVR Includes
#include <avr/pgmspace.h>  // Needed for PROGMEM stuff

/* **********************************************/
/* ***************** INCLUDES *******************/

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "wiring.h"
#endif

//#define membug 
#ifdef membug
#include <MemoryFree.h>
#endif

#include "ArduCam_Max7456.h"
#include <EEPROM.h>
#include "Spi.h"



/* *************************************************/
/* ***************** DEFINITIONS *******************/

//OSD Hardware 
//#define ArduCAM328
#define MinimOSD

//#define TELEMETRY_SPEED  57600  // Serial speed for key map update
#define TELEMETRY_SPEED  38400  // Serial speed for key map update

// switches
#define KEY_A 0 // RX
#define KEY_B 1 // TX
#define KEY_UP 2
#define KEY_DOWN 1
#define KEY_MID 3
#define KEY_NONE 0

#define rssiPin A1   // Depands on patch of minimOSD
#define rx5808_SEL 5 // Depands on patch of minimOSD

//#define POWER_SENSE A0 // difficult to solder
#define POWER_SENSE A2 // easier to solder
#define POWER_SCALE 15.5 // divider 1.5K 22K, tweak to match correct voltage
#define POWER_UPDATE_RATE 20 // how ofter power is updated (loops)

#define spiDataPin 11
#define slaveSelectPin 5
#define spiClockPin 13

// key debounce delay in ms
// NOTE: good values are in the range of 100-200ms
// shorter values will make it more reactive, but may lead to double trigger
#define KEY_DEBOUNCE 20 // debounce in ms

// Set you TV format (PAL = Europe = 50Hz, NTSC = INT = 60Hz)
//#define TV_FORMAT NTSC
#define TV_FORMAT PAL

#define led 13
// RSSI default raw range
#define RSSI_MIN_VAL 90
#define RSSI_MAX_VAL 300
// 75% threshold, when channel is printed in spectrum
#define RSSI_SCANNER_FOUND 35 
// 80% under max value for RSSI 
#define RSSI_SEEK_TRESHOLD 80
// scan loops for setup run
#define RSSI_SETUP_RUN 5
// reduce max value to cover > 100%
#define RSSI_SETUP_MARGE 10 // 10%

#define STATE_SEEK_FOUND 0
#define STATE_SEEK 1
#define STATE_SCAN 2
#define STATE_MANUAL 3
#define STATE_SETUP 4
#define STATE_RSSI_SETUP 5
#define STATE_MODE_SELECT 6

#define START_STATE STATE_SEEK
#define MAX_STATE STATE_MANUAL

#define CHANNEL_BAND_SIZE 8
#define CHANNEL_MIN_INDEX 0
#define CHANNEL_MAX_INDEX 39

#define CHANNEL_MAX 39
#define CHANNEL_MIN 0

#define EEPROM_ADR_STATE 0
#define EEPROM_ADR_TUNE 1
#define EEPROM_ADR_RSSI_MIN_L 2
#define EEPROM_ADR_RSSI_MIN_H 3
#define EEPROM_ADR_RSSI_MAX_L 4
#define EEPROM_ADR_RSSI_MAX_H 5
#define EEPROM_ADR_VIDEO_MODE 6
#define EEPROM_ADR_MANUAL_MODE 7
#define EEPROM_ADR_MAGIC_KEY 8
#define EEPROM_MAGIC_KEY_SIZE 17
const uint8_t MagicKey[] PROGMEM = { // key to check if EEprom matches software
  'R','X','5','8','0','8','O','S','D',
  'R','E','V','1','.','1','.','2'
};


// Screen settings (use smaller NTSC size)
#define SCEEEN_X_MAX 30
#define SCREEN_Y_MAX 13


// Menu settings
#define MENU_HIDE_TIMER 150 // 50 ~ 1 second

#define MENU_MODE_SELECTION_X 6
#define MENU_MODE_SELECTION_Y 2
#define MENU_MODE_SELECTION_HEADER 3
#define MENU_MODE_SELECTION_ENTRY 5
#define WAIT_MODE_ENTRY 5

#define MENU_SETUP_X 6
#define MENU_SETUP_Y 2
#define MENU_SETUP_HEADER 3
#define MENU_MODE_ENTRY 5
#define MENU_SETUP_ENTRY 6

#define MODE_LINEAR 0 // manual step works linar
#define MODE_BAND 1   // manual step works on bands
// band scanner gemetry
#define BAND_SCANNER_SPECTRUM_X_MIN 2
#define BAND_SCANNER_SPECTRUM_X_MAX 27
#define BAND_SCANNER_SPECTRUM_Y_MIN 12
//#define BAND_SCANNER_SPRCTRUM_Y_MAX 5
// band scanner scaling
#define BAND_SCANNER_FREQ_MIN 5645
#define BAND_SCANNER_FREQ_MAX 5945 
#define BAND_SCANNER_RSSI_MAX 100
#define BAND_SCANNER_SUB_BAR 3 // a character can have values 0..3
#define RSSI_SUB_BAR 3 // a character can have values 0..3
#define MIN_TUNE_TIME 30 // tune time for rssi


#define OSD_EXT_SYC 1
#define OSD_INT_SYC 2
#define OSD_OFF     0
// most screens to "Overscan" so offset may be required.
#define OSD_H_OFFSET 40 // adjust to your screen (0...63, 31 = center)
#define OSD_V_OFFSET 25 // adjust to your screen (0...31, 15 = center)




// Objects and Serial definitions
OSD osd; //OSD object 

// global variables
// Channels to sent to the SPI registers
const uint16_t channelTable[] PROGMEM = {
  // Channel 1 - 8
  0x2A05,    0x299B,    0x2991,    0x2987,    0x291D,    0x2913,    0x2909,    0x289F,    // Band A
  0x2903,    0x290C,    0x2916,    0x291F,    0x2989,    0x2992,    0x299C,    0x2A05,    // Band B
  0x2895,    0x288B,    0x2881,    0x2817,    0x2A0F,    0x2A19,    0x2A83,    0x2A8D,    // Band E
  0x2906,    0x2910,    0x291A,    0x2984,    0x298E,    0x2998,    0x2A02,    0x2A0C,    // Band F / Airwave
  0x281d,    0x2890,    0x2902,    0x2915,    0x2987,    0x299a,    0x2a0c,    0x2a1f     // IRC Race Band  
};

// Channels with their Mhz Values
const uint16_t channelFreqTable[] PROGMEM = {
  // Channel 1 - 8
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880,  // Band F / Airwave
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917  // Race Band
};

const uint8_t bandNames[] PROGMEM = { // faster than calculate
  'A','A','A','A','A','A','A','A',
  'B','B','B','B','B','B','B','B',
  'E','E','E','E','E','E','E','E',
  'F','F','F','F','F','F','F','F',
  'R','R','R','R','R','R','R','R'
};
const uint8_t bandNumber[] PROGMEM = { // faster than calculate
  0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,
};
// Symbol for each channel
const uint8_t channelSymbol[] PROGMEM = {
    0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7, // Band A
    0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF, // Band B
    0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7, // Band E
    0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF, // Band F
    0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7  // Band RACE    
};

// All Channels of the above List ordered by Mhz
//  dynamic arry that keeps the channel ID sorted by frequence for seqeunce scan
uint8_t channelList[CHANNEL_MAX_INDEX+1]={};
//const uint8_t channelList[] PROGMEM = {
//  19, 18, 17, 16, 7, 8, 24, 6, 9, 25, 5, 10, 26, 4, 11, 27, 3, 12, 28, 2, 13, 29, 1, 14, 30, 0, 15, 31, 20, 21, 22, 23, 33,34,35,36,37,38,39,40
//};

// gab fill lookup
uint8_t clone_bar_to_lower[BAND_SCANNER_SPECTRUM_X_MAX]={0};
uint8_t clone_bar_to_upper[BAND_SCANNER_SPECTRUM_X_MAX]={0};
uint8_t clone_bar_to_left[BAND_SCANNER_SPECTRUM_X_MAX]={0};
uint8_t clone_bar_to_right[BAND_SCANNER_SPECTRUM_X_MAX]={0};

uint8_t channel = 0;
uint8_t channelIndex = 0;
uint8_t rssi = 0;
uint8_t rssi_scaled = 0;
uint8_t hight = 0;
uint8_t state = START_STATE;
uint8_t state_last_used=START_STATE;
uint8_t last_state= START_STATE+1; // force screen draw
uint8_t writePos = 0;
uint8_t switch_count = 0;
uint8_t man_channel = 0;
uint8_t last_channel_index = 0;
uint8_t force_seek=0;
unsigned long time_of_tune = 0;        // will store last time when tuner was changed
uint8_t last_maker_pos=0;
uint8_t last_active_channel=0;
uint8_t seek_found=0;
uint8_t last_dip_channel=255;
uint8_t last_dip_band=255;
uint8_t first_tune=1;
uint8_t force_menu_redraw=0;
uint16_t rssi_min=0;
uint16_t rssi_max=0;
uint16_t rssi_setup_min=0;
uint16_t rssi_setup_max=0;
uint16_t rssi_seek_found=0;
uint16_t rssi_setup_run=0;
uint8_t menu_first_entry=0;
uint8_t video_mode=PAL;
uint8_t manual_mode=MODE_LINEAR;
uint8_t menu_hide=0; // flag to hide osd
uint16_t menu_hide_timer=MENU_HIDE_TIMER;
uint8_t menu_no_hide=0;
uint8_t osd_mode=0; // keep current osd mode for wakeup
uint8_t power_update_delay=POWER_UPDATE_RATE;
uint8_t channel_scan=0;
uint8_t rssi_seek_threshold=RSSI_SEEK_TRESHOLD;
uint8_t seek_up=0; // keep direction of seek

uint8_t debug=0;

/*
 Array to keep values for spectrum print.
 A special coding is used, since one character has two colums.
 The spectrum on screen has 27 characters with 54 colums (2x27).
 The spectrum can have maxium 6 characters hight.
 For finer vertical resulution each colum character can have
 4 different hight values 0..3
 Coding in array:
 Lower nible  : left column
 Higher nible : right colum
 Example: value 0x13
 colum left = 1
 colum right= 3
 Organisation of array: 0:0 = bottom left corner
*/
uint8_t spectrum_display[BAND_SCANNER_SPECTRUM_X_MAX][6];

// keeps rssi value for each column for fast statistic
// Note to index: We have 40 channels, but NEED 41 places to store.
// By this we can detect a bar at the right edge.
uint8_t spectrum_channel_value[CHANNEL_MAX+2]={0}; 






/**********************************************/
/*                   SETUP()                  */
/**********************************************/
void setup() 
{

//uploadFont();
//while(1);
    // fill clonebar lookup with data
    // its easier to maintain in this manner instead of setting the array static
    // done by manual select the right entries
    // for each "open" gap in spectrum
    clone_bar_to_upper[0]=1;
    clone_bar_to_right[1]=1;
    clone_bar_to_lower[3]=1;
    clone_bar_to_upper[4]=1;
    clone_bar_to_upper[5]=1;    
    clone_bar_to_left[7]=1;
    clone_bar_to_upper[9]=1;
    clone_bar_to_lower[13]=1;
    clone_bar_to_upper[16]=1;    
    clone_bar_to_right[17]=1;
    clone_bar_to_lower[20]=1;
    clone_bar_to_upper[21]=1;
    clone_bar_to_left[23]=1;
    clone_bar_to_upper[23]=1;
    clone_bar_to_left[26]=1;
    clone_bar_to_lower[26]=1;
 

    // create channelList lookup sorted by freqency
    // 1. fill array
    for(int i=0; i<=CHANNEL_MAX_INDEX; i++) 
    {
        channelList[i]=i;
    }
    // Bubble sort algorythm by value of freqency reference
    for(int i=0; i<CHANNEL_MAX_INDEX; i++) {
        for(int o=0; o<((CHANNEL_MAX_INDEX+1)-(i+1)); o++) 
        {
            if(pgm_read_word_near(channelFreqTable + channelList[o])> pgm_read_word_near(channelFreqTable + channelList[o+1])) // compare two elements to each other and swap on demand
            { 
                // swap index;
                int t = channelList[o];
                channelList[o] = channelList[o+1];
                channelList[o+1] = t;
            }
        }
    }
    
    

   // use values only of EEprom is not 255 = unsaved
   // check if eeprom is matching current software
   // the check is done by comparing magic key in eeprom
   uint8_t eeprom_invalid=0;
   for(int i=0; i<EEPROM_MAGIC_KEY_SIZE; i++)
   {
        if((uint8_t)EEPROM.read(EEPROM_ADR_MAGIC_KEY+i) != (uint8_t)pgm_read_word_near(MagicKey + i))
        {
            eeprom_invalid=1;
            break;
        }        
   }
    if(eeprom_invalid) // EEprom does not match current software, must store new defaults
    {
        EEPROM.write(EEPROM_ADR_STATE,START_STATE);
        EEPROM.write(EEPROM_ADR_TUNE,CHANNEL_MIN_INDEX);
        // save 16 bit
        EEPROM.write(EEPROM_ADR_RSSI_MIN_L,lowByte(RSSI_MIN_VAL));        
        EEPROM.write(EEPROM_ADR_RSSI_MIN_H,highByte(RSSI_MIN_VAL));    
        // save 16 bit
        EEPROM.write(EEPROM_ADR_RSSI_MAX_L,lowByte(RSSI_MAX_VAL));
        EEPROM.write(EEPROM_ADR_RSSI_MAX_H,highByte(RSSI_MAX_VAL));
        EEPROM.write(EEPROM_ADR_VIDEO_MODE,video_mode);
        EEPROM.write(EEPROM_ADR_MANUAL_MODE,manual_mode);
        // store magic key for next check
        for(int i=0; i<EEPROM_MAGIC_KEY_SIZE; i++)
        {
            EEPROM.write(EEPROM_ADR_MAGIC_KEY+i,pgm_read_word_near(MagicKey + i));
        }         
    }

    // debug reset EEPROM
    //EEPROM.write(EEPROM_ADR_MAGIC_KEY,255);    
        
    // read last setting from eeprom
    state=EEPROM.read(EEPROM_ADR_STATE);
    channelIndex=EEPROM.read(EEPROM_ADR_TUNE);
    rssi_min=((EEPROM.read(EEPROM_ADR_RSSI_MIN_H)<<8) | (EEPROM.read(EEPROM_ADR_RSSI_MIN_L)));
    rssi_max=((EEPROM.read(EEPROM_ADR_RSSI_MAX_H)<<8) | (EEPROM.read(EEPROM_ADR_RSSI_MAX_L)));

    // debug settings
    // rssi_min=5; // force bars in any case
    // rssi_max=300; //  expect no clipping (typical ~250)
    
    video_mode=EEPROM.read(EEPROM_ADR_VIDEO_MODE);
    manual_mode=EEPROM.read(EEPROM_ADR_MANUAL_MODE);      
    force_menu_redraw=1;
 
    unplugSlaves();
    //SPI Spi = SPI();
    osd.setMode(video_mode);
    osd.init();
    osd.set_h_offset(OSD_H_OFFSET);
    osd.set_v_offset(OSD_V_OFFSET);
    // set pins
    pinMode(rx5808_SEL,OUTPUT);
    digitalWrite(rx5808_SEL,HIGH);

    // SPI pins for RX control
    pinMode (slaveSelectPin, OUTPUT);
    pinMode (spiDataPin, OUTPUT);
	pinMode (spiClockPin, OUTPUT);
    
#if 0    
    while(1)
    {
        setChannelModule(1);
        delay(200);
    }
#endif 

    // simple start screen

    screen_startup();
    // show message if EEPROM need reinit
    if(eeprom_invalid)
    {
       osd_print(3,12,"INVALID EEPROM CLEARED");
    }
    delay(2000);
} // END of setup();


/************************************************/
/*                 MAIN LOOP                    */
/************************************************/

void loop() 
{

    /************************/
    /*  Menu hide handler   */
    /************************/
    if(menu_hide_timer==0 || menu_hide==1)
    {
        // turn OSD off
        osd.control(OSD_OFF);
        while(get_key() == KEY_NONE)
        {
            // wait for wakeup
            delay(100);
        }
        // wakeup!
        menu_hide_timer=MENU_HIDE_TIMER;
        menu_hide=0;        
        osd.control(osd_mode); // we are back
        while(get_key() != KEY_NONE)
        {
            // wait for key release as debounce
            // to avoid change by pressed key
        }        
        menu_hide=0;
    }
    else
    {
        if(!menu_no_hide)
        {
            menu_hide_timer--;
            //osd_print_debug(1,1,"hide",menu_hide_timer);
        }
    }

    /************************/
    /*   Mode Select Enty   */
    /************************/
    // Special handler you must press the mode some time to get in
    if ((state != STATE_SETUP) && get_key() == KEY_MID) // key pressed ?
    {      
//                 osd_print_debug(1,1,"switch_count",switch_count);(1,1,"switch_count",switch_count);
        if (switch_count > WAIT_MODE_ENTRY)
        {   
            state=STATE_MODE_SELECT;
            menu_first_entry=1;
        } 
        else 
        {
            switch_count++;         
        }      
    } 
    else // key pressed
    { // reset hold detection     
        switch_count = 0;    
    }
    /***************************************/
    /*   Draw screen if mode has changed   */
    /***************************************/
    if(force_menu_redraw || state != last_state)
    {
        force_menu_redraw=0;
        /************************/
        /*   Main screen draw   */
        /************************/            
        // changed state, clear an draw new screen       

        switch (state) 
        {    
            case STATE_SCAN: // Band Scanner
            case STATE_RSSI_SETUP: // RSSI setup
            // screen RSSI setup
                if(state==STATE_SCAN)
                {    
                    screen_band_scanner(0);             
                }
                else
                {
                    // rssi setup
                    screen_band_scanner(1);                      
                    // prepare new setup

                    rssi_min=0;
                    rssi_max=400; // set to max range
                    rssi_setup_min=400;
                    rssi_setup_max=0;   
                    rssi_setup_run=RSSI_SETUP_RUN;
                    spectrum_init();
                }   
                // trigger new scan from begin
                channel=CHANNEL_MIN;
                channelIndex = channelList[channel];  
                osd_mode=OSD_INT_SYC; // internal sync  
                menu_no_hide=1;                
            break;
            case STATE_MANUAL: // manual mode 
            case STATE_SEEK: // seek mode
          
                if (state == STATE_MANUAL)
                {
                    screen_manual(0,channelIndex);                  
                }
                else if(state == STATE_SEEK)
                {
                    screen_manual(1,channelIndex);  
                }
                force_seek=1;
                seek_up=1;
                osd_mode=OSD_EXT_SYC; // external sync   
                menu_no_hide=0;  
                //osd.set_background(MAX7556_BACKGROUND_VIDEO);                
            break;
            case STATE_SETUP:
                osd_mode=OSD_INT_SYC; // internal sync                  
                screen_setup();    
                menu_no_hide=1;
            break;
            case STATE_MODE_SELECT:
                osd_mode=OSD_INT_SYC; // internal sync                  
                screen_mode_selection(); 
                menu_no_hide=1;                
            break;    

        } // end switch
        osd.control(osd_mode);
       
        last_state=state;
    }
    /*************************************/
    /*   Processing depending of state   */
    /*************************************/

    /*****************************************/
    /*   Processing MANUAL MODE / SEEK MODE  */
    /*****************************************/
    if(state == STATE_MANUAL || state == STATE_SEEK)
    {
        if(state == STATE_MANUAL) // MANUAL MODE        
        {

            // handling of keys
            if( get_key() == KEY_UP )        // channel UP
            {        
                if (manual_mode== MODE_BAND)
                {            
                    channelIndex++;
                    if (channelIndex > CHANNEL_MAX_INDEX) 
                    {  
                        channelIndex = CHANNEL_MIN_INDEX;
                    } 
                }
                else
                { // linear mode
                    if(channel_scan < CHANNEL_MAX_INDEX)                
                    {
                        channel_scan++;
                    }
                    else
                    {
                        channel_scan=CHANNEL_MIN_INDEX;
                    }
                    // translate to right index
                    channelIndex = channelList[channel_scan];                    
                }
            }
            if( get_key() == KEY_DOWN) // channel DOWN
            {
                if (manual_mode== MODE_BAND)
                {            
                    channelIndex--;
                    if (channelIndex > CHANNEL_MAX_INDEX) // negative overflow
                    {  
                        channelIndex = CHANNEL_MAX_INDEX;
                    }    
                }
                else
                { // linear mode
                    if(channel_scan > CHANNEL_MIN_INDEX)
                    {
                        channel_scan--;
                    }
                    else
                    {
                        channel_scan=CHANNEL_MAX_INDEX;
                    }
                    // translate to right index
                    channelIndex = channelList[channel_scan];                  
                }                    
            }            
        }
    
        // print bar for spectrum
        channel=channel_from_index(channelIndex); // get 0...31 index depending of current channel            
        wait_rssi_ready();
        rssi = readRSSI();
        // add spectrum of current channel
        spectrum_add_column (3, pgm_read_word_near(channelFreqTable + channelIndex), rssi,1);
        spectrum_dump(3);    
        // RSSI bar
        draw_rssi_bar(9, 6, 17, rssi);
        screen_manual_data(channelIndex);
        // handling for seek mode after screen and RSSI has been fully processed
        if(state == STATE_SEEK) //
        { // SEEK MODE
            if(!seek_found) // search if not found
            {
                if ((!force_seek) && (rssi > rssi_seek_threshold)) // check for found channel
                {
                    seek_found=1; 
                    menu_no_hide=0;  // found, screen may be turned off
                } 
                else 
                { // seeking itself
                    force_seek=0;
                    menu_no_hide=1; // prevent hide on search
                    // next channel
                    if(seek_up)
                    {                    
                        if (channel < CHANNEL_MAX) 
                        {
                                channel++;
                        } else {
                            channel=CHANNEL_MIN;
                        }      
                    }
                    else // seek down
                    {
                        if (channel > CHANNEL_MIN) 
                        {
                                channel--;
                        } else {
                            channel=CHANNEL_MAX;
                        }                     
                    }
                    channelIndex = channelList[channel];                      
                    screen_manual_data(channelIndex); // update data on screen
                }        
            }
            else
            { // seek was successful            
                osd_print(BAND_SCANNER_SPECTRUM_X_MIN,2,"\x02  AUTO MODE LOCK");

                if (get_key() == KEY_UP) // restart seek if key pressed
                {              
                    force_seek=1;
                    seek_up=1;
                    seek_found=0; 
                    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,2,"\x02  AUTO MODE SEEK");      
                }             
                else if (get_key() == KEY_DOWN) // restart seek if key pressed
                {              
                    force_seek=1;
                    seek_up=0;
                    seek_found=0; 
                    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,2,"\x02  AUTO MODE SEEK");      
                }                 
            }
        }        
    }
    /****************************/
    /*   Processing SCAN MODE   */
    /****************************/
    else if (state == STATE_SCAN || state == STATE_RSSI_SETUP) 
    {
        // background scan with key interruption
        spectrum_init();
        uint8_t channel_index=0;   
        uint8_t channel_scan_run=0;        
        uint8_t exit=0;
        while(!exit &  (channel_scan_run <= CHANNEL_MAX_INDEX))
        //for (channel_scan=CHANNEL_MIN_INDEX; channel_scan <= CHANNEL_MAX_INDEX;channel_scan++ )
        {
           // osd_print_debug(1,1,"CH:",channel_scan);
       // stay here until key pressed again
        //while(get_key() == KEY_NONE);
        //while(get_key() == KEY_DOWN);          
            channel_index = channelList[channel_scan_run];            
            setChannelModule(channel_index);   // TUNE 
            time_of_tune=millis();   
            // wait for rssi_ready an check keys
            //wait_rssi_ready(); 
            
           // implementation that can be breaked by key
            while ((millis()-time_of_tune) < MIN_TUNE_TIME ) 
            {
                //delay(5);
                if(get_key() == KEY_MID)
                {
                    exit=1;
                    switch_count=WAIT_MODE_ENTRY+1; // faster main menu since one loop is 1 second                       
                }
                  // Hold function
                if(state != STATE_RSSI_SETUP) { // no hold on rssi setuo
                    if (get_key() == KEY_UP)
                    {   
                        // pause
                        osd_print (3,2, "HOLD BAND SCANNER");
                        while(get_key() == KEY_UP);
                        // stay here until key pressed again
                        while(get_key() == KEY_NONE);
                        osd_print (3,2, "     BAND SCANNER");
                        while(get_key() == KEY_UP);            
                    }                     
                }
            }                
            rssi = readRSSI();
            // save raw for channel marker + Filter
            if(rssi >RSSI_SCANNER_FOUND)
            {
                spectrum_channel_value[channel_scan_run]=rssi;
            }
            else
            {
                spectrum_channel_value[channel_scan_run]=0;
            }
            // add spectrum of current channel
            spectrum_add_column (6, pgm_read_word_near(channelFreqTable + channel_index), rssi,0);     
            channel_scan_run++;
        }
        spectrum_dump(6);  
        // analyse spectrum an mark potential channels
        dump_channels(BAND_SCANNER_SPECTRUM_Y_MIN-8);
        
        if(state == STATE_RSSI_SETUP) {
            if(!rssi_setup_run--)    
            {
                // setup done
                rssi_min=rssi_setup_min;
                rssi_max=rssi_setup_max+RSSI_SETUP_MARGE;
                // save 16 bit
                EEPROM.write(EEPROM_ADR_RSSI_MIN_L,(rssi_min & 0xff));        
                EEPROM.write(EEPROM_ADR_RSSI_MIN_H,(rssi_min >> 8));    
                // save 16 bit
                EEPROM.write(EEPROM_ADR_RSSI_MAX_L,(rssi_max & 0xff));
                EEPROM.write(EEPROM_ADR_RSSI_MAX_H,(rssi_max >> 8));                    
                state=state_last_used;                 
                osd_print (MENU_SETUP_X, (MENU_SETUP_Y + 5 + MENU_SETUP_ENTRY ), " Settings saved..");
                delay(1000);
                spectrum_init(); // clear spectrum
            }
            else // update screen
            { 
                osd_print(BAND_SCANNER_SPECTRUM_X_MIN,3,"\x02Run:   MIN:      MAX:    \x02");                        
                osd_print_int(BAND_SCANNER_SPECTRUM_X_MIN+5,3,rssi_setup_run);
                osd_print_int(BAND_SCANNER_SPECTRUM_X_MIN+12,3,rssi_setup_min);
                osd_print_int(BAND_SCANNER_SPECTRUM_X_MIN+22,3,rssi_setup_max);                
            }        
        }
        

    
        if(state == STATE_SCAN)        
        {
            if (rssi > RSSI_SEEK_TRESHOLD) 
            {
                // names of potential found channels                
            }            
        }       
        // next channel
        if (channel < CHANNEL_MAX) 
        {
            channel++; // increment
        } else 
        {
            channel=CHANNEL_MIN;
            if(state == STATE_RSSI_SETUP)        
            {

            }            
        }    
   
        
        // update index after channel change   
        channelIndex = channelList[channel];          
    }
    else if (state == STATE_MODE_SELECT) 
    {
        uint8_t menu_id=0; 
        set_cursor // set cursor to show active menu entry
            (
                MENU_MODE_SELECTION_X+1, 
                MENU_MODE_SELECTION_Y + MENU_MODE_SELECTION_HEADER,
                MENU_MODE_SELECTION_ENTRY,
                menu_id+1
            );   
 
        while(state == STATE_MODE_SELECT)
        {
            // prevent exit on entry with pressed buttom from previous menu
            if(menu_first_entry){
                menu_first_entry=0;
                while(get_key() != KEY_NONE); // wait for key release
            }
            if(get_key() == KEY_MID)
            {
                // Menu navigation
                if (menu_id < MENU_MODE_ENTRY-1)
                {
                    menu_id++; // next menu entry
                } 
                else 
                {
                    menu_id = 0; 
                }                 
                set_cursor // set cursor to show active menu entry
                    (
                        MENU_MODE_SELECTION_X+1, 
                        MENU_MODE_SELECTION_Y + MENU_MODE_SELECTION_HEADER,
                        MENU_MODE_SELECTION_ENTRY,
                        menu_id+1
                    );   
                while(get_key() == KEY_MID)
                {
                }
            }
            // Menu action
            if(get_key() == KEY_UP)
            {
                switch (menu_id) 
                {    
                    case 0: // EXIT
                        state=state_last_used;
                    break;
                    case 1: // AUTO SEARCH
                        state=STATE_SEEK;
                        state_last_used=state;
                        force_seek=1;
                        seek_found=0;
                        spectrum_init();                                            
                    break;
                    case 2: // BAND SCANNER
                        state=STATE_SCAN;
                        state_last_used=state;                         
                        spectrum_init();     
                    break;
                    case 3: // MANUEL MODE
                        state=STATE_MANUAL;   
                        state_last_used=state; 
                        spectrum_init();
                    break;
                    case 4: // SETUP
                        menu_first_entry=1;
                        state=STATE_SETUP;                           
                    break;                    
                } // end switch                
            }
            
        }
    }    
    else if (state == STATE_SETUP) 
    {
        uint8_t menu_id=0; 
        set_cursor // set cursor to show active menu entry
            (
                MENU_SETUP_X+1, 
                MENU_SETUP_Y + MENU_SETUP_HEADER,
                MENU_SETUP_ENTRY,
                menu_id+1
            );               
        while(state == STATE_SETUP)
        {
            // prevent exit on entry with pressed buttom from previous menu
            if(menu_first_entry){
                menu_first_entry=0;
                while(get_key() != KEY_NONE); // wait for key release
            }
            if(get_key() == KEY_MID)
            {
                // Menu navigation
                if (menu_id < MENU_SETUP_ENTRY-1)
                {
                    menu_id++; // next menu entry
                } 
                else 
                {
                    menu_id = 0; 
                }                 
                set_cursor // set cursor to show active menu entry
                    (
                        MENU_SETUP_X+1, 
                        MENU_SETUP_Y + MENU_SETUP_HEADER,
                        MENU_SETUP_ENTRY,
                        menu_id+1
                    );           
                while(get_key() == KEY_MID)
                {
                }
            }
            // Menu action
            if(get_key() == KEY_UP)
            {
                switch (menu_id) 
                {    
                    case 0: // EXIT
                        state=state_last_used;
                    break;
                    case 1: // SAVE SETTINGS
                        EEPROM.write(EEPROM_ADR_STATE,state_last_used);
                        EEPROM.write(EEPROM_ADR_TUNE,channelIndex);  
                        EEPROM.write(EEPROM_ADR_VIDEO_MODE,video_mode);                        
                        osd_print (MENU_SETUP_X, (MENU_SETUP_Y + 4 + MENU_SETUP_ENTRY ), " Settings saved..");
                        delay(1000);
                        osd_print (MENU_SETUP_X, (MENU_SETUP_Y + 4 + MENU_SETUP_ENTRY ), "                 ");
                        state=state_last_used;  // fast exit
                    break;
                    case 2: // MANUAL MODE
                        // toggle and update
                        if(manual_mode== MODE_LINEAR){
                            manual_mode=MODE_BAND;  
                            osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+5,"BAND  ");                              
                        }
                        else
                        {
                            manual_mode=MODE_LINEAR;
                            osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+5,"LINEAR");       
                        } 
                        EEPROM.write(EEPROM_ADR_MANUAL_MODE,manual_mode);
                        osd_print (MENU_SETUP_X+5, (MENU_SETUP_Y + 4 + MENU_SETUP_ENTRY ), "Saved.          ");
                        delay(1000);
                        osd_print (MENU_SETUP_X+5, (MENU_SETUP_Y + 4 + MENU_SETUP_ENTRY ), "                ");
                        
                        // wait key released
                        while(get_key() == KEY_UP);
                    break;
                    case 3: // VIDEO MODE
                        // toggle and update
                        if(video_mode== NTSC){
                            video_mode=PAL;
                            osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+6,"PAL ");                            
                        }
                        else
                        {
                            video_mode=NTSC;
                            osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+6,"NTSC");       
                        } 
                        EEPROM.write(EEPROM_ADR_VIDEO_MODE,video_mode);
                        osd_print (MENU_SETUP_X-2, (MENU_SETUP_Y + 4 + MENU_SETUP_ENTRY ), "Saved. RESTARTING...");
                        delay(2000);
                        asm volatile ("  jmp 0");  // softstart by jumping to start vector  
                        // REBOOT, WILL NEVER GET HERE

                    break;
                    case 4: // RSSI CALIBRATE
                        state=STATE_RSSI_SETUP;
                        spectrum_init();                          
                    break;
                    case 5: // Font upload
                        uploadFont(); 
                        // WILL REBOOT. WILL NEVER GET HERE
                    break;                    
                } // end switch                
            }            
        }
    }

    /*****************************/
    /*   General house keeping   */
    /*****************************/    
    if(last_channel_index != channelIndex)         // tune channel on demand
    {
        setChannelModule(channelIndex);   // TUNE 
        last_channel_index=channelIndex;
        // keep time of tune to make sure that RSSI is stable when required
        time_of_tune=millis();
    }
    // update Power display
    if(!power_update_delay--)
    {
        power_update_delay=POWER_UPDATE_RATE;
        show_power(23,2);
    } 
}


/************************************************/
/*              SUB ROUTINES                    */
/************************************************/

void spi_32_transfer(uint32_t value)
{
    uint8_t* buffer = (uint8_t*) &value; // for simple byte access
    
//    Spi.mode((1<<DORD) | (1<<SPR1) | (1<<SPR0));  // set to SPI LSB first mode  and to 1/64 speed
    Spi.mode((1<<DORD) | (1<<SPR0));  // set to SPI LSB first mode  and to 1/64 speed
    
    digitalWrite(rx5808_SEL,LOW); // select
    delayMicroseconds(1); 
    
    Spi.transfer(*(buffer + 0)); // byte 0

    Spi.transfer(*(buffer + 1)); // byte 1

    Spi.transfer(*(buffer + 2)); // byte 2

    Spi.transfer(*(buffer + 3)); // byte 3
    digitalWrite(rx5808_SEL,HIGH); // transfer done
    delayMicroseconds(1);     
    Spi.mode(0);  // set SPI mode back to MSB first (used by OSD)
   
#if 0   
  //buffer32=0xaabbccdd;
  //debug_x (10, 1, "CH", value);  
  osd_print_debug_x (1, 1, "val0", *(buffer + 0));
  osd_print_debug_x (1, 2, "val1", *(buffer + 1));
  osd_print_debug_x (1, 3, "val2", *(buffer + 2));
  osd_print_debug_x (1, 4, "val3", *(buffer + 3));
#endif
    
    
}

// driver RX module with SPI hardmacro. NOT WORKING YES
void setChannelModule__(uint8_t channel) 
{
  uint16_t channelData;
  channelData = pgm_read_word_near(channelTable + channel);
  //osd_print_debug_x (10, 1, "CH", channelData);  
  uint32_t buffer32=0;
  uint8_t address = 0;
  uint32_t data = 0;
  uint8_t write = 0;
  
  // note to SPI of rx5808 chip (RTC6715)
  // The SPI interface needs 25 bits to be written.
  // The HW SPI controller of the Atmel has 8 bit.
  // Since 8 does not fit 8 bit, we must transfer "more" bits (32 bit)
  // This is possible because the shift register of the RTC6715 will 
  // use the "last" bits when CS goes passive.
  // This means, that we transfer 7x dummy '0' + 25 bits = 32 bits.
  
  
  // data format 25 bits
  // Order: A0-3, !R/W, D0-D19
  // NOTE: LSB first!
  
  
  // This order is required:
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
  
  // LSB first
  // 1. send 7bit x dummy '0'
  // 2. 4 bit adress
  // 3. RW bit
  // 4. 20 bit data
  
  // for efficient code the follwing data structre in a uint32_t is used:
  
  // MSB                           LSB  
  // D19...D0 + RW + A3...A0 + 0000000
  //    20 bit + 1 bit + 4 bit + 7 bit = 32 bit
  // DDDDDDDDDDDDDDDDDDDDRAAAA0000000
  //                    ^
  //                    12
  //                     ^
  //                     11  ^
  //                         7 
  // SPI must set to DORD = 1 (Data Order LSB first
  
  
  // read operation ??? CHECK

 /* TEST VALUES
  uint8_t address = 0xF;
  uint32_t data = 0x12345;
  uint8_t rw = 1;    
*/
 
  address = 0x1;
  data = channelData;
  write = 1;

  buffer32=0; //  init buffer to 0
  buffer32=((data & 0xfffff) << 12 ) | ((write & 1)<<11) | ((address & 0xf) <<7);
  spi_32_transfer(buffer32);
  //delay(200);
}


void setChannelModule(uint8_t channel)
{
  uint8_t i;
  uint16_t channelData;
  
  //channelData = pgm_read_word(&channelTable[channel]);
  //channelData = channelTable[channel];
  channelData = pgm_read_word_near(channelTable + channel);

//   osd_print_debug_x (1, 1, "TUNE:", channelData);  
  SPCR = 0; // release SPI controller for bit banging

  
  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();

  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();

  // Write to register
  SERIAL_SENDBIT1();

  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (i = 16; i > 0; i--)
  {
    // Is bit high or low?
    if (channelData & 0x1)
    {
      SERIAL_SENDBIT1();
    }
    else
    {
      SERIAL_SENDBIT0();
    }

    // Shift bits along to check the next one
    channelData >>= 1;
  }

  // Remaining D16-D19
  for (i = 4; i > 0; i--)
    SERIAL_SENDBIT0();

  // Finished clocking data in
  SERIAL_ENABLE_HIGH();

  
  Spi.mode(0);  // set SPI mode back to MSB first (used by OSD)  
  
}


void SERIAL_SENDBIT1()
{
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);

  digitalWrite(spiDataPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_SENDBIT0()
{
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);

  digitalWrite(spiDataPin, LOW);
  delayMicroseconds(1);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_LOW()
{
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_HIGH()
{
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, HIGH);
  delayMicroseconds(1);
}

//  does fast scan in backgroud and sets spectrum data
// runtime ~1 second on 40 channels
void background_scan(uint8_t size)
{
    spectrum_init();
    uint8_t channel_scan=0;
    uint8_t channel_index;
    uint8_t current_rssi_max=0; // set seek values
    for (channel_scan=CHANNEL_MIN_INDEX; channel_scan <= CHANNEL_MAX_INDEX;channel_scan++ )
    {
       // osd_print_debug(1,1,"CH:",channel_scan);
        channel_index = channelList[channel_scan];            
        setChannelModule(channel_index);   // TUNE 
        time_of_tune=millis();                
        wait_rssi_ready();
        rssi = readRSSI();
        if(rssi>current_rssi_max){
            current_rssi_max=rssi;
        }
        // add spectrum of current channel
        spectrum_add_column (size, pgm_read_word_near(channelFreqTable + channel_index), rssi,0);                
    }
    // set new seek threshold
    rssi_seek_threshold=(uint16_t)current_rssi_max*RSSI_SEEK_TRESHOLD/100;
}
  

uint8_t channel_from_index(uint8_t channelIndex)
{
    uint8_t loop=0;
    uint8_t channel=0;
    for (loop=0;loop<=CHANNEL_MAX;loop++)
    {                
    //if(pgm_read_byte_near(channelList + loop) == channelIndex)
    if(channelList[loop] == channelIndex)
        {
            channel=loop;
            break;
        }
    }
    return (channel);
}    


void draw_rssi_bar(uint8_t xpos, uint8_t ypos, uint8_t scale, uint8_t rssi)
{
    uint8_t x=0;
    uint8_t x_step= BAND_SCANNER_RSSI_MAX/scale;
    uint8_t x_step_fractional= BAND_SCANNER_RSSI_MAX/scale/RSSI_SUB_BAR;
    uint8_t x_max_100=0; // keeps last y with 100%
    
    // NOT USED
    uint8_t x_fill=0; // marker to fill top of comum with 0
    // set all 100% sub bars
#if 0    
    osd_print_debug(1,2,"x_step",x_step);
    osd_print_debug(1,3,"x_step_fractional",x_step_fractional);
    //osd_print_debug(1,4,"xpos",x);    
#endif  
  
    // left corner
    osd_print_char(xpos,ypos,0x83); // left
    for(x=1; x<=scale;x++) // 1...scale
    {
        if(x*x_step < rssi)
        {
            osd_print_char(xpos+x,ypos,0x87); // 100% bar

            x_max_100=x;
        }
        else
        { // fractional rest
            if(x_fill==0)
            {
                // handle fractional values on top of bar and beyond
                uint8_t rssi_fraction= rssi-(x_max_100*x_step);
                uint8_t bar_value= rssi_fraction/ x_step_fractional;
//                osd_print_char(xpos+x,ypos,0x84+bar_value); // fractional bars
                osd_print_char(xpos+x,ypos,0x83+bar_value); // fractional bars
                x_fill=1; // fill rest with "0"
            }
            else
            {
                osd_print_char(xpos+x,ypos,0x88); // 0% bar
            }
        }
    }
    osd_print_char(xpos+scale+1,ypos,0x89); // rigth
}

void wait_rssi_ready()
{
    // CHECK FOR MINIMUM DELAY
    // check if RSSI is stable after tune by checking the time
    uint16_t tune_time = millis()-time_of_tune;
    // module need >20ms to tune.
    // 30 ms will to a 32 channel scan in 1 second.
    if(tune_time < MIN_TUNE_TIME)
    {
        // wait until tune time is full filled
        delay(MIN_TUNE_TIME-tune_time);
    }
}
      
void show_power(uint8_t x, uint8_t y)
{
    uint16_t sensorValue = analogRead(POWER_SENSE);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):    
    double voltage = sensorValue * (5 / 1023.0) * POWER_SCALE;
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%c%2.1f",0xd0,voltage); 
    osd.closePanel();
}


// read values
// to average
// to normisation
// do statistic
uint16_t readRSSI() 
{
    uint16_t rssi = 0;
    for (uint8_t i = 0; i < 10; i++) 
    {
        rssi += analogRead(rssiPin);
    }
    rssi=rssi/10; // average
    // special case for RSSI setup
    if(state==STATE_RSSI_SETUP)
    { // RSSI setup
        if(rssi < rssi_setup_min)
        {
            rssi_setup_min=rssi;
        }
        if(rssi > rssi_setup_max)
        {
            rssi_setup_max=rssi;
        }    
    }   
    
#if 0    
    osd_print_debug (1, 3, " min: ",rssi_min );  
    osd_print_debug (16, 3, " max: ",rssi_max );      
    osd_print_debug (1, 2, " RSSI r: ",rssi );  
#endif    
    rssi = constrain(rssi, rssi_min, rssi_max);    //original 90---250
    rssi=rssi-rssi_min; // set zero point (value 0...160)
    rssi = map(rssi, 0, rssi_max-rssi_min , 1, 100);   // scale from 1..100%
// TEST CODE    
    //rssi=random(0, 100);     
#if 0 
    osd_print_debug (16, 2, " RSSI: ",rssi );     
#endif    
    return (rssi);
}
      
   
void osd_print (uint8_t x, uint8_t y, const char string[30])
{
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%s",string); 
    osd.closePanel(); 
}
// special print using PROGMEM strings
void osd_print_P (uint8_t x, uint8_t y, const prog_char text[])
{
    char P_print_buffer[30];
    strcpy_P(P_print_buffer,text); // copy PROGMEM string to local buffer
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%s",P_print_buffer); 
    osd.closePanel(); 
}
void osd_print_int (uint8_t x, uint8_t y, uint16_t value)
{
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%d",value); 
    osd.closePanel(); 
}

void osd_print_char (uint8_t x, uint8_t y, const char value)
{
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%c",value); 
    osd.closePanel(); 
}
void osd_print_debug (uint8_t x, uint8_t y, const char string[30], uint16_t value)
{
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%s :%i   ",string,value); 
    osd.closePanel(); 
}
void osd_print_debug_x (uint8_t x, uint8_t y, const char string[30], uint16_t value)
{
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%s :0x%x   ",string,value); 
    osd.closePanel(); 
}


/*******************/
/*   BAND SCANNER   */
/*******************/


// special wrapper that makes 3 columns out of 1.
// reason:
//
// We have 40 channels, but 54 colums. Due to rounding and freqencies, some columns
// are never used. This looks like "gaps" in the spectrum
// Technical one channel has anyway a board sepctrum.
// Therfore instead of adding ONE colum, we do 3
// 1. center 
// 2. left 50%
// 3. right50%
//   #
//   #
//  ###
//  ###
//  LCR
// At the end, the spectrum looks "nice".
// The "real" center line has correct value.

#if 0
void spectrum_add_column_DEAD (uint8_t scale, uint16_t frequency, uint8_t rssi, uint8_t marker)
{
    // check x-position first
    // X POSTION HANDLING (range of array 0..26 = 27 positions)
    uint8_t upper=0; // marker for upper or lower sub colum in charcter
    // calculate column position of 54 columns    
    // Note: calculation done on runtime, since preprocessor seems to have issues with forumlars
    // simple interger with 10x factor and /10 at end
    #define INTEGER_GAIN 100
    uint16_t frequency_delta=(frequency-BAND_SCANNER_FREQ_MIN); // no rouding issue
    uint16_t frequency_per_char=((BAND_SCANNER_FREQ_MAX-BAND_SCANNER_FREQ_MIN)*INTEGER_GAIN)/((BAND_SCANNER_SPECTRUM_X_MAX-1)*2);
    // special rounding is required, since lowest in on left side, highest on right sight of character
    #define ROUND_CORRECTION 2 // stretches band a little
    uint8_t x_pos_54= (frequency_delta*(INTEGER_GAIN+ROUND_CORRECTION)) / frequency_per_char;
    // find right column of 27 characters
    uint8_t x=((x_pos_54)/2); // final down scale to single character

    #define SPECTURM_BANDWITH 7
    #define SPECTURM_SIDE_FACTOR 0.7
    #define SPECTURM_SIDE_FACTOR2 0.5
    
    if(frequency > BAND_SCANNER_FREQ_MIN+SPECTURM_BANDWITH) // skip first
    {
        //spectrum_add_column_single (scale, frequency-SPECTURM_BANDWITH, rssi*SPECTURM_SIDE_FACTOR,0); // center   
    }
    //spectrum_add_column_single (scale, frequency+SPECTURM_BANDWITH, rssi); // right
    spectrum_add_column_single (scale, frequency, rssi,marker); // center
    //spectrum_add_column_single (scale, frequency+SPECTURM_BANDWITH, rssi); // right
   
    if(frequency < BAND_SCANNER_FREQ_MAX-SPECTURM_BANDWITH) // next right
    {
        spectrum_add_column_single (scale, frequency+SPECTURM_BANDWITH, rssi*SPECTURM_SIDE_FACTOR,0); // center
    }           
    if(frequency < BAND_SCANNER_FREQ_MAX-2*SPECTURM_BANDWITH) // over next right
    {
        spectrum_add_column_single (scale, frequency+2*SPECTURM_BANDWITH, rssi*SPECTURM_SIDE_FACTOR2,0); // center
    }     
    
}
#endif
// add one spectrum line in spectrum buffer
// this function does all the colum calcuation with rounding
//void spectrum_add_column_single (uint8_t scale, uint16_t frequency, uint8_t rssi, uint8_t marker)
void spectrum_add_column (uint8_t scale, uint16_t frequency, uint8_t rssi, uint8_t marker)
{
    if(rssi>100)
    {
        rssi=100;
    }

    // X POSTION HANDLING (range of array 0..26 = 27 positions)
    uint8_t upper=0; // marker for upper or lower sub colum in charcter
    // calculate column position of 54 columns
    
    // Note: calculation done on runtime, since preprocessor seems to have issues with forumlars
    // simple interger with 10x factor and /10 at end
    #define INTEGER_GAIN 100
    uint16_t frequency_delta=(frequency-BAND_SCANNER_FREQ_MIN); // no rouding issue
    uint16_t frequency_per_char=((BAND_SCANNER_FREQ_MAX-BAND_SCANNER_FREQ_MIN)*INTEGER_GAIN)/((BAND_SCANNER_SPECTRUM_X_MAX-1)*2);
    // special rounding is required, since lowest in on left side, highest on right sight of character
    #define ROUND_CORRECTION 2 // stretches band a little
    uint8_t x_pos_54= (frequency_delta*(INTEGER_GAIN+ROUND_CORRECTION)) / frequency_per_char;
    // find right column of 27 characters
    uint8_t x=((x_pos_54)/2); // final down scale to single character
    //osd_print_debug(1,2,"x_pos_54",x_pos_54);
    //osd_print_debug(1,3,"x",x);    
    // check for upper or lower nibble for each character
    if (x_pos_54 % 2)
    {
        upper=0;
    }
    else
    {
        upper=1;
    }
    if(marker)
    {
        // set arrow at current frequency
        char arrow_string[]="                           "; // clear line
        if(upper){
            arrow_string[x]=0x81; // insert arrow
        }
        else
        {
            arrow_string[x]=0x82; // insert arrow
        }
        // print arrow line
        osd_print (BAND_SCANNER_SPECTRUM_X_MIN, BAND_SCANNER_SPECTRUM_Y_MIN,arrow_string );

    }
    // Y SCALING
    //
    uint8_t y=0;
    uint8_t y_step= BAND_SCANNER_RSSI_MAX/scale;
    uint8_t y_step_fractional= BAND_SCANNER_RSSI_MAX/scale/BAND_SCANNER_SUB_BAR;
    uint8_t y_max_100=0; // keeps last y with 100%
    uint8_t y_fill=0; // marker to fill top of comum with 0
    // set all 100% sub bars
#if 0    
    osd_print_debug(1,2,"y_step",y_step);
    osd_print_debug(1,3,"y_step_fractional",y_step_fractional);
    osd_print_debug(1,4,"xpos",x);    
#endif  
    uint8_t value=0;
  
    for(y=1; y<=scale;y++) // 1...scale
    {
        value=spectrum_display[x][y-1];
        if(value==0xff) // remove filling center marker
        {
            value=0;
        }
        if(y*y_step < rssi)
        {
            // sub colum to 100%
            //osd_print_debug_x(15,1,"val_in",value); 
            y_max_100=y;
            // set value
            if(upper){
                // mask value to be keep other sub column
                value=value&0x0f;
                
                // set 100%
                value=value|0x30;
            }
            else
            {
                // mask value to be keept
                value=value&0xf0;
                // set 100%
                value=value|0x03;                
            }
            //osd_print_debug_x(15,2,"val_out",value); 
        }
        else
        {
            if(y_fill==0)
            {
                // handle fractional values on top of bar and beyond
                uint8_t rssi_fraction= rssi-(y_max_100*y_step);
                uint8_t colum_value= rssi_fraction/ y_step_fractional;
                // set value
                if(upper){
                    // mask value to be keep other sub column
                    value=value&0x0f;
                    // set 100%
                    value=value|(colum_value <<4);
                }
                else
                {
                    // mask value to be keept
                    value=value&0xf0;
                    // set 100%
                    value=value|colum_value;                
                }           
                y_fill=1;
            }
            else
            {
                // fill area on top with "0"
                if(upper){
                    // mask value to be keep other sub column
                    value=value&0x0f;
                    // set 0%
                    value=value|0x00;
                }
                else
                {
                    // mask value to be keept
                    value=value&0xf0;
                    // set 0%
                    value=value|0x00;                
                }
            }
        }

        
        // gap fill handler
        // We have 40 channels, but 54 colums. Due to rounding and frenqencies, some columns
        // are never used. This looks like "gaps" in the spectrum
        // To fill this gaps (they are looking ugly, and they are "filled" anyway with "spectrum")
        // special lookup tables do exist with rules, how to handle this gaps.
        // There are 4 tables
        // 1. clone upper by lower value
        // 2. clone lower by upper value
        // 3. clone a full character from right side
        // 4. clone a full character from left side
        
        // NOTE: To keep code maintainable this is NOT integrated in segement above.
        //       ... less efficient, but easier to change / fix

        // clone sub bars
        if(clone_bar_to_upper[x])
        {
            value=value|((value&0xf0) >> 4);
        }
        if(clone_bar_to_lower[x])
        {
            value=value|((value&0x0f) <<4);
        }
        // Store character
        spectrum_display[x][y-1]=value;   // set character        
        
        // clone full bar
        if(clone_bar_to_right[x])
        {
            spectrum_display[x+1][y-1]=spectrum_display[x][y-1];
        }
        if(clone_bar_to_left[x])
        {
            spectrum_display[x-1][y-1]=spectrum_display[x][y-1];
        }     
    }    
}

void spectrum_init(void)
{
    // clear array spectrum
    // fill all with ff to get cente dot.
    // botton line getes black bar (0x00).
    uint8_t  x=0;
    uint8_t  y=0;
    for (x=0; x<BAND_SCANNER_SPECTRUM_X_MAX;x++)
    {
        for (y=0; y<6;y++) 
        {
            spectrum_display[x][y]=0xff; // center dot coded as 255            
        }
    }
}

// calculate correct character for two bars depending on value
char spectrum_get_char(uint8_t value)
{
    // Note: This can be done by formular, but case it easier to adapter...
    // can be optimized

    uint8_t ret=0;
    switch(value)
    {
        case 0x00: 
            ret = 0x90;
            break;
        case 0x10: 
            ret = 0x91;
            break;
        case 0x20: 
            ret = 0x92;
            break;            
        case 0x30: 
            ret = 0x93;
            break;            
        case 0x01: 
            ret = 0x94;
            break;            
        case 0x11: 
            ret = 0x95;
            break;            
        case 0x21: 
            ret = 0x96;
            break;            
        case 0x31: 
            ret = 0x97;
            break;            
        case 0x02: 
            ret = 0x98;
            break;            
        case 0x12: 
            ret = 0x99;
            break;            
        case 0x22: 
            ret = 0x9a;
            break;            
        case 0x32: 
            ret = 0x9b;
            break;            
        case 0x03: 
            ret = 0x9c;
            break;            
        case 0x13: 
            ret = 0x9d;
            break;            
        case 0x23: 
            ret = 0x9e;
            break;            
        case 0x33: 
            ret = 0x9f;
            break;            
        default:
            ret = 0x8e; // center dot            
    }    
    return(ret);
}

void dump_channels(uint8_t y_pos)
{
    // dumps potential channels by hill climb anaysis.
    uint8_t channel_scan=0;
    uint8_t last_value=0;
    uint8_t channel_index=0;
//    char marker_string[]="                           "; // clear line
//    char marker_string[]="aaaaaaaaaabbbbbbbbbbccccccc"; // clear line
    char marker_string[]="                           "; // clear line    
    #if 1
    // Note we must run 40 + 1 to print right side (detect max)
    for (channel_scan=CHANNEL_MIN_INDEX; channel_scan <= CHANNEL_MAX_INDEX+1;channel_scan++ )
    {
        if(spectrum_channel_value[channel_scan] >= last_value)
        {
            last_value=spectrum_channel_value[channel_scan];
        }
        else // last must have been the maximum, add channel marker
        {
            // X POSTION HANDLING (range of array 0..26 = 27 positions)
            // calculate column position of 54 columns
            
            // Note: calculation done on runtime, since preprocessor seems to have issues with forumlars
            // simple interger with 10x factor and /10 at end
            #define INTEGER_GAIN 100
            channel_index = channelList[channel_scan-1];
            uint16_t frequency_delta=(pgm_read_word_near(channelFreqTable + channel_index)-BAND_SCANNER_FREQ_MIN); // no rouding issue
            uint16_t frequency_per_char=((BAND_SCANNER_FREQ_MAX-BAND_SCANNER_FREQ_MIN)*INTEGER_GAIN)/((BAND_SCANNER_SPECTRUM_X_MAX-1)*2);
            // special rounding is required, since lowest in on left side, highest on right sight of character
            #define ROUND_CORRECTION 2 // stretches band a little
            uint8_t x_pos_54= (frequency_delta*(INTEGER_GAIN+ROUND_CORRECTION)) / frequency_per_char;
            // find right column of 27 characters
            uint8_t x=((x_pos_54)/2); // final down scale to single character
            // add right marker at last position
            
            marker_string[x]=pgm_read_byte_near(channelSymbol + channelList[channel_scan-1]);
            last_value=0;
            //marker_string[10]=0xA0;
        }
    }
    #endif
    // print marler line
    osd_print (BAND_SCANNER_SPECTRUM_X_MIN, y_pos,marker_string );
    
}

void spectrum_dump (uint8_t height)
{
    // for fast dump, each line is printed at once.
    // the strings will be created from the spetrum array
    char string[BAND_SCANNER_SPECTRUM_X_MAX+1];
    string[BAND_SCANNER_SPECTRUM_X_MAX]=0; // string termination
    uint8_t  x=0;
    uint8_t  y=0;
    for (y=0; y<height;y++)     
    {
        for (x=0; x<BAND_SCANNER_SPECTRUM_X_MAX;x++)
        {
            // create line
            string[x]=spectrum_get_char(spectrum_display[x][y]);
        }
        // dump line
        osd_print(BAND_SCANNER_SPECTRUM_X_MIN,SCREEN_Y_MAX-3-y,string);
    }
}

void screen_manual_data(uint8_t channelIndex)
{
    // clear last line
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,4,"\x02 CHAN: ?  \x10 \x11 \x12 \x13 \x14 \x15 \x16 \x17\x02");
    // set correct values by replace some characters (simple code)
    // BAND
    osd_print_char(BAND_SCANNER_SPECTRUM_X_MIN+8,4,pgm_read_byte_near(bandNames + channelIndex));  
    // ACTIVE CHANNEL
    uint8_t active_channel = channelIndex%CHANNEL_BAND_SIZE; // get channel inside band
    char active=0x18 + active_channel;
    osd_print_char(BAND_SCANNER_SPECTRUM_X_MIN+11+(2*active_channel),4,active);  
    // FREQUENCY
    osd_print_int(BAND_SCANNER_SPECTRUM_X_MIN+8,5,pgm_read_word_near(channelFreqTable + channelIndex));
    // add marker for all channel per active band
    // set available channels marker
    // clear symbol line
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,SCREEN_Y_MAX,"                              ");
    uint8_t loop=0;
    for(loop=0;loop<8;loop++)
    {
        uint8_t band_number=pgm_read_byte_near(bandNumber + channelIndex);
        uint8_t channel=(band_number*8 + loop);
        uint16_t frequency=pgm_read_word_near(channelFreqTable + channel);
        // calculate x postion (see code of spectrm_add_column for details)
        uint16_t frequency_delta=(frequency-BAND_SCANNER_FREQ_MIN); // no rouding issue
        uint16_t frequency_per_char=((BAND_SCANNER_FREQ_MAX-BAND_SCANNER_FREQ_MIN)*INTEGER_GAIN)/((BAND_SCANNER_SPECTRUM_X_MAX-1)*2);
        uint8_t x_pos_54= (frequency_delta*(INTEGER_GAIN+ROUND_CORRECTION)) / frequency_per_char;
        uint8_t x=((x_pos_54)/2); // final down scale to single character
        // print marker
        osd_print_char(BAND_SCANNER_SPECTRUM_X_MIN+x,SCREEN_Y_MAX,pgm_read_byte_near(channelSymbol + channel));
    }
}

///////////////////////////////////////
//  SCREENS
///////////////////////////////////////
/*******************/
/*  START SCREEN   */
/*******************/

void screen_startup(void)
{
    const static char P_text_1[] PROGMEM  ="\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x04";
    const static char P_text_2[] PROGMEM  ="\x02 RX5808 PRO OSD \x02";
    const static char P_text_3[] PROGMEM  ="\x07\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x08";
    const static char P_text_4[] PROGMEM  ="\x02   22.11.2015   \x02";
    const static char P_text_5[] PROGMEM  ="\x02    V1.1.3      \x02";
    const static char P_text_6[] PROGMEM  ="\x02  MARKO HOEPKEN \x02";
    const static char P_text_7[] PROGMEM  ="\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06";

    uint8_t y=MENU_MODE_SELECTION_Y;
    osd.clear();
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_1);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_2);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_3);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_4);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_5);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_6);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_7);   
#ifdef membug    
    osd_print_int (MENU_MODE_SELECTION_X,y++, freeMem()); 
#endif
}

/*******************/
/*   MODE SCREEN   */
/*******************/

void screen_mode_selection(void)
{
    const static char P_text_1[] PROGMEM  = "\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x04";
    const static char P_text_2[] PROGMEM  = "\x02 MODE SELECTION \x02";
    const static char P_text_3[] PROGMEM  = "\x07\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x08";
    const static char P_text_4[] PROGMEM  = "\x02  EXIT          \x02";
    const static char P_text_5[] PROGMEM  = "\x02  AUTO SEARCH   \x02";
    const static char P_text_6[] PROGMEM  = "\x02  BAND SCANNER  \x02";
    const static char P_text_7[] PROGMEM  = "\x02  MANUEL MODE   \x02";
    const static char P_text_8[] PROGMEM  = "\x02  SETUP         \x02";
    const static char P_text_9[] PROGMEM  = "\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06";

    uint8_t y=MENU_MODE_SELECTION_Y;
    osd.clear();       
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_1);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_2);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_3);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_4);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_5);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_6);    
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_7);
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_8);
    osd_print_P(MENU_MODE_SELECTION_X,y++,P_text_9);
}

// Setup screen
void screen_setup(void)
{
    const static char P_text_1[] PROGMEM  = "\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x04";
    const static char P_text_2[] PROGMEM  = "\x02       SETUP     \x02";
    const static char P_text_3[] PROGMEM  = "\x07\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x08";
    const static char P_text_4[] PROGMEM  = "\x02  EXIT           \x02";
    const static char P_text_5[] PROGMEM  = "\x02  SAVE SETTINGS  \x02";
    const static char P_text_6[] PROGMEM  = "\x02  MANUAL:        \x02";
    const static char P_text_7[] PROGMEM  = "\x02  VIDEO :        \x02";
    const static char P_text_8[] PROGMEM  = "\x02  RSSI CALIBRATE \x02";
    const static char P_text_9[] PROGMEM  = "\x02  FONT UPLOAD    \x02";
    const static char P_text_10[] PROGMEM  ="\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06";   
    uint8_t y=MENU_SETUP_Y;
    osd.clear();
    osd_print_P(MENU_SETUP_X,y++,P_text_1);    
    osd_print_P(MENU_SETUP_X,y++,P_text_2);    
    osd_print_P(MENU_SETUP_X,y++,P_text_3);    
    osd_print_P(MENU_SETUP_X,y++,P_text_4);    
    osd_print_P(MENU_SETUP_X,y++,P_text_5);    
    osd_print_P(MENU_SETUP_X,y++,P_text_6);    
    osd_print_P(MENU_SETUP_X,y++,P_text_7);
    osd_print_P(MENU_SETUP_X,y++,P_text_8);
    osd_print_P(MENU_SETUP_X,y++,P_text_9);
    osd_print_P(MENU_SETUP_X,y++,P_text_10);
    
    // manual mode handler
    if(manual_mode== MODE_LINEAR){
        osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+5,"LINEAR");  
    }
    else
    {
        osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+5,"BAND  ");      
    }    
    // video mode handler
    if(video_mode== NTSC){
        osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+6,"NTSC");
    }
    else
    {
         osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+6,"PAL ");       
    }
}


// Band scanner screen
void screen_band_scanner(uint8_t mode)
{
    const static char P_text_1[] PROGMEM  = "\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x04";
    const static char P_text_2[] PROGMEM  = "\x02     BAND SCANNER   \xd0 0.0\x02";
    const static char P_text_3[] PROGMEM  = "\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06";
    const static char P_text_4[] PROGMEM  = "\x02  RSSI CALIBRATION       \x02";
    const static char P_text_5[] PROGMEM  = "\x02Run:?? MIN:???   MAX:??? \x02";
    const static char P_text_6[] PROGMEM  = "\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06";
    const static char P_text_7[] PROGMEM  = "\x09\x0d\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0a\x0c\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0b\x0d";

    //  mode
    // 0 : scanner
    // 1 : RSSI calibration
    osd.clear();
    osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,1,P_text_1);      
    if(mode==0)
    {
        osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,2,P_text_2);  
        osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,3,P_text_3);     
    }
    else
    {
        osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,2,P_text_4);  
        osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,3,P_text_5);  
        osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,4,P_text_6);      
    }
    osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,SCREEN_Y_MAX-2,P_text_7);     
    spectrum_dump(6); 
    // add test to show that background scan runs
    osd_print(10,7,"Scanning.."); 
    show_power(23,2);
}

// Manual settings screen
void screen_manual(uint8_t mode, uint8_t channelIndex)
{
    const static char P_text_1[] PROGMEM  = "\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x04";
    const static char P_text_2[] PROGMEM  = "\x02  MANUAL BAND MODE       \x02";
    const static char P_text_3[] PROGMEM  = "\x02  MANUAL LINEAR MODE     \x02";
    const static char P_text_4[] PROGMEM  = "\x02  AUTO MODE SEEK         \x02";
    const static char P_text_5[] PROGMEM  = "\x07\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x08";
    const static char P_text_6[] PROGMEM  = "\x02 FREQ: ???? GHz          \x02";
    const static char P_text_7[] PROGMEM  = "\x02 RSSI:\x83\x88\x88\x88\x88\x88\x88\x88\x88\x88\x88\x88\x88\x88\x88\x88\x88\x88\x89\x02";
    const static char P_text_8[] PROGMEM  = "\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06";
    const static char P_text_9[] PROGMEM  = "\x09\x0d\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0a\x0c\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0b\x0d";

    // mode
    // 0: manual
    // 1: seek
    osd.clear();
    // static default text
    osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,1,P_text_1);     
    if(mode == 0)
    {    
        if (manual_mode== MODE_BAND)
        {    
            osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,2,P_text_2);  
        }
        else
        {
            osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,2,P_text_3);                
        }
    }
    else
    {    
        osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,2,P_text_4);       
    }    
    show_power(23,2); 
    osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,3,P_text_5);           
    // CHAN comes from update function
    //    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,4,"\x02 CHAN: ?  \x10 \x11 \x12 \x13 \x14 \x15 \x16 \x17\x02");
    osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,5,P_text_6);     
    osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,6,P_text_7); 
    osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,7,P_text_8); 
    osd_print_P(BAND_SCANNER_SPECTRUM_X_MIN,SCREEN_Y_MAX-2,P_text_9); 
    
    // fill in data
    screen_manual_data(channelIndex);
    // init spectrum    
    spectrum_dump(3);   // show empty
    // add scanning text
    osd_print(10,9,"Scanning..");     
    // run background scan and show results
    background_scan(3);
    last_channel_index=255; // force new tune, since specturn changed channel    
    spectrum_dump(3);

}
// cursor handling for menue
void set_cursor(uint8_t x_offset, uint8_t y_offset, uint8_t entry, uint8_t pos)
{
    uint8_t y=0;
    for(y=1;
        y<=entry;
        y++)
        {
            uint8_t y_pos=y-1;
            if(pos == y) // set arrow
            {
                osd_print(x_offset,y_pos+y_offset,"\x80");
            }
            else // clear arrow
            {
                 osd_print(x_offset,y_pos+y_offset,"  ");           
            }
        }
}
// debounce wrapper for keys
uint8_t get_key (void)
{   
    // waits until value is stable for KEY_DEBOUNCE loops
    uint8_t last_key=0xff;
    uint8_t current_key=0;
    uint16_t key_stable=0;
    
    if(get_key_raw()) // fast exit if no key is press, to prevent slow down of main loop
    {
        while(key_stable < KEY_DEBOUNCE) // loop until stable
        {
            current_key=get_key_raw();
            if(current_key == last_key){
                key_stable++;
            }
            else
            {
                key_stable=0; // glitch, reset timer
            }
            last_key=current_key;
            delay(1);
        }
    }
    // reset hide timer, on any key
    if(current_key != KEY_NONE)
    {
        menu_hide_timer=MENU_HIDE_TIMER;
    }
    
    return(current_key);
}

// no debounce get key function
uint8_t get_key_raw (void)
{   
    uint8_t sw_dir_a2b = 0;
    uint8_t sw_dir_b2a = 0;    
    // try both directions
    // KEY_A -> KEY_B
    pinMode(KEY_A, OUTPUT);
    pinMode(KEY_B, INPUT);
    digitalWrite(KEY_B, INPUT_PULLUP);
    digitalWrite(KEY_A, LOW);
    // check if the LOW will get to port
    if(digitalRead(KEY_B) == 0)
    {
        sw_dir_a2b=1;
    }
    // KEY_B -> KEY_A
    pinMode(KEY_B, OUTPUT);
    pinMode(KEY_A, INPUT);
    digitalWrite(KEY_A, INPUT_PULLUP);
    digitalWrite(KEY_B, LOW);
    // check if the LOW will get to port
    if(digitalRead(KEY_A) == 0)
    {
        sw_dir_b2a=1;
    }    
    // turn off key driver
    pinMode(KEY_A, INPUT);
    digitalWrite(KEY_A, INPUT_PULLUP);    
    pinMode(KEY_B, INPUT);
    digitalWrite(KEY_B, INPUT_PULLUP);
    // check results
    // 0 = no key
    // 1 = Key 1
    // 2 = Key 2
    // 3 = both keys, or bypass key
    if(sw_dir_a2b && sw_dir_b2a)
    {
        return (3);
    }
    else if (sw_dir_a2b)
    {
        return (1);
    }
    else if (sw_dir_b2a)
    {
        return (2);
    }
    else
    {
        return (0);
    }    
}

void unplugSlaves(){
    //Unplug list of SPI
    digitalWrite(MAX7456_SELECT,  HIGH); // unplug OSD
}

void uploadFont()
{ 
    const static char P_text_1[] PROGMEM  = "WAITING FOR CHARACTER UPDATE";
    const static char P_text_2[] PROGMEM  = " PLESE SEND CHARACTERS WITH";
    const static char P_text_3[] PROGMEM  = "   38400 BAUD VIA TERMNAL";
    const static char P_text_4[] PROGMEM  = "SYSTEM WILL REBOOT WHEN DONE";   
    const static char P_text_5[] PROGMEM  = " SKIP UPDATE BY RESTARTING";

    uint16_t byte_count = 0;
    byte bit_count=0;
    byte ascii_binary[0x08];

    // move these local to prevent ram usage
    uint8_t character_bitmap[0x40];
    int font_count = 0;
    osd.clear();
    osd_print_P(1,2,P_text_1);     
    osd_print_P(1,5,P_text_2); 
    osd_print_P(1,6,P_text_3); 
    osd_print_P(1,8,P_text_4); 
    osd_print_P(1,11,P_text_5); 
    // enable UART for update
    Serial.begin(TELEMETRY_SPEED);    
    Serial.print(F("\n\nReady for font upload. Please send *.mcm file.\n\n"));            
    Serial.print(F("You will get a message for each character stored.\n"));            
    Serial.print(F("System will reboot when all 255 characters are stored.\n"));      
    Serial.print(F("\nTo skip the update, just restart the system.\n"));      

    while(font_count < 255) { 
        int8_t incomingByte = Serial.read();
        switch(incomingByte) // parse and decode mcm file
        {
        case 0x0d: // carridge return, end of line
            //Serial.println("cr");
            if (bit_count == 8 && (ascii_binary[0] == 0x30 || ascii_binary[0] == 0x31))
            {
                // turn 8 ascii binary bytes to single byte '01010101' = 0x55
                // fill in 64 bytes of character data
                // made this local to prevent needing a global
                byte ascii_byte;

                ascii_byte = 0;

                if (ascii_binary[0] == 0x31) // ascii '1'
                    ascii_byte = ascii_byte + 128;

                if (ascii_binary[1] == 0x31)
                    ascii_byte = ascii_byte + 64;

                if (ascii_binary[2] == 0x31)
                    ascii_byte = ascii_byte + 32;

                if (ascii_binary[3] == 0x31)
                    ascii_byte = ascii_byte + 16;

                if (ascii_binary[4] == 0x31)
                    ascii_byte = ascii_byte + 8;

                if (ascii_binary[5] == 0x31)
                    ascii_byte = ascii_byte + 4;

                if (ascii_binary[6] == 0x31)
                    ascii_byte = ascii_byte + 2;

                if (ascii_binary[7] == 0x31)
                    ascii_byte = ascii_byte + 1;

                character_bitmap[byte_count] = ascii_byte;
                byte_count++;
                bit_count = 0;
            }
            else
                bit_count = 0;
            break;
        case 0x0a: // line feed, ignore
            //Serial.println("ln");   
            break;
        case 0x30: // ascii '0'
        case 0x31: // ascii '1' 
            ascii_binary[bit_count] = incomingByte;
            bit_count++;
            break;
        default:
            break;
        }

        // we have one completed character
        // write the character to NVM 
        if(byte_count == 64)
        {
            osd.write_NVM(font_count, character_bitmap);    
            byte_count = 0;
            font_count++;
            Serial.print(font_count, DEC);
            Serial.println(" chars done");            
        }
    }
    Serial.print(F("Font update done. Restarting system..\n\n"));      
    delay(2000);
    asm volatile ("  jmp 0");  // softstart by jumping to start vector   
    // REBOOT, WILL NEVER GET HERE
}


