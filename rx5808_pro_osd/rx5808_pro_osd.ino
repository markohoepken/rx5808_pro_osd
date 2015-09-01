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

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 


/* **********************************************/
/* ***************** INCLUDES *******************/

//#define membug 

// AVR Includes
#include <FastSerial.h> // better steam

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif

#ifdef membug
#include <MemoryFree.h>
#endif

// Configurations

#include "ArduCam_Max7456.h"
#include <EEPROM.h>

/* *************************************************/
/* ***************** DEFINITIONS *******************/

//OSD Hardware 
//#define ArduCAM328
#define MinimOSD

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port
#define BOOTTIME         2000   // Time in milliseconds that we show boot loading bar and wait user input

// switches
#define KEY_A 0 // RX
#define KEY_B 1 // TX
#define KEY_UP 2
#define KEY_DOWN 1
#define KEY_MID 3
#define KEY_NONE 0

#define rssiPin A6

// key debounce delay in ms
// NOTE: good values are in the range of 100-200ms
// shorter values will make it more reactive, but may lead to double trigger
#define KEY_DEBOUNCE 30 // debounce in ms

// Set you TV format (PAL = Europe = 50Hz, NTSC = INT = 60Hz)
//#define TV_FORMAT NTSC
#define TV_FORMAT PAL

#define led 13
// RSSI default raw range
#define RSSI_MIN_VAL 90
#define RSSI_MAX_VAL 300
// 75% threshold, when channel is printed in spectrum
#define RSSI_SEEK_FOUND 75 
// 80% under max value for RSSI 
#define RSSI_SEEK_TRESHOLD 80
// scan loops for setup run
#define RSSI_SETUP_RUN 10

#define STATE_SEEK_FOUND 0
#define STATE_SEEK 1
#define STATE_SCAN 2
#define STATE_MANUAL 3
#define STATE_SWITCH 4
#define STATE_SETUP 5
#define STATE_RSSI_SETUP 6
#define STATE_MODE_SELECT 7

#define START_STATE STATE_SEEK
#define MAX_STATE STATE_MANUAL

#define CHANNEL_BAND_SIZE 8
#define CHANNEL_MIN_INDEX 0
#define CHANNEL_MAX_INDEX 39

#define CHANNEL_MAX 39
#define CHANNEL_MIN 0

#define TV_COLS 128
#define TV_ROWS 96
#define TV_Y_MAX TV_ROWS-1
#define TV_X_MAX TV_COLS-1
#define TV_SCANNER_OFFSET 14
#define SCANNER_BAR_SIZE 52
#define SCANNER_LIST_X_POS 4
#define SCANNER_LIST_Y_POS 16
#define SCANNER_MARKER_SIZE 2

#define EEPROM_ADR_STATE 0
#define EEPROM_ADR_TUNE 1
#define EEPROM_ADR_RSSI_MIN_L 2
#define EEPROM_ADR_RSSI_MIN_H 3
#define EEPROM_ADR_RSSI_MAX_L 4
#define EEPROM_ADR_RSSI_MAX_H 5
#define EEPROM_VIDEO_MODE 6

// Screen settings (use smaller NTSC size)
#define SCEEEN_X_MAX 30
#define SCREEN_Y_MAX 13


// Menu settings
#define MENU_MODE_SELECTION_X 6
#define MENU_MODE_SELECTION_Y 2
#define MENU_MODE_SELECTION_HEADER 3
#define MENU_MODE_SELECTION_ENTRY 5
#define WAIT_MODE_ENTRY 5

#define MENU_SETUP_X 6
#define MENU_SETUP_Y 2
#define MENU_SETUP_HEADER 3
#define MENU_SETUP_ENTRY 5
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

// CHECK
#define SCANNER_BAR_SIZE 55
#define SCANNER_LIST_X_POS 4
#define SCANNER_LIST_Y_POS 16
#define SCANNER_MARKER_SIZE 2

// Objects and Serial definitions
FastSerialPort0(Serial); // just for character update
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

const uint8_t bandNames[] PROGMEM = {
  'A','A','A','A','A','A','A','A',
  'B','B','B','B','B','B','B','B',
  'E','E','E','E','E','E','E','E',
  'F','F','F','F','F','F','F','F',
  'R','R','R','R','R','R','R','R'
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
const uint8_t channelList[] PROGMEM = {
  19, 18, 17, 16, 7, 8, 24, 6, 9, 25, 5, 10, 26, 4, 11, 27, 3, 12, 28, 2, 13, 29, 1, 14, 30, 0, 15, 31, 20, 21, 22, 23
};


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
uint8_t first_channel_marker=1;
uint8_t update_frequency_view=0;
uint8_t seek_found=0;
uint8_t last_dip_channel=255;
uint8_t last_dip_band=255;
uint8_t scan_start=0;
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


/**********************************************/
/*                   SETUP()                  */
/**********************************************/
void setup() 
{
   // use values only of EEprom is not 255 = unsaved
    uint8_t eeprom_check = EEPROM.read(EEPROM_ADR_STATE);
    if(eeprom_check == 255) // unused
    {
        EEPROM.write(EEPROM_ADR_STATE,START_STATE);
        EEPROM.write(EEPROM_ADR_TUNE,CHANNEL_MIN_INDEX);
        // save 16 bit
        EEPROM.write(EEPROM_ADR_RSSI_MIN_L,lowByte(RSSI_MIN_VAL));        
        EEPROM.write(EEPROM_ADR_RSSI_MIN_H,highByte(RSSI_MIN_VAL));    
        // save 16 bit
        EEPROM.write(EEPROM_ADR_RSSI_MAX_L,lowByte(RSSI_MAX_VAL));
        EEPROM.write(EEPROM_ADR_RSSI_MAX_H,highByte(RSSI_MAX_VAL));
        EEPROM.write(EEPROM_VIDEO_MODE,video_mode);
    }
    // debug reset EEPROM
    //EEPROM.write(EEPROM_ADR_STATE,255);    
        
    // read last setting from eeprom
    state=EEPROM.read(EEPROM_ADR_STATE);
    channelIndex=EEPROM.read(EEPROM_ADR_TUNE);
    rssi_min=((EEPROM.read(EEPROM_ADR_RSSI_MIN_H)<<8) | (EEPROM.read(EEPROM_ADR_RSSI_MIN_L)));
    rssi_max=((EEPROM.read(EEPROM_ADR_RSSI_MAX_H)<<8) | (EEPROM.read(EEPROM_ADR_RSSI_MAX_L)));
    video_mode=EEPROM.read(EEPROM_VIDEO_MODE);
    force_menu_redraw=1;
 
    unplugSlaves();
    osd.setMode(video_mode);
    osd.init();
 
    // setup spectrum screen array
    spectrum_init();
    //screen_mode_selection();  
    screen_band_scanner(0);

    #if 0
    // simple test code for spectrum buffer
    // adds columns direct to spectrum buffer
    spectrum_display[4][0]=0x33;
    spectrum_display[4][1]=0x13;
    spectrum_display[4][2]=0x03;
    spectrum_display[4][3]=0x03; 
    
    spectrum_display[5][0]=0x33;
    spectrum_display[5][1]=0x32;
    spectrum_display[5][2]=0x30;
    spectrum_display[5][3]=0x10;    
    
    spectrum_display[6][0]=0x33;
    spectrum_display[6][1]=0x33;
    spectrum_display[6][2]=0x31;
    spectrum_display[6][3]=0x20;    
    
    spectrum_dump(6);  
    #endif
    
    #if 0
    // Testcode to add single colums to buffer
    // test with split column
    // one since they are to close
    spectrum_add_column (6, 5845, 55); // left colum   // upper 1 
    spectrum_add_column (6, 5847, 88); // right column  // upper 0
                 
    //two colums
    spectrum_add_column (6, 5710, 77); // left colum   // upper 1 
    spectrum_add_column (6, 5718, 22); // right column  // upper 0
                
    // corners   
    spectrum_add_column (6, 5645, 100); // left
    spectrum_add_column (6, 5800, 100); // middle
    spectrum_add_column (6, 5945, 100); // righ
    
    spectrum_dump(6);  
    #endif
    
  

} // END of setup();


    int8_t menu=1;
/************************************************/
/*                 MAIN LOOP                    */
/************************************************/
uint16_t freq=5645;

void loop() 
{

#if 0
    uint8_t key_pressed = get_key();
    
    if(key_pressed == KEY_MID)
    {
        uploadFont(); // will not return
    }    
    if(key_pressed == KEY_UP)
    {
        menu++;
        if(menu > 4)
        {
            menu=1;
        }
    }   
    if(key_pressed == KEY_DOWN)
    {
        menu--;
        if(menu < 1)
        {
            menu=4;
        }        
    }     
#endif
    

    /************************/
    /*   Mode Select Enty   */
    /************************/
    // Special handler you must press the mode some time to get in
    if ((state != STATE_SETUP) && get_key() == KEY_MID) // key pressed ?
    {      
        osd_print_debug(1,1,"switch_count",switch_count);
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
                    screen_band_scanner(1);                      
                  //  TV.printPGM(10, TV_Y_OFFSET,  PSTR("  RSSI SETUP "));
//                    TV.print(10, SCANNER_LIST_Y_POS, "RSSI Min:     RSSI Max:   ");                    
                    // prepare new setup
                    rssi_min=0;
                    rssi_max=400; // set to max range
                    rssi_setup_min=400;
                    rssi_setup_max=0;   
                    rssi_setup_run=RSSI_SETUP_RUN;
                }   
                // trigger new scan from begin
                channel=CHANNEL_MIN;
                writePos=SCANNER_LIST_X_POS; // reset channel list
                channelIndex = pgm_read_byte_near(channelList + channel);  
                scan_start=1;
            break;
            case STATE_MANUAL: // manual mode 
            case STATE_SEEK: // seek mode
          
                if (state == STATE_MANUAL)
                {
                    screen_manual(0,channelIndex);                  
 //                   TV.printPGM(10, TV_Y_OFFSET,  PSTR(" MANUAL MODE"));                
                }
                else if(state == STATE_SEEK)
                {
                    screen_manual(1,channelIndex);  
//                    TV.printPGM(10, TV_Y_OFFSET,  PSTR("AUTO MODE SEEK"));                
                }
                first_channel_marker=1;
                update_frequency_view=1;
                force_seek=1;
            break;
            case STATE_SETUP:
                screen_setup();       
            break;
            case STATE_MODE_SELECT:
                screen_mode_selection();       
            break;            
        } // end switch
        last_state=state;
    }
    /*************************************/
    /*   Processing depending of state   */
    /*************************************/

    /*****************************************/
    /*   Processing MANUAL MODE / SEEK MODE  */
    /*****************************************/
    if(state == STATE_MANUAL || state == STATE_SEEK || state == STATE_SWITCH)
    {
        if(state == STATE_MANUAL) // MANUAL MODE
        {
            // handling of keys
            if( get_key() == KEY_UP )        // channel UP
            {         
                channelIndex++;
                if (channelIndex > CHANNEL_MAX_INDEX) 
                {  
                    channelIndex = CHANNEL_MIN_INDEX;
                } 
                force_menu_redraw=1; // show changes
            }
            if( get_key() == KEY_DOWN) // channel DOWN
            {
                channelIndex--;
                if (channelIndex > CHANNEL_MAX_INDEX) // negative overflow
                {  
                    channelIndex = CHANNEL_MAX_INDEX;
                }    
                force_menu_redraw=1; // show changes
            }            
        }
    
        // show signal strength            
        #define RSSI_BAR_SIZE 100
        rssi_scaled=map(rssi, 1, 100, 1, RSSI_BAR_SIZE);        
        //  draw new bar
//        TV.draw_rect(25, TV_Y_OFFSET+4*TV_Y_GRID, rssi_scaled, 4 , WHITE, WHITE);        
        // print bar for spectrum
        channel=channel_from_index(channelIndex); // get 0...31 index depending of current channel            
        wait_rssi_ready();
        #define SCANNER_BAR_MINI_SIZE 14
        rssi = readRSSI();
        rssi_scaled=map(rssi, 1, 100, 1, SCANNER_BAR_MINI_SIZE);
        hight = (TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled);
       //  draw new bar
        // set marker in spectrum to show current scanned channel
        if(channel < CHANNEL_MAX_INDEX)
        {
            // clear last square
            // draw next
            last_maker_pos=channel;
        }
        else
        {
          //  No action on last position to keep frame intact
        }        
        // handling for seek mode after screen and RSSI has been fully processed
        if(state == STATE_SEEK) //
        { // SEEK MODE
            if(!seek_found) // search if not found
            {
                if ((!force_seek) && (rssi > RSSI_SEEK_TRESHOLD)) // check for found channel
                {
                    seek_found=1; 
                    // beep twice as notice of lock
                    // PRINT LOCK
                } 
                else 
                { // seeking itself
                    force_seek=0;
                    // next channel
                    if (channel < CHANNEL_MAX) 
                    {
                        channel++;
                    } else {
                        channel=CHANNEL_MIN;
                    }    
                    channelIndex = pgm_read_byte_near(channelList + channel);        
                }        
            }
            else
            { // seek was successful            
//                TV.printPGM(10, TV_Y_OFFSET,  PSTR("AUTO MODE LOCK"));        
                if (get_key() == KEY_UP) // restart seek if key pressed
                {              
                    force_seek=1;
                    seek_found=0; 
//                    TV.printPGM(10, TV_Y_OFFSET,  PSTR("AUTO MODE SEEK"));        
                }                
            }
        }        
    }
    /****************************/
    /*   Processing SCAN MODE   */
    /****************************/
    else if (state == STATE_SCAN || state == STATE_RSSI_SETUP) 
    {
        // force tune on new scan start to get right RSSI value
        if(scan_start)
        {
            scan_start=0;
            setChannelModule(channelIndex);    
            last_channel_index=channelIndex;
            // keep time of tune to make sure that RSSI is stable when required
            time_of_tune=millis();
        }
        // channel marker
        if(channel < CHANNEL_MAX_INDEX)
        {
            // clear last square
//            TV.draw_rect((last_maker_pos * 4)+2, (TV_ROWS - TV_SCANNER_OFFSET + 8),SCANNER_MARKER_SIZE,SCANNER_MARKER_SIZE,  BLACK, BLACK);                
            // draw next
//            TV.draw_rect((channel * 4)+2, (TV_ROWS - TV_SCANNER_OFFSET + 8),SCANNER_MARKER_SIZE,SCANNER_MARKER_SIZE,  WHITE, WHITE);
            last_maker_pos=channel;
        }
        else
        {
          //  No action on last position to keep frame intact
        }
        // print bar for spectrum
        wait_rssi_ready();
        // value must be ready
        rssi = readRSSI();
        rssi_scaled=map(rssi, 1, 100, 5, SCANNER_BAR_SIZE);
        hight = (TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled);
        // clear last bar
//        TV.draw_rect((channel * 4), (TV_ROWS - TV_SCANNER_OFFSET - SCANNER_BAR_SIZE), 3, SCANNER_BAR_SIZE , BLACK, BLACK);
        //  draw new bar
//        TV.draw_rect((channel * 4), hight, 3, rssi_scaled , WHITE, WHITE);
        // print channelname
        if(state == STATE_SCAN)        
        {
            if (rssi > RSSI_SEEK_TRESHOLD) 
            {
//              TV.draw_rect(writePos, SCANNER_LIST_Y_POS, 20, 6,  BLACK, BLACK);
//              TV.print(writePos, SCANNER_LIST_Y_POS, pgm_read_byte_near(channelNames + channelIndex), HEX);
//              TV.print(writePos+10, SCANNER_LIST_Y_POS, pgm_read_word_near(channelFreqTable + channelIndex));
//              writePos += 30;
//              // mark bar
//              TV.print((channel * 4) - 3, hight - 5, pgm_read_byte_near(channelNames + channelIndex), HEX);            
            }
        }       
        // next channel
        if (channel < CHANNEL_MAX) 
        {
            channel++;
        } else 
        {
            channel=CHANNEL_MIN;
            writePos=SCANNER_LIST_X_POS; // reset channel list
            if(state == STATE_RSSI_SETUP)        
            {
                if(!rssi_setup_run--)    
                {
                    // setup done
                    rssi_min=rssi_setup_min;
                    rssi_max=rssi_setup_max;
                    // save 16 bit
                    EEPROM.write(EEPROM_ADR_RSSI_MIN_L,(rssi_min & 0xff));        
                    EEPROM.write(EEPROM_ADR_RSSI_MIN_H,(rssi_min >> 8));    
                    // save 16 bit
                    EEPROM.write(EEPROM_ADR_RSSI_MAX_L,(rssi_max & 0xff));
                    EEPROM.write(EEPROM_ADR_RSSI_MAX_H,(rssi_max >> 8));                    
                    state=EEPROM.read(EEPROM_ADR_STATE);
                    //beep(1000);
                }
            }            
        }    
        // new scan possible by press scan
        if (get_key() == KEY_UP) // force new full new scan
        {   
            last_state=255; // force redraw by fake state change ;-)
            channel=CHANNEL_MIN;
            writePos=SCANNER_LIST_X_POS; // reset channel list
            scan_start=1;
        }            
        // update index after channel change
        channelIndex = pgm_read_byte_near(channelList + channel);            
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
                        state=STATE_SETUP;
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
                        scan_start=1;                    
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
                        EEPROM.write(EEPROM_VIDEO_MODE,video_mode);                        
                        osd_print (MENU_SETUP_X, (MENU_SETUP_Y + 4 + MENU_SETUP_ENTRY ), " Settings saved..");
                        delay(1000);
                        osd_print (MENU_SETUP_X, (MENU_SETUP_Y + 4 + MENU_SETUP_ENTRY ), "                 ");
                    break;
                    case 2: // VIDEO MODE
                        // toggle and update
                        if(video_mode== NTSC){
                            video_mode=PAL;
                            osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+5,"PAL ");
                        }
                        else
                        {
                            video_mode=NTSC;
                            osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+5,"NTSC");       
                        }  
                        // wait key released
                        while(get_key() == KEY_UP);
                    break;
                    case 3: // RSSI CALIBRATE
                        state=STATE_RSSI_SETUP;
                    break;
                    case 4: // FONT UPLAD
                        uploadFont();                        
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
        setChannelModule(channelIndex);    
        last_channel_index=channelIndex;
        // keep time of tune to make sure that RSSI is stable when required
        time_of_tune=millis();
        // give 3 beeps when tuned to give feedback of correct start
        if(first_tune)
        {
//          first_tune=0;
//          #define UP_BEEP 100
//          beep(UP_BEEP);
//          delay(UP_BEEP);
//          beep(UP_BEEP);
//          delay(UP_BEEP);
//          beep(UP_BEEP);
        }
    }
    //rssi = readRSSI();   
 
    
    
    
    
    
    
    #if 0
    set_cursor
        (
        MENU_MODE_SELECTION_X+1, 
        MENU_MODE_SELECTION_Y + MENU_MODE_SELECTION_HEADER,
        MENU_MODE_SELECTION_ENTRY,
        menu
        );
    #endif
    
    #if 1
    // Dummy Testcode
    if (state == STATE_SCAN)
    {    
        
        if(freq <= 5945)
        {        
            spectrum_add_column (6, freq, rssi);  
            rssi=random(0, 100);         
    //        rssi+=3;
            freq+=5;
            if(rssi>100)
            {
                rssi=10;
                }
            osd_print_debug(4,3,"freq",freq);
            osd_print_debug(15,3,"rssi",rssi);
        }
        else
        {
            freq=5645;
            }
    
        spectrum_dump(6); 
        delay(10);        
    }
        
    #endif
}


/************************************************/
/*              SUB ROUTINES                    */
/************************************************/

// driver RX module
void setChannelModule(uint8_t channel) 
{
  uint8_t i;
  uint16_t channelData;
}

uint8_t channel_from_index(uint8_t channelIndex)
{
    uint8_t loop=0;
    uint8_t channel=0;
    for (loop=0;loop<=CHANNEL_MAX;loop++)
    {
        if(pgm_read_byte_near(channelList + loop) == channelIndex)
        {
            channel=loop;
            break;
        }
    }
    return (channel);
}    


void wait_rssi_ready()
{
    // CHECK FOR MINIMUM DELAY
    // check if RSSI is stable after tune by checking the time
    uint16_t tune_time = millis()-time_of_tune;
    // module need >20ms to tune.
    // 30 ms will to a 32 channel scan in 1 second.
    #define MIN_TUNE_TIME 30
    if(tune_time < MIN_TUNE_TIME)
    {
        // wait until tune time is full filled
        delay(MIN_TUNE_TIME-tune_time);
    }
}
      


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
//            TV.print(50, SCANNER_LIST_Y_POS, "   ");
//            TV.print(50, SCANNER_LIST_Y_POS, rssi_setup_min , DEC);            
        }
        if(rssi > rssi_setup_max)
        {
            rssi_setup_max=rssi;
//        TV.print(110, SCANNER_LIST_Y_POS, "   ");
//        TV.print(110, SCANNER_LIST_Y_POS, rssi_setup_max , DEC);                    
        }    
        // dump current values
    }   
    //TV.print(50, SCANNER_LIST_Y_POS-10, rssi_min , DEC);  
    //TV.print(110, SCANNER_LIST_Y_POS-10, rssi_max , DEC); 
    // scale AD RSSI Valaues to 1-100%     
    //#define RSSI_DEBUG 

    // Filter glitches
    #ifdef RSSI_DEBUG
//        TV.print(1,20, "RAW:             ");
//        TV.print(30,20, rssi, DEC);    
    #endif
    rssi = constrain(rssi, rssi_min, rssi_max);    //original 90---250
    rssi=rssi-rssi_min; // set zero point (value 0...160)
    rssi = map(rssi, 0, rssi_max-rssi_min , 1, 100);   // scale from 1..100%
    #ifdef RSSI_DEBUG
//        TV.print(1,40, "SCALED:           ");    
//        TV.print(50,40, rssi, DEC);    
    #endif
// TEST CODE    
    rssi=random(0, 100);        
    return (rssi);
}
      
   
void osd_print (uint8_t x, uint8_t y, char string[30])
{
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%s",string); 
    osd.closePanel(); 
}
void osd_print_int (uint8_t x, uint8_t y, uint16_t value)
{
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%d",value); 
    osd.closePanel(); 
}
void osd_print_char (uint8_t x, uint8_t y, char value)
{
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%c",value); 
    osd.closePanel(); 
}
void osd_print_debug (uint8_t x, uint8_t y, char string[30], uint16_t value)
{
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%s :%i   ",string,value); 
    osd.closePanel(); 
}
void osd_print_debug_x (uint8_t x, uint8_t y, char string[30], uint16_t value)
{
    osd.setPanel(x-1,y-1);  
    osd.openPanel();
    osd.printf("%s :0x%x   ",string,value); 
    osd.closePanel(); 
}

/*******************/
/*   MODE SCREEN   */
/*******************/
void screen_mode_selection(void)
{
    uint8_t y=MENU_MODE_SELECTION_Y;
    osd.clear();
    osd_print(MENU_MODE_SELECTION_X,y++,"\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x04");
    osd_print(MENU_MODE_SELECTION_X,y++,"\x02 MODE SELECTION \x02");
    osd_print(MENU_MODE_SELECTION_X,y++,"\x07\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x08");
    osd_print(MENU_MODE_SELECTION_X,y++,"\x02  EXIT          \x02");
    osd_print(MENU_MODE_SELECTION_X,y++,"\x02  AUTO SEARCH   \x02");
    osd_print(MENU_MODE_SELECTION_X,y++,"\x02  BAND SCANNER  \x02");
    osd_print(MENU_MODE_SELECTION_X,y++,"\x02  MANUEL MODE   \x02");
    osd_print(MENU_MODE_SELECTION_X,y++,"\x02  SETUP         \x02");
    osd_print(MENU_MODE_SELECTION_X,y++,"\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06");
}
/*******************/
/*   BAND SCANNER   */
/*******************/

// add one spectrum line in spectrum buffer
// this function does all the colum calcuation with rounding
void spectrum_add_column (uint8_t scale, uint16_t frequency, uint8_t rssi)
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
        spectrum_display[x][y-1]=value;        
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
            if(y==0) // fill lowest line with black bar
            {
                spectrum_display[x][y]=0x00; // black bar = 00
            }
            else
            {
                spectrum_display[x][y]=0xff; // center dot coded as 255
            }            
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
            string[x]=spectrum_get_char(spectrum_display[x][y]);
        }
        // dump string
        osd_print(BAND_SCANNER_SPECTRUM_X_MIN,SCREEN_Y_MAX-3-y,string);
    }
}

// Band scanner screen
void screen_band_scanner(uint8_t mode)
{
    //  mode
    // 0 : scanner
    // 1 : RSSI calibration
    osd.clear();
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,1,"\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x04");
    if(mode==0)
    {
        osd_print(BAND_SCANNER_SPECTRUM_X_MIN,2,"\x02       BAND SCANNER      \x02");    
        osd_print(BAND_SCANNER_SPECTRUM_X_MIN,3,"\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06");
    }
    else
    {
        osd_print(BAND_SCANNER_SPECTRUM_X_MIN,2,"\x02     RSSI CALIBRATION    \x02");        
        osd_print(BAND_SCANNER_SPECTRUM_X_MIN,3,"\x02Run:?? MIN:???   MAX:??? \x02");        
        osd_print(BAND_SCANNER_SPECTRUM_X_MIN,4,"\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06");
        }
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,SCREEN_Y_MAX-3,"\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f");
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,SCREEN_Y_MAX-2,"\x09\x0d\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0a\x0c\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0b\x0d");    
    spectrum_dump(6);    
}

// Manual settings screen
void screen_manual(uint8_t mode, uint8_t channelIndex)
{
    uint8_t y=1;
//    String buffer; // for dynamic output
    char buffer[31]; // for dynamic output
    char value; // for character insertion
    // mode
    // 0: manual
    // 1: seek
    osd.clear();
    // static default text
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,1,"\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x04");
    if(mode == 0)
    {
        osd_print(BAND_SCANNER_SPECTRUM_X_MIN,2,"\x02          MANUAL         \x02");
    }
    else
    {
        osd_print(BAND_SCANNER_SPECTRUM_X_MIN,2,"\x02          SEEK           \x02");
    }    
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,3,"\x07\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x08");
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,4,"\x02 CHAN: ?  \x10 \x11 \x12 \x13 \x14 \x15 \x16 \x17\x02");
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,5,"\x02 FREQ: ???? GHz          \x02");
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,6,"\x02 RSSI:\x83\x87\x87\x87\x87\x87\x84\x88\x88\x88\x88\x88\x88\x88\x88\x88\x88\x88\x89\x02");    
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,7,"\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06");
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,SCREEN_Y_MAX-3,"\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f");
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,SCREEN_Y_MAX-2,"\x09\x0d\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0a\x0c\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0e\x0b\x0d");        

    // set correct values by replace some characters (simple code)
    // BAND
    osd_print_char(BAND_SCANNER_SPECTRUM_X_MIN+8,4,pgm_read_byte_near(bandNames + channelIndex));  
    // ACTIVE CHANNEL
    uint8_t active_channel = channelIndex%CHANNEL_BAND_SIZE; // get channel inside band
    char active=0x18 + active_channel;
    osd_print_char(BAND_SCANNER_SPECTRUM_X_MIN+11+(2*active_channel),4,active);  
    // FREQUENCY
    osd_print_int(BAND_SCANNER_SPECTRUM_X_MIN+8,5,pgm_read_word_near(channelFreqTable + channelIndex));
    // set available channels
    osd_print(BAND_SCANNER_SPECTRUM_X_MIN,SCREEN_Y_MAX,"\xb0 \xb3 \xb2 \xb1\xb2  \xb4 \xb5 \xb6 \xb7");
    // add spectrum of current channel
    spectrum_add_column (3, pgm_read_word_near(channelFreqTable + channelIndex), random(0, 100));
    spectrum_dump(3);    
    
}

// Band scanner screen
void screen_setup(void)
{
    uint8_t y=MENU_SETUP_Y;
    osd.clear();
    osd_print(MENU_SETUP_X,y++,"\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x04");
    osd_print(MENU_SETUP_X,y++,"\x02 SETUP (UP=SAVE) \x02");
    osd_print(MENU_SETUP_X,y++,"\x07\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x08");
    osd_print(MENU_SETUP_X,y++,"\x02  EXIT           \x02");
    osd_print(MENU_SETUP_X,y++,"\x02  SAVE SETTINGS  \x02");
    osd_print(MENU_SETUP_X,y++,"\x02  VIDEO :        \x02");
    osd_print(MENU_SETUP_X,y++,"\x02  RSSI CALIBRATE \x02");
    osd_print(MENU_SETUP_X,y++,"\x02  FONT UPLOAD    \x02");
    osd_print(MENU_SETUP_X,y++,"\x05\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x06");   
    // video mode handler
    if(video_mode== NTSC){
        osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+5,"NTSC");
    }
    else
    {
         osd_print(MENU_SETUP_X+11,MENU_SETUP_Y+5,"PAL ");       
    }
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
// debounce wrapper
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
    return(current_key);
}

// no debounce
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
    
    uint16_t byte_count = 0;
    byte bit_count;
    byte ascii_binary[0x08];

    // move these local to prevent ram usage
    uint8_t character_bitmap[0x40];
    int font_count = 0;

    osd.clear();
    osd_print(2,3,"Waiting for Character Update");
    osd_print(2,5,"  Reboot to skip update");
    
    delay(1000);

    #define TELEMETRY_SPEED  57600 
    Serial.begin(TELEMETRY_SPEED);    
    Serial.println(""); // CR
    Serial.println("Ready for Font upload");

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
//            Serial.printf_P(PSTR("Char Done\n"));
            Serial.print(font_count, DEC);
            Serial.println(" chars done");
        }
    }
    Serial.println("Font update done, please reboot.");
    while(1); // wait ENDLESS

    //  character_bitmap[]
}



