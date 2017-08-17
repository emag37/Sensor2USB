/* Name: main.c
 * Project: EasyLogger
 * Author: Christian Starkjohann
 * Creation Date: 2006-04-23
 * Tabsize: 4
 * Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: Proprietary, free under certain conditions. See Documentation.
 * This Revision: $Id$
 */
#define F_CPU 16500000
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "vusb/usbdrv.h"
#include "vusb/oddebug.h"

/*
Pin assignment:
PB1 = key input (active low with pull-up)
PB3 = analog input (ADC3)
PB4 = LED output (active high)

PB0, PB2 = USB data lines
*/

#define FALSE 0
#define TRUE !FALSE
#define FIFO_SIZE 20
#define UTIL_BIN4(x)        (uint8_t)((0##x & 01000)/64 + (0##x & 0100)/16 + (0##x & 010)/4 + (0##x & 1))
#define UTIL_BIN8(hi, lo)   (uint8_t)(UTIL_BIN4(hi) * 16 + UTIL_BIN4(lo))
#define TICKS_1SEC 63
#define SET_MODE_ENTER_TICKS (TICKS_1SEC * 4)
#define DEV_ID_MAX 5 /* max device IDs*/

#ifndef NULL
#define NULL    ((void *)0)
#endif

/* ------------------------------------------------------------------------- */

typedef enum _pinChg_t { NOCHANGE=0,RISING,FALLING} pinChg_t;
typedef struct _pinChgFifo_t {
	 pinChg_t dPinState[FIFO_SIZE];
     uint8_t readIdx;
	 uint8_t writeIdx;	 
} pinChgFifo_t;

 volatile uint8_t reportBuffer[2];    /* buffer for HID reports */
 volatile char keyBuff[5] = {'\0','\0','\0','\0','\0'},*keySendPtr;
 volatile uint8_t prevDpins=0,devId;
 volatile uint8_t idleRate,tickCnt,inSetMode,newDevId,keySendTerm;
 volatile char pinIdKbrd[2] = {30,31};
 uint8_t EEMEM oscCalSavedVal, savedDevId;

 volatile pinChgFifo_t d1EvtBuff,d2EvtBuff;
/* ------------------------------------------------------------------------- */

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* Keyboard usage values, see usb.org's HID-usage-tables document, chapter
 * 10 Keyboard/Keypad Page for more codes.
 */
#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)



/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */
void InitPinChgFifo(pinChgFifo_t* fifo){
	fifo->readIdx = 0;
	fifo->writeIdx = 0;
}

uint8_t PinChgFifoEmpty(pinChgFifo_t* fifo){
	if(fifo->readIdx == fifo->writeIdx){
		return TRUE;
		}	else{
		return FALSE;
	}
}

void PinChgFifoPush(pinChgFifo_t* fifo,pinChg_t ev){
	fifo->dPinState[fifo->writeIdx] = ev;
	fifo->writeIdx = (fifo->writeIdx + 1) % FIFO_SIZE;
}

pinChg_t PinChgFifoPop(pinChgFifo_t* fifo){
	pinChg_t ret;
	if(!PinChgFifoEmpty(fifo)){
		ret = fifo->dPinState[fifo->readIdx];
		fifo->readIdx = (fifo->readIdx + 1) % FIFO_SIZE;
		return ret;
	}
	return NOCHANGE;
}


uint8_t	usbFunctionSetup(uint8_t data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uint8_t       step = 128;
uint8_t       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void    usbEventResetReady(void)
{
    /* Disable interrupts during oscillator calibration since
     * usbMeasureFrameLength() counts CPU cycles.
     */
    cli();
    calibrateOscillator();
    sei();
    eeprom_write_byte(&oscCalSavedVal, OSCCAL);   /* store the calibrated value in EEPROM */
}
uint8_t ReadDataPins(void){
	uint8_t pins = PINB & ((1<<PB1) | (1<<PB3));
	return pins;
}
void InitPinChgInt(void){
	DDRB &= ~((1 << PB1) | (1 << PB3));
	PCMSK |= (1 << PCINT1) | (1 << PCINT3);
	GIMSK |= (1 << PCIE);
	prevDpins = ReadDataPins();
	
}

void InitIdBtn(void){
	DDRB &= ~((1 << PB4));

	TCCR0A = 0;	
	TCCR0B = 0b101; /* Prescale by 1024 @ 16.5MHz Overflow rate: 62.94 Hz */
	TIMSK |= (1<<TOIE0);
}

uint8_t SetBtnPressed(void){
	//ID button is active-low!
	if(PINB & (1<<PB4)){
		return FALSE;
	}else{
		return TRUE;
	}
	
}
pinChg_t CheckPinChange(uint8_t dPins,uint8_t pinId){
	pinChg_t retState;
	if((dPins & (1<<pinId)) != 0 && (prevDpins & (1<< pinId)) == 0){
		retState = RISING;
		}else if((dPins & (1<<pinId)) == 0 && (prevDpins & (1<< pinId)) != 0){
		retState =FALLING;
		}else{
		retState = NOCHANGE;
	}
	return retState;
}
void WriteKeyBuff(char* toWrite,uint8_t len){
	if(len < (sizeof(keyBuff)-1)){
		memcpy(keyBuff,toWrite,len);
	}
	keyBuff[len] = '\0';
	keySendPtr=keyBuff;
}
void SetKeyBuffEvt(pinChg_t ev,uint8_t idx){
	char keys[2];
	keys[0] = pinIdKbrd[idx] + (devId << 1);
	if(ev == RISING){
		keys[1] = 21;
		}else{
		keys[1] = 9;
	}
	WriteKeyBuff(keys,sizeof(keys));
}
void SetKeyBuffId(uint8_t id){
	char keys[1];
	keys[0] = id + 30;
	WriteKeyBuff(keys,sizeof(keys));
}

uint8_t KeySendActive(){
	return (*keySendPtr != '\0' || keySendTerm);
}
/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */
int main(void)
{
uint8_t   i;
uint8_t   calibrationValue;
	keySendPtr = keyBuff;
    calibrationValue = eeprom_read_byte(&oscCalSavedVal); /* calibration value from last time */
	
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
	devId = eeprom_read_byte(&savedDevId);
	if(devId > DEV_ID_MAX){
		devId = 0;	
		eeprom_write_byte(&savedDevId,devId);
	}
   // odDebugInit();
    usbDeviceDisconnect();
    _delay_ms(500);
    usbDeviceConnect();

    wdt_enable(WDTO_1S);
    usbInit();

	InitPinChgFifo(&d1EvtBuff);
	InitPinChgFifo(&d2EvtBuff);
	InitPinChgInt();
	InitIdBtn();
    sei();
    for(;;){    /* main event loop */
        wdt_reset();
		if(!KeySendActive()){
			if(newDevId){
				eeprom_write_byte(&savedDevId,devId);
				SetKeyBuffId(devId);
				newDevId=FALSE;
			}else if(!PinChgFifoEmpty(&d1EvtBuff)){
				SetKeyBuffEvt(PinChgFifoPop(&d1EvtBuff),0);
			}else if(!PinChgFifoEmpty(&d2EvtBuff))	{
				SetKeyBuffEvt(PinChgFifoPop(&d2EvtBuff),1);
			}
		}
        usbPoll();
        if(usbInterruptIsReady() && KeySendActive()){ /* we can send another key */
			if(!keySendTerm){
				reportBuffer[1] = *keySendPtr;
				keySendPtr++;
				if(*keySendPtr == '\0'){
					keySendTerm = TRUE;
				}
			}else{
				reportBuffer[1] = 0;
				keySendTerm = FALSE;
			}
            usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
        }
    }
    return 0;
}


ISR(PCINT0_vect){
	uint8_t dPins = ReadDataPins();
	pinChg_t pinState;
	pinState = CheckPinChange(dPins,PB1);
	if(pinState != NOCHANGE){
		PinChgFifoPush(&d1EvtBuff,pinState);
	}
	pinState = CheckPinChange(dPins,PB3);
	if(pinState != NOCHANGE){
		PinChgFifoPush(&d2EvtBuff,pinState);
	}
	prevDpins = dPins;
}

ISR(TIMER0_OVF_vect){
	if(SetBtnPressed()){
		tickCnt++;
		if(!inSetMode && tickCnt >= SET_MODE_ENTER_TICKS){
			inSetMode = TRUE;
			tickCnt = 0;
		}else if(inSetMode && tickCnt >= TICKS_1SEC){
			devId = (devId + 1) % DEV_ID_MAX;
			newDevId = TRUE;
			tickCnt = 0;
		}
	}else{
		tickCnt = 0;
		inSetMode = FALSE;
	}
}