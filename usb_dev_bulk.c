#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "grlib/grlib.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usb_bulk_structs.h"
#include "driverlib/adc.h"



// The system tick rate expressed both as ticks per second and a millisecond period.
#define SYSTICKS_PER_SECOND 100
#define SYSTICK_PERIOD_MS (1000 / SYSTICKS_PER_SECOND)

// The global system tick counter.
volatile unsigned long g_ulSysTickCount = 0;

// Flags used to pass commands from interrupt context to the main loop.
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile unsigned long g_ulFlags = 0;
char *g_pcStatus;

// Global flag indicating that a USB configuration has been set.
static volatile tBoolean g_bUSBConfigured = false;

// Global flags for oscilloscope operation
volatile tBoolean triggering = false;
volatile tBoolean TX_REQUEST = false;

// Global oscilloscope variables
volatile unsigned short triggerLevel = 0;
volatile unsigned long sampling_rate = 100000;

// Definitions of screen size and trigger buffer length
#define TRIG_BUFF_LEN 64
#define SCREEN_SIZE 1500;

// Interrupt handler for the system tick counter.
void
SysTickIntHandler(void)
{
    // Update our system tick counter.
    g_ulSysTickCount++;
}

// Function to obtain a screen worth of samples
void getScreen(unsigned long offset) {
	int sample_num;
	unsigned long ulWriteIndex;
	tUSBRingBufObject sTxRing;
	USBBufferInfoGet(&g_sTxBuffer, &sTxRing);
	ulWriteIndex = sTxRing.ulWriteIndex + (offset * 4);

	if(sampling_rate != 2000000) {
		for( sample_num = 0; sample_num < (1500 - offset); sample_num++ ) {
			// wait for sample to be ready
			while(!ADCIntStatus(ADC0_BASE, 3, false));
			// clear the interrupt
			ADCIntClear(ADC0_BASE, 3);
			// put data into USB buffer and increment indexes
			ADCSequenceDataGet(ADC0_BASE, 3, &g_pucUSBTxBuffer[ulWriteIndex]);
			ulWriteIndex += 4;
			ulWriteIndex = (ulWriteIndex >= BULK_BUFFER_SIZE) ? 0 : ulWriteIndex;
		}
	}
	else {
		for( sample_num = 0; sample_num < (1500 - offset); sample_num++ ) {
			while(!ADCIntStatus(ADC1_BASE, 3, false));
			ADCIntClear(ADC0_BASE, 3);
			ADCIntClear(ADC1_BASE, 3);
			ADCSequenceDataGet(ADC0_BASE, 3, &g_pucUSBTxBuffer[ulWriteIndex]);
			ulWriteIndex += 4;
			ulWriteIndex = (ulWriteIndex >= BULK_BUFFER_SIZE) ? 0 : ulWriteIndex;
			ADCSequenceDataGet(ADC1_BASE, 3, &g_pucUSBTxBuffer[ulWriteIndex]);
			ulWriteIndex += 4;
			ulWriteIndex = (ulWriteIndex >= BULK_BUFFER_SIZE) ? 0 : ulWriteIndex;
		}
	}
}

// function to change the period of the timer
void setSamplingRate() {
	if(sampling_rate != 2000000) {
		TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/sampling_rate);
	}
	else if(sampling_rate == 2000000) {
		TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/1000000);
	}
}

// function to return a power of 10
int pow10(char byte) {
	int i;
	int res = 1;
	for(i=0; i<byte; i++) {
		res = res * 10;
	}
	return res;
}

// Function that decodes the recieved data and performs the commands from it
static unsigned long GetSettings(tUSBDBulkDevice *psDevice, unsigned char *pcData, unsigned long ulNumBytes) {
	// Check that recieved packet is of correct size
	if(ulNumBytes == 3) {
		unsigned long ulReadIndex;
		ulReadIndex = (unsigned long)(pcData - g_pucUSBRxBuffer);
		char byte[3];

		byte[0] = g_pucUSBRxBuffer[ulReadIndex];

		switch(byte[0]) {
		case 0x01 : // Trigger on
			triggering = true;
			break;
		case 0x10 : // Trigger off
			triggering = false;
			triggerLevel = 0;
			break;
		case 0x03 : // Set trigger
			ulReadIndex++;
			ulReadIndex = (ulReadIndex == BULK_BUFFER_SIZE) ? 0 : ulReadIndex;
			byte[1] = g_pucUSBRxBuffer[ulReadIndex];
			ulReadIndex++;
			ulReadIndex = (ulReadIndex == BULK_BUFFER_SIZE) ? 0 : ulReadIndex;
			byte[2] = g_pucUSBRxBuffer[ulReadIndex];
			// convert byte array into short
			triggerLevel = ((unsigned short) byte[1] + ((unsigned short) byte[2] << 8));
			break;
		case 0x04 : // set the sampling rate
			ulReadIndex++;
			ulReadIndex = (ulReadIndex == BULK_BUFFER_SIZE) ? 0 : ulReadIndex;
			byte[1] = g_pucUSBRxBuffer[ulReadIndex];
			ulReadIndex++;
			ulReadIndex = (ulReadIndex == BULK_BUFFER_SIZE) ? 0 : ulReadIndex;
			byte[2] = g_pucUSBRxBuffer[ulReadIndex];
			sampling_rate = ((int) byte[1]) * pow10(byte[2]);
			setSamplingRate();
			break;
		case 0x05 : // data request
			TX_REQUEST = true;
		}
		// return the number of btes recieved
		return 3;
	}
	else {
		// return 0 for error in number of bytes
		return 0;
	}
}

//
unsigned long
TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    //
    // We are not required to do anything in response to any transmit event
    // in this example. All we do is update our transmit counter.
    //
    if(ulEvent == USB_EVENT_TX_COMPLETE)
    {
    }

    return(0);
}

// Handler for recieve events
unsigned long
RxHandler(void *pvCBData, unsigned long ulEvent,
               unsigned long ulMsgValue, void *pvMsgData)
{
    switch(ulEvent)
    {
    	// Update flag if usb connected
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
            g_ulFlags |= COMMAND_STATUS_UPDATE;

            // Flush our buffers.
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            break;
        }

        // Update flag if usb disconnected
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            break;
        }

        // A new packet has been received.
        case USB_EVENT_RX_AVAILABLE:
        {
            tUSBDBulkDevice *psDevice;

            // Get a pointer to our instance data from the callback data
            // parameter.
            psDevice = (tUSBDBulkDevice *)pvCBData;
            // return the valye of getsettings
            return GetSettings(psDevice, pvMsgData, ulMsgValue);

        }

        // Ignore SUSPEND and RESUME for now.
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        // Ignore all other events and return 0.
        default:
            break;
    }

    return(0);
}

//function that returns the average of the input array
unsigned long average(unsigned long * Buffer, unsigned long length) {
	unsigned long sum = 0;
	int i;
	for(i=0; i<length; i++) {
		sum += Buffer[i];
	}
	return sum/length;
}

//function that returns the smallest value from the input array
unsigned long smallest(unsigned long * Buffer, unsigned long length) {
	unsigned long small = Buffer[0];
	int i;
	for(i=1; i<length; i++) {
		if(small > Buffer[i]) {
			small = Buffer[i];
		}
	}
	return small;
}

// Main function
int main(void) {
    unsigned long triggerBuff[TRIG_BUFF_LEN];
    unsigned short OldestSample = 0;
    unsigned short nOldestSample = 1;
    unsigned long ulWriteIndex;
    tUSBRingBufObject sTxRing;


    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    ROM_FPULazyStackingEnable();

    // Set the clocking to run from the PLL at 50MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Not configured initially.
    g_bUSBConfigured = false;


    // Enable the GPIO peripheral used for USB, and configure the USB
    // pins.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // Enable the system tick.
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();

    // Initialise the transmit and receive buffers.
    USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
    USBBufferInit((tUSBBuffer *)&g_sRxBuffer);

    // Pass our device information to the USB library and place the device
    // on the bus.
    USBDBulkInit(0, (tUSBDBulkDevice *)&g_sBulkDevice);

    //Enable the timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerDisable(TIMER0_BASE, TIMER_A);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/100000);
	TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
	TimerEnable(TIMER0_BASE, TIMER_A);


	// Enable ADC0 and ADC1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

	// Enable pins on port E for analog ADC input
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

	// Set up ADCs
	SysCtlADCSpeedSet(SYSCTL_ADCSPEED_1MSPS);
	ADCPhaseDelaySet(ADC0_BASE, ADC_PHASE_0);
	ADCPhaseDelaySet(ADC1_BASE, ADC_PHASE_180);

	// Configure ADC sequences
	ADCSequenceDisable(ADC0_BASE, 3);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
	ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_TIMER, 1);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_D | ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_D | ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);

	// Clear ADC interrupts
	ADCIntClear(ADC0_BASE, 3);
	ADCIntClear(ADC1_BASE, 3);


    // Main application loop.
    while(1) {
    	while(!g_bUSBConfigured);
			if(triggering) {
				int i, j;
				for(i=0; i<TRIG_BUFF_LEN; i++) {
					while(!ADCIntStatus(ADC0_BASE, 3, false));
					ADCIntClear(ADC0_BASE, 3);
					ADCSequenceDataGet(ADC0_BASE, 3, &triggerBuff[i]);
				}

				// check trigger conditions
    	    	if(triggerBuff[OldestSample] < triggerLevel && triggerBuff[nOldestSample] >= triggerLevel
    	    			&& triggerBuff[OldestSample] < average(&triggerBuff[0], TRIG_BUFF_LEN)) {
//				if(triggerBuff[OldestSample] < triggerLevel && triggerBuff[nOldestSample] >= triggerLevel
//						&& triggerBuff[OldestSample] == smallest(&triggerBuff[0], TRIG_BUFF_LEN)) {

    	    		// if true put data in USB buffer and call getscreen
					USBBufferInfoGet(&g_sTxBuffer, &sTxRing);
					ulWriteIndex = sTxRing.ulWriteIndex;
					j = OldestSample;

					for(i=0; i++; i<TRIG_BUFF_LEN) {
						g_pucUSBTxBuffer[ulWriteIndex] = triggerBuff[j];
						ulWriteIndex += 4;
						ulWriteIndex = (ulWriteIndex >= BULK_BUFFER_SIZE) ? 0 : ulWriteIndex;
						j++;
						j &= (TRIG_BUFF_LEN - 1);
					}
					getScreen(TRIG_BUFF_LEN);

					// wait for the data request and then send data
					while(!TX_REQUEST);
					TX_REQUEST = false;
					USBBufferDataWritten(&g_sTxBuffer, 6000);
				}

				else {
					// If trigger event not occured put new sample in buffer
					// and update indexes
					while(!ADCIntStatus(ADC0_BASE, 3, false));
					ADCIntClear(ADC0_BASE, 3);
					ADCSequenceDataGet(ADC0_BASE, 3, &triggerBuff[OldestSample]);

					nOldestSample = OldestSample;
					OldestSample++;
					OldestSample &= (TRIG_BUFF_LEN - 1);
				}
			}
			else {
				// If not triggering call getscreen with no offset
				getScreen(0);
				while(!TX_REQUEST);
				TX_REQUEST = false;
				USBBufferDataWritten(&g_sTxBuffer, 6000);
			}
    }
}
