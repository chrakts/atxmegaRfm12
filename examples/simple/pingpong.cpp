#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

#include "../../../communication/Communication.h"
#include "rfm12.h"

//#include "../uart_lib/uart.h"
#define LED_PORT

void init_clock(int sysclk, int pll);


Serial debug(0);

enum{QUARZ,CLK2M,CLK32M};

#define SYSCLK QUARZ

#define PLL 0


int main ( void )
{
	uint8_t *bufptr;
	uint8_t i;
	uint8_t tv[] = "foobar";

	uint16_t ticker = 0;
	init_clock(SYSCLK, PLL);

	sei();           //interrupts on
    debug.open(Serial::BAUD_57600,F_CPU);
    debug.print("Pingpong-Setup ...\n\r");

	rfm12_init();    //init the RFM12

	#ifdef LED_PORT
		//LED_DDR |= _BV(LED_BIT); //enable LED if any
		LED_ROT_DIRECTION;
	#endif
	//uart// uart_init();

	for(i=0;i<20;i++)
    {
        _delay_ms(100);  //little delay for the rfm12 to initialize properly
        LED_ROT_TOGGLE;
    }


	//uart// uart_putstr ("\r\n" "RFM12 Pingpong test\r\n");
	debug.print("RFM12 Pingpong test\r\n");

	while (42) //while the universe and everything
	{
		if (rfm12_rx_status() == STATUS_COMPLETE)
		{
			//so we have received a message

			//blink the LED
			#ifdef LED_PORT
				//LED_PORT ^= _BV(LED_BIT);
				LED_ROT_TOGGLE;
			#endif

			//uart// uart_putstr ("new packet: \"");
			debug.print("new packet:");

			bufptr = rfm12_rx_buffer(); //get the address of the current rx buffer

			// dump buffer contents to uart
			for (i=0;i<rfm12_rx_len();i++)
			{
				// ### uart_putc ( bufptr[i] );
				debug.transmit(bufptr[i]);
			}

			//uart// uart_putstr ("\"\r\n");
            debug.print("-\r\n");
			// tell the implementation that the buffer
			// can be reused for the next data.
			rfm12_rx_clear();
		}


		ticker ++;
		if(ticker == 3000){
			ticker = 0;
			//uart// uart_putstr (".\r\n");
			debug.print(".\r\n");
			LED_ROT_TOGGLE;
			rfm12_tx(sizeof(tv), 0, tv);
		}

		//rfm12 needs to be called from your main loop periodically.
		//it checks if the rf channel is free (no one else transmitting), and then
		//sends packets, that have been queued by rfm12_tx above.
		rfm12_tick();

		_delay_us(100); //small delay so loop doesn't run as fast
	}
}

void init_clock(int sysclk, int pll)
{
	CLK_t *mein_clock;
	OSC_t *mein_osc;
	mein_clock = &CLK;
	mein_osc = &OSC;
	switch(sysclk)
	{
		case QUARZ:
			mein_osc->XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;
//			mein_osc->XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCPWR_bm | OSC_XOSCSEL_XTAL_16KCLK_gc;
			mein_osc->CTRL = OSC_XOSCEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 32 MHz-Clock ein

			while((mein_osc->STATUS & OSC_XOSCRDY_bm) == 0)			// wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)		// wartet bis diese stabil
			;

			if ( (pll>0) & (pll<16) )
			{
				mein_osc->PLLCTRL = OSC_PLLSRC_XOSC_gc | pll;
				mein_osc->CTRL = OSC_PLLEN_bm | OSC_XOSCEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet zusätzlich die PLL ein

				while((mein_osc->STATUS & OSC_PLLRDY_bm) == 0)		// wartet bis diese stabil
				;
				CCP = CCP_IOREG_gc;										// geschuetztes Register freigeben
				mein_clock->CTRL = CLK_SCLKSEL_PLL_gc;					// umschalten auf PLL-Clock
				mein_osc->CTRL = OSC_PLLEN_bm | OSC_XOSCEN_bm | OSC_RC32KEN_bm;
			}
			else
			{
				CCP = CCP_IOREG_gc;										// geschuetztes Register freigeben
				mein_clock->CTRL = CLK_SCLKSEL_XOSC_gc;					// umschalten auf XOSC-Clock
				mein_osc->CTRL = OSC_XOSCEN_bm | OSC_RC32KEN_bm;
			}
		break; // QUARZ
		case CLK2M:
			mein_osc->CTRL = OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 2 MHz-Clock ein
			while((mein_osc->STATUS & OSC_RC2MRDY_bm) == 0)  // wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)  // wartet bis diese stabil
			;
			CCP = CCP_IOREG_gc;								// geschuetztes Register freigeben
			mein_clock->CTRL = CLK_SCLKSEL_RC2M_gc;		// umschalten auf 2 MHz-Clock
//			CLKSYS_AutoCalibration_Enable(OSC_RC2MCREF_RC32K_gc,false); // OSC_RC32MCREF_bm
		break;
		case CLK32M:
			mein_osc->CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 32 MHz-Clock ein
			while((mein_osc->STATUS & OSC_RC32MRDY_bm) == 0)  // wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)  // wartet bis diese stabil
			;
			CCP = CCP_IOREG_gc;								// geschuetztes Register freigeben
			mein_clock->CTRL = CLK_SCLKSEL_RC32M_gc;		// umschalten auf 32 MHz-Clock
			mein_osc->CTRL = OSC_RC32MEN_bm | OSC_RC32KEN_bm;		// abschalten der 2 MHz-Clock
//			CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_RC32K_gc,false); // OSC_RC32MCREF_bm
		break;
	}
}

/*! \brief This function enables automatic calibration of the selected internal
 *         oscillator.
 *
 *  Either the internal 32kHz RC oscillator or an external 32kHz
 *  crystal can be used as a calibration reference. The user must make sure
 *  that the selected reference is ready and running.
 *
 *  \param  clkSource    Clock source to calibrate, either OSC_RC2MCREF_bm or
 *                       OSC_RC32MCREF_bm.
 *  \param  extReference True if external crystal should be used as reference.
 */
/*
void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, bool extReference )
{
	OSC.DFLLCTRL = ( OSC.DFLLCTRL & ~clkSource ) |
	               ( extReference ? clkSource : 0 );
	if (clkSource == OSC_RC2MCREF_bm) {
		DFLLRC2M.CTRL |= DFLL_ENABLE_bm;
	} else if (clkSource == OSC_RC32MCREF_RC32K_gc) {   // OSC_RC32MCREF_bm
		DFLLRC32M.CTRL |= DFLL_ENABLE_bm;
	}
}
*/
