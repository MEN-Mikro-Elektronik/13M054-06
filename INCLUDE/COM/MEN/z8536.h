/***********************  I n c l u d e  -  F i l e  ************************
 *
 *         Name: z8536.h
 *
 *       Author: ds
 *
 *  Description: Header file for Z8536 chip
 *               - Z8536 register defines
 *
 *     Switches: ---
 *
 *---------------------------------------------------------------------------
 * Copyright 1999-2019, MEN Mikro Elektronik GmbH
 ****************************************************************************/
/*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _Z8536_H
#define _Z8536_H

#ifdef __cplusplus
      extern "C" {
#endif


/*-----------------------+
|  Z8536 register defs   |
+-----------------------*/
/* main control registers */
#define MICR    0x00	/* master interrupt control */
#define MCCR    0x01	/* master config control */
#define PAIVR   0x02	/* port A interrupt vector */
#define PBIVR   0x03	/* port B interrupt vector */
#define CTIVR	0x04	/* counter/timer interrupt vector */
#define PCDPPR  0x05    /* port C data path polarity */
#define PCDDR   0x06    /* port C data direction */
#define PCSIOCR 0x07    /* port C special i/o control */

/* most often accessed registers */
#define PACSR   0x08	/* port A command/status */
#define PBCSR   0x09	/* port B command/status */

#define PADR    0x0d	/* port A data */
#define PBDR    0x0e	/* port B data */
#define PCDR    0x0f	/* port C data */
#define PAMSR   0x20	/* port A mode specification */
#define PADPR   0x22	/* port A data path polarity */
#define PADDR   0x23	/* port A data direction */
#define PASIR   0x24	/* port A special i/o control */
#define PAPPR   0x25	/* port A pattern polarity */
#define PAPTR   0x26	/* port A pattern transition */
#define PAPMR   0x27	/* port A pattern mask */

#define PBMSR   0x28	/* port B mode specification */
#define PBDPR   0x2a	/* port B data path polarity */
#define PBDDR   0x2b	/* port B data direction */
#define PBSIR   0x2c	/* port B special i/o control */
#define PBPPR   0x2d	/* port B pattern polarity */
#define PBPTR   0x2e	/* port B pattern transition */
#define PBPMR   0x2f	/* port B pattern mask */

#define PCDPR   0x05	/* port C data path polarity */
#define PCDDR   0x06	/* port C data direction */
#define PCSIR   0x07	/* port C special i/o control */

#define T1CSR	0x0a	/* timer 1 command/status */
#define T2CSR	0x0b	/* timer 2 command/status */
#define T3CSR	0x0c	/* timer 3 command/status */
#define T1CNT	0x10	/* timer 1 counter (MSB) (LSB=this reg+1) */
#define T2CNT	0x12	/* timer 2 counter (MSB) (LSB=this reg+1) */
#define T3CNT	0x14	/* timer 3 counter (MSB) (LSB=this reg+1) */
#define T1CONS	0x16	/* timer 1 constant (MSB) (LSB=this reg+1) */
#define T2CONS	0x18	/* timer 2 constant (MSB) (LSB=this reg+1) */
#define T3CONS	0x1a	/* timer 3 constant (MSB) (LSB=this reg+1) */
#define T1MSR	0x1c	/* timer 1 mode specification */
#define T2MSR	0x1d	/* timer 2 mode specification */
#define T3MSR	0x1e	/* timer 3 mode specification */

/* MICR */
#define MIE		0x80	/* master irq enable */
#define NV		0x20	/* no vector */
#define _RESET	0x01	/* reset */

/* MCCR */
#define PBE		0x80	/* port B enable */
#define PAE		0x04	/* port A enable */
#define PCE_CT3E 0x10	/* port C+timer 3 enable */
#define CT1E    0x40	/* timer 1 enable */
#define CT2E	0x20	/* timer 2 enable */

/* PxMSR */
#define PTS_OUT	0x80	/* port type: output */
#define PTS_IN	0x40	/* port type: input */
#define IMO		0x08	/* irq on match only */
#define PMS_AND	0x02	/* pattern mode: and */
#define PMS_OR	0x04	/* pattern mode: or */
#define LPM		0x01	/* latch on pattern match */

/* PxCSR */
#define IUS		0x80	/* rd: irq under service */
#define IE		0x40	/* rd: irq enable */
#define IP		0x20	/* rd: irq pending */
#define CLR_IPS	0x20	/* wr: clear IP+IUS */
#define SET_M54IP	0x80	/* wr: set IP */
#define CLR_IP	0xa0	/* wr: clear IP */
#define SET_IE	0xc0	/* wr: set IE */
#define CLR_IE	0xe0	/* wr: clear IE */
#define ERR		0x10	/* irq error */
#define ORE		0x08	/* output reg empty */
#define IRF		0x04	/* input reg full */
#define PMF		0x02	/* pattern match flag */
#define IOE		0x01	/* irq on error */

/* PxDR */
#define Px0		0x01	/* Px0 port */
#define Px1		0x02	/* Px1 port */
#define Px2		0x04	/* Px2 port */
#define Px3		0x08	/* Px3 port */
#define Px4		0x10	/* Px4 port */
#define Px5		0x20	/* Px5 port */
#define Px6		0x40	/* Px6 port */
#define Px7		0x80	/* Px7 port */

/*--- macros to access 16 bit cio regs ---*/
#define CIO_WRITE16(h,reg,val)	\
   {cio_write(h,reg,val>>8);cio_write(h,reg+1,val);}

#define CIO_READ16(h,reg)	\
   ((cio_read(h,reg)<<8) + cio_read(h,reg+1))

/*---------------------------+
|  Definitions of IO groups  |
+---------------------------*/
#define GROUP_A			0
#define GROUP_B			1
#define GROUP_P6		GROUP_A
#define GROUP_MMOD		GROUP_A
#define GROUP_PWROUT	GROUP_B
#define	GROUP_HEXSW		GROUP_B

/*------------------------+
|  Interrupt handler defs |
+------------------------*/
#define CIO_HND_PORTA	0
#define CIO_HND_PORTB	1
#define CIO_HND_CT1		2
#define CIO_HND_CT2		3
#define CIO_HND_CT3		4


#ifdef __cplusplus
}
#endif

#endif /* Z8536_H */



