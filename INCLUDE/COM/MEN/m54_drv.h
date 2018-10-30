/***********************  I n c l u d e  -  F i l e  ************************
 *
 *         Name: m54_drv.h
 *
 *       Author: kp
 *        $Date: 2015/02/18 16:20:20 $
 *    $Revision: 2.2 $
 *
 *  Description: Header file for M54 driver
 *               - M54 specific status codes
 *               - M54 function prototypes
 *
 *     Switches: _ONE_NAMESPACE_PER_DRIVER_
 *               _LL_DRV_
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2001 by MEN mikro elektronik GmbH, Nuernberg, Germany
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

#ifndef _M54_DRV_H
#define _M54_DRV_H

#ifdef __cplusplus
      extern "C" {
#endif


/*-----------------------------------------+
|  TYPEDEFS                                |
+-----------------------------------------*/

/* structure to be passed via block getstats/setstats */
typedef struct {
	u_int16	opcode;				/* LM628 opcode */
	u_int16	data[7];			/* command specific data words [0..7] */
} M54_LM628_CMDBLOCK;

/*-----------------------------------------+
|  DEFINES                                 |
+-----------------------------------------*/
/* channel defines */
#define M54_CH_LM628	0		/* channel to access LM628 */
#define M54_CH_BININ	1		/* binary inputs */
#define M54_CH_BINOUT	2		/* binary outputs */

/* M54 specific status codes (STD) */			/* S,G: S=setstat, G=getstat */
#define M54_LM_STATUS			(M_DEV_OF+0x00) /*   G: get LM628 status */
#define M54_LM_INT_REASON		(M_DEV_OF+0x01)	/*   G: get LM628 int reason */

#define M54_BININ_EDGE_CONFIG 	(M_DEV_OF+0x08)	/* S,G: define sensitive edge 
												        of binary input irqs */
#define M54_QUAD_COMP			(M_DEV_OF+0x09)	/* S,G: define threshold for
												        glitch detection*/
#define M54_POLARITY			(M_DEV_OF+0x0a)	/* S  : output polarity */

#define M54_SETSIG_LM628		(M_DEV_OF+0x10)	/* S  : install signal for 
												        LM628 ints */
#define M54_CLRSIG_LM628		(M_DEV_OF+0x11) /* S  : remove signal for 
												        LM628 ints */
#define M54_SETSIG_BININ_EDGE	(M_DEV_OF+0x12) /* S  : install signal for 
												        binary input ints */
#define M54_CLRSIG_BININ_EDGE	(M_DEV_OF+0x13) /* S  : remove signal for 
												        binary input ints */
#define M54_SETSIG_LINE_BREAK	(M_DEV_OF+0x14) /* S  : install signal for 
												        line break ints */
#define M54_CLRSIG_LINE_BREAK	(M_DEV_OF+0x15) /* S  : remove signal for 
												        line break ints */
#define M54_SETSIG_GLITCH		(M_DEV_OF+0x16) /* S  : install signal for 
												        glitch ints */
#define M54_CLRSIG_GLITCH		(M_DEV_OF+0x17) /* S  : remove signal for 
												        glitch ints */

/* M54 specific status codes (BLK) */			/* S,G: S=setstat, G=getstat */
#define M54_LM628_PERFORM_CMD	(M_DEV_BLK_OF+0x00)/* S,G: perform LM628 cmd */

/* M54_BININ_EDGE_CONFIG: irq on edge flags */
#define M54_IENOFF 0x0			/* irq disabled */
#define M54_IEALL 0x1			/* irq on all edges */
#define M54_IELH  0x2			/* irq on rising edge */
#define M54_IEHL  0x3			/* irq on falling edge */

/* flags returned by M54_LM_STATUS in addition to the status flags of LM628 */
#define M54_STAT_LINE_BREAK	0x100 	/* line break detected */
#define M54_STAT_GLITCH		0x200 	/* glitch detected */

/*-----------------------------------------+
|  PROTOTYPES                              |
+-----------------------------------------*/
#ifdef _LL_DRV_

# ifdef _ONE_NAMESPACE_PER_DRIVER_
#	define M54_GetEntry		LL_GetEntry
# else
	/* variant for swapped access */
#	ifdef M54_SW
#		define M54_GetEntry		M54_SW_GetEntry
#	endif
	extern void M54_GetEntry(LL_ENTRY* drvP);
# endif
#endif /* _LL_DRV_ */

/*-----------------------------------------+
|  BACKWARD COMPATIBILITY TO MDIS4         |
+-----------------------------------------*/
#ifndef U_INT32_OR_64
 /* we have an MDIS4 men_types.h and mdis_api.h included */
 /* only 32bit compatibility needed!                     */
 #define INT32_OR_64  int32
 #define U_INT32_OR_64 u_int32
 typedef INT32_OR_64  MDIS_PATH;
#endif /* U_INT32_OR_64 */


#ifdef __cplusplus
      }
#endif

#endif /* _M54_DRV_H */

