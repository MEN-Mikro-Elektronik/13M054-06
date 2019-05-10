/***********************  I n c l u d e  -  F i l e  ************************
 *  
 *         Name: lm628.h
 *
 *       Author: kp
 *        $Date: 2001/09/26 08:03:30 $
 *    $Revision: 2.1 $
 * 
 *  Description: Definitions for the LM628 DC motor controller
 *                      
 *     Switches: -
 *
 *---------------------------------------------------------------------------
 * Copyright (c) 2001-2019, MEN Mikro Elektronik GmbH
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

#ifndef _LM628_H
#define _LM628_H

#ifdef __cplusplus
	extern "C" {
#endif

/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/

/*------------------------------+
| command opcodes               |
+------------------------------*/
#define LM628_RESET    0x00
#define LM628_PORT8    0x05
#define LM628_PORT12   0x06
#define LM628_DFH      0x02
#define LM628_SIP      0x03
#define LM628_LPEI     0x1b
#define LM628_LPES     0x1a
#define LM628_SBPA     0x20
#define LM628_SBPR     0x21
#define LM628_MSKI     0x1c
#define LM628_RSTI     0x1d
#define LM628_LFIL     0x1e
#define LM628_UDF      0x04
#define LM628_LTRJ     0x1f
#define LM628_STT      0x01
#define LM628_RDSIGS   0x0c
#define LM628_RDIP     0x09
#define LM628_RDDP     0x08
#define LM628_RDRP     0x0a
#define LM628_RDDV     0x07
#define LM628_RDRV     0x0b
#define LM628_RDSUM    0x0d

/*------------------------------+
|  status/interrupt mask bits   |
+------------------------------*/
#define LM628_MOTORF   0x80    /* motor off */
#define LM628_BRKPNT   0x40    /* breakpoint reached */
#define LM628_POSERR   0x20    /* position error */
#define LM628_WRAPAR   0x10    /* wrap around occured */
#define LM628_INDEXP   0x08    /* index pulse detected */
#define LM628_TRAJEC   0x04    /* trajectory completed */
#define LM628_CMDERR   0x02    /* command error */
#define LM628_BUSY	   0x01	   /* LM628 busy */
/*------------------------------+
| filter control word bits      |
+------------------------------*/
#define LM628_LOAD_KP 0x0008   /* load P-coefficient */
#define LM628_LOAD_KI 0x0004   /* load I-coefficient */
#define LM628_LOAD_KD 0x0002   /* load D-coefficient */
#define LM628_LOAD_IL 0x0001   /* load integration-limit */

/*------------------------------+
| trajectory control words      |
+------------------------------*/
#define LM628_FWDDIR 0x1000    /* forward direction */
#define LM628_VELMOD 0x0800    /* velocity mode */
#define LM628_STOPSM 0x0400    /* stop smoothly */
#define LM628_STOPAB 0x0200    /* stop abruptly */
#define LM628_MOTOFF 0x0100    /* turn motor off */
#define LM628_ACCLOA 0x0020    /* load acceleration data */
#define LM628_ACCREL 0x0010    /* acceleration data is relative */
#define LM628_VELLOA 0x0008    /* load velocy data */
#define LM628_VELREL 0x0004    /* velocity data is relative */
#define LM628_POSLOA 0x0002    /* load position data */
#define LM628_POSREL 0x0001    /* position data is relative */

/*--------------------------------------+
|   TYPDEFS                             |
+--------------------------------------*/
/* none */

/*--------------------------------------+
|   EXTERNALS                           |
+--------------------------------------*/
/* none */

/*--------------------------------------+
|   GLOBALS                             |
+--------------------------------------*/
/* none */

/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/
/* none */


#ifdef __cplusplus
	}
#endif

#endif	/* _LM628_H */
