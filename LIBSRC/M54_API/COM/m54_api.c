/*********************  P r o g r a m  -  M o d u l e ***********************
 *  
 *         Name: m54_api.c
 *      Project: M54 MDIS5 driver
 *
 *       Author: kp
 *
 *  Description: Application interface to perform LM628 commands
 *                      
 *  Note that most of the comments were copied 1:1 from the LM628 manual.
 *
 *     Required: M54 MDIS5 LL driver
 *     Switches: -
 *
 *---------------------------[ Public Functions ]----------------------------
 *  
 *  
 *---------------------------------------------------------------------------
 * Copyright (c) 2001-2019, MEN Mikro Elektronik GmbH
 ****************************************************************************/
/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
 
#include <MEN/men_typs.h>
#include <MEN/mdis_err.h>
#include <MEN/mdis_api.h>
#include <MEN/m54_api.h>
#include <MEN/lm628.h>
#include <MEN/m54_drv.h>


/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/
/* flags for command direction */
#define CMD_READ   	0
#define CMD_WRITE	1

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

/********************************* Lm628Exec *********************************
 *
 *  Description: Execute any LM628 command
 *			   
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *				 dir		direction of data (CMD_WRITE, CMD_READ)
 *				 cblk		pointer to M54_LM628_CMDBLOCK structure
 *				 dataLen 	number of 16-bit cblk.data words to send/receive
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
static int32 Lm628Exec(			/* nodoc */
	MDIS_PATH path,
	int32 dir,
	M54_LM628_CMDBLOCK *cblk,
	int32 dataLen)
{
	M_SG_BLOCK blk;
	int32 rv;

	blk.size = (dataLen + 1) << 1;
	blk.data = (void *)cblk;

	if( dir == CMD_READ )
		rv = M_getstat( path, M54_LM628_PERFORM_CMD, (int32 *)&blk );
	else
		rv = M_setstat( path, M54_LM628_PERFORM_CMD, (INT32_OR_64)&blk );

	return rv;
}

/********************************* M54_LmReset *******************************
 *
 *  Description: Perform LM628 RESET and PORT12 command
 *			   
 * This command results in setting the following data items to zero: 
 * filter coefficients and their input buffers, trajectory parameters and
 * their input buffers, and the motor control output. A zero motor
 * control output is a half-scale, offset-binary code: (0x80 for the
 * 8-bit output mode; 0x800 for 12-bit mode). During reset, the DAC
 * port outputs 0x800 to "zero" a 12-bit DAC and reverts to
 * 0x80 to "zero" an 8-bit DAC. The command also clears five
 * of the six interrupt masks (only the SBPA/SBPR interrupt is
 * masked), sets the output port size to 8 bits, and defines the
 * current absolute position as home.
 * Reset, which may be executed at any time, will be completed in less
 * than 1.5 ms. 
 * 
 * The M54_LmReset() function also issues the PORT12 command after
 * the RESET command, since the M54 uses a 12-bit DAC.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmReset( MDIS_PATH path )
{
	M54_LM628_CMDBLOCK cblk;
	int32 rv;

	cblk.opcode = LM628_RESET;

	if( (rv = Lm628Exec( path, CMD_WRITE, &cblk, 0 )) < 0 )
		return rv;
	
	cblk.opcode = LM628_PORT12;

	return Lm628Exec( path, CMD_WRITE, &cblk, 0 );
}

/********************************* M54_LmDefineHome **************************
 *
 *  Description: Perform LM628 DFH command
 *			   
 * This command declares the current position as "home", or
 * absolute position 0 (Zero). If DFH is executed during motion
 * it will not affect the stopping position of the on-going move
 * unless command STT is also executed.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmDefineHome(	MDIS_PATH path )
{
	M54_LM628_CMDBLOCK cblk;

	cblk.opcode = LM628_DFH;

	return Lm628Exec( path, CMD_WRITE, &cblk, 0 );
}

/********************************* M54_LmSetIndexPosition ********************
 *
 *  Description: Perform LM628 SIP command
 *			   
 * After this command is executed, the absolute position which
 * corresponds to the occurrence of the next index pulse input
 * will be recorded in the index register, and bit 3 of the status
 * byte will be set to logic high. The position is recorded when
 * both encoder-phase inputs and the index pulse input are
 * logic low. This register can then be read by the user (see
 * description for command RDIP) to facilitate aligning the defi-
 * nition of home position (see description of command DFH)
 * with an index pulse. The user can also arrange to have the
 * LM628 interrupt the host to signify that an index pulse has
 * occurred. See the descriptions for commands MSKI and RSTI.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmSetIndexPosition( MDIS_PATH path )
{
	M54_LM628_CMDBLOCK cblk;

	cblk.opcode = LM628_SIP;

	return Lm628Exec( path, CMD_WRITE, &cblk, 0 );
}

/********************************* M54_LmLoadPositionErrorForInt ************
 *
 *  Description: Perform LM628 LPEI command
 *	
 * An excessive position error (the output of the loop summing
 * junction) can indicate a serious system problem; e.g., a
 * stalled rotor. Instruction LPEI allows the user to input a
 * threshold for position error detection. Error detection occurs
 * when the absolute magnitude of the position error exceeds
 * the threshold, which results in bit 5 of the status byte being
 * set to logic high. If it is desired to also stop (turn off) the
 * motor upon detecting excessive position error, see command
 * LPES, below. The user can have the LM628 interrupt the host to
 * signify that an excessive position error has occurred. See the
 * descriptions for commands MSKI and RSTI. 		   
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *				 thres		position error threshold (0x0000..0x7FFF)
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmLoadPositionErrorForInt( MDIS_PATH path, u_int16 thres )
{
	M54_LM628_CMDBLOCK cblk;

	cblk.opcode = LM628_LPEI;
	cblk.data[0] = thres;

	return Lm628Exec( path, CMD_WRITE, &cblk, 1 );
}

/********************************* M54_LmLoadPositionErrorForStop ************
 *
 *  Description: Perform LM628 LPES command
 *
 * Instruction LPES is essentially the same as command LPEI
 * above, but adds the feature of turning off the motor upon
 * detecting excessive position error. The motor drive is not
 * actually switched off, it is set to half-scale, the offset-binary
 * code for zero. As with command LPEI, bit 5 of the status
 * byte is also set to logic high. The user can have the LM628
 * interrupt the host to signify that an excessive position error
 * has occurred. See the descriptions for commands MSKI and RSTI. 			   
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *				 thres		position error threshold (0x0000..0x7FFF)
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmLoadPositionErrorForStop( MDIS_PATH path, u_int16 thres )
{
	M54_LM628_CMDBLOCK cblk;

	cblk.opcode = LM628_LPES;
	cblk.data[0] = thres;

	return Lm628Exec( path, CMD_WRITE, &cblk, 1 );
}

/********************************* M54_LmSetBreakpointAbsolute **************
 *
 *  Description: Perform LM628 SBPA command
 * 			   
 * This command enables the user to set a breakpoint in terms
 * of absolute position. Bit 6 (LM628_BRKPNT) of the status byte is set 
 * to logic high when the breakpoint position is reached. This condition
 * is useful for signaling trajectory and/or filter parameter updates
 * The user can also arrange to have the LM628 interrupt the host to 
 * signify that a breakpoint position has been reached. See the 
 * descriptions for commands MSKI and RSTI.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *				 bkpoint	breakpoint position (-0x40000000..0x3FFFFFFF)
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmSetBreakpointAbsolute( MDIS_PATH path, int32 bkpoint )
{
	M54_LM628_CMDBLOCK cblk;

	cblk.opcode = LM628_SBPA;

	cblk.data[0] = (u_int16)(bkpoint >> 16);
	cblk.data[1] = (u_int16)(bkpoint & 0xffff);

	return Lm628Exec( path, CMD_WRITE, &cblk, 2 );
}

/********************************* M54_LmSetBreakpointRelative **************
 *
 *  Description: Perform LM628 SBPA command
 *	
 * This command enables the user to set a breakpoint in terms of
 * relative position. As with command SBPA, bit 6 (LM628_BRKPNT) of the
 * status byte is set to logic high when the breakpoint position
 * (relative to the current commanded target position) is
 * reached. The relative breakpoint input value must be such that
 * when this value is added to the target position the result remains
 * within the absolute position range of the system (-0x40000000 to 
 * 0x3FFFFFFF).
 * This condition is useful for signaling trajectory and/or filter parameter
 * updates. The user can also arrange to have the LM628 interrupt the host
 * to signify that a breakpoint position has been reached. See the
 * descriptions for commands MSKI and RSTI.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *				 bkpoint	relative breakpoint position
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmSetBreakpointRelative( MDIS_PATH path, int32 bkpoint )
{
	M54_LM628_CMDBLOCK cblk;

	cblk.opcode = LM628_SBPR;

	cblk.data[0] = (u_int16)(bkpoint >> 16);
	cblk.data[1] = (u_int16)(bkpoint & 0xffff);

	return Lm628Exec( path, CMD_WRITE, &cblk, 2 );
}

/********************************* M54_LmMaskInterrupts *********************
 *
 *  Description: Perform LM628 MSKI command
 *
 * The MSKI command lets the user determine which potential
 * interrupt condition(s) will interrupt the host. Bits 1 through 6
 * of the status byte are indicators of the six conditions which
 * are candidates for host interrupt(s). When interrupted, the
 * host then reads the status byte to learn which condition(s)
 * occurred. 
 * 
 * The <mask> paramter is encoded as follows:
 * Bits 1 through 6 (LM628_BKPNT..LM628_CMDERR) determine the masked/
 * unmasked status of each potential interrupt. Any zero(s) in this
 * 6-bit field will mask the corresponding interrupt(s); any one(s)
 * enable the interrupt(s). Other bits comprising <mask> have no
 * effect. The mask controls only the host interrupt process;
 * reading the status byte will still reflect the actual conditions
 * independent of the mask byte. 
 *			
 * In order to enable LM628 interrupts on the M54, setstat M_MK_IRQ_ENABLE
 * needs to be called too.
 *
 * Once an interrupt has been enabled for a condition, the corresponding
 * status bit may no longer show up when polling the device with 
 * M54_LmReadStatus(), since the driver internally resets the interrupt.
 * Instead, you need to call getstat M54_LM_INT_REASON for those status
 * bits.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *				 mask		interrupt mask (1=enable 0=disable)
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmMaskInterrupts( MDIS_PATH path, u_int16 mask )
{
	M54_LM628_CMDBLOCK cblk;

	cblk.opcode = LM628_MSKI;
	cblk.data[0] = mask;

	return Lm628Exec( path, CMD_WRITE, &cblk, 1 );
}

/********************************* M54_LmResetInterrupts *********************
 *
 *  Description: Perform LM628 RSTI command
 * 
 * When one of the potential interrupt conditions occurs, command
 * RSTI is used to reset the corresponding interrupt flag bit
 * in the status byte. The host may reset one or all flag bits.
 * Resetting them one at a time allows the host to service them one
 * at a time according to a priority programmed by the user. As in
 * the MSKI command, bits 1 through 6 in <mask> correspond to the
 * potential interrupt conditions. Also see description of RDSTAT command.
 *
 * CAUTION: Any zero(s) in <mask> reset the corresponding interrupt(s). 
 * Bits set to 1 leave the corresponding interrupt flags unchanged.
 *		
 * Be careful when resetting status bits for which interrupts have been 
 * enabled. Clearing those bits may produce spurious interrupts.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *				 mask		interrupt reset mask (1=reset 0=keep)
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmResetInterrupts( MDIS_PATH path, u_int16 mask )
{
	M54_LM628_CMDBLOCK cblk;

	cblk.opcode = LM628_RSTI;
	cblk.data[0] = mask;

	return Lm628Exec( path, CMD_WRITE, &cblk, 1 );
}

/********************************* M54_LmLoadFilterParameters ***************
 *
 *  Description: Perform LM628 LFIL command
 * 			   
 * The filter parameters (coefficients) which are written to the
 * LM628 to control loop compensation are: kp, ki, kd, and il
 * (integration limit). The integration limit (il) constrains the
 * contribution of the integration term. See the LM628 manual for more
 * info.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *				 control	control word (ORed bit masks, see lm628.h)
 *				 kp			proportional coefficient (used only when
 *							LM628_LOAD_KP is set in <control>)
 *				 ki			integral coefficient (used only when
 *							LM628_LOAD_KI is set in <control>)
 *				 kd			differential coefficient (used only when
 *							LM628_LOAD_KD is set in <control>)
 *				 il			integration limit (used only when
 *							LM628_LOAD_IL is set in <control>)
 *				 si 		derivate sampling interval. Will be ORed into
 *							high word of <control>
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmLoadFilterParameters(
	MDIS_PATH path,
	u_int16 control,
	u_int16 kp,
	u_int16 ki,
	u_int16 kd,
	u_int16 il,
	u_int8  si)
{
	M54_LM628_CMDBLOCK cblk;
	int32 dataLen = 1;
	u_int16 *dataP = cblk.data;

	cblk.opcode = LM628_LFIL;
	*dataP++ 	= (control & 0xff) | ((u_int16)si<<8);

	if( control & LM628_LOAD_KP ){
		*dataP++ = kp;
		dataLen++;
	}
	if( control & LM628_LOAD_KI ){
		*dataP++ = ki;
		dataLen++;
	}
	if( control & LM628_LOAD_KD ){
		*dataP++ = kd;
		dataLen++;
	}
	if( control & LM628_LOAD_IL ){
		*dataP++ = il;
		dataLen++;
	}
	return Lm628Exec( path, CMD_WRITE, &cblk, dataLen );	
}

/********************************* M54_LmUpdateFilter ************************
 *
 *  Description: Perform LM628 UDF command
 * 
 * The UDF command is used to update the filter parameters, the
 * specifics of which have been programmed via the LFIL command.
 * Any or all parameters (derivative-term sampling interval, kp,
 * ki, kd, and/or il) may be changed by the appropriate command(s),
 * but command UDF must be executed to affect the change in filter
 * tuning. Filter updating is synchronized with the calculations
 * to eliminate erratic or spurious behavior.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmUpdateFilter( MDIS_PATH path )
{
	M54_LM628_CMDBLOCK cblk;

	cblk.opcode = LM628_UDF;

	return Lm628Exec( path, CMD_WRITE, &cblk, 0 );
}

/********************************* M54_LmLoadTrajectoryParameters ************
 *
 *  Description: Perform LM628 LTRJ command
 *  		
 * The trajectory control parameters which are written to the
 * LM628 to control motion are: 
 *  - acceleration,
 *  - velocity,
 *  - position. 
 * 
 * In addition, indications as to whether these three parameters are
 * to be considered as absolute or relative inputs, selection of
 * velocity mode and direction, and manual stopping mode selection
 * and execution are programmable via this command. 
 * 
 * The <control> word specifies which parameter(s) is/are being
 * changed and is encoded as follows. It can be compromised by
 * ORing LM628_xxx flags together as specified in lm682.h:
 * 
 * Bit 12 (LM628_FWDDIR) Forward Direction (Velocity Mode Only)
 * Bit 11 (LM628_VELMOD) Velocity Mode
 * Bit 10 (LM628_STOPSM) Stop Smoothly (Decelerate as Programmed)
 * Bit  9 (LM628_STOPAB) Stop Abruptly (Maximum Deceleration)
 * Bit  8 (LM628_MOTOFF) Turn Off Motor (Output Zero Drive)
 * Bit  5 (LM628_ACCLOA) Acceleration Will Be Loaded
 * Bit  4 (LM628_ACCREL) Acceleration Data Is Relative
 * Bit  3 (LM628_VELLOA) Velocity Will Be Loaded
 * Bit  2 (LM628_VELREL) Velocity Data Is Relative
 * Bit  1 (LM628_POSLOA) Position Will Be Loaded
 * Bit  0 (LM628_POSREL) Position Data Is Relative
 * 
 * Bit 12 determines the motor direction when in the velocity
 * mode. A logic one indicates forward direction. This bit has
 * no effect when in position mode.
 * 
 * Bit 11 determines whether the LM628 operates in velocity
 * mode (Bit 11 logic one) or position mode (Bit 11 logic zero).
 * 
 * Bits 8 through 10 are used to select the method of manually
 * stopping the motor. These bits are not provided for one to
 * merely specify the desired mode of stopping, in position
 * mode operations, normal stopping is always smooth and
 * occurs automatically at the end of the specified trajectory.
 * Under exceptional circumstances it may be desired to manually
 * intervene with the trajectory generation process to affect a
 * premature stop. In velocity mode operations, however, the
 * normal means of stopping is via bits 8 through 10 (usually bit
 * 10). Bit 8 is set to logic one to stop the motor by turning off
 * motor drive output (outputting the appropriate offset-binary
 * code to apply zero drive to the motor); bit 9 is set to one to
 * stop the motor abruptly (at maximum available acceleration, by
 * setting the target position equal to the current position); and
 * bit 10 is set to one to stop the motor smoothly by using the
 * current user-programmed acceleration value. Bits 8 through 10
 * are to be used exclusively; only one bit should be a logic one
 * at any time.
 * 
 * Bits 0 through 5 inform the LM628 as to whether any or all of
 * the trajectory controlling parameters are about to be written,
 * and whether the data should be interpreted as absolute or
 * relative. The user may choose to update any or all (or none
 * of the trajectory parameters. Those chosen for updating are
 * so indicated by logic one(s) in the corresponding bit
 * position(s). Any parameter may be changed while the motor
 * is in motion; however, if acceleration is changed then the
 * next STT command must not be issued until the LM628 has
 * completed the current move or has been manually stopped.
 * 
 * Acceleration <acc> and velocity <vel> are 32 bits, positive only, but
 * range only from 0 (0x00000000) to (0x3FFFFFFF). The bottom 16 bits
 * of both acceleration and velocity are scaled as fractional data;
 * therefore, the least-significant integer data bit for these
 * parameters is bit 16 (where the bits are numbered 0 through 31).
 * To determine the coding for a given velocity, for example, one
 * multiplies the desired velocity (in counts per sample interval)
 * times 65,536 and converts the result to binary. The units of
 * acceleration are counts per sample. The value loaded for
 * acceleration must not exceed the value loaded for velocity. 
 * 
 * Position <pos> is a signed, 32-bit integer, but ranges only from 
 * -0x40000000 to 0x3FFFFFFF.
 * 
 * The required data is written to the primary buffers of a double-
 * buffered scheme by the above described operations; it is not
 * transferred to the secondary (working) registers until the STT
 * command is executed. This fact can be used advantageously; the
 * user can input numerous data ahead of their actual use. This
 * simple pipeline effect can relieve potential host computer data
 * communications bottlenecks, and facilitates easier synchronization
 * of multiple-axis controls.
 *			   
 * In order to activate the parameters, you must call M54_LmStartMotion!
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *				 control	control word (ORed bit masks, see lm628.h)
 *				 acc		acceleration parameter (used only when
 *							LM628_ACCLOA is set in <control>)
 *				 vel		velocity parameter (used only when
 *							LM628_VELLOA is set in <control>)
 *				 pos		position parameter (used only when
 *							LM628_POSLOA is set in <control>)
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmLoadTrajectoryParameters(
	MDIS_PATH path,
	u_int16 control,
	u_int32 acc,
	u_int32 vel,
	int32 pos)
{
	M54_LM628_CMDBLOCK cblk;
	int32 dataLen = 1;
	u_int16 *dataP = cblk.data;

	cblk.opcode = LM628_LTRJ;
	*dataP++ 	= control;

	if( control & LM628_ACCLOA ){
		*dataP++ = (u_int16)(acc >> 16);
		*dataP++ = (u_int16)(acc & 0xffff);
		dataLen+=2;
	}
	if( control & LM628_VELLOA ){
		*dataP++ = (u_int16)(vel >> 16);
		*dataP++ = (u_int16)(vel & 0xffff);
		dataLen+=2;
	}
	if( control & LM628_POSLOA ){
		*dataP++ = (u_int16)(pos >> 16);
		*dataP++ = (u_int16)(pos & 0xffff);
		dataLen+=2;
	}
	return Lm628Exec( path, CMD_WRITE, &cblk, dataLen );
}

/********************************* M54_LmStartMotion ************************
 *
 *  Description: Perform LM628 STT command
 *		
 * The STT command is used to execute the desired trajectory,
 * the specifics of which have been programmed via the LTRJ
 * command. Synchronization of multi-axis control (to within
 * one sample interval) can be arranged by loading the
 * required trajectory parameters for each (and every) axis and
 * then simultaneously issuing a single STT command to all axes.
 * This command may be executed at any time, unless the
 * acceleration value has been changed and a trajectory has not
 * been completed or the motor has not been manually stopped.
 * If STT is issued during motion and acceleration has been
 * changed, a command error interrupt will be generated and
 * the command will be ignored.	   
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmStartMotion( MDIS_PATH path )
{
	M54_LM628_CMDBLOCK cblk;

	cblk.opcode = LM628_STT;

	return Lm628Exec( path, CMD_WRITE, &cblk, 0 );
}

/********************************* M54_LmReadStatus **************************
 *
 *  Description: Read the LM628 status byte plus M54-specific status bits
 *			   
 * This function differs from the other M54_LmXXX functions in that it
 * does not issue an LM628 command, but reads the status register directly.
 *
 * In addition to the LM628 status byte, this command reports the M54-
 * specific status bits "Line break" and "Glitch".
 *
 * Bit 9 (M54_STAT_GLITCH)     Glitch detected by quadrature compare circuit
 * Bit 8 (M54_STAT_LINE_BREAK) Quadrature encoder line break detected		   
 * Bit 7 (LM628_MOTORF)        Motor is Off
 * Bit 6 (LM628_BRKPNT)        Breakpoint Reached 
 * Bit 5 (LM628_POSERR)        Excessive Position Error
 * Bit 4 (LM628_WRAPAR)        Wraparound Occurred
 * Bit 3 (LM628_INDEXP)        Index Pulse Acquired
 * Bit 2 (LM628_TRAJEC)        Trajectory Complete
 * Bit 1 (LM628_CMDERR)        Command Error
 * Bit 0 (LM628_BUSY)	       (this bit doesn't care in application context)
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *				 *statusP	bits from status register (see lm628.h)
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmReadStatus( MDIS_PATH path, u_int32 *statusP )
{
	return M_getstat( path, M54_LM_STATUS, (int32 *)statusP );
}


/********************************* M54_LmReadSignalsRegister ****************
 *
 *  Description: Perform LM628 RDSIGS command
 *			   
 * The LM628 internal "signals" register may be read using
 * this command. 
 * The less significant byte of this register (with the exception
 * of bit 0) duplicates the status byte.
 * 
 * Bit 15 Host Interrupt
 * Bit 14 Acceleration Loaded (But Not Updated)
 * Bit 13 UDF Executed (But Filter Not yet Updated)
 * Bit 12 Forward Direction
 * Bit 11 Velocity Mode
 * Bit 10 On Target
 * Bit 9 Turn Off upon Excessive Position Error
 * Bit 8 Eight-Bit Output Mode
 * Bit 7 Motor Off
 * Bit 6 Breakpoint Reached [Interrupt]
 * Bit 5 Excessive Position Error [Interrupt]
 * Bit 4 Wraparound Occurred [Interrupt]
 * Bit 3 Index Pulse Acquired [Interrupt]
 * Bit 2 Trajectory Complete [Interrupt]
 * Bit 1 Command Error [Interrupt]
 * Bit 0 Acquire Next Index (SIP Executed)
 * 
 * Bit 15, the host interrupt flag, is set to logic one when the
 * host interrupt output (Pin 17) is logic one. Pin 17 is set to
 * logic one when any of the six host interrupt conditions occur
 * (if the corresponding interrupt has not been masked). Bit 15
 * (and Pin 17) are cleared via command RSTI.
 * 
 * Bit 14, the acceleration-loaded flag, is set to logic one when
 * acceleration data is written to the LM628. Bit 14 is cleared
 * by the STT command.
 * 
 * Bit 13, the UDF-executed flag, is set to logic one when the
 * UDF command is executed. Because bit 13 is cleared at the
 * end of the sampling interval in which it has been set, this
 * signal is very short-lived and probably not very profitable for
 * monitoring.
 * 
 * Bit 12, the forward direction flag, is meaningful only when
 * the LM628 is in velocity mode. The bit is set to logic one to
 * indicate that the desired direction of motion is "forward";
 * zero indicates "reverse" direction. Bit 12 is set and cleared
 * via command LTRJ. The actual setting and clearing of bit 12
 * does not occur until command STT is executed.
 * 
 * Bit 11, the velocity mode flag, is set to logic one to indicate
 * that the user has selected (via command LTRJ) velocity
 * mode. Bit 11 is cleared when position mode is selected (via
 * command LTRJ). The actual setting and clearing of bit 11
 * does not occur until command STT is executed.
 * 
 * Bit 10, the on-target flag, is set to logic one when the trajectory
 * generator has completed its functions for the last-issued STT
 * command. Bit 10 is cleared by the next STT command.
 * 
 * Bit 9, the turn-off on-error flag, is set to logic one when
 * command LPES is executed. Bit 9 is cleared by command LPEI.
 * 
 * Bit 8, the 8-bit output flag, is set to logic one when the
 * LM628 is reset, or when command PORT8 is executed. Bit 8
 * is cleared by command PORT12.
 * 
 * Bits 0 through 7 replicate the status byte, with the exception
 * of bit 0. Bit 0, the acquire next index flag, is set to logic
 * one when command SIP is executed; it then remains set until
 * the next index pulse occurs. 
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *				 *sigsP		signals read from LM628
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmReadSignalsRegister( MDIS_PATH path, u_int16 *sigsP)
{
	M54_LM628_CMDBLOCK cblk;
	int32 rv;

	cblk.opcode = LM628_RDSIGS;

	rv = Lm628Exec( path, CMD_READ, &cblk, 1 );
	if( rv == ERR_SUCCESS )
		*sigsP = cblk.data[0];
		
	return rv;
}

/********************************* M54_LmReadIndexPosition *******************
 *
 *  Description: Perform LM628 RDIP command
 * 
 * This command reads the position recorded in the index register
 * Reading the index register can be part of a system error
 * checking scheme. Whenever the SIP command is executed, the new
 * index position minus the old index position, divided by the
 * incremental encoder resolution (encoder lines times four),
 * should always be an integral number. The RDIP command facilitates
 * acquiring these data for host-based calculations. The command
 * can also be used to identify/verify home or some other special
 * position. 
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *				 *posP		position read from LM628
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmReadIndexPosition(
	MDIS_PATH path,
	int32 *posP)
{
	M54_LM628_CMDBLOCK cblk;
	int32 rv;

	cblk.opcode = LM628_RDIP;

	rv = Lm628Exec( path, CMD_READ, &cblk, 2 );
	if( rv == ERR_SUCCESS )
		*posP = (int32)(((u_int32)cblk.data[0]<<16) | cblk.data[1]);
		
	return rv;
}

/********************************* M54_LmReadDesiredPosition *****************
 *
 *  Description: Perform LM628 RDDP command
 *			   
 * This command reads the instantaneous desired (current temporal)
 * position output of the profile generator. This is the "setpoint"
 * input to the position-loop summing junction.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *				 *posP		position read from LM628
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmReadDesiredPosition(
	MDIS_PATH path,
	int32 *posP)
{
	M54_LM628_CMDBLOCK cblk;
	int32 rv;

	cblk.opcode = LM628_RDDP;

	rv = Lm628Exec( path, CMD_READ, &cblk, 2 );
	if( rv == ERR_SUCCESS )
		*posP = (int32)(((u_int32)cblk.data[0]<<16) | cblk.data[1]);
		
	return rv;
}


/********************************* M54_LmReadRealPosition *****************
 *
 *  Description: Perform LM628 RDRP command
 *	
 * This command reads the current actual position of the motor.
 * This is the feedback input to the loop summing junction.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *				 *posP		position read from LM628
 *  Globals....: -
 ****************************************************************************/
int32 M54_LmReadRealPosition(
	MDIS_PATH path,
	int32 *posP)
{
	M54_LM628_CMDBLOCK cblk;
	int32 rv;

	cblk.opcode = LM628_RDRP;

	rv = Lm628Exec( path, CMD_READ, &cblk, 2 );
	if( rv == ERR_SUCCESS )
		*posP = (int32)(((u_int32)cblk.data[0]<<16) | cblk.data[1]);
		
	return rv;
}


/********************************* M54_LmReadDesiredVelocity *****************
 *
 *  Description: Perform LM628 RDDV command
 *
 * This command reads the integer and fractional portions of the
 * instantaneous desired (current temporal) velocity, as used to
 * generate the desired position profile. The value read is properly
 * scaled for numerical comparison with the user-supplied (commanded)
 * velocity; however, because the two least-significant bytes represent
 * fractional velocity, only the two most-significant bytes are
 * appropriate for comparison with the data obtained via command RDRV
 * (see below).
 * Also note that, although the velocity input data is constrained to
 * positive numbers (see command LTRJ), the data returned by command
 * RDDV represents a signed quantity where negative numbers represent
 * operation in the reverse direction.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *				 *velP		velocity read from LM628
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmReadDesiredVelocity(
	MDIS_PATH path,
	int32 *velP)
{
	M54_LM628_CMDBLOCK cblk;
	int32 rv;

	cblk.opcode = LM628_RDDV;

	rv = Lm628Exec( path, CMD_READ, &cblk, 2 );
	if( rv == ERR_SUCCESS )
		*velP = (int32)(((u_int32)cblk.data[0]<<16) | cblk.data[1]);
		
	return rv;
}

/********************************* M54_LmReadRealVelocity *****************
 *
 *  Description: Perform LM628 RDRV command
 *	
 * This command reads the integer portion of the instantaneous actual
 * velocity of the motor. The internally maintained fractional portion
 * of velocity is not reported because the reported data is derived
 * by reading the incremental encoder, which produces only integer data.
 * For comparison with the result obtained by executing command RDDV (or
 * the user-supplied input value), the value returned by command RDRV
 * must be multiplied by 65536. 
 * Also, as with command RDDV above, data returned by command RDRV is a
 * signed quantity, with negative values representing reverse-direction
 * motion.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *				 *velP		velocity read from LM628
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmReadRealVelocity(
	MDIS_PATH path,
	int16 *velP)
{
	M54_LM628_CMDBLOCK cblk;
	int32 rv;

	cblk.opcode = LM628_RDRV;

	rv = Lm628Exec( path, CMD_READ, &cblk, 1 );
	if( rv == ERR_SUCCESS )
		*velP = (int16)cblk.data[0];
		
	return rv;
}

/********************************* M54_LmReadIntegrationSum ****************
 *
 *  Description: Perform LM628 RDSUM command
 *			   
 * This command reads the value to which the integration term has
 * accumulated. The ability to read this value may be helpful in
 * initially or adaptively tuning the system.
 *---------------------------------------------------------------------------
 *  Input......: path		MDIS path number for device	
 *  Output.....: returns:	0 = ok, <0=error (errno set)
 *				 *sumP		summation read from LM628
 *  Globals....: -
 ****************************************************************************/
int32 __MAPILIB M54_LmReadIntegrationSum( MDIS_PATH path, u_int16 *sumP)
{
	M54_LM628_CMDBLOCK cblk;
	int32 rv;

	cblk.opcode = LM628_RDSUM;

	rv = Lm628Exec( path, CMD_READ, &cblk, 1 );
	if( rv == ERR_SUCCESS )
		*sumP = cblk.data[0];
		
	return rv;
}
