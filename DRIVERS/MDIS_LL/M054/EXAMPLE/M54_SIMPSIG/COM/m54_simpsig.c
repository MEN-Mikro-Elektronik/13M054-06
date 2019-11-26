/****************************************************************************
 ************                                                    ************
 ************                   M54_SIMPSIG                      ************
 ************                                                    ************
 ****************************************************************************
 *  
 *       Author: kp
 *
 *  Description: Simple example program for the M54 driver using signals
 *
 *  Performs a simple motor movement using interrupts/signals.
 *  Move from the current position 400000 pulses forward.
 *
 *  For trajectory complete, positioning error and line break events, 
 *  an interrupt/signal is installed.
 *
 *  Uses hardcoded PID control, velocity and acceleration parameters 
 *  that should be adapted for the actual motor in use.
 *
 *  This program does not use the binary inputs, outputs and the 
 *  glitch detection of the M54.
 *                      
 *     Required: libraries: mdis_api, m54_api
 *     Switches: -
 *
 *---------------------------------------------------------------------------
 * Copyright 2001-2019, MEN Mikro Elektronik GmbH
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

#include <stdio.h>
#include <string.h>
#include <MEN/men_typs.h>
#include <MEN/mdis_api.h>
#include <MEN/usr_oss.h>
#include <MEN/m54_drv.h>
#include <MEN/m54_api.h>
#include <MEN/lm628.h>


/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/
#define CHK(expression) \
 if( !(expression)) {\
	 printf("Error during: %s file %s line %d\n", \
      #expression,__FILE__,__LINE__);\
      printf("MDIS Error %s\n",M_errstring(UOS_ErrnoGet()));\
     goto abort;\
 }

/*--------------------------------------+
|   GLOBALS                             |
+--------------------------------------*/
static MDIS_PATH G_path;
static volatile u_int32 G_intReason;

/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/
static void PrintError(char *info);
/* commented out since not used at the moment (call commented out)
*static void ShowIndications(void);
* commented out since not used at the moment (call commented out) */
static void ShowStatus( u_int32 status );

static void __MAPILIB SigHandler( u_int32 sigCode )
{
	u_int32 status;

	switch( sigCode ){
	case UOS_SIG_USR1:			/* LM628 signal */
	case UOS_SIG_USR2:			/* other signal (line break) */
		/* get reason for interrupt */
		M_getstat( G_path, M54_LM_INT_REASON, (int32 *)&status );
		G_intReason |= status;
		break;
	}
}

/********************************* main *************************************
 *
 *  Description: Program main function
 *
 *---------------------------------------------------------------------------
 *  Input......: argc,argv	argument counter, data ..
 *  Output.....: return	    success (0) or error (1)
 *  Globals....: -
 ****************************************************************************/
int main(int argc, char *argv[])
{
	char *device;

	if (argc < 2 || strcmp(argv[1],"-?")==0) {
		printf("Syntax: m54_simpsig <device> <chan>\n");
		printf("Function: M54 example for simple motor movement with signals\n");
		printf("Options:\n");
		printf("    device       device name\n");
		printf("\n");
		return(1);
	}

	device = argv[1];

	G_intReason = 0;

	/*--------------------+
	|  open path          |
	+--------------------*/
	if ((G_path = M_open(device)) < 0) {
		PrintError("open");
		return(1);
	}

	/*------------------------+
	|  Get ready for signals  |
	+------------------------*/
	UOS_SigInit(SigHandler);
	UOS_SigInstall( UOS_SIG_USR1 );
	UOS_SigInstall( UOS_SIG_USR2 );
	UOS_SigMask();

	/* globally enable interrupts on M54 */
	CHK( M_setstat( G_path, M_MK_IRQ_ENABLE, TRUE ) == 0 );

	/* install signal for LM628 interrupts */
	CHK( M_setstat( G_path, M54_SETSIG_LM628, UOS_SIG_USR1 ) == 0 );

	/* reset pending status bits */
	CHK( M54_LmResetInterrupts( G_path, 
								~(LM628_TRAJEC|LM628_POSERR|LM628_BRKPNT))==0);

	/* unmask interrupts for trajectory complete/positioning error */
	CHK( M54_LmMaskInterrupts( G_path, 
							   LM628_TRAJEC|LM628_POSERR) == 0 );

	/* activate line break interrupts */
	CHK( M_setstat( G_path, M54_SETSIG_LINE_BREAK, UOS_SIG_USR2 ) == 0 );

	/*--------------------+
	|  config             |
	+--------------------*/

	printf("Load Filter Parameters\n");
	CHK( M54_LmLoadFilterParameters( G_path,
								LM628_LOAD_KP | LM628_LOAD_KI | LM628_LOAD_KD |
								LM628_LOAD_IL,
                   0x0170, /* kp */
                   0x0322, /* ki */
                   0x0550, /* kd */
                   0x1510, /* il */
                   6)      /* si */
		 >=0);

	CHK( M54_LmUpdateFilter(G_path) == 0); /* update filter */

	printf("Load Position Error for stop\n");
	CHK( M54_LmLoadPositionErrorForStop( G_path, 0x100 ) >= 0 );

	printf("Load Trajectory Parameters\n");
	CHK( M54_LmLoadTrajectoryParameters( G_path,
										 LM628_ACCLOA | LM628_VELLOA | 
										 LM628_POSLOA,
										 0x00000100,      /* acceleration */
										 21L<<16,         /* velocity */
										 400000)          /* position */
		 >=0 );

	/*------------------+
	|  Start motion     |
	+------------------*/
	printf("Start motion\n");
	CHK( M54_LmStartMotion( G_path ) >= 0);

	printf("Waiting for signals...\n");
	UOS_SigUnMask();

	while( 1 ){
		UOS_Delay(100);

		UOS_SigMask();
		if( G_intReason ){
			u_int32 status = G_intReason;
			G_intReason = 0;

			UOS_SigUnMask();

			printf("Signal received due to: "); 
			ShowStatus(status);
			printf("\n");

			if( status & LM628_TRAJEC ){
				printf("trajectory complete\n");
				break;
			}

		}
		UOS_SigUnMask();
		/*ShowIndications();*/
	}

	/*--------------------+
	|  cleanup            |
	+--------------------*/
	abort:
	CHK( M_setstat( G_path, M54_CLRSIG_LINE_BREAK, 0 ) == 0 );
	CHK( M54_LmMaskInterrupts( G_path, 0 ) == 0 );
	CHK( M_setstat( G_path, M54_CLRSIG_LM628, 0 ) == 0 );

	if (M_close(G_path) < 0)
		PrintError("close");

	UOS_SigRemove( UOS_SIG_USR2 );
	UOS_SigRemove( UOS_SIG_USR1 );
	UOS_SigExit();

	return(0);
}


/********************************* ShowStatus ********************************
 *
 *  Description: Display LM628 given status on stdout
 *---------------------------------------------------------------------------
 *  Input......: status		status from LM628
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void ShowStatus( u_int32 status )
{
	if( status & LM628_BRKPNT )
		printf("[BreakpointReached] ");
	if( status & LM628_POSERR )
		printf("[PositionError] ");
	if( status & LM628_WRAPAR )
		printf("[WraparoundObserved] ");
	if( status & LM628_INDEXP )
		printf("[IndexObserved] ");
	if( status & LM628_TRAJEC )
		printf("[TrajectoryComplete] ");
	if( status & LM628_CMDERR )
		printf("[CommandError] ");
	if( status & M54_STAT_LINE_BREAK )
		printf("[LineBreak] ");
	if( status & M54_STAT_GLITCH )
		printf("[Glitch] ");
}

/********************************* ShowIndications ***************************
 *
 *  Description: Display LM628 status and counters
 *---------------------------------------------------------------------------
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
/* commented out since not used at the moment (call commented out)
*  prototype commented out as well
*static void ShowIndications(void)
*{
*	u_int16 sigs;
*	int32 idxPos, desPos, realPos, desVel;
*	int16 realVel;
*	u_int32 status;
*
*	printf("---------------------------------------------\n");
*	CHK( M54_LmReadStatus( G_path, &status ) >= 0 );
*	CHK( M54_LmReadSignalsRegister( G_path, &sigs) >= 0 );
*	CHK( M54_LmReadIndexPosition( G_path, &idxPos ) >= 0 );
*	CHK( M54_LmReadDesiredPosition( G_path, &desPos ) >= 0 );
*	CHK( M54_LmReadRealPosition( G_path, &realPos ) >= 0 );
*	CHK( M54_LmReadDesiredVelocity( G_path, &desVel ) >= 0 );
*	CHK( M54_LmReadRealVelocity( G_path, &realVel ) >= 0 );
*
*	printf(" Status          : 0x%04x\n", (int)status );
*	printf("   ");
*	ShowStatus( status );
*	printf("\n");
*	printf(" Signals         : 0x%04x\n", sigs );
*	printf(" Index Position  : %d\n", (int)idxPos );
*	printf(" Desired Position: %d\n", (int)desPos );
*	printf(" Real Position   : %d\n", (int)realPos );
*	printf(" Desired Velocity: %f\n", (double)desVel/65536.0 );
*	printf(" Real Velocity   : %d\n", realVel );
*
* abort:
*	return;
*}
* commented out since not used at the moment (call commented out) */

/********************************* PrintError *******************************
 *
 *  Description: Print MDIS error message
 *			   
 *---------------------------------------------------------------------------
 *  Input......: info	info string
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void PrintError(char *info)
{
	printf("*** can't %s: %s\n", info, M_errstring(UOS_ErrnoGet()));
}
