/****************************************************************************
 ************                                                    ************
 ************                    M54_SIMP                        ************
 ************                                                    ************
 ****************************************************************************
 *
 *       Author: kp
 *
 *  Description: Simple example program for the M54 driver
 *
 *  Performs a simple motor movement without using interrupts/signals.
 *  Move from the current position 400000 pulses forward.
 *  During movement, all indications of the LM628 are displayed.
 *
 *  Uses hardcoded PID control, velocity and acceleration parameters 
 *  that should be adapted for the actual motor in use.
 *
 *  This program does not use the binary inputs, outputs and the line break
 *  and glitch detection of the M54.
 *
 *     Required: libraries: mdis_api, m54_api
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

#include <stdio.h>
#include <string.h>
#include <MEN/men_typs.h>
#include <MEN/mdis_api.h>
#include <MEN/usr_oss.h>
#include <MEN/m54_drv.h>
#include <MEN/m54_api.h>
#include <MEN/lm628.h>

static const char IdentString[]=MENT_XSTR(MAK_REVISION);

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
|   PROTOTYPES                          |
+--------------------------------------*/
static void PrintError(char *info);
static void ShowIndications( MDIS_PATH path );

/********************************* main *************************************
 *
 *  Description: Program main function
 *
 *---------------------------------------------------------------------------
 *  Input......: argc,argv  argument counter, data ..
 *  Output.....: return     success (0) or error (1)
 *  Globals....: -
 ****************************************************************************/
int main(int argc, char *argv[])
{
	MDIS_PATH path;
	char *device;
	u_int32 status;

	if (argc < 2 || strcmp(argv[1],"-?")==0) {
		printf("Syntax: m54_simp <device> <chan>\n");
		printf("Function: M54 example for simple motor movement\n");
		printf("Options:\n");
		printf("    device       device name\n");
		printf("\n");
		return(1);
	}

	device = argv[1];

	/*--------------------+
	|  open path          |
	+--------------------*/
	if ((path = M_open(device)) < 0) {
		PrintError("open");
		return(1);
	}

	/*--------------------+
	|  config             |
	+--------------------*/
	printf("Load Filter Parameters\n");
	CHK( M54_LmLoadFilterParameters( path, 
								LM628_LOAD_KP | LM628_LOAD_KI | LM628_LOAD_KD |
								LM628_LOAD_IL,
                   0x0170, /* kp */
                   0x0322, /* ki */
                   0x0550, /* kd */
                   0x1510, /* il */
                   6)	   /* si */
		 >=0);

	CHK( M54_LmUpdateFilter(path) == 0); /* update filter */

	printf("Status before movement:\n");
	ShowIndications(path);

	printf("Load Position Error for interrupt\n");
	CHK( M54_LmLoadPositionErrorForInt( path, 0x7f00 ) >= 0 );

	printf("Load Trajectory Parameters\n");
	CHK( M54_LmLoadTrajectoryParameters( path,
										 LM628_ACCLOA | LM628_VELLOA | 
										 LM628_POSLOA,
										 0x00000100,      /* acceleration */
										 21L<<16,      	  /* velocity */
										 400000)      	  /* position */
		 >=0 );

	/*------------------+
	|  Start motion     |
	+------------------*/
	printf("Start motion\n");
	CHK( M54_LmStartMotion( path ) >= 0);

	while( 1 ){
		CHK( M54_LmReadStatus( path, &status ) == 0 );
		if( status & LM628_TRAJEC ){
			printf("\nTrajectory complete\n");
			break;
		}

		ShowIndications(path);
		UOS_Delay(3000);
	}

	/*--------------------+
	|  cleanup            |
	+--------------------*/
	abort:
	if (M_close(path) < 0)
		PrintError("close");

	return(0);
}

/**************************** ShowIndications *******************************
 *
 *  Description: Display LM628 status and counters
 *---------------------------------------------------------------------------
 *  Input......: path  M54 MDIS path
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void ShowIndications( MDIS_PATH path )
{
	u_int16 sigs;
	int32 idxPos, desPos, realPos, desVel;
	int16 realVel;
	u_int32 status;

	printf("---------------------------------------------\n");
	CHK( M54_LmReadStatus( path, &status ) >= 0 );
	CHK( M54_LmReadSignalsRegister( path, &sigs) >= 0 );
	CHK( M54_LmReadIndexPosition( path, &idxPos ) >= 0 );
	CHK( M54_LmReadDesiredPosition( path, &desPos ) >= 0 );
	CHK( M54_LmReadRealPosition( path, &realPos ) >= 0 );
	CHK( M54_LmReadDesiredVelocity( path, &desVel ) >= 0 );
	CHK( M54_LmReadRealVelocity( path, &realVel ) >= 0 );

	printf(" Status          : 0x%04x\n", (int)status );
	printf("   ");
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
	printf("\n");
	printf(" Signals         : 0x%04x\n", sigs );
	printf(" Index Position  : %d\n", (int)idxPos );
	printf(" Desired Position: %d\n", (int)desPos );
	printf(" Real Position   : %d\n", (int)realPos );
	printf(" Desired Velocity: %f\n", (double)desVel/65536.0 );
	printf(" Real Velocity   : %d\n", realVel );

 abort:
	return;
}

/********************************* PrintError *******************************
 *
 *  Description: Print MDIS error message
 *
 *---------------------------------------------------------------------------
 *  Input......: info  info string
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void PrintError(char *info)
{
	printf("*** can't %s: %s\n", info, M_errstring(UOS_ErrnoGet()));
}
