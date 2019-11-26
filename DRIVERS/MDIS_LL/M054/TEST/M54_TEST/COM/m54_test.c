/****************************************************************************
 ************                                                    ************
 ************                   M54_TEST                         ************
 ************                                                    ************
 ****************************************************************************
 *
 *       Author: kp
 *
 *  Description: Test program to verify M54 MDIS5 driver and API
 *
 *  Covers the following items:
 *
 *  - All M54_API functions except 
 *     - M54_LmLoadPositionErrorForStop
 *     - M54_LmSetBreakpointRelative
 *     - M54_LmReadIntegrationSum
 *
 *  - Test movements (use interrupts for trajectory complete and breakpoint)
 *
 *  - Test M54 line break and glitch detection
 *
 *  - Test M54 polarity inversion
 *
 *  - M54 features not tested by this program:
 *    - binary inputs and ouputs
 *
 *     Required:
 *          - A_1120 test adapter with motor A_1130
 *          - Motor must provide index pulse
 *
 *          libraries: mdis_api, m54_api
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
#include <stdlib.h>
#include <stdarg.h>
#include <MEN/men_typs.h>
#include <MEN/mdis_api.h>
#include <MEN/usr_oss.h>
#include <MEN/usr_utl.h>
#include <MEN/m54_drv.h>
#include <MEN/m54_api.h>
#include <MEN/lm628.h>

static const char IdentString[]=MENT_XSTR(MAK_REVISION);

/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/
/* add some codes to workaround bug in glitch detect logic */
#define GLITCH_LOGIC_BUG_WORKAROUND

#define CHK(expression) \
 if(!(expression)) {\
     printf("\n*** Error during: %s\nfile %s\nline %d\n", \
      #expression,__FILE__,__LINE__);\
     if( UOS_ErrnoGet()!=0)\
       printf("%s\n",M_errstring(UOS_ErrnoGet()));\
     goto abort;\
 }

#define ACCELERATION_NORMAL			0x200
#define VELOCITY_NORMAL 			21L

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
static int G_verbose = 0;

/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/
static void PrintError(char *info);
static void PrintIndications( MDIS_PATH path );
static int FindPulsesPerRevolution( MDIS_PATH path );
static int MovementWithInterrupt( MDIS_PATH path, int32 endPos );
static int TestLm628Interrupts( MDIS_PATH path );
static int TestPolarity( MDIS_PATH path );
static int WaitForLm628Status(
	MDIS_PATH path,
	u_int16 mask,
	int32 timeout,
	int useIrq,
	u_int32 *statusP);
static int VerifyPosition( MDIS_PATH path, int32 expectedPos );
static int TestM54Indicators( MDIS_PATH path );

/********************************* usage ************************************
 *
 *  Description: Print program usage
 *
 *---------------------------------------------------------------------------
 *  Input......: -
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void usage(void)
{
	printf("Usage: m54_test [<opts>] <device> [<opts>]\n");
	printf("Function: Test program for M54 MDIS5 driver\n");
	printf("          Requires testadapter A_1120\n");
	printf("Options:\n");
	printf("  device       device name..................... [none]    \n");
	printf("  -v=<chan>    verbosity level (0-2)........... [0]       \n");
	printf("  -t=<list>    perform only those tests listed: [all]     \n");
	printf("       1        FindPulsesPerRevolution\n");
	printf("       2        TestLm628Interrupts\n");
	printf("       3        TestM54Indicators\n");
	printf("       4        TestPolarity\n");
	printf("                 don't combine test with other tests\n");
	printf("  -n=<runs>    number of runs through all tests [1]\n");
	printf("\n");
	printf("Copyright 2001-2019, MEN Mikro Elektronik GmbH\n%s\n", IdentString);
}

static void printmsg( int level, char *fmt, ... )
{
	va_list ap;

	if( level <= G_verbose ){
		va_start(ap,fmt);
		vprintf( fmt, ap );
		va_end(ap);
	}
}

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
	int32 rv=1, n;
	char *device, *str, buf[40], *errstr, *testlist;
	int runs, run;

	/*--------------------+
	|  check arguments    |
	+--------------------*/
	if ((errstr = UTL_ILLIOPT("v=?n=t=", buf))) {	/* check args */
		printf("*** %s\n", errstr);
		return(1);
	}

	if (UTL_TSTOPT("?")) {							/* help requested ? */
		usage();
		return(1);
	}

	/*--------------------+
	|  get arguments      |
	+--------------------*/
	for (device=NULL, n=1; n<argc; n++)
		if (*argv[n] != '-') {
			device = argv[n];
			break;
		}

	if (!device) {
		usage();
		return(1);
	}

	G_verbose = ((str = UTL_TSTOPT("v=")) ? atoi(str) : 0);
	testlist  = ((str = UTL_TSTOPT("t=")) ? str : "123");
	runs      = ((str = UTL_TSTOPT("n=")) ? atoi(str) : 1);

	/*--------------------+
	|  open path          |
	+--------------------*/
	if ((path = M_open(device)) < 0) {
		PrintError("open");
		return(1);
	}

	/*------------------------+
	|  Get ready for signals  |
	+------------------------*/
	UOS_SigInit(NULL);
	UOS_SigInstall( UOS_SIG_USR1 );
	UOS_SigInstall( UOS_SIG_USR2 );
	UOS_SigMask();
	CHK( M_setstat( path, M_MK_IRQ_ENABLE, TRUE ) == 0 );

	/*-----------------------------+
	|  Configure basic parameters  |
	+-----------------------------*/
	PrintIndications(path);

	printmsg(1, "Load Filter Parameters\n");
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

	/* set max. threshold for positioning error */
	CHK( M54_LmLoadPositionErrorForInt( path, 0x7f00 ) >= 0 );

	/*-----------------------+
	|  Perform Tests         |
	+-----------------------*/
	for (run=1; run<=runs; run++) {
		char *t;

		printf("\n\n====== RUN %d of %d =========\n", run, runs );

		for (t=testlist; *t; t++) {
			switch(*t){
			case '1':
				if( FindPulsesPerRevolution( path ) )
					goto abort;
				break;
			case '2':
				if( TestLm628Interrupts( path ) )
					goto abort;
				break;
			case '3':
				if( TestM54Indicators( path ) )
					goto abort;
				break;
			case '4':
				if( TestPolarity( path ) )
					goto abort;
				break;
			default:
				printf("Illegal test %c\n", *t );
				goto abort;
			}
		}
	}
	rv = 0;

	/*--------------------+
	|  cleanup            |
	+--------------------*/
 abort:
	if (M_close(path) < 0)
		PrintError("close");
	UOS_SigRemove( UOS_SIG_USR2 );
	UOS_SigRemove( UOS_SIG_USR1 );
	UOS_SigExit();

	printf("\n**** Test %s *****\n", rv ? "FAILED" : "ok" );

	return rv;
}

/**************************** TestLm628Interrupts ***************************
 *
 *  Description: Perform several movements with interrupts
 *
 *---------------------------------------------------------------------------
 *  Input......: path       MDIS device path
 *               endPos     end position to move to
 *  Output.....: returns:   0=ok -1=some error
 *  Globals....: -
 ****************************************************************************/
static int TestLm628Interrupts( MDIS_PATH path )
{
	int32 endPos;

	printmsg(0, "\n=== TestLm628Interrupts ===\n");

	/* define current pos as home */
	CHK( M54_LmDefineHome( path ) >= 0 );

	for( endPos = 500000; endPos > 5000; endPos>>=1 ){
		if( MovementWithInterrupt( path, endPos ) )
			return -1;
	}
	return 0;
 abort:
	return -1;
}

/**************************** MovementWithInterrupt *************************
 *
 *  Description: Move the motor and test interrupts
 *
 *  This function moves the motor from the current position to <endPos>
 *  and then back home to the original pos.
 *
 *  A breakpoint is installed half way.
 *  This function tests TrajectoryComplete, PositionError and Breakpoint
 *  interrupts.
 *
 *  After each movement, the desired position is verified.
 *---------------------------------------------------------------------------
 *  Input......: path       MDIS device path
 *               endPos     end position to move to
 *  Output.....: returns:   0=ok -1=some error
 *  Globals....: -
 ****************************************************************************/
static int MovementWithInterrupt( MDIS_PATH path, int32 endPos )
{
	int useIrq = 1, bkpointreached=FALSE;
	u_int32 status;

	if( useIrq ){
		CHK( M_setstat( path, M54_SETSIG_LM628, UOS_SIG_USR1 ) == 0 );
		CHK( M54_LmResetInterrupts( path, 
									~(LM628_TRAJEC|LM628_POSERR|LM628_BRKPNT)) 
			 >= 0 );
		CHK( M54_LmMaskInterrupts( path, 
								   LM628_TRAJEC|LM628_POSERR)
			 == 0 );
	}

	/*-------------------------------+
	|  Move to position endPos		 |
	+-------------------------------*/
	printf("Move to %d\n", endPos);

	CHK( M54_LmResetInterrupts( path, ~(LM628_TRAJEC|LM628_POSERR) ) >= 0 );

	/* load parameters for movement */
	printmsg(1, "Load Trajectory Parameters\n");
	CHK( M54_LmLoadTrajectoryParameters( path,
										 LM628_ACCLOA | LM628_VELLOA | 
										 LM628_POSLOA,
										 ACCELERATION_NORMAL,/* acceleration */
										 VELOCITY_NORMAL<<16,/* velocity */
										 endPos)      /* position */
		 >=0 );

	/* start motion */
	CHK( M54_LmStartMotion( path ) >= 0);

	/* set breakpoint half way */
	CHK( M54_LmSetBreakpointAbsolute( path, endPos/2 ) == 0 );
	CHK( M54_LmResetInterrupts( path, ~(LM628_BRKPNT)) == 0 ); 
	CHK( M54_LmMaskInterrupts( path, 
							   LM628_TRAJEC|LM628_POSERR|LM628_BRKPNT) == 0 );

	printmsg(1,"Waiting for trajectory complete\n");
	while( 1 ){

		if( WaitForLm628Status( path, LM628_TRAJEC|LM628_POSERR|LM628_BRKPNT, 
								20000, useIrq, &status ) < 0 ){
			printf("Trajectory did not complete!\n");
			goto abort;
		}
		if( status & LM628_POSERR )
			goto positionerr;
		if( status & LM628_TRAJEC ){
			if( !bkpointreached ){
				printf("Trajectory complete without BRKPNT!\n");
				goto abort;
			}
			break;
		}
		if( status & LM628_BRKPNT )
			bkpointreached = TRUE;
		
	}
	if( VerifyPosition( path, endPos ) ) 
		goto abort;

	/*------------------+
	|  Move back home   |
	+------------------*/
	printf("Move back home\n");
	CHK( M54_LmResetInterrupts( path, ~(LM628_TRAJEC|LM628_POSERR) ) >= 0 );
	/* load parameters for movement */
	printmsg(1, "Load Trajectory Parameters\n");
	CHK( M54_LmLoadTrajectoryParameters( path, LM628_POSLOA, 0, 0, 0 ) >=0 );
	/* start motion */
	CHK( M54_LmStartMotion( path ) >= 0);

	printmsg(1,"Waiting for trajectory complete\n");
	if( WaitForLm628Status( path, LM628_TRAJEC|LM628_POSERR,20000, 
							useIrq, &status ) < 0 ){
		printf("Trajectory did not complete!\n");
		goto abort;
	}
	if( status & LM628_POSERR )
		goto positionerr;
	if( VerifyPosition( path, 0 ) ) 
		goto abort;

	
	if( useIrq ){
		CHK( M54_LmMaskInterrupts( path, 0 ) == 0 );
		CHK( M_setstat( path, M54_CLRSIG_LM628, 0 ) == 0 );
	}
	
	return 0;
 abort:
	return -1;
 positionerr:
	printf("Excessive Positioning Error!\n");
	goto abort;
}


/***************************** FindPulsesPerRevolution **********************
 *
 *  Description: Find out number of encoder pulses per revolution
 *
 *  This is done by starting a movement, waiting for the first and second
 *  index pulse to occur and recording the positions of the two index pulses
 *
 *  This routine is somewhat CPU speed dependend.
 *---------------------------------------------------------------------------
 *  Input......: path       MDIS device path
 *  Output.....: returns    0=ok -1=some error
 *  Globals....: -
 ****************************************************************************/
static int FindPulsesPerRevolution( MDIS_PATH path )
{
	u_int32 start, stop, status;
	int32 idxPos1, idxPos2;

	printmsg(0, "\n=== FindPulsesPerRevolution ===\n");
	printmsg(1, "Load Trajectory Parameters\n");

	/* load parameters for movement */
	CHK( M54_LmLoadTrajectoryParameters( path,
										 LM628_ACCLOA | LM628_VELLOA | 
										 LM628_POSLOA,
										 ACCELERATION_NORMAL,/* acceleration */
										 VELOCITY_NORMAL<<16,/* velocity */
										 0x00080000)      /* position */
		 >=0 );

	/* record index pulse */
	CHK( M54_LmSetIndexPosition( path ) >= 0 );

	/*------------------------------------------+
	|  Move the motor and wait for index pulse  |
	+------------------------------------------*/
	CHK( M54_LmStartMotion( path ) >= 0);

	if( WaitForLm628Status( path, LM628_INDEXP, 2000, 0, &status ) < 0 ){
		printf("No index pulse found!\n");
		goto abort;
	}

	/* read the recorded index position */
	CHK( M54_LmReadIndexPosition( path, &idxPos1 ) >= 0 );

	/* let motor skip beyond index */
	UOS_Delay(50);

	/* record index pulse and clear index indication */
	CHK( M54_LmResetInterrupts( path, ~LM628_INDEXP ) >= 0 );
	printmsg(1,"First index pulse: %d\n", idxPos1 );

	/*------------------------------------------+
	|  wait for next index pulse                |
	+------------------------------------------*/
	CHK( M54_LmSetIndexPosition( path ) >= 0 );

	start = UOS_MsecTimerGet();
	if( WaitForLm628Status( path, LM628_INDEXP, 2000, 0, &status ) < 0 ){
		printf("Second index pulse found!\n");
		goto abort;
	}
	stop = UOS_MsecTimerGet();

	/* read the recorded index position */
	CHK( M54_LmReadIndexPosition( path, &idxPos2 ) >= 0 );

	/* record index pulse and clear index indication */
	CHK( M54_LmResetInterrupts( path, ~LM628_INDEXP ) >= 0 );
	printmsg(1,"Second index pulse: %d\n", idxPos2 );
	printmsg(1,"Time for second revolution: %d ms\n", stop-start );

	printf("%d pulses per revolution\n", idxPos2 - idxPos1 );

	/* stop motor */
	printmsg(1,"Stop motor\n");
	CHK( M54_LmLoadTrajectoryParameters( path, LM628_STOPSM, 0,0,0 ) >= 0 );
	CHK( M54_LmStartMotion( path ) >= 0);

	if( WaitForLm628Status( path, LM628_TRAJEC, 2000, 0, &status ) < 0 ){
		printf("Can't stop motor!\n");
		goto abort;
	}

	return 0;
 abort:
	return -1;
}

/********************************* TestM54Indicators ************************
 *
 *  Description: Test linebreak/glitch detection
 *
 *  This routine activates the line break and glitch interrupts/signals and
 *  starts a movement of the motor. The glitch threshold is set to max so
 *  that every pulse will be interpreted as a glitch. 
 *
 *  The user must manually break some quadrature encoder lines in order to
 *  test the line break (Red button on A_1120)
 *---------------------------------------------------------------------------
 *  Input......: path       MDIS device path
 *  Output.....: returns    0=ok -1=some error
 *  Globals....: -
 ****************************************************************************/
static int TestM54Indicators( MDIS_PATH path )
{
	u_int32 status, sigCode;
	int32 rv, gotGlitchSigs=0, gotBreakSigs=0;

	printmsg(0, "\n=== TestM54Indicators (glitch, line break) ===\n");
	printmsg(0, "Hit read button on A_1120 during movement\n");
	/* 
	 * set glitch detector threshold to maximum
	 * -> any movement is taken as a glicth
	 */
	CHK( M_setstat( path, M54_QUAD_COMP, 0xff ) == 0);

	/* activate line break and glitch interrupts */
	CHK( M_setstat( path, M54_SETSIG_LINE_BREAK, UOS_SIG_USR1 ) == 0 );
	CHK( M_setstat( path, M54_SETSIG_GLITCH, UOS_SIG_USR2 ) == 0 );

	/* move the motor->force glitches */
	CHK( M54_LmLoadTrajectoryParameters( path,
										 LM628_ACCLOA | LM628_VELLOA | 
										 LM628_POSLOA,
										 ACCELERATION_NORMAL,/* acceleration */
										 10<<16,/* velocity */
										 800000)      /* position */
		 >=0 );

	CHK( M54_LmStartMotion( path ) >= 0);
	printmsg(1, "Motor started\n");

	while( 1 ){
#ifdef GLITCH_LOGIC_BUG_WORKAROUND
		/* retrigger glitch detect logic */
		CHK( M_setstat( path, M54_QUAD_COMP, 0xff ) == 0);
#endif

		CHK( M54_LmReadStatus( path, &status ) == 0 );
		printmsg(2,"status: 0x%x\n", status );

		if( status & LM628_TRAJEC )
			break;

		rv = UOS_SigWait( 300, &sigCode );
		if( rv == 0 ){
			printmsg(1,"signal %d\n", sigCode );

			CHK( M54_LmReadStatus( path, &status ) == 0 );
			printmsg(2,"status2: 0x%x\n", status );

			if( sigCode == UOS_SIG_USR2 ){
				gotGlitchSigs++;

#ifndef GLITCH_LOGIC_BUG_WORKAROUND
				CHK( status & M54_STAT_GLITCH );
#endif
				/* reactivate glitch detector */
				CHK( M_setstat( path, M54_QUAD_COMP, 0xff ) == 0);
			}				
			if( sigCode == UOS_SIG_USR1 ){
				gotBreakSigs++;
				CHK( status & M54_STAT_LINE_BREAK );
			}
		}
	}
	printf("Got %d glitch signals and %d break signals\n", 
			gotGlitchSigs, gotBreakSigs );

	CHK( gotGlitchSigs != 0 );
	CHK( gotBreakSigs != 0 );
	return 0;

 abort:
	return -1;
}

/********************************* TestPolarity *****************************
 *
 *  Description: Test polarity inversion of M54
 *
 *---------------------------------------------------------------------------
 *  Input......: path       MDIS device path
 *  Output.....: returns    0=ok -1=some error
 *  Globals....: -
 ****************************************************************************/
static int TestPolarity( MDIS_PATH path )
{
	printmsg(0, "\n=== TestPolarity (inversion of LM628 output) ===\n");
	printmsg(0, "Connect Oscilloscope to A_1120, output \"Abgleich\"\n");
	printmsg(0, "Set A_1120 switch to pos \"Abgleich\"\n");
	printmsg(0, "Verify that displayed voltages are present at the M54 "
			 "output\n");
	printmsg(1, "Load Filter Parameters\n");
	CHK( M54_LmLoadFilterParameters( path, 
								LM628_LOAD_KP | LM628_LOAD_KI | LM628_LOAD_KD |
								LM628_LOAD_IL,
                   0x0870, /* kp */
                   0x0000, /* ki */
                   0x0000, /* kd */
                   0x0000, /* il */
                   0)	   /* si */
		 >=0);

	CHK( M54_LmUpdateFilter(path) == 0); /* update filter */

	CHK( M54_LmLoadTrajectoryParameters( path,
										 LM628_ACCLOA | LM628_VELLOA | 
										 LM628_POSLOA,
										 0x00002000,/* acceleration */
										 0x05000000,/* velocity */
										 0x3FFFFFFF)/* position */
		 >=0 );

	printmsg(0, "M54 output should now be +10V\n");
	CHK( M54_LmStartMotion( path ) >= 0);

	UOS_Delay(2000);

	printmsg(0, "M54 output should now be -10V\n");
	CHK( M_setstat( path, M54_POLARITY, 1 ) == 0 );	/* invert polarity */

	UOS_Delay(2000);
	printmsg(0, "M54 output should now be +10V\n");
	CHK( M_setstat( path, M54_POLARITY, 0 ) == 0 );	/* invert polarity */

	return 0;
 abort:
	return -1;
}

/****************************** WaitForLm628Status **************************
 *
 *  Description: Wait until the Lm628 status contains one of the specified
 *               bits
 *
 *---------------------------------------------------------------------------
 *  Input......: path       MDIS device path
 *               mask       bits to test in status
 *               timeout    timeout in ms
 *               useIrq     1=use interrupts, 0=use polling
 *  Output.....: returns:   0=ok -1=timeout
 *               *statusP   complete status read from LM628
 *  Globals....: -
 ****************************************************************************/
static int WaitForLm628Status(
	MDIS_PATH path,
	u_int16 mask,
	int32 timeout,
	int useIrq,
	u_int32 *statusP)
{
	u_int32 status;
	u_int32 start;
	int32 rv;

	start = UOS_MsecTimerGet();

	if( !useIrq ){

		while( UOS_MsecTimerGet() - start < (u_int32)timeout ){ 

			CHK( M54_LmReadStatus( path, &status ) >= 0 );
			PrintIndications(path);

			if( status & mask ){
				*statusP = status;
				return 0;
			}
		}
	}
	else {
		u_int32 signal;
		u_int32 stop = start + timeout;
		int32 intReason;

		while( UOS_MsecTimerGet() - start < (u_int32)timeout ){ 
			rv = UOS_SigWait( stop - UOS_MsecTimerGet(), &signal );
			printmsg(2,"UOS_SigWait: rv=%d sig=%d\n", rv, signal );
			if( rv == 0 ){
				
				CHK( M_getstat( path, M54_LM_INT_REASON, &intReason ) == 0 );
				printmsg(1,"intReason=0x%04x\n", 
						 intReason);
				
				if( intReason & mask ){
					*statusP = intReason;
					return 0;
				}
			}
		}
	}

 abort:
	return -1;					/* timeout */
}

/******************************** VerifyPosition ****************************
 *
 *  Description: Verify that motor reached expected position
 *
 *  Read the "real position" from LM628 and compare it against <expectedPos>
 *  A tolerance of +/- 3 pulses is tolerated
 *
 *  If the expected position is not reached within 5 seconds, return error
 *---------------------------------------------------------------------------
 *  Input......: path        MDIS device path
 *               expectedPos expected motor position
 *  Output.....: returns:    0=ok -1=error
 *  Globals....: -
 ****************************************************************************/
static int VerifyPosition( MDIS_PATH path, int32 expectedPos )
{
	int32 realPos, i;

	for( i=0; i<10; i++ ){
	
		CHK( M54_LmReadRealPosition( path, &realPos ) >= 0 );

		if( !((realPos > expectedPos+3) || (realPos < expectedPos-3)))
			return 0;

		printmsg(1,"Position verify error. Rewaiting...\n");
		UOS_Delay(500);
		PrintIndications(path);
	}
	printf("Error verifying position: current=%d expected=%d\n",
		   realPos, expectedPos );

 abort:
	return -1;
}

static void PrintIndications( MDIS_PATH path )
{
	u_int16 sigs;
	int32 idxPos, desPos, realPos, desVel;
	int16 realVel;
	u_int32 status;

	if( G_verbose < 2 ) return;

	printf("---------------------------------------------\n");
	CHK( M54_LmReadStatus( path, &status ) >= 0 );
	CHK( M54_LmReadSignalsRegister( path, &sigs) >= 0 );
	CHK( M54_LmReadIndexPosition( path, &idxPos ) >= 0 );
	CHK( M54_LmReadDesiredPosition( path, &desPos ) >= 0 );
	CHK( M54_LmReadRealPosition( path, &realPos ) >= 0 );
	CHK( M54_LmReadDesiredVelocity( path, &desVel ) >= 0 );
	CHK( M54_LmReadRealVelocity( path, &realVel ) >= 0 );

	printf(" Status          : 0x%04x\n", status );
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
	printf(" Index Position  : %d\n", idxPos );
	printf(" Desired Position: %d\n", desPos );
	printf(" Real Position   : %d\n", realPos );
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


