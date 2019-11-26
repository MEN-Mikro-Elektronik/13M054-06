/****************************************************************************
 ************                                                    ************
 ************                   M54_BININ                        ************
 ************                                                    ************
 ****************************************************************************
 *  
 *       Author: kp
 *
 *  Description: Example program for binary inputs of the M54 driver 
 *
 *  This program continously prints the state of the binary inputs. It also
 *  installs interrupts/signals for some of them.
 *                      
 *     Required: libraries: mdis_api
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


/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/
#define CHK(expression) \
 if( !(expression)) {\
	 printf("*** Error during: %s\nfile %s\nline %d\n", \
      #expression,__FILE__,__LINE__);\
      printf("%s\n",M_errstring(UOS_ErrnoGet()));\
     goto abort;\
 }


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
static void PrintError(char *info);

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
	MDIS_PATH path;
	int32 rv;
	char	*device;
	u_int32 sigCode;
	
	if (argc < 2 || strcmp(argv[1],"-?")==0) {
		printf("Syntax: m54_binin <device>\n");
		printf("Function: M54 example for binary inputs\n");
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
	UOS_SigInit(NULL);

	UOS_SigInstall( UOS_SIG_USR1 );
	UOS_SigMask();
	CHK( M_setstat( path, M54_SETSIG_BININ_EDGE, UOS_SIG_USR1 ) == 0 );
	CHK( M_setstat( path, M_MK_IRQ_ENABLE, TRUE ) == 0 );

	CHK( M_setstat( path, M_MK_CH_CURRENT, M54_CH_BININ ) == 0 );

	/*
	 * Configure sensitive edges for binary inputs:
	 * 0..1 = irq disabled
	 * 2..3	= irq on both edges
	 * 4..5	= irq on rising edge
	 * 6..7 = irq on falling edge
	 */
	CHK( M_setstat( path, M54_BININ_EDGE_CONFIG, 0xfa50 ) == 0 );

	

	/*--------------------+
    |  Display inputs     |
    +--------------------*/
	printf("Displaying binary inputs. Hit any key to stop...\n");

	while( UOS_KeyPressed() == -1){
		int32 value;

		rv = UOS_SigWait(2000, &sigCode);

		CHK( M_read( path, &value ) == 0 );

		if( rv == 0 ){
			if( sigCode == UOS_SIG_USR1 )
				printf("Got signal for binary inputs\t");
		}
		else {
			printf("No signal during last 2 seconds\t");
		}

		printf("Binary inputs=0x%02x\n", (int)value );

	}
	/*--------------------+
    |  cleanup            |
    +--------------------*/
	abort:
	CHK( M_setstat( path, M54_CLRSIG_BININ_EDGE, 0 ) == 0 );
	UOS_SigRemove( UOS_SIG_USR1 );
	UOS_SigExit();

	if (M_close(path) < 0)
		PrintError("close");

	return(0);
}

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
