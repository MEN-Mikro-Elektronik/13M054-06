/****************************************************************************
 ************                                                    ************
 ************                   M54_BINOUT                       ************
 ************                                                    ************
 ****************************************************************************
 *  
 *       Author: kp
 *
 *  Description: Example program for binary outputs of the M54 driver 
 *
 *  This simple example program stimulates the binary outputs of the M54
 *                      
 *     Required: libraries: mdis_api
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
	int32 value, rvalue;
	char	*device;
	
	if (argc < 2 || strcmp(argv[1],"-?")==0) {
		printf("Syntax: m54_binout <device>\n");
		printf("Function: M54 example for binary outputs\n");
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
	CHK( M_setstat( path, M_MK_CH_CURRENT, M54_CH_BINOUT ) == 0 );

	/*--------------------+
    |  Stimulate outputs  |
    +--------------------*/
	printf("Stimulating outputs. Press any key to stop...\n");
	value = 0;

	while( UOS_KeyPressed() == -1){

		printf("Writing %d\n", (int)value );
		CHK( M_write( path, value ) == 0 );
		CHK( M_read( path, &rvalue ) == 0 );

	    CHK( value == rvalue );	/* check that written value can be read back */

		UOS_Delay(1000);

		if( ++value == 4 )
			value = 0;
	}
	/*--------------------+
    |  cleanup            |
    +--------------------*/
	abort:
	if (M_close(path) < 0)
		PrintError("close");

	return(0);
}


static void PrintError(char *info)
{
	printf("*** can't %s: %s\n", info, M_errstring(UOS_ErrnoGet()));
}
