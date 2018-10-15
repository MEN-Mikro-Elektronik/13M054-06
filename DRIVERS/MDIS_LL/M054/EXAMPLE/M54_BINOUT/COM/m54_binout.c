/****************************************************************************
 ************                                                    ************
 ************                   M54_BINOUT                       ************
 ************                                                    ************
 ****************************************************************************
 *  
 *       Author: kp
 *        $Date: 2015/02/18 16:20:12 $
 *    $Revision: 1.3 $
 *
 *  Description: Example program for binary outputs of the M54 driver 
 *
 *  This simple example program stimulates the binary outputs of the M54
 *                      
 *     Required: libraries: mdis_api
 *     Switches: -
 *
 *-------------------------------[ History ]---------------------------------
 *
 * $Log: m54_binout.c,v $
 * Revision 1.3  2015/02/18 16:20:12  MRoth
 * R: not compatible to MDIS5 (no 64bit support)
 * M: ported to MDIS5 according porting guide rev. 0.10
 *
 * Revision 1.2  2004/04/19 15:56:51  cs
 * eliminated VXWorks compiler warnings (type casts in printf calls)
 *
 * Revision 1.1  2001/09/25 16:27:47  kp
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2001 by MEN mikro elektronik GmbH, Nuernberg, Germany 
 ****************************************************************************/

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
