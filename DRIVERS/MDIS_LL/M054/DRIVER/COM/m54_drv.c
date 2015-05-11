/*********************  P r o g r a m  -  M o d u l e ***********************
 *
 *         Name: m54_drv.c
 *      Project: M54 module driver (MDIS5)
 *
 *       Author: kp
 *        $Date: 2015/02/18 16:20:03 $
 *    $Revision: 1.5 $
 *
 *  Description: Low-level driver for M54 M-Modules
 *
 * The M54 M-Module is a DC motor controller using the LM628. It has an
 * additional 8-bit input port and a 2-bit output port.
 *				 
 * Additionally, the M54 supports quadrature encoder glitch detection
 * and quadrature encoder line-break detection.
 *
 * See PDF user manual 21M054-01 for more information.
 *
 *     Required: OSS, DESC, DBG, ID libraries 
 *     Switches: _ONE_NAMESPACE_PER_DRIVER_
 *				 M54_SW				- use swapped register access
 *				 _LITTLE_ENDIAN_, _BIG_ENDIAN_
 *-------------------------------[ History ]---------------------------------
 *
 * $Log: m54_drv.c,v $
 * Revision 1.5  2015/02/18 16:20:03  MRoth
 * R: not compatible to MDIS5 (no 64bit support)
 * M: ported to MDIS5 according porting guide rev. 0.10
 *
 * Revision 1.4  2004/04/19 15:56:34  cs
 * Bugfix: switched parameters 2 and 3 in call of M54_SetStat in M54_Init
 * Replaced all OSS_IrqMask/OSS_IrqUnMask with OSS_IrqMaskR/OSS_IrqRestore
 * Minor modifications for MDIS4/2004 conformity
 *       type casts
 *
 * Revision 1.3  2001/09/28 08:41:46  kp
 * comments reworked by Susanne
 * eliminated VisualC compiler warnings
 *
 * Revision 1.2  2001/09/26 10:40:12  kp
 * first paramter to OSS_IrqMask/OSS_IrqUnMask corrected
 *
 * Revision 1.1  2001/09/25 16:27:34  kp
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2001 by MEN Mikro Elektronik GmbH, Nuernberg, Germany
 ****************************************************************************/

static const char RCSid[]="$Id: m54_drv.c,v 1.5 2015/02/18 16:20:03 MRoth Exp $";

#ifdef M54_SW				/* swapped variant */
#	define MAC_BYTESWAP
#	define ID_SW	
#endif

#include <MEN/men_typs.h>   /* system dependent definitions   */
#include <MEN/maccess.h>    /* hw access macros and types     */
#include <MEN/dbg.h>        /* debug functions                */
#include <MEN/oss.h>        /* oss functions                  */
#include <MEN/desc.h>       /* descriptor functions           */
#include <MEN/modcom.h>     /* ID PROM functions              */
#include <MEN/mdis_api.h>   /* MDIS global defs               */
#include <MEN/mdis_com.h>   /* MDIS common defs               */
#include <MEN/mdis_err.h>   /* MDIS error codes               */
#include <MEN/ll_defs.h>    /* low-level driver definitions   */

#include <MEN/lm628.h>    	/* LM628 definitions			  */
#include <MEN/z8536.h>    	/* Z8536 definitions			  */
#include <MEN/ll_entry.h>   /* low-level driver branch table  */
#include <MEN/m54_drv.h>    /* driver definitions		      */

/*-----------------------------------------+
|  DEFINES                                 |
+-----------------------------------------*/
/* general */
#define CH_NUMBER			3			/* number of device channels */
#define USE_IRQ				TRUE		/* interrupt required  */
#define ADDRSPACE_COUNT		1			/* nr of required address spaces */
#define ADDRSPACE_SIZE		256			/* size of address space */
#define MOD_ID_MAGIC		0x5346      /* ID PROM magic word */
#define MOD_ID_SIZE			128			/* ID PROM size [bytes] */
#define MOD_ID				54			/* ID PROM module ID */

/* debug settings */
#define DBG_MYLEVEL			h->dbgLevel
#define DBH					h->dbgHdl

/*--- register offsets ---*/

/* 8-bit registers have different offset on different archs */

#if ((!defined(MAC_BYTESWAP) && defined(_BIG_ENDIAN_)) || (defined(MAC_BYTESWAP) && defined(_LITTLE_ENDIAN_)))
/* big endian or little endian/swapped */
# define M54_LM628_STATCMD		0x01 	/* LM628 command/status register */
# define M54_LM628_DATA			0x03 	/* LM628 data register */
# define M54_Z8536_STATCMD		0x0f 	/* Z8536 command/status reg */
# define M54_POLARITY_REG		0x80 	/* M54 polarity reg */
# define M54_QUAD_COMP_REG		0x81 	/* M54 quad compare reg */
# define M54_ISR_REG			0xff 	/* M54 interrupt status/idprom */
# define M54_Z8536_PORTA		0x0d	/* Z8536 port A reg */
# define M54_Z8536_PORTB		0x0b	/* Z8536 port B reg */
# define M54_Z8536_PORTC		0x09	/* Z8536 port C reg */
#else
/* little endian or big endian/swapped */
# define M54_LM628_STATCMD		0x00 	/* LM628 command/status register */
# define M54_LM628_DATA			0x02 	/* LM628 data register */
# define M54_Z8536_STATCMD		0x0e 	/* Z8536 command/status reg */
# define M54_POLARITY_REG		0x81 	/* M54 polarity reg */
# define M54_QUAD_COMP_REG		0x80 	/* M54 quad compare reg */
# define M54_ISR_REG			0xfe 	/* M54 interrupt status/idprom */
# define M54_Z8536_PORTA		0x0c	/* Z8536 port A reg */
# define M54_Z8536_PORTB		0x0a	/* Z8536 port B reg */
# define M54_Z8536_PORTC		0x08	/* Z8536 port C reg */
#endif

/* Bit defs of M54_ISR_REG */
#define M54_ISR_POL				0x08	/* reflects output polarity */
#define M54_ISR_LMI				0x04	/* interrupt from LM628 */
#define M54_ISR_ZI				0x02	/* interrupt from Z8536 */


/* GET_PARAM - get global parameter from descriptor */
#define GET_PARAM(key,def,var)\
		if ((error = DESC_GetUInt32(h->descHdl, (def), \
									(u_int32 *)&var, \
									key)) &&\
			error != ERR_DESC_KEY_NOTFOUND){\
			    return( Cleanup(h,error) );\
        }   

/* flags for command direction */
#define CMD_READ   	0
#define CMD_WRITE	1

/* number of times to read LM628 status register for busy bit */
#define BUSY_TOUT	5000

/* 10ms intervals to wait for CIO reset */
#define CIO_TOUT	10

/*-----------------------------------------+
|  TYPEDEFS                                |
+-----------------------------------------*/
/* low-level handle */
typedef struct {
	/* general */
    int32           memAlloc;		/* size allocated for the handle */
    OSS_HANDLE      *osHdl;         /* oss handle */
    OSS_IRQ_HANDLE  *irqHdl;        /* irq handle */
    DESC_HANDLE     *descHdl;       /* desc handle */
    MACCESS         ma;             /* hw access handle */
	MDIS_IDENT_FUNCT_TBL idFuncTbl;	/* id function table */
	/* debug */
    u_int32         dbgLevel;		/* debug level */
	DBG_HANDLE      *dbgHdl;        /* debug handle */
	/* misc */
    u_int32         idCheck;		/* id check enabled */
	u_int32			lmIsrAccu;		/* LM628 interrupt cause accumulator */
	u_int16			binInEdgeCfg; 	/* edge configuration for binary inputs */
	u_int8			quadComp;		/* value written to quad compare reg */
	u_int8			lmIsrMask;		/* LM628 current isr mask */
	u_int8			lineBreakFlg;	/* flag for lineBreak signal sent */
	u_int8			glitchFlg;		/* flag for glitch signal sent */

	/* signal handles */
	OSS_SIG_HANDLE 	*sigLm628;		/* signal for LM628 */
	OSS_SIG_HANDLE 	*sigBinIn;		/* signal for binary inputs */
	OSS_SIG_HANDLE 	*sigLineBreak;	/* signal for line break */
	OSS_SIG_HANDLE 	*sigGlitch;		/* signal for glitch */

} M54_HANDLE;

/*-----------------------------------------+
|  PROTOTYPES                              |
+-----------------------------------------*/
static int32 M54_Init(DESC_SPEC *descSpec, OSS_HANDLE *osHdl,
					   MACCESS *ma, OSS_SEM_HANDLE *devSemHdl,
					   OSS_IRQ_HANDLE *irqHdl, LL_HANDLE **hP);
static int32 M54_Exit(LL_HANDLE **hP );
static int32 M54_Read(LL_HANDLE *h, int32 ch, int32 *value);
static int32 M54_Write(LL_HANDLE *h, int32 ch, int32 value);
static int32 M54_SetStat(LL_HANDLE *h,int32 ch, int32 code, INT32_OR_64 value32_or_64);
static int32 M54_GetStat(LL_HANDLE *h, int32 ch, int32 code,INT32_OR_64 *value32_or_64P);
static int32 M54_BlockRead(LL_HANDLE *h, int32 ch, void *buf, int32 size,
							int32 *nbrRdBytesP);
static int32 M54_BlockWrite(LL_HANDLE *h, int32 ch, void *buf, int32 size,
							 int32 *nbrWrBytesP);
static int32 M54_Irq(LL_HANDLE *h );
static int32 M54_Info(int32 infoType, ... );

static char* Ident( void );
static int32 Cleanup(M54_HANDLE *h, int32 retCode);
static int32 LM628_Command(
	M54_HANDLE *h, 
	u_int8 cmd,
	u_int8 dir,
	u_int16 dataLen,
	u_int16 *dataP );
static int32 WaitBusyLow( M54_HANDLE *h );
static int32 CioReset(M54_HANDLE *h);
static u_int8 CioRead(M54_HANDLE *h, u_int8 regno);
static void CioWrite(M54_HANDLE *h, u_int8 regno, u_int8 value);
static void CioEdgeCfg( 
	M54_HANDLE *h, 
	int port, 
	u_int16 spec, 
	u_int8 bit );

/**************************** M54_GetEntry *********************************
 *
 *  Description:  Initialize driver's branch table
 *
 *---------------------------------------------------------------------------
 *  Input......:  ---
 *  Output.....:  drvP  pointer to the initialized branch table structure
 *  Globals....:  ---
 ****************************************************************************/
void M54_GetEntry( LL_ENTRY* drvP )
{
    drvP->init        = M54_Init;
    drvP->exit        = M54_Exit;
    drvP->read        = M54_Read;
    drvP->write       = M54_Write;
    drvP->blockRead   = M54_BlockRead;
    drvP->blockWrite  = M54_BlockWrite;
    drvP->setStat     = M54_SetStat;
    drvP->getStat     = M54_GetStat;
    drvP->irq         = M54_Irq;
    drvP->info        = M54_Info;
}

/******************************** M54_Init ***********************************
 *
 *  Description:  Allocate and return low-level handle, initialize hardware
 * 
 * The function
 *  - checks the module ID of the M54
 *	- initializes the Z8536 (all binary input interrupts are disabled)
 *	- sets the binary outputs to zero
 *	- resets the LM628
 *
 *
 * The following descriptor keys are used:
 * 
 * Descriptor key        Default          Range
 * --------------------  ---------------  -------------
 * DEBUG_LEVEL_DESC      OSS_DBG_DEFAULT  see dbg.h
 * DEBUG_LEVEL           OSS_DBG_DEFAULT  see dbg.h
 * ID_CHECK              1                0..1 
 *---------------------------------------------------------------------------
 *  Input......:  descSpec   pointer to descriptor data
 *                osHdl      oss handle
 *                ma         hardware access handle
 *                devSemHdl  device semaphore handle
 *                irqHdl     irq handle
 *  Output.....:  _hP     	 pointer to low-level driver handle
 *                return     success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M54_Init(
    DESC_SPEC       *descP,
    OSS_HANDLE      *osHdl,
    MACCESS         *ma,
    OSS_SEM_HANDLE  *devSemHdl,
    OSS_IRQ_HANDLE  *irqHdl,
    LL_HANDLE       **_hP
)
{
    M54_HANDLE *h = NULL;
    u_int32 gotsize;
    int32 error;
    u_int32 value;

    /*------------------------------+
    |  prepare the handle           |
    +------------------------------*/
	*_hP = NULL;		/* set low-level driver handle to NULL */ 
    
	/* alloc */
    if ((h = (M54_HANDLE*)OSS_MemGet(
    				osHdl, sizeof(M54_HANDLE), &gotsize)) == NULL)
       return(ERR_OSS_MEM_ALLOC);

	/* clear */
    OSS_MemFill(osHdl, gotsize, (char*)h, 0x00);

	/* init */
    h->memAlloc   = gotsize;
    h->osHdl      = osHdl;
    h->irqHdl     = irqHdl;
    h->ma		  = *ma;
	h->lmIsrAccu  = 0;

    /*------------------------------+
    |  init id function table       |
    +------------------------------*/
	/* driver's ident function */
	h->idFuncTbl.idCall[0].identCall = Ident;
	/* library's ident functions */
	h->idFuncTbl.idCall[1].identCall = DESC_Ident;
	h->idFuncTbl.idCall[2].identCall = OSS_Ident;
	/* terminator */
	h->idFuncTbl.idCall[3].identCall = NULL;

    /*------------------------------+
    |  prepare debugging            |
    +------------------------------*/
	DBG_MYLEVEL = OSS_DBG_DEFAULT;	/* set OS-specific debug level */
	DBGINIT((NULL,&DBH));

    /*------------------------------+
    |  scan descriptor              |
    +------------------------------*/
	/* prepare access */
    if ((error = DESC_Init(descP, osHdl, &h->descHdl)))
		return( Cleanup(h,error) );

	GET_PARAM( "DEBUG_LEVEL_DESC", OSS_DBG_DEFAULT, value );
	DESC_DbgLevelSet(h->descHdl, value);	/* set level */

	GET_PARAM( "DEBUG_LEVEL", OSS_DBG_DEFAULT, h->dbgLevel );

    DBGWRT_1((DBH, "LL - M54_Init\n"));

	GET_PARAM( "ID_CHECK", TRUE, h->idCheck );

	DESC_Exit( &h->descHdl );
    /*------------------------------+
    |  check module ID              |
    +------------------------------*/
	if (h->idCheck) {
		int modIdMagic = m_read((U_INT32_OR_64)h->ma, 0);
		int modId      = m_read((U_INT32_OR_64)h->ma, 1);

		if (modIdMagic != MOD_ID_MAGIC) {
			DBGWRT_ERR((DBH," *** M54_Init: illegal magic=0x%04x\n",
						modIdMagic));
			error = ERR_LL_ILL_ID;
			return( Cleanup(h,error) );
		}
		if (modId != MOD_ID) {
			DBGWRT_ERR((DBH," *** M54_Init: illegal id=%d\n",modId));
			error = ERR_LL_ILL_ID;
			return( Cleanup(h,error) );
		}
	}

    /*------------------------------+
    |  init hardware                |
    +------------------------------*/
	/* set defaults */

	/*------------------------------+
	|  init cio						|
	+------------------------------*/
	DBGWRT_2((DBH," cio reset\n"));
	if( (error = CioReset(h)))
		goto abort;

	/* config irq */
	CioWrite(h,MICR,NV);			/* master irq disable, no vect */
	
	/* config port A */
	CioWrite(h,PADDR,0xff);			/* D7..D0 input */
	CioWrite(h,PAMSR,PMS_OR);		/* bit port, pattern OR mode */ 

	CioWrite(h,PACSR,SET_IE);		/* enable port irqs */

	/* config port B */
	CioWrite(h,PBDDR,0xc0);			/* D7..D6 input, D5..D0 output */
	CioWrite(h,PBMSR,PMS_OR|LPM);	/* bit port, pattern OR mode, 
									   latch on pattern match */ 
	CioWrite(h,PBDPR,0x40);			/* D6 inverted */

	CioWrite(h,PBDR,0x00);			/* clear port */
	CioWrite(h,PBCSR,SET_IE);		/* enable port irqs */

	/* enable port A+B */
	CioWrite(h,MCCR,PAE | PBE);		/* enable ports A+B */
	

	M54_SetStat( (LL_HANDLE *)h, M54_BININ_EDGE_CONFIG, M54_CH_BININ, 0 );
	M54_SetStat( (LL_HANDLE *)h, M54_QUAD_COMP, M54_CH_LM628, 0 );
	M54_Write( (LL_HANDLE *)h, M54_CH_BINOUT, 0 );

	/*------------------------------+
	|  init motor controller        |
	+------------------------------*/
	DBGWRT_2((DBH," motor reset\n"));

	{
		u_int16 cmdData[1];

		cmdData[0] = 0;

		/* reset LM628 */
		error = LM628_Command( h, LM628_RESET, CMD_WRITE, 0, cmdData );
		if( error ) goto abort;

		/* mask interrupts */
		error = LM628_Command( h, LM628_MSKI, CMD_WRITE, 1, cmdData );
		if( error ) goto abort;
		
		/* reset pending interrupts */
		error = LM628_Command( h, LM628_RSTI, CMD_WRITE, 1, cmdData );
		if( error ) goto abort;

		/* set LM628 DAC port to 12 bits */
		error = LM628_Command( h, LM628_PORT12, CMD_WRITE, 0, cmdData );
		if( error ) goto abort;
	}		


	*_hP = (LL_HANDLE *)h;	/* set low-level driver handle */

	return(ERR_SUCCESS);

abort:
	return( Cleanup( h, error ));
}

/****************************** M54_Exit *************************************
 *
 *  Description:  De-initialize hardware and clean up memory
 *
 *  The function 
 *  - sets the binary outputs to zero
 *	- disables all interrupts
 *  - removes all handles
 *---------------------------------------------------------------------------
 *  Input......:  _hP  		pointer to low-level driver handle
 *  Output.....:  return    success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M54_Exit(
   LL_HANDLE    **_hP
)
{
    M54_HANDLE *h = (M54_HANDLE *)*_hP;
	int32 error = 0;
	u_int16 cmdData[1];


    DBGWRT_1((DBH, "LL - M54_Exit\n"));

    /*------------------------------+
    |  de-init hardware             |
    +------------------------------*/
	M54_Write( (LL_HANDLE *)h, M54_CH_BINOUT, 0 );

	cmdData[0] = 0;

	/* reset LM628 */
	LM628_Command( h, LM628_RESET, CMD_WRITE, 0, cmdData );

	/* mask interrupts */
	LM628_Command( h, LM628_MSKI, CMD_WRITE, 1, cmdData );

	CioWrite(h,MICR,NV);			/* master irq disable, no vect */
    /*------------------------------+
    |  clean up memory               |
    +------------------------------*/
	error = Cleanup(h,error);
	*_hP = NULL;		/* set low-level driver handle to NULL */ 

	return(error);
}

/****************************** M54_Read *************************************
 *
 *  Description:  Read a value from the device
 *
 *  For channel 0 (LM628), this function returns ERR_LL_ILL_DIR.
 *
 *  For channel 1 (binary inputs), this function returns the current state
 *  of the binary inputs:
 *      bit 0 = binary input 0
 *      bit 1 = binary input 1
 *      ...
 *      bit 7 = binary input 7
 *
 *  For channel 2 (binary outputs), this function returns the current value
 *  of the binary output latch:
 *      bit 0 = binary output O8
 *      bit 1 = binary output O9
 *---------------------------------------------------------------------------
 *  Input......:  h    	   low-level handle
 *                ch       current channel
 *  Output.....:  valueP   read value
 *                return   success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M54_Read(
    LL_HANDLE *_h,
    int32 ch,
    int32 *valueP
)
{
	M54_HANDLE *h = (M54_HANDLE *)_h;
    DBGWRT_1((DBH, "LL - M54_Read: ch=%d\n",ch));

	switch( ch ){
	case M54_CH_BININ:
		*valueP = MREAD_D8( h->ma, M54_Z8536_PORTA ); 
		break;

	case M54_CH_BINOUT:
		*valueP = MREAD_D8( h->ma, M54_Z8536_PORTB ) & 0x3; 
		break;

	default:
		return ERR_LL_ILL_DIR;
	}

	return(ERR_SUCCESS);
}

/****************************** M54_Write ************************************
 *
 *  Description:  Write a value to the device
 *
 *  For channel 0 (LM628) and channel 1 (binary inputs), this function 
 *  returns ERR_LL_ILL_DIR.
 *
 *  For channel 2 (binary outputs), this function writes bits 0 and 1 of
 *  <value> to binary outputs O8 and O9, respectively.
 *---------------------------------------------------------------------------
 *  Input......:  h    	   low-level handle
 *                ch       current channel
 *                value    value to write 
 *  Output.....:  return   success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M54_Write(
    LL_HANDLE *_h,
    int32 ch,
    int32 value
)
{
	M54_HANDLE *h = (M54_HANDLE *)_h;
    DBGWRT_1((DBH, "LL - M54_Write: ch=%d val=0x%x\n",ch, value));
	
	switch( ch ){
	case M54_CH_BINOUT:
	    MWRITE_D8( h->ma, M54_Z8536_PORTB, value & 0x3); 
		break;
	default:
		return ERR_LL_ILL_DIR;
	}
	return(ERR_SUCCESS);
}

/****************************** M54_SetStat **********************************
 *
 *  Description:  Set the driver status
 *
 * The following status codes are supported:
 *
 * Code                 	Description                 Values
 * -------------------  	--------------------------  ----------
 * M54_LM628_PERFORM_CMD	perform a LM628 cmd 		M_SG_BLOCK
 * M54_BININ_EDGE_CONFIG	sensitive edges bin. inputs 0x0000..0xFFFF
 * M54_QUAD_COMP			glitch detect threshold		0x00..0xFF
 * M54_POLARITY				define LM628 output polarity 0/1
 * M_MK_IRQ_ENABLE      	interrupt enable            0/1
 * M54_SETSIG_LM628			install signal for LM628	signal number
 * M54_CLRSIG_LM628			remove signal for LM628		don't care
 * M54_SETSIG_BININ_EDGE	install signal for bin inp	signal number
 * M54_CLRSIG_BININ_EDGE	remove signal for bin inp	don't care
 * M54_SETSIG_LINE_BREAK	install signal for line bk	signal number
 * M54_CLRSIG_LINE_BREAK	remove signal for line bk	don't care
 * M54_SETSIG_GLITCH		install signal for glitch	signal number
 * M54_CLRSIG_GLITCH		remove signal for glitch	don't care
 * M_LL_DEBUG_LEVEL     	driver debug level          see dbg.h
 *
 * M54_LM628_PERFORM_CMD issues a command to the LM628, where all data bytes
 *  are sent TO the LM628. It is heavily used by the M54_API functions. 
 *  This is a block setstat call. <value> must point to an M_SG_BLOCK 
 *  structure, where blk.data must point to an M54_LM628_CMDBLOCK struct, and
 *  blk.size/2-1 specifies the number of 16-bit data words sent to the LM628.
 *
 * M54_BININ_EDGE_CONFIG defines the sensitive edges of the binary inputs
 *  that cause an interrupt. <value> is comprised of eight 2-bit fields:
 *   bits 1..0   = binary input 0
 *   bits 3..2   = binary input 1
 *   ...
 *   bits 15..14 = binary input 7
 *  Each field can be set to 
 *   - M54_IENOFF (no interrupts),
 *   - M54_IEALL (interrupt on both edges)
 *   - M54_IELH  (interrupt on low-to-high transition)
 *   - M54_IEHL  (interrupt on high-to-low transition)
 *
 * M54_QUAD_COMP controls the threshold for the glitch detection logic (see
 *  hardware manual). This call must be issued again after a glitch was detected
 *  in order to reactivate the glitch detection logic.
 *
 * M54_POLARITY controls the output polarity of the M54:
 *  value==0: don't invert polarity
 *  value==1: invert polarity
 *
 * M_MK_IRQ_ENABLE enables (value==1) or disables (value==0) 
 *  interrupts on the M54 globally. This call is handled by the MDIS kernel
 *  too, therefore it will perform actions also on the carrier board.
 *
 * M54_SETSIG_LM628 installs a signal that is sent when the LM628 issues
 *  an interrupt. This call does not enable interrupts. This must be done
 *  using API function M54_LmMaskInterrupts and setstat M_MK_IRQ_ENABLE.
 *
 * M54_CLRSIG_LM628 removes signal M54_SETSIG_LM628.
 *
 * M54_SETSIG_BININ_EDGE installs a signal that is sent when the state of
 *  the binary inputs change as defined by M54_BININ_EDGE_CONFIG.
 *  This call does not enable interrupts. This must be done using
 *  setstat M54_BININ_EDGE_CONFIG and M_MK_IRQ_ENABLE.
 *
 * M54_CLRSIG_BININ_EDGE removes signal M54_SETSIG_BININ_EDGE.
 *
 * M54_SETSIG_LINE_BREAK installs a signal that is sent when a line break
 *  on the quadrature encoder signals has been detected. This call 
 *  implicitly enables line break interrupts.
 *
 * M54_CLRSIG_LINE_BREAK removes signal M54_SETSIG_LINE_BREAK and disables
 *  the line break interrupt.
 *
 * M54_SETSIG_GLITCH installs a signal that is sent when a glitch
 *  on the quadrature encoder signals has been detected. This call 
 *  implicitly enables glitch interrupts. Note that after a glitch has
 *  been detected, the glitch detection logic must be re-enabled by setstat
 *  M54_QUAD_COMP.
 *
 * M54_CLRSIG_GLITCH removes signal M54_SETSIG_GLITCH and disables the glitch
 *  interrupt.
 *---------------------------------------------------------------------------
 *  Input......:  h             low-level handle
 *                code          status code
 *                ch            current channel
 *                value32_or_64 data or
 *                              pointer to block data structure (M_SG_BLOCK)  (*)
 *                (*) = for block status codes
 *  Output.....:  return     success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M54_SetStat(
    LL_HANDLE *_h,
    int32  code,
    int32  ch,
    INT32_OR_64 value32_or_64
)
{
	int32 error = ERR_SUCCESS;
	M54_HANDLE *h = (M54_HANDLE *)_h;
	int32 value = (int32)value32_or_64;		/* 32bit value     */
	INT32_OR_64 valueP = value32_or_64;		/* stores 32/64bit pointer */

	DBGWRT_1((DBH, "LL - M54_SetStat: ch=%d code=0x%04x value=0x%x\n",
			  ch,code,value));

	switch(code) {
	case M54_LM628_PERFORM_CMD:
	{
		/*-----------------------------------------------+
		|  Issue an LM628 command with output data only  |
		+-----------------------------------------------*/
		M_SG_BLOCK *blk = (M_SG_BLOCK*)valueP;
		M54_LM628_CMDBLOCK *cmdblk = (M54_LM628_CMDBLOCK *)blk->data;

		if( blk->size < sizeof(u_int16) ){
			error = ERR_LL_ILL_PARAM;
			break;
		}

		error = LM628_Command( h, (u_int8)cmdblk->opcode, CMD_WRITE, 
							   ((int16)blk->size-sizeof(u_int16))>>1,
							   cmdblk->data );				   

		break;
	}
	case M54_POLARITY:
		if( (value && !(MREAD_D8(h->ma, M54_ISR_REG) & 0x8)) ||
			(!value && (MREAD_D8(h->ma, M54_ISR_REG) & 0x8)))

			/* toggle output polarity */
			MWRITE_D8( h->ma, M54_POLARITY_REG, 0xff );

		/* verify setting */
		if( !!value	!= !!(MREAD_D8(h->ma, M54_ISR_REG) & 0x8)){
			DBGWRT_ERR((DBH,"*** M54_POLARITY: value did not store!\n"));
			error = ERR_LL_WRITE;
		}
		break;

	case M54_QUAD_COMP:
	{
		OSS_IRQ_STATE irqState;
		irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl );
		h->quadComp = (u_int8)value;
		h->glitchFlg = FALSE;  				/* flag glitch signal not sent */
		MWRITE_D8( h->ma, M54_QUAD_COMP_REG, (u_int8)value );
		OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
		break;
	}
	case M_LL_DEBUG_LEVEL:
		h->dbgLevel = value;
		break;

	case M_MK_IRQ_ENABLE:		
		/* don't do anything for LM628, just enable/disable LM628 */
		if( value ){
			OSS_IRQ_STATE irqState;
			irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl );
			CioWrite(h, MICR, CioRead(h, MICR) | MIE);
			OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
		}
		else
			CioWrite(h, MICR, CioRead(h, MICR) & ~MIE);
		break;					

	case M54_BININ_EDGE_CONFIG:
	{
		u_int16 spec, i;
		OSS_IRQ_STATE irqState;
		/*--- configure edge irqs ---*/

		irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl );

		/* disable port A */
		CioWrite(h,MCCR, CioRead(h,MCCR) & ~PAE);

		for (i=0; i<8; i++) {					/* reconfigure */
			spec = (int16)((value >> (i*2)) & 0x3);		/* get config */
			CioEdgeCfg(h, 0, spec, (u_int8)i);	/* config edge irqs */
		}

		/* enable port A */
		CioWrite(h,MCCR, CioRead(h,MCCR) | PAE);
		
		h->binInEdgeCfg = (u_int16)value;		/* save config */

		OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
		break;
	}
	case M54_SETSIG_LM628:
		if( h->sigLm628 ){
			error = ERR_OSS_SIG_SET; /* already installed */
			break;
		}
		error = OSS_SigCreate(h->osHdl, value, &h->sigLm628 );
		break;
		
	case M54_CLRSIG_LM628:
		if( h->sigLm628 == NULL ){		
			error = ERR_OSS_SIG_CLR;
			break;
		}
		error = OSS_SigRemove(h->osHdl, &h->sigLm628 );
		break;

	case M54_SETSIG_BININ_EDGE:
		if( h->sigBinIn ){
			error = ERR_OSS_SIG_SET; /* already installed */
			break;
		}
		error = OSS_SigCreate(h->osHdl, value, &h->sigBinIn );
		break;
		
	case M54_CLRSIG_BININ_EDGE:
		if( h->sigBinIn == NULL ){		
			error = ERR_OSS_SIG_CLR;
			break;
		}
		error = OSS_SigRemove(h->osHdl, &h->sigBinIn );
		break;

	case M54_SETSIG_LINE_BREAK:
	{
		OSS_IRQ_STATE irqState;

		if( h->sigLineBreak ){
			error = ERR_OSS_SIG_SET; /* already installed */
			break;
		}
		error = OSS_SigCreate(h->osHdl, value, &h->sigLineBreak );

		/* enable irq on any transition for PB7 */
		irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl );
		h->lineBreakFlg = FALSE;  		  /* flag line break signal not sent */
		CioEdgeCfg( h, 1, M54_IEALL, 7 );
		OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
		break;
	}
		
	case M54_CLRSIG_LINE_BREAK:
	{
		OSS_IRQ_STATE irqState;

		if( h->sigLineBreak == NULL ){		
			error = ERR_OSS_SIG_CLR;
			break;
		}
		error = OSS_SigRemove(h->osHdl, &h->sigLineBreak );

		/* disable low->high transition irq for PB7 */
		irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl );
		CioEdgeCfg( h, 1, M54_IENOFF, 7 );
		OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
		
		break;
	}

	case M54_SETSIG_GLITCH:
	{
		OSS_IRQ_STATE irqState;

		if( h->sigGlitch ){
			error = ERR_OSS_SIG_SET; /* already installed */
			break;
		}
		error = OSS_SigCreate(h->osHdl, value, &h->sigGlitch );

		/* enable low->high transition irq for PB6 */
		irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl );
		h->glitchFlg = FALSE;  				/* flag glitch signal not sent */
		CioEdgeCfg( h, 1, M54_IELH, 6 );
		OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
		break;
	}
		
	case M54_CLRSIG_GLITCH:
	{
		OSS_IRQ_STATE irqState;

		if( h->sigGlitch == NULL ){		
			error = ERR_OSS_SIG_CLR;
			break;
		}
		error = OSS_SigRemove(h->osHdl, &h->sigGlitch );

		/* disable low->high transition irq for PB6 */
		irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl );
		CioEdgeCfg( h, 1, M54_IENOFF, 6 );
		OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
		break;
	}

	default:
		error = ERR_LL_UNK_CODE;
		break;
    }

	return(error);
}

/****************************** M54_GetStat **********************************
 *
 *  Description:  Get the driver status
 *
 *  The following status codes are supported:
 *	
 *  Code                 	Description                 Values
 *  ------------------- 	 --------------------------  ----------
 *  M54_LM628_PERFORM_CMD	perform a LM628 cmd 		 M_SG_BLOCK
 *	M54_LM_STATUS		 	read LM628 status byte		 see text
 *	M54_LM_INT_REASON  	 	interrupt cause 			 see text
 *	M54_BININ_EDGE_CONFIG 	binary input edges		 	 0x0000..0xffff
 *	M54_QUAD_COMP		 	glitch detect threshold	 	 0x00..0xFF
 *  M54_POLARITY			LM628 output polarity		 0/1
 *  M_LL_DEBUG_LEVEL     	driver debug level           see dbg.h
 *  M_LL_CH_NUMBER       	number of channels           3
 *  M_LL_CH_DIR          	direction of curr. chan.     M_CH_IN/OUT
 *  M_LL_CH_LEN          	length of curr. ch. [bits]   16/8/2
 *  M_LL_CH_TYP          	description of curr. chan.   M_CH_BINARY
 *  M_LL_ID_CHECK        	EEPROM is checked            0..1
 *  M_LL_ID_SIZE         	EEPROM size [bytes]          128
 *  M_LL_BLK_ID_DATA     	EEPROM raw data              -
 *  M_MK_BLK_REV_ID      	ident function table ptr     -
 *
 * M54_LM628_PERFORM_CMD issues a command to the LM628, where all data bytes
 *  are read FROM the LM628. It is heavily used by the M54_API functions. 
 *  This is a block setstat call. <valueP> must point to an M_SG_BLOCK 
 *  structure, where blk.data must point to an M54_LM628_CMDBLOCK struct, and
 *  blk.size/2-1 specifies the number of 16-bit data words read from the 
 *  LM628.
 *
 * M54_LM_STATUS is used by API function M54_LmReadStatus and returns 
 *  the status byte of the LM628 plus the M54-specific status bits 
 *  M54_STAT_LINE_BREAK and M54_STAT_GLITCH.
 *
 * M54_LM_INT_REASON will return accumulated LM628 interrupt flags that
 *  have occurred since the last call to M54_LM_INT_REASON. It also includes
 *  the additional M54_STAT_LINE_BREAK and M54_STAT_GLITCH flags. The internal
 *  flag accumulator is cleared by this call. Only interrupts that have
 *  been enabled through the MSKI command will appear in the return value.
 *
 * M54_BININ_EDGE_CONFIG returns the value that has been defined by setstat
 *  M54_BININ_EDGE_CONFIG.
 *
 * M54_QUAD_COMP returns the value that has been defined by setstat
 *  M54_QUAD_COMP.
 *
 * M54_POLARITY returns the current polarity of the LM628 output circuit.
 *  0 = not inverted
 *  1 = inverted
 *---------------------------------------------------------------------------
 *  Input......:  h               low-level handle
 *                code            status code
 *                ch              current channel
 *                value32_or_64P  pointer to block data structure (M_SG_BLOCK)  (*) 
 *                (*) = for block status codes
 *  Output.....:  value32_or_64P  data pointer or
 *                                pointer to block data structure (M_SG_BLOCK)  (*) 
 *                return     success (0) or error code
 *                (*) = for block status codes
 *  Globals....:  ---
 ****************************************************************************/
static int32 M54_GetStat(
    LL_HANDLE *_h,
    int32  code,
    int32  ch,
    INT32_OR_64 *value32_or_64P
)
{
	int32 error = ERR_SUCCESS;
	M54_HANDLE *h = (M54_HANDLE *)_h;
	int32 *valueP = (int32*)value32_or_64P;			/* pointer to 32bit value  */
	INT32_OR_64 *value64P = value32_or_64P;			/* stores 32/64bit pointer  */

	DBGWRT_1((DBH, "LL - M54_GetStat: ch=%d code=0x%04x\n",
			  ch,code));

	switch(code)
	{
	case M54_LM_STATUS:
		/* read LM628 status byte */
		*valueP = MREAD_D8( h->ma, M54_LM628_STATCMD );

		/* check for line break */
		if( MREAD_D8( h->ma, M54_Z8536_PORTB ) & 0x80 )
			*valueP |= M54_STAT_LINE_BREAK;

		/* check for glitch */
		if( MREAD_D8( h->ma, M54_Z8536_PORTB ) & 0x40 )
			*valueP |= M54_STAT_GLITCH;

		break;

	case M54_LM_INT_REASON:
	{
		OSS_IRQ_STATE irqState;
		irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl );
		*valueP = h->lmIsrAccu;
		h->lmIsrAccu = 0;
		OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
		break;
	}

	case M54_LM628_PERFORM_CMD:
	{
		/*-----------------------------------------------+
		|  Issue an LM628 command with input data only   |
		+-----------------------------------------------*/
		M_SG_BLOCK *blk = (M_SG_BLOCK*)value32_or_64P; 	/* stores block struct pointer */
		M54_LM628_CMDBLOCK *cmdblk = (M54_LM628_CMDBLOCK *)blk->data;

		if( blk->size < sizeof(u_int16) ){
			error = ERR_LL_ILL_PARAM;
			break;
		}

		error = LM628_Command( h, (u_int8)cmdblk->opcode, CMD_READ, 
							   ((int16)blk->size-sizeof(u_int16))>>1,
							   cmdblk->data );

		break;
	}

	case M54_BININ_EDGE_CONFIG:
		*valueP = h->binInEdgeCfg;
		break;

	case M54_QUAD_COMP:
		*valueP = h->quadComp;
		break;

	case M54_POLARITY:
		*valueP =  !!(MREAD_D8(h->ma, M54_ISR_REG) & 0x8);
		break;

	case M_LL_DEBUG_LEVEL:
		*valueP = h->dbgLevel;
		break;
	case M_LL_CH_NUMBER:
		*valueP = CH_NUMBER;
		break;
	case M_LL_CH_DIR:
		*valueP = M_CH_INOUT;
		break;
	case M_LL_CH_LEN:
		switch(ch){
		case M54_CH_LM628:	*valueP = 16;	break;
		case M54_CH_BININ:	*valueP = 8;	break;
		case M54_CH_BINOUT:	*valueP = 2;	break;
		}
		break;
	case M_LL_CH_TYP:
		*valueP = M_CH_BINARY;
		break;
	case M_LL_ID_CHECK:
		*valueP = h->idCheck;
		break;
	case M_LL_ID_SIZE:
		*valueP = MOD_ID_SIZE;
		break;
	case M_LL_BLK_ID_DATA:
	{
		M_SG_BLOCK *blk = (M_SG_BLOCK*)value32_or_64P; 	/* stores block struct pointer */
		u_int32 n;
		u_int16 *dataP = (u_int16*)blk->data;

		if (blk->size < MOD_ID_SIZE)		/* check buf size */
			return(ERR_LL_USERBUF);

		for (n=0; n<MOD_ID_SIZE/2; n++)		/* read MOD_ID_SIZE/2 words */
			*dataP++ = (int16)m_read((U_INT32_OR_64)h->ma, (u_int8)n);

		break;
	}
	case M_MK_BLK_REV_ID:
		*value64P = (INT32_OR_64)&h->idFuncTbl;
		break;

		
	default:
		error = ERR_LL_UNK_CODE;
		break;
    }

	return(error);
}

/******************************* M54_BlockRead *******************************
 *
 *  Description:  Not implemented for M54
 *
 *---------------------------------------------------------------------------
 *  Input......:  h            low-level handle
 *                ch           current channel
 *                buf          data buffer
 *                size         data buffer size
 *  Output.....:  nbrRdBytesP  number of read bytes
 *                return       always ERR_LL_ILL_FUNC
 *  Globals....:  ---
 ****************************************************************************/
static int32 M54_BlockRead(
     LL_HANDLE *_h,
     int32     ch,
     void      *buf,
     int32     size,
     int32     *nbrRdBytesP
)
{
	return ERR_LL_ILL_FUNC;
}

/****************************** M54_BlockWrite *******************************
 *
 *  Description:  Not implemented for M54
 *
 *---------------------------------------------------------------------------
 *  Input......:  h        low-level handle
 *                ch           current channel
 *                buf          data buffer
 *                size         data buffer size
 *  Output.....:  nbrWrBytesP  number of written bytes
 *                return       always ERR_LL_ILL_FUNC
 *  Globals....:  ---
 ****************************************************************************/
static int32 M54_BlockWrite(
     LL_HANDLE *h,
     int32     ch,
     void      *buf,
     int32     size,
     int32     *nbrWrBytesP
)
{
	return ERR_LL_ILL_FUNC;
}


/****************************** M54_Irq *************************************
 *
 *  Description:  Interrupt service routine
 *
 *  The interrupt is triggered when
 *  - the LM628 interrupts.
 *  - the Z8636 interrupts due to
 *     - binary input changes (PA0..7)
 *	   - a line break (PB7)
 *	   - a glitch (PB6)	
 *---------------------------------------------------------------------------
 *  Input......:  h    	   low-level handle
 *  Output.....:  return   LL_IRQ_DEVICE	irq caused by device
 *                         LL_IRQ_DEV_NOT   irq not caused by device
 *  Globals....:  ---
 ****************************************************************************/
static int32 M54_Irq(
   LL_HANDLE *_h
)
{
	M54_HANDLE *h = (M54_HANDLE *)_h;
	MACCESS ma = h->ma;
	u_int8 isrReg, status;

	/* read interrupt status reg */
	isrReg = MREAD_D8( ma, M54_ISR_REG );
	
    IDBGWRT_1((DBH, ">>> M54_Irq: isrReg=0x%02x\n", isrReg));

	if( isrReg & M54_ISR_LMI ){
		/*-----------------------+
		|  Interrupt from LM628  |
		+-----------------------*/
		
		/* read LM628 interrupt status */
		status = MREAD_D8( ma, M54_LM628_STATCMD ) & 0xfe;
		IDBGWRT_2((DBH, " lm628 status=0x%02x isrMask=0x%02x\n", status,
				   h->lmIsrMask ));

		/* save it into accumulator (except busy bit) */	
		h->lmIsrAccu |= status & h->lmIsrMask & 0xfe;

		/* reset all unmasked LM628 interrupts */
	    WaitBusyLow( h );
		MWRITE_D8( ma, M54_LM628_STATCMD, LM628_RSTI );

	    WaitBusyLow( h );
		MWRITE_D8( ma, M54_LM628_DATA, 0x00 ); /* upper byte always zero */
		MWRITE_D8( ma, M54_LM628_DATA, ~h->lmIsrMask ); 

		/* send signal to user if installed */
		if( h->sigLm628 ){
			IDBGWRT_2((DBH, " send LM628 sig\n"));
			OSS_SigSend( h->osHdl, h->sigLm628 );
		}

		/* 
		 * Now wait until the interrupt is released. Sometimes it takes
		 * a while until the IRQ line is released, therefore causing
		 * an unexpected second interrupt. Waiting for busy low seems
		 * to solve the problem.
		 */
	    WaitBusyLow( h );
	}

	if( isrReg & M54_ISR_ZI ){
		/*-----------------------+
		|  Interrupt from Z8536  |
		+-----------------------*/
		if (CioRead(h, PACSR) & IP) {	/* port A irq pending ? */
			CioWrite(h, PACSR, CLR_IP);	/* reset IP, force IACK */

			IDBGWRT_2((DBH, " BinIn irq\n"));
			/* send signal to user if installed */
			if( h->sigBinIn ){
				IDBGWRT_2((DBH, " send BinIn sig\n"));
				OSS_SigSend( h->osHdl, h->sigBinIn );
			}
		}

		if (CioRead(h, PBCSR) & IP) {	/* port B irq pending ? */
			u_int8 pbdr;

			CioWrite(h, PBCSR, CLR_IP);	/* reset IP, force IACK */

			pbdr = MREAD_D8( ma, M54_Z8536_PORTB );
			IDBGWRT_2((DBH, " PortB irq pbdr=0x%02x\n", pbdr));

			if( h->sigLineBreak ){
				if( (pbdr & 0x80) && !h->lineBreakFlg ){ /* B7: line break ? */

					h->lmIsrAccu |= M54_STAT_LINE_BREAK;

					/* send signal to user if installed */
					IDBGWRT_2((DBH, " send LineBreak sig\n"));
					OSS_SigSend( h->osHdl, h->sigLineBreak );
					h->lineBreakFlg = TRUE;
				}
				
				if( !(pbdr & 0x80) && h->lineBreakFlg )  /* B7: line ok? */
					h->lineBreakFlg = FALSE;
			}

			if( h->sigGlitch ){
				if( (pbdr & 0x40) && !h->glitchFlg ){ 	 /* B6: glitch ? */

					h->lmIsrAccu |= M54_STAT_GLITCH;

					/* send signal to user if installed */
					IDBGWRT_2((DBH, " send Glitch sig\n"));
					OSS_SigSend( h->osHdl, h->sigGlitch );
					h->glitchFlg = TRUE;
				}
			}

		}
	}


	return (isrReg & (M54_ISR_LMI|M54_ISR_ZI)) ? 
		LL_IRQ_DEVICE : LL_IRQ_DEV_NOT;
}

/****************************** M54_Info ************************************
 *
 *  Description:  Get information about hardware and driver requirements
 *
 *                The following info codes are supported:
 *
 *                Code                      Description
 *                ------------------------  -----------------------------
 *                LL_INFO_HW_CHARACTER      hardware characteristics
 *                LL_INFO_ADDRSPACE_COUNT   nr of required address spaces
 *                LL_INFO_ADDRSPACE         address space information
 *                LL_INFO_IRQ               interrupt required
 *                LL_INFO_LOCKMODE          process lock mode required
 *
 *                The LL_INFO_HW_CHARACTER code returns all address and 
 *                data modes (ORed) which are supported by the hardware
 *                (MDIS_MAxx, MDIS_MDxx).
 *
 *                The LL_INFO_ADDRSPACE_COUNT code returns the number
 *                of address spaces used by the driver.
 *
 *                The LL_INFO_ADDRSPACE code returns information about one
 *                specific address space (MDIS_MAxx, MDIS_MDxx). The returned 
 *                data mode represents the widest hardware access used by 
 *                the driver.
 *
 *                The LL_INFO_IRQ code returns whether the driver supports an
 *                interrupt routine (TRUE or FALSE).
 *
 *                The LL_INFO_LOCKMODE code returns which process locking
 *                mode the driver needs (LL_LOCK_xxx).
 *---------------------------------------------------------------------------
 *  Input......:  infoType	   info code
 *                ...          argument(s)
 *  Output.....:  return       success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M54_Info(
   int32  infoType,
   ...
)
{
    int32   error = ERR_SUCCESS;
    va_list argptr;

    va_start(argptr, infoType );

    switch(infoType) {
		/*-------------------------------+
        |  hardware characteristics      |
        |  (all addr/data modes ORed)   |
        +-------------------------------*/
        case LL_INFO_HW_CHARACTER:
		{
			u_int32 *addrModeP = va_arg(argptr, u_int32*);
			u_int32 *dataModeP = va_arg(argptr, u_int32*);

			*addrModeP = MDIS_MA08;
			*dataModeP = MDIS_MD08 | MDIS_MD16;
			break;
	    }
		/*-------------------------------+
        |  nr of required address spaces |
        |  (total spaces used)           |
        +-------------------------------*/
        case LL_INFO_ADDRSPACE_COUNT:
		{
			u_int32 *nbrOfAddrSpaceP = va_arg(argptr, u_int32*);

			*nbrOfAddrSpaceP = ADDRSPACE_COUNT;
			break;
	    }
		/*-------------------------------+
        |  address space type            |
        |  (widest used data mode)       |
        +-------------------------------*/
        case LL_INFO_ADDRSPACE:
		{
			u_int32 addrSpaceIndex = va_arg(argptr, u_int32);
			u_int32 *addrModeP = va_arg(argptr, u_int32*);
			u_int32 *dataModeP = va_arg(argptr, u_int32*);
			u_int32 *addrSizeP = va_arg(argptr, u_int32*);

			if (addrSpaceIndex >= ADDRSPACE_COUNT)
				error = ERR_LL_ILL_PARAM;
			else {
				*addrModeP = MDIS_MA08;
				*dataModeP = MDIS_MD16;
				*addrSizeP = ADDRSPACE_SIZE;
			}

			break;
	    }
		/*-------------------------------+
        |   interrupt required           |
        +-------------------------------*/
        case LL_INFO_IRQ:
		{
			u_int32 *useIrqP = va_arg(argptr, u_int32*);

			*useIrqP = USE_IRQ;
			break;
	    }
		/*-------------------------------+
        |   process lock mode            |
        +-------------------------------*/
        case LL_INFO_LOCKMODE:
		{
			u_int32 *lockModeP = va_arg(argptr, u_int32*);

			*lockModeP = LL_LOCK_CALL;
			break;
	    }
		/*-------------------------------+
        |   (unknown)                    |
        +-------------------------------*/
        default:
          error = ERR_LL_ILL_PARAM;
    }

    va_end(argptr);
    return(error);
}

/*******************************  Ident  ************************************
 *
 *  Description:  Return ident string
 *
 *---------------------------------------------------------------------------
 *  Input......:  -
 *  Output.....:  return  pointer to ident string
 *  Globals....:  -
 ****************************************************************************/
static char* Ident( void )	/* nodoc */
{
    return( "M54 - M54 low level driver: $Id: m54_drv.c,v 1.5 2015/02/18 16:20:03 MRoth Exp $" );
}

/********************************* Cleanup **********************************
 *
 *  Description: Close all handles, free memory and return error code
 *		         NOTE: The low-level handle is invalid after this function is
 *                     called.
 *			   
 *---------------------------------------------------------------------------
 *  Input......: h		    low-level handle
 *               retCode    return value
 *  Output.....: return	    retCode
 *  Globals....: -
 ****************************************************************************/
static int32 Cleanup(
   M54_HANDLE    *h,
   int32        retCode		/* nodoc */
)
{
    /*------------------------------+
    |  close handles                |
    +------------------------------*/

	/* free signals */
	if( h->sigLm628 ) 		OSS_SigRemove(h->osHdl, &h->sigLm628 );
	if( h->sigBinIn ) 		OSS_SigRemove(h->osHdl, &h->sigBinIn );
	if( h->sigLineBreak ) 	OSS_SigRemove(h->osHdl, &h->sigLineBreak );
	if( h->sigGlitch ) 		OSS_SigRemove(h->osHdl, &h->sigGlitch );

	/* clean up debug */
	DBGEXIT((&DBH));

    /*------------------------------+
    |  free memory                  |
    +------------------------------*/
    /* free my handle */
    OSS_MemFree(h->osHdl, (int8*)h, h->memAlloc);

    /*------------------------------+
    |  return error code            |
    +------------------------------*/
	return(retCode);
}

/********************************* LM628_Command *****************************
 *
 *  Description: Send a command to LM628 and write/read corresponding data 
 *			   
 *	If interrupts of LM628 have been enabled and the data length is
 *  greater than 2 (words), this function will issue a MSKI command to
 *  mask all interrupts before sending the requested command. Afterwards, the
 *	old mask is restored.
 *  Otherwise if the data length is smaller than two words, interrupts will
 *  be masked by OSS_IrqMaskR.
 *  This assures that interrupts will not be blocked for more than approx 30us.
 *---------------------------------------------------------------------------
 *  Input......: h		low level handle
 *				 cmd	command opcode
 *				 dir	direction of data (CMD_WRITE, CMD_READ)
 *				 dataLen number of 16 bit data words to send/receive
 *				 dataP	buffer to send/receive
 *  Output.....: returns:  MDIS error code
 *							ERR_LL_DEV_BUSY		timeout waiting for busy
 *							
 *  Globals....: h->lmIsrMask is updated when MSKI command is requested
 ****************************************************************************/
static int32 LM628_Command(			/* nodoc */
	M54_HANDLE *h, 
	u_int8 cmd,
	u_int8 dir,
	u_int16 dataLen,
	u_int16 *dataP )
{
	int32 error=0;
	u_int8 savedMask = 0, irqmasked=FALSE;
	u_int16 cnt = dataLen;
	u_int16 *dP = dataP;
	MACCESS ma = h->ma;
	OSS_IRQ_STATE irqState;

	DBGWRT_2((DBH,"LM628_Command cmd=0x%02x dir=%d len=%d isrMask=0x%x\n",
			  cmd, dir, dataLen, h->lmIsrMask ));
#ifdef DBG
	if( dir == CMD_WRITE )
		DBGDMP_3((DBH,"Cmd Write Data",dataP, dataLen*2, 2 ));
#endif
	if( h->lmIsrMask ){
		if( dataLen > 2 ){
			u_int16 data[1];

			/*
			 * Interrupts have been enabled by the user. 
			 * mask all interrupts from LM628 before sending command
			 */		
			irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl );
			savedMask = h->lmIsrMask;
			data[0] = 0x00;			/* mask all LM628 interrupts */
			error = LM628_Command( h, LM628_MSKI, CMD_WRITE, 1, data );
			OSS_IrqRestore( h->osHdl, h->irqHdl, irqState ); /* unmask interrupts */
			if( error ) goto abort;
		}
		else {
			irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl ); /* mask interrupts during command */
			irqmasked = TRUE;
		}
	}

	/* wait for busy low */
	if( (error = WaitBusyLow(h)) )
		goto abort;

	/* issue the requested command */
	MWRITE_D8( ma, M54_LM628_STATCMD, cmd );

	if( cmd == LM628_MSKI )
		h->lmIsrMask = (u_int8)dP[0];
	
	/* write/read data words */
	while( cnt-- ){

		/* wait for busy low */
		if( (error = WaitBusyLow(h)) )
			goto abort;
		
		if( dir == CMD_WRITE ){
			MWRITE_D8( ma, M54_LM628_DATA, (*dP >> 8));
			MWRITE_D8( ma, M54_LM628_DATA, (u_int8)*dP);
		}
		else {
			*dP = MREAD_D8( ma, M54_LM628_DATA) << 8;
			*dP |= MREAD_D8( ma, M54_LM628_DATA);
		}
		dP++;
	}


 abort:
	if( irqmasked ) 
		OSS_IrqRestore( h->osHdl, h->irqHdl, irqState ); /* unmask interrupts */
	else if( savedMask ){
		/* re-enable LM628 interrupts */
		u_int16 data[1];

		data[0] = savedMask;
		LM628_Command( h, LM628_MSKI, CMD_WRITE, 1, data );
	}

#ifdef DBG
	if( dir == CMD_READ )
		DBGDMP_3((DBH,"Cmd Read Data",dataP, dataLen*2, 2 ));
#endif

	return error;	
}

/********************************* WaitBusyLow *******************************
 *
 *  Description: Wait until LM628 BUSY flag goes low (or timeout)
 *			   
 *---------------------------------------------------------------------------
 *  Input......: h	        low-level handle
 *  Output.....: returns:	error code (ERR_LL_DEV_BUSY if timeout)
 *  Globals....: -
 ****************************************************************************/
static int32 WaitBusyLow( M54_HANDLE *h ) /* nodoc */
{
	int32 busyTimeout;

	/* wait for busy low */
	busyTimeout = BUSY_TOUT;
	while( MREAD_D8( h->ma, M54_LM628_STATCMD) & LM628_BUSY ){
		if( --busyTimeout == 0 ){
			DBGWRT_ERR((DBH,"*** LM628_Command: Timeout waiting for busy "
						"low\n"));
			return ERR_LL_DEV_BUSY;
		}
	}
	return 0;
}



/****************************** CioReset ***********************************
 *
 *  Description:  Reset the CIO. (After reset CIO is in state 0).
 *
 *---------------------------------------------------------------------------
 *  Input......:  h	        low level handle
 *  Output.....:  returns:	error code (ERR_LL_DEV_BUSY if CIO doesn't reset)
 *  Globals....:  ---
 ****************************************************************************/
static int32 CioReset(M54_HANDLE *h)	/* nodoc */
{
	int32 timeout;
	OSS_IRQ_STATE irqState;

	irqState = OSS_IrqMaskR( h->osHdl, h->irqHdl );

	/*-------------------+
	| sync to state 0    |
	+-------------------*/
	MREAD_D8( h->ma, M54_Z8536_STATCMD );
	MWRITE_D8( h->ma, M54_Z8536_STATCMD, 0x00 );
	MREAD_D8( h->ma, M54_Z8536_STATCMD );

	/*-------------------+
	| reset cio          |
	+-------------------*/
	MWRITE_D8( h->ma, M54_Z8536_STATCMD, MICR ); /* select MICR */
	MWRITE_D8( h->ma, M54_Z8536_STATCMD, _RESET ); /* force reset state */
	
    /* wait for reset */
	timeout = CIO_TOUT;

	while( MREAD_D8( h->ma, M54_Z8536_STATCMD ) != _RESET ){
		if( --timeout == 0 ){
			DBGWRT_ERR((DBH,"*** CioReset: Timeout waiting for reset\n"));
			OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
			return ERR_LL_DEV_BUSY;
		}
		OSS_Delay( h->osHdl, 10 ); /* delay 10ms */
	}
			
	MWRITE_D8( h->ma, M54_Z8536_STATCMD, 0x00 ); /* clear reset state */

    /* wait until cleared */
	timeout = CIO_TOUT;

	while( MREAD_D8( h->ma, M54_Z8536_STATCMD ) & _RESET ){
		if( --timeout == 0 ){
			DBGWRT_ERR((DBH,"*** CioReset: Timeout waiting for reset gone\n"));
			OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
			return ERR_LL_DEV_BUSY;
		}
		OSS_Delay( h->osHdl, 10 ); /* delay 10ms */
	}
	
	OSS_IrqRestore( h->osHdl, h->irqHdl, irqState );
	return ERR_SUCCESS;
}

/****************************** CioRead *************************************
*
*  Description:  Read CIO register. (CIO must be in state 0).
*
*---------------------------------------------------------------------------
*  Input......:  h	       low-level handle
*                regno     register number
*  Output.....:  returns:  register value
*  Globals....:  ---
****************************************************************************/
static u_int8 CioRead(M54_HANDLE *h, u_int8 regno) /* nodoc */
{
	u_int8 value;

	MWRITE_D8( h->ma, M54_Z8536_STATCMD, regno ); /* select register */
	value = MREAD_D8( h->ma, M54_Z8536_STATCMD ); /* read register */
	
	return(value);
}

/****************************** CioWrite ************************************
*
*  Description:  Write to CIO register. (CIO must be in state 0).
*
*---------------------------------------------------------------------------
*  Input......:  h	       low-level handle
*                regno     register number
*				 value	   register value
*  Output.....:  -
*  Globals....:  ---
****************************************************************************/
static void CioWrite(M54_HANDLE *h, u_int8 regno, u_int8 value)	/* nodoc */
{
	MWRITE_D8( h->ma, M54_Z8536_STATCMD, regno ); /* select register */
	MWRITE_D8( h->ma, M54_Z8536_STATCMD, value ); /* write register */

	return;
}

/******************************* CioEdgeCfg **********************************
 *
 *  Description:  Configure port A edge interrupt of specified port bit.
 *
 *---------------------------------------------------------------------------
 *  Input......:  h	       low-level handle
 *				  port	   0 = portA, 1 = portB
 *                spec	   edge specification (M54_IExx)
 *                bit	   portbit (0..7) 
 *  Output.....:  -
 *  Globals....:  ---
 ****************************************************************************/
static void CioEdgeCfg( 
	M54_HANDLE *h, 
	int port, 
	u_int16 spec, 
	u_int8 bit ) /* nodoc */
{
	u_int8 papmr,paptr,pappr,portbit;

	DBGWRT_2((DBH, " CioEdgeCfg: port=%c bit=%d spec=%d\n", 
			  port ? 'B' : 'A', bit, spec ));
	
	portbit = 1 << (bit);						/* select port bit */
	port *= 8;									/* convert 0/1 to 0/8 */
	papmr =	CioRead(h,PAPMR+port) & ~portbit;	/* clear port bit */
	paptr =	CioRead(h,PAPTR+port) & ~portbit;
	pappr = CioRead(h,PAPPR+port) & ~portbit;
	
	switch(spec) {
	case M54_IENOFF:							/* (disable) */
		CioWrite(h,PAPMR+port,papmr);			/* clear */
		CioWrite(h,PAPTR+port,paptr);			/* clear */
		CioWrite(h,PAPPR+port,pappr);			/* clear */
		break;
	case M54_IEALL:								/* any transition */
		CioWrite(h,PAPMR+port,papmr);			/* clear */
		CioWrite(h,PAPTR+port,paptr | portbit);	/* set */
		CioWrite(h,PAPPR+port,pappr);			/* clear */
		break;
	case M54_IELH:								/* 0->1 transition */
		CioWrite(h,PAPMR+port,papmr | portbit);	/* set */
		CioWrite(h,PAPTR+port,paptr | portbit);	/* clear */
		CioWrite(h,PAPPR+port,pappr | portbit);	/* set */
		break;
	case M54_IEHL:								/* 1->0 transition */
		CioWrite(h,PAPMR+port,papmr | portbit);	/* set */
		CioWrite(h,PAPTR+port,paptr | portbit);	/* set */
		CioWrite(h,PAPPR+port,pappr);			/* clear */
		break;
	}
}




