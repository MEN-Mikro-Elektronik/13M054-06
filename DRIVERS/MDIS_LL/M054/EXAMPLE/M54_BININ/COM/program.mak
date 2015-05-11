#***************************  M a k e f i l e  *******************************
#
#         Author: kp
#          $Date: 2004/04/19 15:56:45 $
#      $Revision: 1.2 $
#
#    Description: Makefile definitions for the M54 example program
#
#---------------------------------[ History ]---------------------------------
#
#   $Log: program.mak,v $
#   Revision 1.2  2004/04/19 15:56:45  cs
#   Minor modifications for MDIS4/2004 conformity
#         removed unnecessary include file mdis_err.h
#         added include file usr_oss.h
#
#   Revision 1.1  2001/09/25 16:27:44  kp
#   Initial Revision
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2001 by MEN mikro elektronik GmbH, Nuernberg, Germany
#*****************************************************************************

MAK_NAME=m54_binin

MAK_LIBS=$(LIB_PREFIX)$(MEN_LIB_DIR)/mdis_api$(LIB_SUFFIX)	\
		 $(LIB_PREFIX)$(MEN_LIB_DIR)/usr_oss$(LIB_SUFFIX)	\

MAK_INCL=$(MEN_INC_DIR)/m54_drv.h	\
         $(MEN_INC_DIR)/men_typs.h	\
         $(MEN_INC_DIR)/mdis_api.h	\
         $(MEN_INC_DIR)/usr_oss.h	\

MAK_INP1=m54_binin$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
