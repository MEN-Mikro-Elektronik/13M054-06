#***************************  M a k e f i l e  *******************************
#
#         Author: kp
#          $Date: 2004/04/19 15:56:31 $
#      $Revision: 1.3 $
#
#    Description: Makefile definitions for the M54 driver (swapped variant)
#
#---------------------------------[ History ]---------------------------------
#
#   $Log: driver_sw.mak,v $
#   Revision 1.3  2004/04/19 15:56:31  cs
#   Minor modifications for MDIS4/2004 conformity
#         removed unnecessary inclusion of mbuf.h
#
#   Revision 1.2  2001/09/26 10:40:09  kp
#   changed library order and changed id to id_sw
#
#   Revision 1.1  2001/09/25 16:27:33  kp
#   Initial Revision
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2001 by MEN mikro elektronik GmbH, Nuernberg, Germany
#*****************************************************************************

MAK_NAME=m54_sw

MAK_SWITCH=$(SW_PREFIX)MAC_MEM_MAPPED\
		 $(SW_PREFIX)M54_SW

MAK_LIBS=$(LIB_PREFIX)$(MEN_LIB_DIR)/desc$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/id_sw$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/oss$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/dbg$(LIB_SUFFIX)	\


MAK_INCL=$(MEN_INC_DIR)/m54_drv.h	\
         $(MEN_INC_DIR)/lm628.h		\
         $(MEN_INC_DIR)/z8536.h		\
         $(MEN_INC_DIR)/men_typs.h	\
         $(MEN_INC_DIR)/oss.h		\
         $(MEN_INC_DIR)/mdis_err.h	\
         $(MEN_INC_DIR)/maccess.h	\
         $(MEN_INC_DIR)/desc.h		\
         $(MEN_INC_DIR)/mdis_api.h	\
         $(MEN_INC_DIR)/mdis_com.h	\
         $(MEN_INC_DIR)/modcom.h	\
         $(MEN_INC_DIR)/ll_defs.h	\
         $(MEN_INC_DIR)/ll_entry.h	\
         $(MEN_INC_DIR)/dbg.h		\

MAK_INP1=m54_drv$(INP_SUFFIX)
MAK_INP2=

MAK_INP=$(MAK_INP1) \
        $(MAK_INP2)
