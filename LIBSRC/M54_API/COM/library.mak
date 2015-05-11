#***************************  M a k e f i l e  *******************************
#  
#         Author: kp
#          $Date: 2001/09/26 10:40:18 $
#      $Revision: 1.2 $
#                      
#    Description: Makefile descriptor file for M54_API lib
#                      
#---------------------------------[ History ]---------------------------------
#
#   $Log: library.mak,v $
#   Revision 1.2  2001/09/26 10:40:18  kp
#   fixed typo
#
#   Revision 1.1  2001/09/25 16:27:50  kp
#   Initial Revision
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2001 by MEN mikro elektronik GmbH, Nuernberg, Germany 
#*****************************************************************************

MAK_NAME=m54_api


MAK_INCL=$(MEN_INC_DIR)/men_typs.h    	\
		 $(MEN_INC_DIR)/mdis_err.h		\
         $(MEN_INC_DIR)/mdis_api.h		\
		 $(MEN_INC_DIR)/m54_api.h		\
		 $(MEN_INC_DIR)/m54_drv.h		\
		 $(MEN_INC_DIR)/lm628.h			\


MAK_INP1 = m54_api$(INP_SUFFIX)

MAK_INP  = $(MAK_INP1)
