#***************************  M a k e f i l e  *******************************
#  
#         Author: kp
#          $Date: 2001/09/26 10:40:18 $
#      $Revision: 1.2 $
#                      
#    Description: Makefile descriptor file for M54_API lib
#                      
#-----------------------------------------------------------------------------
#   Copyright (c) 2001-2019, MEN Mikro Elektronik GmbH
#*****************************************************************************
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

MAK_NAME=m54_api


MAK_INCL=$(MEN_INC_DIR)/men_typs.h    	\
		 $(MEN_INC_DIR)/mdis_err.h		\
         $(MEN_INC_DIR)/mdis_api.h		\
		 $(MEN_INC_DIR)/m54_api.h		\
		 $(MEN_INC_DIR)/m54_drv.h		\
		 $(MEN_INC_DIR)/lm628.h			\


MAK_INP1 = m54_api$(INP_SUFFIX)

MAK_INP  = $(MAK_INP1)
