#***************************  M a k e f i l e  *******************************
#  
#         Author: kp
#                      
#    Description: Makefile descriptor file for M54_API lib
#                      
#-----------------------------------------------------------------------------
#   Copyright (c) 2001-2019, MEN Mikro Elektronik GmbH
#*****************************************************************************
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

MAK_NAME=m54_api
# the next line is updated during the MDIS installation
STAMPED_REVISION="13M054-06_02_03-6-ga4fbaab-dirty_2019-05-29"

DEF_REVISION=MAK_REVISION=$(STAMPED_REVISION)
MAK_SWITCH=$(SW_PREFIX)$(DEF_REVISION)


MAK_INCL=$(MEN_INC_DIR)/men_typs.h    	\
		 $(MEN_INC_DIR)/mdis_err.h		\
         $(MEN_INC_DIR)/mdis_api.h		\
		 $(MEN_INC_DIR)/m54_api.h		\
		 $(MEN_INC_DIR)/m54_drv.h		\
		 $(MEN_INC_DIR)/lm628.h			\


MAK_INP1 = m54_api$(INP_SUFFIX)

MAK_INP  = $(MAK_INP1)
