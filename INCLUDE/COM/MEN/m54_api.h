/***********************  I n c l u d e  -  F i l e  ************************
 *  
 *         Name: m54_api.h
 *
 *       Author: kp
 * 
 *  Description: M54_API header file
 *
 *     Switches: 
 *
 *---------------------------------------------------------------------------
 * Copyright 2001-2019, MEN Mikro Elektronik GmbH
 ****************************************************************************/
/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _M54_API_H
#define _M54_API_H

#ifdef __cplusplus
	extern "C" {
#endif

/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/

/*--------------------------------------+
|   TYPEDEFS                            |
+--------------------------------------*/
/*--------------------------------------+
|   EXTERNALS                           |
+--------------------------------------*/
/*--------------------------------------+
|   GLOBALS                             |
+--------------------------------------*/
/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/

int32 __MAPILIB M54_LmReset( MDIS_PATH path );
int32 __MAPILIB M54_LmDefineHome(	MDIS_PATH path );
int32 __MAPILIB M54_LmSetIndexPosition( MDIS_PATH path );
int32 __MAPILIB M54_LmLoadPositionErrorForInt( MDIS_PATH path, u_int16 thres );
int32 __MAPILIB M54_LmLoadPositionErrorForStop( MDIS_PATH path, u_int16 thres );
int32 __MAPILIB M54_LmSetBreakpointAbsolute( MDIS_PATH path, int32 bkpoint );
int32 __MAPILIB M54_LmSetBreakpointRelative( MDIS_PATH path, int32 bkpoint );
int32 __MAPILIB M54_LmMaskInterrupts( MDIS_PATH path, u_int16 mask );
int32 __MAPILIB M54_LmResetInterrupts( MDIS_PATH path, u_int16 mask );
int32 __MAPILIB M54_LmLoadFilterParameters(
	MDIS_PATH path,
	u_int16 control,
	u_int16 kp,
	u_int16 ki,
	u_int16 kd,
	u_int16 il,
	u_int8  si);
int32 __MAPILIB M54_LmUpdateFilter( MDIS_PATH path );
int32 __MAPILIB M54_LmLoadTrajectoryParameters(
	MDIS_PATH path,
	u_int16 control,
	u_int32 acc,
	u_int32 vel,
	int32 pos);
int32 __MAPILIB M54_LmStartMotion( MDIS_PATH path );
int32 __MAPILIB M54_LmReadStatus( MDIS_PATH path, u_int32 *statusP );
int32 __MAPILIB M54_LmReadSignalsRegister( MDIS_PATH path, u_int16 *sigsP);
int32 __MAPILIB M54_LmReadIndexPosition(
	MDIS_PATH path,
	int32 *posP);
int32 __MAPILIB M54_LmReadDesiredPosition(
	MDIS_PATH path,
	int32 *posP);
int32 __MAPILIB M54_LmReadRealPosition(
	MDIS_PATH path,
	int32 *posP);
int32 __MAPILIB M54_LmReadDesiredVelocity(
	MDIS_PATH path,
	int32 *velP);
int32 __MAPILIB M54_LmReadRealVelocity(
	MDIS_PATH path,
	int16 *velP);
int32 __MAPILIB M54_LmReadIntegrationSum( MDIS_PATH path, u_int16 *sumP);

#ifdef __cplusplus
	}
#endif

#endif	/* _M54_API_H */



