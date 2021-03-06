/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, and Sebastian Thrun
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef CARMEN_HOKUYO_INTERFACE_H
#define CARMEN_HOKUYO_INTERFACE_H

#include <carmen/hokuyo_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_hokuyo_subscribe_pb9_message(carmen_hokuyo_pb9_message *hokuyo,
					   carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how);

void
carmen_hokuyo_subscribe_alive_message(carmen_hokuyo_alive_message *alive,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how);


#ifdef __cplusplus
}
#endif

#endif
