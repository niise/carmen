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

/* Standard include file */

#ifndef CARMEN_H
#define CARMEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#ifndef va_copy
#define va_copy __va_copy
#endif 

#include <unistd.h>
#include <ctype.h>
#ifdef __USE_BSD
#undef __USE_BSD
#include <string.h>
#define __USE_BSD
#else
#include <string.h>
#endif
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include <limits.h>
#include <float.h>
#ifndef MAXDOUBLE
#define MAXDOUBLE DBL_MAX
#endif
#ifndef MAXFLOAT
#define MAXFLOAT FLT_MAX
#endif

#ifdef EXTERNAL_IPC
#include <ipc.h>
#else
#include <carmen/ipc.h>
#endif

#include <carmen/global.h>
#include <carmen/serial.h>

#include <carmen/pswrap.h>

#include <carmen/base_interface.h>

#include <carmen/param_interface.h>

#include <carmen/simulator_messages.h>

#include <carmen/laser_messages.h>
#include <carmen/laser_interface.h>

#include <carmen/robot_messages.h>
#include <carmen/robot_interface.h>

#include <carmen/keyctrl.h>

#include <carmen/map.h>
#include <carmen/map_messages.h>
#include <carmen/map_interface.h>
#include <carmen/map_util.h>

#include <carmen/geometry.h>

#include <carmen/localizecore.h>
#include <carmen/localize_messages.h>
#include <carmen/localize_interface.h>

#include <carmen/navigator_interface.h>
#include <carmen/navigator_messages.h>

#include <carmen/logger.h>
#include <carmen/logger_messages.h>
#include <carmen/logger_io.h>
#include <carmen/playback_interface.h>
#include <carmen/playback_messages.h>

#include <carmen/simulator_interface.h>

#ifdef CARMEN_MEMORY_DEBUG
#include <dmalloc.h>
#endif

#ifdef __cplusplus
}
#endif

#endif