
/** @addtogroup global libglobal **/
// @{

/** \file ipc_wrapper.h
 * \brief Definition of the wrapped IPC functions for CARMEN in libglobal.
 *
 * This file specifies the wrapped IPC functions for CARMEN. IPC is
 * the communication infrastructure used to communicate messages in
 * CARMEN.
 **/


#ifndef CARMEN_GENERIC_INTERFACE_H
#define CARMEN_GENERIC_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>

typedef enum {CARMEN_UNSUBSCRIBE, 
	      CARMEN_SUBSCRIBE_LATEST, 
	      CARMEN_SUBSCRIBE_ALL} carmen_subscribe_t;

typedef void (*carmen_handler_t)(void *);

extern MSG_INSTANCE current_msgRef;

  /** carmen_subscribe_message - generic IPC subscribe function.  It attaches 
     a callback and a memory destination to a particular IPC message. **/
void 
carmen_subscribe_message(char *message_name, char *message_fmt, 
			 void *message_mem, int message_size, 
			 carmen_handler_t handler, carmen_subscribe_t subscribe_how);

  /** carmen_unsubscribe_message - Generic IPC unsubscribe message function. **/
void
carmen_unsubscribe_message(char *message_name, carmen_handler_t handler);

void 
carmen_ipc_subscribe_fd(int fd, carmen_handler_t handler);

void 
carmen_ipc_unsubscribe_fd(int fd, carmen_handler_t handler);

int 
carmen_ipc_connect_locked(char *module_name);

int
carmen_ipc_connect(char *module_name);

void
carmen_ipc_initialize_locked(int argc, char **argv);

void 
carmen_ipc_initialize_locked_with_name(int argc, char **argv, char *name);

void 
carmen_ipc_initialize(int argc, char **argv);

void 
carmen_ipc_dispatch(void);

void 
carmen_ipc_sleep(double timeout);

void 
carmen_ipc_disconnect(void);

void
carmen_ipc_addPeriodicTimer(double interval, TIMER_HANDLER_TYPE handler,
			 void *clientData);

#define carmen_test_ipc_return(ERR, ERR_MSG, IPC_MSG) do {carmen_test_ipc((ERR), (ERR_MSG), (IPC_MSG)); if ((ERR) != IPC_OK) return; } while (0)

#define carmen_test_ipc_return_int(ERR, ERR_MSG, IPC_MSG) do {carmen_test_ipc((ERR), (ERR_MSG), (IPC_MSG)); if ((ERR) != IPC_OK) return -1; } while (0)

#define carmen_test_ipc_return_null(ERR, ERR_MSG, IPC_MSG) do {carmen_test_ipc((ERR), (ERR_MSG), (IPC_MSG)); if ((ERR) != IPC_OK) return NULL; } while (0)

#define carmen_test_ipc_exit(ERR, ERR_MSG, IPC_MSG) do {carmen_test_ipc((ERR), (ERR_MSG), (IPC_MSG)); if ((ERR) != IPC_OK) {fprintf(stderr, "This is a fatal error. Exiting.\n"); exit(-1);} } while (0)

#define carmen_ipc_define_test_exit(name,fmt) { IPC_RETURN_TYPE err; err = IPC_defineMsg(name, IPC_VARIABLE_LENGTH, fmt); carmen_test_ipc_exit(err, "Could not define", name); }

#define carmen_ipc_publish_exit(name,msg) { IPC_RETURN_TYPE err; err = IPC_publishData(name, &msg); carmen_test_ipc_exit(err, "Could not publish", name); }

#ifdef __cplusplus
}
#endif

#endif
// @}
