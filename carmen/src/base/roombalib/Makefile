include ../../Makefile.conf

ifndef CYGWIN
CFLAGS += -fPIC
endif

LFLAGS += -lcarmenserial -lglobal -lipc -lparam_interface

MODULE_NAME = "ROOMBA"
MODULE_COMMENT = "Program for controlling irobot Roomba"

SOURCES = roomba_lib.c
PUBLIC_INCLUDES =
PUBLIC_LIBRARIES = libroomba.a
PUBLIC_BINARIES = roomba

TARGETS = libroomba.a

libroomba.a: roomba_lib.o

include ../../Makefile.rules
