
# -------------------------------------------------------------- 
# (C)Copyright 2006
# International Business Machines Corporation
#
# All Rights Reserved
# -------------------------------------------------------------- 

########################################################################
#			Subdirectories
########################################################################

DIRS		:= 	

########################################################################
#			Target
########################################################################

LIBRARY_ppu	:= libjasper_jpc.a

ifdef DEBUG
CC_OPT_LEVEL   := -g
else
CC_OPT_LEVEL   := -O5
endif

CFLAGS += -DPPU
CFLAGS += -DFLOAT_MODE
#CFLAGS +=-fstack-check

########################################################################
#                       Local Defines
########################################################################

INCLUDE         = -I ../../../include -I ../../../include/ppu -I ../../include/jpc
ifdef CELL_TOP
INCLUDE += -I $(CELL_TOP)/src/lib/misc
INCLUDE += -I $(CELL_TOP)/src/lib/sync
else
INCLUDE += -I /opt/ibm/cell-sdk/prototype/src/lib/misc
INCLUDE += -I /opt/ibm/cell-sdk/prototype/src/lib/sync
endif

########################################################################
#                       make.footer
########################################################################

ifdef CELL_TOP
	include $(CELL_TOP)/buildutils/make.footer
else
	include /opt/cell/sdk/buildutils/make.footer
endif
