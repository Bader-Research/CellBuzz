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

LIBRARY_spu	:= libjasper_spu_base.a

ifdef DEBUG
CC_OPT_LEVEL   := -g
else
CC_OPT_LEVEL   := -O5
endif

CFLAGS += -DSPU
#CFLAGS +=-fstack-check

########################################################################
#                       Local Defines
########################################################################

INCLUDE         = -I ../../../include -I ../../../include/spu
ifdef CELL_TOP
INCLUDE += -I $(CELL_TOP)/src/lib/misc
INCLUDE += -I $(CELL_TOP)/src/lib/sync/spu
else
INCLUDE += -I /opt/ibm/cell-sdk/prototype/src/lib/misc
INCLUDE += -I /opt/ibm/cell-sdk/prototype/src/lib/sync/spu
endif

########################################################################
#                       make.footer
########################################################################

ifdef CELL_TOP
	include $(CELL_TOP)/buildutils/make.footer
else
	include /opt/cell/sdk/buildutils/make.footer
endif
