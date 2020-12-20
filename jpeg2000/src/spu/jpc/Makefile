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

LIBRARY_spu	:= libjasper_spu_jpc.a

ifdef DEBUG
CC_OPT_LEVEL   := -g
else
CC_OPT_LEVEL   := -O5
endif

CFLAGS += -DSPU
CFLAGS += -DFLOAT_MODE
CFLAGS += -DSIMD_EN #SIMD_EN requires FLOAT_MODE
CFLAGS += -DEN_INTERLEAVING #EN_INTERLEAVING requires FLOAT_MODE and SIMD_EN
CFLAGS += -DEN_OVERLAP #EN_OVERLAP requires FLOAT_MODE, SIMD_EN and EN_INTERLEAVING
#CFLAGS +=-fstack-check

########################################################################
#                       Local Defines
########################################################################

INCLUDE         = -I ../../../include -I ../../../include/spu -I ../../include/jpc
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
