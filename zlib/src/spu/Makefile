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

LIBRARY_spu	:= libz_spu.a

#CC_OPT_LEVEL   := -g
CC_OPT_LEVEL   := -O5

CFLAGS +=-DMAX_WBITS_COMPRESS=13 -DMAX_WBITS_DECOMPRESS=15 -DMAX_MEM_LEVEL=7
#CFLAGS +=-fstack-check

########################################################################
#                       Local Defines
########################################################################

INCLUDE = -I ../../include/spu

########################################################################
#                       make.footer
########################################################################

ifdef CELL_TOP
	include $(CELL_TOP)/buildutils/make.footer
else
	include /opt/cell/sdk/buildutils/make.footer
endif

