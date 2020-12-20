# --------------------------------------------------------------  
# (C)Copyright 2001,2006,                                         
# International Business Machines Corporation,                    
# Sony Computer Entertainment, Incorporated,                      
# Toshiba Corporation,                                            
#                                                                 
# All Rights Reserved.                                            
# --------------------------------------------------------------  
# PROLOG END TAG zYx

include ../Makefile.defs


########################################################################
#			Target
########################################################################

PROGRAM_ppu 	:= mpeg2decode


########################################################################
#			Local Defines
########################################################################

CFLAGS 		+= $(USE_DISP) $(USE_SHMEM) $(INCLUDEDIR) $(TRACE) $(VERBOSE) $(VERIFY) $(WARNINGS) $(PROFILE) $(TIME_PERF)
CFLAGS		+= -s
LIBS            += -lm
INCLUDE		+= -I. -I..
IMPORTS 	= -lspe2 ../spu/mpeg2decode_spu.a -lmisc $(LIBS)


########################################################################
#			make.footer
########################################################################

ifdef CELL_TOP
        include $(CELL_TOP)/buildutils/make.footer
else
        include /opt/cell/sdk/buildutils/make.footer
endif

