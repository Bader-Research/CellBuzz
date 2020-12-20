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

PROGRAM_spu	:= mpeg2decode_spu
LIBRARY_embed = mpeg2decode_spu.a

########################################################################
#			Local Defines
########################################################################

CFLAGS 		+= $(INCLUDEDIR) $(PROFILE)
INCLUDE		:= -I. -I ..
LDFLAGS         = -Wl,-N
CFLAGS		+= -s $(PROFILE)
IMPORTS 	=  -lmisc 

########################################################################
#                       make.footer
########################################################################

ifdef CELL_TOP
        include $(CELL_TOP)/buildutils/make.footer
else
        include /opt/cell/sdk/buildutils/make.footer
endif

