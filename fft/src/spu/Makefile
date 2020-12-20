# --------------------------------------------------------------  
# (C)Copyright 2001,2006,                                         
# International Business Machines Corporation,                    
# Sony Computer Entertainment, Incorporated,                      
# Toshiba Corporation,                                            
#                                                                 
# All Rights Reserved.                                            
# --------------------------------------------------------------  
# PROLOG END TAG zYx                                              

########################################################################
#			Target
########################################################################

PROGRAMS_spu	:= fft_spu
LIBRARY_embed	:= fft_spu.a
#SPU_COMPILER = xlc

CFLAGS = -DPROFILING
########################################################################
#			Local Defines
########################################################################

IMPORTS       = -lm


########################################################################
#                       make.footer
########################################################################
include ../make.defs
include $(MAKE_FOOTER)
