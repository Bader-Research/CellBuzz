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

PROGRAM_ppu	= ../fft

#PPU_COMPILER = xlc

CFLAGS = -DPROFILING #-DCHECK_C
########################################################################
#			Local Defines
########################################################################

IMPORTS 	= ../spu/fft_spu.a -lspe2 -lm


########################################################################
#			make.footer
########################################################################
include ../make.defs
include $(MAKE_FOOTER)
