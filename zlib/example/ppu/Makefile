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

PROGRAM_ppu := minigzip

CC_OPT_LEVEL := -O5
#CC_OPT_LEVEL := -g

#CFLAGS += -DPERFORMANCE_MEASURE

########################################################################
#			Local Defines
########################################################################

IMPORTS	:= ../../src/ppu/libz_ppu.a ../spu_compress/lib_minigzip_spu_compress.a ../spu_decompress/lib_minigzip_spu_decompress.a -lspe2

INCLUDE := -I ../../include/ppu
ifdef CELL_TOP
INCLUDE += -I $(CELL_TOP)/src/lib/misc
else
INCLUDE += -I /opt/ibm/cell-sdk/prototype/src/lib/misc
endif

########################################################################
#			make.footer
########################################################################

ifdef CELL_TOP
	include $(CELL_TOP)/buildutils/make.footer
else
	include /opt/cell/sdk/buildutils/make.footer
endif

