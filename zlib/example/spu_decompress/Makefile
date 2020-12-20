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

PROGRAMS_spu := minigzip_spu_decompress

LIBRARY_embed := lib_minigzip_spu_decompress.a

#CC_OPT_LEVEL := -g
CC_OPT_LEVEL := -O5

#CFLAGS +=-fstack-check
#LDFLAGS +=-Wl,-Map,minigzip_spu_decompress.map

########################################################################
#			Local Defines
########################################################################

IMPORTS	:= ../../src/spu/libz_spu.a

INCLUDE := -I ../../include/spu


########################################################################
#			make.footer
########################################################################

ifdef CELL_TOP
	include $(CELL_TOP)/buildutils/make.footer
else
	include /opt/cell/sdk/buildutils/make.footer
endif

