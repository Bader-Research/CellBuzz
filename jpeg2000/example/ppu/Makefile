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

PROGRAM_ppu := jasper-cell

ifdef DEBUG
CC_OPT_LEVEL   := -g
else
CC_OPT_LEVEL   := -O5
endif

CFLAGS += -DPPU
#CFLAGS +=-fstack-check
#LDFLAGS +=-Wl,-Map,$(PROGRAM_ppu).map

########################################################################
#			Local Defines
########################################################################

IMPORTS	:= ../../src/ppu/base/libjasper_base.a ../../src/ppu/bmp/libjasper_bmp.a ../../src/ppu/jp2/libjasper_jp2.a ../../src/ppu/jpc/libjasper_jpc.a ../../src/ppu/pnm/libjasper_pnm.a ../../src/spu/main_enc/lib_jpc_enc_spu.a ../../src/spu/main_dec/lib_jpc_dec_spu.a -lm -lspe2

INCLUDE := -I ../../include -I ../../include/ppu

########################################################################
#			make.footer
########################################################################

ifdef CELL_TOP
	include $(CELL_TOP)/buildutils/make.footer
else
	include /opt/cell/sdk/buildutils/make.footer
endif

