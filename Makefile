# Copyright (C) ARM Ltd 2012-2013
# Select build rules based on Windows or Unix
#######################################################################
ifdef WINDIR
DONE=@if exist $(1) echo Build completed.
RM=if exist $(1) del /q $(1)
SHELL=$(WINDIR)\system32\cmd.exe
SEP=\\
else
ifdef windir
DONE=@if exist $(1) echo Build completed.
RM=if exist $(1) del /q $(1)
SHELL=$(windir)\system32\cmd.exe
SEP=\\
else
DONE=@if [ -f $(1) ]; then echo Build completed.; fi
RM=rm -f $(1)
SEP=/
endif
endif

# Space-safe version of 'notdir' suitable for Windows
spacetoquery = $(subst $(empty) ,?,$1)
querytospace = $(subst ?, ,$1)
spacesafenotdir = $(call querytospace,$(notdir $(call spacetoquery,$1)))
#######################################################################

TARGET = $(call spacesafenotdir, $(CURDIR))

CC = armcc
CXX = armcc
AS = armasm
LD = armlink

ROOT = E:\correlation_de1_yao\dual_attena\de0_dual_antenna\GPS_C_code_xdwahaha\CMSIS_RTOS_RTX

CFLAGS = -D__CMSIS_RTOS -D__MICROLIB -D__EVAL -D__FPU_PRESENT --apcs=interwork --thumb --cpu Cortex-A9 -g -O0 --md --c99\
         -I $(ROOT)/RTOS/RTX/SRC \
         -I $(ROOT)/RTOS/RTX/INC \
         -I $(ROOT)/Include \
         -I . \
         -I ../INC \
         -I C:\altera\14.1\embedded\ip\altera\hps\altera_hps\hwlib\include 
CXXFLAGS = $(CFLAGS) --exceptions
ASFLAGS = --pd "__CMSIS_RTOS SETA 1" --pd "__EVAL SETA 1" --pd "__MICROLIB SETA 1" --pd "__FPU_PRESENT SETA 1" \
		  --apcs=interwork --cpu=Cortex-A9 -g --diag_suppress=1786
LDFLAGS = --cpu Cortex-A9 --library_type=microlib --strict --scatter scatter.scat \
          --summary_stderr --info summarysizes --map --xref --callgraph --symbols \
          --info sizes --info totals --info unused --info veneers \
          --list $(TARGET).map --entry __Vectors

#This uses sort just to strip duplicates (which will exist if the directory is already initialised)
OBJECTS = $(sort $(addsuffix .o, $(basename $(wildcard *.c)) $(basename $(wildcard *.cpp)) $(basename $(wildcard ARM/*.c)) $(basename $(wildcard ARM/*.cpp)) $(basename $(wildcard ARM/*.s))))

all: lib $(TARGET).axf

lib:
	make all -C $(ROOT)/RTOS/RTX/SRC

$(TARGET).axf: $(OBJECTS) $(ROOT)/RTOS/RTX/LIB/ARM/RTX_CA9_L.lib scatter.scat
	$(LD) $(LDFLAGS) $(OBJECTS) $(ROOT)/RTOS/RTX/LIB/ARM/RTX_CA9_L.lib -o $@

includes = $(wildcard *.h)
$(OBJECTS) : $(includes)

nearlyclean:
	$(call RM,*.o)
	$(call RM,*.d)
	$(call RM,*.map)
	$(call RM,*.lst)
	$(call RM,$(TARGET).htm)
	$(call RM,ARM$(SEP)*.o)

clean : nearlyclean
	$(call RM,*.axf)
