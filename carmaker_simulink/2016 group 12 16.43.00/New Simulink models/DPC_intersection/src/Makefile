#******************************************************************************
#**  CarMaker - Version 5.0.3
#**  Vehicle Dynamic Simulation Toolkit
#**
#**  Copyright (C)   IPG Automotive GmbH
#**                  Bannwaldallee 60             Phone  +49.721.98520.0
#**                  76185 Karlsruhe              Fax    +49.721.98520.99
#**                  Germany                      WWW    http://www.ipg.de
#******************************************************************************
#**
#**  Application Makefile
#**  Run with 'make V=1 ...' for a verbose build.
#**
#******************************************************************************

include C:/IPG/hil/win32-5.0.3/include/MakeDefs

APP_VER =		"Car_Generic <insert.your.version.no>"
APP_NAME =		CarMaker.$(ARCH)$(EXE_EXT)

#OPT_CFLAGS =		-g -O1

LD_LIBS =		$(CAR_LIB) \
			$(CARMAKER_LIB) $(DRIVER_LIB) $(ROAD_LIB) $(TIRE_LIB) $(TAME_LIB)
OBJS =			CM_Main.o CM_Vehicle.o User.o
OBJS_xeno =		IO.o $(OBJS_CANIOGEN) $(OBJS_CANIOGEN_USER)
OBJS_linux =
OBJS_linux64 =
OBJS_win32 =
OBJS_win64 =

# Prepend local include/library directory to include path:
# PREINC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib

# Append local include/library directory to include path:
# INC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib


### Linking with RTW-built Simulink models

#MATSUPP_MATVER =	R2010a
#LD_LIBS +=		$(MATSUPP_LIB)

#OBJS += libSimuModel_$(ARCH).a

# @@PLUGIN-BEGIN-LIBS@@ - Automatically generated code - don't edit!
# @@PLUGIN-END@@

### END (Linking with RTW-built Simulink models)


### FlexRay
#DEF_CFLAGS +=		-DWITH_FLEXRAY

### SOME/IP
#DEF_CFLAGS +=		-DWITH_SOMEIP

### XCP / CCP
#DEF_CFLAGS +=		-DWITH_XCP
#DEF_CFLAGS +=		-DWITH_CCP


### I/O generator for CANdb files
#
# <---
#OBJS_CANIOGEN :=	IO_CAN.o
#OBJS_CANIOGEN_USER :=	$(OBJS_CANIOGEN:.o=_User.o)
#DEF_CFLAGS +=		-DWITH_IO_CAN
# <---
# I/O Module IO_CAN.o
#IO_CAN_DB =	../CANdb/CANiogenGeneric.dbc
#IO_CAN_FLAGS =	-srvECU ESP
# <---
#
### END (I/O generator for CANdb files)


default:	$(APP_NAME)
$(APP_NAME):	$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK) app_tmp.o
	$(QECHO) " LD     $@"
	$Q $(CC) $(CFLAGS) $(LDFLAGS) -o $@ \
		$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS) $(LD_LIBS_OS) \
		app_tmp.o
	$(SET_EXE_PERMISSIONS) $@


install: $(APP_NAME)
	$(INSTALL_APP) $(APP_NAME) $(ARCH)

clean:
	-rm -f 	*~ *% *.o core

app_tmp.c: Makefile $(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK)
	$(QECHO) " MK     $@"
	$Q $(CREATE_INFO_CMD)

depend .depend: Makefile
	$(QECHO) " MK     $@"
	$Q-$(CC)  $(CFLAGS)  $(DEPCFLAGS)  *.c > .depend 2>/dev/null
#	$Q-$(CCC) $(CCFLAGS) $(DEPCCFLAGS) *.cc >> .depend 2>/dev/null
include .depend

%.depend_caniogen:	Makefile
	$(QECHO) " MK     $@"
	$Q $(CANIOGEN) -mkrules "$*.o" > $@
ifneq ($(OBJS_CANIOGEN),)
include $(OBJS_CANIOGEN:.o=.depend_caniogen)
endif
