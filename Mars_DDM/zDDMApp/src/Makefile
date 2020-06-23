TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += zDDMSupport

# xxxRecord.h will be created from xxxRecord.dbd
DBDINC += zDDMRecord
# install devXxxSoft.dbd into <top>/dbd
#DBD += zDDMSupport.dbd

# The following are compiled and added to the Support library
zDDMSupport_SRCS += devzDDM.c
zDDMSupport_SRCS += zDDMRecord.c
zDDMSupport_SRCS += devTmp100.c
zDDMSupport_SRCS += devZtemp.c
zDDMSupport_SRCS += devI2C.c
zDDMSupport_SRCS += pl_lib.c
zDDMSupport_LIBS += $(EPICS_BASE_IOC_LIBS)
zDDM_SYS_LIBS += readline
#zDDM_SYS_LIBS += zmq

#=============================
# build an ioc application

PROD_IOC = zDDM
# <name>.dbd will be created from <name>Include.dbd
DBD += zDDM.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
zDDM_SRCS += zDDM_registerRecordDeviceDriver.cpp
zDDM_SRCS_DEFAULT += zDDMMain.cpp
zDDM_SRCS_vxWorks += -nil-

# Add locally compiled object code
#zDDM_SRCS += dbSubExample.c

zDDM_LIBS += zDDMSupport

# NOTES:
#      1)It is not possible to build sncExample both as a component of zDDM
#        and standalone. You must choose only one.
#      2)To build sncExample SNCSEQ must be defined in <top>/configure/RELEASE

# The following builds sncExample as a component of zDDM
#zDDM_SRCS += sncExample.stt
#zDDM_LIBS += seq pv

zDDM_LIBS += $(EPICS_BASE_IOC_LIBS)

# The following builds sncExample as a standalone application
#PROD_HOST += sncExample
#sncExample_SNCFLAGS += +m
#sncExample_SRCS += sncExample.stt
#sncExample_LIBS += seq pv
#sncExample_LIBS += $(EPICS_BASE_HOST_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

