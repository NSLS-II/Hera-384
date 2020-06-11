TOP=..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

PROD_HOST += testenable
testenable_SRCS	+= testenable.c
testenable_LIBS += ezca
testenable_LIBS	+= $(EPICS_BASE_HOST_LIBS)

PROD_HOST += testdisable
testdisable_SRCS += testdisable.c
testdisable_LIBS += ezca
testdisable_LIBS += $(EPICS_BASE_HOST_LIBS)

PROD_HOST += testallon
testallon_SRCS	+= testallon.c
testallon_LIBS	+= ezca
testallon_LIBS	+= $(EPICS_BASE_HOST_LIBS)

PROD_HOST += testalloff
testalloff_SRCS	+= testalloff.c
testalloff_LIBS	+= ezca
testalloff_LIBS	+= $(EPICS_BASE_HOST_LIBS)

PROD_HOST += enablealloff
enablealloff_SRCS	+= enablealloff.c
enablealloff_LIBS	+= ezca
enablealloff_LIBS	+= $(EPICS_BASE_HOST_LIBS)

PROD_HOST += enableallon
enableallon_SRCS	+= enableallon.c
enableallon_LIBS	+= ezca
enableallon_LIBS	+= $(EPICS_BASE_HOST_LIBS)

PROD_HOST += chandisable
chandisable_SRCS	+= chandisable.c
chandisable_LIBS	+= ezca
chandisable_LIBS	+= $(EPICS_BASE_HOST_LIBS)

PROD_HOST += chanenable
chanenable_SRCS	+= chanenable.c
chanenable_LIBS	+= ezca
chanenable_LIBS	+= $(EPICS_BASE_HOST_LIBS)

PROD_HOST += getudpevents
getudpevents_SRCS	+= getudpevents.c
getudpevents_LIBS	+= ezca
getudpevents_LIBS	+= $(EPICS_BASE_HOST_LIBS)

PROD_HOST += loadcals
loadcals_SRCS	+= loadcals.c
loadcals_LIBS	+= ezca
loadcals_LIBS	+= $(EPICS_BASE_HOST_LIBS)


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

