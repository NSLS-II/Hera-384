#!../../bin/linux-arm/zDDM

## You may have to change zDDM to something else
## everywhere it appears in this file

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/zDDM.dbd"
zDDM_registerRecordDeviceDriver pdbbase
devI2CConfig(0,1,8)
##devSPIConfig(0,1,8)
devzDDMConfig(1,1,384)


## Load record instances
#dbLoadTemplate "db/user.substitutions"
dbLoadRecords "db/det1.db"
dbLoadRecords "db/i2cDacs.db"
##dbLoadRecords "db/SpiDacs.db","user=det1"


## Set this to see messages from mySub
#var mySubDebug 1

## Run this to trace the stages of iocInit
traceIocInit
var zDDMRecordDebug 5
var devzDDMdebug 5

cd "${TOP}/iocBoot/${IOC}"
epicsEnvSet EPICS_CA_MAX_ARRAY_BYTES 7000000

iocInit

dbpr det1 4 >det1_pvs.txt
## Start any sequence programs
#seq sncExample, "user=peter"
