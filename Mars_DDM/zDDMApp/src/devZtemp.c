/**********************************************************************/
 /** Brief Description of device support                              **/
 /**                                                                  **/
 /** THis support provide access to the ADC on the ZynQ device, in    **/
 /** particular it reads the CPU die temperature.                     **/
 /** germanium detectors.                                             **/
 /**********************************************************************/
  
 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include        <stdlib.h>
#include        <stdio.h>
#include        <string.h>
#include        <math.h>
#include        <epicsTimer.h>
#include        <epicsThread.h>
#include        <epicsRingBytes.h>
#include        <epicsMutex.h>
#include        <epicsEvent.h>

#include        <alarm.h>
#include        <dbDefs.h>
#include        <dbAccess.h>
#include        <dbCommon.h>
#include        <dbScan.h>
#include        <recGbl.h>
#include        <recSup.h>
#include        <devSup.h>

#include        <special.h>

#include        <aoRecord.h>
#include        <aiRecord.h>
#include        <boRecord.h>
#include        <biRecord.h>
#include        <longinRecord.h>
#include        <longoutRecord.h>
#include        <mbboRecord.h>                                                                          
#include        <mbbiRecord.h>                                                                          

#include        <link.h>                                                                                
#include        <iocsh.h>                                                                               
#include        <epicsExport.h>                                                                         

#include <errno.h>
#include <errlog.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


#define FAST_LOCK epicsMutexId
#define FASTLOCKINIT(PFAST_LOCK) (*(PFAST_LOCK) = epicsMutexCreate())                                   
     
#define FASTLOCK(PFAST_LOCK) epicsMutexLock(*(PFAST_LOCK));                                             
     
#define TRYLOCK(PFAST_LOCK) epicsMutexTryLock(*(PFAST_LOCK));                                           
     
#define FASTUNLOCK(PFAST_LOCK) epicsMutexUnlock(*(PFAST_LOCK));                                         
     
#define ERROR -1                                                                                        
     
#define OK 0


#define Debug0(l,FMT) {  if(l <= devZTMPdebug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT); } }
#define Debug(l,FMT,V) {  if(l <= devZTMPdebug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT,V); } }

/* FAST_LOCK       ZTMP_lock; */
volatile int  devZTMPDebug = 0;
epicsExportAddress(int, devZTMPDebug);


/* Create the dset for devZTMP */
static long ZTMP_init_ai(), ZTMP_read_ai();

typedef struct {
        long number;
        DEVSUPFUN report;
        DEVSUPFUN init;
        DEVSUPFUN init_record;
        DEVSUPFUN get_ioint_info;
        DEVSUPFUN read_write;
        DEVSUPFUN special_linconv;
        }ZTMPdset;

ZTMPdset devAiZTMP={
                6,
                NULL,
                NULL,
                ZTMP_init_ai,
                NULL,
                ZTMP_read_ai,
                NULL
                };

epicsExportAddress(dset,devAiZTMP);

int devZTMPConfig(card)
{

/*      FASTLOCKINIT(&(ZTMP_lock)); */ /* Init the board lock */      

  return(OK);
}

static const iocshArg devZTMPConfigArg0 = {"Card #", iocshArgInt};

static const iocshArg * const devZTMPConfigArgs[1] = {&devZTMPConfigArg0};
                                                       
static const iocshFuncDef devZTMPConfigFuncDef={"devZTMPConfig",1,devZTMPConfigArgs};
static void devZTMPConfigCallFunc(const iocshArgBuf *args)
{
 devZTMPConfig((int) args[0].ival);
}

void registerZTMPConfig(void){
        iocshRegister(&devZTMPConfigFuncDef,&devZTMPConfigCallFunc);
        }

epicsExportRegistrar(registerZTMPConfig);

static long ZTMP_init_ai(void *precord)
{
aiRecord *pai = (aiRecord *)precord;
long status;
int                 card;
/*ZTMPDpvt         *pPvt;*/

switch (pai->inp.type) {
        case (INST_IO) : /* doesn't need any parameters */
        break;
	default :
                recGblRecordError(S_db_badField, (void *)pai,"devAiZTMP (init_record) Illegal INP field"
);
        return(S_db_badField);
        }
/* Make sure record processing routine does not perform any conversion*/
//      pai->linr=0;
return(0);
}


static long ZTMP_read_ai(void *precord)
{
  aiRecord *pai =(aiRecord *)precord;
  int t_raw, t_offset;
  double t_scale;
  
  FILE* ztemp_raw = fopen("/sys/bus/platform/drivers/xadc/f8007100.adc/iio:device0/in_temp0_raw", "r");
  FILE* ztemp_scale = fopen("/sys/bus/platform/drivers/xadc/f8007100.adc/iio:device0/in_temp0_scale","r");
  FILE* ztemp_offset = fopen("/sys/bus/platform/drivers/xadc/f8007100.adc/iio:device0/in_temp0_offset", "r");

  fscanf(ztemp_raw,"%d",&t_raw);
  fscanf(ztemp_offset,"%d",&t_offset);
  fscanf(ztemp_scale,"%lf",&t_scale);

  fclose(ztemp_raw);
  fclose(ztemp_scale);
  fclose(ztemp_offset);
  if (devZTMPDebug >= 20)
          errlogPrintf("ZTMP_read_ai: t_raw=%i t_offset=%i t_scale=%g\n\r", t_raw, t_offset, t_scale);

  pai->rval = t_raw;


  return(OK);
}

