/**********************************************************************/
 /** Brief Description of device support                              **/
 /**                                                                  **/
 /** THis support provide access to the TMP100 chips on the board, in **/
 /** particular it reads three temperatures.                          **/
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
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define FAST_LOCK epicsMutexId
#define FASTLOCKINIT(PFAST_LOCK) (*(PFAST_LOCK) = epicsMutexCreate())                                   
     
#define FASTLOCK(PFAST_LOCK) epicsMutexLock(*(PFAST_LOCK));                                             
     
#define TRYLOCK(PFAST_LOCK) epicsMutexTryLock(*(PFAST_LOCK));                                           
     
#define FASTUNLOCK(PFAST_LOCK) epicsMutexUnlock(*(PFAST_LOCK));                                         
     
#define ERROR -1                                                                                        
     
#define OK 0


#define Debug0(l,FMT) {  if(l <= devTmp100debug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT); } }
#define Debug(l,FMT,V) {  if(l <= devTmp100debug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT,V); } }

FAST_LOCK       Tmp100_lock;

volatile int  devTmp100Debug = 0;
epicsExportAddress(int, devTmp100Debug);
int card;


/* Create the dset for devTmp100 */
static long Tmp100_init_ai(), Tmp100_read_ai();

typedef struct {
        long number;
        DEVSUPFUN report;
        DEVSUPFUN init;
        DEVSUPFUN init_record;
        DEVSUPFUN get_ioint_info;
        DEVSUPFUN read_write;
        DEVSUPFUN special_linconv;
        }Tmp100dset;

Tmp100dset devAiTmp100={
                6,
                NULL,
                NULL,
                Tmp100_init_ai,
                NULL,
                Tmp100_read_ai,
                NULL
                };

epicsExportAddress(dset,devAiTmp100);
int devTmp100Config(card)
{

     FASTLOCKINIT(&(Tmp100_lock));  /* Init the board lock */      

  return(OK);
}

static const iocshArg devTmp100ConfigArg0 = {"Card #", iocshArgInt};

static const iocshArg * const devTmp100ConfigArgs[1] = {&devTmp100ConfigArg0};
                                                       
static const iocshFuncDef devTmp100ConfigFuncDef={"devTmp100Config",1,devTmp100ConfigArgs};
static void devTmp100ConfigCallFunc(const iocshArgBuf *args)
{
 devTmp100Config((int) args[0].ival);
}

void registerTmp100Config(void){
        iocshRegister(&devTmp100ConfigFuncDef,&devTmp100ConfigCallFunc);
        }

epicsExportRegistrar(registerTmp100Config);

static long Tmp100_init_ai(void *precord)
{
aiRecord *pai = (aiRecord *)precord;
long status;
char *parm;

switch (pai->inp.type) {
        case (INST_IO) : /* doesn't need any parameters */
	parm = pai->inp.value.instio.string;
	printf("parm=%s\n",parm);
	sscanf(parm,"%i",&card);
	printf("Card: %i\n",card);
	break;
        default :
                recGblRecordError(S_db_badField, (void *)pai,"devAiTmp100 (init_record) Illegal INP field"
);
        return(S_db_badField);
        }
	
    switch(card){
	case 72:{
         }
	 break;
	case 73:{
	 }
	 break;
	case 74:{
	 }
	break;
	default:{
	 recGblRecordError(S_db_badField, (void *)pai,"devAiTmp100 (init_record) Illegal card number");
	}
    }	
/* Make sure record processing routine does not perform any conversion*/
//      pai->linr=0;
return(0);
}


static long Tmp100_read_ai(void *precord)
{
  aiRecord *pai =(aiRecord *)precord;

  int card;
  int file, i, bytes;
  char *bus = "/dev/i2c-0";
  char config[2] = {0};
  char data[2] = {0};
  char reg[1] = {0x00};
  float tmp[2] = {0};
  int temp;

      sscanf(pai->inp.value.instio.string,"%i",&card);
 FASTLOCK(&Tmp100_lock);
	if ((file = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}

	if (ioctl(file, I2C_SLAVE, card) < 0) {
	   printf("Device %i not selected\n",card);
	}
	
	if(read(file, data, 2) != 2){
	    printf("Error : Input/Output error \n");
	    }
	else {
            // Convert the data to 12-bits
   	    temp = (data[0] * 256 + (data[1] & 0xF0)) / 16;
	    if (temp > 2047) {
		temp -= 4096;
		}
//   	    cTemp = temp * 0.0625;
 	}
	close(file);
  FASTUNLOCK(&Tmp100_lock);
	pai->rval=temp;
   return(OK);
}

