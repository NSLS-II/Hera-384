/************************************************************************
* Record support for BNL germanium multi-element germanium detectors    *
*                                                                       *
* D. P Siddons, Jan. 2016.                                              *
*                                                                       *
* Provides controls for mars and mars ASICs used in                *
* 64- and 384-element strip detectors based on Maia ASIC configuration. *
*                                                                       *
*************************************************************************/

#define VERSION 1.0

#include        <epicsVersion.h>

#if EPICS_VERSION < 3 || (EPICS_VERSION==3 && EPICS_REVISION < 14)
#define NOT_YET_OSI
#endif

#if defined(NOT_YET_OSI) || defined(VXWORKSTARGET)
#include	<vxWorks.h>
#ifndef __vxworks
#define __vxworks 1
#endif
#include	<types.h>
#include	<stdio.h>
#include	<unistd.h>
#include	<stdioLib.h>
#include	<lstLib.h>
#include	<string.h>
#include	<wdLib.h>
#else
#include        <epicsTimer.h>

extern epicsTimerQueueId	zDDMWdTimerQ;

#endif

#include	<stdio.h>
#include	<stdlib.h>
#include	<alarm.h>
#include	<callback.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include        <dbScan.h>
#include        <dbEvent.h>
#include	<dbFldTypes.h>
#include	<errMdef.h>
#include	<recSup.h>
#include        <recGbl.h>
#include	<devSup.h>
#include	<special.h>
#include 	<iocsh.h>
#include	<epicsExport.h>
#define GEN_SIZE_OFFSET
#include	"zDDMRecord.h"
#undef GEN_SIZE_OFFSET
#include	"devzDDM.h"
#include	<zmq.h>

#define zDDM_STATE_IDLE 0
#define zDDM_STATE_WAITING 1
#define zDDM_STATE_COUNTING 2

#define USER_STATE_IDLE 0
#define USER_STATE_WAITING 1
#define USER_STATE_REQSTART 2
#define USER_STATE_COUNTING 3

#define MARS_CONF_LOAD 0
#define LEDS 1
#define MARS_CONFIG 2
#define VERSIONREG 3
#define MARS_CALPULSE 4
#define MARS_PIPE_DELAY 5
#define MARS_RDOUT_ENB 8 
#define EVENT_TIME_CNTR 9
#define SIM_EVT_SEL 10
#define SIM_EVENT_RATE 11
#define ADC_SPI 12
#define CALPULSE_CNT 16
#define CALPULSE_RATE 17
#define CALPULSE_WIDTH 18
#define CALPULSE_MODE 19
#define TD_CAL 20
#define EVENT_FIFO_DATA 24
#define EVENT_FIFO_CNT 25
#define EVENT_FIFO_CNTRL 26
#define DMA_CONTROL 32
#define DMA_STATUS 33
#define DMA_BASEADDR 34
#define DMA_BURSTLEN 35
#define DMA_BUFLEN 36
#define DMA_CURADDR 37
#define DMA_THROTTLE 38
#define DMA_IRQ_THROTTLE 48
#define DMA_IRQ_ENABLE 49
#define DMA_IRQ_COUNT 50
#define TRIG  52
#define COUNT_TIME 53
#define FRAME_NO  54
#define COUNT_MODE 55

#ifdef NODEBUG
#define Debug(l,FMT,V) ;
#else
#define Debug(l,FMT,V) {  if(l <= zDDMRecordDebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT,V); } }
#endif

int I2Cdev; /* The i2c device file name */

volatile int zDDMRecordDebug = 0;
epicsExportAddress(int,zDDMRecordDebug);
volatile int zDDM_wait_time = 3;
epicsExportAddress(int,zDDM_wait_time);

#define MIN(a,b) (a)<(b)?(a):(b)
#define MAX(a,b) (a)>(b)?(a):(b)

volatile int zDDM_NCHAN;
volatile int zDDM_NCHIPS;

extern det_state zDDM_state;
extern int stuff_mars();

/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long init_record(zDDMRecord *, int);
static long process(zDDMRecord *);
static long special();
#define get_value NULL
static long cvt_dbaddr(DBADDR *);
static long get_array_info(DBADDR *, long *, long *);
#define put_array_info NULL
#define get_units NULL
static long get_precision(DBADDR *, long *);
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
#define get_graphic_double NULL
#define get_control_double NULL
#define get_alarm_double NULL

rset zDDMRSET = {
	RSETNUMBER,
	report,
	initialize,
	init_record,
	process,
	special,
	get_value,
	cvt_dbaddr,
	get_array_info,
	put_array_info,
	get_units,
	get_precision,
	get_enum_str,
	get_enum_strs,
	put_enum_str,
	get_graphic_double,
	get_control_double,
	get_alarm_double
};
epicsExportAddress(rset,zDDMRSET);

extern void *context;
extern void *requester;

extern int *fpgabase;

static void do_alarm();
static void monitor();
static void updateCounts(zDDMRecord *pscal);
static long init_record(zDDMRecord *pscal, int pass);

static void deviceCallbackFunc(CALLBACK *pcb)
{
        zDDMRecord *pscal;

        callbackGetUser(pscal, pcb);
        Debug(5,"scaler deviceCallbackFunc: entry for '%s'\n", pscal->name);
        dbScanLock((struct dbCommon *)pscal);
        process((struct zDDMRecord *)pscal);
        dbScanUnlock((struct dbCommon *)pscal);
}


static void updateCallbackFunc(CALLBACK *pcb)
{
        zDDMRecord *pscal;
        struct rpvtStruct *prpvt;

        callbackGetUser(pscal, pcb);
        Debug(5, "scaler updateCallbackFunc: entry for '%s'\n", pscal->name);
        prpvt = (struct rpvtStruct *)pscal->rpvt;
        epicsMutexLock(prpvt->updateMutex);
        updateCounts(pscal);
        epicsMutexUnlock(prpvt->updateMutex);
}

static void delayCallbackFunc(CALLBACK *pcb)
{
        zDDMRecord *pscal;

        /*
         * User asked us to start counting after a delay that has now expired.
         * If user has not rescinded that request in the meantime, tell
         * process() to start counting.
         */
        callbackGetUser(pscal, pcb);
        Debug(5, "scaler delayCallbackFunc: entry for '%s'\n", pscal->name);
        if (pscal->us == USER_STATE_WAITING && pscal->cnt) {
                pscal->us = USER_STATE_REQSTART;
                (void)scanOnce((void *)pscal);
        }
}

static void autoCallbackFunc(CALLBACK *pcb)
{
        zDDMRecord *pscal;

        callbackGetUser(pscal, pcb);
        Debug(5, "scaler autoCallbackFunc: entry for '%s'\n", pscal->name);
        (void)scanOnce((void *)pscal);
}



static long cvt_dbaddr(paddr)
struct dbAddr *paddr;
{
   int index;
   index=dbGetFieldIndex(paddr);
   
   zDDMRecord *pzDDM=(zDDMRecord *)paddr->precord;
   switch(index){
   case zDDMRecordMCA:{
      paddr->pfield = (void *)(pzDDM->pmca);
      paddr->no_elements = zDDM_NCHAN * 4096; /*1572864;  = 384 x 4096, Energy spectra accumulated for real-time display */
      paddr->field_type = DBF_LONG;
      paddr->field_size = sizeof(int);
      paddr->dbr_field_type = DBR_LONG;
      break;
      }
   case zDDMRecordTDC:{
      paddr->pfield = (void *)(pzDDM->ptdc);
      paddr->no_elements = zDDM_NCHAN * 1024; /* 393216; = 384 x 1024, TDC spectra accumulated for real-time display */
      paddr->field_type = DBF_LONG;
      paddr->field_size = sizeof(int);
      paddr->dbr_field_type = DBR_LONG;
      break;
      }
   case zDDMRecordSPCT:{
      paddr->pfield = (void *)(pzDDM->pspct);
      paddr->no_elements = 4096; /* One spectrum for real-time display */
      paddr->field_type = DBF_LONG;
      paddr->field_size = sizeof(int);
      paddr->dbr_field_type = DBR_LONG;
      break;
      }
    case zDDMRecordSPCTX:{
      paddr->pfield = (void *)(pzDDM->pspctx);
      paddr->no_elements = 4096; /*Calibrated X-axis values for real-time display */
      paddr->field_type = DBF_FLOAT;
      paddr->field_size = sizeof(int);
      paddr->dbr_field_type = DBR_FLOAT;
      break;
      }
   case zDDMRecordCHEN:{
      paddr->pfield = (void *)(pzDDM->pchen);
      paddr->no_elements = pzDDM->nelm;
      paddr->field_type = DBF_CHAR;
      paddr->field_size = sizeof(char);
      paddr->dbr_field_type = DBR_CHAR;
      paddr->special = SPC_MOD;
      break;
      }
    case zDDMRecordSLP:{
      paddr->pfield = (void *)(pzDDM->pslp);
      paddr->no_elements = pzDDM->nelm;
      paddr->field_type = DBF_FLOAT;
      paddr->field_size = sizeof(float);
      paddr->dbr_field_type = DBR_FLOAT;
      paddr->special = SPC_MOD;
      break;
      }
    case zDDMRecordOFFS:{
      paddr->pfield = (void *)(pzDDM->poffs);
      paddr->no_elements = pzDDM->nelm;
      paddr->field_type = DBF_FLOAT;
      paddr->field_size = sizeof(float);
      paddr->dbr_field_type = DBR_FLOAT;
      paddr->special = SPC_MOD;
      break;
      }
    case zDDMRecordTSEN:{
      paddr->pfield = (void *)(pzDDM->ptsen);
      paddr->no_elements = pzDDM->nelm;
      paddr->field_type = DBF_CHAR;
      paddr->field_size = sizeof(char);
      paddr->dbr_field_type = DBR_CHAR;
      paddr->special = SPC_MOD;
      break;
      }
    case zDDMRecordPUTR:{
      paddr->pfield = (void *)(pzDDM->pputr);
      paddr->no_elements = pzDDM->nelm;
      paddr->field_type = DBF_CHAR;
      paddr->field_size = sizeof(char);
      paddr->dbr_field_type = DBR_CHAR;
      paddr->special = SPC_MOD;
      break;
      }
   case zDDMRecordTHTR:{
      paddr->pfield = (void *)(pzDDM->pthtr);
      paddr->no_elements = pzDDM->nelm;
      paddr->field_type = DBF_CHAR;
      paddr->field_size = sizeof(char);
      paddr->dbr_field_type = DBR_CHAR;
      paddr->special = SPC_MOD;
      break;
      }
    case zDDMRecordTHRSH:{
      paddr->pfield = (void *)(pzDDM->pthrsh);
      paddr->no_elements = zDDM_NCHIPS;
      paddr->field_type = DBF_LONG;
      paddr->field_size = sizeof(int);
      paddr->dbr_field_type = DBR_LONG;
      paddr->special = SPC_MOD;
      break;
      }
      
    }
   return(0);
}


static long get_array_info(DBADDR *paddr, long *no_elements, long *offset)
{
   int index;
   index=dbGetFieldIndex(paddr);
 
   zDDMRecord *pzDDM=(zDDMRecord *)paddr->precord;
   switch(index){
   case zDDMRecordMCA:{
      *no_elements =  1572864;
      *offset = 0;
      printf("get_array_info: MCA\n");
      break;
      }
   case zDDMRecordTDC:{
      *no_elements =  393216;
      *offset = 0;
      printf("get_array_info: TDC\n");
      break;
      }
   case zDDMRecordSPCT:{
      *no_elements =  4096;
      *offset = 0;
      printf("get_array_info: SPCT\n");
      break;
      }
    case zDDMRecordSPCTX:{
      *no_elements =  4096;
      *offset = 0;
      printf("get_array_info: SPCTX\n");
      break;
      }
   case zDDMRecordCHEN:{
      *no_elements =  pzDDM->nelm;
      *offset = 0;
      printf("get_array_info: CHEN\n");
      break;
      }
    case zDDMRecordSLP:{
      *no_elements =  pzDDM->nelm;
      *offset = 0;
      printf("get_array_info: SLO\n");
      break;
      }
    case zDDMRecordOFFS:{
      *no_elements =  pzDDM->nelm;
      *offset = 0;
      printf("get_array_info: OFFS\n");
      break;
      }
    case zDDMRecordTSEN:{
      *no_elements =  pzDDM->nelm;
      *offset = 0;
      printf("get_array_info: TSEN\n");
      break;
      }
    case zDDMRecordTHTR:{
      *no_elements =  pzDDM->nelm;
      *offset = 0;
      printf("get_array_info: THTR\n");    
      break;
      }
    case zDDMRecordTHRSH:{
      *no_elements =  pzDDM->nchips;
      *offset = 0;
      printf("get_array_info: THRSH\n");
      break;
      }
     }
   return(0);
}


//static long put_array_info(paddr,nNew)
//struct dbAddr *paddr;
//long nNew;
//{
//zDDMRecord *pzDDM=(zDDMRecord *)paddr->precord;
//
//   pzDDM->nelm /*nord*/ = nNew;
//   /*if (pzDDM->nord > pzDDM->nelm) pzDDM->nord = pzDDM->nelm;*/
//   return(0);
//}





static long init_record(zDDMRecord *pscal, int pass)
{
	long status;
	int i,j, nchips;
	unsigned int *mca, *tdc, *spct;
	int *thrsh;
	float *slp, *offs, *spctx;
	char *thtr, *chen, *tsen, *putr;
	
	det_DSET *pdset = (det_DSET *)(pscal->dset);
	CALLBACK *pcallbacks, *pupdateCallback, *pdelayCallback, *pautoCallback, *pdeviceCallback;
	struct rpvtStruct *prpvt;
	printf("zDDM init_record: pass = %d\n", pass);
	pscal->nelm=zDDM_NCHAN;
	pscal->nchips=zDDM_NCHIPS;
	nchips=zDDM_NCHIPS;        
	Debug(5, "scaler init_record: pass = %d\n", pass);
        Debug(5, "init_record: .PR1 = %ld\n", (unsigned long)pscal->pr1);

	if (pass == 0) {
	
	
	
		if(pscal->nelm<32) pscal->nelm=32; /* Minimum system is 1 chip, 32 channels */
		if(nchips<=0) nchips=1;
		pscal->pchen = (char *)calloc(pscal->nelm,sizeof(char));
		pscal->ptsen = (char *)calloc(pscal->nelm,sizeof(char));
		pscal->pslp = (float *)calloc(pscal->nelm,sizeof(float));
		pscal->poffs = (float *)calloc(pscal->nelm,sizeof(float));
		pscal->pthtr = (int *)calloc(pscal->nelm,sizeof(int));
		pscal->pthrsh = (int *)calloc(pscal->nchips,sizeof(int));
		pscal->pputr = (char *)calloc(pscal->nelm,sizeof(char));
		pscal->pmca = (unsigned int *)calloc(pscal->nelm*4096,sizeof(int));
		pscal->ptdc = (unsigned int *)calloc(pscal->nelm*1024,sizeof(int));
		pscal->pspct = (unsigned int *)calloc(4096,sizeof(int));
		pscal->pspctx = (float *)calloc(4096,sizeof(float));

		pscal->vers = VERSION;
		pscal->fver=fpgabase[VERSIONREG];
		
		pscal->rpvt = (void *)calloc(1, sizeof(struct rpvtStruct));
                prpvt = (struct rpvtStruct *)pscal->rpvt;
                if ((prpvt->updateMutex = epicsMutexCreate()) == 0) {
                        errlogPrintf("zDDMRecord:init_record: could not create mutex.\n");
                        return(-1);
                	}


		/* Gotta have a .val field.  Make its value reproducible. */
		pscal->val=0;
		
				
		chen=pscal->pchen;
		slp=pscal->pslp;
		offs=pscal->poffs; 
		tsen=pscal->ptsen;
		thtr=pscal->pthtr;
		putr=pscal->pputr;
		thrsh=pscal->pthrsh;
		mca=pscal->pmca;
		tdc=pscal->ptdc;
		spct=pscal->pspct;
		spctx=pscal->pspctx; 
		
		pscal->tysize = pscal->nelm;
		pscal->eysize = pscal->nelm;
		
		for(i=0;i<pscal->nelm;i++){
		   for(j=0;j<4096;j++){
		   mca[i*4096+j] = j*i+1; /* gray ramp along X and Y*/
		   }
		}
		printf("Initialized MCA counts\n");
		
		for(i=0;i<pscal->nelm;i++){
		   for(j=0;j<1024;j++){
		   tdc[i*1024+j] = j*i+1;  /* gray ramp along X and Y*/
		   }
		}
		printf("Initialized TDC counts\n");

		for(j=0;j<4096;j++){
		   spct[j] = j; /* gray ramp */
		   spctx[j]=j;
		   }
		printf("Initialized SPCT counts\n");
		
		pscal->monch=0;
		pscal->pol=0;
		
		for(i=0;i<pscal->nchips;i++){
			thrsh[i]=300;
			globalstr[i].pa = 300;
			printf("Thresh[%i]=%i\n",i,thrsh[i]);
			}
		for(i=0;i<pscal->nelm;i++){
			channelstr[i].sm = 0; 
			chen[i]=0;
			channelstr[i].st = 0;
			tsen[i]=0;
			channelstr[i].sel = 0;
			pscal->loao=0;
			channelstr[i].da = 4; /* 3-bits, mid-scale*/
			thtr[i]=4; 
			channelstr[i].dp = 8; /* 4-bits, mid-scale*/
			putr[i]=8;
			slp[i]=1.0;
			offs[i]=0.0;
			}
			
		stuff_mars(pscal);
		return (0);
	}

	prpvt = (struct rpvtStruct *)pscal->rpvt;
	
        /*** setup callback stuff (note: array of 4 callback structures) ***/
        pcallbacks = (CALLBACK *)(calloc(4,sizeof(CALLBACK)));
        prpvt->pcallbacks = pcallbacks;

        /* callback to implement periodic updates */
        pupdateCallback = (CALLBACK *)&(pcallbacks[0]);
        callbackSetCallback(updateCallbackFunc, pupdateCallback);
        callbackSetPriority(pscal->prio, pupdateCallback);
        callbackSetUser((void *)pscal, pupdateCallback);

        /* second callback to implement delay */
        pdelayCallback = (CALLBACK *)&(pcallbacks[1]);
        callbackSetCallback(delayCallbackFunc, pdelayCallback);
        callbackSetPriority(pscal->prio, pdelayCallback);
        callbackSetUser((void *)pscal, pdelayCallback);

        /* third callback to implement auto-count */
        pautoCallback = (CALLBACK *)&(pcallbacks[2]);
        callbackSetCallback(autoCallbackFunc, pautoCallback);
        callbackSetPriority(pscal->prio, pautoCallback);
        callbackSetUser((void *)pscal, pautoCallback);

        /* fourth callback for device support */
        pdeviceCallback = (CALLBACK *)&(pcallbacks[3]);
        callbackSetCallback(deviceCallbackFunc, pdeviceCallback);
        callbackSetPriority(pscal->prio, pdeviceCallback);
        callbackSetUser((void *)pscal, pdeviceCallback);


	/* Check that we have everything we need. */
	if (!(pdset = (det_DSET *)(pscal->dset)))
	{
		recGblRecordError(S_dev_noDSET,(void *)pscal, "zDDM: init_record");
		return(S_dev_noDSET);
	}

	Debug(2, "init_record: calling dset->init_record %d\n", 0);
	if (pdset->init_record)
	{
		status=(*pdset->init_record)(pscal, pdeviceCallback);
		Debug(3, "init_record: dset->init_record returns %i\n", status);
		if (status) {
			pscal->card = -1;
			return (status);
		}
		pscal->card = pscal->inp.value.vmeio.card;
		db_post_events(pscal,&(pscal->card),DBE_VALUE|DBE_ARCHIVE);
	}
        /* default clock freq */
        if (pscal->freq == 0.0) {
                pscal->freq = 1.0e6;
                db_post_events(pscal,&(pscal->freq),DBE_VALUE|DBE_ARCHIVE);
        }

        /* default count time */
        if ((pscal->tp == 0.0) && (pscal->pr1 == 0)) {
                pscal->tp = 1.0;
                db_post_events(pscal,&(pscal->pr1),DBE_VALUE|DBE_ARCHIVE);
        }

        /* convert between time and clock ticks */
        if (pscal->tp) {
                /* convert time to clock ticks */
                pscal->pr1 = (unsigned long) (pscal->tp * pscal->freq);
                db_post_events(pscal,&(pscal->pr1),DBE_VALUE|DBE_ARCHIVE);
                Debug(3, "init_record: .TP != 0, so .PR1 set to %ld\n", (unsigned long)pscal->pr1);
        } else if (pscal->pr1 && pscal->freq) {
                /* convert clock ticks to time */
                pscal->tp = (double)(pscal->pr1 / pscal->freq);
                db_post_events(pscal,&(pscal->tp),DBE_VALUE|DBE_ARCHIVE);
                Debug(3, "init_record: .PR1/.FREQ != 0, so .TP set to %f\n", pscal->tp);
        }
	db_post_events(pscal,pscal->pmca,DBE_VALUE|DBE_ARCHIVE);
	db_post_events(pscal,pscal->ptdc,DBE_VALUE|DBE_ARCHIVE);
	db_post_events(pscal,pscal->pspct,DBE_VALUE|DBE_ARCHIVE);
	db_post_events(pscal,pscal->pchen,DBE_VALUE|DBE_ARCHIVE);
	db_post_events(pscal,pscal->ptsen,DBE_VALUE|DBE_ARCHIVE);
	return(0);
}


static long process(zDDMRecord *pscal)
{
        int status, prev_scaler_state, handled;
        int justFinishedUserCount=0, justStartedUserCount=0, putNotifyOperation=0;
        unsigned long *ppreset = (unsigned long *)&(pscal->pr1);
        struct rpvtStruct *prpvt = (struct rpvtStruct *)pscal->rpvt;
        CALLBACK *pcallbacks = prpvt->pcallbacks;
        det_DSET *pdset = (det_DSET *)(pscal->dset);
	CALLBACK *pupdateCallback = (CALLBACK *)&(pcallbacks[0]);
	CALLBACK *pdelayCallback = (CALLBACK *)&(pcallbacks[1]); 
	CALLBACK *pautoCallback = (CALLBACK *)&(pcallbacks[2]);
	CALLBACK *pdeviceCallback = (CALLBACK *)&(pcallbacks[3]); 

        if(zDDMRecordDebug>2){
                printf("At process entry[%d]:\n",__LINE__);
                printf("User state = %d\n",pscal->us);
                printf("Scaler state = %d\n",pscal->ss);
                }

        Debug(5, "process: entry %d\n",0);
        epicsMutexLock(prpvt->updateMutex);
        Debug(5, "Mutex locked %d\n",0);
        pscal->pact = TRUE;
        pscal->udf = FALSE;
        prev_scaler_state = pscal->ss;

        /* If we're being called as a result of a done-counting interrupt, */
        /* (*pdset->done)(pscal) will return TRUE */
        if ((*pdset->done)(pscal)) {
                pscal->ss = zDDM_STATE_IDLE;

                Debug(5, "done=1; zDDM_STATE_IDLE %i\n",pscal->ss);

                /* Auto-count cycle is not allowed to reset .CNT field. */
                if (pscal->us == USER_STATE_COUNTING) {
                        Debug(5, "Setting CNT to 0 %d\n",0);
                        pscal->cnt = 0;
                        db_post_events(pscal,&pscal->cnt,DBE_VALUE|DBE_ARCHIVE);
                        pscal->us = USER_STATE_IDLE;

                        Debug(5, "USER_STATE_IDLE %d\n",0);

                        justFinishedUserCount = 1;
                        if (pscal->ppn) putNotifyOperation = 1;
                }
        }

        if (pscal->cnt != pscal->pcnt) {
                handled = 0;
                if (pscal->cnt && ((pscal->us == USER_STATE_REQSTART) ||
                              (pscal->us == USER_STATE_WAITING))) 
		{
                        /*** if we're already counting (auto-count), stop ***/
                        if (pscal->ss == zDDM_STATE_COUNTING) {
                                Debug(5, "Killing count %d\n",0);
                                (*pdset->arm)(pscal, 0);
                                pscal->ss = zDDM_STATE_IDLE;
                        }

                        if (pscal->us == USER_STATE_REQSTART) {
                                Debug(5, "process: start counting %d\n",0);
                                (*pdset->reset)(pscal);
                                pscal->pr1 = (unsigned long)(pscal->tp * pscal->freq);
                                (*pdset->write_preset)(pscal, pscal->pr1);
                                Debug(5, "process: Arming scaler %d\n",0);
                                (*pdset->arm)(pscal, 1);
                                pscal->ss = zDDM_STATE_COUNTING;
                                pscal->us = USER_STATE_COUNTING;
                                handled = 1;
                                justStartedUserCount = 1; 
                                Debug(5, "process: handled=%d\n",handled);
                                Debug(5, "process: justStartedUserCount=%d\n",justStartedUserCount);
                        }
                } else if (!pscal->cnt) {
                        if (pscal->ss != zDDM_STATE_IDLE)  (*pdset->arm)(pscal, 0);
                        pscal->ss = zDDM_STATE_IDLE;
                        pscal->us = USER_STATE_IDLE;
                        justFinishedUserCount = 1;
                        handled = 1;
                        Debug(5, "process: cnt=%d\n",pscal->cnt);
                        Debug(5, "process: User state=%d\n",pscal->us);
                        Debug(5, "process: Scaler state=%d\n",pscal->ss);
                }
                if (handled) {
                        pscal->pcnt = pscal->cnt;
                        Debug(5, "process:handled: pcnt=%d -> cnt\n",pscal->pcnt);
			db_post_events(pscal,&pscal->cnt,DBE_VALUE|DBE_ARCHIVE);	
                }
        }

        /* read and display scalers */
        updateCounts(pscal);

        if (justStartedUserCount || justFinishedUserCount) {
                /* fire .cout link to trigger anything that should coincide with scaler integration */
                status = dbPutLink(&pscal->cout, DBR_SHORT, &pscal->cnt, 1);
                if (!RTN_SUCCESS(status)) {
                        Debug(5, "scaler:process: ERROR %d PUTTING TO COUT LINK.\n", status);
                }
                if (justFinishedUserCount) {
                        /* fire .coutp link to trigger anything that should coincide with scaler integration */
                        status = dbPutLink(&pscal->coutp, DBR_SHORT, &pscal->cnt, 1);
                        if (!RTN_SUCCESS(status)) {
                                Debug(5, "scaler:process: ERROR %d PUTTING TO COUTP LINK.\n", status);
                        }
                }
        }

        /* done counting? */
        if (pscal->ss == zDDM_STATE_IDLE) {
                Debug(5, "Done counting? %d\n",0);
                recGblGetTimeStamp(pscal);
                do_alarm(pscal);
                monitor(pscal);
                if ((pscal->pcnt==0) && (pscal->us == USER_STATE_IDLE)) {
                        if (prev_scaler_state == zDDM_STATE_COUNTING) {
                                db_post_events(pscal,pscal->pmca,DBE_VALUE|DBE_ARCHIVE);
				db_post_events(pscal,pscal->ptdc,DBE_VALUE|DBE_ARCHIVE);
				db_post_events(pscal,pscal->pspct,DBE_VALUE|DBE_ARCHIVE);
                        }
                        recGblFwdLink(pscal);
                }
        }

        /* Are we in auto-count mode and not already counting? */
        if (pscal->us == USER_STATE_IDLE && pscal->cont && pscal->ss != zDDM_STATE_COUNTING) {
        double dly_sec=pscal->dly1;  /* seconds to delay */

                if (justFinishedUserCount) dly_sec = MAX(pscal->dly1, zDDM_wait_time);
                if (putNotifyOperation) dly_sec = MAX(pscal->dly1, zDDM_wait_time);
                /* if (we-have-to-wait && we-haven't-already-waited) { */
                if (dly_sec > 0 && pscal->ss != zDDM_STATE_WAITING) {
                        Debug(5, "process: scheduling autocount restart: %d\n",0);
                        /*
                         * Schedule a callback, and make a note that counting should start
                         * the next time we process (if pscal->ss is still zDDM_STATE_WAITING
                         * at that time).
                         */
                        pscal->ss = zDDM_STATE_WAITING;  /* tell ourselves to start next time */
                        callbackRequestDelayed(pautoCallback, dly_sec);
                } else {
                        Debug(5, "process: restarting autocount %d\n",0);
                        /* Either the delay time is zero, or pscal->ss = zDDM_STATE_WAITING
                         * (we've already waited), so start auto-count counting.
                         * Different rules apply for auto-count counting:
                         * If (.TP1 >= .001 s) we count .TP1 seconds, regardless of any
                         * presets the user may have set.
                         */
                        (*pdset->reset)(pscal);
                        if (pscal->tp1 >= 1.e-3) {
                                (*pdset->write_preset)(pscal, (unsigned long)(pscal->tp1*pscal->freq));
                        }
                        (*pdset->arm)(pscal, 1);
                        pscal->ss = zDDM_STATE_COUNTING;
                        Debug(5, "zDDM_STATE_COUNTING: %d\n", 0);

                        /* schedule first update callback */
                        if (pscal->rat1 > .1) {
                                callbackRequestDelayed(pupdateCallback, 1.0/pscal->rat1);
                        }
                }
        }

        pscal->pact = FALSE;
	pscal->runno=fpgabase[FRAME_NO];
        db_post_events(pscal,&(pscal->runno),DBE_VALUE|DBE_ARCHIVE);
        db_post_events(pscal,pscal->pmca,DBE_VALUE|DBE_ARCHIVE);
        db_post_events(pscal,pscal->ptdc,DBE_VALUE|DBE_ARCHIVE);
        db_post_events(pscal,pscal->pspct,DBE_VALUE|DBE_ARCHIVE);
        Debug(5, "process:Mutex unlocked\nScaler state=%d\n",pscal->ss);
        Debug(5, "process:Mutex unlocked\nUser state=%d\n",pscal->us);
        epicsMutexUnlock(prpvt->updateMutex);
        return(0);
}

static void updateCounts(zDDMRecord *pscal)
{
        unsigned int *mca;
	unsigned int *tdc;
	unsigned int *spct;

        mca=pscal->pmca;
        tdc=pscal->ptdc;
	spct=pscal->pspct;


        int i, j, called_by_process;
        float rate;
        struct rpvtStruct *prpvt = (struct rpvtStruct *)pscal->rpvt;
        CALLBACK *pcallbacks = prpvt->pcallbacks;
        CALLBACK *pupdateCallback = (CALLBACK *)&(pcallbacks[0]);

        called_by_process = (pscal->pact == TRUE);
        Debug(5, "updateCounts: %s called by process()\n", called_by_process ? " " : "NOT");
        if (!called_by_process) {
                if (pscal->ss != zDDM_STATE_IDLE) pscal->pact = TRUE;
                else return;
        }

        /* read scalers (get pointer to actual VME-resident scaler-data array) */
        if (pscal->us != USER_STATE_WAITING) {
	        /* mca and tdc updated in event_publish() */
                /* update monitored channel spectrum */
                for(i=0;i<4096;i++){     
                     spct[i]=mca[4096*pscal->monch+i]; 
		     }
                Debug(5, "updateCounts: got counts[] %d\n",0);
        } else {
               Debug(5, "New frame: clear mca and spct %d\n",0);
	       /* clear spectrum of selected channel */
	       for (i=0; i<4096; i++) {
	       spct[i] = 0;
	       }
	       /* clear mca array */
	       for (i=0; i<zDDM_NCHAN; i++){
	         for (j=0; j<4096;j++){
		   mca[4096*i+j]=0;
		   }
		}
        }

        Debug(5, "updateCounts: posting scaler values %d\n",0);
        /* copy and post spectrum values */
        for (i=0; i<4096; i++) {
                if (mca[4096*pscal->monch+i] != spct[i]) {
			spct[i]=mca[4096*pscal->monch+i];
			}
		}
                        db_post_events(pscal,pscal->pmca,DBE_VALUE|DBE_ARCHIVE);
			db_post_events(pscal,pscal->ptdc,DBE_VALUE|DBE_ARCHIVE);
			db_post_events(pscal,pscal->pspct,DBE_VALUE|DBE_ARCHIVE);
        
        /* convert clock ticks to time. Note device support may have changed freq. */
	/* freq is fixed in this device, so skip this bit */
	

        if (pscal->ss == zDDM_STATE_COUNTING) {
                /* arrange to call this routine again after user-specified time */
                Debug(5, "updateCounts: arranging for callback %d\n",0); 
                rate = ((pscal->us == USER_STATE_COUNTING) ? pscal->rate : pscal->rat1);
                if (rate > .1) {
                        callbackRequestDelayed(pupdateCallback, 1.0/rate);
                }
        }

        if (!called_by_process) pscal->pact = FALSE;
        Debug(5, "updateCounts: exit %d\n",0);
}


static long special(dbAddr *paddr, int after)
{
	
	zDDMRecord *pscal = (zDDMRecord *)(paddr->precord);
	int status, i=0;
	int j, rt, chip;
	int mars_modified;
	static int pmode;
	unsigned int *mca, *spct;
	float *spctx, *slp, *offs;
        struct rpvtStruct *prpvt = (struct rpvtStruct *)pscal->rpvt;
	devPVT *pPvt = (devPVT *)pscal->dpvt;	
        CALLBACK *pcallbacks = prpvt->pcallbacks;
        CALLBACK *pdelayCallback = (CALLBACK *)&(pcallbacks[1]);
        int fieldIndex = dbGetFieldIndex(paddr);
	double dly=0.0;
	mca=pscal->pmca;
	spct=pscal->pspct;
	spctx=pscal->pspctx;
	slp=pscal->pslp;
	offs=pscal->poffs;

	Debug(5, "special: entry; after=%d\n", after);
	if (!after) return (0);

	mars_modified=0;
	mars_modified=0;
	pscal->chip=pscal->monch/32; /* 32 channels per chip */
	pscal->chan=pscal->monch%32;
	Debug(5, "special: chip; %d\n", pscal->chip);
	Debug(5, "special: channel; %d\n", pscal->chan);
	
	switch (fieldIndex) {
        case zDDMRecordCNT:
                /* Ignore redundant (pscal->cnt == 1) commands */
                Debug(5, "special: CNT; User State =%d\n", pscal->us);
                if (pscal->cnt && (pscal->us != USER_STATE_IDLE)) return(0);

                /* fire .coutp link to trigger anything that should coincide with scaler integration */
                status = dbPutLink(&pscal->coutp, DBR_SHORT, &pscal->cnt, 1);
                if (!RTN_SUCCESS(status)) {
                        Debug(5, "scaler:special: ERROR %d PUTTING TO COUTP LINK.\n", status); 
                }

                /* Scan record if it's not Passive.  (If it's Passive, it'll get */
                /* scanned automatically, since .cnt is a Process-Passive field.) */
                /* Arrange to process after user-specified delay time */
                dly = pscal->dly;   /* seconds to delay */
                if (dly<0.0) dly = 0.0;
                if (dly == 0.0 || pscal->cnt == 0) {
                        /*** handle request now ***/
                        if (pscal->cnt) {
                                pscal->us = USER_STATE_REQSTART;
                                Debug(5, "special: start counting. User State= %d\n",pscal->us);
                        } else {
                                /* abort any counting or request to start counting */
                                switch (pscal->us) {
                                case USER_STATE_WAITING:
                                        /* We may have a watchdog timer going.  Cancel it. */
                                        if (pdelayCallback->timer) epicsTimerCancel(pdelayCallback->timer);
                                        pscal->us = USER_STATE_IDLE;
                                        break;
                                case USER_STATE_REQSTART:
                                        pscal->us = USER_STATE_IDLE;
                                        break;
                                default:
                                        break;
                                }
                        }
                        if (pscal->scan) scanOnce((void *)pscal);
                }
                else {
                        /* schedule callback to handle request */
                        pscal->us = USER_STATE_WAITING;
                        callbackRequestDelayed(pdelayCallback, pscal->dly);
                }
                break;

        case zDDMRecordCONT:
                /* Scan record if it's not Passive.  (If it's Passive, it'll get */
                /* scanned automatically, since .cont is a Process-Passive field.) */
                if (pscal->scan) scanOnce((void *)pscal);
                break;

        case zDDMRecordTP:
                /* convert time to clock ticks */
                pscal->pr1 = (unsigned long) (pscal->tp * pscal->freq);
                db_post_events(pscal,&(pscal->pr1),DBE_VALUE|DBE_ARCHIVE);
                break;

        case zDDMRecordPR1:
                /* convert clock ticks to time */
                pscal->tp = (double)(pscal->pr1 / pscal->freq);
                db_post_events(pscal,&(pscal->tp),DBE_VALUE|DBE_ARCHIVE);
                break;

        case zDDMRecordRATE:
                pscal->rate = MIN(60.,MAX(0.,pscal->rate));
                db_post_events(pscal,&(pscal->rate),DBE_VALUE|DBE_ARCHIVE);
                break;

        case zDDMRecordRUNNO:
                /* Get frame number */
                fpgabase[FRAME_NO]=pscal->runno;
		Debug(2, "special: RUNNO %i\n", pscal->runno);
               db_post_events(pscal,&(pscal->runno),DBE_VALUE|DBE_ARCHIVE);
                break;

        case zDDMRecordMODE:
                /* framing mode */
		if(pscal->mode==0){
		     fpgabase[COUNT_MODE]=0;
		     if(pmode==1){
		       frame_done(1); /* simulate end-of-count interrupt */
		       pscal->cnt=0;
		       pmode=0;
		       }
		     }
		/* Continuous acquisition */
		if(pscal->mode==1){
		    fpgabase[COUNT_MODE]=1;
		    pmode=1;
		    }		
		Debug(5, "special: COUNT_MODE %i\n", pscal->mode);
		Debug(5, "special: hardware COUNT_MODE %i\n", fpgabase[COUNT_MODE]);
                db_post_events(pscal,&(pscal->mode),DBE_VALUE|DBE_ARCHIVE);
		db_post_events(pscal,&(pscal->cnt),DBE_VALUE|DBE_ARCHIVE);
                break;

	case zDDMRecordGMON: /* set switch for channel monitors or others */
	         if(pscal->gmon==0){/* All monitors off. M0=0, C0-C4=00000*/
		    for(chip=0;chip<12;chip++){
		    globalstr[chip].c=0;
		    globalstr[chip].m0=0;
		    }
		   }
		 if(pscal->gmon==1){/* All off except this chip temp. M0=0 C0-C4=00100*/
		    for(chip=0;chip<12;chip++){
		    globalstr[chip].c=0;
		    globalstr[chip].m0=0;
		    }
		    globalstr[pscal->chip].c=4;
		  }
		 if(pscal->gmon==2){/* All off except this chip base M0=0 C0-C4=10100 */
 		    for(chip=0;chip<12;chip++){
		    globalstr[chip].c=0;
		    globalstr[chip].m0=0;
		    }
		    globalstr[pscal->chip].c=5;
		 }
		 if(pscal->gmon==3){/* All off except this chip thresh M0=0 C0-C4=01100*/
 		    for(chip=0;chip<12;chip++){
		    globalstr[chip].c=0;
		    globalstr[chip].m0=0;
		    }
		    globalstr[pscal->chip].c=6;
		 }
		 if(pscal->gmon==4){/* All off except this chip test pulse M0=0 C0-C4=11100*/
 		    for(chip=0;chip<12;chip++){
		    globalstr[chip].c=0;
		    globalstr[chip].m0=0;
		    }
		    globalstr[pscal->chip].c=7;

		 }
		 if(pscal->gmon==5){/* All off except this chip channel monitor M0=1; channel number to C0-C4*/
		    for(chip=0;chip<12;chip++){
		    globalstr[chip].c=0;
		    globalstr[chip].m0=0;
		    }
		 globalstr[pscal->chip].c=pscal->chan;
		 globalstr[pscal->chip].m0=1;
		 }
		Debug(2, "special: GMON %i\n", pscal->gmon);
		/* bits set by device support. Tell record to write hardware */
		mars_modified=1;
		db_post_events(pscal,&(pscal->gmon),DBE_VALUE|DBE_ARCHIVE);
		break;

	case zDDMRecordMONCH: /* set channel which has monitor out enabled */
		Debug(2, "special: MONCH %i\n", pscal->monch);
		/* write hardware */
		pscal->chip=pscal->monch/32;
		pscal->chan=pscal->monch%32;
                for(i=0;i<4096;i++){     
                     spct[i]=mca[4096*pscal->monch+i];
		     }
		printf("SPCT updated\n");
		for(i=0;i<4096;i++){
		     spctx[i]=(float)i* slp[pscal->monch]+offs[pscal->monch];
		     }
 		printf("SPCTX updated\n");
		if(pscal->gmon==5){
		  for(chip=0;chip<12;chip++){
		    globalstr[chip].c=0;
		    globalstr[chip].m0=0;
		    }
		  globalstr[pscal->chip].c=pscal->chan;
		  globalstr[pscal->chip].m0=1;
		  channelstr[pscal->monch].sel=pscal->loao;
		  mars_modified=1;
		  }
		db_post_events(pscal,&(pscal->monch),DBE_VALUE|DBE_ARCHIVE);
		db_post_events(pscal,pscal->pspct,DBE_VALUE|DBE_ARCHIVE);
		db_post_events(pscal,pscal->pspctx,DBE_VALUE|DBE_ARCHIVE);
		break;

	case zDDMRecordPOL: /* set input polarity */
	        for(chip=0;chip<12;chip++){
		   globalstr[chip].sp=pscal->pol;
		   }
		mars_modified=1;
		break;

	case zDDMRecordLOAO: /* set channel monitor to leakage or pulse */
		Debug(2, "special: LOAO\n %i", pscal->loao);
		/* write hardware */
		channelstr[pscal->monch].sel=pscal->loao;
		mars_modified=1; 
		db_post_events(pscal,&(pscal->loao),DBE_VALUE|DBE_ARCHIVE);
		break;

	case zDDMRecordEBLK: /* Enable on-chip bias current generator */
		Debug(2, "special: EBLK\n %i", pscal->eblk);
		/* write hardware. Do all or nothing for now. */
		  for(chip=0;chip<12;chip++){
		    globalstr[chip].sl=pscal->eblk;
		    }
		mars_modified=1; 
		db_post_events(pscal,&(pscal->eblk),DBE_VALUE|DBE_ARCHIVE);
		break;

	case zDDMRecordTPAMP: /* Set test pulse amplitude */
		Debug(2, "special: TPAMP\n %i", pscal->eblk);
		/* write hardware. Do all or nothing for now. */
		  for(chip=0;chip<12;chip++){
		    globalstr[chip].pb=pscal->tpamp;
		    }
		mars_modified=1; 
		db_post_events(pscal,&(pscal->tpamp),DBE_VALUE|DBE_ARCHIVE);
		break;
	case zDDMRecordTPFRQ: /* Set test pulse frequency */
		Debug(2, "special: TPFRQ\n %i", pscal->eblk);
		/* write hardware. Do all or nothing for now. */
		fpgabase[CALPULSE_RATE]=25000000/pscal->tpfrq/2;
		fpgabase[CALPULSE_WIDTH]=25000000/pscal->tpfrq/2;
		mars_modified=1; 
		db_post_events(pscal,&(pscal->tpfrq),DBE_VALUE|DBE_ARCHIVE);
		break;
	case zDDMRecordTPCNT: /* Set test pulse count */
		Debug(2, "special: TPCNT\n %i", pscal->eblk);
		/* write hardware. Do all or nothing for now. */
		fpgabase[CALPULSE_CNT]=pscal->tpcnt;
		mars_modified=1; 
		db_post_events(pscal,&(pscal->tpcnt),DBE_VALUE|DBE_ARCHIVE);
		break;
	case zDDMRecordTPENB: /* Enable test pulses */
		Debug(2, "special: TPENB\n %i", pscal->eblk);
		/* write hardware. Do all or nothing for now. */
		if(pscal->tpenb==1){
		  fpgabase[MARS_CALPULSE]=0xFFF;
		  fpgabase[CALPULSE_MODE]=1;
		  }
		else {
		  fpgabase[MARS_CALPULSE]=0;
		  fpgabase[CALPULSE_MODE]=0;
		  }
		mars_modified=1; 
		db_post_events(pscal,&(pscal->tpenb),DBE_VALUE|DBE_ARCHIVE);
		break;

	case zDDMRecordSHPT: /* set shaping time */
		Debug(2, "special: SHPT %i\n", pscal->shpt);
		  for(chip=0;chip<12;chip++){
		    globalstr[chip].ts=pscal->shpt;
		    }
		mars_modified=1;
		db_post_events(pscal,&(pscal->shpt),DBE_VALUE|DBE_ARCHIVE);
		break;

	case zDDMRecordGAIN: /* set gain */
		Debug(2, "special: GAIN %i\n", pscal->gain);
		  for(chip=0;chip<12;chip++){
		    globalstr[chip].g=pscal->gain;
		    }
		mars_modified=1;
		db_post_events(pscal,&(pscal->gain),DBE_VALUE|DBE_ARCHIVE);
		break;
		
	case zDDMRecordCHEN: /* load 'channel-disabled' array */ 
		mars_modified=1;
		db_post_events(pscal,pscal->pchen,DBE_VALUE|DBE_ARCHIVE);/* post change */
		Debug(2, "special: CHEN %i\n", 0);
		break;
		
	case zDDMRecordTSEN: /* load 'test pulse input enabled' array */
		mars_modified=1;
		Debug(2, "special: TSEN %i\n", 0);
		db_post_events(pscal,pscal->ptsen,DBE_VALUE|DBE_ARCHIVE);
		break;
		
	case zDDMRecordTHTR:  /* array of NCHAN trim DAC values */
		mars_modified=1;
		Debug(2, "special: THTR %i\n", 0);
		db_post_events(pscal,pscal->pthtr,DBE_VALUE|DBE_ARCHIVE);
		break;

	case zDDMRecordPUTR:  /* array of NCHAN pileup threshold trim values */
		mars_modified=1;
		Debug(2, "special: PUTR %i\n", 0);
		db_post_events(pscal,pscal->pputr,DBE_VALUE|DBE_ARCHIVE);
		break;

	case zDDMRecordPUEN: /* Pileup Rejection enable */
		  for(chip=0;chip<12;chip++){
		    globalstr[chip].spur=pscal->puen;
		    }
		mars_modified=1;
		Debug(2, "special: PUEN %i\n", pscal->puen);
		db_post_events(pscal,&(pscal->puen),DBE_VALUE|DBE_ARCHIVE);
		break;
						
	case zDDMRecordMFS: /* Multi-fire suppression */
		  for(chip=0;chip<12;chip++){
		    globalstr[chip].sse=pscal->mfs;
		    }
		mars_modified=1;
		Debug(2, "special: MFS %i\n", pscal->mfs);
		db_post_events(pscal,&(pscal->mfs),DBE_VALUE|DBE_ARCHIVE);
		break;

        case zDDMRecordTDS: /* set time detector slope */
		switch(pscal->tds){
		   case 1:
		     j=0;
		     rt=0;
		     break;
		   case 2:
		     j=1;
		     rt=0;
		     break;
		   case 3:
		     j=2;
		     rt=0;
		     break;
		   case 4:
		     j=3;
		     rt=0;
		     break;
		   case 5:
		     j=1;
		     rt=1;
		     break;
		   case 6:
		     j=2;
		     rt=1;
		     break;
		   case 7:
		     j=3;
		     rt=1;
		     break;
		   }
		for(chip=0;chip<12;chip++){
		    globalstr[chip].tr=j;
		    globalstr[chip].rt=rt;
		    }

		mars_modified=1;
	        Debug(2, "special: TDS %i\n", pscal->tds);
	        db_post_events(pscal,&(pscal->tds),DBE_VALUE|DBE_ARCHIVE);
	        break;

	case zDDMRecordTDM: /* set TDC mode */
		for(chip=0;chip<12;chip++){
		    globalstr[chip].tm=pscal->tdm;
		    }	
		mars_modified=1;
		Debug(2, "special: TDM %i\n", 0);
		db_post_events(pscal,&(pscal->tdm),DBE_VALUE|DBE_ARCHIVE);
		break;

	case zDDMRecordTHRSH: /* Threshold */
		mars_modified=1;
		Debug(2, "special: THRSH %i\n", 0);
		db_post_events(pscal,pscal->pthrsh,DBE_VALUE|DBE_ARCHIVE);
		break;
		
	default:
		Debug(2, "special: Bad field index: %i\n", fieldIndex);
		break;
	} /* switch (fieldIndex) */
	if(mars_modified){
		Debug(2, "special: mars controls modified: modifed=%i\n", mars_modified);
		stuff_mars(pscal);
		}
//	db_post_events(pscal,NULL,0xf);		
//	scanOnce((void *)pscal);
	return(0);
}

static long get_precision(paddr, precision)
struct dbAddr *paddr;
long          *precision;
{
	zDDMRecord *pscal = (zDDMRecord *) paddr->precord;
    int fieldIndex = dbGetFieldIndex(paddr);

	*precision = pscal->prec;
	if (fieldIndex == zDDMRecordVERS) {
		*precision = 2;
	} else if (fieldIndex >= zDDMRecordVAL) {
		*precision = pscal->prec;
	} else {
		recGblGetPrec(paddr, precision);	/* Field is in dbCommon */
	}
	return (0);
}


static void do_alarm(pscal)
zDDMRecord *pscal;
{
	if(pscal->udf == TRUE ){
		recGblSetSevr(pscal,UDF_ALARM,INVALID_ALARM);
		return;
	}
	return;
}

static void monitor(pscal)
zDDMRecord *pscal;
{
	unsigned short monitor_mask;

	monitor_mask = recGblResetAlarms(pscal);

	monitor_mask|=(DBE_VALUE|DBE_ARCHIVE|DBE_LOG);

	/* check all value fields for changes */
	return;
}


