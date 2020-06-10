/*******************************************************************************
devzDDM.c

Device support for the BNL Multi-element germanium detector based on MARS 
electronics
D. P. Siddons
********************************************************************************
1.0  3/24/2017  dps     first ZynQ implementation
*******************************************************************************/

#include        <epicsVersion.h>

#if EPICS_VERSION < 3 || (EPICS_VERSION==3 && EPICS_REVISION < 14)
#define NOT_YET_OSI
#endif

#ifdef HAS_IOOPS_H
#include        <basicIoOps.h>
#endif

#include	<zmq.h>
#include	<stdio.h>
#include        <stdlib.h>
#include        <stdio.h>
#include 	<stdarg.h>
#include 	<stdint.h>
#include	<signal.h>
#include 	<unistd.h>
#include	<fcntl.h>
#include	<sys/mman.h>
#include        <epicsTimer.h>
#include        <epicsThread.h>
#include        <epicsExport.h>
#include	<registryFunction.h>
#include	<iocsh.h>

epicsTimerQueueId       zDDMWdTimerQ=0;
epicsTimerQueueId       TPgenTimerQ=0;


#include	<string.h>
#include	<math.h>

#include	<alarm.h>
/* #include	<dbRecType.h>  tmm: EPICS 3.13 */
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbCommon.h>
#include	<dbScan.h>
#include	<dbEvent.h>
#include        <recGbl.h>
#include	<recSup.h>
#include	<devSup.h>
/*#include	<drvSup.h> */
#include	<dbScan.h>
#include	<special.h>
#include        <callback.h>
#include	<epicsInterrupt.h>
#include	<errlog.h>
#include 	<iocsh.h>
#include	<epicsExport.h>
#include 	<cantProceed.h>
#include 	<recSup.h>
#include 	<devSup.h>
#include 	<caeventmask.h>

#include	"zDDMRecord.h"
#include	"devzDDM.h"
#include	"pl_lib.h"

#define FAST_LOCK epicsMutexId
#define FASTLOCKINIT(PFAST_LOCK) (*(PFAST_LOCK) = epicsMutexCreate())
#define FASTLOCK(PFAST_LOCK) epicsMutexLock(*(PFAST_LOCK));
#define TRYLOCK(PFAST_LOCK) epicsMutexTryLock(*(PFAST_LOCK));
#define FASTUNLOCK(PFAST_LOCK) epicsMutexUnlock(*(PFAST_LOCK));
#define ERROR -1
#define OK 0

/*** Debug support ***/
#define PRIVATE_FUNCTIONS 0	/* normal:1, debug:0 */
#if PRIVATE_FUNCTIONS
#define STATIC static
#else
#define STATIC
#endif
#ifdef NODEBUG
#define Debug0(l,f) ;
#define Debug(l,f,v) ;
#else
#define Debug0(l,FMT) {  if(l <= devzDDMdebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT); } }
#define Debug(l,FMT,V) {  if(l <= devzDDMdebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT,V); } }
#endif

#define zDDM_DELTA 2

#define BIT_SET(reg,bit) reg = reg | (1 << bit)
#define BIT_CLR(reg,bit) reg = reg & ~(1 << bit)
#define BIT_TST(reg,bit) reg & (1 << bit)

extern int zDDM_NCHAN;
extern int zDDM_NCHIPS;

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
#define DMA_STAT 33
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

 #define SIMUL 1  /* For testing without hardware, define SIMUL */

#ifdef SIMUL
static unsigned int fpga_data[1024];
unsigned int *fpgabase = fpga_data;
#else
volatile unsigned int *fpgabase; /* mmap'd fpga registers */
#endif
int fd, fe; /* file descriptor for memory map device and IRQ service*/

epicsMutexId SPI_lock, fpga_write_lock;

volatile int devzDDMdebug=0;
epicsExportAddress(int, devzDDMdebug);

/*static long vsc_num_cards = 1;*/
STATIC long zDDM_report(int level);
STATIC long zDDM_init(int after);
STATIC long zDDM_init_record(struct zDDMRecord *psr, CALLBACK *pcallback);
//STATIC long zDDM_get_ioint_info(int cmd, struct dbCommon *p1, IOSCANPVT *p2);
#define zDDM_get_ioint_info NULL
STATIC long zDDM_reset(struct zDDMRecord *pscal);
//#define zDDM_reset NULL
//STATIC long zDDM_read(void *pscal);
#define zDDM_read NULL
STATIC long zDDM_write_preset(zDDMRecord *psr, int val);
STATIC long zDDM_arm(struct zDDMRecord *pscal, int val);
STATIC long zDDM_done(zDDMRecord *pscal);

det_DSET devzDDM = {
	9, 
	zDDM_report,
	zDDM_init,
	zDDM_init_record,
	zDDM_get_ioint_info,
	zDDM_reset,
	zDDM_read,
	zDDM_write_preset,
	zDDM_arm,
	zDDM_done
};
epicsExportAddress(dset, devzDDM);


STATIC det_state zDDM_state;
//STATIC int channels[12][12];
STATIC unsigned int globals[MAX_NCHIPS][2];
unsigned char glb;
unsigned int dacs[5];

/**************************************************
* zDDM_report()
***************************************************/
STATIC long zDDM_report(int level)
{
	if (zDDM_state.card_exists) {
		printf("  NSLS Multi-element Silicon Detector #%d\n: No. elements= %d\n",0,zDDM_state.num_channels);
	}
	return (0);
}

/**************************************************
* zDDM_shutdown()
***************************************************/
STATIC int zDDM_shutdown()
{
// if (zDDM_reset()<0) return(ERROR);
 return(0);
}

/* Interrupt service routine for scaler clock timeout */
void frame_done(int signum)
{
CALLBACK *pcallbacks;

   zDDMRecord *psr=zDDM_state.psr;
   devPVT *pPvt = (devPVT *)psr->dpvt;
   struct rpvtStruct *rpvt = (struct rpvtStruct *)psr->rpvt;
   pcallbacks = (struct CALLBACK *)rpvt->pcallbacks;

   pPvt->done=1;
   Debug(2, "Counting exit: done = %d\n\r", pPvt->done); 
   Debug(2, "Frame no. %d\n\r", psr->runno); 
   callbackRequest(&pcallbacks[3]);
}

#define CMD_REG_READ  0
#define CMD_REG_WRITE 1
#define CMD_START_DMA 2

#define FIFODATAREG 24
#define FIFORDCNTREG 25
#define FIFOCNTRLREG 26

#define FRAMEACTIVEREG 52
#define FRAMENUMREG 54
#define FRAMELENREG 53 


/*******************************************
* mmap fpga register space 
* returns pointer fpgabase
*   
********************************************/
#ifndef SIMUL
void mmap_fpga()
{
   int fd;

   fd = open("/dev/mem",O_RDWR|O_SYNC);
   if (fd < 0) {
      printf("Can't open /dev/mem\n");
      exit(1);
   }

   fpgabase = (unsigned int *) mmap(0,255,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0x43C00000);

   if (fpgabase == NULL) {
      printf("Can't map FPGA space\n");
      exit(1);
   }
   else{
      printf("Memory map succsesful\n");
      }

}

#endif

int databuf[65536];
int evttot;
int dbg_lvl=1;
int lkuptbl[512];

/*****************************************
* fifo_enable 
*  FIFO Control Register
*     bit 0 = enable 
*     bit 1 = testmode
*     bit 2 = rst 
******************************************/
void fifo_enable()
{
   int rdback, newval;

   FASTLOCK(&fpga_write_lock);
   
   /*read current value of register */
   rdback = fpgabase[FIFOCNTRLREG];
   newval = rdback | 0x1;
   fpgabase[FIFOCNTRLREG] = newval;
   
   FASTUNLOCK(&fpga_write_lock);


}

/*****************************************
* fifo_disable 
*  FIFO Control Register
*     bit 0 = enable 
*     bit 1 = testmode
*     bit 2 = rst 
******************************************/
void fifo_disable()
{
   int rdback, newval;
   FASTLOCK(&fpga_write_lock);
   
   /* read current value of register */
   rdback = fpgabase[FIFOCNTRLREG];
   newval = rdback & 0x60;
      fpgabase[FIFOCNTRLREG] = newval;
   
   FASTUNLOCK(&fpga_write_lock);
}


/****************************************** 
* fifo reset
*  FIFO Control Register
*     bit 0 = enable 
*     bit 1 = testmode
*     bit 2 = rst 
******************************************/
void fifo_reset()
{
   int rdback, newval;
   FASTLOCK(&fpga_write_lock);
   /* read current value of register */
   rdback = fpgabase[FIFOCNTRLREG];
   newval = rdback | 0x4;
   /* set reset high */
   fpgabase[FIFOCNTRLREG] = newval;
   /* set reset low */
   newval = rdback & 0x3;
   fpgabase[FIFOCNTRLREG] = newval;
   FASTUNLOCK(&fpga_write_lock);
}

/*****************************************
* check_framestatus 
*    Check is frame is Active
******************************************/
int check_framestatus()
{
   /* read current value of register */
   return fpgabase[FRAMEACTIVEREG];

}

/*****************************************
* get_framenum 
*   Read the current frame Number
*   Framenumber is incremented by FPGA at end of frame
* 
******************************************/
int get_framenum()
{
   /* read current value of register */
   return fpgabase[FRAMENUMREG];

}


/*****************************************
* get_framelen 
*   Read the current frame length 
*   Counter is 25MHz, so 40ns period 
* 
******************************************/
float get_framelen()
{
   /* read current value of register */
   return (float)fpgabase[FRAMELENREG] / 25000000.0;
}


/*****************************************
* fifo_numwords 
*  Reads back number of words in fifo 
******************************************/
int fifo_numwords()
{
    return fpgabase[FIFORDCNTREG];
}


/*****************************************
* fifo_getdata  
*  Reads data in FIFO 
******************************************/
int fifo_getdata(int len, int *data)
{

   int i;
   for (i=0;i<len;i++) {
      data[i] = fpgabase[FIFODATAREG];
   }
   return 0;
}


EPICSTHREADFUNC event_publish (struct zDDMRecord *psr)
{

    int i, chipnum, channel, pd, td, ts, ts_prev;
    int numwords, prevframestat, framestat, pxl;
    float evtrate, framelen;
    unsigned int *mca, *tdc, *spct, *intens;
    int monch;
    unsigned int intensity[384];
    printf("Starting Data Thread...\n");
    framestat = 0;
    evttot = 0;
    mca=psr->pmca;
    tdc=psr->ptdc;
    spct=psr->pspct; 
    monch=psr->monch;
    intens=psr->pintens;

    while (1) {
        prevframestat = framestat;
        framestat = check_framestatus(); 
        //printf("PrevFrameStat=%d\tFrameStatus=%d\n",prevframestat,framestat); 
        if (framestat == 1) {
           if ((prevframestat == 0) && (framestat == 1))           
              printf("Frame Started...\n"); 
//	      for(i=0;i<384;i++){
//	        intens[i]=0;
//		}
           numwords = fifo_numwords();
           if (numwords > 0) {
              if (numwords > 10000)
                printf("Event Rate kinda too high...   Numwords in FIFO: %d\n",numwords);
              fifo_getdata(numwords,databuf); 
              evttot += numwords/2; 
              if (dbg_lvl == 1) { 
                 for (i=0;i<numwords;i=i+2) { 
                  //newaddr = lkuptbl[addr];
		  //databuf[i] = (databuf[i] & ~0x7FC00000) | (newaddr << 22);
                  chipnum = (databuf[i] & 0x78000000) >> 27;
                  channel = (databuf[i] & 0x07C00000) >> 22;                  
                  pd = (databuf[i] & 0xFFF);
                  td = (databuf[i] & 0x3FF000) >> 12; 
                  ts_prev = ts;
                  ts = (databuf[i+1] & 0x1FFFFFFF);
		  pxl=32*chipnum+channel;
		  intens[pxl]+=1;
		  mca[pxl*4096+pd]+=1;
		  tdc[pxl*1024+td]+=1;
		  if(devzDDMdebug>=6){
                   if (ts > ts_prev + 19){
                    printf("\n");
		    }
	 	    printf("ASIC: %4d   Chan: %4d   PD: %4d   TD: %4d   Timestamp: %d\n",chipnum,channel,pd,td,ts);
			}
                  }
              }
           }
        }
        else
            if ((prevframestat == 1) && (framestat == 0)) {          
                printf("Frame Complete...\n");
                framelen = get_framelen(); 
                evtrate = evttot / framelen;
		for(i=0;i<4096;i++){
		   spct[i]=mca[monch*4096+i];
		   }
               if(devzDDMdebug>=6){ 
                printf("Total Events in Frame=%d\t Rate=%.1f\n",evttot,evtrate);
		}
                evttot = 0; 
       }
        	
	usleep(1000);

    }   
}




/***************************************************
* initialize all software and hardware
* zDDM_init()
****************************************************/

STATIC long zDDM_init(int after)
{
	Debug(2,"zDDM_init(): entry, after = %d\n\r", after);
	if (after) return(0);
	FASTLOCKINIT(&fpga_write_lock);
	FASTLOCKINIT(&SPI_lock);
#ifndef SIMUL
	mmap_fpga();
#endif
	zDDM_state.card_exists = 1;
	printf("Hardware init.\n");
	Debug(3,"zDDM_init: Total cards = %d\n\n\r",1);

	if (zDDM_shutdown() < 0)
		errlogPrintf ("zDDM_init: zDDM_shutdown() failed\n\r"); 

        zDDMWdTimerQ=epicsTimerQueueAllocate(
				1, /* ok to share queue */
				epicsThreadPriorityLow);
        TPgenTimerQ=epicsTimerQueueAllocate(
				1, /* ok to share queue */
				epicsThreadPriorityLow);
	
 	Debug0(3,"zDDMWdTimerQ created\n\r");

        FASTLOCK(&fpga_write_lock);
	
	ad9252_cnfg(1,22,8); /* clock skew adjust */
	ad9252_cnfg(1,255,1); /* latch regs */
	ad9252_cnfg(2,22,8); /* clock skew adjust */
	ad9252_cnfg(2,255,1); /* latch regs */
	ad9252_cnfg(3,22,8); /* clock skew adjust */
	ad9252_cnfg(3,255,1); /* latch regs */
	fpgabase[MARS_RDOUT_ENB]=0x8fff; /* Enable all ASIC outputs to FPGA */
	
	FASTUNLOCK(&fpga_write_lock);
	Debug(3,"zDDM_init: zDDM %i initialized\n\r",0);
	return(0);
}

/***************************************************
* zDDM_init_record()
****************************************************/
STATIC long zDDM_init_record(struct zDDMRecord *psr, CALLBACK *pcallback)
{
	int channels;
	char thread_name[32];
	
	Debug(5,"zDDM_init_record: card %d\n\r", 0);

	/* inp must be an VME_IO */
	switch (psr->inp.type)
	{
	case (VME_IO) : break;
	default:
		recGblRecordError(S_dev_badBus,(void *)psr,
			"devzDDM (init_record) Illegal OUT Bus Type");
		return(S_dev_badBus);
	}

	Debug(5,"VME zDDM: card %d\n", 0);
	if (!zDDM_state.card_exists)
	{
		recGblRecordError(S_dev_badCard,(void *)psr,
		    "devzDDM (init_record) card does not exist!");
		return(S_dev_badCard);
	}

	if (zDDM_state.card_in_use)
	{
		recGblRecordError(S_dev_badSignal,(void *)psr,
		    "devzDDM (init_record) card already in use!");
		return(S_dev_badSignal);
	}

	zDDM_state.card_in_use = 1;
	sscanf(psr->inp.value.vmeio.parm, "%d", &channels);
	
	printf("devzDDM: channels=%d  zDDM_NCHAN=%d\n",channels, zDDM_NCHAN);
	
	if(channels != zDDM_NCHAN)
	{
		recGblRecordError(S_dev_badSignal,(void *)psr,
		    "devzDDM: Wrong number of channels!");
		return(S_dev_badSignal);
	}
	psr->nch = channels;
	zDDM_state.num_channels = channels;
	zDDM_state.tp = psr->tp;
	zDDM_state.psr = psr;
        Debug(2, "scaler_init_record(): entry %d\n\r", 1);
        devPVT *dpvt;
        dpvt = callocMustSucceed(1, sizeof(devPVT), "devzDDM init_record()");
        dpvt->exists=1;
        dpvt->done=0;
        dpvt->newdat=0;
        dpvt->pcallback=pcallback; 
        psr->dpvt=dpvt;
        dpvt->psr=psr;

/* Set up timer interrupt service routine */
#ifndef SIMUL
       pl_open(&fe);
       signal(SIGIO, &frame_done); 
       fcntl(fe, F_SETOWN, getpid());
       int oflags = fcntl(fe, F_GETFL);
       fcntl(fe, F_SETFL, oflags | FASYNC); 
#endif
 
/* Set up thread to accumulate spectra */
       sprintf(thread_name,"Spectra_%i",1);
       epicsThreadCreate(thread_name,epicsThreadPriorityHigh,
           epicsThreadGetStackSize(epicsThreadStackMedium),
           (EPICSTHREADFUNC)event_publish,psr);		
return(0);
}


STATIC long zDDM_arm(struct zDDMRecord *pscal, int val)
{    
  Debug(2, "scaler_arm(): entry, val = %d\n\r", val); 
  if(pscal->mode==0){
    FASTLOCK(&fpga_write_lock);
	if(val==1){
	   fifo_disable();/* disable fifo */
	   fifo_reset(); /* reset it */
	   fifo_enable(); /* enable fifo */
	   fpgabase[TRIG]=val; /* start count */
	   }
	if(val==0){
	   fifo_disable(); /* disable fifo */
	   fpgabase[TRIG]=val; /* stop count */
	   }
    FASTUNLOCK(&fpga_write_lock);
    }
  if(pscal->mode==1){
    }
  return(0);
}

/* Abort counting */
STATIC long zDDM_reset(struct zDDMRecord *pscal)
{
unsigned int *mca, *tdc, *spct;
int i,j;
    Debug(2, "scaler_reset(): entry %d\n\r", 1);
    mca=pscal->pmca;
    tdc=pscal->ptdc;
    spct=pscal->pspct; 

    if(pscal->mode==0){
    FASTLOCK(&fpga_write_lock);
    fifo_disable();
    fpgabase[TRIG]=0;
    FASTUNLOCK(&fpga_write_lock);
    }
    /* clear monitor spectrum */
    for (i=0; i<4096; i++) {
	       spct[i] = 0;
	       }
    /* clear mca array */
    for (i=0; i<zDDM_NCHAN; i++){
	 for (j=0; j<4096;j++){
	    mca[4096*i+j]=0;
	    }
	}
    /* clear tdc array */
    for (i=0; i<zDDM_NCHAN; i++){
	 for (j=0; j<1024;j++){
	    tdc[1024*i+j]=0;
	    }
	}

    return(0);
}

/* Write timer preset into register */
STATIC long zDDM_write_preset(zDDMRecord *psr, int val)
{
    Debug(2, "scaler_write_preset(): entry, after = %d\n\r", 1); 
    zDDMRecord *pdet = (zDDMRecord *)psr;
    
    pdet->pr1=val;
    FASTLOCK(&fpga_write_lock);
    fpgabase[COUNT_TIME]=pdet->pr1;
    FASTUNLOCK(&fpga_write_lock);
    return(0);
}



/*****************************************************
* VSCSetup()
* User (startup file) calls this function to configure
* us for the hardware.
*****************************************************/
void devzDDMConfig(int num_cards,	/* maximum number of cards in crate */
	   int ndets,		/* Max. no. detectors */
	   unsigned channels)	/* Max. no. channels */
{
/* For NSLS detector this is null, since this is an embedded system
   and only one configuration is possible. */
extern int zDDM_NCHAN, zDDM_NCHIPS;

   if(num_cards != 1){
   	printf("Configure: Illegal number of cards\n");
   }
   if(ndets != 1){
   	printf("Configure: Illegal number of detectors\n");
   }
   if((channels >384)||(channels<32)){
   	printf("Configure: Illegal number of channels\n");
   }
   else {
   	zDDM_NCHAN = channels;
	zDDM_NCHIPS = channels/32;
	}
}

/******************************************************************
* Routines to assemble SPI strings from arrays, and to talk to 
* ASIC via SPI port.
*******************************************************************/
unsigned int reverseBits(int nobits, unsigned int num) 
{  
//	unsigned int reverse_num = 0; 
//	int i; 
//	for (i = 0; i < nobits; i++) 
//	{ 
//		if((num & (1 << i))) 
//		reverse_num |= 1 << ((nobits - 1) - i); 
//	} 
//	return reverse_num;
	return num; 
} 

int wrap(void *pscal){
    int chip, tmp, j, chn;
    int NCHIPS;
    NCHIPS=zDDM_NCHIPS;
    unsigned int tword,tword2;
    for(chip=0;chip<NCHIPS;chip++){
    printf("wrap: NCHIPS=%i chip=%i\n",NCHIPS, chip);
	/* do globals first */
	j=0;
	tword=0;
	tword=globalstr[chip].tm &1;
/*1 */	j++;	
	tword=tword<<1|(globalstr[chip].sbm &1);
/*2 */ 	j++;
	tword=tword<<1|(globalstr[chip].saux &1);
/*3 */	j++;
	tword=tword<<1|(globalstr[chip].sp &1);
/*4 */	j++;
	tword=tword<<1|(globalstr[chip].slh &1);
/*5 */	j++;
	tword=tword<<2|(globalstr[chip].g&3);
/*7 */	j+=2;
	tword=tword<<5|(globalstr[chip].c&0x1f);
/*12 */	j+=5;
	tword=tword<<2|(globalstr[chip].ss&3);
/*14 */	j+=2;
	tword=tword<<2|(globalstr[chip].tr&3);
/*16 */	j+=2;
	tword=tword<<1|(globalstr[chip].sse &1);
/*17 */	j++;
	tword=tword<<1|(globalstr[chip].spur &1);
/*18 */	j++;
	tword=tword<<1|(globalstr[chip].rt &1);
/*19 */	j++;
	tword=tword<<2|(globalstr[chip].ts&3);
/*21 */	j+=2;
	tword=tword<<1|(globalstr[chip].sl &1);
/*22 */	j++;
	tword=tword<<1|(globalstr[chip].sb &1);
/*23 */	j++;
	tword=tword<<1|(globalstr[chip].sbn &1);
/*24 */	j++;
	tword=tword<<1|(globalstr[chip].m1 &1);
/*25 */	j++;
	tword=tword<<1|(globalstr[chip].m0 &1);
/*26 */	j++;
	tword=tword<<1|(globalstr[chip].senfl2 &1);
/*27 */	j++;
	tword=tword<<1|(globalstr[chip].senfl1 &1);
/*28 */	j++;
	tword=tword<<1|(globalstr[chip].rm &1);
/*29 */	j++;
	tmp=globalstr[chip].pb;
	tword=tword<<3|((tmp>>7)&7);
/*32 */	j+=3;
	
	if(devzDDMdebug>5){ printf("# bits=%i   tword=%x\n",j, tword);
	   }
	loads[chip][0]=tword;
	tword2=0;
	j=0;
	tword2=tword2|(tmp & 0x7f);
/*7 */	j+=7;
	tword2=tword2<<10|(globalstr[chip].pa&0x3ff);
/*17 */	j+=10;

	chn=31;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*18 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*19 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*20 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*21 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*24 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*25 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*29 */	j+=4;
	
	chn=30;	
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*30 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*31 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*32 */	j++;

	if(devzDDMdebug>5){ printf("# bits=%i   tword2=%x\n",j, tword2);
	   }
	loads[chip][1]=tword2;
	tword2=0;

	j=0;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*1 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*4 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*5 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*9 */	j+=4;

	chn=29;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*10 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*11 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*12 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*13 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*16 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*17 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*21 */	j+=4;

	chn=28;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*22 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*23 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*24 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*25 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*28 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*29 */	j++;
	tmp=(channelstr[chip*32+chn].dp&15);
	tword2=tword2<<3|((tmp >>1)&7);
/*32 */	j+=3;
	loads[chip][2]=tword2;
	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	}
	tword2=0;
	j=0;
	tword2=tword2|(tmp &0x1);
/*1 */	j++;
	chn=27;

	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*2 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*3 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*4 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*5 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*8 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*9 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*13 */	j+=4;

	chn=26;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*14 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*15 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*16 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*17 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*20 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*21 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*25 */	j+=4;

	chn=25;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*26 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*27 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*28 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*29 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*32 */	j+=3;

	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	  }
	loads[chip][3]=tword2;

	j=0;

	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*1 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*5 */	j+=4;

	chn=24;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*6 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*7 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*8 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*9 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*12 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*13 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*17 */	j+=4;

	chn=23;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*18 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*19 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*20 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*21 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*24 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*25 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*29 */	j+=4;

	chn=22;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*30 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*31 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*32 */	j++;

	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	  }
	loads[chip][4]=tword2;
	tword2=0;
	j=0;

	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*1 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*4 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*5 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*9 */	j+=4;

	chn=21;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*10 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*11 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*12 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*13 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*16 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*17 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*21 */	j+=4;

	chn=20;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*22 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*23 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*24 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*25 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*28 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*29 */	j++;
	tmp=(channelstr[chip*32+chn].dp&15);
	tword2=tword2<<3|(tmp>>1);
/*32 */	j+=3;

	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	  }
	loads[chip][5]=tword2;
	tword2=0;
	j=0;
	tword2=tword2|(tmp&1);
/*1 */	j++;
	chn=19;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*2 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*3 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*4 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*5 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*8 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*9 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*13 */	j+=4;

	chn=18;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*14 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*15 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*16 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*17 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*20 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*21 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*25 */	j+=4;

	chn=17;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*26 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*27 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*28 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*29 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*32 */	j+=3;

	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	  }
	loads[chip][6]=tword2;
	tword2=0;
	j=0;

	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*1 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*5 */	j+=4;

	chn=16;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*6 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*7 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*8 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*9 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*12 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*13 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*17 */	j+=4;

	chn=15;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*18 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*19 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*20 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*21 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*24 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*25 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*29 */	j+=4;
	
	chn=14;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*30 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*31 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*32 */	j++;

	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	  }
	loads[chip][7]=tword2;
	tword2=0;
	j=0;

	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*1 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*4 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*5 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*9 */	j+=4;

	chn=13;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*10 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*11 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*12 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*13 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*16 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*17 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*21 */	j+=4;

	chn=12;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*22 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*23 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*24 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*25 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*28 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*29 */	j++;
	tmp=(channelstr[chip*32+chn].dp&15);
	tword2=tword2<<3|(tmp>>1);
/*32 */	j+=3;

	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	  }
	loads[chip][8]=tword2;
	tword2=0;
	j=0;

	tword2=tword2|(tmp&1);
/*1 */	j+=1;

	chn=11;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*2 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*3 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*4 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*5 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*8 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*9 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*13 */	j+=4;

	chn=10;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*14 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*15 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*16 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*17 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*20 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*21 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*25 */	j+=4;

	chn=9;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*26 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*27 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*28 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*29 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*32 */	j+=3;

	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	  }
	loads[chip][9]=tword2;
	tword2=0;
	j=0;

	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*1 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*5 */	j+=4;
	
	chn=8;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*6 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*7 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*8 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*9 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*12 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*13 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*17 */	j+=4;

	chn=7;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*18 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*19 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*20 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*21 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*24 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*25 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*29 */	j+=4;

	chn=6;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*30 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*31 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*32 */	j++;


	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	  }
	loads[chip][10]=tword2;
	tword2=0;
	j=0;
	
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*1 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*4 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*5 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*9 */	j+=4;


	chn=5;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*10 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*11 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*12 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*13 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*16 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*17 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*21 */	j+=4;

 	chn=4;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*22 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*23 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*24 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*25 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*28 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*29 */	j++;
	tmp=(channelstr[chip*32+chn].dp&15);
	tword2=tword2<<3|(tmp>>1);
/*32 */	j+=3;

 	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	  }
	loads[chip][11]=tword2;
	tword2=0;
	j=0;

	tword2=tword2|(tmp&1);
/*1 */	j++;
	chn=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*2 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*3 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*4 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*5 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*8 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*9 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*13 */	j+=4;

	chn=2;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*14 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*15 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*16 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*17 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*20 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*21 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*25 */	j+=4;

	chn=1;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*26 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*27 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*28 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*29 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*32 */	j+=3;

	if(devzDDMdebug>5){  printf("# bits=%i   tword2=%x\n",j, tword2);
	  }
	loads[chip][12]=tword2;
	tword2=0;
	j=0;

	tword2=tword2|(channelstr[chip*32+chn].nc1&1);
/*1 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*5 */	j+=4;

	chn=0;
	tword2=tword2<<1|(channelstr[chip*32+chn].st&1);
/*6 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sm&1);
/*7 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc2&1);
/*8 */	j++;
	tword2=tword2<<1|(channelstr[chip*32+chn].sel&1);
/*9 */	j++;
	tword2=tword2<<3|(channelstr[chip*32+chn].da&7);
/*12 */	j+=3;
	tword2=tword2<<1|(channelstr[chip*32+chn].nc1&1);
/*13 */	j++;
	tword2=tword2<<4|(channelstr[chip*32+chn].dp&15);
/*17 */	j+=4;
	tword2=tword2<<15;
/*32 */	j+=15;

	if(devzDDMdebug>5){ printf("# bits=%i  chip=%i tword2=%x\n",j, chip, tword2);
	   }
	loads[chip][13]=tword2;
    }
return(0);
}


STATIC long zDDM_done(zDDMRecord *psr)
{
    devPVT *pPvt = (devPVT *)psr->dpvt;
    Debug(2, "scaler_done(): entry, pPvt->done = %d\n\r", pPvt->done);
    if (pPvt->done) {
        pPvt->done = 0;
        return(1);
    } else {
        return(0);
    }
}


void latch_conf(void){
   /* toggle load flag */
   fpgabase[MARS_CONF_LOAD]=2;
   fpgabase[MARS_CONF_LOAD]=0;
   }
   
   
   
int stuff_mars(void *pscal){ /* stuff structs into chp */
int i, j;

  wrap(pscal); /* set up all bits and regs in loads[][] array */

/* send them out */
FASTLOCK(&fpga_write_lock);

for(i=0;i<12;i++){
fpgabase[MARS_CONF_LOAD]=4; /* Reset config FIFO */
fpgabase[MARS_CONF_LOAD]=0;
 for(j=0;j<14;j++){
   fpgabase[MARS_CONFIG]=loads[i][j];
   latch_conf();
   usleep(1000);
   }
 switch(i){
   case 0: fpgabase[MARS_CONF_LOAD]=0x00010000;
      break;
   case 1:fpgabase[MARS_CONF_LOAD]=0x00020000;
      break;
   case 2: fpgabase[MARS_CONF_LOAD]=0x00040000;
      break;
   case 3: fpgabase[MARS_CONF_LOAD]=0x00080000;
      break;
   case 4: fpgabase[MARS_CONF_LOAD]=0x00100000;
      break;
   case 5: fpgabase[MARS_CONF_LOAD]=0x00200000;
      break;
   case 6: fpgabase[MARS_CONF_LOAD]=0x00400000;
      break;
   case 7: fpgabase[MARS_CONF_LOAD]=0x00800000;
      break;
   case 8: fpgabase[MARS_CONF_LOAD]=0x01000000;
      break;
   case 9: fpgabase[MARS_CONF_LOAD]=0x02000000;
      break;
   case 10: fpgabase[MARS_CONF_LOAD]=0x04000000;
      break;
   case 11: fpgabase[MARS_CONF_LOAD]=0x08000000;
      break;
   } 
   fpgabase[MARS_CONF_LOAD]=0;
   usleep(1000);
 }
FASTUNLOCK(&fpga_write_lock);
return(0);
}

/*****************************************
* Send out an SPI bit to fast ADC config.
* SPI Register 
*   bit 0 = data
*   bit 1 = clk
******************************************/
void send_spi_bit(int chipSel, int val)
{
   int sda;

   sda = val & 0x1;
   //printf("%d ",(val & 0x1));

   //set sclk low  
   fpgabase[ADC_SPI] = chipSel | 0;
   //set data with clock low
   fpgabase[ADC_SPI] = chipSel | sda;
   //set clk high 
   fpgabase[ADC_SPI] =  chipSel | 0x2 | sda;
   //set clk low
   fpgabase[ADC_SPI] =  chipSel | sda;
   //set data low
   fpgabase[ADC_SPI] = chipSel | 0;


}

int load_ad9252reg(int chipSel, int addr, int data)
{

    int i,j,k;

   // small delay
   for (k=0;k<100;k++);
  
   // Read/Write bit 
   send_spi_bit(chipSel,0); 

   // W1=W0=0 (word length = 1 byte)
   for (i=1;i>=0;i--) send_spi_bit(chipSel,0);
   
   // address 
   for (j=12;j>=0;j--)  send_spi_bit(chipSel,addr >> j);
   
    // data   
    for (j=7;j>=0;j--)  send_spi_bit(chipSel,data >> j);

   // small delay 
   for (k=0;k<100;k++);
   return(0);

}

int ad9252_cnfg(int chipNum, int addr, int data){

int chipSel;

  //chipSel is defined as: bit2=ADC0, bit3=ADC1, bit4=ADC2
   if (chipNum == 1)        chipSel = 0b11000;
   else if (chipNum == 2)   chipSel = 0b10100;
   else if (chipNum == 3)   chipSel = 0b01100;
   else                     chipSel = 0b00000;


       //Assert CSB
       printf("Writing Register...\n"); 
       fpgabase[ADC_SPI] = chipSel;
       load_ad9252reg(chipSel,addr,data);
       fpgabase[ADC_SPI] = 0b11100;
       printf("Value Written...\n"); 
   return(0); 
}

int ldaddrlkup(){
int lkuptbl[512];
int i;
int indx[384]={25, 24, 31, 23, 30, 22, 29, 21, 28, 20, 27,\
     19, 26, 18, 17, 16, 15, 14, 13, 5, 12, 4, 11,\
     3, 10, 2, 9, 1, 8, 0, 7, 6, 57, 56, 63, 55, 62,\
     54, 61, 53, 60, 52, 59, 51, 58, 50, 49, 48, 47,\
     46, 45, 37, 44, 36, 43, 35, 42, 34, 41, 33, 40,\
     32, 39, 38, 89, 88, 95, 87, 94, 86, 93, 85, 92,\
     84, 91, 83, 90, 82, 81, 80, 79, 78, 77, 69, 76,\
     68, 75, 67, 74, 66, 73, 65, 72, 64, 71, 70, 153,\
     152, 159, 151, 158, 150, 157, 149, 156, 148, 155,\
     147, 154, 146, 145, 144, 143, 142, 141, 133, 140,\
     132, 139, 131, 138, 130, 137, 129, 136, 128, 135,\
     134, 185, 184, 191, 183, 190, 182, 189, 181, 188,\
     180, 187, 179, 186, 178, 177, 176, 175, 174, 173,\
     165, 172, 164, 171, 163, 170, 162, 169, 161, 168,\
     160, 167, 166, 217, 216, 223, 215, 222, 214, 221,\
     213, 220, 212, 219, 211, 218, 210, 209, 208, 207,\
     206, 205, 197, 204, 196, 203, 195, 202, 194, 201,\
     193, 200, 192, 199, 198, 281, 280, 287, 279, 286,\
     278, 285, 277, 284, 276, 283, 275, 282, 274, 273,\
     272, 271, 270, 269, 261, 268, 260, 267, 259, 266,\
     258, 265, 257, 264, 256, 263, 262, 313, 312, 319,\
     311, 318, 310, 317, 309, 316, 308, 315, 307, 314,\
     306, 305, 304, 303, 302, 301, 293, 300, 292, 299,\
     291, 298, 290, 297, 289, 296, 288, 295, 294, 345,\
     344, 351, 343, 350, 342, 349, 341, 348, 340, 347,\
     339, 346, 338, 337, 336, 335, 334, 333, 325, 332,\
     324, 331, 323, 330, 322, 329, 321, 328, 320, 327,\
     326, 409, 408, 415, 407, 414, 406, 413, 405, 412,\
     404, 411, 403, 410, 402, 401, 400, 399, 398, 397,\
     389, 396, 388, 395, 387, 394, 386, 393, 385, 392,\
     384, 391, 390, 441, 440, 447, 439, 446, 438, 445,\
     437, 444, 436, 443, 435, 442, 434, 433, 432, 431,\
     430, 429, 421, 428, 420, 427, 419, 426, 418, 425,\
     417, 424, 416, 423, 422, 473, 472, 479, 471, 478,\
     470, 477, 469, 476, 468, 475, 467, 474, 466, 465,\
     464, 463, 462, 461, 453, 460, 452, 459, 451, 458,\
     450, 457, 449, 456, 448, 455, 454};
     
   for (i=0;i<384;i++){
       lkuptbl[i] = indx[i];
       }
   for (i=384;i<512;i++){
       lkuptbl[i] = 400;
       }
   
   for (i=0;i<512;i++) {
//       val = (lkuptbl[i]<<16)+i;
//       fpgabase[LKUPADDRREG] = val;
//       printf("%d\t%d\t%x\n",i,lkuptbl[i],val);
       }
   Debug(10,"Loaded address map %i\n",0);
return(0);
}



static const iocshArg devzDDMConfigArg0 = {"Card #", iocshArgInt};
static const iocshArg devzDDMConfigArg1 = {"No. detectors", iocshArgInt};
static const iocshArg devzDDMConfigArg2 = {"No. channels", iocshArgInt};

static const iocshArg * const devzDDMConfigArgs[3] = {&devzDDMConfigArg0,
                                                       &devzDDMConfigArg1,
                                                       &devzDDMConfigArg2,
                                                       };

static const iocshFuncDef devzDDMConfigFuncDef={"devzDDMConfig",3,devzDDMConfigArgs};
static void devzDDMConfigCallFunc(const iocshArgBuf *args)
{
 devzDDMConfig((int) args[0].ival, (unsigned short) args[1].ival, (int) args[2]
.ival);
}

void registerzDDMConfig(void){
        iocshRegister(&devzDDMConfigFuncDef,&devzDDMConfigCallFunc);
        }

epicsExportRegistrar(registerzDDMConfig);

int peek(int reg){
	printf("Register %i = %i\n",reg, fpgabase[reg]);
	return(0);
	}

static const iocshArg peekArg0 = {"Register #", iocshArgInt};
static const iocshArg * const peekArgs[1] = {&peekArg0};

static const iocshFuncDef peekFuncDef={"peek",1,peekArgs};
static void peekCallFunc(const iocshArgBuf *args)
{
 peek((int) args[0].ival);
}

void registerpeek(void){
        iocshRegister(&peekFuncDef,&peekCallFunc);
        }

epicsExportRegistrar(registerpeek);


int poke(int reg, int val){
	/*zmq_write(reg,val);*/
	fpgabase[reg]=val;
	peek(reg);
	return(0);
	}



static const iocshArg pokeArg0 = {"Register #", iocshArgInt};
static const iocshArg pokeArg1 = {"Val", iocshArgInt};
static const iocshArg * const pokeArgs[2] = {&pokeArg0,&pokeArg1,};


static const iocshFuncDef pokeFuncDef={"poke",2,pokeArgs};
static void pokeCallFunc(const iocshArgBuf *args)
{
 poke((int) args[0].ival, (int) args[1].ival);
}

void registerpoke(void){
        iocshRegister(&pokeFuncDef,&pokeCallFunc);
        }

epicsExportRegistrar(registerpoke);

int bits(int chan){
  int chip, chn, i;

	chip=chan/32;
	chn=chan%32; 

	printf("Settings for channel %i, chip %i\n\n",chn,chip);
	
	printf("pa=%x\n",globalstr[chip].pa);   
	printf("pb=%x\n",globalstr[chip].pb);   
	printf("rm=%x\n",globalstr[chip].rm);   
	printf("senfl1=%x\n",globalstr[chip].senfl1);
	printf("senfl2=%x\n",globalstr[chip].senfl2);
	printf("m0=%x\n",globalstr[chip].m0);   
	printf("m1=%x\n",globalstr[chip].m1);   
	printf("sbn=%x\n",globalstr[chip].sbn);  
	printf("sb=%x\n",globalstr[chip].sb);   
	printf("sl=%x\n",globalstr[chip].sl);   
	printf("ts=%x\n",globalstr[chip].ts);   
	printf("rt=%x\n",globalstr[chip].rt);   
	printf("spur=%x\n",globalstr[chip].spur); 
	printf("sse=%x\n",globalstr[chip].sse);  
	printf("tr=%x\n",globalstr[chip].tr);   
	printf("ss=%x\n",globalstr[chip].ss);   
	printf("c=%x\n",globalstr[chip].c);    
	printf("g=%x\n",globalstr[chip].g);    
	printf("slh=%x\n",globalstr[chip].slh);  
	printf("sp=%x\n",globalstr[chip].sp);   
	printf("saux=%x\n",globalstr[chip].saux); 
	printf("sbm=%x\n",globalstr[chip].sbm);  
	printf("tm=%x\n",globalstr[chip].tm);   
	printf("\n");
	printf("dp=%x\n",channelstr[chip*32+chn].dp); 
	printf("nc1=%x\n",0);
	printf("da=%x\n",channelstr[chip*32+chn].da); 
	printf("sel=%x\n",channelstr[chip*32+chn].sel);
	printf("nc2=%x\n",0);
	printf("sm=%x\n",channelstr[chip*32+chn].sm); 
	printf("st=%x\n",channelstr[chip*32+chn].st);
	for(i=0;i<14;i++){
	  printf("%x\n",loads[chip][i]);
	  }
	return(0);
	} 

static const iocshArg bitsArg0 = {"Channel #", iocshArgInt};
static const iocshArg * const bitsArgs[1] = {&bitsArg0};

static const iocshFuncDef bitsFuncDef={"bits",1,bitsArgs};
static void bitsCallFunc(const iocshArgBuf *args)
{
 bits((int) args[0].ival);
}

void registerbits(void){
        iocshRegister(&bitsFuncDef,&bitsCallFunc);
        }

epicsExportRegistrar(registerbits); 


