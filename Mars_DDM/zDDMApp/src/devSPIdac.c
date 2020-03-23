/*To Use this device support, Include the following before iocInit */
/* devSPIConfig(card,a32base,nreg)  */
/*    card    = card number                           */
/*    a32base = base address of card                  */
/*    nreg    = number of A32 registers on this card  */
/* For Example                                        */
/* devSPIConfig(0, 0x10000000, 1)        */
/* card is interpreted as ID of SPI device */
/* a32base is ignored for SPI devices, but must be non-zero. */
/* nreg is number of signals in device */

 /**********************************************************************/
 /** Brief Description of device support                              **/
 /**                                                                  **/
 /** THis support provide access to devices attached to the microzed  **/
 /** SPI interface, in particular DAC's and ADC's. Only ao            **/
 /** records are supported.                                           **/
 /**      **/
 /**      **/
 /**      **/
 /**                                                                  **/
 /** Record type     Signal #           Parm Field                    **/
 /**                                                                  **/
 /**    ao          reg                width, type;                   **/
 /**                                                                  **/
 /** reg specifies the register number (0 - 7) for chips with multiple**/
 /** channels.                                                        **/
 /** Parm field must be provided, no defaults are assumed ...         **/
 /** In ai and ao type is either 0 - unipolar, 1 -bipolar             **/
 /**                                                                  **/
 /**                                                                  **/
 /**********************************************************************/
#define FAST_LOCK epicsMutexId
#define FASTLOCKINIT(PFAST_LOCK) (*(PFAST_LOCK) = epicsMutexCreate())                                        
#define FASTLOCK(PFAST_LOCK) epicsMutexLock(*(PFAST_LOCK));                                                  
#define TRYLOCK(PFAST_LOCK) epicsMutexTryLock(*(PFAST_LOCK));                                                
#define FASTUNLOCK(PFAST_LOCK) epicsMutexUnlock(*(PFAST_LOCK));                                              
#define ERROR -1                                                                                             
#define OK 0

/*#include      <vme.h>*/
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

/*#include      <module_types.h> */

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


/*
 * http://elinux.org/Interfacing_with_SPI_Devices
 * https://www.kernel.org/doc/Documentation/SPI/dev-interface
 * http://blog.chrysocome.net/2013/03/programming-SPI.html
 *
 */


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

#define MAX_NUM_CARDS    8
#define MAX_ACTIVE_REGS  8   /* max registers allowed */
#define MAX_ACTIVE_BITS  16   /* largest bnumber of bits expected */
#define MAX_A32_ADDRESS  7      /* 3 bits to decode for SPI chip addresses */
#define MIN_A32_ADDRESS  0
#define SPI_INPUT	0
#define SPI_OUTPUT	1

extern FAST_LOCK       SPI_lock;

typedef struct ioCard {  /* Unique for each card */
  unsigned short    base;    /* index of this card's chip */
  int		    card_type; /* 0 or 1, input or output: SPI is ro or wo, not rw */
  int		    nReg;    /* Number of registers in this chip */
  FAST_LOCK	    lock;    /* semaphore */
  IOSCANPVT	    ioscanpvt; /* records to process upon interrupt */
}ioCard;

static struct ioCard cards[MAX_NUM_CARDS]; /* array of card info */

typedef struct SPIDpvt { /* unique for each record */
  int  reg;   /* index of sub-device (determined by signal #*/
  int  nbit;  /* no of significant bits */
  int  type;  /* Type either 0 or 1 for uni, bi polar */
  unsigned long  value; /* to provide readback for ao records */
}SPIDpvt;



/* Create the dset for devSPI */
// static long init_ai(), read_ai();
static long init_ao(), write_ao();

#define Debug0(l,FMT) {  if(l <= devSPIdebug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT); } }
#define Debug(l,FMT,V) {  if(l <= devSPIdebug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT,V); } }


volatile int  devSPIDebug = 0;
epicsExportAddress(int, devSPIDebug);


typedef struct {
	long number;
	DEVSUPFUN report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_write;
	DEVSUPFUN special_linconv;
	}SPIdset;

//SPIdset devAiSPI={
//		6,
//		NULL,
//		NULL,
//		init_ai,
//		NULL,
//		read_ai,
//		NULL
//		};

SPIdset devAoSPI={
		6,
		NULL,
		NULL,
		init_ao,
		NULL,
		write_ao,
		NULL
		};
		
//epicsExportAddress(dset,devAiSPI);
epicsExportAddress(dset,devAoSPI);


int devSPIConfig(card,a32base,nregs)
int           card;
unsigned short a32base;
int           nregs;
/*int         iVector;
int           iLevel;*/
{

int temp;

  if((card >= MAX_NUM_CARDS) || (card < 0)) {
      errlogPrintf("devSPIConfig: Invalid Card # %d \n",card);
      return(ERROR);
  }
  else{
        errlogPrintf("devSPIConfig: Valid Card # %d \n",card);
        }

  if((a32base >= MAX_A32_ADDRESS)||(a32base <= MIN_A32_ADDRESS)) {
      errlogPrintf("devSPIConfig: Invalid Card Address %d \n",a32base);
      return(ERROR);
  }
  else {
        cards[card].base=a32base;
        errlogPrintf("devSPIConfig: Valid Card Address %d \n",a32base);
        }


  if(nregs > MAX_ACTIVE_REGS) {
      errlogPrintf("devSPIConfig: # of registers (%d) exceeds max\n",nregs);
      return(ERROR);
  }
  else {
      cards[card].nReg = nregs;
      errlogPrintf("devSPIConfig: # of registers = %d\n",nregs);
  }

//      FASTLOCKINIT(&(SPI_lock));
//      FASTUNLOCK(&(SPI_lock));  /* Init the board lock */      

  return(OK);
}

static const iocshArg devSPIConfigArg0 = {"Card #", iocshArgInt};
static const iocshArg devSPIConfigArg1 = {"Base address", iocshArgInt};
static const iocshArg devSPIConfigArg2 = {"No. regs", iocshArgInt};

static const iocshArg * const devSPIConfigArgs[3] = {&devSPIConfigArg0,
                                                       &devSPIConfigArg1,
                                                       &devSPIConfigArg2,
                                                       };
                                                       
static const iocshFuncDef devSPIConfigFuncDef={"devSPIConfig",3,devSPIConfigArgs};
static void devSPIConfigCallFunc(const iocshArgBuf *args)
{
 devSPIConfig((int) args[0].ival, (unsigned short) args[1].ival, (int) args[2].ival);
}

void registerSPIConfig(void){
        iocshRegister(&devSPIConfigFuncDef,&devSPIConfigCallFunc);
        }

epicsExportRegistrar(registerSPIConfig);



static long init_ao(void *precord)
{
aoRecord *pao = (aoRecord *)precord;
long status = 0L;
unsigned long rawVal = 0L;
int             card,
		reg, 
		args, 
		numBits, 
		twotype;
SPIDpvt         *pPvt;

//FASTLOCK(&SPI_lock);
switch (pao->out.type) {
	case (VME_IO) :
		card = pao->out.value.vmeio.card;
		reg = pao->out.value.vmeio.signal;
		if(card > MAX_NUM_CARDS){
		  pao->pact = 1;          /* make sure we don't process this thing */
                  errlogPrintf("devSPI: Card #%d exceeds max: ->%s<- \n",pao->out.value.vmeio.card, pao->name);
                  return(ERROR);
                  }
                if (reg >= MAX_ACTIVE_REGS) {
	         pao->pact = 1;          /* make sure we don't process this thing */
	         errlogPrintf("devSPI: Signal #%d exceeds %i registers: ->%s<-\n",reg,cards[card].nReg,pao->name);
        return(ERROR);
      }
      args = sscanf(pao->out.value.vmeio.parm, "%d,%d", &numBits, &twotype);

      if( (args != 2) || (numBits <= 0) ||
         	(numBits > MAX_ACTIVE_BITS) ||
         	(twotype > 1) || (twotype < 0) ) {
        errlogPrintf("devSPI: Invalid Width/Type in parm field: ->%s<-\n",pao->name);
        return(ERROR);
      }

      cards[card].card_type=SPI_OUTPUT;
      
      pao->dpvt = (void *)calloc(1, sizeof(struct SPIDpvt));
      pPvt = (SPIDpvt *)pao->dpvt;

      pPvt->reg =  pao->out.value.vmeio.signal;
      pPvt->nbit = numBits;
      pPvt->type = twotype;
      pPvt->value = rawVal;
      if (devSPIDebug >= 20)
        errlogPrintf("init_ao 1:\n\tpPvt->reg %i\n\tpPvt->nbit %i\n\tpPvt->type %i\n\tpPvt->value %i \n\r",pPvt->reg,pPvt->nbit,pPvt->type,pPvt->value);


      pao->eslo = (pao->eguf - pao->egul)/(pow(2,numBits)-1);

/*  Shift Raw value if Bi-polar */
      if (pPvt->type == 1) 
         pao->roff = pow(2,(numBits-1));

      /* Init rval to current setting */ 
//      if(ReadDac(card,reg,&pPvt,&rawVal) == OK) {
//        pao->rbv = rawVal;
//      if (devSPIDebug >= 20)
//        errlogPrintf("init_ao 2: pao->rbv %i\n",pao->rbv);
//	}
/* here is where we do the sign extensions for Bipolar.... */        
//        if (pPvt->type ==1) {
//           if (pao->rbv & (2<<(pPvt->nbit-2)))
//               pao->rbv |= ((2<<31) - (2<<(pPvt->nbit-2)))*2 ;
//	}

//        pao->rval = pao->rbv;

      if (devSPIDebug >= 20)
        errlogPrintf("init_ao 3: pao->rval %i\n",pao->rval);

		status = OK;
		break;
	default :
		recGblRecordError(S_db_badField, (void *)pao,"devAoSPI (init_record) Illegal OUT field");
		return(S_db_badField);
	}
/* Make sure record processing routine does not perform any conversion*/
//	pao->linr=3;
//FASTUNLOCK(&SPI_lock);
return(0);
}

static long write_ao(void *precord)
{
aoRecord *pao =(aoRecord *)precord;
  long status;
  unsigned long         rawVal=0;
  int                   card,reg;
  SPIDpvt             *pPvt = (SPIDpvt *)pao->dpvt;

  card = pao->out.value.vmeio.card;
  reg = pao->out.value.vmeio.signal;
  errlogPrintf("Card = %i, Signal = %i\n",card,reg);
  if(epicsMutexTryLock(&SPI_lock)==0){
       epicsMutexShow(&SPI_lock,1);
       printf("We don't own SPI_lock!\n");
       }
  FASTLOCK(&SPI_lock);
//  if (
  SPI_WriteDac(card,reg,pao->rval); // == OK)
//  {
//    if(ReadDac(card,reg,&rawVal)== OK)
//    {
//      pao->rbv = rawVal;
//      if (devSPIDebug >= 20)
//          errlogPrintf("write_ao: card %i  reg %i rawVal %i\n\r", card, reg, rawVal);
//
//
/* here is where we do the sign extensions for Bipolar.... */        
//        if (pPvt->type ==1) {
//           if (pao->rbv & (2<<(pPvt->nbit-2)))
//               pao->rbv |= ((2<<31) - (2<<(pPvt->nbit-2)))*2;
//
//        }
//  FASTUNLOCK(&SPI_lock);      
//      return(0);
//    }
//  }
  FASTUNLOCK(&SPI_lock);
  /* Set an alarm for the record */
  recGblSetSevr(pao, WRITE_ALARM, INVALID_ALARM);
  return(2);
}

/******************************************
* Program DAC 
*    Addr 0 => ChanA : E4 testpoint
*    Addr 1 => ChanB : E5 testpoint
*    Addr 2 => ChanC : E6 testpoint
*    Addr 3 => ChanD : E7 testpoint
*    Addr 4 => ChanE : Offset-PD
*    Addr 5 => ChanF : Guard-Bias
*    Addr 6 => ChanG : Offset-TD
*    Addr 7 => ChanH : Cal-DAC
* 
*  SDI stream is 24 bits (4 bit command, 4 bit address, 16 bit data)
*  Command use 0011, Addr and Data is MSB first
*
******************************************/
int prog_dac(int addr, int value)
{

   int i;

   //send command bits 
   for (i=3;i>=0;i--)  send_spi_bit(0x3 >> i);
   //send address bits
   for (i=3;i>=0;i--)  send_spi_bit(addr >> i);
   //send data bits 
   for (i=15;i>=0;i--)  send_spi_bit(value >> i);

}


int SPI_WriteDac(int card, int channel, int val)
{
    extern int zDDM_NCHIPS;
    char buf[3] = {0};
    int i, bytesWritten;
    short int dacWord; 
    
    dacWord = val;
    if (dacWord > 65535)  dacWord = 65535; 
    if (dacWord < 0)     dacWord = 0;
    if (devSPIDebug >= 20){
       errlogPrintf("Set DAC: %i\n",val);
       errlogPrintf("DAC Word: %d   (0x%x)\n",dacWord,dacWord);
       }

   // pass token until the dac
   set_token();
   for (i=0;i<zDDM_NCHIPS;i++) {
     clock_token();
     if (i == 0) clear_token();
   }

   for (i=0;i<zDDM_NCHIPS;i++)
     clock_token();
  
   // load 8 channel DAC
   prog_dac(channel,val);

   clock_token();
   // Skip analog switch
   clock_token();

   //give some extra tck strobes just for fun
   for (i=0;i<5;i++)
      clock_token();
    if (devSPIDebug >= 20) errlogPrintf("Chip %i DAC %i Written...  Bytes Written : %d\n",card, channel,bytesWritten);
    return(OK);


}
