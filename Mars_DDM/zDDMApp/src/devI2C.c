/*      This version has all spi devices selected from external decoder */

/*To Use this device support, Include the following before iocInit */
/* devI2CConfig(card,a32base,nreg)  */
/*    card    = card number                           */
/*    a32base = base address of card                  */
/*    nreg    = number of A32 registers on this card  */
/* For Example                                        */
/* devI2CConfig(0, 0x10000000, 1)        */
/* card is interpreted as ID of SPI device */
/* a32base is ignored for SPI devices, but must be non-zero. */
/* nreg is number of signals in device */

 /**********************************************************************/
 /** Brief Description of device support                              **/
 /**                                                                  **/
 /** THis support provide access to devices attached to the microzed  **/
 /** I2C interface, in particular DAC's and ADC's. Only ao and ai     **/
 /** records are supported.                                           **/
 /**      **/
 /**      **/
 /**      **/
 /**                                                                  **/
 /** Record type     Signal #           Parm Field                    **/
 /**                                                                  **/
 /**    ai          reg                width, type;                   **/
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
 * http://elinux.org/Interfacing_with_I2C_Devices
 * https://www.kernel.org/doc/Documentation/i2c/dev-interface
 * http://blog.chrysocome.net/2013/03/programming-i2c.html
 *
 */


#include <errno.h>
#include <errlog.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define MAX_NUM_CARDS    10
#define MAX_ACTIVE_REGS  8    /* max registers allowed */
#define MAX_ACTIVE_BITS  16   /* largest bnumber of bits expected */
#define MAX_A32_ADDRESS  255  /* I2C Address of chip */
#define MIN_A32_ADDRESS  0
#define I2C_INPUT	0
#define I2C_OUTPUT	1

FAST_LOCK       I2C_lock_1; /* lock on i2c interface 1 */

int I2Cdev; /* file number for i2c interface */
/*extern int addr; */  /* i2c vendor-specific base address of DAC chips */
int addr = 0b01001010;        // The I2C address of the DAC


typedef struct ioCard {  /* Unique for each card */
  unsigned short    base;    /* index of this card's chip */
  int		    card_type; /* 0 or 1, input or output: SPI is ro or wo, not rw */
  int		    nReg;    /* Number of registers in this chip */
  FAST_LOCK	    lock;    /* semaphore */
  IOSCANPVT	    ioscanpvt; /* records to process upon interrupt */
}ioCard;

static struct ioCard cards[MAX_NUM_CARDS]; /* array of card info */

typedef struct I2CDpvt { /* unique for each record */
  int  reg;   /* index of sub-device (determined by signal #*/
  int  nbit;  /* no of significant bits */
  int  type;  /* Type either 0 or 1 for uni, bi polar */
  unsigned long  value; /* to provide readback for ao records */
}I2CDpvt;



/* Create the dset for devI2C */
static long init_ai(), read_ai();
static long init_ao(), write_ao();

#define Debug0(l,FMT) {  if(l <= devI2Cdebug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT); } }
#define Debug(l,FMT,V) {  if(l <= devI2Cdebug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT,V); } }


volatile int  devI2CDebug = 0;
epicsExportAddress(int, devI2CDebug);


typedef struct {
	long number;
	DEVSUPFUN report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read;
	DEVSUPFUN special_linconv;
	}aiI2Cdset;

typedef struct {
	long number;
	DEVSUPFUN report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN write;
	DEVSUPFUN special_linconv;
	}aoI2Cdset;

aiI2Cdset devAiI2C={
		6,
		NULL,
		NULL,
		init_ai,
		NULL,
		read_ai,
		NULL
		};

aoI2Cdset devAoI2C={
		6,
		NULL,
		NULL,
		init_ao,
		NULL,
		write_ao,
		NULL
		};
		
epicsExportAddress(dset,devAiI2C);
epicsExportAddress(dset,devAoI2C);


int devI2CConfig(int card, unsigned short a32base, int nregs)
{
int temp;
char filename[32];

  if((card >= MAX_NUM_CARDS) || (card < 0)) {
      errlogPrintf("devI2CConfig: Invalid Card # %d \n",card);
      return(ERROR);
  }
  else{
        errlogPrintf("devI2CConfig: Valid Card # %d \n",card);
        }

  if((a32base >= MAX_A32_ADDRESS)||(a32base <= MIN_A32_ADDRESS)) {
      errlogPrintf("devI2CConfig: Invalid Card Address %d \n",a32base);
      return(ERROR);
  }
  else {
        cards[card].base=a32base;
        errlogPrintf("devI2CConfig: Valid Card Address %d \n",a32base);
        }


  if(nregs > MAX_ACTIVE_REGS) {
      errlogPrintf("devI2CConfig: # of registers (%d) exceeds max\n",nregs);
      return(ERROR);
  }
  else {
      cards[card].nReg = nregs;
      cards[card].base = a32base;
      errlogPrintf("devI2CConfig: # of registers = %d\n",nregs);
  }

      FASTLOCKINIT(&(I2C_lock_1));

  return(OK);
}

static const iocshArg devI2CConfigArg0 = {"Card #", iocshArgInt};
static const iocshArg devI2CConfigArg1 = {"Base address", iocshArgInt};
static const iocshArg devI2CConfigArg2 = {"No. regs", iocshArgInt};

static const iocshArg * const devI2CConfigArgs[3] = {&devI2CConfigArg0,
                                                       &devI2CConfigArg1,
                                                       &devI2CConfigArg2,
                                                       };
                                                       
static const iocshFuncDef devI2CConfigFuncDef={"devI2CConfig",3,devI2CConfigArgs};
static void devI2CConfigCallFunc(const iocshArgBuf *args)
{
 devI2CConfig((int) args[0].ival, (unsigned short) args[1].ival, (int) args[2].ival);
}

void registerI2CConfig(void){
        iocshRegister(&devI2CConfigFuncDef,&devI2CConfigCallFunc);
        }

epicsExportRegistrar(registerI2CConfig);

int EnableIntRef(int card)
{
    int dacaddr = cards[card].base;        // The I2C address of the DAC    
    char buf[3] = {0};
    char filename[20];
    int bytesWritten;
    sprintf(filename,"/dev/i2c-1");
   FASTLOCK(&I2C_lock_1);
    if ((I2Cdev = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
	FASTUNLOCK(&I2C_lock_1);
        exit(1);
    } 

    if (ioctl(I2Cdev,I2C_SLAVE,dacaddr) < 0) {
	errlogPrintf("Failed to acquire bus access and/or talk to slave.\n");
	close(I2Cdev);
	FASTUNLOCK(&I2C_lock_1);
	return(-1);
	}

    buf[0] = 0x80;  //Command Access Byte
    buf[1] = 0x00;
    buf[2] = 0x10;
    bytesWritten=write(I2Cdev,buf,3);
    if (bytesWritten != 3){
       errlogPrintf("Error Writing DAC to Set Int Ref...   Bytes Written: %d\n",bytesWritten);
       close(I2Cdev);
       FASTUNLOCK(&I2C_lock_1);
       return(ERROR);
       }
    else {
       errlogPrintf("Internal Reference Enabled...\n");
       close(I2Cdev);
       FASTUNLOCK(&I2C_lock_1);
       return(OK);
       }
       
}


int WriteDac(int card, int channel, int val)
{
    int dacaddr = cards[card].base;        // The I2C address of the DAC 
    char buf[3] = {0};
    char filename[20];
    int bytesWritten;
    int test;
    short int dacWord; 
        
    dacWord = val;
    if (dacWord > 4095)  dacWord = 4095; 
    if (dacWord < 0)     dacWord = 0;
    if (devI2CDebug >= 20){
       errlogPrintf("Set DAC: %i\n",val);
       errlogPrintf("DAC Word: %d   (0x%x)\n",dacWord,dacWord);
       }
       sprintf(filename,"/dev/i2c-1");
    FASTLOCK(&I2C_lock_1);
    if ((I2Cdev = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
	FASTUNLOCK(&I2C_lock_1);
	exit(1);
    } 

    if (ioctl(I2Cdev,I2C_SLAVE,dacaddr) < 0) {
	errlogPrintf("Failed to acquire bus access and/or talk to slave.\n");
	close(I2Cdev);
        FASTUNLOCK(&I2C_lock_1);	
	return(-1);
	}  
    buf[0] = 0x30 + channel; //Command Access Byte
    buf[1] = (char)((dacWord & 0x0FF0) >> 4);
    buf[2] = (char)((dacWord & 0x000F) << 4);
//    printf("MSB: %x    LSB: %x\n",buf[1],buf[2]);

    bytesWritten = write(I2Cdev,buf,3);
    if (devI2CDebug >= 20) errlogPrintf("Chip %i DAC %i Written...  Bytes Written : %d\n",card, channel,bytesWritten);
    close(I2Cdev);
    FASTUNLOCK(&I2C_lock_1);
    return(OK);
}

int ReadDac(int card, int channel, unsigned long *val)
{
   char buf[3] = {0};
   char filename[20];
   int bytesWritten, bytesRead;
   int voltage;
   
   FASTLOCK(&I2C_lock_1);
    sprintf(filename,"/dev/i2c-1");
    if ((I2Cdev = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
	FASTUNLOCK(&I2C_lock_1);
        exit(1);
    } 

   if (ioctl(I2Cdev,I2C_SLAVE,cards[card].base) < 0) {
      errlogPrintf("Failed to acquire bus access and/or talk to slave.\n");
      close(I2Cdev);
      FASTUNLOCK(&I2C_lock_1);
      return(-1);
      }

   buf[0] = 0x10+channel;  //Command Access Byte

   bytesWritten = write(I2Cdev,buf,1);
   bytesRead = read(I2Cdev,buf,2);
   close(I2Cdev);
   FASTUNLOCK(&I2C_lock_1);
 if (devI2CDebug >= 20)  errlogPrintf("DAC Read : BytesRead:%d     Byte1Val: %x   Byte2Val: %x\n",bytesRead,buf[0],buf[1]);   
   *val = (unsigned long)(((buf[0] << 8) | buf[1]) >> 4);
   return (OK);
}

int ReadAdc(int card, int channel, int *val)
{
    char filename[20], buf[2];
    int cntrlword, bytesWritten, bytesRead;
    
        sprintf(filename,"/dev/i2c-1");
    FASTLOCK(&I2C_lock_1);
     if ((I2Cdev = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
	FASTUNLOCK(&I2C_lock_1);
        exit(1);
    }
    if (ioctl(I2Cdev,I2C_SLAVE,cards[card].base) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
	close(I2Cdev);
	FASTUNLOCK(&I2C_lock_1);
        exit(1);
    }
//   printf("Channel: %i\n",channel);
   
   switch (channel) {
      case 0 : cntrlword = 0x88; break;
      case 2 : cntrlword = 0x98; break;
      case 4 : cntrlword = 0xA8; break;
      case 6 : cntrlword = 0xB8; break;
      case 1 : cntrlword = 0xC8; break;
      case 3 : cntrlword = 0xD8; break;
      case 5 : cntrlword = 0xE8; break;
      case 7 : cntrlword = 0xF8; break;
   } 
   buf[0] = cntrlword; 
   bytesWritten = write(I2Cdev,buf,1);
    if(bytesWritten != 1){
      printf("ReadADC: cntrlWord not written: %i\n", bytesWritten);
      close(I2Cdev);
      FASTUNLOCK(&I2C_lock_1);
      }
   usleep(10000);
   buf[0]=0;
   buf[1]=0;
   bytesRead = read(I2Cdev,buf,2);
   if(devI2CDebug>=10){
   printf("ReadADC #%i: bytesRead = %i. Buf= %i:%i\n",channel,bytesRead, buf[1],buf[0]);
     }
   if(bytesRead !=2){
      printf("ReadADC: Bad # bytes received: %i\n", bytesRead);
      }
   close(I2Cdev);
   FASTUNLOCK(&I2C_lock_1);
   *val = (((buf[0] << 8) | buf[1]) >> 4);
   if(devI2CDebug>=10){
   printf("ReadADC #%i: val=%i\n",channel,*val);
     }
   return(0);
}


static long init_ao(void *precord)
{
aoRecord *pao = (aoRecord *)precord;
long status = 0L;
unsigned long rawVal = 0L;
int card, reg, args, numBits, twotype;
I2CDpvt  *pPvt;

switch (pao->out.type) {
	case (VME_IO) :
		card = pao->out.value.vmeio.card;
		reg = pao->out.value.vmeio.signal;
		if(card > MAX_NUM_CARDS){
		  pao->pact = 1;          /* make sure we don't process this thing */
                  errlogPrintf("devI2C: Card #%d exceeds max: ->%s<- \n",pao->out.value.vmeio.card, pao->name);
                  return(ERROR);
                  }
                if (reg >= cards[card].nReg) {
	         pao->pact = 1;          /* make sure we don't process this thing */
	         errlogPrintf("devI2C: Signal #%d exceeds registers: ->%s<-\n",reg,pao->name);
        return(ERROR);
      }
      args = sscanf(pao->out.value.vmeio.parm, "%d,%d", &numBits, &twotype);

      if( (args != 2) || (numBits <= 0) ||
         	(numBits > MAX_ACTIVE_BITS) ||
         	(twotype > 1) || (twotype < 0) ) {
        errlogPrintf("devI2C: Invalid Width/Type in parm field: ->%s<-\n",pao->name);
        return(ERROR);
      }

      cards[card].card_type=I2C_OUTPUT;
      
      pao->dpvt = (void *)calloc(1, sizeof(struct I2CDpvt));
      pPvt = (I2CDpvt *)pao->dpvt;

      pPvt->reg =  pao->out.value.vmeio.signal;
      pPvt->nbit = numBits;
      pPvt->type = twotype;
      pPvt->value = rawVal;
      if (devI2CDebug >= 20)
        errlogPrintf("init_ao 1:\n\tpPvt->reg %i\n\tpPvt->nbit %i\n\tpPvt->type %i\n\tpPvt->value %i \n\r",pPvt->reg,pPvt->nbit,pPvt->type,pPvt->value);


      pao->eslo = (pao->eguf - pao->egul)/(pow(2,numBits)-1);

/*  Shift Raw value if Bi-polar */
      if (pPvt->type == 1) 
         pao->roff = pow(2,(numBits-1));

      /* Init rval to current setting */ 
      if(ReadDac(card,reg,&rawVal) == OK) {
        pao->rbv = rawVal;
      if (devI2CDebug >= 20)
        errlogPrintf("init_ao 2: pao->rbv %i\n",pao->rbv);
	}
/* here is where we do the sign extensions for Bipolar.... */        
//        if (pPvt->type ==1) {
//           if (pao->rbv & (2<<(pPvt->nbit-2)))
//               pao->rbv |= ((2<<31) - (2<<(pPvt->nbit-2)))*2 ;
//	}

//        pao->rval = pao->rbv;

      if (devI2CDebug >= 20)
        errlogPrintf("init_ao 3: pao->rval %i\n",pao->rval);


/*		if (ioctl(I2Cdev,I2C_SLAVE,addr+card) < 0) {
		errlogPrintf("Failed to acquire bus access and/or talk to slave.\n");
		return(-1);
		} */
		EnableIntRef(card);      
		status = OK;
		break;
	default :
		recGblRecordError(S_db_badField, (void *)pao,"devAoI2C (init_record) Illegal OUT field");
		return(S_db_badField);
	}
/* Make sure record processing routine does not perform any conversion*/
//	pao->linr=3;
//FASTUNLOCK(&I2C_lock_1);
return(0);
}

static long init_ai(void *precord)
{
aiRecord *pai = (aiRecord *)precord;
long status;
int  card, reg, args, numBits, twotype;
I2CDpvt  *pPvt;

switch (pai->inp.type) {
	case (VME_IO) :
	   card = pai->inp.value.vmeio.card;
	   reg=pai->inp.value.vmeio.signal;
	   if(card > MAX_NUM_CARDS) {
	        pai->pact = 1;          /* make sure we don't process this thing */
	        errlogPrintf("devI2C: Card #%d exceeds max: ->%s<- \n",pai->inp.value.vmeio.card, pai->name);
	        return(ERROR);
	        }
           if(cards[card].base == NULL) {
	        pai->pact = 1;          /* make sure we don't process this thing */
	        errlogPrintf("devI2C: Card #%d not initialized: ->%s<-\n",pai->inp.value.vmeio.card,
                     pai->name);
	        return(ERROR);
	        }

           if (pai->inp.value.vmeio.signal >= cards[card].nReg) {
                pai->pact = 1;          /* make sure we don't process this thing */
                errlogPrintf("devI2C: Signal # exceeds registers: ->%s<-\n",pai->name);
                return(ERROR);
      	        }

           args = sscanf(pai->inp.value.vmeio.parm, "%d,%d", &numBits, &twotype);

           if( (args != 2) || (numBits <= 0) || (numBits > MAX_ACTIVE_BITS) ||
         	(twotype > 1) || (twotype < 0) ) {
                pai->pact = 1;          /* make sure we don't process this thing */
                errlogPrintf("devI2C: Invalid Width/Type in parm field: ->%s<-\n",pai->name);
                return(ERROR);
                }
      
            cards[card].card_type = I2C_INPUT;
      
            pai->dpvt = (void *)calloc(1, sizeof(struct I2CDpvt));
            pPvt = (I2CDpvt *)pai->dpvt;

            pPvt->reg =  reg;
            pPvt->nbit = numBits;
            pPvt->type = twotype;
            pai->eslo = (pai->eguf - pai->egul)/(pow(2,numBits)-1);
      
/*  Shift Raw value if Bi-polar */
           if (pPvt->type ==1) 
              pai->roff = pow(2,(numBits-1));

      status = OK;

/*           if (ioctl(I2Cdev,I2C_SLAVE,addr+card) < 0) {
              errlogPrintf("Failed to acquire bus access and/or talk to slave.\n");
 	      return(-1);
	      }  */
	   break;
	default :
		recGblRecordError(S_db_badField, (void *)pai,"devAiI2C (init_record) Illegal INP field");
	return(S_db_badField);
	}
/* Make sure record processing routine does not perform any conversion*/
//	pai->linr=0;
return(0);
}

static long write_ao(void *precord)
{
aoRecord *pao =(aoRecord *)precord;
  long status;
  unsigned long         rawVal=0;
  int                   card,reg;
  I2CDpvt             *pPvt = (I2CDpvt *)pao->dpvt;

  card = pao->out.value.vmeio.card;
  reg = pao->out.value.vmeio.signal;
  
  if (WriteDac(card,reg,pao->rval) == OK)
  {
    if(ReadDac(card,reg,&rawVal)== OK)
    {
      pao->rbv = rawVal;
      if (devI2CDebug >= 20)
          errlogPrintf("write_ao: card %i  reg %i rawVal %i\n\r", card, reg, rawVal);


/* here is where we do the sign extensions for Bipolar.... */        
        if (pPvt->type ==1) {
           if (pao->rbv & (2<<(pPvt->nbit-2)))
               pao->rbv |= ((2<<31) - (2<<(pPvt->nbit-2)))*2;

        }      
      return(0);
    }
  }
  /* Set an alarm for the record */
  recGblSetSevr(pao, WRITE_ALARM, INVALID_ALARM);
  return(2);
}

static long read_ai(void *precord)
{
  aiRecord* pai =(aiRecord *)precord;
  long status;
  unsigned long         val;
  int                   card,reg;
  I2CDpvt           *pPvt = (I2CDpvt *)pai->dpvt;

  card = pai->inp.value.vmeio.card;
  reg = pai->inp.value.vmeio.signal;

  if (ReadAdc(card,reg, &val) == OK)
  {
     pai->rval = val;

  if (devI2CDebug >= 10){
          errlogPrintf("read_ai: card %i  reg %i val %i\n\r", card, reg, val);
      }

/* here is where we do the sign extensions for Bipolar....  */       
        if (pPvt->type ==1) {
           if (pai->rval & (2<<(pPvt->nbit-2))) 
               pai->rval |= ((2<<31) - (2<<(pPvt->nbit-2)))*2; 

        }

     return(OK);
  }

  /* Set an alarm for the record if not OK */
  recGblSetSevr(pai, READ_ALARM, INVALID_ALARM);
  return(2);
}

#define DACTF 0.001220703   // 5v / 2^12 


