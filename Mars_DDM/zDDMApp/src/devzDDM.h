/* ----------------Device Support Entry Table for devzDDM----------------- */

#define MAX_CHANNELS 384
#define MAX_NCHIPS 12

typedef struct{
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	reset;
	DEVSUPFUN	read;
	DEVSUPFUN	write_preset;
	DEVSUPFUN	arm;
	DEVSUPFUN	done;
} det_DSET;

/*** det_state ***/

typedef struct{
	int card_exists;
	int num_channels;
	int card_in_use;
	int count_in_progress; /* count in progress? */
	int newdat;
	int tp;
	zDDMRecord *psr;
	unsigned short ident; /* identification info for this card */
	IOSCANPVT ioscanpvt;
	int done; 	/* sequence counter for ISR: 
			0=first entry, set up one-shot for time
			1=timer one-shot done
			2=count sequence finished   */
	int newdata; /* flag to tell hardware new data is available */
	CALLBACK *pcallback;
}det_state;


/*** callback stuff ***/
struct callback {
	CALLBACK	callback;
	struct dbCommon *precord;
        epicsTimerId wd_id;
};

typedef struct zDDMDpvt { /* unique for each record */
  int  reg;   /* index of sub-device (determined by signal #*/
  int  nbit;  /* no of significant bits */
  int  type;  /* Type either 0 or 1 for uni, bi polar */
  unsigned long  value; /* to provide readback for ao records */
}zDDMDpvt;

typedef struct {
        int exists;
        int done;
        int newdat;
        zDDMRecord *psr;
        CALLBACK *pcallback;
} devPVT;

typedef struct rpvtStruct {
        epicsMutexId updateMutex;
        CALLBACK *pcallbacks;
} rpvtStruct;

typedef struct{
       char dp;		/* Pileup rejector trim dac */
//     char nc1;	/* no connection, set 0 */
       char da;		/* Threshold trim dac */
       char sel;	/* 1=leakage current, 0=shaper output */
//     char nc2;	/* no connection, set 0 */
       char sm;		/* 1=channel disable */
       char st;		/* 1=enable test input (30fF cap) */
}chanstr;


typedef struct{
      int pa;		/* Threshold dac */
      int pb;		/* Test pulse dac */
     char rm;		/* Readout mods; 1=synch, 0=asynch */
     char senfl1;	/* Lock on peak found */
     char senfl2;	/* Lock on threshold */
     char m0;		/* 1=channel mon, 0=others */
     char m1;		/* 1=pk det on PD/PN; 0=other mons on PD/PN */
     char sbn;		/* enable buffer on pdn & mon outputs */
     char sb;		/* enable buffer on pd & mon outputs */
     char sl;		/* 0=internal 2pA leakage, 1=disabled */
     char ts;		/* Shaping time */
     char rt;		/* 1=timing ramp duration x 3 */
     char spur;		/* 1=enable pileup rejector */
     char sse;		/* 1=enable multiple-firing suppression */
     char tr;		/* timing ramp adjust */
     char ss;		/* multiple firing time adjust */
     char c;		/* m0=0,Monitor select. m0=1, channel being monitored */
     char g;		/* Gain select */
     char slh;		/* internal leakage adjust */
     char sp;		/* Input polarity; 1=positive, 0=negative */
     char saux;		/* Enable monitor output */
     char sbm;		/* Enable output monitor buffer */
     char tm;		/* Timing mode; 0=ToA, 1=ToT */
}chipstr;

volatile chanstr channelstr[MAX_CHANNELS];
volatile chipstr globalstr[MAX_NCHIPS];


/*epicsMutexId  SPI_lock;  */     /* SPI is shared resource; lock accesses */

/*epicsMutexId  fpga_write_lock;  */     /* fpga registers are shared */
