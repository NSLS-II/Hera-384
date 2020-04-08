/* zDDMRecord.h generated from zDDMRecord.dbd */

#ifndef INC_zDDMRecord_H
#define INC_zDDMRecord_H

#include "epicsTypes.h"
#include "link.h"
#include "epicsMutex.h"
#include "ellLib.h"
#include "epicsTime.h"

typedef enum {
    zddmscPOL_0                     /* Negative */,
    zddmscPOL_1                     /* Positive */
} zddmPOL;
#define zddmPOL_NUM_CHOICES 2

typedef enum {
    zddmscTPENB_0                   /* Off */,
    zddmscTPENB_1                   /* On */
} zddmTPENB;
#define zddmTPENB_NUM_CHOICES 2

typedef enum {
    leak_1                          /* Off */,
    leak_2                          /* 2pA */,
    leak_3                          /* 8pA */
} zddmLeak;
#define zddmLeak_NUM_CHOICES 3

typedef enum {
    gain_0                          /* 75keV */,
    gain_1                          /* 37.5keV */,
    gain_2                          /* 25keV */,
    gain_3                          /* 12.5keV */
} zddmGain;
#define zddmGain_NUM_CHOICES 4

typedef enum {
    scalerCONT_OneShot              /* OneShot */,
    scalerCONT_AutoCount            /* AutoCount */
} zddmCONT;
#define zddmCONT_NUM_CHOICES 2

typedef enum {
    zddmscPUEN_0                    /* Disable */,
    zddmscPUEN_1                    /* Enable */
} zddmPUEN;
#define zddmPUEN_NUM_CHOICES 2

typedef enum {
    shapeT_1                        /* 0.25us */,
    shapeT_2                        /* 0.5us */,
    shapeT_3                        /* 1us */,
    shapeT_4                        /* 2us */
} zddmshapeT;
#define zddmshapeT_NUM_CHOICES 4

typedef enum {
    zddmscTDM_0                     /* Time of arrival */,
    zddmscTDM_1                     /* Time over threshold */
} zddmTDM;
#define zddmTDM_NUM_CHOICES 2

typedef enum {
    zddmscMFS_0                     /* Off */,
    zddmscMFS_1                     /* 250ns */,
    zddmscMFS_2                     /* 500ns */,
    zddmscMFS_3                     /* 1us */,
    zddmscMFS_4                     /* 2us */
} zddmMFS;
#define zddmMFS_NUM_CHOICES 5

typedef enum {
    zddmGMON_0                      /* Off */,
    zddmGMON_1                      /* Temperature */,
    zddmGMON_2                      /* Baseline */,
    zddmGMON_3                      /* Threshold */,
    zddmGMON_4                      /* Test pulse */,
    zddmGMON_5                      /* Channel monitor */
} zddmGMON;
#define zddmGMON_NUM_CHOICES 6

typedef enum {
    scalerCNT_Done                  /* Done */,
    scalerCNT_Count                 /* Count */
} zddmCNT;
#define zddmCNT_NUM_CHOICES 2

typedef enum {
    zddmscTOS_0                     /* 1us */,
    zddmscTOS_1                     /* 2us */,
    zddmscTOS_2                     /* 3us */,
    zddmscTOS_3                     /* 4us */,
    zddmscTOS_4                     /* 6us */,
    zddmscTOS_5                     /* 9us */,
    zddmscTOS_6                     /* 12us */
} zddmTDS;
#define zddmTDS_NUM_CHOICES 7

typedef enum {
    zddmscMODE_0                    /* Framing */,
    zddmscMODE_1                    /* Continuous */
} zddmMODE;
#define zddmMODE_NUM_CHOICES 2

typedef enum {
    zddmscLOAO_0                    /* Pulse */,
    zddmscLOAO_1                    /* Leakage */
} zddmLOAO;
#define zddmLOAO_NUM_CHOICES 2

typedef struct zDDMRecord {
    char                name[61];   /* Record Name */
    char                desc[41];   /* Descriptor */
    char                asg[29];    /* Access Security Group */
    epicsEnum16         scan;       /* Scan Mechanism */
    epicsEnum16         pini;       /* Process at iocInit */
    epicsInt16          phas;       /* Scan Phase */
    char                evnt[40];   /* Event Name */
    epicsInt16          tse;        /* Time Stamp Event */
    DBLINK              tsel;       /* Time Stamp Link */
    epicsEnum16         dtyp;       /* Device Type */
    epicsInt16          disv;       /* Disable Value */
    epicsInt16          disa;       /* Disable */
    DBLINK              sdis;       /* Scanning Disable */
    epicsMutexId        mlok;       /* Monitor lock */
    ELLLIST             mlis;       /* Monitor List */
    epicsUInt8          disp;       /* Disable putField */
    epicsUInt8          proc;       /* Force Processing */
    epicsEnum16         stat;       /* Alarm Status */
    epicsEnum16         sevr;       /* Alarm Severity */
    epicsEnum16         nsta;       /* New Alarm Status */
    epicsEnum16         nsev;       /* New Alarm Severity */
    epicsEnum16         acks;       /* Alarm Ack Severity */
    epicsEnum16         ackt;       /* Alarm Ack Transient */
    epicsEnum16         diss;       /* Disable Alarm Sevrty */
    epicsUInt8          lcnt;       /* Lock Count */
    epicsUInt8          pact;       /* Record active */
    epicsUInt8          putf;       /* dbPutField process */
    epicsUInt8          rpro;       /* Reprocess  */
    struct asgMember    *asp;       /* Access Security Pvt */
    struct processNotify *ppn;      /* pprocessNotify */
    struct processNotifyRecord *ppnr; /* pprocessNotifyRecord */
    struct scan_element *spvt;      /* Scan Private */
    struct rset         *rset;      /* Address of RSET */
    struct dset         *dset;      /* DSET address */
    void                *dpvt;      /* Device Private */
    struct dbRecordType *rdes;      /* Address of dbRecordType */
    struct lockRecord   *lset;      /* Lock Set */
    epicsEnum16         prio;       /* Scheduling Priority */
    epicsUInt8          tpro;       /* Trace Processing */
    char                bkpt;       /* Break Point */
    epicsUInt8          udf;        /* Undefined */
    epicsEnum16         udfs;       /* Undefined Alarm Sevrty */
    epicsTimeStamp      time;       /* Time */
    DBLINK              flnk;       /* Forward Process Link */
    epicsFloat32        vers;       /* Code Version */
    epicsUInt32         val;        /* Value */
    void *           mca;           /* Value */
    void *           pmca;          /* Buffer Pointer */
    void *           tdc;           /* Value */
    void *           ptdc;          /* Buffer Pointer */
    void *           spct;          /* Value */
    void *           pspct;         /* Buffer Pointer */
    void *           spctx;         /* Value */
    void *           pspctx;        /* Buffer Pointer */
    epicsUInt32         exsize;     /* Display X size */
    epicsUInt32         eysize;     /* Display Y size */
    epicsUInt32         txsize;     /* Display X size */
    epicsUInt32         tysize;     /* Display Y size */
    char                fnam[64];   /* Filename */
    char                calf[64];   /* Calib. Filename */
    void *rpvt;                     /* Record Private */
    epicsFloat64        freq;       /* Time base freq */
    epicsEnum16         cnt;        /* Count */
    epicsEnum16         pol;        /* Polarity */
    epicsEnum16         pcnt;       /* Prev Count */
    DBLINK              out;        /* Output Specification */
    epicsFloat32        rate;       /* Display Rate (Hz.) */
    epicsFloat32        rat1;       /* Auto Display Rate (Hz.) */
    epicsFloat32        dly;        /* Delay */
    epicsFloat32        dly1;       /* Auto-mode Delay */
    epicsFloat64        tp;         /* Time Preset */
    epicsFloat64        tp1;        /* Auto Time Preset */
    epicsUInt32         pr1;        /* Preset */
    epicsInt16          ss;         /* Scaler state */
    epicsInt16          us;         /* User state */
    epicsEnum16         cont;       /* OneShot/AutoCount */
    epicsEnum16         mode;       /* Timed/Continuous mode */
    epicsUInt32         runno;      /* Run Number */
    epicsFloat64        t;          /* Timer */
    epicsUInt32         fver;       /* Firmware version */
    epicsUInt32         nelm;       /* Number of Elements */
    epicsUInt32         nch;        /* No. channels */
    epicsUInt32         chan;       /* Channel in chip */
    epicsUInt32         chip;       /* Selected chip */
    DBLINK              inp;        /* Input Specification */
    epicsInt16          card;       /* Card Number */
    epicsInt16          nchips;     /* Number of Chips */
    epicsEnum16         shpt;       /* Shaping time */
    epicsEnum16         gain;       /* Gain */
    epicsEnum16         gmon;       /* Monitor mode */
    epicsInt16          monch;      /* Monitor Channel */
    epicsEnum16         puen;       /* Pileup rejection enable */
    epicsEnum16         eblk;       /* Enable input bias current */
    void *           chen;          /* Value */
    void *           pchen;         /* Buffer Pointer */
    void *           slp;           /* Value */
    void *           pslp;          /* Buffer Pointer */
    void *           offs;          /* Value */
    void *           poffs;         /* Buffer Pointer */
    void *           tsen;          /* Value */
    void *           ptsen;         /* Buffer Pointer */
    void *           putr;          /* Value */
    void *           pputr;         /* Buffer Pointer */
    epicsEnum16         loao;       /* Value */
    char                egu[16];    /* Units Name */
    epicsInt16          prec;       /* Display Precision */
    epicsEnum16         mfs;        /* Comparator multi-fire suppr. */
    epicsEnum16         tds;        /* TDC slope */
    epicsEnum16         tdm;        /* TDC mode */
    void *           thtr;          /* Threshold trim */
    void *           pthtr;         /* Buffer Pointer */
    void *           thrsh;         /* Threshold */
    void *           pthrsh;        /* Threshold */
    epicsUInt32         tpamp;      /* Test pulse amplitude */
    epicsUInt32         tpfrq;      /* Test pulse frequency */
    epicsUInt32         tpcnt;      /* Number of est pulse */
    epicsEnum16         tpenb;      /* Start test pulses */
    DBLINK              cout;       /* CNT Output link */
    DBLINK              coutp;      /* CNT Output Prompt */
} zDDMRecord;

typedef enum {
	zDDMRecordNAME = 0,
	zDDMRecordDESC = 1,
	zDDMRecordASG = 2,
	zDDMRecordSCAN = 3,
	zDDMRecordPINI = 4,
	zDDMRecordPHAS = 5,
	zDDMRecordEVNT = 6,
	zDDMRecordTSE = 7,
	zDDMRecordTSEL = 8,
	zDDMRecordDTYP = 9,
	zDDMRecordDISV = 10,
	zDDMRecordDISA = 11,
	zDDMRecordSDIS = 12,
	zDDMRecordMLOK = 13,
	zDDMRecordMLIS = 14,
	zDDMRecordDISP = 15,
	zDDMRecordPROC = 16,
	zDDMRecordSTAT = 17,
	zDDMRecordSEVR = 18,
	zDDMRecordNSTA = 19,
	zDDMRecordNSEV = 20,
	zDDMRecordACKS = 21,
	zDDMRecordACKT = 22,
	zDDMRecordDISS = 23,
	zDDMRecordLCNT = 24,
	zDDMRecordPACT = 25,
	zDDMRecordPUTF = 26,
	zDDMRecordRPRO = 27,
	zDDMRecordASP = 28,
	zDDMRecordPPN = 29,
	zDDMRecordPPNR = 30,
	zDDMRecordSPVT = 31,
	zDDMRecordRSET = 32,
	zDDMRecordDSET = 33,
	zDDMRecordDPVT = 34,
	zDDMRecordRDES = 35,
	zDDMRecordLSET = 36,
	zDDMRecordPRIO = 37,
	zDDMRecordTPRO = 38,
	zDDMRecordBKPT = 39,
	zDDMRecordUDF = 40,
	zDDMRecordUDFS = 41,
	zDDMRecordTIME = 42,
	zDDMRecordFLNK = 43,
	zDDMRecordVERS = 44,
	zDDMRecordVAL = 45,
	zDDMRecordMCA = 46,
	zDDMRecordPMCA = 47,
	zDDMRecordTDC = 48,
	zDDMRecordPTDC = 49,
	zDDMRecordSPCT = 50,
	zDDMRecordPSPCT = 51,
	zDDMRecordSPCTX = 52,
	zDDMRecordPSPCTX = 53,
	zDDMRecordEXSIZE = 54,
	zDDMRecordEYSIZE = 55,
	zDDMRecordTXSIZE = 56,
	zDDMRecordTYSIZE = 57,
	zDDMRecordFNAM = 58,
	zDDMRecordCALF = 59,
	zDDMRecordRPVT = 60,
	zDDMRecordFREQ = 61,
	zDDMRecordCNT = 62,
	zDDMRecordPOL = 63,
	zDDMRecordPCNT = 64,
	zDDMRecordOUT = 65,
	zDDMRecordRATE = 66,
	zDDMRecordRAT1 = 67,
	zDDMRecordDLY = 68,
	zDDMRecordDLY1 = 69,
	zDDMRecordTP = 70,
	zDDMRecordTP1 = 71,
	zDDMRecordPR1 = 72,
	zDDMRecordSS = 73,
	zDDMRecordUS = 74,
	zDDMRecordCONT = 75,
	zDDMRecordMODE = 76,
	zDDMRecordRUNNO = 77,
	zDDMRecordT = 78,
	zDDMRecordFVER = 79,
	zDDMRecordNELM = 80,
	zDDMRecordNCH = 81,
	zDDMRecordCHAN = 82,
	zDDMRecordCHIP = 83,
	zDDMRecordINP = 84,
	zDDMRecordCARD = 85,
	zDDMRecordNCHIPS = 86,
	zDDMRecordSHPT = 87,
	zDDMRecordGAIN = 88,
	zDDMRecordGMON = 89,
	zDDMRecordMONCH = 90,
	zDDMRecordPUEN = 91,
	zDDMRecordEBLK = 92,
	zDDMRecordCHEN = 93,
	zDDMRecordPCHEN = 94,
	zDDMRecordSLP = 95,
	zDDMRecordPSLP = 96,
	zDDMRecordOFFS = 97,
	zDDMRecordPOFFS = 98,
	zDDMRecordTSEN = 99,
	zDDMRecordPTSEN = 100,
	zDDMRecordPUTR = 101,
	zDDMRecordPPUTR = 102,
	zDDMRecordLOAO = 103,
	zDDMRecordEGU = 104,
	zDDMRecordPREC = 105,
	zDDMRecordMFS = 106,
	zDDMRecordTDS = 107,
	zDDMRecordTDM = 108,
	zDDMRecordTHTR = 109,
	zDDMRecordPTHTR = 110,
	zDDMRecordTHRSH = 111,
	zDDMRecordPTHRSH = 112,
	zDDMRecordTPAMP = 113,
	zDDMRecordTPFRQ = 114,
	zDDMRecordTPCNT = 115,
	zDDMRecordTPENB = 116,
	zDDMRecordCOUT = 117,
	zDDMRecordCOUTP = 118
} zDDMFieldIndex;

#ifdef GEN_SIZE_OFFSET

#include <epicsAssert.h>
#include <epicsExport.h>
#ifdef __cplusplus
extern "C" {
#endif
static int zDDMRecordSizeOffset(dbRecordType *prt)
{
    zDDMRecord *prec = 0;

    assert(prt->no_fields == 119);
    prt->papFldDes[zDDMRecordNAME]->size = sizeof(prec->name);
    prt->papFldDes[zDDMRecordDESC]->size = sizeof(prec->desc);
    prt->papFldDes[zDDMRecordASG]->size = sizeof(prec->asg);
    prt->papFldDes[zDDMRecordSCAN]->size = sizeof(prec->scan);
    prt->papFldDes[zDDMRecordPINI]->size = sizeof(prec->pini);
    prt->papFldDes[zDDMRecordPHAS]->size = sizeof(prec->phas);
    prt->papFldDes[zDDMRecordEVNT]->size = sizeof(prec->evnt);
    prt->papFldDes[zDDMRecordTSE]->size = sizeof(prec->tse);
    prt->papFldDes[zDDMRecordTSEL]->size = sizeof(prec->tsel);
    prt->papFldDes[zDDMRecordDTYP]->size = sizeof(prec->dtyp);
    prt->papFldDes[zDDMRecordDISV]->size = sizeof(prec->disv);
    prt->papFldDes[zDDMRecordDISA]->size = sizeof(prec->disa);
    prt->papFldDes[zDDMRecordSDIS]->size = sizeof(prec->sdis);
    prt->papFldDes[zDDMRecordMLOK]->size = sizeof(prec->mlok);
    prt->papFldDes[zDDMRecordMLIS]->size = sizeof(prec->mlis);
    prt->papFldDes[zDDMRecordDISP]->size = sizeof(prec->disp);
    prt->papFldDes[zDDMRecordPROC]->size = sizeof(prec->proc);
    prt->papFldDes[zDDMRecordSTAT]->size = sizeof(prec->stat);
    prt->papFldDes[zDDMRecordSEVR]->size = sizeof(prec->sevr);
    prt->papFldDes[zDDMRecordNSTA]->size = sizeof(prec->nsta);
    prt->papFldDes[zDDMRecordNSEV]->size = sizeof(prec->nsev);
    prt->papFldDes[zDDMRecordACKS]->size = sizeof(prec->acks);
    prt->papFldDes[zDDMRecordACKT]->size = sizeof(prec->ackt);
    prt->papFldDes[zDDMRecordDISS]->size = sizeof(prec->diss);
    prt->papFldDes[zDDMRecordLCNT]->size = sizeof(prec->lcnt);
    prt->papFldDes[zDDMRecordPACT]->size = sizeof(prec->pact);
    prt->papFldDes[zDDMRecordPUTF]->size = sizeof(prec->putf);
    prt->papFldDes[zDDMRecordRPRO]->size = sizeof(prec->rpro);
    prt->papFldDes[zDDMRecordASP]->size = sizeof(prec->asp);
    prt->papFldDes[zDDMRecordPPN]->size = sizeof(prec->ppn);
    prt->papFldDes[zDDMRecordPPNR]->size = sizeof(prec->ppnr);
    prt->papFldDes[zDDMRecordSPVT]->size = sizeof(prec->spvt);
    prt->papFldDes[zDDMRecordRSET]->size = sizeof(prec->rset);
    prt->papFldDes[zDDMRecordDSET]->size = sizeof(prec->dset);
    prt->papFldDes[zDDMRecordDPVT]->size = sizeof(prec->dpvt);
    prt->papFldDes[zDDMRecordRDES]->size = sizeof(prec->rdes);
    prt->papFldDes[zDDMRecordLSET]->size = sizeof(prec->lset);
    prt->papFldDes[zDDMRecordPRIO]->size = sizeof(prec->prio);
    prt->papFldDes[zDDMRecordTPRO]->size = sizeof(prec->tpro);
    prt->papFldDes[zDDMRecordBKPT]->size = sizeof(prec->bkpt);
    prt->papFldDes[zDDMRecordUDF]->size = sizeof(prec->udf);
    prt->papFldDes[zDDMRecordUDFS]->size = sizeof(prec->udfs);
    prt->papFldDes[zDDMRecordTIME]->size = sizeof(prec->time);
    prt->papFldDes[zDDMRecordFLNK]->size = sizeof(prec->flnk);
    prt->papFldDes[zDDMRecordVERS]->size = sizeof(prec->vers);
    prt->papFldDes[zDDMRecordVAL]->size = sizeof(prec->val);
    prt->papFldDes[zDDMRecordMCA]->size = sizeof(prec->mca);
    prt->papFldDes[zDDMRecordPMCA]->size = sizeof(prec->pmca);
    prt->papFldDes[zDDMRecordTDC]->size = sizeof(prec->tdc);
    prt->papFldDes[zDDMRecordPTDC]->size = sizeof(prec->ptdc);
    prt->papFldDes[zDDMRecordSPCT]->size = sizeof(prec->spct);
    prt->papFldDes[zDDMRecordPSPCT]->size = sizeof(prec->pspct);
    prt->papFldDes[zDDMRecordSPCTX]->size = sizeof(prec->spctx);
    prt->papFldDes[zDDMRecordPSPCTX]->size = sizeof(prec->pspctx);
    prt->papFldDes[zDDMRecordEXSIZE]->size = sizeof(prec->exsize);
    prt->papFldDes[zDDMRecordEYSIZE]->size = sizeof(prec->eysize);
    prt->papFldDes[zDDMRecordTXSIZE]->size = sizeof(prec->txsize);
    prt->papFldDes[zDDMRecordTYSIZE]->size = sizeof(prec->tysize);
    prt->papFldDes[zDDMRecordFNAM]->size = sizeof(prec->fnam);
    prt->papFldDes[zDDMRecordCALF]->size = sizeof(prec->calf);
    prt->papFldDes[zDDMRecordRPVT]->size = sizeof(prec->rpvt);
    prt->papFldDes[zDDMRecordFREQ]->size = sizeof(prec->freq);
    prt->papFldDes[zDDMRecordCNT]->size = sizeof(prec->cnt);
    prt->papFldDes[zDDMRecordPOL]->size = sizeof(prec->pol);
    prt->papFldDes[zDDMRecordPCNT]->size = sizeof(prec->pcnt);
    prt->papFldDes[zDDMRecordOUT]->size = sizeof(prec->out);
    prt->papFldDes[zDDMRecordRATE]->size = sizeof(prec->rate);
    prt->papFldDes[zDDMRecordRAT1]->size = sizeof(prec->rat1);
    prt->papFldDes[zDDMRecordDLY]->size = sizeof(prec->dly);
    prt->papFldDes[zDDMRecordDLY1]->size = sizeof(prec->dly1);
    prt->papFldDes[zDDMRecordTP]->size = sizeof(prec->tp);
    prt->papFldDes[zDDMRecordTP1]->size = sizeof(prec->tp1);
    prt->papFldDes[zDDMRecordPR1]->size = sizeof(prec->pr1);
    prt->papFldDes[zDDMRecordSS]->size = sizeof(prec->ss);
    prt->papFldDes[zDDMRecordUS]->size = sizeof(prec->us);
    prt->papFldDes[zDDMRecordCONT]->size = sizeof(prec->cont);
    prt->papFldDes[zDDMRecordMODE]->size = sizeof(prec->mode);
    prt->papFldDes[zDDMRecordRUNNO]->size = sizeof(prec->runno);
    prt->papFldDes[zDDMRecordT]->size = sizeof(prec->t);
    prt->papFldDes[zDDMRecordFVER]->size = sizeof(prec->fver);
    prt->papFldDes[zDDMRecordNELM]->size = sizeof(prec->nelm);
    prt->papFldDes[zDDMRecordNCH]->size = sizeof(prec->nch);
    prt->papFldDes[zDDMRecordCHAN]->size = sizeof(prec->chan);
    prt->papFldDes[zDDMRecordCHIP]->size = sizeof(prec->chip);
    prt->papFldDes[zDDMRecordINP]->size = sizeof(prec->inp);
    prt->papFldDes[zDDMRecordCARD]->size = sizeof(prec->card);
    prt->papFldDes[zDDMRecordNCHIPS]->size = sizeof(prec->nchips);
    prt->papFldDes[zDDMRecordSHPT]->size = sizeof(prec->shpt);
    prt->papFldDes[zDDMRecordGAIN]->size = sizeof(prec->gain);
    prt->papFldDes[zDDMRecordGMON]->size = sizeof(prec->gmon);
    prt->papFldDes[zDDMRecordMONCH]->size = sizeof(prec->monch);
    prt->papFldDes[zDDMRecordPUEN]->size = sizeof(prec->puen);
    prt->papFldDes[zDDMRecordEBLK]->size = sizeof(prec->eblk);
    prt->papFldDes[zDDMRecordCHEN]->size = sizeof(prec->chen);
    prt->papFldDes[zDDMRecordPCHEN]->size = sizeof(prec->pchen);
    prt->papFldDes[zDDMRecordSLP]->size = sizeof(prec->slp);
    prt->papFldDes[zDDMRecordPSLP]->size = sizeof(prec->pslp);
    prt->papFldDes[zDDMRecordOFFS]->size = sizeof(prec->offs);
    prt->papFldDes[zDDMRecordPOFFS]->size = sizeof(prec->poffs);
    prt->papFldDes[zDDMRecordTSEN]->size = sizeof(prec->tsen);
    prt->papFldDes[zDDMRecordPTSEN]->size = sizeof(prec->ptsen);
    prt->papFldDes[zDDMRecordPUTR]->size = sizeof(prec->putr);
    prt->papFldDes[zDDMRecordPPUTR]->size = sizeof(prec->pputr);
    prt->papFldDes[zDDMRecordLOAO]->size = sizeof(prec->loao);
    prt->papFldDes[zDDMRecordEGU]->size = sizeof(prec->egu);
    prt->papFldDes[zDDMRecordPREC]->size = sizeof(prec->prec);
    prt->papFldDes[zDDMRecordMFS]->size = sizeof(prec->mfs);
    prt->papFldDes[zDDMRecordTDS]->size = sizeof(prec->tds);
    prt->papFldDes[zDDMRecordTDM]->size = sizeof(prec->tdm);
    prt->papFldDes[zDDMRecordTHTR]->size = sizeof(prec->thtr);
    prt->papFldDes[zDDMRecordPTHTR]->size = sizeof(prec->pthtr);
    prt->papFldDes[zDDMRecordTHRSH]->size = sizeof(prec->thrsh);
    prt->papFldDes[zDDMRecordPTHRSH]->size = sizeof(prec->pthrsh);
    prt->papFldDes[zDDMRecordTPAMP]->size = sizeof(prec->tpamp);
    prt->papFldDes[zDDMRecordTPFRQ]->size = sizeof(prec->tpfrq);
    prt->papFldDes[zDDMRecordTPCNT]->size = sizeof(prec->tpcnt);
    prt->papFldDes[zDDMRecordTPENB]->size = sizeof(prec->tpenb);
    prt->papFldDes[zDDMRecordCOUT]->size = sizeof(prec->cout);
    prt->papFldDes[zDDMRecordCOUTP]->size = sizeof(prec->coutp);
    prt->papFldDes[zDDMRecordNAME]->offset = (unsigned short)((char *)&prec->name - (char *)prec);
    prt->papFldDes[zDDMRecordDESC]->offset = (unsigned short)((char *)&prec->desc - (char *)prec);
    prt->papFldDes[zDDMRecordASG]->offset = (unsigned short)((char *)&prec->asg - (char *)prec);
    prt->papFldDes[zDDMRecordSCAN]->offset = (unsigned short)((char *)&prec->scan - (char *)prec);
    prt->papFldDes[zDDMRecordPINI]->offset = (unsigned short)((char *)&prec->pini - (char *)prec);
    prt->papFldDes[zDDMRecordPHAS]->offset = (unsigned short)((char *)&prec->phas - (char *)prec);
    prt->papFldDes[zDDMRecordEVNT]->offset = (unsigned short)((char *)&prec->evnt - (char *)prec);
    prt->papFldDes[zDDMRecordTSE]->offset = (unsigned short)((char *)&prec->tse - (char *)prec);
    prt->papFldDes[zDDMRecordTSEL]->offset = (unsigned short)((char *)&prec->tsel - (char *)prec);
    prt->papFldDes[zDDMRecordDTYP]->offset = (unsigned short)((char *)&prec->dtyp - (char *)prec);
    prt->papFldDes[zDDMRecordDISV]->offset = (unsigned short)((char *)&prec->disv - (char *)prec);
    prt->papFldDes[zDDMRecordDISA]->offset = (unsigned short)((char *)&prec->disa - (char *)prec);
    prt->papFldDes[zDDMRecordSDIS]->offset = (unsigned short)((char *)&prec->sdis - (char *)prec);
    prt->papFldDes[zDDMRecordMLOK]->offset = (unsigned short)((char *)&prec->mlok - (char *)prec);
    prt->papFldDes[zDDMRecordMLIS]->offset = (unsigned short)((char *)&prec->mlis - (char *)prec);
    prt->papFldDes[zDDMRecordDISP]->offset = (unsigned short)((char *)&prec->disp - (char *)prec);
    prt->papFldDes[zDDMRecordPROC]->offset = (unsigned short)((char *)&prec->proc - (char *)prec);
    prt->papFldDes[zDDMRecordSTAT]->offset = (unsigned short)((char *)&prec->stat - (char *)prec);
    prt->papFldDes[zDDMRecordSEVR]->offset = (unsigned short)((char *)&prec->sevr - (char *)prec);
    prt->papFldDes[zDDMRecordNSTA]->offset = (unsigned short)((char *)&prec->nsta - (char *)prec);
    prt->papFldDes[zDDMRecordNSEV]->offset = (unsigned short)((char *)&prec->nsev - (char *)prec);
    prt->papFldDes[zDDMRecordACKS]->offset = (unsigned short)((char *)&prec->acks - (char *)prec);
    prt->papFldDes[zDDMRecordACKT]->offset = (unsigned short)((char *)&prec->ackt - (char *)prec);
    prt->papFldDes[zDDMRecordDISS]->offset = (unsigned short)((char *)&prec->diss - (char *)prec);
    prt->papFldDes[zDDMRecordLCNT]->offset = (unsigned short)((char *)&prec->lcnt - (char *)prec);
    prt->papFldDes[zDDMRecordPACT]->offset = (unsigned short)((char *)&prec->pact - (char *)prec);
    prt->papFldDes[zDDMRecordPUTF]->offset = (unsigned short)((char *)&prec->putf - (char *)prec);
    prt->papFldDes[zDDMRecordRPRO]->offset = (unsigned short)((char *)&prec->rpro - (char *)prec);
    prt->papFldDes[zDDMRecordASP]->offset = (unsigned short)((char *)&prec->asp - (char *)prec);
    prt->papFldDes[zDDMRecordPPN]->offset = (unsigned short)((char *)&prec->ppn - (char *)prec);
    prt->papFldDes[zDDMRecordPPNR]->offset = (unsigned short)((char *)&prec->ppnr - (char *)prec);
    prt->papFldDes[zDDMRecordSPVT]->offset = (unsigned short)((char *)&prec->spvt - (char *)prec);
    prt->papFldDes[zDDMRecordRSET]->offset = (unsigned short)((char *)&prec->rset - (char *)prec);
    prt->papFldDes[zDDMRecordDSET]->offset = (unsigned short)((char *)&prec->dset - (char *)prec);
    prt->papFldDes[zDDMRecordDPVT]->offset = (unsigned short)((char *)&prec->dpvt - (char *)prec);
    prt->papFldDes[zDDMRecordRDES]->offset = (unsigned short)((char *)&prec->rdes - (char *)prec);
    prt->papFldDes[zDDMRecordLSET]->offset = (unsigned short)((char *)&prec->lset - (char *)prec);
    prt->papFldDes[zDDMRecordPRIO]->offset = (unsigned short)((char *)&prec->prio - (char *)prec);
    prt->papFldDes[zDDMRecordTPRO]->offset = (unsigned short)((char *)&prec->tpro - (char *)prec);
    prt->papFldDes[zDDMRecordBKPT]->offset = (unsigned short)((char *)&prec->bkpt - (char *)prec);
    prt->papFldDes[zDDMRecordUDF]->offset = (unsigned short)((char *)&prec->udf - (char *)prec);
    prt->papFldDes[zDDMRecordUDFS]->offset = (unsigned short)((char *)&prec->udfs - (char *)prec);
    prt->papFldDes[zDDMRecordTIME]->offset = (unsigned short)((char *)&prec->time - (char *)prec);
    prt->papFldDes[zDDMRecordFLNK]->offset = (unsigned short)((char *)&prec->flnk - (char *)prec);
    prt->papFldDes[zDDMRecordVERS]->offset = (unsigned short)((char *)&prec->vers - (char *)prec);
    prt->papFldDes[zDDMRecordVAL]->offset = (unsigned short)((char *)&prec->val - (char *)prec);
    prt->papFldDes[zDDMRecordMCA]->offset = (unsigned short)((char *)&prec->mca - (char *)prec);
    prt->papFldDes[zDDMRecordPMCA]->offset = (unsigned short)((char *)&prec->pmca - (char *)prec);
    prt->papFldDes[zDDMRecordTDC]->offset = (unsigned short)((char *)&prec->tdc - (char *)prec);
    prt->papFldDes[zDDMRecordPTDC]->offset = (unsigned short)((char *)&prec->ptdc - (char *)prec);
    prt->papFldDes[zDDMRecordSPCT]->offset = (unsigned short)((char *)&prec->spct - (char *)prec);
    prt->papFldDes[zDDMRecordPSPCT]->offset = (unsigned short)((char *)&prec->pspct - (char *)prec);
    prt->papFldDes[zDDMRecordSPCTX]->offset = (unsigned short)((char *)&prec->spctx - (char *)prec);
    prt->papFldDes[zDDMRecordPSPCTX]->offset = (unsigned short)((char *)&prec->pspctx - (char *)prec);
    prt->papFldDes[zDDMRecordEXSIZE]->offset = (unsigned short)((char *)&prec->exsize - (char *)prec);
    prt->papFldDes[zDDMRecordEYSIZE]->offset = (unsigned short)((char *)&prec->eysize - (char *)prec);
    prt->papFldDes[zDDMRecordTXSIZE]->offset = (unsigned short)((char *)&prec->txsize - (char *)prec);
    prt->papFldDes[zDDMRecordTYSIZE]->offset = (unsigned short)((char *)&prec->tysize - (char *)prec);
    prt->papFldDes[zDDMRecordFNAM]->offset = (unsigned short)((char *)&prec->fnam - (char *)prec);
    prt->papFldDes[zDDMRecordCALF]->offset = (unsigned short)((char *)&prec->calf - (char *)prec);
    prt->papFldDes[zDDMRecordRPVT]->offset = (unsigned short)((char *)&prec->rpvt - (char *)prec);
    prt->papFldDes[zDDMRecordFREQ]->offset = (unsigned short)((char *)&prec->freq - (char *)prec);
    prt->papFldDes[zDDMRecordCNT]->offset = (unsigned short)((char *)&prec->cnt - (char *)prec);
    prt->papFldDes[zDDMRecordPOL]->offset = (unsigned short)((char *)&prec->pol - (char *)prec);
    prt->papFldDes[zDDMRecordPCNT]->offset = (unsigned short)((char *)&prec->pcnt - (char *)prec);
    prt->papFldDes[zDDMRecordOUT]->offset = (unsigned short)((char *)&prec->out - (char *)prec);
    prt->papFldDes[zDDMRecordRATE]->offset = (unsigned short)((char *)&prec->rate - (char *)prec);
    prt->papFldDes[zDDMRecordRAT1]->offset = (unsigned short)((char *)&prec->rat1 - (char *)prec);
    prt->papFldDes[zDDMRecordDLY]->offset = (unsigned short)((char *)&prec->dly - (char *)prec);
    prt->papFldDes[zDDMRecordDLY1]->offset = (unsigned short)((char *)&prec->dly1 - (char *)prec);
    prt->papFldDes[zDDMRecordTP]->offset = (unsigned short)((char *)&prec->tp - (char *)prec);
    prt->papFldDes[zDDMRecordTP1]->offset = (unsigned short)((char *)&prec->tp1 - (char *)prec);
    prt->papFldDes[zDDMRecordPR1]->offset = (unsigned short)((char *)&prec->pr1 - (char *)prec);
    prt->papFldDes[zDDMRecordSS]->offset = (unsigned short)((char *)&prec->ss - (char *)prec);
    prt->papFldDes[zDDMRecordUS]->offset = (unsigned short)((char *)&prec->us - (char *)prec);
    prt->papFldDes[zDDMRecordCONT]->offset = (unsigned short)((char *)&prec->cont - (char *)prec);
    prt->papFldDes[zDDMRecordMODE]->offset = (unsigned short)((char *)&prec->mode - (char *)prec);
    prt->papFldDes[zDDMRecordRUNNO]->offset = (unsigned short)((char *)&prec->runno - (char *)prec);
    prt->papFldDes[zDDMRecordT]->offset = (unsigned short)((char *)&prec->t - (char *)prec);
    prt->papFldDes[zDDMRecordFVER]->offset = (unsigned short)((char *)&prec->fver - (char *)prec);
    prt->papFldDes[zDDMRecordNELM]->offset = (unsigned short)((char *)&prec->nelm - (char *)prec);
    prt->papFldDes[zDDMRecordNCH]->offset = (unsigned short)((char *)&prec->nch - (char *)prec);
    prt->papFldDes[zDDMRecordCHAN]->offset = (unsigned short)((char *)&prec->chan - (char *)prec);
    prt->papFldDes[zDDMRecordCHIP]->offset = (unsigned short)((char *)&prec->chip - (char *)prec);
    prt->papFldDes[zDDMRecordINP]->offset = (unsigned short)((char *)&prec->inp - (char *)prec);
    prt->papFldDes[zDDMRecordCARD]->offset = (unsigned short)((char *)&prec->card - (char *)prec);
    prt->papFldDes[zDDMRecordNCHIPS]->offset = (unsigned short)((char *)&prec->nchips - (char *)prec);
    prt->papFldDes[zDDMRecordSHPT]->offset = (unsigned short)((char *)&prec->shpt - (char *)prec);
    prt->papFldDes[zDDMRecordGAIN]->offset = (unsigned short)((char *)&prec->gain - (char *)prec);
    prt->papFldDes[zDDMRecordGMON]->offset = (unsigned short)((char *)&prec->gmon - (char *)prec);
    prt->papFldDes[zDDMRecordMONCH]->offset = (unsigned short)((char *)&prec->monch - (char *)prec);
    prt->papFldDes[zDDMRecordPUEN]->offset = (unsigned short)((char *)&prec->puen - (char *)prec);
    prt->papFldDes[zDDMRecordEBLK]->offset = (unsigned short)((char *)&prec->eblk - (char *)prec);
    prt->papFldDes[zDDMRecordCHEN]->offset = (unsigned short)((char *)&prec->chen - (char *)prec);
    prt->papFldDes[zDDMRecordPCHEN]->offset = (unsigned short)((char *)&prec->pchen - (char *)prec);
    prt->papFldDes[zDDMRecordSLP]->offset = (unsigned short)((char *)&prec->slp - (char *)prec);
    prt->papFldDes[zDDMRecordPSLP]->offset = (unsigned short)((char *)&prec->pslp - (char *)prec);
    prt->papFldDes[zDDMRecordOFFS]->offset = (unsigned short)((char *)&prec->offs - (char *)prec);
    prt->papFldDes[zDDMRecordPOFFS]->offset = (unsigned short)((char *)&prec->poffs - (char *)prec);
    prt->papFldDes[zDDMRecordTSEN]->offset = (unsigned short)((char *)&prec->tsen - (char *)prec);
    prt->papFldDes[zDDMRecordPTSEN]->offset = (unsigned short)((char *)&prec->ptsen - (char *)prec);
    prt->papFldDes[zDDMRecordPUTR]->offset = (unsigned short)((char *)&prec->putr - (char *)prec);
    prt->papFldDes[zDDMRecordPPUTR]->offset = (unsigned short)((char *)&prec->pputr - (char *)prec);
    prt->papFldDes[zDDMRecordLOAO]->offset = (unsigned short)((char *)&prec->loao - (char *)prec);
    prt->papFldDes[zDDMRecordEGU]->offset = (unsigned short)((char *)&prec->egu - (char *)prec);
    prt->papFldDes[zDDMRecordPREC]->offset = (unsigned short)((char *)&prec->prec - (char *)prec);
    prt->papFldDes[zDDMRecordMFS]->offset = (unsigned short)((char *)&prec->mfs - (char *)prec);
    prt->papFldDes[zDDMRecordTDS]->offset = (unsigned short)((char *)&prec->tds - (char *)prec);
    prt->papFldDes[zDDMRecordTDM]->offset = (unsigned short)((char *)&prec->tdm - (char *)prec);
    prt->papFldDes[zDDMRecordTHTR]->offset = (unsigned short)((char *)&prec->thtr - (char *)prec);
    prt->papFldDes[zDDMRecordPTHTR]->offset = (unsigned short)((char *)&prec->pthtr - (char *)prec);
    prt->papFldDes[zDDMRecordTHRSH]->offset = (unsigned short)((char *)&prec->thrsh - (char *)prec);
    prt->papFldDes[zDDMRecordPTHRSH]->offset = (unsigned short)((char *)&prec->pthrsh - (char *)prec);
    prt->papFldDes[zDDMRecordTPAMP]->offset = (unsigned short)((char *)&prec->tpamp - (char *)prec);
    prt->papFldDes[zDDMRecordTPFRQ]->offset = (unsigned short)((char *)&prec->tpfrq - (char *)prec);
    prt->papFldDes[zDDMRecordTPCNT]->offset = (unsigned short)((char *)&prec->tpcnt - (char *)prec);
    prt->papFldDes[zDDMRecordTPENB]->offset = (unsigned short)((char *)&prec->tpenb - (char *)prec);
    prt->papFldDes[zDDMRecordCOUT]->offset = (unsigned short)((char *)&prec->cout - (char *)prec);
    prt->papFldDes[zDDMRecordCOUTP]->offset = (unsigned short)((char *)&prec->coutp - (char *)prec);
    prt->rec_size = sizeof(*prec);
    return 0;
}
epicsExportRegistrar(zDDMRecordSizeOffset);

#ifdef __cplusplus
}
#endif
#endif /* GEN_SIZE_OFFSET */

#endif /* INC_zDDMRecord_H */
