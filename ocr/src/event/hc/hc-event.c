/*
 * This file is subject to the license agreement located in the file LICENSE
 * and cannot be distributed without it. This notice cannot be
 * removed or modified.
 */

#include "ocr-config.h"
#ifdef ENABLE_EVENT_HC

#include "ocr-hal.h"
#include "debug.h"
#include "event/hc/hc-event.h"
#include "ocr-datablock.h"
#include "ocr-edt.h"
#include "ocr-event.h"
#include "ocr-policy-domain.h"
#include "ocr-sysboot.h"
#include "ocr-task.h"
#include "utils/ocr-utils.h"
#include "ocr-worker.h"
#include "ocr-errors.h"
#include "extensions/ocr-hints.h"

#if defined (ENABLE_RESILIENCY) && defined (ENABLE_CHECKPOINT_VERIFICATION)
#include "policy-domain/hc/hc-policy.h"
#endif

#ifdef OCR_ENABLE_STATISTICS
#include "ocr-statistics.h"
#include "ocr-statistics-callbacks.h"
#endif

#define SEALED_LIST ((void *) -1)
#define END_OF_LIST NULL

#define DEBUG_TYPE EVENT

// Custom DEBUG_LVL for debugging
#define DBG_HCEVT_LOG   DEBUG_LVL_VERB
#define DBG_HCEVT_ERR   DEBUG_LVL_WARN
#define DBG_COL_EVT     DEBUG_LVL_VVERB

/******************************************************/
/* OCR-HC Debug                                       */
/******************************************************/

#if defined(OCR_DEBUG) && !defined(OCR_TRACE_BINARY)
static char * eventTypeToString(ocrEvent_t * base) {
    ocrEventTypes_t type = base->kind;
    if(type == OCR_EVENT_ONCE_T) {
        return "once";
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    } else if (type == OCR_EVENT_COUNTED_T) {
        return "counted";
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    } else if (type == OCR_EVENT_CHANNEL_T) {
        return "channel";
#endif
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
    } else if (type == OCR_EVENT_COLLECTIVE_T) {
        return "collective";
#endif
    } else if (type == OCR_EVENT_IDEM_T) {
        return "idem";
    } else if (type == OCR_EVENT_STICKY_T) {
        return "sticky";
    } else if (type == OCR_EVENT_LATCH_T) {
        return "latch";
    } else {
        return "unknown";
    }
}
#endif


/***********************************************************/
/* OCR-HC Event Hint Properties                             */
/* (Add implementation specific supported properties here) */
/***********************************************************/

u64 ocrHintPropEventHc[] = {
#ifdef ENABLE_HINTS
#endif
};

//Make sure OCR_HINT_COUNT_EVT_HC in hc-task.h is equal to the length of array ocrHintPropEventHc
ocrStaticAssert((sizeof(ocrHintPropEventHc)/sizeof(u64)) == OCR_HINT_COUNT_EVT_HC);
ocrStaticAssert(OCR_HINT_COUNT_EVT_HC < OCR_RUNTIME_HINT_PROP_BITS);


/******************************************************/
/* OCR-HC Distributed Events Implementation           */
/******************************************************/

//To forge a local copy without communication.
//This ability is implementation dependent.
#ifndef ENABLE_EVENT_MDC_FORGE
#define ENABLE_EVENT_MDC_FORGE 0
#endif

// Metadata synchronization operations
#define M_CLONE 0
#define M_REG 1
#define M_SAT 2
#define M_DEL 3

typedef struct {
    ocrLocation_t location;
    ocrGuid_t guid;
} locguid_payload;

typedef struct {
    ocrLocation_t location;
} loc_payload;

typedef struct {
    ocrGuid_t guid;
} guid_payload;

#define M_SAT_payload    locguid_payload
#define M_REG_payload    loc_payload
#define M_DEL_payload    loc_payload
#define M_CLONE_payload  guid_payload

#define GET_PAYLOAD_DATA(buffer, mode, type, name)       ((type)((mode##_payload *) buffer)->name)
#define SET_PAYLOAD_DATA(buffer, mode, type, name, val)  ((((mode##_payload *) buffer)->name) = (type) val)
#define WR_PAYLOAD_DATA(buffer, type, val)               (((type*)buffer)[0] = (type) val)

static void mdPushHcDist(ocrGuid_t evtGuid, ocrLocation_t loc, ocrGuid_t dbGuid, u32 mode, u32 factoryId);

/******************************************************/
/* OCR-HC Collective Events Additional types           */
/******************************************************/

#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT

// 'IN' size of PD_MSG_METADATA_COMM
#define MSG_MDCOMM_SZ       (_PD_MSG_SIZE_IN(PD_MSG_METADATA_COMM))

// Collide with M_REG and M_SAT but ok
#define M_UP   1
#define M_DOWN 2

// Mask of a field at the LSB
#define MASK_ZB(name)       ((((u64)1)<<(REDOP_##name##_SIZE))-1)
// Mask for a field at its rightful position in the REDOP
#define REDOP_MASK(name)     (MASK_ZB(name)<<(REDOP_EIDX(name)))
// Shift a value representing a field toward MSB at its rightful position in the REDOP
#define LSHIFT(name, val)   (((u64)(val)) << REDOP_EIDX(name))
// Shift a value representing a field from its position in the REDOP toward LSB and masks the result
#define RSHIFT(name, val)   (((val) >> REDOP_EIDX(name)) & (MASK_ZB(name)))
#define MAX_VAL(name)       (((u64)1) << REDOP_SIZE(name))

// Checks whether or not a value identified by name is set in a variable
#define REDOP_IS(NAME, VALUE, VAR) (((VAR) & REDOP_MASK(NAME)) == REDOP_##VALUE)
#define REDOP_GET(NAME, VAR)       (RSHIFT(NAME, (VAR)))
#define REDOP_EQ(CHECK, VAR) (CHECK == VAR)

// Apply a reduction operator
#define REDUCE_FCT_OP(DST, SRC, NB_DATUM, TYPE, OP) { \
    u32 i; \
    TYPE * tdst = (TYPE *) DST; \
    TYPE * tsrc = (TYPE *) SRC; \
    for(i=0;i<NB_DATUM;i++) { \
        *tdst++ OP##= *tsrc++;\
    } \
}

// Apply a min/max operator
#define REDUCE_FCT_MINMAX(DST, SRC, NB_DATUM, TYPE, OP) { \
    u32 i; \
    TYPE * tdst = (TYPE *) DST; \
    TYPE * tsrc = (TYPE *) SRC; \
    for(i=0;i<NB_DATUM;i++) { \
        if(*tdst++ OP *tsrc++) *(tdst-1) = *(tsrc-1); \
    } \
}

// Reduction functions
static void reduceDoubleAdd(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, double, +)
}

static void reduceDoubleMult(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, double, *)
}

static void reduceDoubleMin(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, double, <)
}

static void reduceDoubleMax(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, double, >)
}

static void reduceU64Add(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, u64, +)
}

static void reduceU64Mult(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, u64, *)
}

static void reduceU64Min(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, u64, <)
}

static void reduceU64Max(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, u64, >)
}

static void reduceU64BitAnd(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, u64, &)
}

static void reduceU64BitOr(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, u64, |)
}

static void reduceU64BitXor(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, u64, ^)
}

static void reduceS64Min(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, s64, <)
}

static void reduceS64Max(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, s64, >)
}

static void reduceFloatAdd(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, float, +)
}

static void reduceFloatMult(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, float, *)
}

static void reduceFloatMin(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, float, <)
}

static void reduceFloatMax(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, float, >)
}

static void reduceU32Add(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, u32, +)
}

static void reduceU32Mult(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, u32, *)
}

static void reduceU32Min(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, u32, <)
}

static void reduceU32Max(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, u32, >)
}

static void reduceS32Min(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, s32, <)
}

static void reduceS32Max(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_MINMAX(dst, src, nbDatum, s32, >)
}

static void reduceU32BitAnd(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, u32, &)
}

static void reduceU32BitOr(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, u32, |)
}

static void reduceU32BitXor(void * dst, void * src, u32 nbDatum) {
    REDUCE_FCT_OP(dst, src, nbDatum, u32, ^)
}

void setReductionFctPtr(ocrEventHcCollective_t * dself, redOp_t redOp) {
    // Mask associativity and commutativity flags
    redOp &= ~((REDOP_ASSOCIATIVE)|(REDOP_COMMUTATIVE));
    if (REDOP_EQ((REDOP_BS8 | REDOP_SIGNED | REDOP_REAL | REDOP_ADD), redOp)) {
        dself->reduce = reduceDoubleAdd; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_SIGNED | REDOP_REAL | REDOP_MULT), redOp)) {
        dself->reduce = reduceDoubleMult; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_SIGNED | REDOP_REAL | REDOP_MIN), redOp)) {
        dself->reduce = reduceDoubleMin; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_SIGNED | REDOP_REAL | REDOP_MAX), redOp)) {
        dself->reduce = reduceDoubleMax; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_ADD), redOp)) {
        dself->reduce = reduceU64Add; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_MULT), redOp)) {
        dself->reduce = reduceU64Mult; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_MIN), redOp)) {
        dself->reduce = reduceU64Min; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_MAX), redOp)) {
        dself->reduce = reduceU64Max; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_BITAND), redOp)) {
        dself->reduce = reduceU64BitAnd; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_BITOR), redOp)) {
        dself->reduce = reduceU64BitOr; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_BITXOR), redOp)) {
        dself->reduce = reduceU64BitXor; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_SIGNED | REDOP_INTEGER | REDOP_MIN), redOp)) {
        dself->reduce = reduceS64Min; return;
    } else if (REDOP_EQ((REDOP_BS8 | REDOP_SIGNED | REDOP_INTEGER | REDOP_MAX), redOp)) {
        dself->reduce = reduceS64Max; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_SIGNED | REDOP_REAL | REDOP_ADD), redOp)) {
        dself->reduce = reduceFloatAdd; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_SIGNED | REDOP_REAL | REDOP_MULT), redOp)) {
        dself->reduce = reduceFloatMult; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_SIGNED | REDOP_REAL | REDOP_MIN), redOp)) {
        dself->reduce = reduceFloatMin; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_SIGNED | REDOP_REAL | REDOP_MAX), redOp)) {
        dself->reduce = reduceFloatMax; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_ADD), redOp)) {
        dself->reduce = reduceU32Add; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_MULT), redOp)) {
        dself->reduce = reduceU32Mult; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_MIN), redOp)) {
        dself->reduce = reduceU32Min; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_MAX), redOp)) {
        dself->reduce = reduceU32Max; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_SIGNED | REDOP_INTEGER | REDOP_MIN), redOp)) {
        dself->reduce = reduceS32Min; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_SIGNED | REDOP_INTEGER | REDOP_MAX), redOp)) {
        dself->reduce = reduceS32Max; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_BITAND), redOp)) {
        dself->reduce = reduceU32BitAnd; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_BITOR), redOp)) {
        dself->reduce = reduceU32BitOr; return;
    } else if (REDOP_EQ((REDOP_BS4 | REDOP_UNSIGNED | REDOP_INTEGER | REDOP_BITXOR), redOp)) {
        dself->reduce = reduceU32BitXor; return;
    }
    ocrAssert(false && "Unresolved reduction function pointer");
}

typedef u16 rph_t;

typedef struct _contributor_t {
    rph_t iph;
    rph_t oph;
    regNode_t * deps; // linearized over maxGen
    void * contribs; // linearized over nbDatum * maxGen
} contributor_t;

#endif /*ENABLE_EXTENSION_COLLECTIVE_EVT*/

/******************************************************/
/* OCR-HC Events Implementation                       */
/******************************************************/
#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
#define ARG_SSLOT u32 sslot,
#else
#define ARG_SSLOT
#endif
#define FSIG_UNREGISTERSIGNALER struct _ocrEvent_t *self, ARG_SSLOT ocrFatGuid_t signaler, u32 slot, bool isDepRem

static u8 createDbRegNode(ocrFatGuid_t * dbFatGuid, u32 nbElems, bool doRelease, regNode_t ** node) {
    ocrPolicyDomain_t *pd = NULL;
    PD_MSG_STACK(msg);
    ocrTask_t *curTask = NULL;
    u32 i;
    getCurrentEnv(&pd, NULL, &curTask, &msg);
    ocrFatGuid_t curEdt = {.guid = curTask!=NULL?curTask->guid:NULL_GUID, .metaDataPtr = curTask};
#define PD_MSG (&msg)
#define PD_TYPE PD_MSG_DB_CREATE
    msg.type = PD_MSG_DB_CREATE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
    PD_MSG_FIELD_IO(guid) = (*dbFatGuid);
    PD_MSG_FIELD_IO(size) = sizeof(regNode_t)*nbElems;
    PD_MSG_FIELD_IO(properties) = DB_PROP_RT_ACQUIRE;
    PD_MSG_FIELD_I(edt) = curEdt;
    PD_MSG_FIELD_I(hint) = NULL_HINT;
    PD_MSG_FIELD_I(dbType) = RUNTIME_DBTYPE;
    PD_MSG_FIELD_I(allocator) = NO_ALLOC;
    RESULT_PROPAGATE(pd->fcts.processMessage(pd, &msg, true));
    (*dbFatGuid) = PD_MSG_FIELD_IO(guid);
    regNode_t * temp = (regNode_t*) PD_MSG_FIELD_O(ptr);
    *node = temp;
    for(i = 0; i < nbElems; ++i) {
        temp[i].guid = UNINITIALIZED_GUID;
        temp[i].slot = 0;
        temp[i].mode = -1;
    }
#undef PD_TYPE
#define PD_TYPE PD_MSG_DB_RELEASE
    if (doRelease) {
        getCurrentEnv(NULL, NULL, NULL, &msg);
        msg.type = PD_MSG_DB_RELEASE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
        PD_MSG_FIELD_IO(guid) = (*dbFatGuid);
        PD_MSG_FIELD_I(srcLoc) = pd->myLocation;
        PD_MSG_FIELD_I(edt) = curEdt;
        PD_MSG_FIELD_I(ptr) = NULL;
        PD_MSG_FIELD_I(size) = 0;
        PD_MSG_FIELD_I(properties) = DB_PROP_RT_ACQUIRE;
        RESULT_PROPAGATE(pd->fcts.processMessage(pd, &msg, true));
        *node = NULL;
    }
#undef PD_MSG
#undef PD_TYPE
    return 0;
}

//
// OCR-HC Single Events Implementation
//

static void destructEventHcPeers(ocrEvent_t *base, locNode_t * curHead);

u8 destructEventHc(ocrEvent_t *base) {
    ocrEventHc_t *event = (ocrEventHc_t*)base;
    ocrPolicyDomain_t *pd = NULL;
    PD_MSG_STACK(msg);
    ocrTask_t *curTask = NULL;
    getCurrentEnv(&pd, NULL, &curTask, &msg);

    DPRINTF(DEBUG_LVL_INFO, "Destroy %s: "GUIDF"\n", eventTypeToString(base), GUIDA(base->guid));
    OCR_TOOL_TRACE(false, OCR_TRACE_TYPE_EVENT, OCR_ACTION_DESTROY, traceEventDestroy, base->guid);

#ifdef OCR_ENABLE_STATISTICS
    statsEVT_DESTROY(pd, getCurrentEDT(), NULL, base->guid, base);
#endif

    // Destroy datablocks linked with this event
    if (!(ocrGuidIsUninitialized(event->waitersDb.guid))) {
#define PD_MSG (&msg)
#define PD_TYPE PD_MSG_DB_FREE
        msg.type = PD_MSG_DB_FREE | PD_MSG_REQUEST;
        PD_MSG_FIELD_I(guid) = event->waitersDb;
        PD_MSG_FIELD_I(edt.guid) = curTask ? curTask->guid : NULL_GUID;
        PD_MSG_FIELD_I(edt.metaDataPtr) = curTask;
        PD_MSG_FIELD_I(srcLoc) = pd->myLocation;
        PD_MSG_FIELD_I(properties) = DB_PROP_RT_ACQUIRE | DB_PROP_NO_RELEASE;
        RESULT_PROPAGATE(pd->fcts.processMessage(pd, &msg, false));
#undef PD_MSG
#undef PD_TYPE
    }

    // Now destroy the GUID
#define PD_MSG (&msg)
#define PD_TYPE PD_MSG_GUID_DESTROY
    getCurrentEnv(NULL, NULL, NULL, &msg);
    msg.type = PD_MSG_GUID_DESTROY | PD_MSG_REQUEST;
    // These next two statements may be not required. Just to be safe.
    PD_MSG_FIELD_I(guid.guid) = base->guid;
    PD_MSG_FIELD_I(guid.metaDataPtr) = base;
    PD_MSG_FIELD_I(properties) = 1; // Free metadata
    RESULT_PROPAGATE(pd->fcts.processMessage(pd, &msg, false));
#undef PD_MSG
#undef PD_TYPE
    return 0;
}

#define STATE_CHECKED_IN ((u32)-1)
#define STATE_CHECKED_OUT ((u32)-2)
#define STATE_DESTROY_SEEN ((u32)-3)

// For Sticky and Idempotent
u8 destructEventHcPersist(ocrEvent_t *base) {
    ocrEventHc_t *event = (ocrEventHc_t*) base;
    // Addresses a race when the EDT that's satisfying the
    // event is still using the event's metadata but the children
    // EDT is has already invoked the destruct function.
    // BUG #537 could potentially improve on that by creating a lightweight
    // asynchronous operation to reschedule the destruction instead
    // of competing.
    u32 wc = event->waitersCount;
    // - Can be STATE_CHECKED_IN: competing for destruction with satisfy
    // - Can be STATE_CHECKED_OUT: We should win this competition
    // - Can be any other value: We are deleting the event before it is satisfied.
    //   It's either a race in the user program or an early destruction of the event.
    //
    // By contract the satisfy code should directly call destructEventHc
    // if it wins the right to invoke the destruction code
    ocrAssert(wc != STATE_DESTROY_SEEN);
    if (wc == STATE_CHECKED_IN) {
        // Competing with the satisfy waiters code
        u32 oldV = hal_cmpswap32(&(event->waitersCount), wc, STATE_DESTROY_SEEN);
        if (wc == oldV) {
            // Successfully CAS from STATE_CHECKED_IN => STATE_DESTROY_SEEN
            // i.e. we lost competition: Satisfier will destroy the event
            // Return code zero as the event is 'scheduled' for deletion,
            // it just won't happen through this path.
            return 0;
        } else { // CAS failed
            // Was competing with the CAS in satisfy which only
            // does one thing: STATE_CHECKED_IN => STATE_CHECKED_OUT
            ocrAssert(event->waitersCount == STATE_CHECKED_OUT);
            // fall-through and destroy the event
        }
    }
    //BUG #989: MT opportunity
    destructEventHcPeers(base, event->mdClass.peers);
    return destructEventHc(base);
}

#ifdef ENABLE_EXTENSION_CHANNEL_EVT
static u32 channelSatisfyCount(ocrEventHcChannel_t * devt) {
    return (devt->tailSat - devt->headSat);
}

static u32 channelWaiterCount(ocrEventHcChannel_t * devt) {
    return (devt->tailWaiter - devt->headWaiter);
}
#endif

ocrFatGuid_t getEventHc(ocrEvent_t *base) {
    ocrFatGuid_t res = {.guid = NULL_GUID, .metaDataPtr = NULL};
    switch(base->kind) {
    case OCR_EVENT_ONCE_T:
    case OCR_EVENT_LATCH_T:
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
    case OCR_EVENT_COLLECTIVE_T:
        break;
#endif
    case OCR_EVENT_STICKY_T:
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    case OCR_EVENT_COUNTED_T:
#endif
    {
        ocrEventHcPersist_t *event = (ocrEventHcPersist_t*)base;
        res.guid = event->data;
        break;
    }
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    case OCR_EVENT_CHANNEL_T:
    {
        // Warning: Not thread safe and only used for mpi-lite legacy support to
        // let the caller know if the channel got enough deps/sat to trigger.
        // If so, it returns the first guid in the satisfy list without consuming it.
        ocrEventHcChannel_t *devt = (ocrEventHcChannel_t*)base;
        u32 satCount = channelSatisfyCount(devt);
        u32 waitCount = channelWaiterCount(devt);
        if ((satCount >= devt->nbSat) && (waitCount >= devt->nbDeps)) {
            res.guid = devt->satBuffer[devt->headSat];
        } else {
            res.guid = UNINITIALIZED_GUID;
        }
        break;
    }
#endif
    default:
        ocrAssert(0);
    }
    return res;
}

static u8 commonSatisfyRegNode(ocrPolicyDomain_t * pd, ocrPolicyMsg_t * msg,
                         ocrGuid_t evtGuid,
                         ocrFatGuid_t db, ocrFatGuid_t currentEdt,
                         regNode_t * node) {
#ifdef OCR_ENABLE_STATISTICS
    //TODO the null should be the base but it's a race
    statsDEP_SATISFYFromEvt(pd, evtGuid, NULL, node->guid,
                            db.guid, node->slot);
#endif
    ocrAssert(!ocrGuidIsUninitialized(node->guid));
    DPRINTF(DEBUG_LVL_INFO, "SatisfyFromEvent: src: "GUIDF" dst: "GUIDF" slot:%"PRIu32" \n", GUIDA(evtGuid), GUIDA(node->guid), node->slot);
#define PD_MSG (msg)
#define PD_TYPE PD_MSG_DEP_SATISFY
    getCurrentEnv(NULL, NULL, NULL, msg);
    msg->type = PD_MSG_DEP_SATISFY | PD_MSG_REQUEST;
    // Need to refill because out may overwrite some of the in fields
    PD_MSG_FIELD_I(satisfierGuid.guid) = evtGuid;
    // Passing NULL since base may become invalid
    PD_MSG_FIELD_I(satisfierGuid.metaDataPtr) = NULL;
    PD_MSG_FIELD_I(guid.guid) = node->guid;
    PD_MSG_FIELD_I(guid.metaDataPtr) = NULL;
    PD_MSG_FIELD_I(payload) = db;
    PD_MSG_FIELD_I(currentEdt) = currentEdt;
    PD_MSG_FIELD_I(slot) = node->slot;
#ifdef REG_ASYNC_SGL
    PD_MSG_FIELD_I(mode) = node->mode;
#endif
    PD_MSG_FIELD_I(properties) = 0;
    RESULT_PROPAGATE(pd->fcts.processMessage(pd, msg, false));
#undef PD_MSG
#undef PD_TYPE

    return 0;
}

#ifdef ALLOW_EAGER_DB
static u8 commonSatisfyRegNodeEager(ocrPolicyDomain_t * pd, ocrPolicyMsg_t * msg,
                         ocrGuid_t evtGuid,
                         ocrFatGuid_t db, ocrFatGuid_t currentEdt,
                         regNode_t * node) {
    if (!ocrGuidIsNull(db.guid)) {
        u64 val = 0;
        RESULT_ASSERT(pd->guidProviders[0]->fcts.getVal(pd->guidProviders[0], db.guid, &val, NULL, MD_LOCAL, NULL), ==, 0);
        if (val != 0) { // Only check if local
            ocrLocation_t dstLocation;
            pd->guidProviders[0]->fcts.getLocation(pd->guidProviders[0], node->guid, &dstLocation);
            if (dstLocation != pd->myLocation) {
                // Check if the db is eager
                ocrDataBlock_t * dbSelf = (ocrDataBlock_t *) val;
                ocrHint_t hint;
                RESULT_ASSERT(ocrHintInit(&hint, OCR_HINT_DB_T), ==, 0);
                RESULT_ASSERT(((ocrDataBlockFactory_t *)pd->factories[dbSelf->fctId])->fcts.getHint(dbSelf, &hint), ==, 0);
                u64 hintValue = 0ULL;
                if ((ocrGetHintValue(&hint, OCR_HINT_DB_EAGER, &hintValue) == 0) && (hintValue != 0)) {
                    DPRINTF(DEBUG_LVL_VVERB, "Eager: DETECTED Eager hint "GUIDF"\n", GUIDA(db.guid));
                    //TODO-DB-EAGER: this is just handling a single node
                    ocrPolicyMsg_t * msgClone;
                    PD_MSG_STACK(msgStack);
                    u64 msgSize = (_PD_MSG_SIZE_IN(PD_MSG_GUID_METADATA_CLONE)) + sizeof(u32) + sizeof(regNode_t) - sizeof(char*);
                    ocrAssert(dstLocation != pd->myLocation); // Per the getVal above
                    if (msgSize > sizeof(ocrPolicyMsg_t)) {
                        //TODO-MD-SLAB
                        msgClone = (ocrPolicyMsg_t *) pd->fcts.pdMalloc(pd, msgSize);
                        initializePolicyMessage(msgClone, msgSize);
                    } else {
                        msgClone = &msgStack;
                    }
                    getCurrentEnv(NULL, NULL, NULL, msgClone);
    #define PD_MSG (msgClone)
    #define PD_TYPE PD_MSG_GUID_METADATA_CLONE
                    msgClone->type = PD_MSG_GUID_METADATA_CLONE | PD_MSG_REQUEST;
                    PD_MSG_FIELD_IO(guid) = db;
                    PD_MSG_FIELD_I(type) = MD_CLONE | MD_NON_COHERENT;
                    PD_MSG_FIELD_I(dstLocation) = dstLocation;
                    //TODO-EAGER Need to support multiple dependences to be satisfied
                    char *  writePtr = (char *) &PD_MSG_FIELD_I(addPayload);
                    ((u32*)writePtr)[0] = ((u32)1);
                    writePtr+=sizeof(u32);
                    ((regNode_t *)writePtr)[0] = *node;
                    RESULT_PROPAGATE(pd->fcts.processMessage(pd, msgClone, false));
    #undef PD_MSG
    #undef PD_TYPE
                    return 0;
                }
            }
        }
    }
    return commonSatisfyRegNode(pd, msg, evtGuid, db, currentEdt, node);
}
#endif

static u8 commonSatisfyWaiters(ocrPolicyDomain_t *pd, ocrEvent_t *base, ocrFatGuid_t db, u32 waitersCount,
                                ocrFatGuid_t currentEdt, ocrPolicyMsg_t * msg,
                                bool isPersistentEvent) {
    ocrEventHc_t * event = (ocrEventHc_t *) base;
    // waitersDb is safe to read because non-persistent event forbids further
    // registrations and persistent event registration is closed because of
    // event->waitersCount set to STATE_CHECKED_IN.
    ocrFatGuid_t dbWaiters = event->waitersDb;
    u32 i;
#if HCEVT_WAITER_STATIC_COUNT
    u32 ub = ((waitersCount < HCEVT_WAITER_STATIC_COUNT) ? waitersCount : HCEVT_WAITER_STATIC_COUNT);
    // Do static waiters first
    for(i = 0; i < ub; ++i) {
        RESULT_PROPAGATE(commonSatisfyRegNode(pd, msg, base->guid, db, currentEdt, &event->waiters[i]));
    }
    waitersCount -= ub;
#endif

    if(waitersCount > 0) {
        ocrAssert(!(ocrGuidIsUninitialized(dbWaiters.guid)));
        // First acquire the DB that contains the waiters
#define PD_MSG (msg)
#define PD_TYPE PD_MSG_DB_ACQUIRE
        getCurrentEnv(NULL, NULL, NULL, msg);
        msg->type = PD_MSG_DB_ACQUIRE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
        PD_MSG_FIELD_IO(guid) = dbWaiters;
        PD_MSG_FIELD_IO(edt) = currentEdt;
        PD_MSG_FIELD_IO(destLoc) = pd->myLocation;
        PD_MSG_FIELD_IO(edtSlot) = EDT_SLOT_NONE;
        if (isPersistentEvent) {
            // !! Warning !! RW here (and not RO) works in pair with the lock
            // being unlocked before DB_RELEASE is called in 'registerWaiterEventHcPersist'
            PD_MSG_FIELD_IO(properties) = DB_MODE_RW | DB_PROP_RT_ACQUIRE;
        } else {
            PD_MSG_FIELD_IO(properties) = DB_MODE_CONST | DB_PROP_RT_ACQUIRE;
        }
        u8 res __attribute__((unused)) = pd->fcts.processMessage(pd, msg, true);
        ocrAssert(!res);
        regNode_t * waiters = (regNode_t*)PD_MSG_FIELD_O(ptr);
        //BUG #273: we should not get an updated deguidification...
        dbWaiters = PD_MSG_FIELD_IO(guid); //Get updated deguidification if needed
        ocrAssert(waiters);
#undef PD_TYPE

        // Second, call satisfy on all the waiters
        for(i = 0; i < waitersCount; ++i) {
            RESULT_PROPAGATE(commonSatisfyRegNode(pd, msg, base->guid, db, currentEdt, &waiters[i]));
        }

        // Release the DB
#define PD_TYPE PD_MSG_DB_RELEASE
        getCurrentEnv(NULL, NULL, NULL, msg);
        msg->type = PD_MSG_DB_RELEASE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
        PD_MSG_FIELD_IO(guid) = dbWaiters;
        PD_MSG_FIELD_I(edt) = currentEdt;
        PD_MSG_FIELD_I(srcLoc) = pd->myLocation;
        PD_MSG_FIELD_I(ptr) = NULL;
        PD_MSG_FIELD_I(size) = 0;
        PD_MSG_FIELD_I(properties) = DB_PROP_RT_ACQUIRE;
        RESULT_PROPAGATE(pd->fcts.processMessage(pd, msg, true));
#undef PD_MSG
#undef PD_TYPE
    }

    return 0;
}

// For once events, we don't have to worry about
// concurrent registerWaiter calls (this would be a programmer error)
u8 satisfyEventHcOnce(ocrEvent_t *base, ocrFatGuid_t db, u32 slot) {
    ocrEventHc_t *event = (ocrEventHc_t*)base;
    ocrAssert(slot == 0); // For non-latch events, only one slot

    DPRINTF(DEBUG_LVL_INFO, "Satisfy: "GUIDF" with "GUIDF"\n", GUIDA(base->guid), GUIDA(db.guid));

    ocrPolicyDomain_t *pd = NULL;
    ocrTask_t *curTask = NULL;
    PD_MSG_STACK(msg);
    getCurrentEnv(&pd, NULL, &curTask, &msg);
    ocrFatGuid_t currentEdt;
    currentEdt.guid = (curTask == NULL) ? NULL_GUID : curTask->guid;
    currentEdt.metaDataPtr = curTask;
    u32 waitersCount = event->waitersCount;
    // This is only to help users find out about wrongful use of events
    event->waitersCount = STATE_CHECKED_IN; // Indicate that the event is satisfied

#ifdef OCR_ENABLE_STATISTICS
    statsDEP_SATISFYToEvt(pd, currentEdt.guid, NULL, base->guid, base, data, slot);
#endif

    if (waitersCount) {
        RESULT_PROPAGATE(commonSatisfyWaiters(pd, base, db, waitersCount, currentEdt, &msg, false));
    }
#ifdef NANNYMODE_ONCE_EVT
    else {
        DPRINTF(DEBUG_LVL_WARN, "Once event "GUIDF" satisfied with no dependences\n", GUIDA(base->guid));
    }
#endif
    // Since this a ONCE event, we need to destroy it as well
    // This is safe to do so at this point as all the messages have been sent
    return destructEventHc(base);
}

static u8 commonSatisfyEventHcPersist(ocrEvent_t *base, ocrFatGuid_t db, u32 slot, u32 waitersCount) {
    ocrAssert(slot == 0); // Persistent-events are single slot
    DPRINTF(DEBUG_LVL_INFO, "Satisfy %s: "GUIDF" with "GUIDF"\n", eventTypeToString(base),
            GUIDA(base->guid), GUIDA(db.guid));

#ifdef OCR_ENABLE_STATISTICS
    ocrPolicyDomain_t *pd = getCurrentPD();
    ocrGuid_t edt = getCurrentEDT();
    statsDEP_SATISFYToEvt(pd, edt, NULL, base->guid, base, data, slot);
#endif
    // Process waiters to be satisfied
    if(waitersCount) {
        ocrPolicyDomain_t *pd = NULL;
        ocrTask_t *curTask = NULL;
        PD_MSG_STACK(msg);
        getCurrentEnv(&pd, NULL, &curTask, &msg);
        ocrFatGuid_t currentEdt;
        currentEdt.guid = (curTask == NULL) ? NULL_GUID : curTask->guid;
        currentEdt.metaDataPtr = curTask;
        RESULT_PROPAGATE(commonSatisfyWaiters(pd, base, db, waitersCount, currentEdt, &msg, true));
    }
    u32 oldV = hal_cmpswap32(&(((ocrEventHc_t*)base)->waitersCount), STATE_CHECKED_IN, STATE_CHECKED_OUT);
    if (oldV == STATE_DESTROY_SEEN) {
        // CAS has failed because of a concurrent destroy operation, which means that we
        // won the right to destroy the event. i.e. we are logically checked out and the
        // destroy code marked the event for deletion but couldn't destroy because we were
        // checked in.
        destructEventHc(base);
    }
    // else we just checked out and there may be a concurrent deletion happening
    // => do not touch the event pointer anymore
    return 0;
}

// Notify peers we got a satisfy notification.
static void satisfyEventHcPeers(ocrEvent_t *base, ocrGuid_t dbGuid, u32 slot, locNode_t * curHead) {
    ocrLocation_t fromLoc = ((ocrEventHc_t *) base)->mdClass.satFromLoc;
    // We may get concurrent registrations but that's ok as they'll get added before curHead.
    while (curHead != NULL) {
        // NOTE-1: There's an ordering constraint between the M_SAT here and a concurrent
        // destruct operation that's on hold (because of ->waitersCount != -2).
        // => The M_DEL message that's triggered MUST be processed after the M_SAT at destination.
        // TODO: I think the M_DEL/M_SAT issue is currently a live bug because we do not have a way
        // of ordering messages processing at destination. Although all M_SAT are sent, the M_DEL may
        // outrun it.
        // NOTE-2: do not send a M_SAT to the emitter of the satisfy.
        if (curHead->loc != fromLoc) {
            mdPushHcDist(base->guid, curHead->loc, dbGuid, M_SAT, base->fctId);
        }
        curHead = curHead->next;
    }
}

// Notify peers we got a satisfy notification.
static void destructEventHcPeers(ocrEvent_t *base, locNode_t * curHead) {
    ocrLocation_t fromLoc = ((ocrEventHc_t *) base)->mdClass.delFromLoc;
    ocrPolicyDomain_t * pd = NULL;
    getCurrentEnv(&pd, NULL, NULL, NULL);
    while (curHead != NULL) {
        if (curHead->loc != fromLoc) {
            mdPushHcDist(base->guid, curHead->loc, NULL_GUID , M_DEL, base->fctId);
        }
        locNode_t * oldHead = curHead;
        curHead = curHead->next;
        pd->fcts.pdFree(pd, oldHead);
    }
}

#ifdef ENABLE_EXTENSION_COUNTED_EVT
// For counted event
u8 satisfyEventHcCounted(ocrEvent_t *base, ocrFatGuid_t db, u32 slot) {
    ocrEventHc_t * event = (ocrEventHc_t*) base;
    bool destroy = false;
    hal_lock(&(event->waitersLock));
    //BUG #809 Nanny-mode
    if ((event->waitersCount == STATE_CHECKED_IN) ||
        (event->waitersCount == STATE_CHECKED_OUT)) {
        DPRINTF(DBG_HCEVT_ERR, "User-level error detected: try to satisfy a counted event that's already satisfied: "GUIDF"\n", GUIDA(base->guid));
        ocrAssert(false);
        hal_unlock(&(event->waitersLock));
        return 1; //BUG #603 error codes: Put some error code here.
    }
    ((ocrEventHcPersist_t*)event)->data = db.guid;
    u32 waitersCount = event->waitersCount;
    event->waitersCount = STATE_CHECKED_IN; // Indicate the event is satisfied
    ocrEventHcCounted_t * devt = (ocrEventHcCounted_t *) event;
    ASSERT_BLOCK_BEGIN(waitersCount <= devt->nbDeps)
    DPRINTF(DBG_HCEVT_ERR, "User-level error detected: too many registrations on counted-event "GUIDF"\n", GUIDA(base->guid));
    ASSERT_BLOCK_END
    // Do not substract to nbDeps else concurrent registerWaiter
    // could reach zero and destruct the event while this code
    // unlocks and try to satisfy already registered waiters
    hal_unlock(&(event->waitersLock));
    // Nobody can be added to the event now...
    u8 ret = 0;
    if (waitersCount != 0) {
        ret = commonSatisfyEventHcPersist(base, db, slot, waitersCount);
        // ... and we're competing with registerWaiter for the
        //     event destruction through the lock.
        hal_lock(&(event->waitersLock));
        devt->nbDeps -= waitersCount;
        destroy = (devt->nbDeps == 0);
        hal_unlock(&(event->waitersLock));
        if (destroy) {
            ret = destructEventHc(base);
        }
    }
    return ret;
}
#endif


static u32 setSatisfiedEventHcPersist(ocrEvent_t *base, ocrFatGuid_t db, locNode_t ** curHead, bool checkError) {
    ocrEventHc_t * devt = (ocrEventHc_t*) base;
    hal_lock(&(devt->waitersLock));
    if ((devt->waitersCount == STATE_CHECKED_IN) ||
        (devt->waitersCount == STATE_CHECKED_OUT)) {
        if (checkError) {
            // Sticky needs to check for error, idem just ignores by definition.
            //BUG #809 Nanny-mode
            DPRINTF(DBG_HCEVT_ERR, "User-level error detected: try to satisfy a sticky event that's already satisfied: "GUIDF"\n", GUIDA(base->guid));
            ocrAssert(false);
        }
        hal_unlock(&(devt->waitersLock));
        return STATE_CHECKED_IN;
    }
    ((ocrEventHcPersist_t*)devt)->data = db.guid;
    u32 waitersCount = devt->waitersCount;
    devt->waitersCount = STATE_CHECKED_IN; // Indicate the event is satisfied
    //RACE-1: Get the current head for the peer list. Note that once we release the lock
    // there may be new registrations on the peer list. It's ok though, they will be
    // getting the GUID the event is satisfied with as part of the serialization protocol.
    *curHead = devt->mdClass.peers;
    //Note that we do not close registrations here because we still want to record
    //subsequent peers for destruction purpose
    hal_unlock(&(devt->waitersLock));
    return waitersCount;
}

// For idempotent events, accessed through the fct pointers interface
u8 satisfyEventHcPersistIdem(ocrEvent_t *base, ocrFatGuid_t db, u32 slot) {
    // Register the satisfy
    locNode_t * curHead;
    u32 waitersCount = setSatisfiedEventHcPersist(base, db, &curHead, /*checkError*/ false);
    if (waitersCount != STATE_CHECKED_IN) {
        //BUG #989: MT opportunity with two following calls micro-tasks and
        //          have a 'join' event to set the destruction flag.
        // Notify peers
        satisfyEventHcPeers(base, db.guid, slot, curHead);
        // Notify waiters
        commonSatisfyEventHcPersist(base, db, slot, waitersCount);
    }
    return 0;
}

// For sticky events, accessed through the fct pointers interface
u8 satisfyEventHcPersistSticky(ocrEvent_t *base, ocrFatGuid_t db, u32 slot) {
    // Register the satisfy
    locNode_t * curHead;
    u32 waitersCount = setSatisfiedEventHcPersist(base, db, &curHead, /*checkError*/ true);
    ocrAssert(waitersCount != STATE_CHECKED_IN); // i.e. no two satisfy on stickies
    //BUG #989: MT opportunity with two following calls micro-tasks and
    //          have a 'join' event to set the destruction flag.
    // Notify peers
    satisfyEventHcPeers(base, db.guid, slot, curHead);
    // Notify waiters
    commonSatisfyEventHcPersist(base, db, slot, waitersCount);
    return 0;
}

// This is for latch events
u8 satisfyEventHcLatch(ocrEvent_t *base, ocrFatGuid_t db, u32 slot) {
    ocrEventHcLatch_t *event = (ocrEventHcLatch_t*)base;
    ocrAssert(slot == OCR_EVENT_LATCH_DECR_SLOT ||
           slot == OCR_EVENT_LATCH_INCR_SLOT);

    s32 incr = (slot == OCR_EVENT_LATCH_DECR_SLOT)?-1:1;
    s32 count;
    do {
        count = event->counter;
        // FIXME: the (u32 *) cast on the line below is because event->counter is an (s32 *)
    } while(hal_cmpswap32((u32 *)&(event->counter), count, count+incr) != count);

    DPRINTF(DEBUG_LVL_INFO, "Satisfy %s: "GUIDF" %s\n", eventTypeToString(base),
            GUIDA(base->guid), ((slot == OCR_EVENT_LATCH_DECR_SLOT) ? "decr":"incr"));

    ocrPolicyDomain_t *pd = NULL;
    ocrTask_t *curTask = NULL;
    PD_MSG_STACK(msg);
    getCurrentEnv(&pd, NULL, &curTask, &msg);
    ocrFatGuid_t currentEdt;
    currentEdt.guid = (curTask == NULL) ? NULL_GUID : curTask->guid;
    currentEdt.metaDataPtr = curTask;

#ifdef OCR_ENABLE_STATISTICS
    statsDEP_SATISFYToEvt(pd, currentEdt.guid, NULL, base->guid, base, data, slot);
#endif
    if(count + incr != 0) {
        return 0;
    }
    // Here the event is satisfied
    DPRINTF(DEBUG_LVL_INFO, "Satisfy %s: "GUIDF" reached zero\n", eventTypeToString(base), GUIDA(base->guid));
    u32 waitersCount = event->base.waitersCount;
    // This is only to help users find out about wrongful use of events
    event->base.waitersCount = STATE_CHECKED_IN; // Indicate that the event is satisfied

    if (waitersCount) {
        RESULT_PROPAGATE(commonSatisfyWaiters(pd, base, db, waitersCount, currentEdt, &msg, false));
    }

    // The latch is satisfied so we destroy it
    return destructEventHc(base);
}

#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
u8 registerSignalerHc(ocrEvent_t *self, u32 sslot, ocrFatGuid_t signaler, u32 slot, ocrDbAccessMode_t mode, bool isDepAdd) {
#else
u8 registerSignalerHc(ocrEvent_t *self, ocrFatGuid_t signaler, u32 slot, ocrDbAccessMode_t mode, bool isDepAdd) {
#endif
    return 0; // We do not do anything for signalers
}


u8 unregisterSignalerHc(FSIG_UNREGISTERSIGNALER) {
    return 0; // We do not do anything for signalers
}

#ifdef REG_ASYNC_SGL
static u8 commonEnqueueWaiter(ocrPolicyDomain_t *pd, ocrEvent_t *base, ocrFatGuid_t waiter,
                              u32 slot, ocrDbAccessMode_t mode, ocrFatGuid_t currentEdt, ocrPolicyMsg_t * msg) {
#else
static u8 commonEnqueueWaiter(ocrPolicyDomain_t *pd, ocrEvent_t *base, ocrFatGuid_t waiter,
                              u32 slot, ocrFatGuid_t currentEdt, ocrPolicyMsg_t * msg) {
#endif
    // Warn: Caller must have acquired the waitersLock
    ocrEventHc_t *event = (ocrEventHc_t*)base;
    u32 waitersCount = event->waitersCount;

#if HCEVT_WAITER_STATIC_COUNT
    if (waitersCount < HCEVT_WAITER_STATIC_COUNT) {
        event->waiters[waitersCount].guid = waiter.guid;
        event->waiters[waitersCount].slot = slot;
#ifdef REG_ASYNC_SGL
        event->waiters[waitersCount].mode = mode;
#endif
        ++event->waitersCount;
        // We can release the lock now
        hal_unlock(&(event->waitersLock));
    } else {
#endif
        ocrFatGuid_t oldDbGuid = {.guid = NULL_GUID, .metaDataPtr = NULL};
        ocrFatGuid_t dbGuid = {.guid = NULL_GUID, .metaDataPtr = NULL};
        regNode_t *waiters = NULL;
        regNode_t *waitersNew = NULL;
        u8 toReturn = 0;
        // We're working with the dynamically created waiter list
        if (waitersCount == HCEVT_WAITER_STATIC_COUNT) {
            // Initial setup
            u8 toReturn = createDbRegNode(&(event->waitersDb), HCEVT_WAITER_DYNAMIC_COUNT, false, &waiters);
            if (toReturn) {
                ocrAssert(false && "Failed allocating db waiter");
                hal_unlock(&(event->waitersLock));
                return toReturn;
            }
            dbGuid = event->waitersDb; // for release
            event->waitersMax += HCEVT_WAITER_DYNAMIC_COUNT;
            waitersCount=0;
        } else {
            // Acquire the DB that contains the waiters
#define PD_MSG (msg)
#define PD_TYPE PD_MSG_DB_ACQUIRE
            getCurrentEnv(NULL, NULL, NULL, msg);
            msg->type = PD_MSG_DB_ACQUIRE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
            PD_MSG_FIELD_IO(guid) = event->waitersDb;
            PD_MSG_FIELD_IO(edt) = currentEdt;
            PD_MSG_FIELD_IO(destLoc) = pd->myLocation;
            PD_MSG_FIELD_IO(edtSlot) = EDT_SLOT_NONE;
            PD_MSG_FIELD_IO(properties) = DB_MODE_RW | DB_PROP_RT_ACQUIRE;
            //Should be a local DB
            if((toReturn = pd->fcts.processMessage(pd, msg, true))) {
                // should be the only writer active on the waiter DB since we have the lock
                ocrAssert(false); // debug
                ocrAssert(toReturn != OCR_EBUSY);
                hal_unlock(&(event->waitersLock));
                return toReturn; //BUG #603 error codes
            }
            waiters = (regNode_t*)PD_MSG_FIELD_O(ptr);
            //BUG #273
            event->waitersDb = PD_MSG_FIELD_IO(guid);
            ocrAssert(waiters);
#undef PD_TYPE
            if(waitersCount + 1 == event->waitersMax) {
                // We need to create a new DB and copy things over
#define PD_TYPE PD_MSG_DB_CREATE
                getCurrentEnv(NULL, NULL, NULL, msg);
                msg->type = PD_MSG_DB_CREATE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
                PD_MSG_FIELD_IO(guid) = dbGuid;
                PD_MSG_FIELD_IO(properties) = DB_PROP_RT_ACQUIRE;
                PD_MSG_FIELD_IO(size) = sizeof(regNode_t)*event->waitersMax*2;
                PD_MSG_FIELD_I(edt) = currentEdt;
                PD_MSG_FIELD_I(hint) = NULL_HINT;
                PD_MSG_FIELD_I(dbType) = RUNTIME_DBTYPE;
                PD_MSG_FIELD_I(allocator) = NO_ALLOC;
                if((toReturn = pd->fcts.processMessage(pd, msg, true))) {
                    ocrAssert(false); // debug
                    hal_unlock(&(event->waitersLock));
                    return toReturn; //BUG #603 error codes
                }
                waitersNew = (regNode_t*)PD_MSG_FIELD_O(ptr);
                oldDbGuid = event->waitersDb;
                dbGuid = PD_MSG_FIELD_IO(guid);
                event->waitersDb = dbGuid;
#undef PD_TYPE
                // -HCEVT_WAITER_STATIC_COUNT because part of the waiters are in the statically allocated waiter array
                u32 nbNodes = waitersCount-HCEVT_WAITER_STATIC_COUNT;
                hal_memCopy(waitersNew, waiters, sizeof(regNode_t)*(nbNodes), false);
                event->waitersMax *= 2;
                u32 i;
                u32 maxNbNodes = event->waitersMax-HCEVT_WAITER_STATIC_COUNT;
                for(i = nbNodes; i < maxNbNodes; ++i) {
                    waitersNew[i].guid = NULL_GUID;
                    waitersNew[i].slot = 0;
                    waitersNew[i].mode = -1;
                }
                waiters = waitersNew;
            } else {
                dbGuid = event->waitersDb; // for release
            }
            waitersCount=event->waitersCount-HCEVT_WAITER_STATIC_COUNT;
        }

        waiters[waitersCount].guid = waiter.guid;
        waiters[waitersCount].slot = slot;
#ifdef REG_ASYNC_SGL
        waiters[waitersCount].mode = mode;
#endif
        ++event->waitersCount;

        // We can release the lock now
        hal_unlock(&(event->waitersLock));

        // Release the waiter datablock / free old waiter DB when necessary
        //
        // In both cases it is important to release the GUID read from the cached
        // DB value and not from the event data-structure since we're operating
        // outside the lock there can be a new db created and assigned before getting here
#define PD_MSG (msg)
#define PD_TYPE PD_MSG_DB_RELEASE
        getCurrentEnv(NULL, NULL, NULL, msg);
        msg->type = PD_MSG_DB_RELEASE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
        PD_MSG_FIELD_IO(guid) = dbGuid;
        PD_MSG_FIELD_I(edt) = currentEdt;
        PD_MSG_FIELD_I(srcLoc) = pd->myLocation;
        PD_MSG_FIELD_I(ptr) = NULL;
        PD_MSG_FIELD_I(size) = 0;
        PD_MSG_FIELD_I(properties) = DB_PROP_RT_ACQUIRE;
        RESULT_PROPAGATE(pd->fcts.processMessage(pd, msg, true));
#undef PD_MSG
#undef PD_TYPE

        if(waitersNew) {
            // We need to free the old DB (implicitely released as DB_PROP_NO_RELEASE
            // is not provided here) and release the new one.
#define PD_MSG (msg)
#define PD_TYPE PD_MSG_DB_FREE
            getCurrentEnv(NULL, NULL, NULL, msg);
            msg->type = PD_MSG_DB_FREE | PD_MSG_REQUEST;
            PD_MSG_FIELD_I(guid) = oldDbGuid;
            PD_MSG_FIELD_I(edt) = currentEdt;
            PD_MSG_FIELD_I(srcLoc) = pd->myLocation;
            PD_MSG_FIELD_I(properties) = DB_PROP_RT_ACQUIRE;
            if((toReturn = pd->fcts.processMessage(pd, msg, false))) {
                ocrAssert(false); // debug
                return toReturn; //BUG #603 error codes
            }
#undef PD_MSG
#undef PD_TYPE
        }
#if HCEVT_WAITER_STATIC_COUNT
    }
#endif
    return 0; //Require registerSignaler invocation
}



/**
 * In this call, we do not contend with the satisfy (once and latch events) however,
 * we do contend with multiple registration.
 * By construction, users must ensure a ONCE event is registered before satisfy is called.
 */
#ifdef REG_ASYNC_SGL
#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
u8 registerWaiterEventHc(ocrEvent_t *base, u32 sslot, ocrFatGuid_t waiter, u32 slot, bool isDepAdd, ocrDbAccessMode_t mode) {
#else
u8 registerWaiterEventHc(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot, bool isDepAdd, ocrDbAccessMode_t mode) {
#endif
#else
u8 registerWaiterEventHc(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot, bool isDepAdd) {
#endif
    // Here we always add the waiter to our list so we ignore isDepAdd
    ocrEventHc_t *event = (ocrEventHc_t*)base;
    DPRINTF(DEBUG_LVL_INFO, "Register waiter %s: "GUIDF" with waiter "GUIDF" on slot %"PRId32"\n",
            eventTypeToString(base), GUIDA(base->guid), GUIDA(waiter.guid), slot);

    ocrPolicyDomain_t *pd = NULL;
    ocrTask_t *curTask = NULL;
    PD_MSG_STACK(msg);
    getCurrentEnv(&pd, NULL, &curTask, &msg);
    //BUG #809 this should be part of the n
    if (event->waitersCount == STATE_CHECKED_IN) {
         // This is best effort race check
         DPRINTF(DBG_HCEVT_ERR, "User-level error detected: adding dependence to a non-persistent event that's already satisfied: "GUIDF"\n", GUIDA(base->guid));
         ocrAssert(false);
         return 1; //BUG #603 error codes: Put some error code here.
    }
    ocrFatGuid_t currentEdt = {.guid = curTask!=NULL?curTask->guid:NULL_GUID, .metaDataPtr = curTask};
    hal_lock(&(event->waitersLock)); // Lock is released by commonEnqueueWaiter
#ifdef REG_ASYNC_SGL
    return commonEnqueueWaiter(pd, base, waiter, slot, mode, currentEdt, &msg);
#else
    return commonEnqueueWaiter(pd, base, waiter, slot, currentEdt, &msg);
#endif
}


/**
 * @brief Registers waiters on persistent events such as sticky or idempotent.
 *
 * This code contends with a satisfy call and with concurrent add-dependences that try
 * to register their waiter.
 * The event waiterLock is grabbed, if the event is already satisfied, directly satisfy
 * the waiter. Otherwise add the waiter's guid to the waiters db list. If db is too small
 * reallocate and copy over to a new one.
 *
 * Returns non-zero if the registerWaiter requires registerSignaler to be called there-after
 */
#ifdef REG_ASYNC_SGL
#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
u8 registerWaiterEventHcPersist(ocrEvent_t *base, u32 sslot, ocrFatGuid_t waiter, u32 slot, bool isDepAdd, ocrDbAccessMode_t mode) {
#else
u8 registerWaiterEventHcPersist(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot, bool isDepAdd, ocrDbAccessMode_t mode) {
#endif
#else
u8 registerWaiterEventHcPersist(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot, bool isDepAdd) {
#endif
    ocrEventHcPersist_t *event = (ocrEventHcPersist_t*)base;

    ocrPolicyDomain_t *pd = NULL;
    ocrTask_t *curTask = NULL;
    PD_MSG_STACK(msg);
    getCurrentEnv(&pd, NULL, &curTask, &msg);
    ocrFatGuid_t currentEdt;
    currentEdt.guid = (curTask == NULL) ? NULL_GUID : curTask->guid;
    currentEdt.metaDataPtr = curTask;

    // EDTs incrementally register on their dependences as elements
    // get satisfied (Guarantees O(n) traversal of dependence list).
    // Other events register when the dependence is added
    ocrGuidKind waiterKind = OCR_GUID_NONE;
    RESULT_ASSERT(guidKind(pd, waiter, &waiterKind), ==, 0);

#ifndef REG_ASYNC
#ifndef REG_ASYNC_SGL
    if(isDepAdd && waiterKind == OCR_GUID_EDT) {
        ocrAssert(false && "Should never happen anymore");
        // If we're adding a dependence and the waiter is an EDT we
        // skip this part. The event is registered on the EDT and
        // the EDT will register on the event only when its dependence
        // frontier reaches this event.
        return 0; //Require registerSignaler invocation
    }
#endif
#endif
    ocrAssert(waiterKind == OCR_GUID_EDT || (waiterKind & OCR_GUID_EVENT));

    DPRINTF(DEBUG_LVL_INFO, "Register waiter %s: "GUIDF" with waiter "GUIDF" on slot %"PRId32"\n",
            eventTypeToString(base), GUIDA(base->guid), GUIDA(waiter.guid), slot);
    // Lock to read the event->data
    hal_lock(&(event->base.waitersLock));
    if (!(ocrGuidIsUninitialized(event->data))) {
        ocrFatGuid_t dataGuid = {.guid = event->data, .metaDataPtr = NULL};
        hal_unlock(&(event->base.waitersLock));

#ifdef REG_ASYNC_SGL
        regNode_t node = {.guid = waiter.guid, .slot = slot, .mode = mode};
#else
        regNode_t node = {.guid = waiter.guid, .slot = slot};
#endif
        // We send a message saying that we satisfy whatever tried to wait on us
        return commonSatisfyRegNode(pd, &msg, base->guid, dataGuid, currentEdt, &node);
    }

    // Lock is released by commonEnqueueWaiter
#ifdef REG_ASYNC_SGL
    return commonEnqueueWaiter(pd, base, waiter, slot, mode, currentEdt, &msg);
#else
    return commonEnqueueWaiter(pd, base, waiter, slot, currentEdt, &msg);
#endif
}

/**
 * @brief Registers waiters on a counted-event.
 *
 * Pretty much the same code as for persistent events, see comments there
 *
 * Returns non-zero if the registerWaiter requires registerSignaler to be called there-after
 */
#ifdef ENABLE_EXTENSION_COUNTED_EVT
#ifdef REG_ASYNC_SGL
#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
u8 registerWaiterEventHcCounted(ocrEvent_t *base, u32 sslot, ocrFatGuid_t waiter, u32 slot, bool isDepAdd, ocrDbAccessMode_t mode) {
#else
u8 registerWaiterEventHcCounted(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot, bool isDepAdd, ocrDbAccessMode_t mode) {
#endif
#else
u8 registerWaiterEventHcCounted(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot, bool isDepAdd) {
#endif
    ocrEventHcPersist_t *event = (ocrEventHcPersist_t*)base;

    ocrPolicyDomain_t *pd = NULL;
    ocrTask_t *curTask = NULL;
    PD_MSG_STACK(msg);
    getCurrentEnv(&pd, NULL, &curTask, &msg);
    ocrFatGuid_t currentEdt;
    currentEdt.guid = (curTask == NULL) ? NULL_GUID : curTask->guid;
    currentEdt.metaDataPtr = curTask;

    // EDTs incrementally register on their dependences as elements
    // get satisfied (Guarantees O(n) traversal of dependence list).
    // Other events register when the dependence is added
    ocrGuidKind waiterKind = OCR_GUID_NONE;
    RESULT_ASSERT(guidKind(pd, waiter, &waiterKind), ==, 0);

#ifndef REG_ASYNC
#ifndef REG_ASYNC_SGL
    if(isDepAdd && waiterKind == OCR_GUID_EDT) {
        ocrAssert(false && "Should never happen anymore");
        // If we're adding a dependence and the waiter is an EDT we
        // skip this part. The event is registered on the EDT and
        // the EDT will register on the event only when its dependence
        // frontier reaches this event.
        return 0; //Require registerSignaler invocation
    }
#endif
#endif
    ocrAssert(waiterKind == OCR_GUID_EDT || (waiterKind & OCR_GUID_EVENT));

    DPRINTF(DEBUG_LVL_INFO, "Register waiter %s: "GUIDF" with waiter "GUIDF" on slot %"PRId32"\n",
            eventTypeToString(base), GUIDA(base->guid), GUIDA(waiter.guid), slot);
    // Lock to read the data field
    hal_lock(&(event->base.waitersLock));
    if(!(ocrGuidIsUninitialized(event->data))) {
        ocrFatGuid_t dataGuid = {.guid = event->data, .metaDataPtr = NULL};
        hal_unlock(&(event->base.waitersLock));
#ifdef REG_ASYNC_SGL
        regNode_t node = {.guid = waiter.guid, .slot = slot, .mode = mode};
#else
        regNode_t node = {.guid = waiter.guid, .slot = slot};
#endif
        // We send a message saying that we satisfy whatever tried to wait on us
        RESULT_PROPAGATE(commonSatisfyRegNode(pd, &msg, base->guid, dataGuid, currentEdt, &node));
        // Here it is still safe to use the base pointer because the satisfy
        // call cannot trigger the destruction of the event. For counted-events
        // the runtime takes care of it
        hal_lock(&(event->base.waitersLock));
        ocrEventHcCounted_t * devt = (ocrEventHcCounted_t *) event;
        // Account for this registration. When it reaches zero the event
        // can be deallocated since it is already satisfied and this call
        // was the last ocrAddDependence.
        ocrAssert(devt->nbDeps > 0);
        devt->nbDeps--;
        u64 nbDeps = devt->nbDeps;
        hal_unlock(&(event->base.waitersLock));
        // Check if we'll need to destroy the event
        if (nbDeps == 0) {
            // Can move that after satisfy to reduce CPL
            destructEventHc(base);
        }
        return 0; //Require registerSignaler invocation
    }

    // Lock is released by commonEnqueueWaiter
#ifdef REG_ASYNC_SGL
    return commonEnqueueWaiter(pd, base, waiter, slot, mode, currentEdt, &msg);
#else
    return commonEnqueueWaiter(pd, base, waiter, slot, currentEdt, &msg);
#endif
}
#endif


// In this call, we do not contend with satisfy
#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
u8 unregisterWaiterEventHc(ocrEvent_t *base, u32 sslot, ocrFatGuid_t waiter, u32 slot, bool isDepRem) {
#else
u8 unregisterWaiterEventHc(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot, bool isDepRem) {
#endif
    // Always search for the waiter because we don't know if it registered or not so
    // ignore isDepRem
    ocrEventHc_t *event = (ocrEventHc_t*)base;


    DPRINTF(DEBUG_LVL_INFO, "UnRegister waiter %s: "GUIDF" with waiter "GUIDF" on slot %"PRId32"\n",
            eventTypeToString(base), GUIDA(base->guid), GUIDA(waiter.guid), slot);

    ocrPolicyDomain_t *pd = NULL;
    ocrTask_t *curTask = NULL;
    PD_MSG_STACK(msg);
    regNode_t *waiters = NULL;
    u32 i;

    getCurrentEnv(&pd, NULL, &curTask, &msg);
    ocrFatGuid_t curEdt = {.guid = curTask!=NULL?curTask->guid:NULL_GUID, .metaDataPtr = curTask};

    // Acquire the DB that contains the waiters
#define PD_MSG (&msg)
#define PD_TYPE PD_MSG_DB_ACQUIRE
    msg.type = PD_MSG_DB_ACQUIRE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
    PD_MSG_FIELD_IO(guid) = event->waitersDb;
    PD_MSG_FIELD_IO(edt) = curEdt;
    PD_MSG_FIELD_IO(destLoc) = pd->myLocation;
    PD_MSG_FIELD_IO(edtSlot) = EDT_SLOT_NONE;
    PD_MSG_FIELD_IO(properties) = DB_MODE_RW | DB_PROP_RT_ACQUIRE;
    //Should be a local DB
    u8 res __attribute__((unused)) = pd->fcts.processMessage(pd, &msg, true);
    ocrAssert(!res); // Possible corruption of waitersDb

    waiters = (regNode_t*)PD_MSG_FIELD_O(ptr);
    //BUG #273
    event->waitersDb = PD_MSG_FIELD_IO(guid);
    ocrAssert(waiters);
#undef PD_TYPE
    // We search for the waiter that we need and remove it
    for(i = 0; i < event->waitersCount; ++i) {
        if(ocrGuidIsEq(waiters[i].guid, waiter.guid) && waiters[i].slot == slot) {
            // We will copy all the other ones
            hal_memCopy((void*)&waiters[i], (void*)&waiters[i+1],
                        sizeof(regNode_t)*(event->waitersCount - i - 1), false);
            --event->waitersCount;
            break;
        }
    }

    // We always release waitersDb
#define PD_TYPE PD_MSG_DB_RELEASE
    getCurrentEnv(NULL, NULL, NULL, &msg);
    msg.type = PD_MSG_DB_RELEASE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
    PD_MSG_FIELD_IO(guid) = event->waitersDb;
    PD_MSG_FIELD_I(edt) = curEdt;
    PD_MSG_FIELD_I(srcLoc) = pd->myLocation;
    PD_MSG_FIELD_I(ptr) = NULL;
    PD_MSG_FIELD_I(size) = 0;
    PD_MSG_FIELD_I(properties) = DB_PROP_RT_ACQUIRE;
    RESULT_PROPAGATE(pd->fcts.processMessage(pd, &msg, true));
#undef PD_MSG
#undef PD_TYPE
    return 0;
}


// In this call, we can have concurrent satisfy
#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
u8 unregisterWaiterEventHcPersist(ocrEvent_t *base, u32 sslot, ocrFatGuid_t waiter, u32 slot) {
#else
u8 unregisterWaiterEventHcPersist(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot) {
#endif
    ocrEventHcPersist_t *event = (ocrEventHcPersist_t*)base;


    DPRINTF(DEBUG_LVL_INFO, "Unregister waiter %s: "GUIDF" with waiter "GUIDF" on slot %"PRId32"\n",
            eventTypeToString(base), GUIDA(base->guid), GUIDA(waiter.guid), slot);

    ocrPolicyDomain_t *pd = NULL;
    ocrTask_t *curTask = NULL;
    PD_MSG_STACK(msg);
    regNode_t *waiters = NULL;
    u32 i;
    u8 toReturn = 0;

    getCurrentEnv(&pd, NULL, &curTask, &msg);
    ocrFatGuid_t curEdt = {.guid = curTask!=NULL?curTask->guid:NULL_GUID, .metaDataPtr = curTask};
    hal_lock(&(event->base.waitersLock));
    if(!(ocrGuidIsUninitialized(event->data))) {
        // We don't really care at this point so we don't do anything
        hal_unlock(&(event->base.waitersLock));
        return 0;
    }

    // Here we need to actually update our waiter list. We still hold the lock
    // Acquire the DB that contains the waiters
#define PD_MSG (&msg)
#define PD_TYPE PD_MSG_DB_ACQUIRE
    msg.type = PD_MSG_DB_ACQUIRE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
    PD_MSG_FIELD_IO(guid) = event->base.waitersDb;
    PD_MSG_FIELD_IO(edt) = curEdt;
    PD_MSG_FIELD_IO(destLoc) = pd->myLocation;
    PD_MSG_FIELD_IO(edtSlot) = EDT_SLOT_NONE;
    PD_MSG_FIELD_IO(properties) = DB_MODE_RW | DB_PROP_RT_ACQUIRE;
    //Should be a local DB
    if((toReturn = pd->fcts.processMessage(pd, &msg, true))) {
        ocrAssert(!toReturn); // Possible corruption of waitersDb
        hal_unlock(&(event->base.waitersLock));
        return toReturn;
    }
    //BUG #273: Guid reading
    waiters = (regNode_t*)PD_MSG_FIELD_O(ptr);
    event->base.waitersDb = PD_MSG_FIELD_IO(guid);
    ocrAssert(waiters);
#undef PD_TYPE
    // We search for the waiter that we need and remove it
    for(i = 0; i < event->base.waitersCount; ++i) {
        if(ocrGuidIsEq(waiters[i].guid, waiter.guid) && waiters[i].slot == slot) {
            // We will copy all the other ones
            hal_memCopy((void*)&waiters[i], (void*)&waiters[i+1],
                        sizeof(regNode_t)*(event->base.waitersCount - i - 1), false);
            --event->base.waitersCount;
            break;
        }
    }

    // We can release the lock now
    hal_unlock(&(event->base.waitersLock));

    // We always release waitersDb
#define PD_TYPE PD_MSG_DB_RELEASE
    getCurrentEnv(NULL, NULL, NULL, &msg);
    msg.type = PD_MSG_DB_RELEASE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
    PD_MSG_FIELD_IO(guid) = event->base.waitersDb;
    PD_MSG_FIELD_I(edt) = curEdt;
    PD_MSG_FIELD_I(srcLoc) = pd->myLocation;
    PD_MSG_FIELD_I(ptr) = NULL;
    PD_MSG_FIELD_I(size) = 0;
    PD_MSG_FIELD_I(properties) = DB_PROP_RT_ACQUIRE;
    RESULT_PROPAGATE(pd->fcts.processMessage(pd, &msg, true));
#undef PD_MSG
#undef PD_TYPE
    return 0;
}

u8 setHintEventHc(ocrEvent_t* self, ocrHint_t *hint) {
    ocrEventHc_t *derived = (ocrEventHc_t*)self;
    ocrRuntimeHint_t *rHint = &(derived->hint);
    OCR_RUNTIME_HINT_SET(hint, rHint, OCR_HINT_COUNT_EVT_HC, ocrHintPropEventHc, OCR_HINT_EVT_PROP_START);
    return 0;
}

u8 getHintEventHc(ocrEvent_t* self, ocrHint_t *hint) {
    ocrEventHc_t *derived = (ocrEventHc_t*)self;
    ocrRuntimeHint_t *rHint = &(derived->hint);
    OCR_RUNTIME_HINT_GET(hint, rHint, OCR_HINT_COUNT_EVT_HC, ocrHintPropEventHc, OCR_HINT_EVT_PROP_START);
    return 0;
}

ocrRuntimeHint_t* getRuntimeHintEventHc(ocrEvent_t* self) {
    ocrEventHc_t *derived = (ocrEventHc_t*)self;
    return &(derived->hint);
}


/******************************************************/
/* OCR-HC Events Factory                              */
/******************************************************/

ocrGuidKind eventTypeToGuidKind(ocrEventTypes_t eventType) {
    switch(eventType) {
        case OCR_EVENT_ONCE_T:
            return OCR_GUID_EVENT_ONCE;
#ifdef ENABLE_EXTENSION_COUNTED_EVT
        case OCR_EVENT_COUNTED_T:
            return OCR_GUID_EVENT_COUNTED;
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
        case OCR_EVENT_CHANNEL_T:
            return OCR_GUID_EVENT_CHANNEL;
#endif
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
        case OCR_EVENT_COLLECTIVE_T:
            return OCR_GUID_EVENT_COLLECTIVE;
#endif
        case OCR_EVENT_IDEM_T:
            return OCR_GUID_EVENT_IDEM;
        case OCR_EVENT_STICKY_T:
            return OCR_GUID_EVENT_STICKY;
        case OCR_EVENT_LATCH_T:
            return OCR_GUID_EVENT_LATCH;
        default:
            ocrAssert(false && "Unknown type of event");
        return OCR_GUID_NONE;
    }
}

static ocrEventTypes_t guidKindToEventType(ocrGuidKind kind) {
    switch(kind) {
        case OCR_GUID_EVENT_ONCE:
            return OCR_EVENT_ONCE_T;
#ifdef ENABLE_EXTENSION_COUNTED_EVT
        case OCR_GUID_EVENT_COUNTED:
            return OCR_EVENT_COUNTED_T;
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
        case OCR_GUID_EVENT_CHANNEL:
            return OCR_EVENT_CHANNEL_T;
#endif
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
        case OCR_GUID_EVENT_COLLECTIVE:
            return OCR_EVENT_COLLECTIVE_T;
#endif
        case OCR_GUID_EVENT_IDEM:
            return OCR_EVENT_IDEM_T;
        case OCR_GUID_EVENT_STICKY:
            return OCR_EVENT_STICKY_T;
        case OCR_GUID_EVENT_LATCH:
            return OCR_EVENT_LATCH_T;
        default:
            ocrAssert(false && "Unknown kind of event");
        return OCR_EVENT_T_MAX;
    }
}

void destructEventFactoryHc(ocrObjectFactory_t * factory) {
    runtimeChunkFree((u64)((ocrEventFactory_t*)factory)->hintPropMap, PERSISTENT_CHUNK);
    runtimeChunkFree((u64)factory, PERSISTENT_CHUNK);
}

// Internal utility function to send a 'push' message to update a remote copy.
//
// - The content of the message is implementation specific. It is dependent on how the
//   implementation decides to maintain coherence across multiple distributed copies of metadata.
// - Only supports M_SAT messages
//BUG #989: MT opportunity in certain circumstances. Check comments in deserializeEventFactoryHc
static void mdPushHcDist(ocrGuid_t evtGuid, ocrLocation_t loc, ocrGuid_t dbGuid, u32 mode, u32 factoryId) {
    ocrPolicyDomain_t *pd = NULL;
    PD_MSG_STACK(msg);
    getCurrentEnv(&pd, NULL, NULL, &msg);
    msg.destLocation = loc;
#define PD_MSG (&msg)
#define PD_TYPE PD_MSG_METADATA_COMM
    msg.type = PD_MSG_METADATA_COMM | PD_MSG_REQUEST;
    PD_MSG_FIELD_I(guid) = evtGuid;
    PD_MSG_FIELD_I(direction) = MD_DIR_PUSH;
    PD_MSG_FIELD_I(op) = 0; /*ocrObjectOperation_t*/ //TODO-MD-OP not clearly defined yet
    PD_MSG_FIELD_I(mode) = mode;
    PD_MSG_FIELD_I(factoryId) = factoryId;
    PD_MSG_FIELD_I(response) = NULL;
    PD_MSG_FIELD_I(mdPtr) = NULL;
    DPRINTF (DBG_HCEVT_LOG, "event-md: push "GUIDF" in mode=%d\n", GUIDA(evtGuid), mode);
    PD_MSG_FIELD_I(sizePayload) = 0; // This MUST be set to zero otherwise base size returns random crap
    ocrAssert((ocrPolicyMsgGetMsgBaseSize(&msg, true) + sizeof(ocrLocation_t) + sizeof(ocrGuid_t)) < sizeof(ocrPolicyMsg_t));
    // Always specify where the push comes from
    // TODO: This is redundant with the message header but the header doesn't make it all
    // the way to the recipient OCR object. Would that change if we collapse object's
    // functions into a big processMessage ?
    // LIMITATION: This is not generic at all. It assumes we always have a location first and maybe a guid afterward.
    //             Addl, we do not have access to the object's ptr hence cannot retrieve data from there
    // Location is enough for M_REG
    PD_MSG_FIELD_I(sizePayload) = sizeof(ocrLocation_t);
    // Serialization
    ocrAssert((mode == M_REG) || (mode == M_SAT) || (mode == M_DEL));
    char * ptr = &(PD_MSG_FIELD_I(payload));
    WR_PAYLOAD_DATA(ptr, ocrLocation_t, pd->myLocation);
    // For M_SAT, add the guid the event is satisfied with
    if (mode == M_SAT) {
        SET_PAYLOAD_DATA(ptr, M_SAT, ocrGuid_t, guid, dbGuid);
        // Check alignment issues
        ocrAssert(GET_PAYLOAD_DATA(ptr, M_SAT, ocrLocation_t, location) == pd->myLocation);
        PD_MSG_FIELD_I(sizePayload) += sizeof(ocrGuid_t);
    }
    pd->fcts.processMessage(pd, &msg, true);
#undef PD_MSG
#undef PD_TYPE
}

// Only used in no-forge mode
static void mdPullHcDist(ocrGuid_t guid, u32 mode, u32 factoryId) {
    // This implementation only pulls in clone mode
    ocrAssert(mode == M_CLONE);
    ocrPolicyDomain_t *pd = NULL;
    PD_MSG_STACK(msg);
    getCurrentEnv(&pd, NULL, NULL, &msg);
    // Since we just pull to clone the destination of this message is
    // always the location that owns the GUID.
    ocrLocation_t destLocation;
    u8 returnValue = pd->guidProviders[0]->fcts.getLocation(pd->guidProviders[0], guid, &destLocation);
    ocrAssert(!returnValue);
    msg.destLocation = destLocation;
#define PD_MSG (&msg)
#define PD_TYPE PD_MSG_METADATA_COMM
    msg.type = PD_MSG_METADATA_COMM | PD_MSG_REQUEST;
    PD_MSG_FIELD_I(guid) = guid;
    PD_MSG_FIELD_I(direction) = MD_DIR_PULL;
    PD_MSG_FIELD_I(op) = 0; /*ocrObjectOperation_t*/
    PD_MSG_FIELD_I(mode) = mode;
    PD_MSG_FIELD_I(factoryId) = factoryId;
    PD_MSG_FIELD_I(response) = NULL;
    PD_MSG_FIELD_I(mdPtr) = NULL;
    DPRINTF (DBG_HCEVT_LOG, "event-md: pull "GUIDF" in mode=%d\n", GUIDA(guid), mode);
    PD_MSG_FIELD_I(sizePayload) = 0; // This MUST be set to zero otherwise base size returns random crap
    ocrAssert((ocrPolicyMsgGetMsgBaseSize(&msg, true) + sizeof(ocrLocation_t)) < sizeof(ocrPolicyMsg_t));
    // Always specify where the push comes from
    // TODO: This is redundant with the message header but the header doesn't make it all
    // the way to the recipient OCR object. Would that change if we collapse object's
    // functions into a big processMessage ?
    PD_MSG_FIELD_I(sizePayload) = sizeof(ocrLocation_t);
    char * ptr = &(PD_MSG_FIELD_I(payload));
    WR_PAYLOAD_DATA(ptr, ocrLocation_t, pd->myLocation);
    pd->fcts.processMessage(pd, &msg, true);
#undef PD_MSG
#undef PD_TYPE
}

/******************************************************/
/* OCR-HC Collective Events                            */
/******************************************************/

#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT

//Fwd declaration needed for event initialization
static contributor_t * getContributor(ocrEventHcCollective_t * dself, u32 lslot);

#ifdef COLEVT_TREE_CONTRIB
typedef struct _remoteContrib_t {
    ocrPolicyMsg_t * msg;
} remoteContrib_t;
#endif

typedef struct _collectiveDbRecord_t {
    ocrFatGuid_t fguid;
    void * dbBackend;
} collectiveDbRecord_t;

void markInOrderParentAt(u16 * rktree, u16 * ktree, u16 k, u16 nbNodes, u16 me, u16 * counter) {
    // visit left children
    u16 r = (me*k)+1;
    if (r < nbNodes) {
        markInOrderParentAt(rktree, ktree, k, nbNodes, r, counter);
    }
    // Set my parent
    rktree[ktree[me]] = (me == 0) ? ((u16)-1) : ktree[(me-1)/k];
    // visit right children
    r=1;
    while(r<k) {
        u16 l = (me*k)+1+r;
        if (l >= nbNodes) {
            break;
        }
        markInOrderParentAt(rktree, ktree, k, nbNodes, l, counter);
        r++;
    }
}

static void markInOrderAt(u16 * ktree, u16 k, u16 nbNodes, u16 me, u16 * counter, u16 myLoc, u16 * myPos) {
    // visit left children
    u16 r = (me*k)+1;
    if (r < nbNodes) {
        markInOrderAt(ktree, k, nbNodes, r, counter, myLoc, myPos);
    }
    // visit me
    if (*counter == myLoc) {
        // Remember 'myLoc' maps to 'me'
        *myPos = me;
    }
    ktree[me] = (*counter)++;
    // visit right children
    r=1;
    while(r<k) {
        u16 l = (me*k)+1+r;
        if (l >= nbNodes) {
            break;
        }
        markInOrderAt(ktree, k, nbNodes, l, counter, myLoc, myPos);
        r++;
    }
}

static void markInOrder(u16 * ktree, u16 k, u16 nbNodes, u16 myLoc, u16 * myPos) {
    u16 counter = 0;
    markInOrderAt(ktree, k, nbNodes, 0, &counter, myLoc, myPos);
}

static void printKtree(u16 * ktree, u16 nbNodes) {
    u16 i=0;
    DPRINTF(DBG_COL_EVT, "[");
    while(i < (nbNodes-1)) {
        DPRINTF(DBG_COL_EVT, "%d, ", ktree[i]);
        i++;
    }
    DPRINTF(DBG_COL_EVT, "%d]\n", ktree[i]);
}

static u16 fctRedNbOfDescendants(u16 arity, ocrLocation_t myLoc, u32 nbPDs) {
    // Hardcode a k-ary tree based on participation of all PDs, ordered according to PD locations.
    u16 k = arity;
    u16 ktree[nbPDs];
    u16 myPos = -1;
    // Mark the k-tree in-order starting from the root and returning the position in the tree for myLoc
    markInOrder(ktree, k, nbPDs, (u16) myLoc, &myPos);
    // Determine 'i' child of myPos
    u16 i=0;
    while(i<k) {
        u16 cPos = (k*myPos)+(i+1);
        DPRINTF(DBG_COL_EVT, "fctRedNbOfDescendants arity=%"PRIu16" myLoc=%"PRIu64" nbPDs=%"PRIu32" i=%"PRIu16" myPos=%"PRIu16" cPos=%"PRIu16"\n", arity, myLoc, nbPDs, i, myPos, cPos);
        if (cPos >= nbPDs) {
            break;
        }
        i++;
    }
    DPRINTF(DBG_COL_EVT, "exit fctRedNbOfDescendants arity=%"PRIu16" myLoc=%"PRIu64" nbPDs=%"PRIu32" i=%"PRIu16" myPos=%"PRIu16"\n", arity, myLoc, nbPDs, i, myPos);
    return i;
}

static u16 fctRedPeerTopology(ocrEventHcCollective_t * dself, ocrLocation_t myLoc, u32 nbPDs) {
    if (nbPDs == 1) {
        dself->ancestorLoc = INVALID_LOCATION;
        return 0;
    }
    // Hardcode a k-ary tree based on participation of all PDs, ordered according to PD locations.
    u16 k = dself->params.arity;
    u16 ktree[nbPDs];
    u16 myPos = -1;
    // Mark the k-tree in-order starting from the root and returning the position in the tree for myLoc
    markInOrder(ktree, k, nbPDs, (u16) myLoc, &myPos);
    if (((u64) myLoc) == 0) printKtree(ktree, nbPDs); // debug
    ocrAssert(myPos != ((u64)-1));
    // Determine ancestor
    if (myPos == 0) { // root
        dself->ancestorLoc = INVALID_LOCATION;
    } else {
        // dself->ancestorLoc = ((ocrLocation_t) ktree[(k+myPos-2)/k]);
        dself->ancestorLoc = ((ocrLocation_t) ktree[(myPos-1)/k]);
    }
    // Determine 'i' child of myPos
    u16 i=0;
    while(i<k) {
        // u16 cPos = (k*myPos)-(k-2)+i;
        u16 cPos = (k*myPos)+(i+1);
        if (cPos >= nbPDs) {
            break;
        }
        dself->descendantsLoc[i] = ((ocrLocation_t) ktree[cPos]);
        DPRINTF(DBG_COL_EVT, "set myLoc=%"PRIu64" descendantsLoc[%"PRIu16"]=%"PRIu16"\n", (u64) myLoc, i, ktree[cPos]);
        i++;
    }
    return i;
}
#endif /*ENABLE_EXTENSION_COLLECTIVE_EVT*/


/******************************************************/
/* OCR-HC Events Master/Slave                         */
/******************************************************/

#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
static u16 getDatumSize(ocrEventHcCollective_t * dself);
static u32 getContribSize(ocrEventHcCollective_t * dself);
static u32 getMaxContribLocal(ocrEventHcCollective_t * dself);
static remoteContrib_t * getRemoteContrib(ocrEventHcCollective_t * dself, rph_t rph, u16 cslot);
static void fctRedBuildInOrderIdxToArrayIdx(ocrEventHcCollective_t * dself, u16 nbOfDescendants, u16 * inOrderIdxToArrayIdx);
#endif /*ENABLE_EXTENSION_COLLECTIVE_EVT*/

static u8 initNewEventHc(ocrEventHc_t * event, ocrEventTypes_t eventType, ocrGuid_t data, ocrEventFactory_t * factory, u32 sizeOfGuid, ocrParamList_t *perInstance) {
    ocrEvent_t * base = (ocrEvent_t*) event;
    base->kind = eventType;
    u32 factoryId = factory->factoryId;
    base->base.fctId = factoryId;
    base->fctId = factoryId;

    // Set-up HC specific structures
    event->waitersCount = 0;
    event->waitersMax = HCEVT_WAITER_STATIC_COUNT;
    event->waitersLock = INIT_LOCK;

    int jj = 0;
    while (jj < HCEVT_WAITER_STATIC_COUNT) {
        event->waiters[jj].guid = NULL_GUID;
        event->waiters[jj].slot = 0;
        event->waiters[jj].mode = -1;
        jj++;
    }

    if(eventType == OCR_EVENT_LATCH_T) {
        // Initialize the counter
        if (perInstance != NULL) {
#ifdef ENABLE_EXTENSION_PARAMS_EVT
            // Expecting ocrEventParams_t as the paramlist
            ocrEventParams_t * params = (ocrEventParams_t *) perInstance;
            ((ocrEventHcLatch_t*)event)->counter = params->EVENT_LATCH.counter;
#endif
        } else {
            ((ocrEventHcLatch_t*)event)->counter = 0;
        }
    }
    event->mdClass.peers = NULL;
    event->mdClass.satFromLoc = INVALID_LOCATION;
    event->mdClass.delFromLoc = INVALID_LOCATION;
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    if(eventType == OCR_EVENT_IDEM_T || eventType == OCR_EVENT_STICKY_T || eventType == OCR_EVENT_COUNTED_T) {
#else
    if(eventType == OCR_EVENT_IDEM_T || eventType == OCR_EVENT_STICKY_T) {
#endif
        ((ocrEventHcPersist_t*)event)->data = data;
        if (!ocrGuidIsUninitialized(data)) {
            // For master-slave impl, we did a clone and the event was already satisfied
            event->waitersCount = STATE_CHECKED_OUT;
        }
    }

#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    if(eventType == OCR_EVENT_CHANNEL_T) {
        // Check current extension implementation restrictions
        ocrEventHcChannel_t * devt = ((ocrEventHcChannel_t*)event);
        // Expecting ocrEventParams_t as the paramlist
        ocrEventParams_t * params = (ocrEventParams_t *) perInstance;
        ocrAssert((params->EVENT_CHANNEL.nbSat == 1) && "Channel-event limitation nbSat must be set to 1");
        ocrAssert((params->EVENT_CHANNEL.nbDeps == 1) && "Channel-event limitation nbDeps must be set to 1");
        ocrAssert((params->EVENT_CHANNEL.maxGen != 0) && "Channel-event maxGen=0 invalid value");
        u32 maxGen = (params->EVENT_CHANNEL.maxGen == EVENT_CHANNEL_UNBOUNDED) ? 1 : params->EVENT_CHANNEL.maxGen;
        devt->maxGen = params->EVENT_CHANNEL.maxGen;
        devt->nbSat = params->EVENT_CHANNEL.nbSat;
        devt->satBufSz = maxGen * devt->nbSat;
        devt->nbDeps = params->EVENT_CHANNEL.nbDeps;
        devt->waitBufSz = maxGen * devt->nbDeps;
        if (devt->maxGen == EVENT_CHANNEL_UNBOUNDED) {
            ocrPolicyDomain_t * pd;
            getCurrentEnv(&pd, NULL, NULL, NULL);
            // Setup backing data-structure pointers
            devt->satBuffer = (ocrGuid_t *) pd->fcts.pdMalloc(pd, sizeof(ocrGuid_t) * devt->satBufSz);
            devt->waiters = (regNode_t *) pd->fcts.pdMalloc(pd, sizeof(regNode_t) * devt->waitBufSz);
        } else {
            // Setup backing data-structure pointers
            u32 baseSize = sizeof(ocrEventHcChannel_t);
            u32 sizeSat = (sizeof(ocrGuid_t) * devt->satBufSz);
            devt->satBuffer = (ocrGuid_t *)((u64)base + baseSize);
            devt->waiters = (regNode_t *)((u64)base + baseSize + sizeSat);
        }
        devt->headSat = 0;
        devt->tailSat = 0;
        devt->headWaiter = 0;
        devt->tailWaiter = 0;
        u32 i;
        for (i=0; i<devt->satBufSz; i++) {
            devt->satBuffer[i] = UNINITIALIZED_GUID;
        }
#ifdef OCR_ASSERT
        regNode_t regnode;
        regnode.guid = NULL_GUID;
        regnode.slot = 0;
        regnode.mode = -1;
        // This is not really necessary outside of debug mode
        for (i=0; i<devt->waitBufSz; i++) {
            devt->waiters[i] = regnode;
        }
#endif
    }
#endif
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
    if (eventType == OCR_EVENT_COLLECTIVE_T) {
        ocrPolicyDomain_t *pd = NULL;
        ocrTask_t *curTask = NULL;
        getCurrentEnv(&pd, NULL, &curTask, NULL);
        ocrFatGuid_t curEdt = {.guid = curTask!=NULL?curTask->guid:NULL_GUID, .metaDataPtr = curTask};
        ocrEventHcCollective_t * devt = ((ocrEventHcCollective_t*)event);
        ocrEventParams_t * params = (ocrEventParams_t *) perInstance;
        devt->params = params->EVENT_COLLECTIVE;
        ocrAssert(params->EVENT_COLLECTIVE.maxGen > 0);
        ocrAssert(params->EVENT_COLLECTIVE.nbDatum > 0);
        ocrAssert(params->EVENT_COLLECTIVE.nbContribs > 0);
        ocrAssert(params->EVENT_COLLECTIVE.nbContribsPd > 0);
        ocrAssert(params->EVENT_COLLECTIVE.nbContribsPd <= params->EVENT_COLLECTIVE.nbContribs);
        // After the reduction event struct, data is organized as is:
        // u32*maxGen | phaseDbResult*maxGen | ((contributor_t | regNodes*maxGen | datum*nbDatum*maxGen) * nbContribsPd)
        setReductionFctPtr(devt, params->EVENT_COLLECTIVE.op);
        devt->myLoc = pd->myLocation;
        char * curPtr = ((char *) devt) + sizeof(ocrEventHcCollective_t);
        // Setup peer information
#ifdef OCR_ASSERT
        devt->ancestorLoc = INVALID_LOCATION;
#endif
        // Warning this assignment must happen before 'fctRedPeerTopology' is called as it uses that member.
        devt->descendantsLoc = (ocrLocation_t *) curPtr;
        // Based on the PD location and given the PD arity, infer ancestor and descendants locations.
        devt->nbOfDescendants = fctRedPeerTopology(devt, pd->myLocation, pd->neighborCount+1);
        DPRINTF(DBG_COL_EVT, "init nbOfDescendants=%"PRIu16"\n", devt->nbOfDescendants);
        curPtr += (sizeof(ocrLocation_t) * devt->nbOfDescendants);
        devt->phaseLocalContribCounters = (u32*) curPtr;
        curPtr += (sizeof(u32) * params->EVENT_COLLECTIVE.maxGen);
        devt->phaseDbResult = (collectiveDbRecord_t*) curPtr;
        curPtr += (sizeof(collectiveDbRecord_t) * params->EVENT_COLLECTIVE.maxGen);
#ifdef COLEVT_TREE_CONTRIB
        // Compute number of tree nodes. We need the array to be that large to accomodate the local 2-ary reduction tree
        u16 remoteContribCount = ((devt->nbOfDescendants) ? ((devt->nbOfDescendants % 2) ? devt->nbOfDescendants : devt->nbOfDescendants+1) : 0);
        devt->inOrderIdxToArrayIdx = (u16 *) curPtr;
        // for that array, we only need the number of leaves
        curPtr += (sizeof(u16) * ((devt->nbOfDescendants) ? ((remoteContribCount/2)+1) : 0));
        devt->remoteContribs = (remoteContrib_t *) curPtr;
        curPtr += (sizeof(remoteContrib_t) * remoteContribCount * params->EVENT_COLLECTIVE.maxGen);
#endif
        devt->contributors = (contributor_t *) curPtr;
        u32 maxGen = params->EVENT_COLLECTIVE.maxGen;
        ocrAssert(devt->reduce != NULL);
        u32 c, ub;
        c=0; ub = params->EVENT_COLLECTIVE.maxGen;
        while(c < ub) {
            devt->phaseLocalContribCounters[c++] = 0;
        }
        c=0; ub = params->EVENT_COLLECTIVE.maxGen;
        collectiveDbRecord_t dbRecord = {.fguid.guid = NULL_GUID, .fguid.metaDataPtr = NULL, .dbBackend = NULL};
        while(c < ub) {
            devt->phaseDbResult[c++] = dbRecord;
        }
#ifdef COLEVT_TREE_CONTRIB
        // Binary reduction tree for aggregating the local and all our descendants remote contributions
        fctRedBuildInOrderIdxToArrayIdx(devt, devt->nbOfDescendants, devt->inOrderIdxToArrayIdx);
        remoteContrib_t rcInit = {.msg = NULL};
        c=0; ub = remoteContribCount * params->EVENT_COLLECTIVE.maxGen;
        while(c < ub) {
            devt->remoteContribs[c++] = rcInit;
        }
        ocrAssert(((ub == 0) || (getRemoteContrib(devt, 0, 0)->msg == NULL)) && "getRemoteContrib not properly init");
#endif

        c=0; ub = params->EVENT_COLLECTIVE.maxGen;
        ocrFatGuid_t dbFatGuid = {.guid = NULL_GUID, .metaDataPtr = NULL};
        while(c < ub) {
            PD_MSG_STACK(msg);
            getCurrentEnv(NULL, NULL, NULL, &msg);
#define PD_MSG (&msg)
#define PD_TYPE PD_MSG_DB_CREATE
            msg.type = PD_MSG_DB_CREATE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
            PD_MSG_FIELD_IO(guid) = dbFatGuid;
            PD_MSG_FIELD_IO(size) = getContribSize(devt);
            PD_MSG_FIELD_IO(properties) = DB_PROP_RT_ACQUIRE;
            PD_MSG_FIELD_I(edt) = curEdt;
            PD_MSG_FIELD_I(hint) = NULL_HINT;
            PD_MSG_FIELD_I(dbType) = RUNTIME_DBTYPE;
            PD_MSG_FIELD_I(allocator) = NO_ALLOC;
            RESULT_PROPAGATE(pd->fcts.processMessage(pd, &msg, true));
            devt->phaseDbResult[c].fguid = PD_MSG_FIELD_IO(guid);
            devt->phaseDbResult[c].dbBackend = PD_MSG_FIELD_O(ptr);
            ocrAssert(!ocrGuidIsNull(devt->phaseDbResult[c].fguid.guid) &&
                    (devt->phaseDbResult[c].fguid.metaDataPtr != NULL));
            c++;
#undef PD_TYPE
#undef PD_MSG
        }

        c=0, ub = params->EVENT_COLLECTIVE.nbContribsPd;
        while(c < ub) {
            contributor_t * contrib = getContributor(devt, c);
            contrib->iph = 0;
            contrib->oph = 0;
            contrib->deps = (regNode_t *) (((char *) contrib) + sizeof(contributor_t));
            contrib->contribs = (((char *) contrib->deps) + (sizeof(regNode_t) * maxGen));
#ifdef OCR_ASSERT
            regNode_t regnode;
            regnode.guid = UNINITIALIZED_GUID;
            regnode.slot = -1;
            regnode.mode = -1;
            rph_t ph;
            for(ph=0; ph < maxGen; ph++) {
                contrib->deps[ph] = regnode;
            }
#endif
            c++;
        }
        DPRINTF(DBG_COL_EVT, "getDatumSize=%"PRIu16"\n", getDatumSize(devt));
        DPRINTF(DBG_COL_EVT, "getContribSize=%"PRIu32"\n", getContribSize(devt));
        DPRINTF(DBG_COL_EVT, "getMaxContribLocal=%"PRIu32"\n", getMaxContribLocal(devt));
        DPRINTF(DBG_COL_EVT, "topology pd=%"PRIu64" ancestorPd=%"PRIu64" arity=%"PRIu16" nbOfDescendants=%"PRIu16"", (u64) devt->myLoc, devt->ancestorLoc, (u16) devt->params.arity, (u16) devt->nbOfDescendants);
        c=0;
        while (c < devt->nbOfDescendants) {
            DPRINTF(DBG_COL_EVT, " d[%"PRIu16"]=%"PRIu64"", (u16) c, (u64) devt->descendantsLoc[c]);
            c++;
        }
        DPRINTF(DBG_COL_EVT, "\n");
    }
#endif
    u32 hintc = OCR_HINT_COUNT_EVT_HC;
    if (hintc == 0) {
        event->hint.hintMask = 0;
        event->hint.hintVal = NULL;
    } else {
        OCR_RUNTIME_HINT_MASK_INIT(event->hint.hintMask, OCR_HINT_EVT_T, factoryId);
        event->hint.hintVal = (u64*)((u64)base + sizeOfGuid);
    }

    // Initialize GUIDs for the waiters data-blocks
    event->waitersDb.guid = UNINITIALIZED_GUID;
    event->waitersDb.metaDataPtr = NULL;

#ifdef ENABLE_EXTENSION_COUNTED_EVT
    if(eventType == OCR_EVENT_COUNTED_T) {
        // Initialize the counter for dependencies tracking
        ocrEventParams_t * params = (ocrEventParams_t *) perInstance;
        if ((params != NULL) && (params->EVENT_COUNTED.nbDeps != 0)) {
            ((ocrEventHcCounted_t*)event)->nbDeps = params->EVENT_COUNTED.nbDeps;
        } else {
            DPRINTF(DBG_HCEVT_ERR, "error: Illegal nbDeps value (zero) for OCR_EVENT_COUNTED_T 0x"GUIDF"\n", GUIDA(base->guid));
            factory->fcts[OCR_EVENT_COUNTED_T].destruct(base);
            ocrAssert(false);
            return OCR_EINVAL; // what ?
        }
    }
#endif
    return 0;
}

static u8 allocateNewEventHc(ocrGuidKind guidKind, ocrFatGuid_t * resultGuid, u32 * sizeofMd, u32 properties, ocrParamList_t *perInstance) {
    ocrPolicyDomain_t *pd = NULL;
    PD_MSG_STACK(msg);
    getCurrentEnv(&pd, NULL, NULL, &msg);
    // Create the event itself by getting a GUID
    *sizeofMd = sizeof(ocrEventHc_t);
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    if(guidKind == OCR_GUID_EVENT_COUNTED) {
        *sizeofMd = sizeof(ocrEventHcCounted_t);
    }
#endif
    if(guidKind == OCR_GUID_EVENT_LATCH) {
        *sizeofMd = sizeof(ocrEventHcLatch_t);
    }
    if((guidKind == OCR_GUID_EVENT_IDEM) || (guidKind == OCR_GUID_EVENT_STICKY)) {
        *sizeofMd = sizeof(ocrEventHcPersist_t);
    }
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    if(guidKind == OCR_GUID_EVENT_CHANNEL) {
#ifndef ENABLE_EXTENSION_PARAMS_EVT
        ocrAssert(false && "ENABLE_EXTENSION_PARAMS_EVT must be defined to use Channel-events");
#endif
        ocrAssert((perInstance != NULL) && "error: No parameters specified at Channel-event creation");
        // Expecting ocrEventParams_t as the paramlist
        ocrEventParams_t * params = (ocrEventParams_t *) perInstance;
        // Allocate extra space to store backing data-structures that are parameter-dependent
        u32 xtraSpace = 0;
        if (params->EVENT_CHANNEL.maxGen != EVENT_CHANNEL_UNBOUNDED) {
            // Allocate extra space to store backing data-structures that are parameter-dependent
            u32 sizeSat = (sizeof(ocrGuid_t) * params->EVENT_CHANNEL.nbSat * params->EVENT_CHANNEL.maxGen);
            u32 sizeWaiters = (sizeof(regNode_t) * params->EVENT_CHANNEL.nbDeps * params->EVENT_CHANNEL.maxGen);
            xtraSpace = (sizeSat + sizeWaiters);
        }
        *sizeofMd = sizeof(ocrEventHcChannel_t) + xtraSpace;
    }
#endif
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
    if(guidKind == OCR_GUID_EVENT_COLLECTIVE) {
#ifndef ENABLE_EXTENSION_PARAMS_EVT
        ocrAssert(false && "ENABLE_EXTENSION_PARAMS_EVT must be defined to use Collective-events");
#endif
        ocrAssert((perInstance != NULL) && "error: No parameters specified at Collective-event creation");
        // Expecting ocrEventParams_t as the paramlist
        ocrEventParams_t * params = (ocrEventParams_t *) perInstance;
        // Allocate extra space to store backing data-structures that are parameter-dependent
        u32 xtraSpace = 0;
        u32 nbPds = pd->neighborCount+1;
        u16 nbOfDescendants = fctRedNbOfDescendants(params->EVENT_COLLECTIVE.arity, pd->myLocation, nbPds);
        DPRINTF(DBG_COL_EVT, "alloc nbOfDescendants=%"PRIu16"\n", nbOfDescendants);
        // For backing storage of 'descendantsLoc'
        xtraSpace += sizeof(ocrLocation_t) * nbOfDescendants;
        // For backing storage of 'phaseLocalContribCounters'
        xtraSpace += sizeof(u32) * params->EVENT_COLLECTIVE.maxGen;
        // For backing storage of 'phaseDbResult'
        xtraSpace += sizeof(collectiveDbRecord_t) * params->EVENT_COLLECTIVE.maxGen;
#ifdef COLEVT_TREE_CONTRIB
        //If has any descendant, then + 1 to also account for the local contribution
        u16 remoteContribCount = ((nbOfDescendants) ? ((nbOfDescendants % 2) ? nbOfDescendants : nbOfDescendants+1) : 0);
        // For backing storage of 'inOrderIdxToArrayIdx'
        xtraSpace += sizeof(u16) * ((nbOfDescendants) ? ((remoteContribCount/2)+1) : 0);
        // Linearized arrays as backing storage for the remote contribution reduction binary tree.
        xtraSpace += sizeof(remoteContrib_t) * remoteContribCount * params->EVENT_COLLECTIVE.maxGen;
#endif
        // For backing storage of 'contributors'
        u32 datumSize = REDOP_GET(DATUM_SIZE, params->EVENT_COLLECTIVE.op)+1; // 0 is 1 hence +1
        u32 sizeContribs = (datumSize * params->EVENT_COLLECTIVE.nbDatum * params->EVENT_COLLECTIVE.maxGen);
        u32 sizeDeps = (sizeof(regNode_t) * params->EVENT_COLLECTIVE.maxGen);
        xtraSpace += (sizeof(contributor_t) + sizeContribs + sizeDeps) * params->EVENT_COLLECTIVE.nbContribsPd;
        *sizeofMd = sizeof(ocrEventHcCollective_t) + xtraSpace;
        DPRINTF(DBG_COL_EVT, "ocrEventHcCollective_t size=%"PRIu32" bytes\n", *sizeofMd);
    }
#endif
    u32 hintc = OCR_HINT_COUNT_EVT_HC;
#define PD_MSG (&msg)
#define PD_TYPE PD_MSG_GUID_CREATE
    msg.type = PD_MSG_GUID_CREATE | PD_MSG_REQUEST | PD_MSG_REQ_RESPONSE;
    PD_MSG_FIELD_IO(guid) = *resultGuid;
    // We allocate everything in the meta-data to keep things simple
    PD_MSG_FIELD_I(size) = (*sizeofMd) + hintc*sizeof(u64);
    PD_MSG_FIELD_I(kind) = guidKind;
    PD_MSG_FIELD_I(targetLoc) = pd->myLocation;
    PD_MSG_FIELD_I(properties) = properties;
    RESULT_PROPAGATE(pd->fcts.processMessage(pd, &msg, true));
    u8 returnValue = PD_MSG_FIELD_O(returnDetail);
    if (returnValue && (returnValue != OCR_EGUIDEXISTS)) {
        ocrAssert(false);
        return returnValue;
    }
    resultGuid->guid = PD_MSG_FIELD_IO(guid.guid);
    resultGuid->metaDataPtr = PD_MSG_FIELD_IO(guid.metaDataPtr);
#ifdef ENABLE_RESILIENCY
    ocrEvent_t *evt = (ocrEvent_t*)resultGuid->metaDataPtr;
    evt->base.kind = guidKind;
    evt->base.size = (*sizeofMd) + hintc*sizeof(u64);
#endif
#undef PD_MSG
#undef PD_TYPE
    return returnValue;
}

// Creates a distributed event that has additional metadata
// REQ: fguid must be a valid GUID so that the event has either already been allocated in some
//       other PD (hence we have a guid) or it's a labeled guid and the guid is well-formed.
// NOTE: This was originally intended to be used for forging event but it also works
//       to allocate an event from the M_CLONE path.
// newEventHcDist systematically calls allocateNewEventHc, which systematically has a ISVALID guid and RECORD the GUID
// - The PD filters references to events in addDependence and calls resolveRemoteMetaData(blocking, fetch) if events are detected.
//   Callers compete on the GP getVal call in fetch mode. Only one succeeds issuing the call and they all spin on the MD coming
//   back. (since call is configured to be blocking).
// - When the clone msg comes back it is deserialized and newEventHcDist is invoked. Calling into allocateNewEventHc which inserts
//   a new MD proxy. hHnce, this is a BUG
static u8 newEventHcDist(ocrFatGuid_t * fguid, ocrGuid_t data, ocrEventFactory_t * factory) {
    ocrAssert(!ocrGuidIsNull(fguid->guid));
    ocrPolicyDomain_t *pd = NULL;
    getCurrentEnv(&pd, NULL, NULL, NULL);
    ocrGuidKind guidKind;
    u8 returnValue = pd->guidProviders[0]->fcts.getKind(pd->guidProviders[0], fguid->guid, &guidKind);
    ocrEventTypes_t eventType = guidKindToEventType(guidKind);
    ocrAssert(!returnValue);
    ocrLocation_t guidLoc;
    returnValue = pd->guidProviders[0]->fcts.getLocation(pd->guidProviders[0], fguid->guid, &guidLoc);
    ocrAssert(!returnValue);
    u32 sizeOfMd;
    returnValue = allocateNewEventHc(guidKind, fguid, &sizeOfMd, /*prop*/GUID_PROP_ISVALID | GUID_PROP_TORECORD, NULL);
    ocrEventHc_t *event = (ocrEventHc_t*) fguid->metaDataPtr;
    u8 ret = initNewEventHc(event, eventType, data, factory, sizeOfMd, NULL);
    if (ret) { return ret; }
    if (pd->myLocation != guidLoc) { //A slave has no peers
        event->mdClass.peers = NULL;
    } else { // master registers the slave location as a peer
        locNode_t * locNode = (locNode_t *) pd->fcts.pdMalloc(pd, sizeof(locNode_t));
        locNode->loc = guidLoc;
        locNode->next = NULL;
        event->mdClass.peers = locNode;
    }
    // Do this at the very end; it indicates that the object of the GUID is actually valid
    hal_fence(); // Make sure sure this really happens last
    ((ocrEvent_t*) event)->guid = fguid->guid;

    DPRINTF(DEBUG_LVL_INFO, "Create %s: "GUIDF"\n", eventTypeToString((ocrEvent_t *)event), GUIDA(fguid->guid));
#ifdef OCR_ENABLE_STATISTICS
    statsEVT_CREATE(getCurrentPD(), getCurrentEDT(), NULL, fguid->guid, ((ocrEvent_t*) event));
#endif
    ocrAssert(!returnValue);
    return returnValue;
}

 u8 cloneEventFactoryHc(ocrObjectFactory_t * pfactory, ocrGuid_t guid, ocrObject_t ** mdPtr, ocrLocation_t dest, u32 type) {
    ocrEventFactory_t * factory = (ocrEventFactory_t *) pfactory;
    if (ENABLE_EVENT_MDC_FORGE) { // Allow forging
        ocrPolicyDomain_t * pd;
        getCurrentEnv(&pd, NULL, NULL, NULL);
        ocrGuidKind guidKind;
        RESULT_ASSERT(pd->guidProviders[0]->fcts.getKind(pd->guidProviders[0], guid, &guidKind), ==, 0 );
        if (MDC_SUPPORT_EVT(guidKind)) { // And is for a supported GUID kind
            // Create a new instance
            ocrFatGuid_t fguid = {.guid = guid, .metaDataPtr = NULL};
            newEventHcDist(&fguid, UNINITIALIZED_GUID, factory);
            *mdPtr = fguid.metaDataPtr;
            ocrPolicyDomain_t * pd;
            getCurrentEnv(&pd, NULL, NULL, NULL);
            ocrLocation_t destLocation;
            pd->guidProviders[0]->fcts.getLocation(pd->guidProviders[0], guid, &destLocation);
            // Generate a registration message for the owner of that guid
            mdPushHcDist(guid, destLocation, NULL_GUID, M_REG, factory->factoryId);
            return 0;
        }
    }
    // Otherwise fall-through to regular cloning
    *mdPtr = NULL;
    mdPullHcDist(guid, M_CLONE, factory->factoryId);
    return OCR_EPEND;
}

u8 serializeEventFactoryHc(ocrObjectFactory_t * factory, ocrGuid_t guid, ocrObject_t * src, u64 * mode, ocrLocation_t destLocation, void ** destBuffer, u64 * destSize) {
    ocrAssert(destBuffer != NULL);
    ocrAssert(!ocrGuidIsNull(guid));
    ocrAssert(*destSize != 0);
#if ENABLE_EVENT_MDC_FORGE
    ocrAssert(false); // Never do a pull when we forge events
#else
    // More of a proof of concept since we can easily forge events in this implementation
    ocrEvent_t * evt = (ocrEvent_t *) src;
    *mode = 0; // clear bits
    // Specialized implementation serialization:
    // - Ignore most of the field and initialize them on deserialization
    // - Must handle concurrency with competing operations being invoked on the event
    //   - By design, there should not be a concurrent destruct
    ocrEventHc_t * devt = (ocrEventHc_t *) src;
    switch(evt->kind) {
        case OCR_EVENT_STICKY_T:
        case OCR_EVENT_IDEM_T:
        {
            // - There's not much information to serialize beside the
            //   GUID the event is currently satisfied with (or not).
            // - We always register the peer so that we can reclaim
            //   all them on destruction
            ocrPolicyDomain_t *pd = NULL;
            getCurrentEnv(&pd, NULL, NULL, NULL);
            ocrAssert(*destSize >= sizeof(ocrGuid_t));
            // Note: this is concurrent with the event being satisfied so it's best effort.
            locNode_t * locNode = (locNode_t *) pd->fcts.pdMalloc(pd, sizeof(locNode_t));
            hal_lock(&(devt->waitersLock));
            ocrGuid_t data = ((ocrEventHcPersist_t *)evt)->data;
            locNode->loc = destLocation;
            locNode->next = devt->mdClass.peers;
            // Enqueuing at the head relies on the fact the slave location doesn't issue
            // multiple simultaneous pull requests for the same OCR object. Otherwise we
            // would have to enforce unicity when enqueuing.
            devt->mdClass.peers = locNode;
            hal_unlock(&(devt->waitersLock));
            // Just send the current GUID the event is satisfied (or not) with.
            SET_PAYLOAD_DATA((*destBuffer), M_CLONE, ocrGuid_t, guid, data);
        break;
        }
        //COL-EVTX: Lazy discovery of collective events would need serialize impl
    default:
        ocrAssert(false && "Metadata-cloning not supported for this event type");
    }
#endif
    return 0;
}

u8 newEventHc(ocrEventFactory_t * factory, ocrFatGuid_t *fguid,
              ocrEventTypes_t eventType, u32 properties,
              ocrParamList_t *perInstance) {
    u32 sizeOfMd;
    ocrGuidKind guidKind = eventTypeToGuidKind(eventType);
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
    bool isColEvent = (guidKind == OCR_GUID_EVENT_COLLECTIVE);
    if (isColEvent) {
        ocrAssert(properties & GUID_PROP_IS_LABELED);
        // For reduction events, we delay recording the GUID until after alloc & init, when calling registerGuid.
        properties |= GUID_PROP_ISVALID; // Instruct the allocation code to reuse the labeled GUID
        properties &= ~GUID_PROP_TORECORD;
        ocrPolicyDomain_t * pd;
        getCurrentEnv(&pd, NULL, NULL, NULL);
        MdProxy_t * mdProxy = NULL;
        u64 val;
        u8 res;
        // Try to resolve an instance. Either we compete and win the right to create
        // the MD proxy or we keep trying to read the MD proxy's pointer to become not NULL.
        do {
            //getVal - resolve
            res = pd->guidProviders[0]->fcts.getVal(pd->guidProviders[0], fguid->guid, &val, NULL, MD_ALLOC, &mdProxy);
            ocrAssert(mdProxy != NULL);
        } while (res == OCR_EPEND);
        if (val != 0) { // Either we're spinning or we got lucky and resolved right away
            fguid->metaDataPtr = (void *) mdProxy->ptr;
            return OCR_EGUIDEXISTS;
        } // else: need to do the actual allocation and initialization
    } else {
         properties |= GUID_PROP_TORECORD;
    }
#else
    properties |= GUID_PROP_TORECORD;
#endif

    u8 returnValue = allocateNewEventHc(guidKind, fguid, &sizeOfMd, properties, perInstance);
    if (returnValue) { ocrAssert(returnValue == OCR_EGUIDEXISTS); return returnValue; }

    ocrEventHc_t *event = (ocrEventHc_t*) fguid->metaDataPtr;
    returnValue = initNewEventHc(event, eventType, UNINITIALIZED_GUID, factory, sizeOfMd, perInstance);
    if (returnValue) { return returnValue; }

    // Do this at the very end; it indicates that the object of the GUID is actually valid
    hal_fence(); // Make sure sure this really happens last
    ((ocrEvent_t*) event)->guid = fguid->guid;
    DPRINTF(DEBUG_LVL_INFO, "Create %s: "GUIDF"\n", eventTypeToString(((ocrEvent_t*) event)), GUIDA(fguid->guid));
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
    if (isColEvent) { // Register the event
        DPRINTF(DBG_COL_EVT, "REDL: Calling register for "GUIDF"\n", GUIDA(fguid->guid));
        ocrPolicyDomain_t * pd;
        getCurrentEnv(&pd, NULL, NULL, NULL);
        // Call GP registration. When the MD proxy ptr is set to the allocated OCR object instance,
        // it unlocks concurrent creations spinning on it and compete on CAS-ing the waiter list.
        // Once CAS is done, it will also wake up process msg calls that had been enqueued.
        returnValue = pd->guidProviders[0]->fcts.registerGuid(pd->guidProviders[0], fguid->guid, (u64) event);
    }
#endif
#ifdef OCR_ENABLE_STATISTICS
    statsEVT_CREATE(getCurrentPD(), getCurrentEDT(), NULL, fguid->guid, ((ocrEvent_t*) event));
#endif
    ocrAssert(!returnValue);
    return returnValue;
}

// Since this is factory wide, we'd need to specialize based on the event kind. Similarly to what serializeEventFactoryHc does
u8 deserializeEventFactoryHc(ocrObjectFactory_t * pfactory, ocrGuid_t evtGuid, ocrObject_t ** dest, u64 mode, void * srcBuffer, u64 srcSize) {
    ocrPolicyDomain_t *pd = NULL;
    getCurrentEnv(&pd, NULL, NULL, NULL);
    ocrGuidKind guidKind;
    u8 ret = pd->guidProviders[0]->fcts.getKind(pd->guidProviders[0], evtGuid, &guidKind);
    ocrAssert(!ret);
    ocrEventFactory_t * factory = (ocrEventFactory_t *) pfactory;
    if ((guidKindToEventType(guidKind) == OCR_EVENT_STICKY_T) || (guidKindToEventType(guidKind) == OCR_EVENT_IDEM_T)) {
        switch(mode) {
            case M_CLONE: {
                // The payload should carry the GUID for the data the event is satisfied with if any.
                // We create the event anyhow: would only make sense for persistent events that
                // may still have ocrAddDependence coming in and would benefit from having the MD local.
                DPRINTF(DBG_HCEVT_LOG, "event-md: deserialize M_CLONE "GUIDF"\n", GUIDA(evtGuid));
                ocrGuid_t dataGuid = GET_PAYLOAD_DATA(srcBuffer, M_CLONE, ocrGuid_t, guid);
                ocrFatGuid_t fguid;
                fguid.guid = evtGuid;
                fguid.metaDataPtr = NULL;
                newEventHcDist(&fguid, dataGuid, factory);
                ocrAssert(fguid.metaDataPtr != NULL);
                *dest = fguid.metaDataPtr;
            break;
            }
            // Register another peer to our peerlist (different from an ocrAddDependence)
            // TODO: We can probably have few slot pre-allocated and extend that dynamically
            // passed the fixed size like we do for events waiters => Actually might be a nice
            // typedef struct to add.
            case M_REG: {
                DPRINTF(DBG_HCEVT_LOG, "event-md: deserialize M_REG "GUIDF"\n", GUIDA(evtGuid));
                ocrEventHc_t * devt = (ocrEventHc_t *) (*dest);
                locNode_t * locNode = (locNode_t *) pd->fcts.pdMalloc(pd, sizeof(locNode_t));
                ocrLocation_t loc = GET_PAYLOAD_DATA(srcBuffer, M_REG, ocrLocation_t, location);
                locNode->loc = loc;
                hal_lock(&(devt->waitersLock));
                //RACE-1: Check inside the lock to avoid race with satisfier. Allows
                //to determine this context is responsible for sending the M_SAT.
                bool doSatisfy = (devt->waitersCount == STATE_CHECKED_IN);
                // Registering while the event is being destroyed: Something is wrong in user code or runtime code
                ocrAssert(devt->waitersCount  != STATE_CHECKED_OUT);
                // Whether the event is already satisfied or not, we need
                // to register so that the peer is notified on 'destruct'
                locNode->next = devt->mdClass.peers;
                devt->mdClass.peers = locNode;
                hal_unlock(&(devt->waitersLock));
                if (doSatisfy) {
                    // The event is already satisfied, need to notify that back.
                    mdPushHcDist(evtGuid, loc, ((ocrEventHcPersist_t *)devt)->data, M_SAT, ((ocrEvent_t*)devt)->fctId);
                }
            break;
            }
            // Processing a satisfy notification from a peer
            case M_SAT: {
                DPRINTF(DBG_HCEVT_LOG, "event-md: deserialize M_SAT "GUIDF"\n", GUIDA(evtGuid));
                ocrEvent_t * base = (ocrEvent_t *) (*dest);
                ((ocrEventHc_t *) base)->mdClass.satFromLoc = GET_PAYLOAD_DATA(srcBuffer, M_SAT, ocrLocation_t, location);
                ocrFatGuid_t fdata;
                fdata.guid = GET_PAYLOAD_DATA(srcBuffer, M_SAT, ocrGuid_t, guid);
                fdata.metaDataPtr = NULL;
                //TODO need the slot for latch events too
                u32 slot = 0;
                factory->fcts[guidKindToEventType(guidKind)].satisfy(base, fdata, slot);
            break;
            }
            case M_DEL: {
                DPRINTF(DBG_HCEVT_LOG, "event-md: deserialize M_DEL "GUIDF"\n", GUIDA(evtGuid));
                ocrEvent_t * base = (ocrEvent_t *) (*dest);
                ((ocrEventHc_t *) base)->mdClass.delFromLoc = GET_PAYLOAD_DATA(srcBuffer, M_DEL, ocrLocation_t, location);
                factory->fcts[guidKindToEventType(guidKind)].destruct(base);
            break;
            }
        }
    } else {
        // Other event implementations are not distributed and should not end up calling here
        ocrAssert(false && "event-md: deserialize not supported for this type of event");
    }
    return 0;
}

//
// Simple Channel
//

#ifdef ENABLE_EXTENSION_CHANNEL_EVT

#ifndef DEBUG_LVL_CHANNEL
#define DEBUG_LVL_CHANNEL DEBUG_LVL_INFO
#endif

static void pushDependence(ocrEventHcChannel_t * devt, regNode_t * node) {
    //TODO rollover u32/u64
    // tail - head cannot go be more that the bound
    ocrAssert(devt->tailWaiter < (devt->headWaiter + devt->waitBufSz));
    u32 idx = devt->tailWaiter % devt->waitBufSz;
    devt->tailWaiter++;
    devt->waiters[idx] = *node;
}

static u8 popDependence(ocrEventHcChannel_t * devt, regNode_t * node) {
    if (devt->headWaiter == devt->tailWaiter) {
        return 1;
    } else {
        u32 idx = devt->headWaiter % devt->waitBufSz;
        devt->headWaiter++;
        *node = devt->waiters[idx];
        return 0;
    }
}

static void pushSatisfy(ocrEventHcChannel_t * devt, ocrGuid_t data) {
    ocrAssert(devt->tailSat < (devt->headSat + devt->satBufSz));
    u32 idx = devt->tailSat % devt->satBufSz;
    devt->tailSat++;
    devt->satBuffer[idx] = data;
}

static ocrGuid_t popSatisfy(ocrEventHcChannel_t * devt) {
    if (devt->headSat == devt->tailSat) {
        return UNINITIALIZED_GUID;
    } else {
        u32 idx = devt->headSat % devt->satBufSz;
        devt->headSat++;
        ocrGuid_t res = devt->satBuffer[idx];
        ocrAssert(!ocrGuidIsUninitialized(res));
        return res;
    }
}

#define CHANNEL_BUFFER_RESIZE(cntFct, bufName, bufSz, headName, tailName, type) \
    s32 nbElems = cntFct(devt); \
    u32 newMaxNbElems = nbElems * 2; \
    type * oldData = devt->bufName; \
    devt->bufName = (type *) pd->fcts.pdMalloc(pd, sizeof(type)*newMaxNbElems); \
    s32 headOffset = devt->headName%nbElems; \
    s32 tailOffset = devt->tailName%nbElems; \
    if ((headOffset > tailOffset) || ((headOffset == tailOffset) && (headOffset != 0))) { \
        s32 nbElemRight = (devt->bufSz - headOffset);  \
        hal_memCopy(devt->bufName, &oldData[headOffset], sizeof(type)*nbElemRight, false);  \
        hal_memCopy(&(devt->bufName[nbElemRight]), oldData, sizeof(type)*tailOffset, false);  \
    } else {  \
        hal_memCopy(devt->bufName, &oldData[headOffset], sizeof(type)*nbElems, false);  \
    }  \
    devt->headName = 0;  \
    devt->tailName = nbElems;  \
    pd->fcts.pdFree(pd, oldData);  \
    devt->bufSz = newMaxNbElems;

static bool isChannelSatisfyFull(ocrEventHcChannel_t * devt) {
    return (channelSatisfyCount(devt)  == devt->satBufSz);
}

static bool isChannelWaiterFull(ocrEventHcChannel_t * devt) {
    return (channelWaiterCount(devt) == devt->waitBufSz);
}

static void channelWaiterResize(ocrEventHcChannel_t * devt) {
    ocrPolicyDomain_t * pd;
    getCurrentEnv(&pd, NULL, NULL, NULL);
    CHANNEL_BUFFER_RESIZE(channelWaiterCount, waiters, waitBufSz, headWaiter, tailWaiter, regNode_t);
}

static void channelSatisfyResize(ocrEventHcChannel_t * devt) {
    ocrPolicyDomain_t * pd;
    getCurrentEnv(&pd, NULL, NULL, NULL);
    CHANNEL_BUFFER_RESIZE(channelSatisfyCount, satBuffer, satBufSz, headSat, tailSat, ocrGuid_t);
}

#ifdef REG_ASYNC_SGL
#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
u8 registerWaiterEventHcChannel(ocrEvent_t *base, u32 sslot, ocrFatGuid_t waiter, u32 slot, bool isDepAdd, ocrDbAccessMode_t mode) {
#else
u8 registerWaiterEventHcChannel(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot, bool isDepAdd, ocrDbAccessMode_t mode) {
#endif
#else
u8 registerWaiterEventHcChannel(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot, bool isDepAdd) {
#endif
    START_PROFILE(ev_hc_registerWaiterEventHcChannel);
    ocrEventHc_t * evt = ((ocrEventHc_t*)base);
    ocrEventHcChannel_t * devt = ((ocrEventHcChannel_t*)base);
    hal_lock(&evt->waitersLock);
    ocrGuid_t data = popSatisfy(devt);
    regNode_t regnode;
    regnode.guid = waiter.guid;
    regnode.slot = slot;
#ifdef REG_ASYNC_SGL
    regnode.mode = mode;
#endif
    if (!ocrGuidIsUninitialized(data)) {
        DPRINTF(DEBUG_LVL_CHANNEL, "registerWaiterEventHcChannel "GUIDF" push dep and deque satisfy\n",
                GUIDA(base->guid));
        hal_unlock(&evt->waitersLock);
        // We can fire the event
        ocrPolicyDomain_t *pd = NULL;
        ocrTask_t *curTask = NULL;
        PD_MSG_STACK(msg);
        getCurrentEnv(&pd, NULL, &curTask, &msg);
        ocrFatGuid_t currentEdt;
        currentEdt.guid = (curTask == NULL) ? NULL_GUID : curTask->guid;
        currentEdt.metaDataPtr = curTask;
        ocrFatGuid_t db;
        db.guid = data;
        db.metaDataPtr = NULL;
        DPRINTF(DEBUG_LVL_CHANNEL, "registerWaiterEventHcChannel satisfy edt with DB="GUIDF"\n",
                GUIDA(data));
#ifdef ALLOW_EAGER_DB
        RETURN_PROFILE(commonSatisfyRegNodeEager(pd, &msg, base->guid, db, currentEdt, &regnode));
#else
        RETURN_PROFILE(commonSatisfyRegNode(pd, &msg, base->guid, db, currentEdt, &regnode));
#endif
    } else {
        DPRINTF(DEBUG_LVL_CHANNEL, "registerWaiterEventHcChannel "GUIDF" push dependence curSize=%"PRIu32"\n",
                GUIDA(base->guid), channelWaiterCount(devt));
        if ((devt->maxGen == EVENT_CHANNEL_UNBOUNDED) && isChannelWaiterFull(devt)) {
            channelWaiterResize(devt);
        }
        pushDependence(devt, &regnode);
        hal_unlock(&evt->waitersLock);
    }
    RETURN_PROFILE(0);
}

u8 satisfyEventHcChannel(ocrEvent_t *base, ocrFatGuid_t db, u32 slot) {
    START_PROFILE(ev_hc_satisfyEventHcChannel);
    ocrEventHc_t * evt = ((ocrEventHc_t*)base);
    ocrEventHcChannel_t * devt = ((ocrEventHcChannel_t*)base);
    hal_lock(&evt->waitersLock);
    regNode_t regnode;
    u8 res = popDependence(devt, &regnode);
    if (res == 0) {
        DPRINTF(DEBUG_LVL_CHANNEL, "satisfyEventHcChannel "GUIDF" satisfy go through\n",
                GUIDA(base->guid));
        hal_unlock(&evt->waitersLock);
        // We can fire the event
        ocrPolicyDomain_t *pd = NULL;
        ocrTask_t *curTask = NULL;
        PD_MSG_STACK(msg);
        getCurrentEnv(&pd, NULL, &curTask, &msg);
        ocrFatGuid_t currentEdt;
        currentEdt.guid = (curTask == NULL) ? NULL_GUID : curTask->guid;
        currentEdt.metaDataPtr = curTask;
        DPRINTF(DEBUG_LVL_CHANNEL, "satisfyEventHcChannel satisfy edt with DB="GUIDF"\n",
                GUIDA(db.guid));
#ifdef ALLOW_EAGER_DB
        RETURN_PROFILE(commonSatisfyRegNodeEager(pd, &msg, base->guid, db, currentEdt, &regnode));
#else
        RETURN_PROFILE(commonSatisfyRegNode(pd, &msg, base->guid, db, currentEdt, &regnode));
#endif
    } else {
        DPRINTF(DEBUG_LVL_CHANNEL, "satisfyEventHcChannel "GUIDF" satisfy enqueued curSize=%"PRIu32"\n",
                GUIDA(base->guid), channelSatisfyCount(devt));
        if ((devt->maxGen == EVENT_CHANNEL_UNBOUNDED) && isChannelSatisfyFull(devt)) {
            channelSatisfyResize(devt);
        }
        pushSatisfy(devt, db.guid);
        hal_unlock(&evt->waitersLock);
    }
    RETURN_PROFILE(0);
}

#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
u8 unregisterWaiterEventHcChannel(ocrEvent_t *base, u32 sslot, ocrFatGuid_t waiter, u32 slot, bool isDepRem) {
#else
u8 unregisterWaiterEventHcChannel(ocrEvent_t *base, ocrFatGuid_t waiter, u32 slot, bool isDepRem) {
#endif
    ocrAssert(false && "Not supported");
    return 0;
}
#endif /*Channel implementation*/


/******************************************************/
/* OCR-HC Collective Events                            */
/******************************************************/

#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT

typedef struct _red_payload_t {
    u64 serSize; // serialized size
    u32 mode;
    ocrLocation_t srcLoc;
} red_payload_t;

typedef struct _up_payload_t {
    red_payload_t header;
    rph_t globalPhase;
    u64 contribSize;
    void * contribPtr;
} up_payload_t;

typedef up_payload_t down_payload_t;

// Accessors on collective's parameter data-structure

static u32 getLocalSlotId(ocrEventHcCollective_t * dself, u32 slot) {
    return slot % dself->params.nbContribsPd;
}

static u32 getMaxContribLocal(ocrEventHcCollective_t * dself) {
    return dself->params.nbContribsPd;
}

static u16 getDatumSize(ocrEventHcCollective_t * dself) {
    // 0 is 1 hence +1
    return REDOP_GET(DATUM_SIZE, dself->params.op)+1;
}

static u16 getMaxGen(ocrEventHcCollective_t * dself) {
    return dself->params.maxGen;
}

static u16 getNbDatum(ocrEventHcCollective_t * dself) {
    return dself->params.nbDatum;
}

static u32 getContribSize(ocrEventHcCollective_t * dself) {
    return (getDatumSize(dself) * getNbDatum(dself));
}

static bool isLeaf(ocrEventHcCollective_t * dself) {
    return dself->nbOfDescendants == 0;
}

static bool isRoot(ocrEventHcCollective_t * dself) {
    return dself->ancestorLoc == INVALID_LOCATION;
}

// Accessors on data-structure that depends on layout and internal organization

//  contribution counter for a given phase
static void resetPhaseContribs(ocrEventHcCollective_t * dself, rph_t ph) {
    dself->phaseLocalContribCounters[ph] = 0;
}

static bool incrAndCheckPhaseContribs(ocrEventHcCollective_t * dself, rph_t ph) {
    u32 res = hal_xadd32((&dself->phaseLocalContribCounters[ph]), 1);
    DPRINTF(DBG_COL_EVT, "phaseLocalContribCounters[%"PRIu16"]=%"PRIu32" nbContribsPd=%"PRIu32"\n", ph, res, dself->params.nbContribsPd);
    return ((res+1) == getMaxContribLocal(dself));
}

static contributor_t * getContributor(ocrEventHcCollective_t * dself, u32 lslot) {
    u16 nbDatum = getNbDatum(dself);
    u16 maxGen = getMaxGen(dself);
    u16 datumSize = getDatumSize(dself);
    u64 strideSize = (sizeof(contributor_t) + (sizeof(regNode_t) * maxGen) + (datumSize * nbDatum * maxGen));
    char * rawDst = ((char *) dself->contributors) + (strideSize * lslot);
    DPRINTF(DBG_COL_EVT, "getContributor lslot=%"PRIu32" nbDatum=%"PRIu16" maxGen=%"PRIu16" datumSize=%"PRIu16" strideSize=%"PRIu64" rawDst=%p\n", lslot, nbDatum, maxGen, datumSize, strideSize, rawDst);
    return (contributor_t *) rawDst;
}

static void * getContribPhase(ocrEventHcCollective_t * dself, u32 lslot, rph_t ph) {
    contributor_t * contributor = getContributor(dself, lslot);
    char * rawPtr = ((char *)contributor->contribs);
    u16 nbDatum = getNbDatum(dself);
    u16 datumSize = getDatumSize(dself);
    rawPtr += (datumSize * nbDatum * ph);
    return (void *) (rawPtr);
}

static regNode_t * getRegNodePhase(ocrEventHcCollective_t * dself, u32 lslot, rph_t ph) {
    contributor_t * contributor = getContributor(dself, lslot);
    return &contributor->deps[ph];
}

static rph_t getLocalPhase(ocrEventHcCollective_t * dself, rph_t gph) {
    return gph % getMaxGen(dself);
}

static rph_t getIPhase(ocrEventHcCollective_t * dself, u32 lslot) {
    contributor_t * contributor = getContributor(dself, lslot);
    return contributor->iph;
}

static rph_t getOPhase(ocrEventHcCollective_t * dself, u32 lslot) {
    contributor_t * contributor = getContributor(dself, lslot);
    return contributor->oph;
}

static rph_t incrIPhase(ocrEventHcCollective_t * dself, u32 lslot) {
    contributor_t * contributor = getContributor(dself, lslot);
    return contributor->iph++;
}

static rph_t incrOPhase(ocrEventHcCollective_t * dself, u32 lslot) {
    contributor_t * contributor = getContributor(dself, lslot);
    return contributor->oph++;
}

// Remote contribs data-structure has
static remoteContrib_t * getRemoteContrib(ocrEventHcCollective_t * dself, rph_t rph, u16 cslot) {
    char * rawDst = (((char *) dself->remoteContribs) + ((sizeof(remoteContrib_t) * getMaxGen(dself) * cslot) + rph));
    return (remoteContrib_t *) rawDst;
}

static void * fctRedResolvePayload(ocrPolicyMsg_t * msg) {
#define PD_MSG (msg)
#define PD_TYPE PD_MSG_METADATA_COMM
    return ((void *) &PD_MSG_FIELD_I(payload));
#undef PD_TYPE
#undef PD_MSG
}

static ocrGuid_t fctRedResolveGuid(ocrEventHcCollective_t * dself) {
    return ((ocrEvent_t *) dself)->guid;
}

static void fctRedInitMdCommMsg(ocrEventHcCollective_t * dself, ocrPolicyMsg_t * msg, u32 mode, u32 serSize) {
#define PD_MSG (msg)
#define PD_TYPE PD_MSG_METADATA_COMM
    msg->type = PD_MSG_METADATA_COMM | PD_MSG_REQUEST;
    PD_MSG_FIELD_I(guid) = fctRedResolveGuid(dself);
    PD_MSG_FIELD_I(direction) = MD_DIR_PUSH;
    PD_MSG_FIELD_I(op) = 0; /*ocrObjectOperation_t*/ //TODO-MD-OP not clearly defined yet
    PD_MSG_FIELD_I(mode) = mode;
    PD_MSG_FIELD_I(factoryId) = ((ocrEvent_t *) dself)->fctId;
    PD_MSG_FIELD_I(response) = NULL;
    PD_MSG_FIELD_I(mdPtr) = NULL;
    PD_MSG_FIELD_I(sizePayload) = serSize;
#undef PD_TYPE
#undef PD_MSG
}

static u8 sendMdMsg(ocrEventHcCollective_t * dself, ocrPolicyMsg_t * msg, ocrLocation_t * destinations, u16 nbDestinations) {
    u16 i = 0;
    ocrLocation_t srcLoc = dself->myLoc;
    msg->srcLocation = srcLoc;
    red_payload_t * payload = ((red_payload_t *) fctRedResolvePayload(msg));
    payload->srcLoc = srcLoc; // Make sure we correctly report this PD is the contributor
    ocrPolicyDomain_t * pd;
    getCurrentEnv(&pd, NULL, NULL, NULL);
    while (i < nbDestinations) {
        // Make sure these are up to date in case we're just forwarding the message
        ocrLocation_t dstLoc = destinations[i];
        msg->destLocation = dstLoc;
        // If last message use that pointer (PERSIST_MSG_PROP) else make a copy (0)
        u32 msgProp = (i == (nbDestinations-1)) ? PERSIST_MSG_PROP : 0;
        pd->fcts.sendMessage(pd, dstLoc, msg, NULL, msgProp);
        i++;
    }
    return (nbDestinations > 0) ? OCR_EPEND : 0;
}

static ocrFatGuid_t fctRedRecordDbPhase(ocrEventHcCollective_t * dself, rph_t lph, void * contribPtr, u64 contribSize) {
    // Copy payload to DB. Phase DBs stay acquired.
    collectiveDbRecord_t dbRecord = dself->phaseDbResult[lph];
    ocrFatGuid_t db = dbRecord.fguid;
    ocrAssert((db.metaDataPtr != NULL) && "Collective event datablock not found");
    DPRINTF(DBG_COL_EVT, "Write result to DB="GUIDF" PTR=%p size=%"PRIu64"\n", GUIDA(db.guid), dbRecord.dbBackend, contribSize);
    hal_memCopy(dbRecord.dbBackend, contribPtr, contribSize, false);
    return db;
}

static void fctRedSetDBSatisfyLocal(ocrEventHcCollective_t * dself, rph_t lph, ocrFatGuid_t db) {
    ocrTask_t *curTask = NULL;
    ocrPolicyDomain_t * pd;
    PD_MSG_STACK(satMsg);
    getCurrentEnv(&pd, NULL, &curTask, NULL);
    // Satisfy each output slot
    ocrFatGuid_t currentEdt ;
    currentEdt.guid = (curTask == NULL) ? NULL_GUID : curTask->guid;
    currentEdt.metaDataPtr = curTask;
    u16 i = 0;
    u32 nbContribsPd = getMaxContribLocal(dself);
    while (i < nbContribsPd) {
        regNode_t * rn = getRegNodePhase(dself, i, lph);
        DPRINTF(DBG_COL_EVT, "fctRedSetDBSatisfyLocal targeting waiterGuid="GUIDF" on slot=%"PRIu32" at lph=%"PRIu16" ptr=%p dself=%p\n",
                GUIDA(rn->guid), rn->slot, lph, rn, dself);
        commonSatisfyRegNode(pd, &satMsg, fctRedResolveGuid(dself), db, currentEdt, rn);
        i++;
    }
}

// Returns the remote contribution slot for the current PD on
// the remote binary reduction tree, which is always zero.
static u16 myRemoteContribSlot(ocrEventHcCollective_t * dself) {
    // if not a leaf, should compete on node 0
    ocrAssert(!isLeaf(dself))
    return dself->inOrderIdxToArrayIdx[0];
}

// Given a PD location returns a remote contribution slot
// Warning, this maps two contributions to a single slot where
// they will compete to perform the reduction
static u16 getRemoteContribSlot(ocrEventHcCollective_t * dself, ocrLocation_t * descendants, u16 count,  ocrLocation_t remoteContributor) {
    // scan the descendants list to resolve the index
    u16 i = 0;
    while (i < count) {
        if (descendants[i] == remoteContributor) {
            // Identified contributor's position
            // From that position, infer the in-order index it resolves to.
            u16 inOrderIdx = (i%2) ? i+1 : i;
            // Now that we have the inOrderIdx, divide by 2 since we only store
            // leaves in inOrderIdxToArrayIdx that maps back to a concrete array slot.
            return dself->inOrderIdxToArrayIdx[inOrderIdx/2];
        }
        i++;
    }
    return (u16) -1;
}

static u8 fctRedProcessDownMsg(ocrEventHcCollective_t * dself, ocrPolicyMsg_t * msg, down_payload_t * payload) {
    rph_t lph = payload->globalPhase % getMaxGen(dself);
    ocrFatGuid_t db = fctRedRecordDbPhase(dself, lph, payload->contribPtr, payload->contribSize);
    fctRedSetDBSatisfyLocal(dself, lph, db);
    // Forward to children if any
    if (dself->nbOfDescendants) {
        return sendMdMsg(dself, msg, dself->descendantsLoc, dself->nbOfDescendants);
    }
    return 0;
}

static void fctRedBuildInOrderIdxToArrayIdx(ocrEventHcCollective_t * dself, u16 nbOfDescendants, u16 * inOrderIdxToArrayIdx) {
    if (nbOfDescendants == 0) {
        return;
    }
    // We want one tree leaf per couple of reduction contributions.
    // Remember we reduce nbOfDescendants + self. So if nbOfDescendants is even we must add 1.
    u16 nbTreeNodes = (nbOfDescendants % 2) ? nbOfDescendants : nbOfDescendants+1;
    // Build a binary tree of that number of tree nodes
    u16 k = 2;
    u16 ktree[nbTreeNodes];
    u16 myPos = -1;
    // Mark the k-tree in-order
    markInOrder(ktree, k, nbTreeNodes, (u16) 0, &myPos);
    // Traverse the tree and find where are located each leaf
    // Leaves are all compacted at the end of the tree so we can start looking at size/2.
    k = nbTreeNodes/2;
    while (k < nbTreeNodes) {
        u16 leafInOrderIdx = ktree[k];
        u16 leafCount=leafInOrderIdx/2;
        inOrderIdxToArrayIdx[leafCount] = k;
        DPRINTF(DBG_COL_EVT, "inOrderIdxToArrayIdx=%p leafInOrderIdx=%"PRIu16" inOrderIdxToArrayIdx[%"PRIu16"]=%"PRIu16"\n", inOrderIdxToArrayIdx, leafInOrderIdx, leafCount, inOrderIdxToArrayIdx[leafCount]);
        k++;
    }
}

// Process incoming UP messages. Potentially compete with both local and descendant contributions.
// To keep things simple the current impl systematically returns EPEND and handle all
// deletions internally or implicitly through 'sendMdMsg'
u8 fctRedProcessUpMsg(ocrEventHcCollective_t * dself, ocrPolicyMsg_t * msg, up_payload_t * payload) {
    //  - if reducible now
    //      - invoke reduce f  unction(childContrib + what we have locally)
    //      - if reduced local + child i.e. all work done here
    //          - if root: generate a M_DOWN msg (can we repurpose the message here ?)
    //          - else sendUp again (can we reuse the message here ?)
    //  - else store somewhere. Are we missing storage for children contributions right now ?
    // N contributions
    ocrPolicyMsg_t * dstMsg;
#ifdef COLEVT_TREE_CONTRIB
    rph_t lph = (payload->globalPhase % getMaxGen(dself));

    // Resolve remote contribution index. This is the index in the remoteContrib array where we compete
    u16 contribSlot = (payload->header.srcLoc == dself->myLoc) ? myRemoteContribSlot(dself) :
                        getRemoteContribSlot(dself, dself->descendantsLoc, dself->nbOfDescendants, payload->header.srcLoc);
    ocrPolicyMsg_t * myMsg = msg;
    // u8 retCode = 0; // default is to systematically delete 'msg'
    // Walk the binary reduction tree
    u16 nbOfDescendants = dself->nbOfDescendants;
    ocrPolicyDomain_t * pd;
    getCurrentEnv(&pd, NULL, NULL, NULL);
    ocrPolicyMsg_t * deleteMsg =  NULL;
    //COL-EVTX: Did not implement phases: would need to get the response
    //of a phase if it comes back before a previous one.
    /* oddContribs: true when the reduction tree has a phantom right-most
     * slot (nbOfDescendants is even → nbTreeNodes = nbOfDescendants + 1).
     * Missing from the original "Collective Event Prototype" commit. */
    bool oddContribs = (nbOfDescendants > 0) && ((nbOfDescendants & 1u) == 0u);
    do {
        remoteContrib_t * rcontrib = getRemoteContrib(dself, lph, contribSlot);
        up_payload_t * myPayload = fctRedResolvePayload(myMsg);
        up_payload_t * dstPayload;
        // DPRINTF(DBG_COL_EVT, "enter fctRedProcessUpMsg at contribSlot=%"PRIu16" msg=%p phase=%"PRIu16"\n", contribSlot, msg, lph);
        bool isRightMostOdd = (oddContribs && (contribSlot == nbOfDescendants));
        if (isRightMostOdd) {// just record
            ocrAssert(rcontrib->msg == NULL);
            ocrAssert(deleteMsg == NULL);
            dstPayload = fctRedResolvePayload(myMsg);
            DPRINTF(DBG_COL_EVT, "fctRedProcessUpMsg detected rightMostOdd at contribSlot=%"PRIu16" msg=%p phase=%"PRIu16"\n", contribSlot, msg, lph);
        } else {
            if (rcontrib->msg == NULL) {
                ocrAssert(myMsg != NULL);
                void * oldV;
                oldV = (void *) hal_cmpswap64(((u64*) &(rcontrib->msg)), ((u64)NULL), myMsg);
                if (oldV == NULL) { // single contribution yet, next contributor will pick up the reduction from here
                    DPRINTF(DBG_COL_EVT, "fctRedProcessUpMsg SUCCESS CAS at contribSlot=%"PRIu16" msg=%p phase=%"PRIu16"\n", contribSlot, msg, lph);
                    return OCR_EPEND;
                } // else, failed to cas, someone went ahead, now we need to reduce
                DPRINTF(DBG_COL_EVT, "fctRedProcessUpMsg failed CAS at contribSlot=%"PRIu16" msg=%p phase=%"PRIu16"\n", contribSlot, msg, lph);
            }
            DPRINTF(DBG_COL_EVT, "fctRedProcessUpMsg do reduction at contribSlot=%"PRIu16" msg=%p phase=%"PRIu16"\n", contribSlot, msg, lph);
            ocrAssert(rcontrib->msg != NULL);
            up_payload_t * storedPayload = fctRedResolvePayload(rcontrib->msg);
            // Correct ordering
            up_payload_t * srcPayload;
            if (storedPayload->header.srcLoc < myPayload->header.srcLoc) {
                // Will reduce into the msg that's already present
                srcPayload = myPayload;
                dstPayload = storedPayload;
                // Two cases when myMsg must be freed:
                // 1: myMsg equals the 'msg' passed as parameter (i.e. must be dealing with a leaf contribution). Not returning EPEND let the caller do the free.
                //    Note: We can reduce the latency of the operation if we let the caller do the free
                // 2: dealing with nodes, must delete the src
                deleteMsg = myMsg;
                myMsg = rcontrib->msg; // for recursion
            } else {
                srcPayload = storedPayload;
                dstPayload = myPayload;
                // Here myPayload will stay so I need to return EPEND and destroy the storePayload
                // at leaf level, I would have to return EPEND since I'm storing the result inside
                // and I can destroy the other one
                deleteMsg = rcontrib->msg;
            }
            // Do the reduction
            dself->reduce(dstPayload->contribPtr, srcPayload->contribPtr, getNbDatum(dself));
            rcontrib->msg = NULL;
            //Now that we've reduced, clean-up the remote contribution entry
            if (deleteMsg != NULL) {
                // if (deleteMsg == msg) {
                //     retCode = OCR_EPEND; // avoids a double destroy
                // }
                pd->fcts.pdFree(pd, deleteMsg); //ASAN-DEL
            }
        }
        myPayload = dstPayload;
        // Set up to compete on parent slot
        DPRINTF(DBG_COL_EVT, "fctRedProcessUpMsg looping contribSlot=%"PRIu16" on parent csParent=%"PRIu16" at  msg=%p phase=%"PRIu16"\n", contribSlot,  (contribSlot-1)/2, msg, lph);
        if (contribSlot == 0) {
            DPRINTF(DBG_COL_EVT, "fctRedProcessUpMsg BREAK loop contribSlot=%"PRIu16" on parent csParent=%"PRIu16" at  msg=%p phase=%"PRIu16"\n", contribSlot,  (contribSlot-1)/2, msg, lph);
            break; // Just reduced at the root, we're done
        }
        contribSlot = (contribSlot-1)/2;
    } while (true);
    // When exiting the while loop, whatever the internal branch taken, myMsg contains the reduced answer.
    bool done = true;
    dstMsg = myMsg;
#endif
    if (done) {
        if (isRoot(dself)) {
            // Single node reduction should not have called into this function
            // but rather directly called fctRedProcessDownMsg.
            ocrAssert(dself->nbOfDescendants);
            up_payload_t * myPayload = fctRedResolvePayload(dstMsg);
            // At this point myPayload/dstMsg contain the reduced answer
            // If there's enough space, repurpose the message to go down
            u32 mdSize = sizeof(down_payload_t) + myPayload->contribSize;
            u64 msgSize = MSG_MDCOMM_SZ + mdSize;
            down_payload_t * downPayload;
            if (msgSize <= dstMsg->bufferSize) {
                // These are alias for now but if we change them we'd need to do more work here
                ocrAssert(sizeof(down_payload_t) == sizeof(up_payload_t));
                DPRINTF(DBG_COL_EVT, "M_DOWN: Repurpose up msg as a down msg\n");
#define PD_MSG (dstMsg)
#define PD_TYPE PD_MSG_METADATA_COMM
                PD_MSG_FIELD_I(mode) = M_DOWN;
#undef PD_TYPE
#undef PD_MSG
                downPayload = (down_payload_t *) fctRedResolvePayload(dstMsg);
                downPayload->header.mode = M_DOWN;
                // The msg we repurpose will be deleted by fctRedProcessDownMsg
            } else {
                DPRINTF(DBG_COL_EVT, "M_DOWN: Allocate new down msg\n");
                ocrPolicyMsg_t * newMsg = (ocrPolicyMsg_t *) allocPolicyMsg(pd, &msgSize);
                initializePolicyMessage(newMsg, msgSize);
                getCurrentEnv(NULL, NULL, NULL, newMsg);
                fctRedInitMdCommMsg(dself, newMsg, M_DOWN, mdSize);
                // copy from up to down
                downPayload = (down_payload_t *) fctRedResolvePayload(newMsg);
                downPayload->header.serSize = mdSize;
                downPayload->header.mode = M_DOWN;
                downPayload->header.srcLoc = dself->myLoc; // also set by sendMdMsg
                downPayload->globalPhase = myPayload->globalPhase;
                downPayload->contribSize = myPayload->contribSize;
                downPayload->contribPtr = (void *) (((char*)downPayload) + myPayload->contribSize);
                hal_memCopy(downPayload->contribPtr, myPayload->contribPtr, myPayload->contribSize, false);
                // dstMsg message may be 'msg' or another contributed msg. To keep things simple EPEND it here and destroy.
                pd->fcts.pdFree(pd, dstMsg);
                dstMsg = newMsg;
            }
            // Safe to ignore return code since we know the context dstMsg is used in
            fctRedProcessDownMsg(dself, dstMsg, downPayload);
        } else {
            DPRINTF(DBG_COL_EVT, "Not root, send M_UP\n");
            sendMdMsg(dself, dstMsg, &dself->ancestorLoc, 1);
        }
    }
    // if go all the way up with same msg, need to return EPEND, so that sendMsgUp does the destroy
    // return retCode;
    return OCR_EPEND;
}

// 'IN' size of PD_MSG_METADATA_COMM
#define MSG_MDCOMM_SZ       (_PD_MSG_SIZE_IN(PD_MSG_METADATA_COMM))

u8 satisfyEventHcCollective(ocrEvent_t *base, ocrFatGuid_t db, u32 slot) {
    DPRINTF(DBG_COL_EVT, "satisfyEventHcCollective invoked in slot=%"PRIu32"\n", slot);
    ocrEventHcCollective_t * dself = (ocrEventHcCollective_t *) base;
    // If associative but not commutative:
    //    - can reduce with slot's neighbors, however will need to keep track of where is the reduced value.
    //      Or use a reduction tree internally here
    // if both assoc and commu:
    //    - Can aggregate into a single buffer. Or may still want to do the same to avoid contention
    //      with everybody contributing to the same result cell. Is this similar to lazy VS eager in Jun's finish accumulator ?
    // If not associative nor commutative:
    //    - we need to store each contribution and send them all upward so that it is properly sequentially reduced at the root node
    // Note: when receiving direct contributions, not through a DB, GUID is UNINITIALIZED but ptr is valid
    u32 lslot = getLocalSlotId(dself, slot);
    void * contribPtr = db.metaDataPtr;
    //COL-EVTX: does not support DB contributions yes. For regular DBs,
    //there's currently no code that would do the acquire on this callpath.
    if (!ocrGuidIsUninitialized(db.guid)) {
        DPRINTF(DEBUG_LVL_WARN, "RedEvt: get UNINITIALIZED_GUID contribution\n");
        ocrAssert(false && "Collective Event error: does not support DB contributions\n")
    }
    ocrAssert(contribPtr != NULL);
    if (COLEVT_LAZY_REDUCE) {
        rph_t gph = incrIPhase(dself, lslot);
        rph_t lph = getLocalPhase(dself, gph);
        //NOTE: we allow to post multiple add before satisfy, hence i/o phase count may be different
        contributor_t * dbgContrib = getContributor(dself, lslot);
        DPRINTF(DBG_COL_EVT, "contributor=%p lslot=%"PRIu32" oph=%"PRIu16" iph=%"PRIu16" phaseLocalContribCounters[%"PRIu16"]=%"PRIu32" ((u64)contribPtr[0])=%"PRIu64" ((double)contribPtr[0])=%24.14E\n",
            dbgContrib, lslot, getOPhase(dself, lslot),
            getIPhase(dself, lslot), gph, dself->phaseLocalContribCounters[lph],
            (u64) ((u64*)contribPtr)[0], (double) ((double*)contribPtr)[0]);
        hal_memCopy(getContribPhase(dself, lslot, lph), contribPtr, getContribSize(dself), false);
        //Atomic incr for that phase, if last to contribute (reaches max contributions), perform the reduction
        if (incrAndCheckPhaseContribs(dself, lph)) {
            u32 i = 1, ub = getMaxContribLocal(dself);
            u16 nbDatum = getNbDatum(dself);
            void * dst = getContribPhase(dself, 0, lph);
            for(; i<ub; i++) {
                void * src = getContribPhase(dself, i, lph);
                // reduce fct ptr is already typed, just pass in the number of arguments
                dself->reduce(dst, src, nbDatum);
            }
            //NOTE: Unless when we are both root and leaf, it may make sense to always
            //create a msg and do the accumulation there and avoid copies from dst to that msg
#ifdef COLEVT_DIST_REDUCE
            if (isLeaf(dself) && isRoot(dself)) {
                DPRINTF(DBG_COL_EVT, "Fire event - single PD - gph=%"PRIu16" lph=%"PRIu16" ((u64)result[0])=%"PRIu64" ((double)result[0])=%24.14E\n", gph, lph, (u64) ((u64*)dst)[0], (double) ((double*)dst)[0]);
                ocrFatGuid_t db = fctRedRecordDbPhase(dself, lph, dst, getContribSize(dself));
                resetPhaseContribs(dself, lph);
                fctRedSetDBSatisfyLocal(dself, lph, db);
            } else { // if leaf send up, else compete with descendants for local reduction
                DPRINTF(DBG_COL_EVT, "Fire event - compete with remote PD - gph=%"PRIu16" lph=%"PRIu16"  ((u64)result[0])=%"PRIu64" ((double)result[0])=%24.14E\n", gph, lph, (u64) ((u64*)dst)[0], (double) ((double*)dst)[0]);
                // Prepare send up message
                u32 contribSize = getContribSize(dself);
                ocrPolicyMsg_t * msg = NULL;
                up_payload_t * payload;
                u32 mdSize = sizeof(up_payload_t) + contribSize;
                ocrPolicyDomain_t * pd;
                getCurrentEnv(&pd, NULL, NULL, NULL);
                u64 msgSize = MSG_MDCOMM_SZ + mdSize;
                //NOTE: we could save this allocation if we workout having
                // a null message as input to fctRedProcessUpMsg.
                msg = (ocrPolicyMsg_t *) allocPolicyMsg(pd, &msgSize);
                initializePolicyMessage(msg, msgSize);
                getCurrentEnv(NULL, NULL, NULL, msg);
                msg->type = PD_MSG_METADATA_COMM | PD_MSG_REQUEST;
                fctRedInitMdCommMsg(dself, msg, M_UP, mdSize);
                payload = ((up_payload_t *) fctRedResolvePayload(msg));
                payload->contribPtr = (void *) (((char*)payload) + sizeof(up_payload_t));
                hal_memCopy(payload->contribPtr, dst, contribSize, false);
                // Reset phase contrib counter before invoking the distributed reduction
                resetPhaseContribs(dself, lph);
                payload->header.serSize = mdSize;
                payload->header.mode = M_UP;
                payload->header.srcLoc = dself->myLoc;
                payload->globalPhase = gph;
                payload->contribSize = contribSize;
                if (isLeaf(dself)) { // just send up
                    sendMdMsg(dself, msg, &dself->ancestorLoc, 1);
                } else {
                    // Contribute an 'up' message to the current PD reduction, competing with descendants.
                    u8 retCode = fctRedProcessUpMsg(dself, msg, payload);
                    if ((retCode != OCR_EPEND) && (msg != NULL)) { // NULL check because of code currently commented
                        // Free the message as callee did not store the pointer
                        pd->fcts.pdFree(pd, msg); //ASAN-INV-READ
                    }
                }
            }
#else
            DPRINTF(DBG_COL_EVT, "Fire event at phase %"PRIu16" with result=%"PRIu64"\n", lph, (u64) ((u64*)dst)[0]);
#endif /*COLEVT_DIST_REDUCE*/
        }
    } else { //eagerly reduce
        ocrAssert(false && "COL-EVTX: implement eager collective");
    }
    return 0;
}

static u8 processEventHcCollective(ocrObjectFactory_t * factory, ocrGuid_t guid, ocrObject_t * mdPtr, ocrPolicyMsg_t * msg) {
    ocrAssert((msg->type & PD_MSG_TYPE_ONLY) == PD_MSG_METADATA_COMM);
    u8 retCode = 0;
#define PD_MSG (msg)
#define PD_TYPE PD_MSG_METADATA_COMM
    msg->type = PD_MSG_METADATA_COMM | PD_MSG_REQUEST;
    u8 direction = PD_MSG_FIELD_I(direction);
    void * payload = &PD_MSG_FIELD_I(payload);
    u64 mdMode = PD_MSG_FIELD_I(mode);
    ocrEventHcCollective_t * dself = (ocrEventHcCollective_t *) mdPtr;
    if (direction == MD_DIR_PUSH) {
        switch(mdMode) {
            case M_UP:
                //Fix up payload pointer
                ((up_payload_t*)payload)->contribPtr = ((char *) payload)+sizeof(up_payload_t);
                return fctRedProcessUpMsg(dself, msg, (up_payload_t *) payload);
            break;
            case M_DOWN:
                //Fix up payload pointer
                ((down_payload_t*)payload)->contribPtr = ((char *) payload)+sizeof(down_payload_t);
                return fctRedProcessDownMsg(dself, msg, (down_payload_t *) payload);
            break;
            case M_CLONE:
                ocrAssert(false && "COL-EVTX: implement support for cloning\n");
            break;
        }
    } else {
        ocrAssert(direction == MD_DIR_PULL);
        ocrAssert(false && "COL-EVTX: implement support for cloning\n");
    }
    return retCode;
}
#endif /*ENABLE_EXTENSION_COLLECTIVE_EVT*/

// unused attribute to workaround TG compiler complaints
static u8 processEventHc(ocrObjectFactory_t * factory, ocrGuid_t guid, ocrObject_t * mdPtr, ocrPolicyMsg_t * msg) __attribute__((unused));
static u8 processEventHc(ocrObjectFactory_t * factory, ocrGuid_t guid, ocrObject_t * mdPtr, ocrPolicyMsg_t * msg) {
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
#ifdef OCR_ASSERT
    ocrPolicyDomain_t *pd = NULL;
    getCurrentEnv(&pd, NULL, NULL, NULL);
    ocrGuidKind guidKind;
    pd->guidProviders[0]->fcts.getKind(pd->guidProviders[0], guid, &guidKind);
    ocrAssert(guidKind == OCR_GUID_EVENT_COLLECTIVE);
#endif
    return processEventHcCollective(factory, guid, mdPtr, msg);
#else
    return 0;
#endif
}

#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT

u8 registerWaiterEventHcCollective(ocrEvent_t *base, u32 sslot, ocrFatGuid_t waiter, u32 slot, bool isDepAdd, ocrDbAccessMode_t mode) {
    ocrEventHcCollective_t * dself = (ocrEventHcCollective_t *) base;
    u32 lslot = getLocalSlotId(dself, sslot);
    rph_t gph = incrOPhase(dself, lslot);
    rph_t lph = getLocalPhase(dself, gph);
    regNode_t * rn = getRegNodePhase(dself, lslot, lph);
    rn->guid = waiter.guid;
    rn->slot = slot;
    rn->mode = mode;
    DPRINTF(DBG_COL_EVT, "registerWaiterEventHcCollective invoked in dself=%p sslot=%"PRIu32" lslot=%"PRIu32" targeting waiterGuid="GUIDF" on slot=%"PRIu32" at gph=%"PRIu16" lph=%"PRIu16" ptr=%p\n",
            dself, sslot, lslot, GUIDA(waiter.guid), slot, gph, lph, rn);
    return 0;
}

u8 unregisterWaiterEventHcCollective(ocrEvent_t *base, u32 sslot, ocrFatGuid_t waiter, u32 slot, bool isDepRem) {
    ocrAssert(false && "Not supported");
    return 0;
}

#endif /*ENABLE_EXTENSION_COLLECTIVE_EVT*/

u8 mdSizeEventFactoryHc(ocrObject_t *dest, u64 mode, u64 * size) {
#if ENABLE_EVENT_MDC_FORGE
#else
    if (mode == M_CLONE) {
        ocrEventTypes_t eventType = ((ocrEvent_t *) dest)->kind;
        *size = 0;
        if((eventType == OCR_EVENT_IDEM_T) || (eventType == OCR_EVENT_STICKY_T)) {
            *size = sizeof(ocrGuid_t); // The 'data' field
        } else if(eventType == OCR_EVENT_LATCH_T) {
            *size = 0;
        } else { // OCR_EVENT_ONCE_T
            *size = 0;
        }
    }
#endif
    return 0;
}

#ifdef ENABLE_RESILIENCY
u8 getSerializationSizeEventHc(ocrEvent_t* self, u64* size) {
    ocrEventHc_t *evtHc = (ocrEventHc_t*)self;
    u32 numPeers = 0;
    locNode_t * curHead;
    for (curHead = evtHc->mdClass.peers; curHead != NULL; curHead = curHead->next)
        numPeers++;

    u64 evtSize = (evtHc->hint.hintVal ? OCR_HINT_COUNT_EVT_HC * sizeof(u64) : 0) +
                  (numPeers * sizeof(locNode_t));
    //NOTE: Waiters DB should be serialized as part of guid provider

    switch(self->kind) {
    case OCR_EVENT_ONCE_T:
        evtSize += sizeof(ocrEventHc_t);
        break;
    case OCR_EVENT_IDEM_T:
    case OCR_EVENT_STICKY_T:
        evtSize += sizeof(ocrEventHcPersist_t);
        break;
    case OCR_EVENT_LATCH_T:
        evtSize += sizeof(ocrEventHcLatch_t);
        break;
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    case OCR_EVENT_COUNTED_T:
        evtSize += sizeof(ocrEventHcCounted_t);
        break;
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    case OCR_EVENT_CHANNEL_T: {
        ocrEventHcChannel_t * evtHcChannel = (ocrEventHcChannel_t*)self;
        evtSize += sizeof(ocrEventHcChannel_t) +
                   sizeof(ocrGuid_t) * evtHcChannel->satBufSz +
                   sizeof(regNode_t) * evtHcChannel->waitBufSz;
        break;
        }
#endif
    //COL-EVTX: resiliency serialization support
    default:
        ocrAssert(0);
        break;
    }
    self->base.size = evtSize;
    *size = evtSize;
    return 0;
}

u8 serializeEventHc(ocrEvent_t* self, u8* buffer) {
    ocrAssert(buffer);
    u8* bufferHead = buffer;
    ocrEventHc_t *evtHc = (ocrEventHc_t*)self;
    ocrEventHc_t *evtHcBuf = (ocrEventHc_t*)buffer;

    //First serialize the base
    u64 len = 0;
    switch(self->kind) {
    case OCR_EVENT_ONCE_T:
        len = sizeof(ocrEventHc_t);
        break;
    case OCR_EVENT_IDEM_T:
    case OCR_EVENT_STICKY_T:
        len = sizeof(ocrEventHcPersist_t);
        break;
    case OCR_EVENT_LATCH_T:
        len = sizeof(ocrEventHcLatch_t);
        break;
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    case OCR_EVENT_COUNTED_T:
        len = sizeof(ocrEventHcCounted_t);
        break;
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    case OCR_EVENT_CHANNEL_T:
        len = sizeof(ocrEventHcChannel_t);
        break;
#endif
    //COL-EVTX: resiliency serialization support
    default:
        ocrAssert(0);
        break;
    }
    ocrAssert(len > 0);
    hal_memCopy(buffer, self, len, false);
    buffer += len;

    //Next serialize the HC event extras
    if (evtHc->hint.hintVal && OCR_HINT_COUNT_EVT_HC) {
        evtHcBuf->hint.hintVal = (u64*)buffer;
        len = OCR_HINT_COUNT_EVT_HC * sizeof(u64);
        hal_memCopy(buffer, evtHc->hint.hintVal, len, false);
        buffer += len;
    }

    if (evtHc->mdClass.peers != NULL) {
        evtHcBuf->mdClass.peers = (locNode_t*)buffer;
        locNode_t * curHead;
        len = sizeof(locNode_t);
        for (curHead = evtHc->mdClass.peers; curHead != NULL; curHead = curHead->next) {
            hal_memCopy(buffer, curHead, len, false);
            locNode_t *peerBuf = (locNode_t*)buffer;
            peerBuf->next = (curHead->next != NULL) ? (locNode_t*)(buffer + len) : NULL;
            buffer += len;
        }
    }

    //Finally serialize the derived event extras
    switch(self->kind) {
    case OCR_EVENT_ONCE_T:
    case OCR_EVENT_IDEM_T:
    case OCR_EVENT_STICKY_T:
    case OCR_EVENT_LATCH_T:
        break;
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    case OCR_EVENT_COUNTED_T:
        break;
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    case OCR_EVENT_CHANNEL_T:
        {
            ocrEventHcChannel_t * evtHcChannel = (ocrEventHcChannel_t*)self;
            ocrEventHcChannel_t * evtHcChannelBuf = (ocrEventHcChannel_t*)evtHcBuf;
            if (evtHcChannel->satBuffer) {
                evtHcChannelBuf->satBuffer = (ocrGuid_t*)buffer;
                len = sizeof(ocrGuid_t) * evtHcChannel->satBufSz;
                hal_memCopy(buffer, evtHcChannel->satBuffer, len, false);
                buffer += len;
            }

            if (evtHcChannel->waiters) {
                evtHcChannelBuf->waiters = (regNode_t*)buffer;
                len = sizeof(regNode_t) * evtHcChannel->waitBufSz;
                hal_memCopy(buffer, evtHcChannel->waiters, len, false);
                buffer += len;
            }
        }
        break;
    //COL-EVTX: resiliency serialization support
#endif
    default:
        ocrAssert(0);
        break;
    }
    ocrAssert((buffer - bufferHead) == self->base.size);
    return 0;
}

//TODO: Need to handle waitersDb ptr
u8 deserializeEventHc(u8* buffer, ocrEvent_t** self) {
    ocrAssert(self);
    ocrAssert(buffer);
    u8* bufferHead = buffer;
    ocrPolicyDomain_t *pd = NULL;
    getCurrentEnv(&pd, NULL, NULL, NULL);

    ocrEvent_t *evtBuf = (ocrEvent_t*)buffer;
    ocrEventHc_t *evtHcBuf = (ocrEventHc_t*)buffer;
    u64 len = 0;
    switch(evtBuf->kind) {
    case OCR_EVENT_ONCE_T:
        len = sizeof(ocrEventHc_t);
        break;
    case OCR_EVENT_IDEM_T:
    case OCR_EVENT_STICKY_T:
        len = sizeof(ocrEventHcPersist_t);
        break;
    case OCR_EVENT_LATCH_T:
        len = sizeof(ocrEventHcLatch_t);
        break;
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    case OCR_EVENT_COUNTED_T:
        len = sizeof(ocrEventHcCounted_t);
        break;
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    case OCR_EVENT_CHANNEL_T:
        len = sizeof(ocrEventHcChannel_t);
        break;
#endif
    //COL-EVTX: resiliency serialization support
    default:
        ocrAssert(0);
        break;
    }
    ocrAssert(len > 0);
    u64 extra = (evtHcBuf->hint.hintVal ? OCR_HINT_COUNT_EVT_HC * sizeof(u64) : 0);
    ocrEvent_t *evt = (ocrEvent_t*)pd->fcts.pdMalloc(pd, (len + extra));

    u64 offset = 0;
    hal_memCopy(evt, buffer, len, false);
    buffer += len;
    offset += len;

    ocrEventHc_t *evtHc = (ocrEventHc_t*)evt;
    if (evtHc->hint.hintVal && OCR_HINT_COUNT_EVT_HC) {
        len = OCR_HINT_COUNT_EVT_HC * sizeof(u64);
        evtHc->hint.hintVal = (u64*)((u8*)evtHc + offset);
        hal_memCopy(evtHc->hint.hintVal, buffer, len, false);
        buffer += len;
        offset += len;
    }

    if ((s32)(evtHcBuf->waitersCount) >= 0 && evtHcBuf->mdClass.peers != NULL) {
        len = sizeof(locNode_t);
        locNode_t * prevNode = NULL;
        bool doContinue = true;
        while (doContinue) {
            locNode_t * curNode = (locNode_t*)pd->fcts.pdMalloc(pd, len);
            hal_memCopy(curNode, buffer, len, false);
            curNode->next = NULL;
            if (prevNode == NULL) {
                evtHc->mdClass.peers = curNode;
            } else {
                prevNode->next = curNode;
            }
            prevNode = curNode;
            doContinue = (((locNode_t*)buffer)->next != NULL);
            buffer += len;
        }
    }

    switch(evt->kind) {
    case OCR_EVENT_ONCE_T:
    case OCR_EVENT_IDEM_T:
    case OCR_EVENT_STICKY_T:
    case OCR_EVENT_LATCH_T:
        break;
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    case OCR_EVENT_COUNTED_T:
        break;
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    case OCR_EVENT_CHANNEL_T:
        {
            ocrEventHcChannel_t * evtHcChannel = (ocrEventHcChannel_t*)evtHc;
            if (evtHcChannel->satBuffer) {
                len = sizeof(ocrGuid_t) * evtHcChannel->satBufSz;
                evtHcChannel->satBuffer = (ocrGuid_t*)pd->fcts.pdMalloc(pd, len);
                hal_memCopy(evtHcChannel->satBuffer, buffer, len, false);
                buffer += len;
            }
            if (evtHcChannel->waiters) {
                len = sizeof(regNode_t) * evtHcChannel->waitBufSz;
                evtHcChannel->waiters = (regNode_t*)pd->fcts.pdMalloc(pd, len);
                hal_memCopy(evtHcChannel->waiters, buffer, len, false);
                buffer += len;
            }
        }
        break;
#endif
    //COL-EVTX: resiliency serialization support
    default:
        ocrAssert(0);
        break;
    }

    *self = evt;
    ocrAssert((buffer - bufferHead) == (*self)->base.size);
    return 0;
}

u8 fixupEventHc(ocrEvent_t *base) {
    ocrEventHc_t *hcEvent = (ocrEventHc_t*)base;
    if (hcEvent->waitersDb.metaDataPtr != NULL) {
        ocrPolicyDomain_t *pd = NULL;
        getCurrentEnv(&pd, NULL, NULL, NULL);
        //Fixup the DB pointer
        ocrAssert(!ocrGuidIsNull(hcEvent->waitersDb.guid));
        ocrGuid_t dbGuid = hcEvent->waitersDb.guid;
        ocrObject_t * ocrObj = NULL;
        pd->guidProviders[0]->fcts.getVal(pd->guidProviders[0], dbGuid, (u64*)&ocrObj, NULL, MD_LOCAL, NULL);
        ocrAssert(ocrObj != NULL && ocrObj->kind == OCR_GUID_DB);
        ocrDataBlock_t *db = (ocrDataBlock_t*)ocrObj;
        ocrAssert(ocrGuidIsEq(dbGuid, db->guid));
        hcEvent->waitersDb.metaDataPtr = db;
    }
    return 0;
}

u8 resetEventHc(ocrEvent_t *base) {
    ocrPolicyDomain_t *pd = NULL;
    getCurrentEnv(&pd, NULL, NULL, NULL);
    pd->fcts.pdFree(pd, base);
    return 0;
}
#endif

ocrEventFactory_t * newEventFactoryHc(ocrParamList_t *perType, u32 factoryId) {
    ocrObjectFactory_t * bbase = (ocrObjectFactory_t *)
                                  runtimeChunkAlloc(sizeof(ocrEventFactoryHc_t), PERSISTENT_CHUNK);
    bbase->clone = FUNC_ADDR(u8 (*)(ocrObjectFactory_t * factory, ocrGuid_t guid, ocrObject_t**, ocrLocation_t, u32), cloneEventFactoryHc);
    bbase->serialize = FUNC_ADDR(u8 (*)(ocrObjectFactory_t * factory, ocrGuid_t guid, ocrObject_t*, u64*, ocrLocation_t, void**, u64*),
        serializeEventFactoryHc);
    bbase->deserialize = FUNC_ADDR(u8 (*)(ocrObjectFactory_t * factory, ocrGuid_t guid, ocrObject_t**, u64, void*, u64),
        deserializeEventFactoryHc);
    bbase->mdSize = FUNC_ADDR(u8 (*)(ocrObject_t * dest, u64, u64*), mdSizeEventFactoryHc);
    bbase->process = FUNC_ADDR(u8 (*)(ocrObjectFactory_t * factory, ocrGuid_t guid, ocrObject_t*, /*MD-COMM*/ocrPolicyMsg_t * msg), processEventHc);

    ocrEventFactory_t *base = (ocrEventFactory_t*) bbase;
    base->instantiate = FUNC_ADDR(u8 (*)(ocrEventFactory_t*, ocrFatGuid_t*,
                                  ocrEventTypes_t, u32, ocrParamList_t*), newEventHc);
    base->base.destruct =  FUNC_ADDR(void (*)(ocrObjectFactory_t*), destructEventFactoryHc);

    // Initialize the base's base
    // For now, we keep it NULL. This is just a placeholder
    base->base.fcts.processEvent = NULL;

    // Initialize the function pointers

    // Setup common functions
    base->commonFcts.setHint = FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrHint_t*), setHintEventHc);
    base->commonFcts.getHint = FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrHint_t*), getHintEventHc);
    base->commonFcts.getRuntimeHint = FUNC_ADDR(ocrRuntimeHint_t* (*)(ocrEvent_t*), getRuntimeHintEventHc);
#ifdef ENABLE_RESILIENCY
    base->commonFcts.getSerializationSize = FUNC_ADDR(u8 (*)(ocrEvent_t*, u64*), getSerializationSizeEventHc);
    base->commonFcts.serialize = FUNC_ADDR(u8 (*)(ocrEvent_t*, u8*), serializeEventHc);
    base->commonFcts.deserialize = FUNC_ADDR(u8 (*)(u8*, ocrEvent_t**), deserializeEventHc);
    base->commonFcts.fixup = FUNC_ADDR(u8 (*)(ocrEvent_t*), fixupEventHc);
    base->commonFcts.reset = FUNC_ADDR(u8 (*)(ocrEvent_t*), resetEventHc);
#endif

    // Setup functions properly
    u32 i;
    for(i = 0; i < (u32)OCR_EVENT_T_MAX; ++i) {
        base->fcts[i].destruct = FUNC_ADDR(u8 (*)(ocrEvent_t*), destructEventHc);
        base->fcts[i].get = FUNC_ADDR(ocrFatGuid_t (*)(ocrEvent_t*), getEventHc);
#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
#define REGISTERSIGNALER_SIG u8 (*)(ocrEvent_t*, u32, ocrFatGuid_t, u32, ocrDbAccessMode_t, bool)
#define UNREGISTERSIGNALER_SIG u8 (*)(ocrEvent_t*, u32, ocrFatGuid_t, u32, bool)
#else
#define REGISTERSIGNALER_SIG u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32, ocrDbAccessMode_t, bool)
#define UNREGISTERSIGNALER_SIG u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32, bool)
#endif
        base->fcts[i].registerSignaler = FUNC_ADDR(REGISTERSIGNALER_SIG, registerSignalerHc);
        base->fcts[i].unregisterSignaler = FUNC_ADDR(UNREGISTERSIGNALER_SIG, unregisterSignalerHc);
    }
    base->fcts[OCR_EVENT_STICKY_T].destruct =
    base->fcts[OCR_EVENT_IDEM_T].destruct = FUNC_ADDR(u8 (*)(ocrEvent_t*), destructEventHcPersist);

    // Setup satisfy function pointers
    base->fcts[OCR_EVENT_ONCE_T].satisfy =
        FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32), satisfyEventHcOnce);
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    base->fcts[OCR_EVENT_COUNTED_T].satisfy =
        FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32), satisfyEventHcCounted);
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    base->fcts[OCR_EVENT_CHANNEL_T].satisfy =
        FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32), satisfyEventHcChannel);
#endif
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
    base->fcts[OCR_EVENT_COLLECTIVE_T].satisfy =
        FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32), satisfyEventHcCollective);
#endif
    base->fcts[OCR_EVENT_LATCH_T].satisfy =
        FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32), satisfyEventHcLatch);
    base->fcts[OCR_EVENT_IDEM_T].satisfy =
        FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32), satisfyEventHcPersistIdem);
    base->fcts[OCR_EVENT_STICKY_T].satisfy =
        FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32), satisfyEventHcPersistSticky);
#ifdef REG_ASYNC_SGL
#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
#define REGISTER_WAITER_SIG() u8 (*)(ocrEvent_t*, u32, ocrFatGuid_t, u32, bool, ocrDbAccessMode_t)
#else
#define REGISTER_WAITER_SIG() u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32, bool, ocrDbAccessMode_t)
#endif
    // Setup registration function pointers
    base->fcts[OCR_EVENT_ONCE_T].registerWaiter =
    base->fcts[OCR_EVENT_LATCH_T].registerWaiter =
         FUNC_ADDR(REGISTER_WAITER_SIG(), registerWaiterEventHc);
    base->fcts[OCR_EVENT_IDEM_T].registerWaiter =
    base->fcts[OCR_EVENT_STICKY_T].registerWaiter =
        FUNC_ADDR(REGISTER_WAITER_SIG(), registerWaiterEventHcPersist);
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    base->fcts[OCR_EVENT_COUNTED_T].registerWaiter =
        FUNC_ADDR(REGISTER_WAITER_SIG(), registerWaiterEventHcCounted);
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    base->fcts[OCR_EVENT_CHANNEL_T].registerWaiter =
        FUNC_ADDR(REGISTER_WAITER_SIG(), registerWaiterEventHcChannel);
#endif
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
    base->fcts[OCR_EVENT_COLLECTIVE_T].registerWaiter =
        FUNC_ADDR(REGISTER_WAITER_SIG(), registerWaiterEventHcCollective);
#endif
#else /*not REG_ASYNC_SGL**/
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
    #error REG_ASYNC_SGL must be enabled when ENABLE_EXTENSION_COLLECTIVE_EVT is
#endif
    base->fcts[OCR_EVENT_ONCE_T].registerWaiter =
    base->fcts[OCR_EVENT_LATCH_T].registerWaiter =
         FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32, bool), registerWaiterEventHc);
    base->fcts[OCR_EVENT_IDEM_T].registerWaiter =
    base->fcts[OCR_EVENT_STICKY_T].registerWaiter =
        FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32, bool), registerWaiterEventHcPersist);
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    base->fcts[OCR_EVENT_COUNTED_T].registerWaiter =
        FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32, bool), registerWaiterEventHcCounted);
#endif
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    base->fcts[OCR_EVENT_CHANNEL_T].registerWaiter =
        FUNC_ADDR(u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32, bool), registerWaiterEventHcChannel);
#endif
#endif
#ifdef ENABLE_EXTENSION_MULTI_OUTPUT_SLOT
#define UNREGISTERWAITER_SIG u8 (*)(ocrEvent_t*, u32, ocrFatGuid_t, u32, bool)
#else
#define UNREGISTERWAITER_SIG u8 (*)(ocrEvent_t*, ocrFatGuid_t, u32, bool)
#endif
    base->fcts[OCR_EVENT_ONCE_T].unregisterWaiter =
    base->fcts[OCR_EVENT_LATCH_T].unregisterWaiter =
        FUNC_ADDR(UNREGISTERWAITER_SIG, unregisterWaiterEventHc);
    base->fcts[OCR_EVENT_IDEM_T].unregisterWaiter =
#ifdef ENABLE_EXTENSION_COUNTED_EVT
    base->fcts[OCR_EVENT_COUNTED_T].unregisterWaiter =
#endif
    base->fcts[OCR_EVENT_STICKY_T].unregisterWaiter =
        FUNC_ADDR(UNREGISTERWAITER_SIG, unregisterWaiterEventHcPersist);
#ifdef ENABLE_EXTENSION_CHANNEL_EVT
    base->fcts[OCR_EVENT_CHANNEL_T].unregisterWaiter =
        FUNC_ADDR(UNREGISTERWAITER_SIG, unregisterWaiterEventHcChannel);
#endif
#ifdef ENABLE_EXTENSION_COLLECTIVE_EVT
    base->fcts[OCR_EVENT_COLLECTIVE_T].unregisterWaiter =
        FUNC_ADDR(UNREGISTERWAITER_SIG, unregisterWaiterEventHcCollective);
#endif
    base->factoryId = factoryId;

    //Setup hint framework
    base->hintPropMap = (u64*)runtimeChunkAlloc(sizeof(u64)*(OCR_HINT_EVT_PROP_END - OCR_HINT_EVT_PROP_START - 1), PERSISTENT_CHUNK);
    OCR_HINT_SETUP(base->hintPropMap, ocrHintPropEventHc, OCR_HINT_COUNT_EVT_HC, OCR_HINT_EVT_PROP_START, OCR_HINT_EVT_PROP_END);
    return base;
}
#endif /* ENABLE_EVENT_HC */
