/*
 * This file is subject to the license agreement located in the file LICENSE
 * and cannot be distributed without it. This notice cannot be
 * removed or modified.
 */


#include "debug.h"
#include "ocr-policy-domain.h"
#include "ocr-sal.h"
#include "ocr-types.h"

#define DEBUG_TYPE API

/* Shutdown drain flag.  When non-zero, the runtime is tearing down
 * across PDs: outgoing blocking request-response messages other than
 * the runlevel notification itself are dropped at send time so callers
 * do not wait indefinitely for a response from a peer that is also
 * draining.  Set on shutdown initiation and on RL_NOTIFY receipt. */
volatile u8 gOcrShutdownDraining = 0;

static void ocrShutdownInternal(u8 errorCode) {
    DPRINTF(DEBUG_LVL_INFO, "ENTER ocrShutdown()\n");
    gOcrShutdownDraining = 1;
    ocrPolicyDomain_t *pd = NULL;
    PD_MSG_STACK(msg);
    ocrPolicyMsg_t * msgPtr = &msg;
    ocrTask_t * curTask;
    getCurrentEnv(&pd, NULL, &curTask, msgPtr);
#define PD_MSG msgPtr
#define PD_TYPE PD_MSG_MGT_RL_NOTIFY
    msgPtr->type = PD_MSG_MGT_RL_NOTIFY | PD_MSG_REQUEST;
#ifdef ENABLE_OCR_API_DEFERRABLE
    if (!errorCode) {
        tagDeferredMsg(msgPtr, curTask);
    }
#endif
    PD_MSG_FIELD_I(runlevel) = RL_COMPUTE_OK;
    PD_MSG_FIELD_I(properties) = RL_REQUEST | RL_BARRIER | RL_TEAR_DOWN;
    PD_MSG_FIELD_I(errorCode) = errorCode;
    u8 returnCode __attribute__((unused)) = pd->fcts.processMessage(pd, msgPtr, true);
    ocrAssert((returnCode == 0));
#undef PD_MSG
#undef PD_TYPE
}

void ocrShutdown() {
    START_PROFILE(api_ocrShutdown);
    ocrShutdownInternal(0);
    RETURN_PROFILE();
}

void ocrAbort(u8 errorCode) {
    START_PROFILE(api_ocrAbort);
    ocrShutdownInternal(errorCode);
    RETURN_PROFILE();
}

u64 ocrGetArgc(void* dbPtr) {
    START_PROFILE(api_ocrGetArgc);
    DPRINTF(DEBUG_LVL_INFO, "ENTER ocrGetArgc(dbPtr=%p)\n", dbPtr);
    DPRINTF(DEBUG_LVL_INFO, "EXIT ocrGetArgc -> %"PRIu64"\n", ((u64*)dbPtr)[0]);
    RETURN_PROFILE(((u64*)dbPtr)[0]);

}

u64 getArgc(void* dbPtr) {
    ocrPrintf("getArgc is deprecated as of OCR v1.2.0... use ocrGetArgc\n");
    return ocrGetArgc(dbPtr);
}

char* ocrGetArgv(void* dbPtr, u64 count) {
    START_PROFILE(api_ocrGetArgv);
    DPRINTF(DEBUG_LVL_INFO, "ENTER ocrGetArgv(dbPtr=%p, count=%"PRIu64")\n", dbPtr, count);
    u64* dbPtrAsU64 = (u64*)dbPtr;
    ocrAssert(count < dbPtrAsU64[0]); // We can't ask for more args than total
    u64 offset = dbPtrAsU64[1 + count];
    DPRINTF(DEBUG_LVL_INFO, "EXIT ocrGetArgv -> %s\n", ((char*)dbPtr) + offset);
    RETURN_PROFILE(((char*)dbPtr) + offset);
}

char* getArgv(void* dbPtr, u64 count) {
    ocrPrintf("getArgv is deprecated as of OCR v1.2.0... use ocrGetArgv\n");
    return ocrGetArgv(dbPtr, count);
}


