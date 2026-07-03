/**
 * @brief Simple data-block implementation.
 **/

/*
 * This file is subject to the license agreement located in the file LICENSE
 * and cannot be distributed without it. This notice cannot be
 * removed or modified.
 */

#ifndef __DATABLOCK_LOCKABLE_H__
#define __DATABLOCK_LOCKABLE_H__

#include "ocr-config.h"
#ifdef ENABLE_DATABLOCK_LOCKABLE

#include "ocr-allocator.h"
#include "ocr-datablock.h"
#include "ocr-hal.h"
#include "ocr-types.h"
#include "utils/ocr-utils.h"
#include "ocr-worker.h"
#include "utils/queue.h"

#ifdef ENABLE_HINTS
/**< The number of hint properties supported by this implementation */
#define OCR_HINT_COUNT_DB_LOCKABLE   3
#else
#define OCR_HINT_COUNT_DB_LOCKABLE   0
#endif

typedef struct {
    ocrDataBlockFactory_t base;
} ocrDataBlockFactoryLockable_t;

typedef union {
    struct {
        u64 state      : 2;   // Current state of the DB
        u64 dbMode     : 2;   // Current DB mode
        u64 hasPeers   : 1;   // MD has peer instances
        u64 writeBack  : 1;   // Shall writeback on release
        u64 isFetching : 1;   // Is currently fetching remotely
        u64 isReleasing : 1;  // Is currently releasing remotely
        u64 isEager    : 1;   // Is an eager DB
        u64 isLazy    : 1;    // Is a lazy DB
        u64 flags      : 16;  // From the object's creation
        u64 numUsers   : 15;  // Number of consumers checked-in
        u64 freeRequested: 1; // dbDestroy has been called
        u64 singleAssign : 1; // Single assignment done
        u64 _padding   : 21;
    };
    u64 data;
} ocrDataBlockLockableAttr_t;

// Declared in .c
struct dbWaiter_t;
struct _ocrPolicyMsg_t;

/**
 * A payload buffer displaced by an incoming write-back while local acquirers
 * still held pointers into it. Pointers handed out by acquire remain valid
 * until the acquirer releases, so a superseding write-back must not free the
 * previous buffer while the user count is non-zero: it is parked on this list
 * and reclaimed once the count drains to zero. Multiple write-backs can stack
 * entries if the count never reaches zero in between.
 */
typedef struct _dbDisplacedBuf_t {
    void * ptr;                      /**< buffer to reclaim */
    bool isMsgBuf;                   /**< true: policy-message buffer (pdFree);
                                          false: plain data payload (MEM_UNALLOC) */
    struct _dbDisplacedBuf_t * next;
} dbDisplacedBuf_t;

// Must match DB_* definitions in the implementation
#define DB_MODE_COUNT   4

// Tracker for MD copies
#define DB_MAX_LOC (1<<GUID_PROVIDER_LOCID_SIZE)
#define DB_MAX_LOC_ARRAY ((DB_MAX_LOC/64)+1)

// DB Stats
#ifdef DB_STATS_LOCKABLE
#define CNT_LOCAL_RELEASE   0
#define CNT_REMOTE_RELEASE  1
#define CNT_LOCAL_ACQUIRE   2
#define CNT_REMOTE_ACQUIRE  3
#define CNT_EAGER_CLONE     4
#define CNT_EAGER_PULL      5
#define CNT_MAX             6

typedef struct {
    u64 counters[CNT_MAX];
} dbLockableStats_t;
#endif

/**
 * @brief Datablock implementation based on locking.
 *
 * The lock approach allows coherent distributed read-write by serializing
 * accesses to the datablock across locations, while allowing intra-location
 * parallelism for the grantee. The implementation allows distributed reads
 * to process in parallel on any location.
 *
 *
 * EAGER DB hint:
 *
 * The implementation also supports the EAGER datablock hint.
 *
 * The hint is meant to convey that when an event is satisfied with the DB, the DAG
 * structure provides enough guarantees that the DB can be eagerly delivered to
 * its consumer EDTs. In other words, the runtime does not need to ensure its coherence.
 *
 * The hint is 'unsafe' meaning that when incorrectly used, it may affects the correctness
 * of the application.
 * To be correct, the execution of the producer EDT that writes to the EAGER DB and the
 * consumer EDT that reads MUST be properly ordered in the DAG. When done iteratively,
 * the user must properly ensure the consumer EDT at phase 'i' is done executing when
 * the producer EDT at phase 'i+1' executes. Otherwise there's a risk of the DB of
 * phase 'i' be erased by phase 'i+1' while the consumer is still using it.
 *
 * Rules:
 * - Producer EDT (writer) and Consumer EDT (reader) are properly ordered
 * - Consumer EDT must NOT write to the DB. No coherence enforced !
 * - The satisfaction of an event with an EAGER DB is always asynchronous
 *
 * BKMS:
 * - The channel-event should be co-located with the consumer EDT so that the addDependence call
 *   is local. The satisfaction will be asynchronous, bundling both the satisfaction message and
 *   the datablock payload. On reception the eager DB current data is discarded and replaced with
 *   the incoming payload.
 * - It is better to keep reusing a datablock instance rather than keep creating/destroying new
 *   ones to avoid extra overheads.
 *
 * Limitations (reason):
 * - The EAGER hint is only honored when satisfying a channel event
 *   (avoids checking the hint for all events)
 * - The channel-event being satisfied with an eager DB must have a single dependence
 *   (just did not impl for multiple recipient, check nonCoherentLoc and waitersCount)
 *
 */
typedef struct _ocrDataBlockLockable_t {
    ocrDataBlock_t base;
    lock_t lock; /**< Lock for this data-block */
    ocrDataBlockLockableAttr_t attributes; /**< Attributes for this data-block */
    struct _dbWaiter_t * localWaitQueues[DB_MODE_COUNT]; /** Per mode local waiters queued to acquire the db */
    Queue_t * remoteWaitQueues[DB_MODE_COUNT]; /** Per mode remote PD waiters queued to acquire the db */
    struct _ocrPolicyMsg_t * backingPtrMsg; /** Pointer to a policy message that stores the DB's data */
    dbDisplacedBuf_t * displacedBufs; /** Write-back-displaced payload buffers awaiting user drain */
    // mdPeers: If the MD instance is the master one it is the PD that owns the guid. Else it is
    // the PD location to write the datablock back to.
    ocrLocation_t mdPeers;
    ocrWorker_t * worker; /**< worker currently owning the DB internal lock */
#ifdef ENABLE_RESILIENCY
    // Stores number of local and remote waiters count in queue for serialization.
    // Don't really need the remote one since it's encoded in the queue...
    u32 waiterQueueCounters[DB_MODE_COUNT*2];
#endif
    u64 mdLocTracker[DB_MAX_LOC_ARRAY]; /**< Tracker for MD copies */
#ifdef DB_STATS_LOCKABLE
    dbLockableStats_t stats; /**< Datablock statistics */
#endif
    ocrLocation_t nonCoherentLoc; /**< Whether or not coherence must be kept */
#ifdef ENABLE_LAZY_DB
    //TODO-LAZY: Limitation: currently track a single location
    ocrLocation_t lazyLoc; /**< Location that's currently owning the DB in lazy mode */
#endif
    ocrRuntimeHint_t hint; // Warning must be the last
} ocrDataBlockLockable_t;

extern ocrDataBlockFactory_t* newDataBlockFactoryLockable(ocrParamList_t *perType, u32 factoryId);

#endif /* ENABLE_DATABLOCK_LOCKABLE */
#endif /* __DATABLOCK_LOCKABLE_H__ */
