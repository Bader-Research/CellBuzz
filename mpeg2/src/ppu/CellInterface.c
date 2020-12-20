/* IBM CELL: This file provides a clean interface for launching jobs
 * on the SPEs from the PPE
 * Various code is commented out from attempting to write to the SPE
 * mailboxes directly (this cause issues)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libmisc.h>
/* SDK 1.2 => SDK 2.0 */
#if 1
#include <pthread.h>
#include <libspe2.h>
#else
#include <libspe.h>
#endif
//#include <cbe_mfc.h>

#include "config.h"
#include "global.h"

static int nextJob = 0;
/* SDK 1.2 => SDK 2.0 */
#if 1
typedef struct _ppe_pthread_data_t {
        spe_context_ptr_t speid;
        pthread_t pthread;
        void* argp;
} ppe_pthread_data_t;

ppe_pthread_data_t* p_pthread_data;
#else
static speid_t *spuIDs;
#endif

static void* ppe_pthread_function( void* p_arg );
//static spe_spu_control_area_t **mbox_ps_area;
//static struct JobCompleteField *spuMarkers;
//static int *currentJobs;
static int nextSPU;

extern spe_program_handle_t mpeg2decode_spu;

static inline int get_in_mbox_status(int index)
{
  //return _spe_in_mbox_status(mbox_ps_area[index]);
/* SDK 1.2 => SDK 2.0 */
#if 1
  return spe_in_mbox_status( p_pthread_data[index].speid );
#else
  return spe_stat_in_mbox(spuIDs[index]);
#endif
}

static inline void in_mbox_write(int index, int msg)
{
/* SDK 1.2 => SDK 2.0 */
#if 1
  unsigned int mbox_data;

  mbox_data = msg;
  spe_in_mbox_write( p_pthread_data[index].speid, &mbox_data, 1, SPE_MBOX_ANY_NONBLOCKING );
#else
  //_spe_in_mbox_write(mbox_ps_area[index], msg);
  spe_write_in_mbox(spuIDs[index], msg);
#endif
}

void InitCellInterface(void)
{
/* SDK 1.2 => SDK 2.0 */
#if 1
  if( numSPUs > spe_cpu_info_get( SPE_COUNT_PHYSICAL_SPES, -1 ) )
#else
  if( numSPUs > spe_count_physical_spes() )
#endif
  {
/* SDK 1.2 => SDK 2.0 */
#if 1
    fprintf(stderr, "There are only %d SPEs; requested %d\n", spe_cpu_info_get( SPE_COUNT_PHYSICAL_SPES, -1 ), numSPUs);
#else
    fprintf(stderr, "There are only %d SPEs; requested %d\n", spe_count_physical_spes(), numSPUs);
#endif
    exit(1);
  }

  printf("Using %d SPEs\n", numSPUs);

  int i;

/* SDK 1.2 => SDK 2.0 */
#if 1
  p_pthread_data = ( ppe_pthread_data_t* )malloc( sizeof( ppe_pthread_data_t ) * numSPUs );
#else
  spuIDs = (speid_t*)malloc(sizeof(speid_t) * numSPUs);
  memset(spuIDs, 0, sizeof(speid_t) * numSPUs);
#endif
  //mbox_ps_area = (spe_spu_control_area_t**)malloc(sizeof(spe_spu_control_area_t*) * numSPUs);
  //spuMarkers = (struct JobCompleteField*)malloc_align(sizeof(struct JobCompleteField)*numSPUs, 4);
  //currentJobs = (int*)malloc(sizeof(int)*numSPUs);

  spuJobs = (struct spuJobData*)malloc_align(sizeof(struct spuJobData)*mb_height, 4);

  for( i = 0; i < numSPUs; ++i )
  {
/* SDK 1.2 => SDK 2.0 */
#if 1
    int ret;

    p_pthread_data[i].speid = spe_context_create( 0, NULL );
    if( p_pthread_data[i].speid == NULL ) {
      printf("Failed creating thread\n");
      exit(1);
    }

    ret = spe_program_load( p_pthread_data[i].speid, &mpeg2decode_spu );
    if( ret != 0 ) {
      printf("Failed creating thread\n");
      exit(1);
    }

    p_pthread_data[i].argp = ( void* )i;

    if( pthread_create( &( p_pthread_data[i].pthread ), NULL, &ppe_pthread_function, &( p_pthread_data[i] ) ) != 0 ) {
      printf("Failed creating thread\n");
      exit(1);
    }
#else
    spuIDs[i] = spe_create_thread( SPE_DEF_GRP, &mpeg2decode_spu, 
				   i,
				   NULL, 
				   -1, 
				   0);//SPE_MAP_PS );
    if( spuIDs[i] == NULL )
    {
      printf("Failed creating thread\n");
      exit(1);
    }
#endif

    //mbox_ps_area[i] = spe_get_ps_area(spuIDs[i], SPE_CONTROL_AREA);
    //spuMarkers[i].data[0] = -1;
  }

  nextSPU = 0;
}

void CloseCellInterface(void)
{
  int i;

  for( i = 0; i < numSPUs; ++i )
  {
    in_mbox_write(i, SPU_OP_EXIT);
  }
  
  for( i = 0; i < numSPUs; ++i )
  {
/* SDK 1.2 => SDK 2.0 */
#if 1
    if( pthread_join( p_pthread_data[i].pthread, NULL ) != 0 ) {
      printf( "Failed pthread join\n" );
      exit( 1 );
    }

    if( spe_context_destroy( p_pthread_data[i].speid ) != 0 ) {
      printf( "Failed spe_context_destroy\n" );
      exit( 1 );
    }
#else
    spe_wait(spuIDs[i], NULL, 0);
#endif
  }
}

void CellSync(void)
{
  int i;

  while( nextJob < mb_height )
  {
    CheckAndSend();
  }

  for( i = 0; i < numSPUs; ++i )
  {
    in_mbox_write(i, SPU_OP_SYNC);
  }

  for( i = 0; i < numSPUs; ++i )
  {
    while( get_in_mbox_status(i) != 4 );
  }
  
  /* Cleanup from this decode cycle */
  for( i = 0; i < mb_height; ++i )
  {
    free_align(spuJobs[i].bitstreamBuffer);
    //memset(spuJobs, 0, sizeof(struct spuJobData) * mb_height);
  }
  nextJob = 0;
  nextSPU = 0;
}

void CheckAndSend(void)
{
  if( get_in_mbox_status(nextSPU) == 4 )
  {
    in_mbox_write(nextSPU, SPU_OP_WORK);
    in_mbox_write(nextSPU, &(spuJobs[nextJob++]));

    nextSPU = (nextSPU + 1) % numSPUs;

    return;
  }

  int i;
  for( i = 0; i < numSPUs; ++i )
  {
    if( nextSPU != i && 
	nextJob < mb_height && 
	get_in_mbox_status(i) == 4 )
    {
      // push a job
      in_mbox_write(i, SPU_OP_WORK);
      in_mbox_write(i, &(spuJobs[nextJob++]));
      nextSPU = (i + 1) % numSPUs;
      break;
    }
  }
}

/* SDK 1.2 => SDK 2.0 */
#if 1
static void* ppe_pthread_function( void* p_arg ) {
        ppe_pthread_data_t* p_data = ( ppe_pthread_data_t* )p_arg;
        unsigned int entry;

        entry = SPE_DEFAULT_ENTRY;

        if( spe_context_run( p_data->speid, &entry, 0, p_data->argp, NULL, NULL ) < 0 ) {
                printf( "Failed spe_context_run\n" );
                pthread_exit( ( void* )-1 );
        }

        pthread_exit( NULL );
}
#endif

