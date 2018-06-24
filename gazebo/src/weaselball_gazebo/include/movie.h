/*-----------------------------------------------------------------------------
Defines logic to allow movie generation from weazelball data in Gazebo
IPC is used to maximize synchronization between rendering system and the 
simulation so that frame rates are stabilized and a minimum number of 
output frames are dropped.  Synchronization is between the rendering plugin
and the contollers.

James R. Taylor (jrt@gwu.edu)
-----------------------------------------------------------------------------*/
#ifndef _MOVIE_H_
#define _MOVIE_H_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <assert.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <boost/shared_ptr.hpp>

//#define OUTPUT_FPS 10
//double frameduration;
//double prevframe;
//double nextframe;

class render_synchronization_buffer_c;
typedef boost::shared_ptr<render_synchronization_buffer_c> render_synchronization_buffer_ptr;

#define RENDER_SYNCH_CHANNEL  "wbcam"
struct camdata_t {
  pthread_mutex_t      mutex;
  gazebo::math::Pose   pose;
  int                  state;
  int                  rendering;
};

class render_synchronization_buffer_c {
private:
  camdata_t*           _camdata;
  int                  _shmfd;

public:
  /// Constructor - Default
  render_synchronization_buffer_c( void ) {
    _camdata = NULL;
    _shmfd = -1;
  }

  /// Destructor
  virtual ~render_synchronization_buffer_c( void ) {
    close( _shmfd );

    if( munmap( (void*)_camdata, sizeof(camdata_t) ) ) {
      std::cerr << "munmap( _camdata )" << std::endl;
    }
    if( shm_unlink( RENDER_SYNCH_CHANNEL ) != 0 ) {
      std::cerr << "shm_unlink()" << std::endl;
    }
  }

  // open the shared camera shared memory data buffer
  int open( void ) {
    return open( NULL );
  }

  // open the shared camera shared memory data buffer and pass an initial pose
  // of the tracked subject into the buffer
  int open( gazebo::math::Pose* initial_pose ) {
    void*                   addr;
    pthread_mutexattr_t     attr;

     _shmfd = shm_open( RENDER_SYNCH_CHANNEL, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
    if( _shmfd == -1 ) {
      std::cerr << "ERROR(movie.h):shm_open()" << std::endl;
      return 1;
    }
    if( ftruncate( _shmfd, sizeof(camdata_t) ) == -1 ) {
      std::cerr << "ftruncate()" << std::endl;
      return 2;
    }
    addr = mmap( NULL, sizeof(camdata_t), PROT_READ | PROT_WRITE, MAP_SHARED, _shmfd, 0 );
    if( addr == MAP_FAILED ) {
      std::cerr << "mmap( _camdata )" << std::endl;
      close( _shmfd );
      return 3;
    }
    _camdata = static_cast<camdata_t*> ( addr );
  
    pthread_mutexattr_init( &attr );
    pthread_mutexattr_setpshared( &attr, PTHREAD_PROCESS_SHARED );
  
    if( pthread_mutex_init( &_camdata->mutex, &attr ) != 0 ) {
      // what other handling should be done in response.  Close the fd?
      std::cerr << "pthread_mutex_init()" << std::endl;
      return 4;
    }
    pthread_mutexattr_destroy( &attr );
  
    if( msync( _camdata, sizeof(pthread_mutex_t), MS_SYNC | MS_INVALIDATE ) != 0 ) {
      // what other handling should be done in response.  Close the fd, release the mutex?
      std::cerr << "msync()" << std::endl;
      return 5;
    }
  
    pthread_mutex_lock( &_camdata->mutex );
    if( initial_pose != NULL ) {
      _camdata->pose = *initial_pose;
    }
    _camdata->state = 1;
    _camdata->rendering = 0;
    pthread_mutex_unlock( &_camdata->mutex );

    // success
    return 0;
  }

  void synchronize( void ) {
    synchronize( NULL );
  }

  void synchronize( gazebo::math::Pose* current_pose ) {
    bool updated = false;
    while( !updated ) {
       pthread_mutex_lock( &_camdata->mutex );
       if( _camdata->state == 0 ) {
         //printf( "c:0\n" );
         updated = true;
       }
       pthread_mutex_unlock( &_camdata->mutex );
    }

    pthread_mutex_lock( &_camdata->mutex );
    if( current_pose != NULL ) {
      _camdata->pose = *current_pose;
    }
    _camdata->state = 1;
    pthread_mutex_unlock( &_camdata->mutex );
    //printf( "c:1\n" );
  }

  void yield_to_render( void ) {
    int slp = 0;
    pthread_mutex_lock( &_camdata->mutex );
    if( _camdata->rendering == 1 ) {
      slp = 1;
    }
    pthread_mutex_unlock( &_camdata->mutex );
    if(slp) usleep( 100000 );
  }
};

#endif  // _MOVIE_H_
