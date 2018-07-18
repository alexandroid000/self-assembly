#ifndef _VIDEO_H_
#define _VIDEO_H_

//-----------------------------------------------------------------------------

#include "common.h"
#include "state.h"
#include <vector>
#include <string>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

//-----------------------------------------------------------------------------
// Forward declarations 
class wb_video_session_c;
class wb_video_event_c;
class wb_video_data_c;
//-----------------------------------------------------------------------------
// Pointer aliases 
typedef boost::shared_ptr<wb_video_session_c> wb_video_session_ptr;
typedef boost::shared_ptr<wb_video_event_c> wb_video_event_ptr;
typedef boost::shared_ptr<wb_video_data_c> wb_video_data_ptr;
//-----------------------------------------------------------------------------
/**
An instance of a wb_video_event_c represents a point in the video where either
a signal from the weazelball was detected or a known gap of signals were lost.
This class is designed to parse the signal data defined in each trial file 
located in the data/interpreted/signals directory.  Each entry in the signal 
data is either a gap or a frame_id with accompanying qualifiers.  The qualifiers
act as notes for the user.  Many of the signals cannot be directly observed
due to the LED being located on one hemisphere of the robot; however, the LED
is bright enough that light projected from it can be viewed reflecting in the
environment.  The velocity of the motor assembly was dependent on the charge
in the battery and on the movement of the robot, so in various cases the LED
signal would span two frames of video which is also annotated in the file. 
*/
class wb_video_event_c {
private:
  // The union of the these two groups
  // -1-
  bool _is_gap;             //< signal may not be observable but may still exist
  // -2-
  int _frame_id;            //< initial video frame where a signal was observed
  bool _is_reflection;      //< signal was observed indirectly
  bool _spans_two_frames;   //< signal was observed in this frame and the next
  // ---

  double _duration;            //< appoximate duration of the signal event
  double _virtual_start_time;  //< approximate virtual time when the event began
  double _virtual_end_time;    //< approximate virtual time when the event ended

  unsigned _idx;               //< index of the event

public:
  wb_video_event_c( unsigned idx ) {
    _idx = idx;
    _is_gap = true;
    _frame_id = -1;
    _is_reflection = false;
    _spans_two_frames = false;
    _duration = 0.0;
    _virtual_start_time = 0.0;
    _virtual_end_time = 0.0;
  }

  wb_video_event_c( unsigned idx, int frame_id ) {
    _idx = idx;
    _is_gap = false;
    _frame_id = frame_id;
    _is_reflection = false;
    _spans_two_frames = false;
    _duration = 0.0;
    _virtual_start_time = 0.0;
    _virtual_end_time = 0.0;
  }

  virtual ~wb_video_event_c( void ) {}

  // Getters
  bool is_gap( void ) { return _is_gap; }
  int frame_id( void ) { return _frame_id; }
  bool is_reflection( void ) { return _is_reflection; }
  bool spans_two_frames( void ) { return _spans_two_frames; }
  double duration( void ) { return _duration; }
  double virtual_start_time( void ) { return _virtual_start_time; }
  double virtual_end_time( void ) { return _virtual_end_time; }
  unsigned index( void ) { return _idx; }

  // Deserialize/Parse the event from the string read from the file stream
  static wb_video_event_ptr deserialize( unsigned idx, std::string str ) {
    wb_video_event_ptr event;
    unsigned count = 0;
    std::string first, second;
    boost::char_separator<char> delim( " " );
    boost::tokenizer< boost::char_separator<char> > tokenizer( str, delim );
    BOOST_FOREACH( const std::string& token, tokenizer ) {
      count++;
      if( count == 1 ) {
        // the first token is either a '-' or a frame number
        // a '-' indicates that this is an unobservable gap in signals
        // a number indicates a signal was detected
        first = token;
        if( first.find( '-' ) != std::string::npos ) {
          event = wb_video_event_ptr( new wb_video_event_c( idx ) );
          return event;
        }
        event = wb_video_event_ptr( new wb_video_event_c( idx, atoi(first.c_str()) ) );
      } else {
        // a second token is a set of flags
        // '*' indicates that this signal was identified by environmental info
        // '+' indicates that this signal spans the current frame and the next
        second = token;
        if( second.find( '*' ) != std::string::npos ) {
          event->_is_reflection = true;
        }
        if( second.find( '+' ) != std::string::npos ) {
          event->_spans_two_frames = true;
        }
      }
    }
    return event;
  }
 
  /// Compute the time for the signal event. 
  /// @param frame_rate the frame rate of the video camera
  /// @param base_video_synch_frame a clearly identifiable and unambiguous 
  ///        frame in the video at which the video data aligns with the vicon
  ///        data.  This is most clearly defined by a collision event.
  /// @param base_virtual_synch_time the virtual time (computed by incrementing
  ///        at the vicon sampling rate) that coincides with the same collision
  ///        event observed in the vicon data.  The alignment of these two
  ///        events allows time to be synchronized between data sets.
  //void compute_time( double frame_rate, unsigned base_video_synch_frame, double base_virtual_synch_time ) {
  void compute_time( double frame_rate, double dropped_video_time ) {
    // if there is no clear signal during a set of frames ignore
    if( _is_gap ) return;

    // compute the duration of a single LED event and an accompanying factor
    // used to scale that length by a percentage
    _duration = BASE_VIDEO_LED_DURATION;
    double factor = BASE_VIDEO_LED_DURATION * VIDEO_LED_TIME_FACTOR/100.0;
    _duration += factor;

    // compute the virtual time at which the event was registered in the video
    // based on the frame number.  For events that span two frames, the first
    // frame of the event is recorded and the second is assumed to follow.
    //double virtual_frame_time = (_frame_id - (int)base_video_synch_frame) * 1.0/frame_rate + base_virtual_synch_time;
    double virtual_frame_time = ((double)_frame_id) * 1.0/frame_rate - dropped_video_time;

    // compute the start time of the event
    if( !_spans_two_frames ) {
      // if the event occured only on a single frame then the duration will
      // straddle that frame evenly
      _virtual_start_time = virtual_frame_time - _duration / 2.0;
    } else {
      // if the event spanned two frames then the duration will straddle both
      // frames and the factor will be used to inflate time so that the event
      // is not so discrete that it falls at or between the frames
      _virtual_start_time = virtual_frame_time - factor / 2.0;
    }
    // compute the end time of the event
    _virtual_end_time = _virtual_start_time + _duration;
  }
};
//-----------------------------------------------------------------------------
/**
A video session is the set of all events for a given trial along with the other
synchronization and statistical information attached to the trial.
*/
class wb_video_session_c {
private:
  unsigned _base_video_synch_frame;  //< video frame to synchronize with vicon
  double _base_vicon_synch_time;     //< vicon time to synchronize with video
  double _base_virtual_synch_time;   //< virtual (vicon) time to synch at 
  unsigned _total_frames;            //< total number of frames of video
  double _frame_rate;                //< frame rate of video
  unsigned _base_safe_start_frame;   //< video frame where all data stable
  double _vicon_sample_rate;         //< sampling rate of the vicon system
  unsigned _vicon_states_to_ignore;  //< number of initial vicon states before _base_safe_start_frame
  double _dropped_video_time;        //< virtual time dropped before _base_safe_start_frame

public:
  wb_video_session_c( void ) {}
  virtual ~wb_video_session_c( void ) {}

  std::vector<wb_video_event_ptr> events;  //< set of video events for session
  
  // Getters
  unsigned base_video_synch_frame( void ) { return _base_video_synch_frame; }
  double base_vicon_synch_time( void ) { return _base_vicon_synch_time; }
  double base_virtual_synch_time( void ) { return _base_virtual_synch_time; }
  unsigned total_frames( void ) { return _total_frames; }
  double frame_rate( void ) { return _frame_rate; }
  unsigned base_safe_start_frame( void ) { return _base_safe_start_frame; }
  double vicon_sample_rate( void ) { return _vicon_sample_rate; }
  unsigned vicon_states_to_ignore( void ) { return _vicon_states_to_ignore; }
  double dropped_video_time( void ) { return _dropped_video_time; }

  // Reads the signal data from the corresponding indexed file into this data
  // structure.
  bool read( unsigned session_id ) {
    assert( session_id > 0 && session_id <= EXPERIMENTAL_SESSIONS );
    std::string path = VIDEO_DATA_PATH;
    std::stringstream ssfile;
    ssfile << path << "trial_" << std::setfill('0') << std::setw(2) << session_id << ".txt";
    printf( "reading file: %s\n", ssfile.str().c_str() );
    if( !read( ssfile.str() ) )
      printf( "failed to read file: %s\n", ssfile.str().c_str() );
    printf( "video events loaded: %u\n", events.size() );

    double vicon_t0 = vicon_start_time();
    double vicon_steps_before_t0 = vicon_t0 / (1.0/_vicon_sample_rate);
    _vicon_states_to_ignore = (unsigned) vicon_steps_before_t0;
    _dropped_video_time = _base_safe_start_frame * 1.0 / (double)_frame_rate;

    // statistics
    unsigned gaps = 0, reflections = 0, two_frames = 0;
    unsigned start = 0, end = 0, count = 0;
    unsigned best_start = 0, best_end = 0, best_count = 0;
    for( std::vector<wb_video_event_ptr>::iterator it = events.begin(); it != events.end(); it++ ) {
      unsigned id = (*it)->frame_id();
      if( (*it)->is_gap() ) { 
        gaps++;
        if(count) {
          if( count > best_count ) {
            best_start = start;
            best_end = end;
            best_count = count;
          }
          count = 0;
        }
      } else {
        if( !count ) {
          start = id;
        }
        end = id;
        count++;
      }
      if( (*it)->is_reflection() ) { reflections++; }
      if( (*it)->spans_two_frames() ) { two_frames++; }
    }
    double t = ((double)best_count) / _frame_rate;
    printf( "signal events[%u], longest_continguous_interval[%d, %d] : sz[%d] : video_duration[%f], gaps[%d], reflections[%d], two_frames[%d]\n", events.size(), best_start, best_end, best_count, t, gaps, reflections, two_frames );

  }

  // Converts virtual time into a frame number depending on the synchronization
  // data computed for this file as computed by read(session_id) above
  unsigned frame( double virtual_time ) {
    return (unsigned)((virtual_time + _dropped_video_time) * _frame_rate);
  }

  // Gets the synchronized virtual start time of the vicon data based on the 
  // synchronization data stored in the trial data
  double vicon_start_time( void ) {
    double video_dt = 1.0 / (double)_frame_rate;
    unsigned frames_to_keep = _base_video_synch_frame - _base_safe_start_frame;
    double video_delta_to_t0 = video_dt * (double)(frames_to_keep - 1);
    double vicon_t0 = _base_vicon_synch_time - video_delta_to_t0;

    printf( "frames_to_keep[%d], video_delta_to_t0[%f], vicon_t0[%f]\n", frames_to_keep, video_delta_to_t0, vicon_t0 );
    return vicon_t0;
  }

private:
  // reads the file contents of a trial file into this class instance
  bool read( std::string filename ) {
    std::string data;
    std::ifstream file( filename.c_str() );
    if( !file.is_open() ) return false;

    int line = 0;
    int comments = 0;
    while( std::getline( file, data ) ) {
      if( data.find('#') == 0 ) {
        comments++;
        continue;
      }
      line++;
      if( line == 1 ) {
        _base_video_synch_frame = atoi( data.c_str() );
      } else if( line == 2 ) {
        _base_vicon_synch_time = atof( data.c_str() );
      } else if( line == 3 ) {
        _base_virtual_synch_time = atof( data.c_str() );
      } else if( line == 4 ) {
        _total_frames = atoi( data.c_str() );
      } else if( line == 5 ) {
        _frame_rate = atof( data.c_str() );
      } else if( line == 6 ) {
        _base_safe_start_frame = atoi( data.c_str() );
        _dropped_video_time = _base_safe_start_frame * 1.0 / (double)_frame_rate;
      } else if( line == 7 ) {
        _vicon_sample_rate = atof( data.c_str() );
      } else {
        wb_video_event_ptr event;
        event = wb_video_event_c::deserialize( events.size(), data );
        if( !event ) { continue; }

        //event->compute_time( _frame_rate, _base_video_synch_frame, _base_virtual_synch_time );
        event->compute_time( _frame_rate, _dropped_video_time );
        events.push_back( event );
      }
    }
    return true;
  }
};
/*
//-----------------------------------------------------------------------------
class wb_video_data_c {
public:
  std::vector< std::vector<wb_video_state_ptr> > sessions;

  wb_video_data_c( void ) {
    sessions.resize( EXPERIMENTAL_SESSIONS );
  }

  virtual ~wb_video_data_c( void ) {}

  bool load_video( void ) {
    std::string path = VIDEO_DATA_PATH;

    for( unsigned i = 0; i < EXPERIMENTAL_SESSIONS; i++ ) {
      std::stringstream ssfile;
      unsigned fileid = i + 1;
      ssfile << path << "trial_" << std::setfill('0') << std::setw(2) << fileid << ".txt";
      printf( "reading file: %s\n", ssfile.str().c_str() );
      if( !read_vicon( ssfile.str(), states[i] ) )
        printf( "failed to read file: %s\n", ssfile.str().c_str() );
      printf( "states loaded: %u\n", states[i].size() );
    }
  }


  bool read_vicon( std::string filename, std::vector<wb_vicon_state_ptr>& states ) {
    std::string data;
    std::ifstream file( filename.c_str() );
    if( !file.is_open() ) return false;

    int min, sec, ms;
    double pos[3];
    double rot[4];

    int line = 0;
    while( std::getline( file, data ) ) {
      int idx = line++ % 4;
      if( idx == 0 ) {
        //timexxx: min:sec:ms [ex: time(min:sec:ms): 30:23:971]
        //prefix is fixed length so will only parse beginning at space
        size_t start = 18, end;
        std::string values = data.substr( start );
        end = values.find( ":" );
        min = atoi( values.substr( 0, end ).c_str() );
        start = end + 1;
        end = values.find( ":", start );
        sec = atoi( values.substr( start, end ).c_str() );
        start = end + 1;
        ms = atoi( values.substr( start ).c_str() );
      } else if( idx == 1 ) {
        //pos: x, y, z [ex: pos: -0.242842, 1.39527, 1.35857]
        //prefix is fixed length so will only parse beginning at first space
        size_t start = 5, end;
        std::string values = data.substr( start );
        end = values.find( "," );
        pos[0] = atof( values.substr( 0, end ).c_str() );
        start = end + 1;
        end = values.find( ",", start );
        pos[1] = atof( values.substr( start, end ).c_str() );
        start = end + 1;
        pos[2] = atof( values.substr( start ).c_str() );
      } else if( idx == 2 ) {
        //quat: x, y, z, w [ex. quat: 0.817646, -0.535734, 0.145617, -0.15245]
        //prefix is fixed length so will only parse beginning at first space
        size_t start = 6, end;
        std::string values = data.substr( start );
        end = values.find( "," );
        rot[0] = atof( values.substr( 0, end ).c_str() );
        start = end + 1;
        end = values.find( ",", start );
        rot[1] = atof( values.substr( start, end ).c_str() );
        start = end + 1;
        end = values.find( ",", start );
        rot[2] = atof( values.substr( start, end ).c_str() );
        start = end + 1;
        rot[3] = atof( values.substr( start ).c_str() );
      } else if( idx == 3 ) {
        // blank line
        // retain the record
 
        //std::cout << "(" << min << ":" << sec << ":" << ms << "):";
        //std::cout << "[" << pos[0] << "," << pos[1] << "," << pos[2] << ",";
        //std::cout << rot[0] << "," << rot[1] << "," << rot[2] << "," << rot[3] << "]" << std::endl;

        wb_vicon_state_ptr state = wb_vicon_state_ptr( new wb_vicon_state_c() );

        state->t( min, sec, ms );
        for( unsigned i = 0; i < 3; i++ )
          state->val(i) = pos[i];
        for( unsigned i = 0; i < 4; i++ )
          state->val(i+3) = rot[i];
        states.push_back( state );

        //state->print();
        //printf( "\n" );
      }

      //std::cout << data << std::endl;

    }
    file.close();

    return true;
  }

};
*/

#endif // _VIDEO_H_
