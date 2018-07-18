#ifndef _VIRTUAL_TIME_H_
#define _VIRTUAL_TIME_H_

class virtual_time_c {
private:
  double _t;
  double _step;
public:
  virtual_time_c( double step ) { _step = step; }
  virtual ~virtual_time_c( void ) {}

  double time( void ) { return _t; }

  unsigned to_frame_number( unsigned fps ) {
    
  }

  unsigned to_frame_time( unsigned fps ) {

  }

  double from_frame_number( unsigned frame, unsigned fps ) {

  }

  double from_frame_time( double frame_time, unsigned fps ) {

  }

  double increment( void ) {
    return _t += _step;
  }
};

#endif // _VIRTUAL_TIME_H_
