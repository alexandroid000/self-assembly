#ifndef _STATE_H_
#define _STATE_H_
//-----------------------------------------------------------------------------

#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>

//-----------------------------------------------------------------------------
class state_c;
//-----------------------------------------------------------------------------
typedef boost::shared_ptr<state_c> state_ptr;
//-----------------------------------------------------------------------------
/**
Base class to manage state data.
**/
class state_c {
protected:
  unsigned _size;             //< number of elements in state
  std::vector<double> _x;     //< state vector
  double _t;                  //< time

public:

  state_c( unsigned size ) {
    _size = size;
    _t = 0.0;
    _x.assign( _size, 0.0 );
  }
  virtual ~state_c( void ) {}

public:
  unsigned size( void ) const {
    return _size;
  }

  double t( void ) {
    return _t;
  }

  void t( double t_ ) {
    _t = t_;
  }

  void t( int min, int sec, int ms ) {
    _t = (double)min * 60.0 + (double)sec + (double)ms/1000.0;
  }

  double& val( unsigned i ) { 
    assert( i < _size );
    return _x[i];
  }

  const double& val( unsigned i ) const { 
    assert( i < _size );
    return _x[i];
  }

  double& operator[]( unsigned i ) { 
    assert( i < _size );
    return _x[i];
  }

  const double& operator[]( unsigned i ) const { 
    assert( i < _size );
    return _x[i];
  }

  void print( std::string label ) {
    printf( "%s{ t[%f] x{", label.c_str(), _t );
    for( unsigned i = 0; i < _size; i++ ) {
      if( i > 0 ) printf( ", " );
      printf( "%f", _x[i] );
    }
    printf( "}}" );
  }
};

//-----------------------------------------------------------------------------
#endif // _STATE_H_
