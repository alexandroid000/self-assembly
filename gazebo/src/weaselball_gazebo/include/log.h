#ifndef _LOG_H_
#define _LOG_H_
//-----------------------------------------------------------------------------

#include <boost/shared_ptr.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

//-----------------------------------------------------------------------------
class log_c;
//-----------------------------------------------------------------------------
/// Alias a shared pointer for the log_c class
typedef boost::shared_ptr<log_c> log_ptr;

//-----------------------------------------------------------------------------
/**
Class to write log data to disk
**/
class log_c {
private:
  std::string   _name;		// file name
  std::ofstream _file;		// file stream
public:
  /// Parameterized constructor that requires the name of the log file 
  /// @param name the name of the log file
  log_c( std::string name ) { _name = name; }
  /// Destructor - automatically closes an open log file
  virtual ~log_c( void ) { close(); }

  /// Opens the file designated through the constructor
  /// @return returns true if the log file was successfully opened; otherwise,
  ///         returns false
  bool open( void ) {
    _file.open( _name.c_str(), std::ios::out | std::ios::trunc );
    if( !_file.is_open() ) return false;
    return true;
  }

  /// Closes the file stream
  void close( void ) {
    if( _file.is_open() ) _file.close();
  }
  
  /// Writes a string to an open file stream
  /// @param data the string to write to the log file
  /// @return returns true if the string was successfully written to the file
  ///         stream; otherwise, returns false.
  bool write( std::string data ) {
    if( !_file.is_open() ) return false;
    _file << data;
    return true;
  }
};

//-----------------------------------------------------------------------------
#endif // _LOG_H_
