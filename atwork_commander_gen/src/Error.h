#include <stdexcept>
#include <iostream>
#include <sstream>

void errorOut(std::ostream& os, const char* v) {
  os << " " << v;
}

template<typename T>
void errorOut(std::ostream& os, T v) {
  os << " " << v;
}

template<typename T>
void errorPass(std::ostream& os, T first) {
  errorOut(os, first);
}

template<typename T, typename... Args>
void errorPass(std::ostream& os, T first, Args... args) {
  errorOut(os, first);
  errorPass(os, args...);
}

template<typename... Args>
void errorImpl(const char* file, size_t line, const char* func, const char* msg, Args... args ) {
  std::ostringstream os;
  os << "Location: " << file << ":" << line << std::endl << "Function: " << func << std::endl << "Message: " << msg << std::endl;
  errorPass(os, args...);
  throw std::runtime_error( os.str() );
}

#define error( msg, ... )  errorImpl(__FILE__, __LINE__, __PRETTY_FUNCTION__, msg, __VA_ARGS__ );
