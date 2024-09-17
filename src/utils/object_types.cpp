#include "salsa/utils/object_types.h"

#ifdef __GNUG__
#include <cxxabi.h>

#include <cstdlib>
#include <memory>
#include <string>
namespace salsa {
std::string demangle(const char* name) {
  int status = -1;
  const std::unique_ptr<char, void (*)(void*)> res{
      abi::__cxa_demangle(name, nullptr, nullptr, &status), std::free};

  return (status == 0) ? std::string(res.get()) : std::string(name);
}
}  // namespace salsa
#else
namespace salsa {
std::string demangle(const char* name) { return std::string(name); }
}  // namespace salsa
#endif  // __GNUG__