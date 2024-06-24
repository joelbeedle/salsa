#include "utils/object_types.h"

#ifdef __GNUG__
#include <cxxabi.h>

#include <cstdlib>
#include <memory>

namespace swarm {
std::string demangle(const char* name) {
  int status = -1;
  std::unique_ptr<char, void (*)(void*)> res{
      abi::__cxa_demangle(name, NULL, NULL, &status), std::free};

  return (status == 0) ? res.get() : name;
}
}  // namespace swarm
#else
namespace swarm {
std::string demangle(const char* name) { return name; }
}  // namespace swarm
#endif  // __GNUG__