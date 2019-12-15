

#ifndef SRC_UTILS_MEM_UTILS_H_
#define SRC_UTILS_MEM_UTILS_H_

#include <memory>

namespace depth_clustering {

namespace mem_utils {
using std::unique_ptr;

template <typename T, typename... Args>
unique_ptr<T> make_unique(Args&&... args) {
  return unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace mem_utils

}  // namespace depth_clustering

#endif  // SRC_UTILS_MEM_UTILS_H_
