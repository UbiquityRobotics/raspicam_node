#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"

namespace mmal
{
// Calls mmal_component_destroy, but returns void

struct component_deleter {
  void operator()(MMAL_COMPONENT_T* ptr) const {
    if (ptr != nullptr) {
      for (size_t i = 0; i < ptr->output_num; ++i) {
        if (ptr->output[i] && ptr->output[i]->is_enabled) {
          mmal_port_disable(ptr->output[i]);
        }
      }
      mmal_component_disable(ptr);
      mmal_component_destroy(ptr);
    }
  }
};
typedef std::unique_ptr<MMAL_COMPONENT_T, component_deleter> component_ptr;

struct connection_deleter {
  void operator()(MMAL_CONNECTION_T* ptr) const {
    if (ptr != nullptr) {
      mmal_connection_destroy(ptr);
    }
  }
};
typedef std::unique_ptr<MMAL_CONNECTION_T, connection_deleter> connection_ptr;

void default_delete_pool(MMAL_POOL_T* ptr) {
  if (ptr != nullptr) {
    fprintf(stderr, "%s\n", "LEAKED POOL! you need to define your own deleter");
  }
}
typedef std::unique_ptr<MMAL_POOL_T, std::function<void(MMAL_POOL_T*)>> pool_ptr;


// Standard ports for the camera component
struct camera_port
{
  // We can't use an enum class here, we rely on implictly converting to an int
  // which is not allowed for an enum class
  enum {
    preview = 0,
    video = 1,
    capture = 2
  };
};

} 