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

}