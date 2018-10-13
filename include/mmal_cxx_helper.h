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
void component_destroy_v(MMAL_COMPONENT_T* ptr) {
  if (ptr != nullptr) {
    mmal_component_destroy(ptr);
  }
}

typedef std::unique_ptr<MMAL_COMPONENT_T, decltype(&component_destroy_v)> component_ptr;

// Calls mmal_component_destroy, but returns void
void connection_destroy_v(MMAL_CONNECTION_T* ptr) {
  if (ptr != nullptr) {
    mmal_connection_destroy(ptr);
  }
}

typedef std::unique_ptr<MMAL_CONNECTION_T, decltype(&connection_destroy_v)> connection_ptr;
}