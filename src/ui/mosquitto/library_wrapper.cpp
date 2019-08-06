#include "library_wrapper.h"
#include "mosquitto.h"

namespace Mosquitto {

LibraryWrapper::LibraryWrapper()
{
    mosquitto_lib_init();
}

LibraryWrapper::~LibraryWrapper()
{
    mosquitto_lib_cleanup();
}

}
