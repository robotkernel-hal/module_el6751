//! robotkernel example module
/*!
  $Id$
 */

#include "robotkernel/kernel.h"
#include "module_el6751.h"
#include "el6751.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

using namespace robotkernel;
using namespace std;

static string mod_name = "";

//! log to kernel logging facility
void el6751_log(robotkernel::loglevel lvl, const char *format, ...) {
    char buf[1024];

    // format argument list
    va_list args;
    va_start(args, format);
    vsnprintf(buf, 1024, format, args);
    klog(lvl, "[module_el6751|%s] %s", mod_name.c_str(), buf);
}

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

//! cyclic process data read
/*!
  \param hdl module handle
  \param buf process data buffer 
  \param bufsize size of process data buffer
  \return size of read bytes
 */
size_t mod_read(MODULE_HANDLE hdl, void* buf, size_t bufsize) {
    beckhoff::el6751 *el6751 = (beckhoff::el6751 *)hdl;
    if (!el6751) {
        errno = EINVAL;
        return -1;
    }

    return el6751->read(buf, bufsize);
}

//! cyclic process data write
/*!
  \param hdl module handle
  \param buf process data buffer
  \param bufsize size of process data buffer 
  \return size of written bytes
 */
size_t mod_write(MODULE_HANDLE hdl, void* buf, size_t bufsize) {
    beckhoff::el6751 *el6751 = (beckhoff::el6751 *)hdl;
    if (!el6751) {
        errno = EINVAL;
        return -1;
    }

    return el6751->write(buf, bufsize);
}

//! configures module
/*!
  \param name module name
  \param config configure string
  \return handle on success, NULL otherwise
*/
MODULE_HANDLE mod_configure(const char* name, const char* config) {
    beckhoff::el6751 *el6751;
    mod_name = string(name);

    // open config
    std::stringstream stream(config);
    YAML::Parser parser(stream);
    YAML::Node doc;

    el6751_log(info, "build by: %s@%s\n", BUILD_USER, BUILD_HOST);
    el6751_log(info, "build date: %s\n", BUILD_DATE);

    if (!parser.GetNextDocument(doc)) {
        el6751_log(error, "parsing config file\n");
        return (MODULE_HANDLE)NULL;
    }
    
    el6751 = new beckhoff::el6751(name, doc);
    if (!el6751) {
        el6751_log(error, "cannot allocate memory");
        return (MODULE_HANDLE)NULL;
    }

    return (MODULE_HANDLE)el6751;
}

//! unconfigure module
/*!
  \param hdl module handle
  \return success or failure
 */
int mod_unconfigure(MODULE_HANDLE hdl) {
    beckhoff::el6751 *el6751 = (beckhoff::el6751 *)hdl;
    if (!el6751) {
        errno = EINVAL;
        return -1;
    }

    delete el6751;
    return 0;
}

//! set module state machine to defined state
/*!
  \param hdl module handle
  \param state requested state
  \return success or failure
 */
int mod_set_state(MODULE_HANDLE hdl, module_state_t state) {
    beckhoff::el6751 *el6751 = (beckhoff::el6751 *)hdl;
    if (!el6751) {
        errno = EINVAL;
        return -1;
    }

    return el6751->set_state(state);
}

//! get module state machine state
/*!
  \param hdl module handle
  \return current state
 */
module_state_t mod_get_state(MODULE_HANDLE hdl) {
    beckhoff::el6751 *el6751 = (beckhoff::el6751 *)hdl;
    if (!el6751) {
        errno = EINVAL;
        return module_state_unknown;
    }

    return el6751->get_state();
}

//! send a request to module
/*!
  \param hdl module handle
  \param reqcode request code
  \param ptr pointer to request structure
  \return success or failure
 */
int mod_request(MODULE_HANDLE hdl, int reqcode, void* ptr) {
    beckhoff::el6751 *el6751 = (beckhoff::el6751 *)hdl;
    if (!el6751) {
        errno = EINVAL;
        return -1;
    }

    return el6751->request(reqcode, ptr);
}

//! module trigger callback
/*!
 * \param hdl module handle
 */
void mod_trigger(MODULE_HANDLE hdl) {
    beckhoff::el6751 *el6751 = (beckhoff::el6751 *)hdl;
    if (!el6751) {
        errno = EINVAL;
        return;
    }

    el6751->trigger();
}

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

