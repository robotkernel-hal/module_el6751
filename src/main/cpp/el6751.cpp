//! robotkernel module schunk el6751
/*!
 * author: Robert Burger <robert.burger@dlr.de>
 */

/*
 * This file is part of robotkernel.
 *
 * robotkernel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * robotkernel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with robotkernel.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "el6751.h"
#include "robotkernel/exceptions.h"
#include "robotkernel/helpers.h"
#include <iomanip>
#include <stdio.h>
#include <string.h>

#include <string_util/string_util.h>

MODULE_DEF(module_el6751, beckhoff::el6751)

using namespace std;
using namespace robotkernel;
using namespace string_util;
using namespace beckhoff;
        
/*

config:
    ec_module: soem_master
    ec_slave_id: 12
    slave_modules: [ pg70_1, pg70_2, ]
 
*/

//! construction
/*!
 * \param node yaml intialization node
 */
el6751::el6751(const std::string& name, const YAML::Node& node) :
    module_base("module_el6751", name, node),
    ::trigger(name, "el6751")
{
    pd_inputs_device  = get_as<string>(node, "pd_inputs_device");
    pd_outputs_device = get_as<string>(node, "pd_outputs_device");


    if (node["slave_modules"]) {
        // parsing slave configurations
        const YAML::Node& slave_modules = node["slave_modules"];
        for (YAML::const_iterator it = slave_modules.begin();
                it != slave_modules.end(); ++it) {
            
            std::string mod_name = it->as<std::string>();
            slave_module_names.push_back(mod_name); 
        }
    }

    memset(&local_can_interface, 0, sizeof(can_interface_t));
}

//! destruction 
el6751::~el6751() {
    set_state(module_state_init);
}

        
//! set module state machine to defined state
/*!
  \param state requested state
  \return success or failure
  */
int el6751::set_state(module_state_t state) {            
    kernel& k = *kernel::get_instance();

    // get transition
    uint32_t transition = GEN_STATE(this->state, state);

    switch (transition) {
        case op_2_safeop:
        case op_2_preop:
        case op_2_init:
        case op_2_boot:
            // ====> stop sending commands
            if (state == module_state_safeop)
                break;
        case safeop_2_preop:
        case safeop_2_init:
        case safeop_2_boot:
            // ====> stop receiving measurements
            k.remove_device(shared_from_this());

            if (state == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
            slaves.clear();
        case init_2_init:
            // ====> do nothing
            if (state == module_state_init)
                break;
        case init_2_boot:
            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> do nothing
            if (state == module_state_init)
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop:
            // ====> get device modules
            for (auto it = slave_module_names.begin(); it != slave_module_names.end(); ++it) {
                sp_module_t m = k.get_module((*it).c_str());

                if (!m)
                    throw str_exception("[module_el6751] module not found %s\n", it->c_str());

                slaves.push_back(m);
            }

            if (state == module_state_preop)
                break;
        case preop_2_op:
        case preop_2_safeop: {
            // ====> get el6751 process data
            el6751_pdin  = k.get_process_data(pd_inputs_device);
            el6751_pdout = k.get_process_data(pd_outputs_device);

            k.add_device(shared_from_this());

            if (state == module_state_safeop)
                break;
        }
        case safeop_2_op:
            // ====> start sending commands
            break;
        case op_2_op:
        case safeop_2_safeop:
        case preop_2_preop:
            // ====> do nothing
            break;

        default:
            break;
    }

    return (this->state = state);
}

//! module trigger callback
/*!
*/
void el6751::tick() {
    const auto& pdin  = el6751_pdin->get_read_buffer();
    auto& pdout = el6751_pdout->get_write_buffer();

    static int testcnt = 0;

//    if ((++testcnt % 1000) == 0) {
//        log(info, "el6751: %p : %d\n", el6751_pdin.get(), el6751_pdin.use_count());

    for (int i = 0; i < pdin.size(); ++i)
        printf("%02X", pdin[i]);
    printf("\n");

//    }
    return;
    switch (state) {
        default: 
            break;
        case module_state_safeop:
        case module_state_op:
            pdin_handler_can(pdin, pdout);

            if (state == module_state_safeop) {
                el6751_pdout->swap_buffers();
                break;
            }

            pdout_handler_can(pdin, pdout);
            el6751_pdout->swap_buffers();
            break;
    }
}

//! process data input callback
/*!
 * \param buf input buffer
 * \param buflen input buffer length
 */
void el6751::pdin_handler_can(const std::vector<uint8_t>& pdin, std::vector<uint8_t>& pdout) {
    if (    (pdin.size() < sizeof(can_interface_t)) ||
            (pdout.size() == 0))
        return; // no process data available

    auto can_pdin  = (can_pdin_t *)&pdin[0];
    auto can_pdout = (can_pdout_t *)&pdout[0];
    auto can_interface = (can_interface_t *)&(pdout[pdout.size() - sizeof(can_interface_t)]);
    
#define interface_check(member) \
    if (can_interface->member != local_can_interface.member) {   \
        log(error, #member" reported: %d\n", can_interface->member); \
        local_can_interface.member = can_interface->member; }

    interface_check(state);
    interface_check(error);
    interface_check(can_state);
    interface_check(rx_error_cnt);
    interface_check(tx_error_cnt);
    interface_check(diag);

    if (can_pdin->rx_cnt == can_pdout->rx_cnt)
        return; // no frames received

    log(verbose, "received %d can frames\n", can_pdin->msg_cnt);

    for (int i = 0; i < can_pdin->msg_cnt; ++i) {
        // decode to std can frame
        can::frame_t frame = (&can_pdin->msg)[i].to_can_frame();

        // process received frame
        for (auto it = slaves.begin(); it != slaves.end(); ++it) {
            sp_module_t m = *it;

            if (m->write((char *)&frame, sizeof(frame)))
                break; // frames should only be processed once
        }
    }

    // acknowledge received frames
    can_pdout->rx_cnt++;
}

void el6751::pdout_handler_can(const std::vector<uint8_t>& pdin, std::vector<uint8_t>& pdout) {
    if (    (pdin.size() < sizeof(can_interface_t)) ||
            (pdout.size() == 0))
        return; // no process data available

    auto can_pdin  = (can_pdin_t *)&pdin[0];
    auto can_pdout = (can_pdout_t *)&pdout[0];
    unsigned can_pdout_bufcnt = (pdout.size() - 6) / sizeof(can_message_29bit_t);

    if (can_pdout->tx_cnt != can_pdin->tx_cnt)
        return; // no frames to send

    unsigned msg_cnt = 0;
    int rd;
    can::frame_t frame;

    // process received frame
    for (auto it = slaves.begin(); it != slaves.end(); ++it) {
        sp_module_t m = *it;
        rd = m->read((char *)&frame, sizeof(frame));

        if (rd == 0)
            continue; // next slave    

        can_message_29bit_t& msg = (&can_pdout->msg)[msg_cnt++];
        msg.from_can_frame(frame);
        
        if (msg_cnt >= can_pdout_bufcnt)
            break;
    }

    if (msg_cnt) {
        log(verbose, "sending %d can frames\n", msg_cnt);
        can_pdout->msg_cnt = msg_cnt;
        can_pdout->tx_cnt++;
    }
}

