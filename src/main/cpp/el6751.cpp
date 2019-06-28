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
  pd_inputs_device: ecat.slave_2.inputs.pd
  pd_outputs_device: ecat.slave_2.outputs.pd
  slave_streams: [ pg70_1.packet.stream, pg70_2.packet.stream, ]
 
*/

//! construction
/*!
 * \param node yaml intialization node
 */
el6751::el6751(const std::string& name, const YAML::Node& node) :
    module_base("module_el6751", name, node), trigger(name, "el6751"),
    pd_consumer(name + ".inputs"), pd_provider(name + ".outputs"),
    el6751_pdin(nullptr), el6751_pdin_trigger(nullptr), el6751_pdin_hash(0),
    el6751_pdout(nullptr), el6751_pdout_trigger(nullptr), el6751_pdout_hash(0)
{
    local_tx_cnt = 0;
    local_rx_cnt = 0;

    pd_inputs_device  = get_as<string>(node, "pd_inputs_device");
    pd_outputs_device = get_as<string>(node, "pd_outputs_device");

    extended_mode     = get_as<bool>  (node, "extended_mode", true);

    if (node["slave_streams"]) {
        // parsing slave configurations
        for (const auto& stream_node : node["slave_streams"]) {
            std::string mod_name = stream_node.as<std::string>();
            slave_stream_names.push_back(mod_name); 
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

            if (el6751_pdin_trigger)
                el6751_pdin_trigger->remove_trigger(shared_from_this());

            el6751_pdin->reset_consumer(el6751_pdin_hash);
            el6751_pdin_hash = 0;
            el6751_pdin = nullptr;
            el6751_pdin_trigger = nullptr;            

            el6751_pdout->reset_provider(el6751_pdout_hash);
            el6751_pdout_hash = 0;
            el6751_pdout = nullptr;
            el6751_pdout_trigger = nullptr;

            if (state == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
            streams.clear();
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
            for (const auto& name : slave_stream_names) {
                sp_stream_t m = k.get_stream(name);

                if (!m)
                    throw str_exception("[module_el6751] stream %s not found\n", name.c_str());

                streams[name] = m;
            }

            if (state == module_state_preop)
                break;
        case preop_2_op:
        case preop_2_safeop: {
            // ====> get el6751 process data
            el6751_pdin = k.get_process_data(pd_inputs_device);
            el6751_pdin_hash = el6751_pdin->set_consumer(shared_from_this());
            if (el6751_pdin->clk_device != "") {
                el6751_pdin_trigger = k.get_trigger(el6751_pdin->clk_device);
                el6751_pdin_trigger->add_trigger(shared_from_this());
            }

            el6751_pdout = k.get_process_data(pd_outputs_device);
            el6751_pdout_hash = el6751_pdout->set_provider(shared_from_this());
            if (el6751_pdout->clk_device != "")
                el6751_pdout_trigger = k.get_trigger(el6751_pdout->clk_device);

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
    auto pdin_ptr   = el6751_pdin->pop(el6751_pdin_hash);
    auto pdout_ptr  = el6751_pdout->next(el6751_pdout_hash);

    switch (state) {
        default: 
            break;
        case module_state_safeop:
        case module_state_op:
            pdin_handler_can(pdin_ptr, el6751_pdin->length, 
                    pdout_ptr, el6751_pdout->length);

            if (state == module_state_op)
                pdout_handler_can(pdin_ptr, el6751_pdin->length, 
                        pdout_ptr, el6751_pdout->length);

            el6751_pdout->push(el6751_pdout_hash);
            if (el6751_pdout_trigger)
                el6751_pdout_trigger->trigger_modules();

            break;
    }
}

// process data input callback
void el6751::pdin_handler_can(uint8_t *pdin, size_t pdin_len, 
        uint8_t *pdout, size_t pdout_len) 
{
    if (    (pdin_len < sizeof(can_interface_t)) ||
            (pdout_len == 0))
        return; // no process data available

    auto can_pdin  = (can_pdin_t *)&pdin[0];
    auto can_pdout = (can_pdout_t *)&pdout[0];
    auto can_interface = (can_interface_t *)&(pdout[pdout_len - sizeof(can_interface_t)]);
    
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

    if (can_pdin->rx_cnt == local_rx_cnt) {
        can_pdout->rx_cnt = local_rx_cnt;
        return; // no frames received
    }

    log(verbose, "received %d can frames\n", can_pdin->msg_cnt);

    for (int i = 0; i < can_pdin->msg_cnt; ++i) {
        can::frame_t frame;

        // decode to std can frame
        if (extended_mode)
            frame = ((can_message_29bit_rx_t *)&can_pdin->msg)[i].to_can_frame();
        else
            frame = ((can_message_11bit_rx_t *)&can_pdin->msg)[i].to_can_frame();

        // process received frame
        for (const auto& kv : streams) {
            if (kv.second->write((char *)&frame, sizeof(frame)))
                break; // frames should only be processed once
        }
    }

    // acknowledge received frames
    can_pdout->rx_cnt = ++local_rx_cnt;
}

void el6751::pdout_handler_can(uint8_t *pdin, size_t pdin_len, 
                uint8_t *pdout, size_t pdout_len)
{
    if (    (pdin_len < sizeof(can_interface_t)) ||
            (pdout_len == 0))
        return; // no process data available

    auto can_pdin  = (can_pdin_t *)&pdin[0];
    auto can_pdout = (can_pdout_t *)&pdout[0];
    unsigned can_pdout_bufcnt = extended_mode ?
        (pdout_len - 6) / sizeof(can_message_29bit_tx_t) : 
        (pdout_len - 6) / sizeof(can_message_11bit_tx_t);

    if (can_pdout->tx_cnt != can_pdin->tx_cnt) {
        can_pdout->tx_cnt = local_tx_cnt;
        return; // no frames to send
    }

    unsigned msg_cnt = 0;
    int rd;
    can::frame_t frame;

    // process received frame
    for (const auto& kv : streams) {
        sp_stream_t m = kv.second;
        rd = m->read((char *)&frame, sizeof(frame));

        if (rd == 0)
            continue; // next slave    

        if (extended_mode) {
            can_message_29bit_tx_t& msg = ((can_message_29bit_tx_t *)&can_pdout->msg)[msg_cnt++];
            msg.from_can_frame(frame);
        } else {
            can_message_11bit_tx_t& msg = ((can_message_11bit_tx_t *)&can_pdout->msg)[msg_cnt++];
            msg.from_can_frame(frame);
        }
        
        if (msg_cnt >= can_pdout_bufcnt)
            break;
    }

    if (msg_cnt) {
        log(verbose, "sending %d can frames\n", msg_cnt);
        can_pdout->msg_cnt = msg_cnt;
        can_pdout->tx_cnt = ++local_tx_cnt;
    }
}

