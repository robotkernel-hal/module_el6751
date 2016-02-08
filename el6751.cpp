//! robotkernel module schunk el6751
/*!
 * author: Robert Burger
 *
 * $Id$
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

MODULE_DEF(module_el6751, beckhoff::el6751)

using namespace std;
using namespace robotkernel;
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
el6751::el6751(const std::string& name, const YAML::Node& node) 
    : module_base("module_el6751", name, node) {
    _ec_mod_name = get_as<string>(node, "ec_module");
    _ec_slave_id = get_as<int>(node, "ec_slave_id");

    if (node["slave_modules"]) {
        // parsing slave configurations
        const YAML::Node& slave_modules = node["slave_modules"];
        for (YAML::const_iterator it = slave_modules.begin();
                it != slave_modules.end(); ++it) {
            
            std::string mod_name = it->as<std::string>();
            _slave_module_names.push_back(mod_name); 
        }
    }

    memset(&local_can_interface, 0, sizeof(can_interface_t));
    this->state = module_state_init;
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
    switch (state) {
        case module_state_init:
        case module_state_preop:
            _slaves.clear();
            break;
        case module_state_safeop: {
            if (this->state >= state)
                break; // old state was op or safeop ... nothing to do

            // get el6751 process data
            process_data_t pd;
            pd.slave_id = _ec_slave_id;

            kernel::request_cb(_ec_mod_name.c_str(), MOD_REQUEST_GET_PDIN, &pd);
            _can_pdin = (can_pdin_t *)pd.pd;
            _can_interface = (can_interface_t *)(((uint8_t *)pd.pd)+(pd.len-sizeof(can_interface_t)));
            _can_pdin_bufcnt = (pd.len - 6 - sizeof(can_interface_t)) / sizeof(can_message_29bit_t);
            kernel::request_cb(_ec_mod_name.c_str(), MOD_REQUEST_GET_PDOUT, &pd);
            _can_pdout = (can_pdout_t *)pd.pd;
            _can_pdout_bufcnt = (pd.len - 6) / sizeof(can_message_29bit_t);

            log(info, "buffer count in: %d, out: %d\n",
                    _can_pdin_bufcnt, _can_pdout_bufcnt);

            kernel *k = kernel::get_instance();
            for (list<string>::iterator it = _slave_module_names.begin();
                    it != _slave_module_names.end(); ++it) {
                module *m = k->get_module((*it).c_str());

                if (!m)
                    throw str_exception("[module_el6751] module not found %s\n", it->c_str());

                _slaves.push_back(m);
            }
            break;
        }
        case module_state_op:
            break;
        default:
            break;
    }

    this->state = state;
    return state;
}

//! module trigger callback
/*!
*/
void el6751::trigger() {
    pdin_handler_can();
    pdout_handler_can();
}

//! check interface counters
void el6751::check_interface() {
#define interface_check(member) \
    if (_can_interface->member != local_can_interface.member) {   \
        log(error, #member" reported: %d\n", _can_interface->member); \
        local_can_interface.member = _can_interface->member; }

    interface_check(state);
    interface_check(error);
    interface_check(can_state);
    interface_check(rx_error_cnt);
    interface_check(tx_error_cnt);
    interface_check(diag);
}

//! process data input callback
/*!
 * \param buf input buffer
 * \param buflen input buffer length
 */
void el6751::pdin_handler_can() {
    if (!_can_pdin || !_can_interface || !_can_pdout)
        return; // no process data available

    check_interface();

    if (_can_pdin->rx_cnt == _can_pdout->rx_cnt)
        return; // no frames received

//    klog(info, "[module_el6751|%s] received %d can frames\n", _name.c_str(), _can_pdin->msg_cnt);

    for (int i = 0; i < _can_pdin->msg_cnt; ++i) {
        // decode to std can frame
        can::frame_t frame = (&_can_pdin->msg)[i].to_can_frame();

        // process received frame
        for (slave_list_t::iterator it = _slaves.begin();
                it != _slaves.end(); ++it) {
            module *m = *it;

            if (m->write((char *)&frame, sizeof(frame)))
                break; // frames should only be processed once
        }
    }

    // acknowledge received frames
    _can_pdout->rx_cnt++;
}

void el6751::pdout_handler_can() {
    if (!_can_pdin || !_can_interface || !_can_pdout)
        return; // no process data available

    if (_can_pdout->tx_cnt != _can_pdin->tx_cnt)
        return; // no frames to send

    unsigned msg_cnt = 0;
    int rd;
    can::frame_t frame;

    // process received frame
    for (slave_list_t::iterator it = _slaves.begin();
            it != _slaves.end(); ++it) {
        module *m = *it;
        rd = m->read((char *)&frame, sizeof(frame));

        if (rd == 0)
            continue; // next slave    

        can_message_29bit_t& msg = (&_can_pdout->msg)[msg_cnt++];
        msg.from_can_frame(frame);
        
        if (msg_cnt >= _can_pdout_bufcnt)
            break;
    }

    if (msg_cnt) {
//        klog(info, "[module_el6751|%s] sending %d can frames\n", _name.c_str(), msg_cnt);
        _can_pdout->msg_cnt = msg_cnt;
        _can_pdout->tx_cnt++;
    }
}

int el6751::request(int reqcode, void* ptr) {
	log(info, "el6751 does not implement request %#x(%#x)\n", reqcode, ptr);
	return 0;
}
