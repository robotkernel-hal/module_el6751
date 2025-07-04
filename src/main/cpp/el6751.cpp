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
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <errno.h>
#include <sys/stat.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <poll.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string_util/string_util.h>

MODULE_DEF(module_el6751, beckhoff::el6751)

using namespace std;
using namespace robotkernel;
using namespace string_util;
using namespace beckhoff;
        
class vcan_stream :
    public robotkernel::stream 
{
    public:
        std::shared_ptr<el6751> parent;
        int vcan_fd;

        //! construction
        vcan_stream(std::shared_ptr<el6751> parent, std::string vcan_name) : 
            robotkernel::stream(parent->name, "vcan_stream"), parent(parent) 
        {
            // ====> initial devices            
            parent->log(info, "opening can_raw %s ...\n", vcan_name.c_str());

            vcan_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if (vcan_fd == -1) {
                throw str_exception("socket SOCK_RAW : CAN_RAW "
                        "failed: %s", strerror(errno));
            }

            struct ifreq ifr;

            memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
            strncpy(ifr.ifr_name, vcan_name.c_str(), min(sizeof(ifr.ifr_name), vcan_name.size()));

            if (strcmp("any", ifr.ifr_name)) {
                if (ioctl(vcan_fd, SIOCGIFINDEX, &ifr) < 0) {
                    throw str_exception("get interface index failed: %s",
                            strerror(errno));
                }
            }

            struct sockaddr_can addr;
            memset(&addr, 0, sizeof(addr));
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            //memset(&addr, 0, sizeof(addr));
            if (bind(vcan_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                throw str_exception("bind failed: %s\n", strerror(errno));
            }
        }

        //! cyclic process data read
        /*!
          \param buf process data buffer
          \param bufsize size of process data buffer
          \return size of read bytes
          */
        size_t read(void* buf, size_t bufsize) {
            can::frame *frame = (can::frame *)buf;
            struct can_frame recv_frame;

            struct pollfd pollset;
            pollset.fd = vcan_fd;
            pollset.events = POLLIN;
            pollset.revents = 0;

            int local_ret = poll(&pollset, 1, 0);

            if (local_ret > 0) {
                ssize_t rd_bytes = ::read(vcan_fd, &recv_frame, sizeof(struct can_frame));

                if (rd_bytes > 0) {// sizeof(struct can_frame)) {
                    frame->hdr = recv_frame.can_id;
                    frame->rtr = 0;
                    frame->dlc = recv_frame.can_dlc;
                    memcpy(&frame->data[0], &recv_frame.data[0], 8);
                    return sizeof(can::frame);
                }
            }

            return 0;
        }

        //! cyclic process data write
        /*!
          \param buf process data buffer
          \param bufsize size of process data buffer
          \return size of written bytes
          */
        size_t write(void* buf, size_t bufsize) {
            can::frame *frame = (can::frame *)buf;
            struct can_frame send_frame;
            send_frame.can_id = frame->hdr;
            send_frame.can_dlc = frame->dlc;
            memcpy(&send_frame.data[0], &frame->data[0], 8);

            if (parent->ll == verbose) {
                static char frame_buf[100];
                int pos = 0;
                for (unsigned i = 0; i < sizeof(struct can_frame); ++i) {
                    pos+=snprintf(&frame_buf[pos], 100-pos, "%02X ", ((uint8_t *)&send_frame)[i]);
                }

                parent->log(verbose, "can id %X, dlc %d\n", send_frame.can_id, send_frame.can_dlc);
            }

            return ::write(vcan_fd, &send_frame, sizeof(struct can_frame));
        }
};

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
    module_base("module_el6751", name, node), trigger(name, "el6751")
{
    local_tx_cnt = 0;
    local_rx_cnt = 0;

    pd_inputs_device  = get_as<string>(node, "pd_inputs_device");
    pd_outputs_device = get_as<string>(node, "pd_outputs_device");

    extended_mode     = get_as<bool>  (node, "extended_mode", true);
    with_padding      = get_as<bool>  (node, "with_padding", true);
    tx_buf_cnt        = get_as<unsigned int>(node, "tx_buf_cnt", 10);
    rx_buf_cnt        = get_as<unsigned int>(node, "rx_buf_cnt", 10);

    if (node["slave_streams"]) {
        // parsing slave configurations
        for (const auto& stream_node : node["slave_streams"]) {
            std::string mod_name = stream_node.as<std::string>();
            slave_stream_names.push_back(mod_name); 
        }
    }

    vcan_name = get_as<string>(node, "vcan_name", "");

    memset(&local_can_interface, 0, sizeof(can_interface_t));
}

//! destruction 
el6751::~el6751() {
    set_state(module_state_init);
}

//! State transition from SAFEOP to PREOP
void el6751::set_state_safeop_2_preop() {
    // ====> stop receiving measurements
    robotkernel::remove_device(shared_from_this());

    if (el6751_pdin->trigger_dev)
        el6751_pdin->trigger_dev->remove_trigger(shared_from_this());

    el6751_pdin->reset_consumer(el6751_pdin_consumer);
    el6751_pdin_consumer = nullptr;
    el6751_pdin = nullptr;

    el6751_pdout->reset_provider(el6751_pdout_provider);
    el6751_pdout_provider = nullptr;
    el6751_pdout = nullptr;
}

//! State transition from PREOP to INIT
void el6751::set_state_preop_2_init() {
    streams.clear();
}

//! State transition from INIT to PREOP
void el6751::set_state_init_2_preop() {
    for (const auto& name : slave_stream_names) {
        sp_stream_t m = robotkernel::get_device<stream>(name);

        if (!m)
            throw str_exception("[module_el6751] stream %s not found\n", name.c_str());

        streams[name] = m;
    }

    if (vcan_name != "") {
        streams[vcan_name] = make_shared<vcan_stream>(shared_from_this(), vcan_name);
    }
}

//! State transition from PREOP to SAFEOP
void el6751::set_state_preop_2_safeop() {
    // ====> get el6751 process data
    el6751_pdin = robotkernel::get_device<process_data>(pd_inputs_device);
    el6751_pdin_consumer = make_shared<pd_consumer>(name + "." + el6751_pdin->id());
    el6751_pdin->set_consumer(el6751_pdin_consumer);
    if (el6751_pdin->trigger_dev) {
        el6751_pdin->trigger_dev->add_trigger(shared_from_this());
    }

    el6751_pdout = robotkernel::get_device<process_data>(pd_outputs_device);
    el6751_pdout_provider = make_shared<pd_provider>(name + "." + el6751_pdout->id());
    el6751_pdout->set_provider(el6751_pdout_provider);

    robotkernel::add_device(shared_from_this());
}

//! module trigger callback
/*!
*/
void el6751::tick() {
    auto pdin_ptr   = el6751_pdin->pop(el6751_pdin_consumer);
    auto pdout_ptr  = el6751_pdout->next(el6751_pdout_provider);

    switch (state) {
        default: 
            break;
        case module_state_safeop:
        case module_state_op:
            pdin_handler_can(pdin_ptr, el6751_pdin->length, 
                    pdout_ptr, el6751_pdout->length);

            if (state == module_state_op) {
                pdout_handler_can(pdin_ptr, el6751_pdin->length, 
                                pdout_ptr, el6751_pdout->length);
            } else {
                auto can_pdout = (can_pdout_t *)&pdout_ptr[0];
                // reset message count to ensure if there's nothing to send, nothing will be sent!
                can_pdout->msg_cnt = 0;
                can_pdout->tx_cnt = local_tx_cnt;
            }

            el6751_pdout->push(el6751_pdout_provider);
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

    uint8_t *act_msg = &can_pdin->msg[0];
    if (with_padding) {
        act_msg += 2;
    }

    for (int i = 0; i < can_pdin->msg_cnt; ++i) {
        can::frame_t frame;

        // decode to std can frame
        if (extended_mode) {
            uint32_t cobid = *((uint32_t *)&act_msg[2]);
            
            frame.dlc = *((uint16_t *)&act_msg[0]);
            frame.rtr = cobid & CAN_COB_29BIT_RTR ? 1 : 0;
            frame.hdr = cobid & ~CAN_COB_29BIT_RTR;

            memcpy(frame.data, &act_msg[6], 8);

            act_msg += 14;
        } else {
            uint16_t cobid = *((uint16_t *)&act_msg[0]);
            frame.dlc = cobid & 0x000F;
            frame.rtr = (cobid & 0x0010) >> 4;
            frame.hdr = (cobid & 0xFFE0) >> 5;
            memcpy(frame.data, &act_msg[2], 8);

            act_msg += 10;
        }

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
            (pdout_len == 0)) {
        return; // no process data available
    }	

    auto can_pdin  = (can_pdin_t *)&pdin[0];
    auto can_pdout = (can_pdout_t *)&pdout[0];

    // reset message count to ensure if there's nothing to send, nothing will be sent!
    can_pdout->msg_cnt = 0;

    if (can_pdout->tx_cnt != can_pdin->tx_cnt) {
        can_pdout->tx_cnt = local_tx_cnt;
        return; // no frames to send
    }

    unsigned msg_cnt = 0;
    int rd;
    can::frame_t frame;

    uint8_t *act_msg = &can_pdout->msg[0];

    // process received frame
    for (const auto& kv : streams) {
        sp_stream_t m = kv.second;
        rd = m->read((char *)&frame, sizeof(frame));

        if (rd == 0)
            continue; // next slave    

        if (with_padding) {
            act_msg += 2;
        }

        if (extended_mode) {
            *((uint16_t *)&act_msg[0]) = frame.dlc;
            *((uint32_t *)&act_msg[2]) = frame.hdr | (frame.rtr << 30);
            memcpy(&act_msg[6], frame.data, 8);

            act_msg += 14;
            msg_cnt++;
        } else {
            *((uint16_t *)&act_msg[0]) = 
                ((frame.hdr & 0x07FF) << 5) | 
                ((frame.rtr & 0x01) << 4) |
                ((frame.dlc & 0x000F));
            memcpy(&act_msg[2], frame.data, 8);

            act_msg += 10;
            msg_cnt++;
        }
        
        if (msg_cnt >= rx_buf_cnt)
            break;
    }

    if (msg_cnt) {
        can_pdout->msg_cnt = msg_cnt;
        can_pdout->tx_cnt = ++local_tx_cnt;
        log(verbose, "sending %d can frames, tx_cnt %d, local_tx_cnt %d\n", msg_cnt, can_pdout->tx_cnt, local_tx_cnt);
    }
}

