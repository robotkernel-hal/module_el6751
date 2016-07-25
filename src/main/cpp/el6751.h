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


#ifndef __EL6751_H__
#define __EL6751_H__

#include "robotkernel/kernel.h"
#include "robotkernel/module.h"
#include "robotkernel/module_base.h"
#include "robotkernel/module_intf.h"

#define PACK __attribute__((__packed__)) 

const static uint32_t CAN_COB_29BIT_RTR  = 0x40000000;

namespace can {

typedef struct PACK frame {
    uint32_t hdr;
    uint8_t  rtr;
    uint8_t  dlc;
    uint8_t  data[8];
} PACK frame_t;

}

namespace beckhoff {

class el6751 : public robotkernel::module_base {    
    public:        
        //! el6751 specific can message - 29 bit cobid format
        typedef struct PACK can_message_29bit {
            uint16_t len;
            uint32_t cobid;
            uint8_t data[8];

            //! decode el6751 can frame to std can frame
            /*!
             * \return std can frame
             */
            can::frame_t to_can_frame() {
                can::frame_t frame = can::frame_t();
                frame.dlc          = len;
                frame.rtr          = cobid & CAN_COB_29BIT_RTR ? 1 : 0;
                frame.hdr          = cobid & ~CAN_COB_29BIT_RTR;
                memcpy(frame.data, data, 8);
                return frame;
            }

            //! assign data from std can frame
            /*!
             * \param frame input can frame
             */
            void from_can_frame(can::frame_t& frame) {
                len   = frame.dlc;
                cobid = frame.hdr | (frame.rtr << 30);
                memcpy(data, frame.data, 8);
            }
        } PACK can_message_29bit_t;

        typedef struct PACK can_pdin {
            uint16_t tx_cnt;
            uint16_t rx_cnt;
            uint16_t msg_cnt;
            can_message_29bit_t msg;
        } PACK can_pdin_t;
            
        typedef struct PACK can_interface {
            uint8_t  state;
            uint8_t  error;
            uint16_t can_state;
            uint8_t  rx_error_cnt;
            uint8_t  tx_error_cnt;
            uint8_t  diag;
        } PACK can_interface_t;

        typedef struct PACK can_pdout {
            uint16_t tx_cnt;
            uint16_t rx_cnt;
            uint16_t msg_cnt;
            can_message_29bit_t msg;
        } PACK can_pdout_t;

        can_interface_t local_can_interface;//! local copy of caninterface 

        can_pdin_t      *_can_pdin;         //! actual process data in - can mode
        can_pdout_t     *_can_pdout;        //! actual process data out - can mode
        can_interface_t *_can_interface;    //! actual interface data in - can mode

        size_t _can_pdin_bufcnt;
        size_t _can_pdout_bufcnt;

        std::string _ec_mod_name;
        int _ec_slave_id;

        typedef std::list<robotkernel::module *> slave_list_t;
        slave_list_t _slaves;

        std::list<std::string> _slave_module_names; //! name of slave modules

        //! construction
        /*!
         * \param node yaml intialization node
         */
        el6751(const std::string& name, const YAML::Node& node);

        //! destruction 
        ~el6751();

        //! set module state machine to defined state
        /*!
          \param state requested state
          \return success or failure
          */
        int set_state(module_state_t state);

        //! module trigger callback
        /*!
        */
        void trigger();

        int request(int reqcode, void* ptr);
	
    private:
        //! check interface counters
        void check_interface();

        //! process data input callback
        /*!
         * \param buf input buffer
         * \param buflen input buffer length
         */
        void pdin_handler_can();

        //! process data input callback
        /*!
         * \param buf input buffer
         * \param buflen input buffer length
         */
        void pdout_handler_can();
};

}; // namespace beckhoff

#endif // __EL6751_H__

