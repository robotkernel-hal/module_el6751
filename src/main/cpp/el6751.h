//! robotkernel module beckhoff el6751 can/canopen terminal
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


#ifndef __EL6751_H__
#define __EL6751_H__

#include "robotkernel/kernel.h"
#include "robotkernel/module.h"
#include "robotkernel/module_base.h"
#include "robotkernel/module_intf.h"
#include "robotkernel/stream.h"

#define PACK __attribute__((__packed__)) 

const static uint32_t CAN_COB_29BIT_RTR  = 0x40000000;

namespace can {
#ifdef EMACS
}
#endif

typedef struct PACK frame {
    uint32_t hdr;
    uint8_t  rtr;
    uint8_t  dlc;
    uint8_t  data[8];
} PACK frame_t;

#ifdef EMACS
{
#endif
}

#include <arpa/inet.h>

namespace beckhoff {
#ifdef EMACS
}
#endif

class el6751 : 
    public std::enable_shared_from_this<el6751>,
    public robotkernel::module_base, 
    public robotkernel::trigger
{    
    public:
        typedef struct PACK can_pdin {
            uint16_t tx_cnt;
            uint16_t rx_cnt;
            uint16_t msg_cnt;
            uint8_t  msg[0];
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
            uint8_t  msg[0];
        } PACK can_pdout_t;

        uint16_t local_tx_cnt, local_rx_cnt;
        bool extended_mode;
        bool with_padding;
        unsigned int tx_buf_cnt;
        unsigned int rx_buf_cnt;

        can_interface_t local_can_interface;//! local copy of caninterface 

        robotkernel::sp_process_data_t el6751_pdin = nullptr;
        robotkernel::sp_pd_consumer_t el6751_pdin_consumer = nullptr;

        robotkernel::sp_process_data_t el6751_pdout = nullptr;
        robotkernel::sp_pd_provider_t el6751_pdout_provider = nullptr;

        std::string pd_inputs_device;               //!< process data device with inputs
        std::string pd_outputs_device;              //!< process data device with outputs
        std::list<std::string> slave_stream_names;  //!< name of slave modules
        std::string vcan_name;

        robotkernel::stream_map_t streams;

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
        void tick();
    
    private:
        //! Process data input callback.
        /*!
         * \param[in]       pdin      Pointer to process data inputs.
         * \param[in]       pdin_len  Length of process data inputs.
         * \param[in,out]   pdout     Pointer to process data outputs.
         * \param[in]       pdout_len Length of process data outputs.
         */
        void pdin_handler_can(uint8_t *pdin, size_t pdin_len,
                uint8_t *pdout, size_t pdout_len);

        //! Process data input callback.
        /*!
         * \param[in]       pdin      Pointer to process data inputs.
         * \param[in]       pdin_len  Length of process data inputs.
         * \param[in,out]   pdout     Pointer to process data outputs.
         * \param[in]       pdout_len Length of process data outputs.
         */
        void pdout_handler_can(uint8_t *pdin, size_t pdin_len, 
                uint8_t *pdout, size_t pdout_len);
};

#ifdef EMACS
{
#endif
}; // namespace beckhoff

#endif // __EL6751_H__

