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

#ifndef __MODULE_EL6751_H__
#define __MODULE_EL6751_H__

#include <sys/queue.h>
#include "robotkernel/module_intf.h"
#include "robotkernel/kernel.h"

void el6751_log(robotkernel::loglevel lvl, std::string mod_name, const char *format, ...);

#endif // __MODULE_EL6751_H__

