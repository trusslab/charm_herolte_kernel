/* Copyright (C) 2016-2018 University of California, Irvine
 * 
 * Authors:
 * Seyed Mohammadjavad Seyed Talebi <mjavad@uci.edu>
 * Ardalan Amiri Sani <arrdalan@gmail.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/proc_fs.h>
#define READB_OPCODE 1
#define READW_OPCODE 2
#define READL_OPCODE 3
#define READQ_OPCODE 4
#define WRITEB_OPCODE 6
#define WRITEW_OPCODE 7
#define WRITEL_OPCODE 8
#define WRITEQ_OPCODE 9
bool reg_callback(void *data, uint64_t *return_value);
int reg_phys_addr_init(void);
