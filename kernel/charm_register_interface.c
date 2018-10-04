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

#include <linux/Charm/prints.h>
#include <linux/Charm/charm_register_interface.h>

extern uint64_t reg_phys_base[64];
extern uint64_t reg_phys_size[64];
extern int num_reg_addrs;
extern uint64_t reg_ptrs[64];

bool reg_callback(void *data, uint64_t *return_value)
{		
	char *opcode;
	uint64_t address;
	uint64_t *value;
	void *ptr;
	bool is_read = false;
	bool address_is_valid = false;
	int i=0;
	opcode = data;  	
	address = *((uint64_t *) (data + sizeof(char))); 
	for (i=0 ; i<num_reg_addrs ; i++){
		if( (address >= reg_phys_base[i]) && (address< (reg_phys_base[i] + reg_phys_size[i])) ){
			address_is_valid = true;		
			ptr = (void *) (reg_ptrs[i] + (address - reg_phys_base[i]));
			break;
		}
	}
	if (!address_is_valid){
		/* FIXME: This is not correct error handling */
		PRINTK_ERR("Invalid address");
		return false;
	}

	switch(*opcode){
	case READB_OPCODE:
		*return_value=(uint64_t)readb(ptr);
		is_read = true;
	break;	

	case READW_OPCODE:
		*return_value=(uint64_t)readw(ptr);
		is_read = true;
	break;

	case READL_OPCODE:
		*return_value=(uint64_t)readl(ptr);
		is_read = true;
	break;

	case READQ_OPCODE:
		*return_value=(uint64_t)readq(ptr);
		is_read = true;
	break;	

	case WRITEB_OPCODE:
	        value = data + sizeof(char) + sizeof(uint64_t);
		writeb(*value,ptr);
	break;	

	case WRITEW_OPCODE:
	        value = data + sizeof(char) + sizeof(uint64_t);
		writew(*value,ptr);
	break;

	case WRITEL_OPCODE:
	        value = data + sizeof(char) + sizeof(uint64_t);
		writel(*value,ptr);
	break;

	case WRITEQ_OPCODE:
	        value = data + sizeof(char) + sizeof(uint64_t);
		writeq(*value,ptr);	
	break;	
	}
	return is_read;
}


int reg_phys_addr_init (void)
{

	void *reg_ptr;
	int i=0;
	reg_phys_base[0] = 0x14d30000;
	reg_phys_size[0] = PAGE_SIZE;


	for(i =0 ; i<num_reg_addrs ; i++){

		reg_ptr = ioremap(reg_phys_base[i], reg_phys_size[i]);
		if (!reg_ptr) {
			PRINTK_ERR("Could not map register page (%d)",i);
			return -EFAULT;
		}
		reg_ptrs[i] = (uint64_t) reg_ptr;
	}
	return 0;

}
