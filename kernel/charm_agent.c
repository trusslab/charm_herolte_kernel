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

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/Charm/charm_irq.h>
#include <linux/Charm/charm_rbtree.h>
#include <linux/Charm/charm_rpc.h>
#include <linux/Charm/charm_register_interface.h>
#include <linux/Charm/charm_usb.h>
#include <linux/Charm/prints.h>
#include <linux/slab.h>



#define PAYLOAD_SIZE_REG 17
#define PAYLOAD_SIZE_RPC 256

int charm_began;
int num_reg_addrs;
uint64_t reg_phys_base[64];
uint64_t reg_phys_size[64];
uint64_t reg_ptrs[64];
struct task_struct *reg_thread, *rpc_thread;
  

struct device *charm_print_all_device_names(struct bus_type *bus,
				       struct device *start, const char *name);

int send_irq_charm(uint64_t irq)
{
	int ret;

	ret = acc_write_loop(acc_write_3, &acc_fp, (char __user *) &irq, 8, NULL);
	if (ret != 8) {
		PRINTK_ERR("Could not send the message, ret = %d\n", ret);
		acc_release(NULL, &acc_fp);
		return -EFAULT;
	}

	return 0;
}

static int rpc_thread_main(void *data)
{
	void *buffer;
	uint64_t response_value;
	int ret;

	buffer = kmalloc(PAYLOAD_SIZE_RPC, GFP_KERNEL);
	if (!buffer) {
		PRINTK_ERR("Could not allocate memory for buffer\n");
		return -ENOMEM;
	}

	
	set_user_nice(current, -20);

	while(true) {
		ret = acc_read_loop(acc_read_2, &acc_fp, (char __user *) buffer,
							PAYLOAD_SIZE_RPC, NULL);
		if (ret != PAYLOAD_SIZE_RPC) {
			PRINTK_ERR("Could not receive the message: ret = %d\n", ret);
			acc_release(NULL, &acc_fp);
			return -EFAULT;
		}
		response_value = rpc_callback(buffer);
		ret = acc_write_loop(acc_write_2, &acc_fp, (char __user *) &response_value,
										8, NULL);
		if (ret != 8) {
			PRINTK_ERR("Could not send the message\n");
			acc_release(NULL, &acc_fp);
			return -EFAULT;
		}
	}

	return 0;
}

static int reg_thread_main(void *data)
{
	void *buffer;
	uint64_t response_value;
	int ret;
	bool is_read;

	buffer = kmalloc(PAYLOAD_SIZE_REG, GFP_KERNEL);
	if (!buffer) {
		PRINTK_ERR("Could not allocate memory for buffer");
		return -ENOMEM;
	}

	set_user_nice(current, -20);

	while(true) {
		ret = acc_read_loop(acc_read, &acc_fp, (char __user *) buffer,
							PAYLOAD_SIZE_REG, NULL);
		if (ret != PAYLOAD_SIZE_REG) {
			PRINTK_ERR("Could not receive the message, ret = %d", ret);
			acc_release(NULL, &acc_fp);
			return -EFAULT;
		}

		is_read = reg_callback(buffer, &response_value);

		if (is_read) {
			ret = acc_write_loop(acc_write, &acc_fp,
					(const char __user *) &response_value, 8, NULL);
			if (ret != 8) {
				PRINTK_ERR("Could not send the message\n");
				acc_release(NULL, &acc_fp);
				return -EFAULT;
			}
		}
	}
}


int charm_agent_init2(void)
{
	
	printk(KERN_INFO"Charm module started\n");
	while (acc_open(NULL, &acc_fp)) {
		PRINTK_ERR("acc_open failed");
		mdelay(1000);
	}
	printk(KERN_INFO"Charm module, usb interface initilized\n");
	reg_thread = kthread_run(reg_thread_main, NULL, "charm_reg_thread");
	rpc_thread = kthread_run(rpc_thread_main, NULL, "charm_rpc_thread");

//Uncomment the bellow block inorder to print names of devices on Platform bus
/*
	charm_print_all_device_names(&platform_bus_type,NULL, "dummy_name");
	mdelay(100000);
	return 0;
*/
	register_irq_handlers();
	charm_began=1;

	return 0;
}

int charm_agent_init(void)
{
	charm_began=0;
	num_reg_addrs=1;

	if(reg_phys_addr_init()<0){
		return -EFAULT;
	}


	return 0;
}

void charm_agent_exit(void)
{
	int ret;

	ret = kthread_stop(reg_thread);
	if (ret)
		PRINTK_ERR("Could not stop charm_reg_thread (err = %d)", ret);

	ret = kthread_stop(rpc_thread);
	if (ret)
		PRINTK_ERR("Could not stop charm_rpc_thread (err = %d)", ret);

}

module_init(charm_agent_init);
module_exit(charm_agent_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Seyed Mohammadjavad Seyed Talebi");
MODULE_DESCRIPTION("mjavad:charm agent for android");
