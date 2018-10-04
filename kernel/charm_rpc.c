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

#include <linux/device.h>
#include <linux/of.h>
#include <linux/clk.h>
//#include <linux/clk/msm-clk-provider.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
//#include <linux/regulator/rpm-smd-regulator.h>
//#include <linux/qcom_iommu.h>
//#include <linux/clk/msm-clk.h>
#include <linux/delay.h>
#include <linux/iommu.h>
//#include <linux/msm_iommu_domains.h>
#include "../drivers/staging/android/ion/ion.h"
#include <linux/reboot.h>
#include <soc/samsung/exynos-powermode.h>
#include <linux/smc.h>

#include <linux/Charm/prints.h>
#include <linux/Charm/charm_rpc.h>
#include <linux/Charm/charm_rbtree.h>


struct device * charm_dev;
extern struct bus_type platform_bus_type;


uint64_t rpc_callback(void *data)
{
	uint64_t opcode;	
	uint64_t return_value;
	//int res;
	int rc;
        unsigned long int offset;
	char dev_id[128];
	char con_id[128];
	char compatible_name[128];
	char state_name[128];
	char my_gpio_label[128];
	char bullhead_name_buf[128];
	char gpio_name[128];

	struct device_node * my_of_node;
	struct device * my_device;
	struct iommu_domain * my_domain;
	int my_index;

        int msg_len;
	int my_iop;
	int my_vmin;
	int my_vmax;
	int my_gpio_value;
        int my_gpio_int;
        bool my_gpio_value_bool;
	int my_domain_num;
	void * my_ptr;
	bool bool_rc;
	struct clk * my_clk;
	struct clk * my_clk_parent;

        int my_int1,my_int2;
        unsigned long my_ulint1,my_ulint2,my_ulint3,my_ulint4;

	struct regulator * my_regul;
	struct pinctrl * my_pinctrl;
	struct pinctrl_state * my_state;
	unsigned long my_rate;
	long out_rate;
	long long_rc;
	unsigned my_gpio;
	unsigned long my_gpio_flags;

	uint64_t ptr1;
	uint64_t ptr2;
	
	struct ion_client * my_ionc;
	struct ion_handle * my_ionh;

	memset(bullhead_name_buf,0,128);
	offset=0;
	memcpy(&opcode , data, sizeof(opcode));
	offset += sizeof(opcode);
	switch(opcode){
	case REBOOT_RPC_CODE:
		PRINTKL("Charm RPC Rebooting\n");
                kernel_restart(NULL);
                
        break;
        case CLK_GET_RPC_CODE:
	        memcpy(&msg_len ,data+offset,  sizeof(msg_len));
		offset+=sizeof(msg_len);	
	        memcpy(dev_id ,data+offset, msg_len);
		offset+=msg_len;
		memcpy(&msg_len ,data+offset,  sizeof(msg_len));	
		offset+=sizeof(msg_len);	
	        memcpy(con_id ,data+offset, msg_len);
		offset+=msg_len;
		
		memcpy(&msg_len ,data+offset,  sizeof(msg_len));	
		offset+=sizeof(msg_len);	

	        memcpy(compatible_name ,data+offset, msg_len);
		offset+=msg_len;

		dev_name_transform(dev_id,bullhead_name_buf);

		charm_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);

		if(IS_ERR_OR_NULL(charm_dev))
			PRINTK_ERR("charm_dev is error or null");
		my_clk = clk_get(charm_dev,con_id);		
		add_valid_pointer((uint64_t)my_clk);
                memcpy(&return_value,&my_clk,8);
        break;
        case CLK_GET_SYS_RPC_CODE:
                memcpy(&msg_len ,data+offset,  sizeof(msg_len));
                offset+=sizeof(msg_len);
                memcpy(dev_id ,data+offset, msg_len);
                offset+=msg_len;
                memcpy(&msg_len ,data+offset,  sizeof(msg_len));
                offset+=sizeof(msg_len);
                memcpy(con_id ,data+offset, msg_len);
                offset+=msg_len;
                my_clk = clk_get_sys(dev_id,con_id);
                PRINTKL("clk ptr= %lx",(unsigned long)my_clk);
                add_valid_pointer((uint64_t)my_clk);
                memcpy(&return_value,&my_clk,8);
        break;
        case CLK_GET_PARENT_RPC_CODE:
                memcpy(&my_clk ,data+offset, 8 );
                offset+=8;
                if(pointer_is_valid((uint64_t)my_clk)){
                        my_clk_parent=clk_get_parent(my_clk);
                        add_valid_pointer((uint64_t)my_clk_parent);
                }
                memcpy(&return_value,&my_clk_parent,sizeof(long));
        break;

	case CLK_ROUND_RATE_RPC_CODE:
		memcpy(&my_clk ,data+offset, 8 );
		offset+=8;
		memcpy(&my_rate,data+offset,sizeof(unsigned long));
		if(pointer_is_valid((uint64_t)my_clk))
			out_rate=clk_round_rate(my_clk,my_rate);
		memcpy(&return_value,&out_rate,sizeof(long));

	break;
	case CLK_SET_RATE_RPC_CODE:	
		memcpy(&my_clk ,data+offset, 8 );
		offset+=8;
		memcpy(&my_rate,data+offset,sizeof(unsigned long));
		if(pointer_is_valid((uint64_t)my_clk))
			rc=clk_set_rate(my_clk,my_rate);
		memcpy(&return_value,&rc,sizeof(int));
	break;
	case CLK_GET_RATE_RPC_CODE:
                memcpy(&my_clk ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_clk))
                	my_rate=clk_get_rate(my_clk);
                memcpy(&return_value,&my_rate,sizeof(unsigned long));
	break;
	case CLK_PREPARE_RPC_CODE:
		memcpy(&my_clk ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_clk))
			rc=clk_prepare(my_clk);
		memcpy(&return_value,&rc,sizeof(int));
	break;
	case CLK_ENABLE_RPC_CODE:
                memcpy(&my_clk ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_clk))
                	rc=clk_enable(my_clk);
                memcpy(&return_value,&rc,sizeof(int));
	break;

	case CLK_SET_PARENT_RPC_CODE:
                memcpy(&my_clk ,data+offset, 8 );
		offset+=8;
                memcpy(&my_clk_parent ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_clk))
                	rc=clk_set_parent(my_clk,my_clk_parent);
                memcpy(&return_value,&rc,sizeof(int));
	break;
	case CLK_DISABLE_RPC_CODE:
                memcpy(&my_clk ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_clk))
                	clk_disable(my_clk);
		rc=999;
                memcpy(&return_value,&rc,sizeof(int));
	break;
	case CLK_UNPREPARE_RPC_CODE:
                memcpy(&my_clk ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_clk))
                	clk_unprepare(my_clk);
		rc=999;
                memcpy(&return_value,&rc,sizeof(int));
	break;
	case CLK_PUT_RPC_CODE:
                memcpy(&my_clk ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_clk)){
                	clk_put(my_clk);
			remove_valid_pointer((uint64_t)my_clk);
		}
		rc=999;
                memcpy(&return_value,&rc,sizeof(int));
	break;
	case CLK_RESET_RPC_CODE:

                //memcpy(&my_clk ,data+offset, 8 );
		//offset+=8;
		//memcpy(&my_clk_action,data+offset,sizeof(unsigned long));
		//if(pointer_is_valid((uint64_t)my_clk))
		//	rc=clk_reset(my_clk,my_clk_action);	
                //memcpy(&return_value,&rc,sizeof(int));
		BUG();
	break;

	case CLK_OPS_LIST_RATE_RPC_CODE:
		//memcpy(&my_clk ,data+offset, 8 );
		//offset+=8;
		//memcpy(&my_rate,data+offset,sizeof(uint32_t));
		//if(pointer_is_valid((uint64_t)my_clk))
		//	out_rate=my_clk->ops->list_rate(my_clk,my_rate);
		//memcpy(&return_value,&out_rate,sizeof(long));
		BUG();
	break;
        case REGULATOR_GET_RPC_CODE:
	        memcpy(&msg_len ,data+offset,  sizeof(msg_len));
		offset+=sizeof(msg_len);	
	        memcpy(dev_id ,data+offset, msg_len);
		offset+=msg_len;
		memcpy(&msg_len ,data+offset,  sizeof(msg_len));	
		offset+=sizeof(msg_len);	
	        memcpy(con_id ,data+offset, msg_len);
		offset+=msg_len;
		
		memcpy(&msg_len ,data+offset,  sizeof(msg_len));	
		offset+=sizeof(msg_len);	

	        memcpy(compatible_name ,data+offset, msg_len);
		offset+=msg_len;

		dev_name_transform(dev_id,bullhead_name_buf);

		charm_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);

		if(IS_ERR_OR_NULL(charm_dev))
			PRINTK_ERR("charm_dev is error or null");
		my_regul = regulator_get(charm_dev,con_id);		
		add_valid_pointer((uint64_t)my_regul);
                memcpy(&return_value,&my_regul,8);
        break;

	case REGULATOR_ENABLE_RPC_CODE:	
                memcpy(&my_regul ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_regul))
                	rc=regulator_enable(my_regul);
                memcpy(&return_value,&rc,sizeof(int));
	break;

        case REGULATOR_IS_ENABLED_RPC_CODE:
                memcpy(&my_regul ,data+offset, 8 );
                if(pointer_is_valid((uint64_t)my_regul))
                        rc=regulator_is_enabled(my_regul);
                memcpy(&return_value,&rc,sizeof(int));
        break;


	case REGULATOR_DISABLE_RPC_CODE:	
                memcpy(&my_regul ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_regul))
                	regulator_disable(my_regul);
		rc=999;
                memcpy(&return_value,&rc,sizeof(int));
	break;


	case REGULATOR_PUT_RPC_CODE:	
                memcpy(&my_regul ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_regul)){
                	regulator_put(my_regul);
			remove_valid_pointer((uint64_t)my_regul);
		}
		rc=999;
                memcpy(&return_value,&rc,sizeof(int));
	break;

	case REGULATOR_SET_VOLTAGE_RPC_CODE:
                memcpy(&my_regul ,data+offset, 8 );
		offset+=8;
		memcpy(&my_vmin,data+offset,sizeof(int));
		offset+=sizeof(int);	
		memcpy(&my_vmax,data+offset,sizeof(int));
		offset+=sizeof(int);
		if(pointer_is_valid((uint64_t)my_regul))
              		 rc=regulator_set_voltage(my_regul,my_vmin,my_vmax);
               memcpy(&return_value,&rc,sizeof(int));
	break;
	case REGULATOR_SET_OPT_RPC_CODE:
                memcpy(&my_regul ,data+offset, 8 );
		offset+=8;
		memcpy(&my_iop,data+offset,sizeof(int));
		if(pointer_is_valid((uint64_t)my_regul))
              		 rc=regulator_set_optimum_mode(my_regul,my_iop);
               memcpy(&return_value,&rc,sizeof(int));
	break;
	case REGULATOR_COUNT_VOLTAGES_RPC_CODE:
               memcpy(&my_regul ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_regul))
               		rc=regulator_count_voltages(my_regul);
               memcpy(&return_value,&rc,sizeof(int));
	break;


        case RPM_REGULATOR_GET_RPC_CODE:
	        //memcpy(&msg_len ,data+offset,  sizeof(msg_len));
		//offset+=sizeof(msg_len);	
	        //memcpy(dev_id ,data+offset, msg_len);
		//offset+=msg_len;
		//memcpy(&msg_len ,data+offset,  sizeof(msg_len));	
		//offset+=sizeof(msg_len);	
	        //memcpy(con_id ,data+offset, msg_len);
		//offset+=msg_len;
		//
		//memcpy(&msg_len ,data+offset,  sizeof(msg_len));	
		//offset+=sizeof(msg_len);	

	        //memcpy(compatible_name ,data+offset, msg_len);
		//offset+=msg_len;

		//dev_name_transform(dev_id,bullhead_name_buf);

		//charm_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);


		//if(IS_ERR_OR_NULL(charm_dev))
		//	PRINTK_ERR("charm_dev is error or null");
		//my_rpm_regul =rpm_regulator_get(charm_dev,con_id);		
		//add_valid_pointer((uint64_t)my_rpm_regul);
                //memcpy(&return_value,&my_rpm_regul,8);
		BUG();
        break;

	case RPM_REGULATOR_SET_MODE_RPC_CODE:	
               // memcpy(&my_rpm_regul ,data+offset, 8 );
	       // offset+=8;
               // memcpy(&my_rpm_mode ,data+offset, sizeof(int) );
	       // if(pointer_is_valid((uint64_t)my_rpm_regul))
               // 	rc=rpm_regulator_set_mode(my_rpm_regul,my_rpm_mode);
               // memcpy(&return_value,&rc,sizeof(int));
	       BUG();
	break;

	case RPM_REGULATOR_PUT_RPC_CODE:	
                //memcpy(&my_rpm_regul ,data+offset, 8 );
		//offset+=8;
		//if(pointer_is_valid((uint64_t)my_rpm_regul)){
                //	rpm_regulator_put(my_rpm_regul);
		//	remove_valid_pointer((uint64_t)my_rpm_regul);
		//}
		//rc=999;
                //memcpy(&return_value,&rc,sizeof(int));
		BUG();
	break;



	case RPM_REGULATOR_ENABLE_RPC_CODE:	
                //memcpy(&my_rpm_regul ,data+offset, 8 );
		//offset+=8;
		//if(pointer_is_valid((uint64_t)my_rpm_regul))
                //	rc=rpm_regulator_enable(my_rpm_regul);
                //memcpy(&return_value,&rc,sizeof(int));
		BUG();
	break;
	case RPM_REGULATOR_DISABLE_RPC_CODE:	
                //memcpy(&my_rpm_regul ,data+offset, 8 );
		//offset+=8;
		//if(pointer_is_valid((uint64_t)my_rpm_regul))
                //	rc=rpm_regulator_disable(my_rpm_regul);
                //memcpy(&return_value,&rc,sizeof(int));
		BUG();
	break;

	case RPM_REGULATOR_SET_VOLTAGE_RPC_CODE:
               // memcpy(&my_rpm_regul ,data+offset, 8 );
	       // offset+=8;
	       // memcpy(&my_vmin,data+offset,sizeof(int));
	       // offset+=sizeof(int);	
	       // memcpy(&my_vmax,data+offset,sizeof(int));
	       // offset+=sizeof(int);
	       // if(pointer_is_valid((uint64_t)my_rpm_regul))
               // 	 rc=rpm_regulator_set_voltage(my_rpm_regul,my_vmin,my_vmax);
               // memcpy(&return_value,&rc,sizeof(int));
	       BUG();
	break;
	case PINCTRL_GET_RPC_CODE:
	        memcpy(&msg_len ,data+offset,  sizeof(msg_len));
		offset+=sizeof(msg_len);	
	        memcpy(dev_id ,data+offset, msg_len);
		offset+=msg_len;
		
		memcpy(&msg_len ,data+offset,  sizeof(msg_len));	
		offset+=sizeof(msg_len);	

	        memcpy(compatible_name ,data+offset, msg_len);
		offset+=msg_len;
		dev_name_transform(dev_id,bullhead_name_buf);

		charm_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);


		if(IS_ERR_OR_NULL(charm_dev))
			PRINTK_ERR("charm_dev is error or null");
		my_pinctrl = pinctrl_get(charm_dev);		
		add_valid_pointer((uint64_t)my_pinctrl);
                memcpy(&return_value,&my_pinctrl,8);
	break;
	case PINCTRL_LOOKUP_STATE_RPC_CODE:
                memcpy(&my_pinctrl ,data+offset, 8 );
                offset+=8;

	        memcpy(&msg_len ,data+offset,  sizeof(msg_len));
		offset+=sizeof(msg_len);	
	        memcpy(state_name ,data+offset, msg_len);
		offset+=msg_len;

		if(pointer_is_valid((uint64_t)my_pinctrl))
                	my_state= pinctrl_lookup_state(my_pinctrl,state_name);
		add_valid_pointer((uint64_t)my_state);
                memcpy(&return_value,&my_state,8);
	
	break;
	case PINCTRL_SELECT_STATE_RPC_CODE:
		
                memcpy(&my_pinctrl,data+offset, 8 );
		offset+=8;
                memcpy(&my_state,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_pinctrl) && pointer_is_valid((uint64_t)my_state) )
                	rc=pinctrl_select_state(my_pinctrl,my_state);
                memcpy(&return_value,&rc,sizeof(int));

	break;
	case PINCTRL_PUT_RPC_CODE:
		
                memcpy(&my_pinctrl,data+offset, 8 );
		offset+=8;
		if(pointer_is_valid((uint64_t)my_pinctrl)){
			pinctrl_put(my_pinctrl);
			remove_valid_pointer((uint64_t)my_pinctrl);
		}
	break;
	case GPIO_REQUEST_ONE_RPC_CODE:
		
                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
		offset+=sizeof(unsigned);
                memcpy(&my_gpio_flags,data+offset, sizeof(unsigned long) );
		offset+=sizeof(unsigned long);

                memcpy(&msg_len ,data+offset,  sizeof(msg_len));
                offset+=sizeof(msg_len);
                memcpy(my_gpio_label ,data+offset, msg_len);
                offset+=msg_len;

                rc=gpio_request_one(my_gpio,my_gpio_flags,my_gpio_label);
                memcpy(&return_value,&rc,sizeof(int));
	break;

	case GPIO_REQUEST_RPC_CODE:
		
                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
		offset+=sizeof(unsigned);

                memcpy(&msg_len ,data+offset,  sizeof(msg_len));
                offset+=sizeof(msg_len);
                memcpy(my_gpio_label ,data+offset, msg_len);
                offset+=msg_len;

                rc=gpio_request(my_gpio,my_gpio_label);
                memcpy(&return_value,&rc,sizeof(int));
	break;
	case GPIO_SET_VALUE_CAN_SLEEP_RPC_CODE:
		
                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
		offset+=sizeof(unsigned);
                memcpy(&my_gpio_value ,data+offset, sizeof(int) );
		offset+=sizeof(int);
                gpio_set_value_cansleep(my_gpio,my_gpio_value);
		rc=999;
                memcpy(&return_value,&rc,sizeof(int));
	break;
        case GPIO_EXPORT_RPC_CODE:

                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
                offset+=sizeof(unsigned);
                memcpy(&my_gpio_value_bool ,data+offset, sizeof(bool) );
                offset+=sizeof(bool);
                rc = gpio_export(my_gpio,my_gpio_value_bool);
                memcpy(&return_value,&rc,sizeof(int));
        break;

        case GPIO_IS_VALID_RPC_CODE:

                memcpy(&my_gpio_int,data+offset, sizeof(int) );
                offset+=sizeof(int);
                my_gpio_value_bool = gpio_is_valid(my_gpio_int);
                memcpy(&return_value,&my_gpio_value_bool,sizeof(bool));
        break;

	case GPIO_SET_VALUE_RPC_CODE:
		
                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
		offset+=sizeof(unsigned);
                memcpy(&my_gpio_value ,data+offset, sizeof(int) );
		offset+=sizeof(int);
                gpio_set_value(my_gpio,my_gpio_value);
		rc=999;
                memcpy(&return_value,&rc,sizeof(int));
	break;

	case GPIO_GET_VALUE_RPC_CODE:		
                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
		offset+=sizeof(unsigned);
                rc=gpio_get_value(my_gpio);
                memcpy(&return_value,&rc,sizeof(int));
	break;

	case GPIO_DIRECTION_INPUT_RPC_CODE:		
                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
		offset+=sizeof(unsigned);
                rc=gpio_direction_input(my_gpio);
                memcpy(&return_value,&rc,sizeof(int));
	break;
	case GPIO_DIRECTION_OUTPUT_RPC_CODE:
		
                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
		offset+=sizeof(unsigned);
                memcpy(&my_gpio_value ,data+offset, sizeof(int) );
		offset+=sizeof(int);
                rc=gpio_direction_output(my_gpio,my_gpio_value);
                memcpy(&return_value,&rc,sizeof(int));
	break;
	case GPIO_TO_IRQ_RPC_CODE:
		
                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
		offset+=sizeof(unsigned);
                rc=gpio_to_irq(my_gpio);
                memcpy(&return_value,&rc,sizeof(int));
	break;
	
	case GPIO_FREE_RPC_CODE:
		
                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
		offset+=sizeof(unsigned);
                gpio_free(my_gpio);
		rc=999;
                memcpy(&return_value,&rc,sizeof(int));
	break;

	case IRQ_TO_GPIO_RPC_CODE:
		
                memcpy(&my_gpio,data+offset, sizeof(unsigned) );
		offset+=sizeof(unsigned);
                rc=irq_to_gpio(my_gpio);
                memcpy(&return_value,&rc,sizeof(int));
	break;

	case IS_ERR_RPC_CODE:
                 memcpy(&my_ptr ,data+offset, 8 );
                 bool_rc = IS_ERR(my_ptr);
                 memcpy(&return_value,&bool_rc,sizeof(bool));
	break;
	case PTR_ERR_RPC_CODE:
                 memcpy(&my_ptr ,data+offset, 8 );
                 bool_rc = PTR_ERR(my_ptr);
                 memcpy(&return_value,&bool_rc,sizeof(bool));
	break;
	case IS_ERR_OR_NULL_RPC_CODE:
                  memcpy(&my_ptr ,data+offset, 8 );
                  long_rc = IS_ERR_OR_NULL(my_ptr);
                  memcpy(&return_value,&long_rc,sizeof(long));
	break;
	case PM_REQUEST_IDLE_RPC_CODE:
               memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               offset+=sizeof(msg_len);
               memcpy(dev_id ,data+offset, msg_len);
               offset+=msg_len;

               memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               offset+=sizeof(msg_len);

               memcpy(compatible_name ,data+offset, msg_len);
               offset+=msg_len;

               dev_name_transform(dev_id,bullhead_name_buf);


		charm_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);

               if(IS_ERR_OR_NULL(charm_dev))
                       PRINTK_ERR("charm_dev is error or null");
	       rc=pm_request_idle(charm_dev);
               memcpy(&return_value,&rc,sizeof(int));	
	break;

	case PM_RUNTIME_BARRIER_RPC_CODE:
		
               memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               offset+=sizeof(msg_len);
               memcpy(dev_id ,data+offset, msg_len);
               offset+=msg_len;

               memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               offset+=sizeof(msg_len);

               memcpy(compatible_name ,data+offset, msg_len);
               offset+=msg_len;
               dev_name_transform(dev_id,bullhead_name_buf);

	       charm_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);

               if(IS_ERR_OR_NULL(charm_dev))
                       PRINTK_ERR("charm_dev is error or null");
	       rc=pm_runtime_barrier(charm_dev);
               memcpy(&return_value,&rc,sizeof(int));	
	break;
		
	case OF_GET_GPIO_RPC_CODE:
		

               memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               offset+=sizeof(msg_len);

               memcpy(compatible_name ,data+offset, msg_len);
               offset+=msg_len;
		
	
               memcpy(&my_index ,data+offset, sizeof(int));
		offset+=sizeof(int);

		my_of_node =of_find_compatible_node(NULL,NULL,compatible_name);

	       rc=of_get_gpio(my_of_node,my_index);
               memcpy(&return_value,&rc,sizeof(int));	
	break;
	
	case OF_GET_NAMED_GPIO_RPC_CODE:
		

               memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               offset+=sizeof(msg_len);

               memcpy(compatible_name ,data+offset, msg_len);
               offset+=msg_len;
		
	
               memcpy(&my_index ,data+offset, sizeof(int));
		offset+=sizeof(int);

               memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               offset+=sizeof(msg_len);

               memcpy(gpio_name ,data+offset, msg_len);
               offset+=msg_len;

		my_of_node =of_find_compatible_node(NULL,NULL,compatible_name);

	       rc=of_get_named_gpio(my_of_node,gpio_name,my_index);
               memcpy(&return_value,&rc,sizeof(int));	
	break;
	case MSM_IOMMU_GET_CTX_RPC_CODE:
		
               //memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               //offset+=sizeof(msg_len);
               //memcpy(ctx_name ,data+offset, msg_len);
               //offset+=msg_len;
	       // my_device=msm_iommu_get_ctx(ctx_name);
	       // if(IS_ERR_OR_NULL(my_device)){
	       // 	PRINTK_ERR("iommu device is null");
	       // }
	       // add_valid_pointer((uint64_t)my_device);
	       // memcpy(&return_value,&my_device,8);
	       BUG();
		
		
	break;
	case MSM_GET_IOMMU_DOMAIN_RPC_CODE:
		
               //memcpy(&my_domain_num ,data+offset,  sizeof(int));
               //offset+=sizeof(int);
	       // my_domain=msm_get_iommu_domain(my_domain_num);
	       // add_valid_pointer((uint64_t)my_domain);
	       // memcpy(&return_value,&my_domain,8);
	       BUG();
		
		
	break;
	case MSM_REGISTER_DOMAIN_RPC:
		//rc=msm_register_domain(&vfe_layout);
		//memcpy(&return_value,&rc,sizeof(int));	
		BUG();
	break;
	case IOMMU_ATTACH_DEVICE_RPC_CODE:
		memcpy(&ptr1 ,data+offset, 8 );
		offset+=8;
		memcpy(&ptr2, data+offset, 8 );
		my_domain=(struct iommu_domain *)ptr1;
		my_device=(struct device *)ptr2;
		if (pointer_is_valid((uint64_t)my_domain) && pointer_is_valid((uint64_t)my_device))
			rc= iommu_attach_device(my_domain,my_device);
		memcpy(&return_value,&rc,sizeof(int));	

	break;

	case IOMMU_DETACH_DEVICE_RPC_CODE:

		memcpy(&ptr1 ,data+offset, 8 );
		offset+=8;
		memcpy(&ptr2, data+offset, 8 );
		my_domain=(struct iommu_domain *)ptr1;
		my_device=(struct device *)ptr2;
		if (pointer_is_valid((uint64_t)my_domain) && pointer_is_valid((uint64_t)my_device))
			iommu_detach_device(my_domain,my_device);
		rc=999;
		memcpy(&return_value,&rc,sizeof(int));	

	break;
	case MSM_ION_CLIENT_CREATE_RPC_CODE:
               // memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               // offset+=sizeof(msg_len);
               // memcpy(ctx_name ,data+offset, msg_len);
               // offset+=msg_len;
               // my_ionc=msm_ion_client_create(ctx_name);
	       // add_valid_pointer((uint64_t)my_ionc);
               // memcpy(&return_value,&my_ionc,8);
	       BUG();

	break;
	case ION_CLIENT_DESTROY_RPC_CODE:
		memcpy(&my_ionc ,data+offset, 8 );
		if(pointer_is_valid((uint64_t)my_ionc ))
			ion_client_destroy(my_ionc);
		rc=999;
		memcpy(&return_value,&rc,sizeof(int));
	break;
	case ION_FREE_RPC_CODE:	
		memcpy(&my_ionc ,data+offset, 8 );
		offset+=8;
		
		memcpy(&my_ionh ,data+offset, 8 );
		offset+=8;
		
		if(pointer_is_valid((uint64_t)my_ionc ))
			ion_free(my_ionc,my_ionh);
		rc=999;
		memcpy(&return_value,&rc,sizeof(int));
	break;
	case ION_MAP_IOMMU_RPC_CODE:
	
		//memcpy(&my_ionc ,data+offset, 8 );
		//memcpy(offset+=8;
		//memcpy(
		//memcpy(memcpy(&my_ionh ,data+offset, 8 );
		//memcpy(offset+=8;
	
		//memcpy(memcpy(&my_domain_num, data+offset, sizeof(int));
		//memcpy(offset+= sizeof(int);

		//memcpy(memcpy(&my_partition_num, data+offset, sizeof(int));
		//memcpy(offset+= sizeof(int);
		//memcpy(
		//memcpy(memcpy(&my_align, data+offset, sizeof(unsigned long));
		//memcpy(offset+= sizeof(unsigned long);
	
		//memcpy(memcpy(&my_iova_length, data+offset, sizeof(unsigned long));
		//memcpy(offset+= sizeof(unsigned long);
	
		//memcpy(memcpy(&my_iova, data+offset,8);
		//memcpy(offset+= 8;

		//memcpy(memcpy(&my_buffer_size, data+offset,8);
		//memcpy(offset+= 8;

		//memcpy(memcpy(&my_flags, data+offset, sizeof(unsigned long));
		//memcpy(offset+= sizeof(unsigned long);


		//memcpy(memcpy(&my_iommu_flags, data+offset, sizeof(unsigned long));
		//memcpy(offset+= sizeof(unsigned long);

		//memcpy(if(pointer_is_valid((uint64_t)my_ionc ))
		//memcpy(	rc=ion_map_iommu(my_ionc,my_ionh,my_domain_num,my_partition_num,my_align,my_iova_length,(ion_phys_addr_t *)my_iova,my_buffer_size,my_flags,my_iommu_flags);
		//memcpy(memcpy(&return_value,&rc,sizeof(int));
		BUG();
	break;
	case ION_UNMAP_IOMMU_RPC_CODE:

		//memcpy(&my_ionc ,data+offset, 8 );
		//offset+=8;
		//
		//memcpy(&my_ionh ,data+offset, 8 );
		//offset+=8;
	
		//memcpy(&my_domain_num, data+offset, sizeof(int));
		//offset+= sizeof(int);

		//memcpy(&my_partition_num, data+offset, sizeof(int));
		//offset+= sizeof(int);
		//

		//if(pointer_is_valid((uint64_t)my_ionc ))
		//	ion_unmap_iommu(my_ionc,my_ionh,my_domain_num,my_partition_num);
		//rc=999;
		//memcpy(&return_value,&rc,sizeof(int));
		BUG();

	break;
	case ION_IMPORT_DMA_RPC_CODE:

		memcpy(&my_ionc ,data+offset, 8 );
		offset+=8;
			
		memcpy(&my_domain_num, data+offset, sizeof(int));
		offset+= sizeof(int);

		
		if(pointer_is_valid((uint64_t)my_ionc ))
			my_ionh = ion_import_dma_buf(my_ionc,my_domain_num);
		rc=999;
		memcpy(&return_value,&my_ionh,8 );
	break;

	case EXYNOS_UPDATE_IP_IDLE_STATUS:
                memcpy(&my_int1,data+offset, sizeof(int) );
		offset+=sizeof(int);
                memcpy(&my_int2 ,data+offset, sizeof(int) );
		offset+=sizeof(int);
                exynos_update_ip_idle_status(my_int1,my_int2);
		rc=999;
                memcpy(&return_value,&rc,sizeof(int));
	break;	
	case EXYNOS_GET_IDLE_IP_INDEX:
	        memcpy(&msg_len ,data+offset,  sizeof(msg_len));
		offset+=sizeof(msg_len);	
	        memcpy(dev_id ,data+offset, msg_len);
		offset+=msg_len;
		rc=exynos_get_idle_ip_index(dev_id);
                memcpy(&return_value,&rc,sizeof(int));	
	break;
	case EXYNOS_SMC:

                memcpy(&my_ulint1,data+offset, sizeof(int) );
		offset+=sizeof(unsigned long int);
                memcpy(&my_ulint2 ,data+offset, sizeof(int) );
		offset+=sizeof(unsigned long int);
                memcpy(&my_ulint3,data+offset, sizeof(int) );
		offset+=sizeof(unsigned long int);
                memcpy(&my_ulint4 ,data+offset, sizeof(int) );
		offset+=sizeof(unsigned long int);
		rc=exynos_smc(my_ulint1,my_ulint2,my_ulint3,my_ulint4);
                memcpy(&return_value,&rc,sizeof(int));
	break;
	case REGULATOR_GET_VOLTAGE_RPC_CODE:
                memcpy(&my_regul ,data+offset, 8 );
		offset+=8;
		if(pointer_is_valid((uint64_t)my_regul))
              		 rc=regulator_get_voltage(my_regul);
               memcpy(&return_value,&rc,sizeof(int));
	default: 
		return_value=1371;
	}
 

 


	return return_value;	
}


int dev_name_transform(char *name_juno, char *name_bullhead)
{

	if( strcmp(name_juno,"f4d30000.spi") == 0 ){
		strcpy(name_bullhead,"14d30000.spi");
		goto complete;
	}

	if( strcmp(name_juno,"spi1.0") == 0 ){
		strcpy(name_bullhead,"3.BCM4773");
		goto complete;
	}
	strcpy(name_bullhead,name_juno);

complete:
	return 0;

}
