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
#include <linux/Charm/charm_irq.h>


extern int send_irq_charm(uint64_t irq);
extern int charm_began;


static irqreturn_t charm_irq_handler(int irq_num, void *data)
{
        if(charm_began){
                send_irq_charm(irq_num);
        }
        return IRQ_HANDLED;
}

void register_irq_handlers (void)
{
	int rc;
	struct platform_device * pdev_irq;
	struct device * dev_irq;

	int spi_irq; 

////irq for spi
		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"14d30000.spi" );
		pdev_irq = to_platform_device(dev_irq);
		spi_irq = platform_get_irq(pdev_irq, 0);
		rc = request_irq(spi_irq, charm_irq_handler,IRQF_TRIGGER_RISING,"spi", dev_irq);

		if (rc < 0) {
			PRINTKL("%s: spi irq request fail\n", __func__);
			BUG();
		}else{
			PRINTKL("spi irq num %d",(int)spi_irq);	
		}

}
