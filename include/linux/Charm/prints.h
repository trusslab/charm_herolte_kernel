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


#define PRINTK0(fmt,args...) 

#define PRINTKL(fmt,args...)// printk(KERN_ALERT"javad: %s:%d" fmt "\n",__func__,__LINE__,##args);

#define PRINTK3(fmt,args...)//  printk(KERN_ALERT"javad:" fmt "\n",##args);
#define PRINTK4(fmt,args...)//  printk(KERN_ALERT"%s: " fmt "\n",__func__,##args);
#define PRINTK5(fmt,args...)  printk(KERN_ALERT"Charm %s: " fmt "\n",__func__,##args);

#define PRINTK_ERR(fmt,args...)  printk(KERN_ALERT"Charm %s: Error: " fmt "\n",__func__,##args);
