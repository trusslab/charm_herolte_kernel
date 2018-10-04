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

ssize_t acc_read_loop(ssize_t (*acc_read_func)(struct file *, char __user *, size_t,
		loff_t *), struct file *fp, char __user  *buf, size_t count, loff_t *pos)
{
	ssize_t ret;

	while (true) {
		ret = (*acc_read_func)(fp, buf, count, pos);
		if (ret >= 0)
			return ret;
	}
}

ssize_t acc_write_loop(ssize_t (*acc_write_func)(struct file *, const char __user *,
	size_t, loff_t *), struct file *fp, const char __user  *buf, size_t count, loff_t *pos)
{
	ssize_t ret;

	while (true) {
		ret = (*acc_write_func)(fp, buf, count, pos);
		if (ret >= 0)
			return ret;
	}
}
