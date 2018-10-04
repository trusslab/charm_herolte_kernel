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

extern int acc_open(struct inode *ip, struct file *fp);
extern ssize_t acc_read(struct file *fp, char __user *buf,
	         size_t count, loff_t *pos);
extern ssize_t acc_read_2(struct file *fp, char __user *buf,
	         size_t count, loff_t *pos);
extern ssize_t acc_write(struct file *fp, const char __user *buf,
	size_t count, loff_t *pos);
extern ssize_t acc_write_2(struct file *fp, const char __user *buf,
	size_t count, loff_t *pos);
extern ssize_t acc_write_3(struct file *fp, const char __user *buf,
	size_t count, loff_t *pos);
extern int acc_release(struct inode *ip, struct file *fp);

struct file acc_fp;

ssize_t acc_read_loop(ssize_t (*acc_read_func)(struct file *, char __user *, size_t,
		loff_t *), struct file *fp, char __user *buf, size_t count, loff_t *pos);

ssize_t acc_write_loop(ssize_t (*acc_write_func)(struct file *, const char __user *,
	size_t, loff_t *), struct file *fp, const char __user *buf, size_t count, loff_t *pos);
