#define PRINTK0(fmt,args...)  printk(KERN_ALERT"javad: %s" fmt "\n",__func__,##args);
//#define PRINTK0(fmt,args...) 

#define PRINTKL(fmt,args...)  printk(KERN_ALERT"javad: %s:%d" fmt "\n",__func__,__LINE__,##args);

#define PRINTK3(fmt,args...)  printk(KERN_ALERT"javad:" fmt "\n",##args);
#define PRINTK4(fmt,args...)  printk(KERN_ALERT"%s: " fmt "\n",__func__,##args);
#define PRINTK5(fmt,args...)  printk(KERN_ALERT"%s: " fmt "\n",__func__,##args);

#define PRINTK_ERR(fmt,args...)  printk(KERN_ALERT"%s: Error: " fmt "\n",__func__,##args);
