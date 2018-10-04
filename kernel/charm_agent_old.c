#include <net/sock.h> 
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/kmod.h>


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/proc_fs.h>

//javad
#include <linux/spinlock.h> 
#include <linux/spinlock_types.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>

#include <linux/kthread.h>  // for threads
#include <linux/sched.h>  // for task_struct
//#include <soc/qcom/rpm-smd.h>

#include <linux/connector.h>

///#include "../drivers/media/platform/msm/camera_v2/sensor/cci/msm_cci.h"
#include <linux/prints.h>

//javad
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
///#include <linux/ksocket.h>
#include <linux/delay.h>
#include <linux/iommu.h>
//#include <linux/msm_iommu_domains.h>
#include "../drivers/staging/android/ion/ion.h"

#include <linux/reboot.h>
#include <soc/samsung/exynos-powermode.h>
#include <linux/smc.h>

int clk_reset(struct clk *clk, enum clk_reset_action action);

#define NETLINK_MJ_TEST 13
#define NETLINK_MJ_INTERRUPT 14
//#define PRINTK3(fmt, args...)  printk("javad:%s " fmt, __func__, ##args)
//javad end

//javad start
//implementing rb_tree support based on : http://chrisincs.blogspot.com/2011/05/introduction-to-linux-red-black-tree.html
#include <linux/rbtree.h>
struct unode{
       struct rb_node __rb_node;
       uint64_t __num;      //this is to record our data.
};

struct unode * rb_search_unode( struct rb_root * root , uint64_t target ){

       struct rb_node * n = root->rb_node;
       struct unode * ans;

       while( n ){
              //Get the parent struct to obtain the data for comparison
              ans = rb_entry( n , struct unode , __rb_node );

              if( target < ans->__num )
                     n = n->rb_left;
              else if( target > ans->__num )
                     n = n->rb_right;
              else
                     return ans;

       }
       return NULL;

}
struct unode * rb_insert_unode( struct rb_root * root , uint64_t target , struct rb_node * source ){

       struct rb_node **p = &root->rb_node;
       struct rb_node *parent = NULL;
       struct unode * ans;

       while( *p ){

              parent = *p;
              ans = rb_entry( parent , struct unode , __rb_node );

              if( target < ans->__num )
                     p = &(*p)->rb_left;
              else if( target > ans->__num )
                     p = &(*p)->rb_right;
              else
                     return ans;

       }
       rb_link_node( source , parent , p );             //Insert this new node as a red leaf.
       rb_insert_color( source , root );           //Rebalance the tree, finish inserting
       return NULL;

}

void rb_erase_unode( struct rb_node * source , struct rb_root * root ){

       struct unode * target;
      
       target = rb_entry( source , struct unode , __rb_node );
       rb_erase( source , root );                           //Erase the node
       kfree( target );                                     //Free the memory

}

struct rb_root pointers_rb_root = RB_ROOT;
//javad end
//------------------------------------------using rb_tree for chekcing valid pointers
//uint64_t valid_pointers[4096];
//int vp_write_index=0;

void add_valid_pointer(uint64_t ptr)
{
//	if(vp_write_index>=4096)
//		BUG();
//	valid_pointers[vp_write_index]=ptr;
//	vp_write_index++;
	struct unode * node;	
	struct unode * found_node=NULL;
	node = ( struct unode * )kmalloc( sizeof( struct unode ),GFP_KERNEL);
	found_node = rb_insert_unode( &pointers_rb_root, ptr , &node->__rb_node ); 
	if(found_node==NULL){
		node->__num = ptr;	
	}else{
		kfree( node );
	}
}

void remove_valid_pointer(uint64_t ptr){
	
	struct unode * found_node=NULL;
	found_node=rb_search_unode( &pointers_rb_root , ptr );
	if(found_node!=NULL){
		rb_erase_unode( &found_node->__rb_node , &pointers_rb_root );
	}
}

int pointer_is_valid(uint64_t ptr)
{
//	int i;
//	for (i=0; i<vp_write_index;i++){
//		if (ptr==valid_pointers[i]){
//			return 1;
//		}
//	}
//	return 0;	
	struct unode * found_node=NULL;
	found_node=rb_search_unode( &pointers_rb_root , ptr );
	if(found_node==NULL){
		return 0;
	}else{
		return 1;
	}
}
#define READB_OPCODE 1
#define READW_OPCODE 2
#define READL_OPCODE 3
#define READQ_OPCODE 4
#define WRITEB_OPCODE 6
#define WRITEW_OPCODE 7
#define WRITEL_OPCODE 8
#define WRITEQ_OPCODE 9


#define TLMM_GP_FUNC_SHFT               2
#define TLMM_GP_FUNC_MASK               0xF

int charm_began=0;
int flag_for_pinctrl_print;
//static struct cb_id cn_reg_id = { CN_NETLINK_USERS + 3, 0x456 };
//static char cn_reg_name[] = "cn_reg";


//static struct cb_id cn_irq_id = { CN_NETLINK_USERS + 4, 0x567 };
//static char cn_irq_name[] = "cn_irq";

//static struct cb_id cn_rpc_id = { CN_NETLINK_USERS + 5, 0x667 };
//static char cn_rpc_name[] = "cn_rpc";

#define IRQ_TCP_PORT 33000
#define RPC_TCP_PORT 20000
#define REG_TCP_PORT 21000

#define PAYLOAD_SIZE_REG 17
#define PAYLOAD_SIZE_RPC 256

//extern struct msm_rpm_request *javad_vio_active_handle;
extern int send_fn_ptr(int(*)(uint64_t));
//extern int send_fn_ptr(int (*)(uint64_t));
  
extern struct bus_type platform_bus_type;
///extern struct cci_device * mj_msm_cci_devices1;
///extern struct cci_device * mj_msm_cci_devices0;
struct device *javad_bus_find_device_by_name(struct bus_type *bus,
                                        struct device *start, const char *name);

uint64_t reg_phys_base[64];
uint64_t reg_phys_size[64];


//int num_reg_addrs=9;
//int num_reg_addrs=3;
int num_reg_addrs=1;

//uint64_t reg_ptr_1, reg_ptr_2;
uint64_t reg_ptrs[64];
//javad end
int acc_open(struct inode *ip, struct file *fp);
ssize_t acc_read(struct file *fp, char __user *buf,
	         size_t count, loff_t *pos);
ssize_t acc_read_2(struct file *fp, char __user *buf,
	         size_t count, loff_t *pos);
ssize_t acc_write(struct file *fp, const char __user *buf,
	size_t count, loff_t *pos);
ssize_t acc_write_2(struct file *fp, const char __user *buf,
	size_t count, loff_t *pos);
ssize_t acc_write_3(struct file *fp, const char __user *buf,
	size_t count, loff_t *pos);
int acc_release(struct inode *ip, struct file *fp);

struct file acc_fp;

static ssize_t acc_read_loop(ssize_t (*acc_read_func)(struct file *, char __user *, size_t,
		loff_t *), struct file *fp, char __user *buf, size_t count, loff_t *pos)
{
	ssize_t ret;

	while (true) {
		ret = (*acc_read_func)(fp, buf, count, pos);
		if (ret >= 0)
			return ret;
	}
}

static ssize_t acc_write_loop(ssize_t (*acc_write_func)(struct file *, const char __user *,
	size_t, loff_t *), struct file *fp, const char __user *buf, size_t count, loff_t *pos)
{
	ssize_t ret;

	while (true) {
		ret = (*acc_write_func)(fp, buf, count, pos);
		if (ret >= 0)
			return ret;
	}
}

//void cn_reg_callback(struct cn_msg *msg, struct netlink_skb_parms *nsp)
static bool reg_callback(void *data, uint64_t *return_value)
{		
	//int res;
	char *opcode;
	uint64_t address;
	uint64_t *value;
	//uint64_t return_value;	 
	void *ptr;
	bool is_read = false;
	bool address_is_valid = false;
	int i=0;
	//return 0;
//	void __iomem *cfg_reg;
//	unsigned int pinctrl_val;
//	u32 func =1;
//	int ret;
//	int pin;
	if(charm_began==0) {

	//	javad_bus_find_device_by_name(&platform_bus_type,NULL, "javad");

	}
	charm_began=1;
//	printk(KERN_INFO "Entering: %s\n", __FUNCTION__);

	//memcpy(&opcode, data, sizeof(char));  	
	//memcpy(&address, data + sizeof(char), sizeof(uint64_t)); 
	//memcpy(&value, data + sizeof(char) + sizeof(uint64_t), sizeof(uint64_t));
	opcode = data;  	
	address = *((uint64_t *) (data + sizeof(char))); 

//	printk(KERN_ALERT"javad: value recieved from userspacei opcode=0x%x, address=0x%lx \n",opcode,(long unsigned int)address);
//	ptr= ioremap(address,64); i dont know why i used 64 let me try 1 for size
	//ptr = ioremap(*address,1);
//javad start
//	if ((address >= reg_phys_base_1) && (address < (reg_phys_base_1 + reg_phys_size_1))) {
//		ptr = (void *) (reg_ptr_1 + (address - reg_phys_base_1));
//	} else if ((address >= reg_phys_base_2) && (address < (reg_phys_base_2 + reg_phys_size_2))) {
//		ptr = (void *) (reg_ptr_2 + (address - reg_phys_base_2));
//	} else {
//		/* FIXME: This is not correct error handling */
//		PRINTK_ERR("Invalid address");
//		return false;
//	}


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
//javad end

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
		//if(address==0xfda0cc0c){
		//	writel(return_value,ioremap(0xfda0cc08,1));
		//	writel(1,ioremap(0xfda0cc00,1));
		//	writel(0,ioremap(0xfda0cc00,1));

		//}
	break;

	case READQ_OPCODE:
		*return_value=(uint64_t)readq(ptr);
		is_read = true;
	break;	

	case WRITEB_OPCODE:
	        value = data + sizeof(char) + sizeof(uint64_t);
		writeb(*value,ptr);
		//return_value=0;
	break;	

	case WRITEW_OPCODE:
	        value = data + sizeof(char) + sizeof(uint64_t);
		writew(*value,ptr);
		//return_value=0;
	break;

	case WRITEL_OPCODE:
	        value = data + sizeof(char) + sizeof(uint64_t);
		writel(*value,ptr);
		//return_value=0;
	break;

	case WRITEQ_OPCODE:
	        value = data + sizeof(char) + sizeof(uint64_t);
		writeq(*value,ptr);	
		//return_value=0;
	break;	
	}
//	printk(KERN_ALERT"Actual reg value is: 0x%lx\n",(long unsigned int)return_value);
	//   return_value=1326;
	//msg->len=sizeof(uint64_t);
	//memcpy(msg->data,&return_value,sizeof(uint64_t));	
	//res= cn_netlink_send(msg, 0, GFP_ATOMIC);
	//if (res < 0)
	//printk(KERN_INFO "Error while sending back to user\n");

	return is_read;
}


//void cn_irq_callback(struct cn_msg *msg, struct netlink_skb_parms *nsp)
//{	
//}

#define CLK_GET_RPC_CODE 1
#define CLK_ROUND_RATE_RPC_CODE 2
#define CLK_SET_RATE_RPC_CODE 3
#define CLK_GET_RATE_RPC_CODE 4
#define CLK_PREPARE_RPC_CODE 5
#define CLK_ENABLE_RPC_CODE 6
#define REGULATOR_GET_RPC_CODE 7
#define REGULATOR_ENABLE_RPC_CODE 8
#define REGULATOR_SET_VOLTAGE_RPC_CODE 9
#define REGULATOR_SET_OPT_RPC_CODE 10
#define REGULATOR_COUNT_VOLTAGES_RPC_CODE 11
#define PINCTRL_GET_RPC_CODE 12
#define PINCTRL_LOOKUP_STATE_RPC_CODE 13
#define PINCTRL_SELECT_STATE_RPC_CODE 14
#define GPIO_REQUEST_ONE_RPC_CODE 15
#define IS_ERR_RPC_CODE 16
#define PINCTRL_PUT_RPC_CODE 17
#define  IS_ERR_OR_NULL_RPC_CODE 18
#define PM_REQUEST_IDLE_RPC_CODE 19
#define PM_RUNTIME_BARRIER_RPC_CODE 20
#define OF_GET_GPIO_RPC_CODE 21
#define OF_GET_NAMED_GPIO_RPC_CODE 22
#define CLK_DISABLE_RPC_CODE 23
#define CLK_UNPREPARE_RPC_CODE 24
#define CLK_PUT_RPC_CODE 25
#define REGULATOR_DISABLE_RPC_CODE 26
#define REGULATOR_PUT_RPC_CODE 27
#define CLK_SET_PARENT_RPC_CODE 28
#define GPIO_SET_VALUE_CAN_SLEEP_RPC_CODE 29
#define RPM_REGULATOR_GET_RPC_CODE 30
#define RPM_REGULATOR_SET_MODE_RPC_CODE 31
#define RPM_REGULATOR_PUT_RPC_CODE 32
#define RPM_REGULATOR_ENABLE_RPC_CODE 33
#define RPM_REGULATOR_DISABLE_RPC_CODE 34
#define RPM_REGULATOR_SET_VOLTAGE_RPC_CODE 35
#define GPIO_REQUEST_RPC_CODE 36
#define GPIO_SET_VALUE_RPC_CODE 37
#define GPIO_GET_VALUE_RPC_CODE 38
#define GPIO_TO_IRQ_RPC_CODE 39
#define IRQ_TO_GPIO_RPC_CODE 40
#define GPIO_DIRECTION_OUTPUT_RPC_CODE 41
#define GPIO_DIRECTION_INPUT_RPC_CODE 42

#define CLK_RESET_RPC_CODE 43
#define MSM_IOMMU_GET_CTX_RPC_CODE 44
#define MSM_GET_IOMMU_DOMAIN_RPC_CODE 45
#define MSM_REGISTER_DOMAIN_RPC 46
#define IOMMU_ATTACH_DEVICE_RPC_CODE 47
#define IOMMU_DETACH_DEVICE_RPC_CODE 48
#define MSM_ION_CLIENT_CREATE_RPC_CODE 49
#define ION_CLIENT_DESTROY_RPC_CODE 50
#define ION_FREE_RPC_CODE 51
#define ION_MAP_IOMMU_RPC_CODE 52
#define ION_UNMAP_IOMMU_RPC_CODE 53
#define ION_IMPORT_DMA_RPC_CODE 54
#define GPIO_FREE_RPC_CODE 55
#define PTR_ERR_RPC_CODE 56

#define CLK_OPS_LIST_RATE_RPC_CODE 61

#define REGULATOR_IS_ENABLED_RPC_CODE 62
#define CLK_GET_PARENT_RPC_CODE 63
#define CLK_GET_SYS_RPC_CODE 64
#define GPIO_EXPORT_RPC_CODE 65
#define GPIO_IS_VALID_RPC_CODE 66
	 
/* Exynos */
#define EXYNOS_UPDATE_IP_IDLE_STATUS 67
#define EXYNOS_GET_IDLE_IP_INDEX 68
#define EXYNOS_SMC 69

#define REGULATOR_GET_VOLTAGE_RPC_CODE 70

#define REBOOT_RPC_CODE 100

int class_exist=0;
int device_exist=0;
struct device * javad_dev;
struct class * javad_class;


int send_irq_charm(uint64_t irq);

int dev_name_transform(char *name_juno, char *name_bullhead)
{
	//if( strcmp(name_juno,"soc:stmvl6180@0") == 0 ){
	//	strcpy(name_bullhead,"stmvl6180.84");
	//	goto complete;
	//}


	//if( strcmp(name_juno,"soc:qcom,camera-flash") == 0 ){
	//	strcpy(name_bullhead,"qcom,camera-flash.83");
	//	goto complete;
	//}

	//if( strcmp(name_juno,"fda0c000.qcom,cci:qcom,eeprom@5") == 0 ){
	//	strcpy(name_bullhead,"5.qcom,eeprom");
	//	goto complete;
	//}

	//if( strcmp(name_juno,"fda0c000.qcom,cci:qcom,eeprom@a0") == 0 ){
	//	strcpy(name_bullhead,"a0.qcom,eeprom");
	//	goto complete;
	//}
	//if( strcmp(name_juno,"fda0c000.qcom,cci:qcom,actuator@0") == 0 ){
	//	strcpy(name_bullhead,"0.qcom,actuator");
	//	goto complete;
	//}
	//
	//if( strcmp(name_juno,"fda0c000.qcom,cci:qcom,camera@0") == 0 ){
	//	strcpy(name_bullhead,"0.qcom,camera");
	//	goto complete;
	//}

	//if( strcmp(name_juno,"fda0c000.qcom,cci:lge,camera1@6c") == 0 ){
	//	strcpy(name_bullhead,"6c.lge,camera1");
	//	goto complete;
	//}

	//if( strcmp(name_juno,"soc:wcd9xxx-irq") == 0 ){
	//	strcpy(name_bullhead,"wcd9xxx-irq.34");
	//	goto complete;
	//}

	//if( strcmp(name_juno,"tomtom-slim-pgd") == 0 ){
	//	strcpy(name_bullhead,"tomtom_codec.35");
	//	goto complete;
	//}

	//if( strcmp(name_juno,"soc:qcom,kgsl-busmon") == 0 ){
	//	strcpy(name_bullhead,"qcom,kgsl-busmon.69");
	//	goto complete;
	//}

	//if( strcmp(name_juno,"soc:qcom,gpubw") == 0 ){
	//	strcpy(name_bullhead,"qcom,gpubw.70");
	//	goto complete;
	//}
	
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
	PRINTKL(" name_bullhead= %s",name_bullhead);
	return 0;

}

///static irqreturn_t charm_msm_cci_irq(int irq_num, void *data)
///{
///        if(charm_began){
///                send_irq_charm(irq_num);
///        }
///
///        return IRQ_HANDLED;
///}
///
///
///
///static irqreturn_t charm_msm_csiphy_irq(int irq_num, void *data)
///{
///        if(charm_began){
///                send_irq_charm(irq_num);
///        }
///
///        return IRQ_HANDLED;
///}
///
///static irqreturn_t charm_msm_csid_irq(int irq_num, void *data)
///{
///        if(charm_began){
///                send_irq_charm(irq_num);
///        }
///
///        return IRQ_HANDLED;
///}
///
///
///static irqreturn_t charm_msm_ispif_irq(int irq_num, void *data)
///{
///        if(charm_began){
///                send_irq_charm(irq_num);
///        }
///
///        return IRQ_HANDLED;
///}
///
///static irqreturn_t charm_msm_vfe_irq(int irq_num, void *data)
///{
///        if(charm_began){
///                send_irq_charm(irq_num);
///        }
///
///        return IRQ_HANDLED;
///}
///
///static irqreturn_t charm_msm_cpp_irq(int irq_num, void *data)
///{
///        if(charm_began){
///                send_irq_charm(irq_num);
///        }
///
///        return IRQ_HANDLED;
///}
///
///
///static irqreturn_t charm_ngd_slim_interrupt(int irq_num, void *data)
///{
///        if(charm_began){
///                send_irq_charm(irq_num);
///        }
///	PRINTKL("slim_ngd hereee!");
///
///        return IRQ_HANDLED;
///}
///
///
///static irqreturn_t charm_wcdxxx_irq_interrupt(int irq_num, void *data)
///{
///        if(charm_began){
///                send_irq_charm(irq_num);
///        }
///	PRINTKL("wcdxxx_irq");
///
///        return IRQ_HANDLED;
///}
//javad
//static irqreturn_t charm_kgsl_interrupt(int irq_num, void *data)
//{
//        if(charm_began){
//                send_irq_charm(irq_num);
//        }
//
//        return IRQ_HANDLED;
//}
//static irqreturn_t charm_bwmon_interrupt(int irq_num, void *data)
//{
//        if(charm_began){
//		//javad do not deliver this interrupt for now
//                //send_irq_charm(irq_num);
//		PRINTKL("not delivering ");
//        }
//
//        return IRQ_HANDLED;
//}
static irqreturn_t charm_spi_interrupt(int irq_num, void *data)
{
        if(charm_began){
                send_irq_charm(irq_num);
        }

        return IRQ_HANDLED;
}
//extern struct bus_type slimbus_type;
//void cn_rpc_callback(struct cn_msg *msg, struct netlink_skb_parms *nsp)
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
	char ctx_name[128];
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
	int my_rpm_mode;
	int my_domain_num;
	int my_partition_num;
	unsigned long my_align;
	unsigned long my_iova_length;
	unsigned long * my_iova;
	unsigned long * my_buffer_size;
	unsigned long my_flags;
	unsigned long my_iommu_flags;
	void * my_ptr;
	bool bool_rc;
	struct clk * my_clk;
	int my_clk_action;
	struct clk * my_clk_parent;
	
	int my_int1,my_int2;
	unsigned long my_ulint1,my_ulint2,my_ulint3,my_ulint4;

	struct regulator * my_regul;
	//struct rpm_regulator * my_rpm_regul;
	struct pinctrl * my_pinctrl;
	struct pinctrl_state * my_state;
	unsigned long my_rate;
	long out_rate;
	long long_rc;
	unsigned my_gpio;
	unsigned long my_gpio_flags;
        //struct msm_iova_partition vfe_partition = {
        //        .start = SZ_128K,
        //        .size = SZ_2G - SZ_128K,
        //};
        //struct msm_iova_layout vfe_layout = {
        //        .partitions = &vfe_partition,
        //        .npartitions = 1,
        //        .client_name = "vfe",
        //        .domain_flags = 0,
        //};

	uint64_t ptr1;
	uint64_t ptr2;
	
	struct ion_client * my_ionc;
	struct ion_handle * my_ionh;
	//for irq handeling
	struct platform_device * pdev_irq;
	struct device * dev_irq;
	///struct resource * cci_irq;	
	///struct resource * csiphy_irq1;
	///struct resource * csiphy_irq2;
	///struct resource * csiphy_irq3;

	///struct resource * csid_irq1;
	///struct resource * csid_irq2;
	///struct resource * csid_irq3;
	///struct resource * csid_irq4;


	///struct resource * vfe_irq; 
	///struct resource * vfe_irq1; 
	///struct resource * cpp_irq; 
	///struct resource * ispif_irq;


	///struct resource * slim_irq; 
	///struct resource * wcdxxx_irq; 
	//struct resource * kgsl_irq; 
	int spi_irq; 
	int bwmon_irq; 

        if(charm_began == 0) {
/*
			javad_bus_find_device_by_name(&slimbus_type, NULL, "javad");
			return 0;
*/
/*
      			javad_bus_find_device_by_name(&platform_bus_type,NULL, "javad");
			mdelay(100000); 
		return 0;
*/

//////irq for cci
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda0c000.qcom,cci" );
///		pdev_irq = to_platform_device(dev_irq);
///		rc = of_platform_populate(dev_irq->of_node, NULL, NULL, dev_irq);
///		if (rc)
///			PRINTKL("%s: failed to add child nodes, rc=%d\n", __func__, rc);
///		cci_irq = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "cci");	
///	        rc = request_irq(cci_irq->start, charm_msm_cci_irq,
///			IRQF_TRIGGER_RISING, "cci", dev_irq);
///		
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("cci irq num %d",(int)cci_irq->start);
///		}		
///		
///////irq for csiphy1
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda0ac00.qcom,csiphy" );
///		pdev_irq = to_platform_device(dev_irq);
///		csiphy_irq1 = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "csiphy");	
///	        rc = request_irq(csiphy_irq1->start, charm_msm_csiphy_irq,
///			IRQF_TRIGGER_RISING, "csiphy", dev_irq);
///		
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("csiphy irq1 num %d",(int)csiphy_irq1->start);	
///		}
///
///
///////irq for csiphy2
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda0b000.qcom,csiphy" );
///		pdev_irq = to_platform_device(dev_irq);
///		csiphy_irq2 = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "csiphy");	
///	        rc = request_irq(csiphy_irq2->start, charm_msm_csiphy_irq,
///			IRQF_TRIGGER_RISING, "csiphy", dev_irq);
///		
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("csiphy irq2 num %d",(int)csiphy_irq2->start);	
///		}
///
///	
///////irq for csiphy3
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda0b400.qcom,csiphy" );
///		pdev_irq = to_platform_device(dev_irq);
///		csiphy_irq3 = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "csiphy");	
///	        rc = request_irq(csiphy_irq3->start, charm_msm_csiphy_irq,
///			IRQF_TRIGGER_RISING, "csiphy", dev_irq);
///		
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("csiphy irq3 num %d",(int)csiphy_irq3->start);	
///		}
///
///
///////irq for csid1
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda08000.qcom,csid" );
///		pdev_irq = to_platform_device(dev_irq);
///		csid_irq1 = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "csid");	
///	        rc = request_irq(csid_irq1->start, charm_msm_csid_irq,
///			IRQF_TRIGGER_RISING, "csid", dev_irq);
///		
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("csid irq1 num %d",(int)csid_irq1->start);	
///		}
///
///////irq for csid2
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda08400.qcom,csid" );
///		pdev_irq = to_platform_device(dev_irq);
///		csid_irq2 = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "csid");	
///	        rc = request_irq(csid_irq2->start, charm_msm_csid_irq,
///			IRQF_TRIGGER_RISING, "csid", dev_irq);
///		
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("csid irq2 num %d",(int)csid_irq2->start);	
///		}
///
///
///////irq for csid3
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda08800.qcom,csid" );
///		pdev_irq = to_platform_device(dev_irq);
///		csid_irq3 = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "csid");	
///	        rc = request_irq(csid_irq3->start, charm_msm_csid_irq,
///			IRQF_TRIGGER_RISING, "csid", dev_irq);
///		
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("csid irq3 num %d",(int)csid_irq3->start);	
///		}
///
///////irq for csid4
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda08c00.qcom,csid" );
///		pdev_irq = to_platform_device(dev_irq);
///		csid_irq4 = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "csid");	
///	        rc = request_irq(csid_irq4->start, charm_msm_csid_irq,
///			IRQF_TRIGGER_RISING, "csid", dev_irq);
///		
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("csid irq3 num %d",(int)csid_irq4->start);	
///		}
///
///
///////irq for ispif
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda0a000.qcom,ispif" );
///		pdev_irq = to_platform_device(dev_irq);
///		ispif_irq = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "ispif");	
///	        rc = request_irq(ispif_irq->start, charm_msm_ispif_irq,
///			IRQF_TRIGGER_RISING, "ispif", dev_irq);
///		
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("ispif irq num %d",(int)ispif_irq->start);	
///		}
///////irq for vfe0
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda10000.qcom,vfe" );
///		pdev_irq = to_platform_device(dev_irq);
///		vfe_irq = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "vfe");	
///	        rc = request_irq(vfe_irq->start, charm_msm_vfe_irq,
///			IRQF_TRIGGER_RISING, "vfe", dev_irq);
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("vfe irq num %d",(int)vfe_irq->start);	
///		}
///
///
///
///////irq for vfe1
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda14000.qcom,vfe" );
///		pdev_irq = to_platform_device(dev_irq);
///		vfe_irq1 = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "vfe");	
///	        rc = request_irq(vfe_irq1->start, charm_msm_vfe_irq,
///			IRQF_TRIGGER_RISING, "vfe", dev_irq);
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("vfe irq1 num %d",(int)vfe_irq1->start);	
///		}
///
///////irq for cpp
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fda04000.qcom,cpp" );
///		pdev_irq = to_platform_device(dev_irq);
///		cpp_irq = platform_get_resource_byname(pdev_irq,IORESOURCE_IRQ, "cpp");	
///	        rc = request_irq(cpp_irq->start, charm_msm_cpp_irq,
///			IRQF_TRIGGER_RISING, "cpp", dev_irq);
///		if (rc < 0) {
///			PRINTKL("%s: irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("cpp irq num %d",(int)cpp_irq->start);	
///		}
///
///
///////irq for slim
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fe12f000.slim" );
///		pdev_irq = to_platform_device(dev_irq);
///	        slim_irq = platform_get_resource_byname(pdev_irq, IORESOURCE_IRQ,"slimbus_irq");
///
///		//rc = request_irq(slim_irq->start,charm_ngd_slim_interrupt,IRQF_TRIGGER_HIGH,"ngd_slim_irq", dev_irq);
///		rc = request_irq(slim_irq->start,charm_ngd_slim_interrupt,IRQF_TRIGGER_RISING,"ngd_slim_irq", dev_irq);
///
///		if (rc < 0) {
///			PRINTKL("%s: slim ngd irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("slim ngd irq num %d",(int)slim_irq->start);	
///		}
///
///
///////irq for wcdxxx
///		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"wcd9xxx-irq.34" );
///		pdev_irq = to_platform_device(dev_irq);
///	        wcdxxx_irq = platform_get_resource_byname(pdev_irq, IORESOURCE_IRQ,"cdc-int");
///
///		//rc = request_irq(slim_irq->start,charm_ngd_slim_interrupt,IRQF_TRIGGER_HIGH,"ngd_slim_irq", dev_irq);
///		rc = request_irq(wcdxxx_irq->start,charm_wcdxxx_irq_interrupt,IRQF_TRIGGER_RISING,"wcd9xxx", dev_irq);
///
///		if (rc < 0) {
///			PRINTKL("%s: wcdxxxx irq request fail\n", __func__);
///			BUG();
///		}else{
///			PRINTKL("wcdxxxx irq num %d",(int)wcdxxx_irq->start);	
///		

////irq for kgsl
		//dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fdb00000.qcom,kgsl-3d0" );
		//pdev_irq = to_platform_device(dev_irq);
	        //kgsl_irq = platform_get_resource_byname(pdev_irq, IORESOURCE_IRQ,"kgsl_3d0_irq");

		////rc = request_irq(slim_irq->start,charm_ngd_slim_interrupt,IRQF_TRIGGER_HIGH,"ngd_slim_irq", dev_irq);
		//rc = request_irq(kgsl_irq->start,charm_kgsl_interrupt,IRQF_TRIGGER_RISING,"kgsl-3d0", dev_irq);

		//if (rc < 0) {
		//	PRINTKL("%s: kgsl irq request fail\n", __func__);
		//	BUG();
		//}else{
		//	PRINTKL("kgsl irq num %d",(int)kgsl_irq->start);	
		//}



////irq for bwmon
		//dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"fc390000.qcom,gpu-bwmon" );
		//pdev_irq = to_platform_device(dev_irq);
		//bwmon_irq= platform_get_irq(pdev_irq, 0);
	        ////kgsl_irq = platform_get_resource_byname(pdev_irq, IORESOURCE_IRQ,"kgsl_3d0_irq");

		////rc = request_irq(slim_irq->start,charm_ngd_slim_interrupt,IRQF_TRIGGER_HIGH,"ngd_slim_irq", dev_irq);
		//rc = request_irq(bwmon_irq,charm_bwmon_interrupt,IRQF_ONESHOT | IRQF_SHARED,"bwmon", dev_irq);

		//if (rc < 0) {
		//	PRINTKL("%s: bwmon irq request fail\n", __func__);
		//	BUG();
		//}else{
		//	PRINTKL("bwmon irq num %d",(int)bwmon_irq);	
		//}

////irq for spi
		dev_irq = bus_find_device_by_name(&platform_bus_type,NULL,"14d30000.spi" );
		pdev_irq = to_platform_device(dev_irq);
		spi_irq = platform_get_irq(pdev_irq, 0);
	        //kgsl_irq = platform_get_resource_byname(pdev_irq, IORESOURCE_IRQ,"kgsl_3d0_irq");

		//rc = request_irq(slim_irq->start,charm_ngd_slim_interrupt,IRQF_TRIGGER_HIGH,"ngd_slim_irq", dev_irq);
		rc = request_irq(spi_irq, charm_spi_interrupt,IRQF_TRIGGER_RISING,"spi", dev_irq);

		if (rc < 0) {
			PRINTKL("%s: spi irq request fail\n", __func__);
			BUG();
		}else{
			PRINTKL("spi irq num %d",(int)spi_irq);	
		}

        }



        charm_began=1;

	memset(bullhead_name_buf,0,128);
	offset=0;
	memcpy(&opcode , data, sizeof(opcode));
	offset += sizeof(opcode);
	PRINTKL("rpc recieved opcode = %ld",(unsigned long)opcode);  	
	switch(opcode){
	case REBOOT_RPC_CODE:
                PRINTKL("***************************\n*********************\n****** reset rpc recieved opcode = %ld",(unsigned long)opcode);    
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

		PRINTKL(" CLK_GET_RPC_CODE  dev_id=%s,con_id=%s,compatible_name=%s\n",dev_id,con_id,compatible_name);
		dev_name_transform(dev_id,bullhead_name_buf);

//javad
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
/*
		if( strcmp(bullhead_name_buf,"tomtom-slim-pgd")&&strcmp(bullhead_name_buf,"tomtom-slim-ifd") ){
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
		}else{	
			javad_dev = bus_find_device_by_name(&slimbus_type,NULL, bullhead_name_buf);
		}
*/
//javad end

		if(IS_ERR_OR_NULL(javad_dev))
			PRINTKL("javad_dev is error or null");
		my_clk = clk_get(javad_dev,con_id);		
		PRINTKL("clk ptr= %lx",(unsigned long)my_clk);
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

                memcpy(&my_clk ,data+offset, 8 );
		offset+=8;
		memcpy(&my_clk_action,data+offset,sizeof(unsigned long));
		//if(pointer_is_valid((uint64_t)my_clk))
		//	rc=clk_reset(my_clk,my_clk_action);	
		BUG();
                memcpy(&return_value,&rc,sizeof(int));
	break;

	case CLK_OPS_LIST_RATE_RPC_CODE:
		memcpy(&my_clk ,data+offset, 8 );
		offset+=8;
		memcpy(&my_rate,data+offset,sizeof(uint32_t));
		//if(pointer_is_valid((uint64_t)my_clk))
		//	out_rate=my_clk->ops->list_rate(my_clk,my_rate);
		BUG();
		memcpy(&return_value,&out_rate,sizeof(long));
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

		PRINTKL("REGULATOR_GET_RPC_CODE dev_id=%s,con_id=%s,compatible_name=%s\n",dev_id,con_id,compatible_name);
		dev_name_transform(dev_id,bullhead_name_buf);

//javad
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
/*
		if( strcmp(bullhead_name_buf,"tomtom-slim-pgd")&&strcmp(bullhead_name_buf,"tomtom-slim-ifd") ){
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
		}else{	
			javad_dev = bus_find_device_by_name(&slimbus_type,NULL, bullhead_name_buf);
		}
*/
//javad end
		if(IS_ERR_OR_NULL(javad_dev))
			PRINTKL("javad_dev is error or null");
		PRINTKL("ofnodeptr=%lx",(unsigned long)(javad_dev->of_node));	
		PRINTKL("name %s, fullname %s, type %s",javad_dev->of_node->name,javad_dev->of_node->full_name,javad_dev->of_node->type);
		my_regul = regulator_get(javad_dev,con_id);		
		PRINTKL("regul ptr= %lx",(unsigned long)my_regul);
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

		PRINTKL("RPM_REGULATOR_GET_RPC_CODE dev_id=%s,con_id=%s,compatible_name=%s\n",dev_id,con_id,compatible_name);
		dev_name_transform(dev_id,bullhead_name_buf);

//javad
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
/*
		if( strcmp(bullhead_name_buf,"tomtom-slim-pgd")&&strcmp(bullhead_name_buf,"tomtom-slim-ifd") ){
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
		}else{	
			javad_dev = bus_find_device_by_name(&slimbus_type,NULL, bullhead_name_buf);
		}
*/
//javad end


		if(IS_ERR_OR_NULL(javad_dev))
			PRINTKL("javad_dev is error or null");
		PRINTKL("ofnodeptr=%lx",(unsigned long)(javad_dev->of_node));	
		PRINTKL("name %s, fullname %s, type %s",javad_dev->of_node->name,javad_dev->of_node->full_name,javad_dev->of_node->type);
		//my_rpm_regul =rpm_regulator_get(javad_dev,con_id);		
		//PRINTKL("regul ptr= %lx",(unsigned long)my_rpm_regul);
		//add_valid_pointer((uint64_t)my_rpm_regul);
                //memcpy(&return_value,&my_rpm_regul,8);
		BUG();
        break;

	case RPM_REGULATOR_SET_MODE_RPC_CODE:	
                //memcpy(&my_rpm_regul ,data+offset, 8 );
		//offset+=8;
                //memcpy(&my_rpm_mode ,data+offset, sizeof(int) );
		//if(pointer_is_valid((uint64_t)my_rpm_regul))
                //	rc=rpm_regulator_set_mode(my_rpm_regul,my_rpm_mode);
		BUG();
                //memcpy(&return_value,&rc,sizeof(int));
	break;

	case RPM_REGULATOR_PUT_RPC_CODE:	
                //memcpy(&my_rpm_regul ,data+offset, 8 );
		//offset+=8;
		//if(pointer_is_valid((uint64_t)my_rpm_regul)){
                //	rpm_regulator_put(my_rpm_regul);
		//	remove_valid_pointer((uint64_t)my_rpm_regul);
		//}
		BUG();
		rc=999;
                memcpy(&return_value,&rc,sizeof(int));
	break;



	case RPM_REGULATOR_ENABLE_RPC_CODE:	
                //memcpy(&my_rpm_regul ,data+offset, 8 );
		//offset+=8;
		//if(pointer_is_valid((uint64_t)my_rpm_regul))
                //	rc=rpm_regulator_enable(my_rpm_regul);
		BUG();
                memcpy(&return_value,&rc,sizeof(int));
	break;
	case RPM_REGULATOR_DISABLE_RPC_CODE:	
                //memcpy(&my_rpm_regul ,data+offset, 8 );
		//offset+=8;
		//if(pointer_is_valid((uint64_t)my_rpm_regul))
                //	rc=rpm_regulator_disable(my_rpm_regul);
		BUG();
                memcpy(&return_value,&rc,sizeof(int));
	break;

	case RPM_REGULATOR_SET_VOLTAGE_RPC_CODE:
                //memcpy(&my_rpm_regul ,data+offset, 8 );
		//offset+=8;
		//memcpy(&my_vmin,data+offset,sizeof(int));
		//offset+=sizeof(int);	
		//memcpy(&my_vmax,data+offset,sizeof(int));
		//offset+=sizeof(int);
		//if(pointer_is_valid((uint64_t)my_rpm_regul))
              	//	 rc=rpm_regulator_set_voltage(my_rpm_regul,my_vmin,my_vmax);
		BUG();
               memcpy(&return_value,&rc,sizeof(int));
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

		PRINTKL(" PINCTRL_GET_RPC_CODE dev_id=%s,compatible_name=%s\n",dev_id,compatible_name);
		dev_name_transform(dev_id,bullhead_name_buf);

//javad
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
/*
		if( strcmp(bullhead_name_buf,"tomtom-slim-pgd")&&strcmp(bullhead_name_buf,"tomtom-slim-ifd") ){
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
		}else{	
			javad_dev = bus_find_device_by_name(&slimbus_type,NULL, bullhead_name_buf);
		}
*/
//javad end


		if(IS_ERR_OR_NULL(javad_dev))
			PRINTKL("javad_dev is error or null");
		PRINTKL("ofnodeptr=%lx",(unsigned long)(javad_dev->of_node));	
		PRINTKL("name %s, fullname %s, type %s",javad_dev->of_node->name,javad_dev->of_node->full_name,javad_dev->of_node->type);
		my_pinctrl = pinctrl_get(javad_dev);		
		PRINTKL("pinctrl ptr= %lx",(unsigned long)my_pinctrl);
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

               PRINTKL(" PM_REQUEST_IDLE_RPC_CODE dev_id=%s,compatible_name=%s\n",dev_id,compatible_name);
               dev_name_transform(dev_id,bullhead_name_buf);


//javad
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
/*
		if( strcmp(bullhead_name_buf,"tomtom-slim-pgd")&&strcmp(bullhead_name_buf,"tomtom-slim-ifd") ){
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
		}else{	
			javad_dev = bus_find_device_by_name(&slimbus_type,NULL, bullhead_name_buf);
		}
*/
//javad end

               if(IS_ERR_OR_NULL(javad_dev))
                       PRINTKL("javad_dev is error or null");
               PRINTKL("ofnodeptr=%lx",(unsigned long)(javad_dev->of_node));
               PRINTKL("name %s, fullname %s, type %s",javad_dev->of_node->name,javad_dev->of_node->full_name,javad_dev->of_node->type);
	       rc=pm_request_idle(javad_dev);
               PRINTKL("power_req_idle_rc= %d",rc);
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

               PRINTKL(" PM_RUNTIME_BARRIER_RPC_CODE dev_id=%s,compatible_name=%s\n",dev_id,compatible_name);
               dev_name_transform(dev_id,bullhead_name_buf);

//javad
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
/*
		if( strcmp(bullhead_name_buf,"tomtom-slim-pgd")&&strcmp(bullhead_name_buf,"tomtom-slim-ifd") ){
			javad_dev = bus_find_device_by_name(&platform_bus_type,NULL, bullhead_name_buf);
		}else{	
			javad_dev = bus_find_device_by_name(&slimbus_type,NULL, bullhead_name_buf);
		}
*/
//javad end

               if(IS_ERR_OR_NULL(javad_dev))
                       PRINTKL("javad_dev is error or null");
               PRINTKL("ofnodeptr=%lx",(unsigned long)(javad_dev->of_node));
               PRINTKL("name %s, fullname %s, type %s",javad_dev->of_node->name,javad_dev->of_node->full_name,javad_dev->of_node->type);
	       rc=pm_runtime_barrier(javad_dev);
               PRINTKL("power_run_barrier_rc= %d",rc);
               memcpy(&return_value,&rc,sizeof(int));	
	break;
		
	case OF_GET_GPIO_RPC_CODE:
		

               memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               offset+=sizeof(msg_len);

               memcpy(compatible_name ,data+offset, msg_len);
               offset+=msg_len;
		
	
               memcpy(&my_index ,data+offset, sizeof(int));
		offset+=sizeof(int);

               PRINTKL("OF_GPIO_GET:compatible_name=%s, my_index=%d\n",compatible_name,my_index);
		my_of_node =of_find_compatible_node(NULL,NULL,compatible_name);

               PRINTKL("ofnodeptr=%lx",(unsigned long)(my_of_node));
               PRINTKL("name %s, fullname %s, type %s",my_of_node->name,my_of_node->full_name,my_of_node->type);
	       rc=of_get_gpio(my_of_node,my_index);
               PRINTKL("power_run_barrier_rc= %d",rc);
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

               PRINTKL("OF_GPIO_NAMED_GET:compatible_name=%s,gpio_name=%s my_index=%d\n",compatible_name,gpio_name,my_index);
		my_of_node =of_find_compatible_node(NULL,NULL,compatible_name);

               PRINTKL("ofnodeptr=%lx",(unsigned long)(my_of_node));
               PRINTKL("name %s, fullname %s, type %s",my_of_node->name,my_of_node->full_name,my_of_node->type);
	       rc=of_get_named_gpio(my_of_node,gpio_name,my_index);
               PRINTKL("power_run_barrier_rc= %d",rc);
               memcpy(&return_value,&rc,sizeof(int));	
	break;
	case MSM_IOMMU_GET_CTX_RPC_CODE:
		
               memcpy(&msg_len ,data+offset,  sizeof(msg_len));
               offset+=sizeof(msg_len);
               memcpy(ctx_name ,data+offset, msg_len);
               offset+=msg_len;
		//my_device=msm_iommu_get_ctx(ctx_name);
		BUG();
		if(IS_ERR_OR_NULL(my_device)){
			PRINTKL("iommu device is null");
		}
		else{
			PRINTKL("iommu devname=%s",dev_name(my_device));
		}
		add_valid_pointer((uint64_t)my_device);
		memcpy(&return_value,&my_device,8);
		
		
	break;
	case MSM_GET_IOMMU_DOMAIN_RPC_CODE:
		
               memcpy(&my_domain_num ,data+offset,  sizeof(int));
               offset+=sizeof(int);
		//my_domain=msm_get_iommu_domain(my_domain_num);
		BUG();
		add_valid_pointer((uint64_t)my_domain);
		memcpy(&return_value,&my_domain,8);
		
		
	break;
	case MSM_REGISTER_DOMAIN_RPC:
		//rc=msm_register_domain(&vfe_layout);
		BUG();
		memcpy(&return_value,&rc,sizeof(int));	
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
                memcpy(&msg_len ,data+offset,  sizeof(msg_len));
                offset+=sizeof(msg_len);
                memcpy(ctx_name ,data+offset, msg_len);
                offset+=msg_len;
                //my_ionc=msm_ion_client_create(ctx_name);
		BUG();
		add_valid_pointer((uint64_t)my_ionc);
                memcpy(&return_value,&my_ionc,8);

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
	
		memcpy(&my_ionc ,data+offset, 8 );
		offset+=8;
		
		memcpy(&my_ionh ,data+offset, 8 );
		offset+=8;
	
		memcpy(&my_domain_num, data+offset, sizeof(int));
		offset+= sizeof(int);

		memcpy(&my_partition_num, data+offset, sizeof(int));
		offset+= sizeof(int);
		
		memcpy(&my_align, data+offset, sizeof(unsigned long));
		offset+= sizeof(unsigned long);
	
		memcpy(&my_iova_length, data+offset, sizeof(unsigned long));
		offset+= sizeof(unsigned long);
	
		memcpy(&my_iova, data+offset,8);
		offset+= 8;

		memcpy(&my_buffer_size, data+offset,8);
		offset+= 8;

		memcpy(&my_flags, data+offset, sizeof(unsigned long));
		offset+= sizeof(unsigned long);


		memcpy(&my_iommu_flags, data+offset, sizeof(unsigned long));
		offset+= sizeof(unsigned long);

		//if(pointer_is_valid((uint64_t)my_ionc ))
		//	rc=ion_map_iommu(my_ionc,my_ionh,my_domain_num,my_partition_num,my_align,my_iova_length,(ion_phys_addr_t *)my_iova,my_buffer_size,my_flags,my_iommu_flags);
		BUG();
		memcpy(&return_value,&rc,sizeof(int));
	break;
	case ION_UNMAP_IOMMU_RPC_CODE:

		memcpy(&my_ionc ,data+offset, 8 );
		offset+=8;
		
		memcpy(&my_ionh ,data+offset, 8 );
		offset+=8;
	
		memcpy(&my_domain_num, data+offset, sizeof(int));
		offset+= sizeof(int);

		memcpy(&my_partition_num, data+offset, sizeof(int));
		offset+= sizeof(int);
		

		//if(pointer_is_valid((uint64_t)my_ionc ))
		//	ion_unmap_iommu(my_ionc,my_ionh,my_domain_num,my_partition_num);
		BUG();
		rc=999;
		memcpy(&return_value,&rc,sizeof(int));

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
 

	//msg->len=sizeof(uint64_t);
	//memcpy(msg->data,&return_value,sizeof(uint64_t));	
	//res= cn_netlink_send(msg, 0, GFP_ATOMIC);
	//if (res < 0)
	//printk(KERN_INFO "Error while sending back to user\n");

	return return_value;	
}

///* Based on dfvn_listen_for_clients() in Rio project. */
//ksocket_t charm_listen_for_connection(ksocket_t listen_fd)
//{
//	ksocket_t comm_fd;
//	int addr_len;
//	struct sockaddr_in addr_cli;
//	//char *tmp;
//	PRINTK4("[1]");
//
//	memset(&addr_cli, 0, sizeof(addr_cli));
//
//	comm_fd = NULL;
//	if (klisten(listen_fd, 10) < 0) {
//		return NULL;
//	}
//	PRINTK4("[2]");
//
//	comm_fd = kaccept(listen_fd, (struct sockaddr *) &addr_cli, &addr_len);
//	if (comm_fd == NULL) 	{
//		return NULL;
//	}
//	PRINTK4("[3]");
//
//	//tmp = inet_ntoa(&addr_cli.sin_addr);
//	//*cli_id = (unsigned long) inet_addr(tmp);
//	//kfree(tmp);
//
//	return comm_fd;
//}
//
///* Based on dfvn_open_server_sock() and socket_handler() in Rio project. */
//ksocket_t charm_open_sock(int port)
//{
//	ksocket_t listen_fd, comm_fd;
//	struct sockaddr_in addr_srv;
//	int addr_len;
//	PRINTK4("[1]: port = %d", port);
//
//	memset(&addr_srv, 0, sizeof(addr_srv));
//	addr_srv.sin_family = AF_INET;
//	addr_srv.sin_port = htons(port);
//	addr_srv.sin_addr.s_addr = INADDR_ANY;
//	addr_len = sizeof(struct sockaddr_in);
//
//	listen_fd = ksocket(AF_INET, SOCK_STREAM, 0);
//	if (listen_fd == NULL) 	{
//		return NULL;
//	}
//	PRINTK4("[2]");
//
//	if (kbind(listen_fd, (struct sockaddr *) &addr_srv, addr_len) < 0) {
//		kclose(listen_fd);
//		return NULL;
//	}
//	PRINTK4("[3]");
//
//	comm_fd = charm_listen_for_connection(listen_fd);
//	PRINTK4("[4]");
//		
//	kclose(listen_fd);
//
//	return comm_fd;
//}
//
///* Based on dfvn_receive() in Rio project. */
//int charm_receive(ksocket_t sockfd, char *data, int size)
//{
//	char *p = data;
//	int n = 0;
//	int current_size = size;
//	int m, restart = 0;
//
//	while (n != size) {
//
//		m = krecvall(sockfd, p, current_size, MSG_WAITALL);
//		if (m == -ERESTARTSYS) {
//			restart++;
//		} else if (m < 0) {
//			PRINTK_ERR("Error: m = %d\n", m);
//			break;
//		} else {
//			n += m;
//			current_size -= m;
//			p += m;
//		}
//	}
//
//	if (n != size) {
//		PRINTK_ERR("Error: failed to receive frame completely. "
//			"received(n) = %d, size = %d\n", n, size);
//	}
//
//	return 0;
//}
//
///* Based on dfvn_send() in Rio project. */
//int charm_send(ksocket_t sockfd, char *data, int size)
//{
//	char *p = data;
//	int n = 0;
//	int current_size = size;
//	int m, restart = 0;
//
//	while (n != size) {
//
//		m = ksend(sockfd, p, current_size, 0);
//		if (m == -ERESTARTSYS) {
//			restart++;
//		} else if (m < 0) {
//			PRINTK_ERR("Error: m = %d\n", m);
//			break;
//		} else {
//			n += m;
//			current_size -= m;
//			p += m;
//		}
//	}
//
//	if (n != size) {
//		PRINTK_ERR("Error: failed to send frame completely. "
//			"sent(n) = %d, size = %d\n", n, size);
//	}
//
//	return 0;
//}

//ksocket_t comm_fd_irq = NULL;

//char send_buf[1024];
int send_irq_charm(uint64_t irq)
{
	int ret;
	PRINTK4("[1]");

	//if (comm_fd_irq == NULL) {
	//	PRINTK_ERR("irq socket not valid\n");
	//	return -EINVAL;
	//}
	//PRINTK4("[2]");

	//ret = charm_send(comm_fd_irq, (char *) &irq, 8);
	//if (ret) {
	//ret = acc_write_3(&acc_fp, (char __user *) &irq, 8, NULL);
	ret = acc_write_loop(acc_write_3, &acc_fp, (char __user *) &irq, 8, NULL);
	if (ret != 8) {
		PRINTK_ERR("Could not send the message, ret = %d\n", ret);
		//kclose(comm_fd_irq);
		//comm_fd_irq = NULL;
		acc_release(NULL, &acc_fp);
		return -EFAULT;
	}
	PRINTK4("[3]");

	return 0;
	//struct cn_msg *msg;
	//int res;
	//
	//msg=(struct cn_msg *)send_buf;
	//msg->id.idx=cn_irq_id.idx;
	//msg->id.val=cn_irq_id.val;	
	//msg->seq=0;
	//msg->ack=0;
	//msg->len=sizeof(uint64_t);
	//memcpy(msg->data,&irq,sizeof(uint64_t));	
	//res = cn_netlink_send(msg,0,GFP_ATOMIC);
	//if (res < 0)
	//	printk(KERN_INFO "Error while sending back to user\n");
	//return 0;	
}


//static int irq_thread_main(void *data)
//{
//	PRINTK4("[1]");
//	comm_fd_irq = charm_open_sock(IRQ_TCP_PORT);
//	if (comm_fd_irq == NULL)
//		return -EFAULT;
//	PRINTK4("[2]");
//
//	while(true)
//		mdelay(1000000);
//
//	return 0;
//}

//static int reg_thread_main(void *data)
//{
//	ksocket_t comm_fd_reg;
//	void *buffer;
//	uint64_t response_value;
//	int ret;
//	bool is_read;
//	PRINTK4("[1]");
//
//	buffer = kmalloc(PAYLOAD_SIZE_REG, GFP_KERNEL);
//	if (!buffer) {
//		PRINTK_ERR("Could not allocate memory for buffer\n");
//		return -ENOMEM;
//	}
//
//	comm_fd_reg = charm_open_sock(REG_TCP_PORT);
//	if (comm_fd_reg == NULL)
//		return -EFAULT;
//	PRINTK4("[2]");
//
//	while(true) {
//		ret = charm_receive(comm_fd_reg, (char *) buffer, PAYLOAD_SIZE_REG);
//		if (ret) {
//			PRINTK_ERR("Could not receive the message\n");
//			kclose(comm_fd_reg);
//			return -EFAULT;
//		}
//
//		is_read = reg_callback(buffer, &response_value);
//
//		if (is_read) {
//			ret = charm_send(comm_fd_reg, (char *) &response_value, 8);
//			if (ret) {
//				PRINTK_ERR("Could not send the message\n");
//				kclose(comm_fd_reg);
//				return -EFAULT;
//			}
//		}
//	}
//
//	return 0;
//}

static int rpc_thread_main(void *data)
{
	//ksocket_t comm_fd_rpc;
	void *buffer;
	uint64_t response_value;
	int ret;
	PRINTK5("[1]");

	buffer = kmalloc(PAYLOAD_SIZE_RPC, GFP_KERNEL);
	if (!buffer) {
		PRINTK_ERR("Could not allocate memory for buffer\n");
		return -ENOMEM;
	}

	//comm_fd_rpc = charm_open_sock(RPC_TCP_PORT);
	//if (comm_fd_rpc == NULL)
	//	return -EFAULT;
	PRINTK5("[2]");
	
	set_user_nice(current, -20);

	while(true) {
		//PRINTK5("[3]");
		//ret = charm_receive(comm_fd_rpc, (char *) buffer, PAYLOAD_SIZE_RPC);
		//if (ret) {
		//ret = acc_read_2(&acc_fp, (char __user *) buffer, PAYLOAD_SIZE_RPC, NULL);
		ret = acc_read_loop(acc_read_2, &acc_fp, (char __user *) buffer,
							PAYLOAD_SIZE_RPC, NULL);
		if (ret != PAYLOAD_SIZE_RPC) {
			PRINTK_ERR("Could not receive the message: ret = %d\n", ret);
			//kclose(comm_fd_rpc);
			acc_release(NULL, &acc_fp);
			return -EFAULT;
		}
		//PRINTK5("[4]");

		response_value = rpc_callback(buffer);
		//response_value = 0x0;
		//PRINTK5("[5]: response_value = %#lx", (unsigned long) response_value);

		//ret = charm_send(comm_fd_rpc, (char *) &response_value, 8);
		//if (ret) {
		//ret = acc_write_2(&acc_fp, (char __user *) &response_value, 8, NULL);
		ret = acc_write_loop(acc_write_2, &acc_fp, (char __user *) &response_value,
										8, NULL);
		if (ret != 8) {
			PRINTK_ERR("Could not send the message\n");
			//kclose(comm_fd_rpc);
			acc_release(NULL, &acc_fp);
			return -EFAULT;
		}
		//PRINTK5("[6]");
	}

	return 0;
}

static int reg_thread_main(void *data)
{
	void *buffer;
	uint64_t response_value;
	//uint64_t *msg;
	int ret;
	bool is_read;
	PRINTK5("[1]");
	//while (true)
	//	mdelay(1000);

	buffer = kmalloc(PAYLOAD_SIZE_REG, GFP_KERNEL);
	if (!buffer) {
		PRINTK_ERR("Could not allocate memory for buffer");
		return -ENOMEM;
	}
	PRINTK5("[2]");

	set_user_nice(current, -20);

	while(true) {
		//PRINTK5("[3]");
		//ret = acc_read(&acc_fp, (char __user *) buffer, PAYLOAD_SIZE_REG, NULL);
		ret = acc_read_loop(acc_read, &acc_fp, (char __user *) buffer,
							PAYLOAD_SIZE_REG, NULL);
		//PRINTK5("[4]: ret = %d", ret);
		if (ret != PAYLOAD_SIZE_REG) {
			PRINTK_ERR("Could not receive the message, ret = %d", ret);
			acc_release(NULL, &acc_fp);
			return -EFAULT;
		}

		//msg = (uint64_t *) buffer;
		//PRINTK5("[4]: *msg = %#lx", (unsigned long) *msg);

		//response_value = 0x14;
		//is_read = true;
		is_read = reg_callback(buffer, &response_value);
		//PRINTK5("[4]: is_read = %d", (int) is_read);
		//PRINTK5("[5]: response_value = %#lx", (unsigned long) response_value);

		if (is_read) {
			//ret = acc_write(&acc_fp, (const char __user *) &response_value, 8, NULL);
			ret = acc_write_loop(acc_write, &acc_fp,
					(const char __user *) &response_value, 8, NULL);
			//PRINTK5("[6]: ret = %d", ret);
			if (ret != 8) {
				PRINTK_ERR("Could not send the message\n");
				acc_release(NULL, &acc_fp);
				return -EFAULT;
			}
		}
	}
}

//0xfd8c0
//0xfda00
//0xfda08
//0xfda0a
//0xfda0b
//0xfda0c

struct task_struct *reg_thread, *rpc_thread;
//, *irq_thread;

int charm_agent_init2(void)
{

	PRINTK5("[1]");
	mdelay(1000);
	while (acc_open(NULL, &acc_fp)) {
		PRINTK_ERR("acc_open failed");
		mdelay(1000);
	}
	PRINTK5("[2]");

	//kernel_thread(usb_thread_main, NULL, 0);
	reg_thread = kthread_run(reg_thread_main, NULL, "charm_reg_thread");
	rpc_thread = kthread_run(rpc_thread_main, NULL, "charm_rpc_thread");
	//irq_thread = kthread_run(irq_thread_main, NULL, "charm_irq_thread");

	return 0;
}

int charm_agent_init(void)
{
	void *reg_ptr;
	int i=0;
	PRINTK5("[1]");

	reg_phys_base[0] = 0x14d30000;
	reg_phys_size[0] = PAGE_SIZE;

	/* GPU */	
	//reg_phys_base[0] = 0xfdb00000;
	//reg_phys_size[0] = 0x40 * PAGE_SIZE;

	//reg_phys_base[1] = 0xfc390000;
	//reg_phys_size[1] = PAGE_SIZE;

	//reg_phys_base[2] = 0xfc381000;
	//reg_phys_size[2] = PAGE_SIZE;

	//reg_phys_base[0] = 0xfe1040000;
	//reg_phys_size[0] = 0x20 * PAGE_SIZE;

	//reg_phys_base[1] = 0xfda00000;
	//reg_phys_size[1] = 0xd * PAGE_SIZE;

	//reg_phys_base[2] = 0xfd8c0000;
	//reg_phys_size[2] = PAGE_SIZE;

	//reg_phys_base[3] = 0xfda10000;
	//reg_phys_size[3] = 0xd* PAGE_SIZE;

	//reg_phys_base[4] = 0xfda40000;
	//reg_phys_size[4] = PAGE_SIZE;


	//reg_phys_base[5] = 0xfda80000;
	//reg_phys_size[5] = PAGE_SIZE;


	//reg_phys_base[6] = 0xfda60000;
	//reg_phys_size[6] = PAGE_SIZE;


	//reg_phys_base[7] = 0xfdaa0000;
	//reg_phys_size[7] = PAGE_SIZE;


	//reg_phys_base[8] = 0xfe12f000;
	//reg_phys_size[8] = 0x2c * PAGE_SIZE;


	for(i =0 ; i<num_reg_addrs ; i++){

		reg_ptr = ioremap(reg_phys_base[i], reg_phys_size[i]);
		if (!reg_ptr) {
			PRINTK_ERR("Could not map register page (%d)",i);
			return -EFAULT;
		}
		reg_ptrs[i] = (uint64_t) reg_ptr;
		PRINTK5("[2]: reg_ptrs[%d] = %#lx\n", i, (unsigned long) reg_ptrs[i]);
	}
//	reg_ptr = ioremap(reg_phys_base_1, reg_phys_size_1);
//	if (!reg_ptr) {
//		PRINTK_ERR("Could not map register page (1)");
//		return -EFAULT;
//	}
//	reg_ptr_1 = (uint64_t) reg_ptr;
//
//	reg_ptr = ioremap(reg_phys_base_2, reg_phys_size_2);
//	if (!reg_ptr) {
//		PRINTK_ERR("Could not map register page (2)");
//		return -EFAULT;
//	}
//	reg_ptr_2 = (uint64_t) reg_ptr;

	//kernel_thread(irq_thread_main, NULL, 0);
	//kernel_thread(reg_thread_main, NULL, 0);
	//kernel_thread(rpc_thread_main, NULL, 0);

	return 0;
	//----------
	//int err;
	//err = cn_add_callback(&cn_reg_id, cn_reg_name, cn_reg_callback);
	//if (err)
	//	goto err_out;


	//err = cn_add_callback(&cn_irq_id, cn_irq_name, cn_irq_callback);
	//if (err)
	//	goto err_out;


	//err = cn_add_callback(&cn_rpc_id, cn_rpc_name, cn_rpc_callback);
	//if (err)
	//	goto err_out;
	//pr_info("cn_reg initialized with id={%u.%u}\n",
	//	cn_reg_id.idx, cn_reg_id.val);

	//pr_info("cn_irq initialized with id={%u.%u}\n",
	//	cn_irq_id.idx, cn_irq_id.val);
	//return 0;

	//err_out:
	//return err;
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

	//ret = kthread_stop(irq_thread);
	//if (ret)
	//	PRINTK_ERR("Could not stop charm_irq_thread (err = %d)", ret);
}

module_init(charm_agent_init);
module_exit(charm_agent_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("mohammad");
MODULE_DESCRIPTION("javad:charm agent for android");
