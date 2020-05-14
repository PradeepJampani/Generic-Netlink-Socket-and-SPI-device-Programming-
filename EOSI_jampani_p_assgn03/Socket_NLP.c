#include "IOCTL.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/timekeeping.h>
#include <linux/math64.h>
#include <linux/kfifo.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>

#define MAJOR_NUMBER 153
#define MINOR_NUMBER 0
#define DEVICE_NAME "LedMatrix"
#define CLASS_NAME "MatrixDisplay"
#define max_measured_distance __UINT64_MAX__
#define DRIVER_NAME "HCSR_DRIVER"

#define spi_led_prd 10
#define spi_led_NS 5

#include <linux/netlink.h>
#ifndef __KERNEL__
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#else
#include <net/genetlink.h>
#endif


#define spi_socket_genl_FAMILY_NAME "spi_socket_genl"
#define spi_socket_genl_MSG_MAX 255
#define spi_socket_genl_MCGROUP0_NAME "MCAST_0"


struct pattern_to_send
{
    uint16_t patternToSend[10][8];

};

struct pin_mux_buf
{
    int trigger_pin;    
    int echo_pin;        
    int chip_select;    

};


struct configuration_struct
{
    int n_samples; 
    int period;    
};

struct calc_dists_buf
{
    int measured_distance;           

};

struct pattern
{
    uint8_t row[8];      

};


enum spi_socket_genl_attrs
{
    spi_socket_genl_ATTR_UNSPEC, 
    spi_socket_genl_ATTR_MSG,    
    spi_socket_genl_ATTR_MAX
};


enum spi_socket_genl_cmd
{
    spi_socket_genl_CMD_UNSPEC,  
    spi_socket_genl_CMD_PATTERN, 
    spi_socket_genl_CMD_CONFIG,  
    spi_socket_genl_CMD_DIST,    
};


enum spi_socket_genl_mcgroups
{
    spi_socket_genl_MCGROUP0,
};


static struct nla_policy spi_socket_genl_policy[spi_socket_genl_ATTR_MAX] = {
    [spi_socket_genl_ATTR_MSG] = {
        .type = NLA_UNSPEC,
    },
};

// Display buffer to hold patterns
uint16_t displayBuffer[10][8]= {{0x0144,0x0238,0x0310,0x0438,0x0538,0x0610,0x0700,0x0800},
                                 {0x0100,0x0228,0x0328,0x0410,0x0538,0x0638,0x0710,0x0800},
                                 {0x0140,0x02c8,0x03f0,0x0470,0x05f0,0x06c8,0x0740,0x0800},
                                 {0x0140,0x0262,0x033c,0x043c,0x053c,0x0662,0x0740,0x0800},
                                 {0x0180,0x0260,0x03f0,0x0470,0x05e0,0x0640,0x07e0,0x0860},
                                 {0x0160,0x02f0,0x03f0,0x0460,0x05c0,0x06e0,0x0760,0x0800},
                                 {0x011c,0x020a,0x030e,0x040c,0x0508,0x060c,0x0708,0x0800},
                                 {0x0108,0x021c,0x0308,0x0428,0x0538,0x0618,0x070c,0x0808},
                                 {0x01b0,0x0288,0x034a,0x043d,0x052a,0x06c8,0x0784,0x0807},
                                 {0x0107,0x0208,0x03ca,0x043d,0x052a,0x06c8,0x0798,0x0820}};



struct spi_led04_dev
{
							
					
	struct miscdevice *miscdev;						
	struct pin_mux_buf cfg;				
	struct configuration_struct config;			
	struct pin_mux_buf l_cfg;			 
	struct task_struct *measured_distance_thread;	 
	struct task_struct *trigger_thread;	 
	struct task_struct *config_thread;	
	struct task_struct *nlp_msg_thread;
	struct calc_dists_buf calc_dist;
	struct semaphore nlp_sem;			
	struct semaphore distance_sem;			
	struct semaphore trigger_sem;			
	struct spinlock spi_led_spinlock;	
	char device_name_[9];					
	int state;
	int gpio_trigg[4];			
	int gpio_echo[4];			 
	unsigned int irq;					 
	uint64_t time1;
	uint64_t time2;
		 
};



int l_gpio[4],v_val[4];

void pin_setup(int pin){
int i;

    if(pin==0)
    {
        int pins_gpio[4] = {11, 32, -1, -1};
        int vals[4] = {0, 0, -1, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==1)
       {
        int pins_gpio[4] = {12, 28, 45, -1};
        int vals[4] = {0, 0, 0, -1};
       for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==2)
    {
        int pins_gpio[4] = {13, 34, 77, -1};
        int vals[4] = {0, 0, 0, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==3)
    {
        int pins_gpio[4] = {14, 16, 76, 64};
        int vals[4] = {0, 0, 0, 0};
       for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }

    }
   if(pin==4)
    {
        int pins_gpio[4] = {6, 36, -1, -1};
        int vals[4] = {0, 0, -1, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==5)
    {
        int pins_gpio[4] = {0, 18, 66, -1};
        int vals[4] = {0, 0, 0, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==6)
    {
        int pins_gpio[4] = {1, 20, 68, -1};
        int vals[4] = {0, 0, 0, -1};
       for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==7)
    {
        int pins_gpio[4] = {38, -1, -1, -1};
        int vals[4] = {0, -1, -1, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==8)
    {
        int pins_gpio[4] = {40, -1, -1, -1};
        int vals[4] = {0, -1, -1, -1};
       for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==9)
    {
        int pins_gpio[4] = {4, 22, 70, -1};
        int vals[4] = {0, 0, 0, -1};
    for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==10)
    {
        int pins_gpio[4] = {10, 26, 74, -1};
        int vals[4] = {0, 0, 0, -1};
       for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
   if(pin==11)
    {
        int pins_gpio[4] = {5, 24, 44, 72};
        int vals[4] = {0, 0, 0, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==12)
    {
        int pins_gpio[4] = {15, 42, -1, -1};
        int vals[4] = {0, 0, -1, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==13)
    {
        int pins_gpio[4] = {7, 30, 46, -1};
        int vals[4] = {0, 0, 0, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    }



int number_of_sensors = 1;
module_param(number_of_sensors, int, 0000);

struct spi_led04_dev **spi_led_devp;
struct miscdevice miscdev[2];

static struct genl_family spi_socket_genl_family;

void one_shot(struct spi_led04_dev *spi_led ){

gpio_set_value_cansleep(spi_led->gpio_trigg[0], 1);

        udelay(11);
        gpio_set_value_cansleep(spi_led->gpio_trigg[0], 0);


}
/* Thread to store the time stamps and calculate the distance followed by storing the distance*/
int measured_distance_thread_fn(void *data)
{
    int check = 0, i = 0;
    int check_Min_dist=0,check_Max_dist=0;
    struct spi_led04_dev *spi_led = (struct spi_led04_dev *)data;
    int n = spi_led->config.n_samples;
    int sleep_for = spi_led->config.period;
    uint64_t measured_distance = 0,  total_time[n + 2];
    
    for(check=0;check-2 < n;)
    {
        down_interruptible(&(spi_led->distance_sem));

        total_time[check] = spi_led->time2 - spi_led->time1;

        if (total_time[check] < max_measured_distance){
	
            check_Min_dist = total_time[check];
		}	
        if (total_time[check] > 0){
	
            check_Max_dist = total_time[check];
	}
           msleep(sleep_for);
            up(&(spi_led->trigger_sem));
	  check++;
        
    }

    for (i = 0; i < (n + 2); i++)
    {
        if (total_time[i] == check_Min_dist || total_time[i] == check_Max_dist)
            continue;
        measured_distance = measured_distance + total_time[i];
    }
    measured_distance = div_u64(measured_distance * 340, (uint64_t)(10000000 * n * 2));
    spi_led->calc_dist.measured_distance = measured_distance;
    up(&(spi_led->nlp_sem));
    spin_lock(&(spi_led->spi_led_spinlock));
    spi_led->state = 0;
    spin_unlock(&(spi_led->spi_led_spinlock));

    return 0;
}


int trigger_thread_fn(void *data)
{
    int i = 0;
    struct spi_led04_dev *spi_led = (struct spi_led04_dev *)data;
    int n = spi_led->config.n_samples;
	    /*for collecting m+2 samples*/
    for(i=0;i-2<n;i++)
    {
        down_interruptible(&(spi_led->trigger_sem));
	one_shot(spi_led);
 
    }

  return 0;
}

/* Thread to store the distance and send distance to user*/
int nlp_msg_thread_fn(void *data)
{
    void *header;
    int rc, flags = GFP_ATOMIC;
    struct spi_led04_dev *spi_led = (struct spi_led04_dev *)data;
    struct sk_buff *msg_to_send = genlmsg_new(NLMSG_DEFAULT_SIZE, flags);

    header = genlmsg_put(msg_to_send, 0, 0, &spi_socket_genl_family, flags, spi_socket_genl_CMD_DIST);
    down_interruptible(&(spi_led->nlp_sem));
    rc = nla_put(msg_to_send, spi_socket_genl_ATTR_MSG, sizeof(struct calc_dists_buf), &(spi_led->calc_dist));
    genlmsg_end(msg_to_send, header);
    rc = genlmsg_multicast(&spi_socket_genl_family, msg_to_send, 0, spi_socket_genl_MCGROUP0, flags);
   

return 0;
    
}


/* Irq handler for the echo pin*/
irq_handler_t gpio_irq_handler(unsigned int irq, void *data)
{

    struct spi_led04_dev *spi_led = (struct spi_led04_dev *)data;
	   /* checking if the interrupt is an rising edge*/
    if (irq_get_trigger_type(irq) == IRQF_TRIGGER_RISING)
    {
        spi_led->time1 = ktime_get_ns();
        irq_set_irq_type(irq, IRQF_TRIGGER_FALLING);
    }
	        /* Detecting the falling edge*/
    else if (irq_get_trigger_type(irq) == IRQF_TRIGGER_FALLING)
    {
        spi_led->time2 = ktime_get_ns();
        up(&(spi_led->distance_sem));//updating the distance_thread
        irq_set_irq_type(irq, IRQF_TRIGGER_RISING);// resetting to rising edge
    }
    return (irq_handler_t)IRQ_HANDLED;
}

int pin_init(int pin, struct spi_led04_dev **spi_led, int isEchopin)
{
    int rc = 0, i = 0, irq = -1;
    pin_setup(pin);
    for (i = 0; i < 4; i++)
    {
        if (isEchopin)
            (*spi_led)->gpio_echo[i] = l_gpio[i];
        else
            (*spi_led)->gpio_trigg[i] = l_gpio[i];

        if (l_gpio[i] < 0)
            continue;

        gpio_request(l_gpio[i], NULL);

        if (i >=2)
            gpio_set_value_cansleep(l_gpio[i], v_val[i]);

        else
        {
                 if (isEchopin)
                {
                rc = gpio_direction_input(l_gpio[i]);
                if (i == 0)
                {
                    irq = gpio_to_irq(l_gpio[i]);
                    rc = request_irq(irq, (irq_handler_t)gpio_irq_handler, IRQF_TRIGGER_RISING, NULL, (void *)(*spi_led));
                    (*spi_led)->irq = irq;
                }
                    }
                     else  gpio_direction_output(l_gpio[i], v_val[i]);
               }
               }

    if (isEchopin)
        (*spi_led)->l_cfg.echo_pin = l_gpio[0];
    else
        (*spi_led)->l_cfg.trigger_pin = l_gpio[0];

    return 0;

}

// User defined structure to store the device data
struct spidev_data {
	dev_t                   devt;
	struct spi_device       *spi;
}*spi_data;

static struct class *spi_class;
// SPI message structure to carry forward the message
static struct spi_message mess;
// Delay flag for guiding the SPI operation
int delayFlag = 0;

// Carrier to send the pattern data in a single SPI transfer
uint16_t transBuf[1];
// Configuration data for setting the 8x8 LED Driver for operation
uint16_t configData[5]={0x0C01,0x0900,0x0A0F,0x0B07,0x0F00};
// Sequence Buffer to hold the current data
int sequence[10];

// No Display data to turn off the LED Display
uint16_t noDisplay[8] = {0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700,0x0800};

// Structure to hold the necessary spi transfer parameters
struct spi_transfer tx = {

	.tx_buf = transBuf,         // Transmission Buffer
	.rx_buf = 0,                // setting receiving buffer as zero to have Half-Duplex Operation
	.len = 2,                   // No of Bytes
	.bits_per_word = 16,        // Bits per Word
	.speed_hz = 10000000,       // Speed of Transfer
	.delay_usecs = 1,           // Delay between Transfers
};


/*************************************************************************************************************
*
*
*   Funtion to send the SPI pattern to the LED matrix Driver
*
*
*
***************************************************************************************************************/

int send_spi_pattern(uint16_t passPattern[], int length){

    int i;
    for(i = 0;i < length; i++){
        transBuf[0]=passPattern[i];
        // Initiating the message variable
        spi_message_init(&mess);
        // Adding the message to the Tail
        spi_message_add_tail((void *)&tx, &mess);
        // Setting the Slave select pin as 0 to enable spi data transmission
        gpio_set_value(15,0);
        // Transferring the data through SPI synchronous operation
        if(spi_sync(spi_data->spi, &mess) < 0){
            printk("\nError in sending SPI message.\n");
            return -1;
        }

        // Setting the slave select pin as 1 to enable Display of transmitted data
        gpio_set_value(15,1);
	}
	return 0;
}






/*************************************************************************************************************
*
*
*   Closing function to release the GPIO pins taken during the operation
*
*
*
***************************************************************************************************************/

static int ledMatrixDriverClose(struct inode *ino, struct file *fil){
    gpio_unexport(30);
	gpio_free(30);
	gpio_unexport(46);
	gpio_free(46);
	gpio_unexport(24);
	gpio_free(24);
	gpio_unexport(44);
	gpio_free(44);
	gpio_unexport(72);
	gpio_free(72);
    gpio_unexport(15);
	gpio_free(15);
    gpio_unexport(42);
	gpio_free(42);

	printk(KERN_INFO "\nReleased all the necessary GPIO Pins for SPI Operation\n");
	return 0;
}


/*************************************************************************************************************
*
*
*   SPI probe function to create the defined SPI device under path /dev/
*
*
*
***************************************************************************************************************/

static int spi_probe_func(struct spi_device *spi)
{
	//struct spidev_data *spidev;
	int status = 0;
	struct device *dev;

	/* Allocate driver data */

	spi_data = kzalloc(sizeof(*spi_data), GFP_KERNEL);
	if(!spi_data)
	{
		return -ENOMEM;
	}

	/* Initialize the driver data */

	spi_data->spi = spi;
    // Assigning the Major and Minor Number
	spi_data->devt = MKDEV(MAJOR_NUMBER, MINOR_NUMBER);
    // Creating the character device
    dev = device_create(spi_class, &spi->dev, spi_data->devt, spi_data, DEVICE_NAME);

    if(dev == NULL)
    {
		printk("Device Creation Failed\n");
		kfree(spi_data);
		return -1;
	}
	printk("SPI LED Driver Probed.\n");
	return status;
}

/*************************************************************************************************************
*
*   Thread body for printing the patterns sent from the User Space
*
*
***************************************************************************************************************/

int spi_printing_thread(void *data){

    int i;
    // Setting the delay flag to avoid repeated calling of kernel threads before the termination of previous operations
    delayFlag = 1;
    for(i = 0; i < ARRAY_SIZE(sequence); i+=2){
        // Sending NO DISPLAY data at the end of the sequence
        if(sequence[i] == 0 && sequence[i+1] == 0){
            send_spi_pattern(noDisplay,ARRAY_SIZE(noDisplay));
            break;
        }
        // Sending the required SPI pattern
        send_spi_pattern(displayBuffer[sequence[i]],ARRAY_SIZE(displayBuffer[sequence[i]]));
        // Sleeping as per the delay given in the sequence
        msleep(sequence[i+1]);
    }
    delayFlag = 0;
    return 0;

}

/*************************************************************************************************************
*
*   Function to remove the spi device created
*
*
***************************************************************************************************************/

static int spi_remove(struct spi_device *spi){
	device_destroy(spi_class, spi_data->devt);
	kfree(spi_data);
	printk(" Driver Removed.\n");
	return 0;
}

/*************************************************************************************************************
*
*   IOCTL function to copy the pattern buffer from the User space to the pattern buffer in the kernel space
*
***************************************************************************************************************/

static long ledMatrixDriverIoctl(struct file *file, unsigned int cmd, unsigned long arg){
    copy_from_user(displayBuffer,(uint16_t **)arg, sizeof(displayBuffer));
   return 0;
}



static ssize_t ledMatrixDriverWrite(struct file *file, const char *buf,
           size_t count, loff_t *ppos){
    struct task_struct *task;
    // If there exist already SPI thread operation, return busy
    if (delayFlag == 1){
        return EBUSY;
    }
    copy_from_user(sequence,(int*)buf,sizeof(sequence));
    // Running Kthread for SPI transfer
    task = kthread_run(&spi_printing_thread,(void *)sequence,"SPI Printing thread");
    msleep(20); // Sleeping for sometime to avoid kernel panic errors
    return 0;
}

/*************************************************************************************************************
*
*   SPI driver structure for SPI device
*
***************************************************************************************************************/

static struct spi_driver spi_ledMatrix_driver = {
         .driver = {
                 .name =         "spidev",
                 .owner =        THIS_MODULE,
         },
         .probe =        spi_probe_func,
         .remove =       spi_remove,

};


void init_sem(void *spi_led){


}

static int spi_socket_genl_rx_dist(struct sk_buff *skb, struct genl_info *info)
{
    struct calc_dists_buf request;
    struct spi_led04_dev *spi_led;

    if (!info->attrs[spi_socket_genl_ATTR_MSG])
    {
        printk("Empty message from user\n");
        return -EINVAL;
    }

    request = *(struct calc_dists_buf *)nla_data(info->attrs[spi_socket_genl_ATTR_MSG]);
    spi_led = spi_led_devp[0];


    sema_init(&(spi_led->distance_sem), 0);
    sema_init(&(spi_led->trigger_sem), 1);
    sema_init(&(spi_led->nlp_sem), 0);


    spi_led->measured_distance_thread = kthread_run(measured_distance_thread_fn, (void *)spi_led, "%s_measured_distance_thread", spi_led->device_name_);
    spi_led->trigger_thread = kthread_run(trigger_thread_fn, (void *)spi_led, "%s_trigger_thread", spi_led->device_name_);
    spi_led->nlp_msg_thread =kthread_run(nlp_msg_thread_fn, (void *)spi_led, "%s_nlp_msg_thread", spi_led->device_name_);

    return 0;
}
/*************************************************************************************************************
*
function to Configue the CS pin, trigger pin , Echo pin 
*
***************************************************************************************************************/

static int spi_socket_genl_rx_config(struct sk_buff *skb, struct genl_info *info)
{
     int rc = 0,j;
    struct pin_mux_buf cfg;
    struct spi_led04_dev *spi_led;

    if (!info->attrs[spi_socket_genl_ATTR_MSG])
    {
        printk("Empty message from user\n");
        return -EINVAL;
    }

    cfg = *(struct pin_mux_buf *)nla_data(info->attrs[spi_socket_genl_ATTR_MSG]);

    spi_led = spi_led_devp[0];

    spin_lock(&(spi_led->spi_led_spinlock));
    spi_led->cfg = cfg;
    spin_unlock(&(spi_led->spi_led_spinlock));

    spin_lock(&(spi_led->spi_led_spinlock));
    spi_led->state = 1;
    spin_unlock(&(spi_led->spi_led_spinlock));

    // Free Already set gpios and isrs in case this isn't a first config
    if (spi_led->irq >= 0)
    {
        free_irq(spi_led->irq, (void *)(spi_led));
        spi_led->irq = -1;
    }
    
	/*freeing previously set pins*/
   for (j = 0; j < 4; j++)
    {
        if (spi_led->gpio_trigg[j] >= 0)
        {
            gpio_free(spi_led->gpio_trigg[j]);
        }

        if (spi_led->gpio_echo[j] >= 0)
        {
            gpio_free(spi_led->gpio_echo[j]);
        }
    }
	/*Intiatlizing the pins*/
    rc = pin_init(spi_led->cfg.trigger_pin, &spi_led, 0);
   
    rc = pin_init(spi_led->cfg.echo_pin, &spi_led, 1);
   
    spi_led->config.n_samples = spi_led_NS;
    spi_led->config.period = spi_led_prd;

    spin_lock(&(spi_led->spi_led_spinlock));
    spi_led->state = 0;
    spin_unlock(&(spi_led->spi_led_spinlock));
     // Initiating the GPIO pins for SPI Operation

    // Configuring IO 13 as SPI_CLK
    gpio_export(30,true);
    gpio_export(46,true);
    gpio_direction_output(30,1);
    gpio_set_value(30,0);
    gpio_direction_output(46,1);
    gpio_set_value(46,1);

    // Configuring IO 11 as SPI_MOSI
    gpio_export(24,true);
    gpio_direction_output(24,1);
    gpio_set_value(24,0);

    gpio_export(44,true);
    gpio_direction_output(44,1);
    gpio_set_value(44,1);

    gpio_export(72,true);
    gpio_set_value(72,0);


    // Configuring IO 12 as Slave_Select
    gpio_export(15,true);
    gpio_direction_output(15,1);
    gpio_set_value(15,0);

    gpio_export(42,true);
    gpio_direction_output(42,1);
    gpio_set_value(42,0);

    // Sending the configuration data for Normal operation of the LED Display
    send_spi_pattern(configData, ARRAY_SIZE(configData));

    printk(KERN_INFO "\nInitiated all the necessary GPIO Pins for SPI Operation\n");

    return 0;
}

/*************************************************************************************************************
*
*    function to write the sequence obtained from the user space
*
***************************************************************************************************************/

static int spi_socket_genl_rx_pattern(struct sk_buff *skb, struct genl_info *info)
{

    struct pattern pattern;
	

    int i = 0;
        struct task_struct *task;


        if (delayFlag == 1){
        return EBUSY;
    }
    if (!info->attrs[spi_socket_genl_ATTR_MSG])
    {
        printk("Empty message from user\n");
        return -EINVAL;
    }

        pattern = *(struct pattern *)nla_data(info->attrs[spi_socket_genl_ATTR_MSG]);

	i=0;

    for(i=0;i<8;i++){
          sequence[i]=pattern.row[i];

        }


    task = kthread_run(&spi_printing_thread,(void *)sequence,"SPI Printing thread");
    msleep(20);
    return 0;
}


static struct file_operations spi_led04_fops = {
    .owner = THIS_MODULE,
};


static const struct genl_ops spi_socket_genl_ops[] = {
    {
        .cmd = spi_socket_genl_CMD_DIST,
        .policy = spi_socket_genl_policy,
        .doit = spi_socket_genl_rx_dist,
        .dumpit = NULL,
    },
    {
        .cmd = spi_socket_genl_CMD_CONFIG,
        .policy = spi_socket_genl_policy,
        .doit = spi_socket_genl_rx_config,
        .dumpit = NULL,
    },
    {
        .cmd = spi_socket_genl_CMD_PATTERN,
        .policy = spi_socket_genl_policy,
        .doit = spi_socket_genl_rx_pattern,
        .dumpit = NULL,
    },
};

static const struct genl_multicast_group spi_socket_genl_mcgroups[] = {
    [spi_socket_genl_MCGROUP0] = {
        .name = spi_socket_genl_MCGROUP0_NAME,
    },
};

static struct genl_family spi_socket_genl_family = {
    .name = spi_socket_genl_FAMILY_NAME,
    .n_ops = ARRAY_SIZE(spi_socket_genl_ops),
    .mcgrps = spi_socket_genl_mcgroups,
    .n_mcgrps = ARRAY_SIZE(spi_socket_genl_mcgroups),
    .maxattr = spi_socket_genl_ATTR_MAX - 1,
    .netnsok = false,
    .module = THIS_MODULE,
    .ops = spi_socket_genl_ops,
   
};


static const struct file_operations spi_file_ops = {

	.owner = THIS_MODULE,
	.release = ledMatrixDriverClose,
	.write	= ledMatrixDriverWrite,
	.unlocked_ioctl = ledMatrixDriverIoctl,

};

int __init SPI_SOCKET_INIT(void)
{
    int rc = 0;
    int i = 0, j = 0;

    // Registering the SPI Character device with the already defined major number
    if(register_chrdev(MAJOR_NUMBER,spi_ledMatrix_driver.driver.name, &spi_file_ops) > 0){
        printk("Failed to register the SPI device\n");
        return -1;
    }

    // Creating a Device Driver Class
    spi_class = class_create(THIS_MODULE, CLASS_NAME);

    if (spi_class == NULL){
        printk("Class Creation failed.");
        unregister_chrdev(MAJOR_NUMBER, spi_ledMatrix_driver.driver.name);
        return -1;
    }

    // Registering the  SPI driver
    if(spi_register_driver(&spi_ledMatrix_driver) < 0){
        printk("SPI Driver Registration failed.");
        class_destroy(spi_class);
        unregister_chrdev(MAJOR_NUMBER, spi_ledMatrix_driver.driver.name);
    }
    printk("SPI LED Driver got initialised.\n");

    spi_led_devp = kmalloc(sizeof(struct spi_led04_dev *) * number_of_sensors, GFP_KERNEL);
    if (!spi_led_devp)
    {
        printk("Error allocating memory for pointer to spi_led devp object\n");
        return -ENOMEM;
    }

    // Register the genl family
    rc = genl_register_family(&spi_socket_genl_family);
    if (rc)
    {
        printk("Error registering the genl family\n");
        return rc;
    }

    // Initialise each of the misc devices
    for (i = 0; i < number_of_sensors; i++)
    {
        // Allocate memory for each of the spi_led super object
        spi_led_devp[i] = kmalloc(sizeof(struct spi_led04_dev), GFP_KERNEL);
        if (!spi_led_devp[i])
        {
            printk("Error allocating memory for spi_led devp object\n");
            return -ENOMEM;
        }

        sprintf(spi_led_devp[i]->device_name_, "%s%d", "spi_led_", i);

        miscdev[i].minor = MISC_DYNAMIC_MINOR;
        miscdev[i].fops = &spi_led04_fops;
        miscdev[i].name = spi_led_devp[i]->device_name_;

        // Register the device
        rc = misc_register(&miscdev[i]);
        if (rc)
        {
            printk("Error registering device at index %d\n", i);
            return rc;
        }

        spi_led_devp[i]->miscdev = &miscdev[i];
        spi_led_devp[i]->state = 0;
        for (j = 0; j < 4; j++)
        {
            spi_led_devp[i]->gpio_echo[j] = -1;
            spi_led_devp[i]->gpio_trigg[j] = -1;
            
        }
        spi_led_devp[i]->irq = -1;
        sema_init(&(spi_led_devp[i]->distance_sem), 0);
        sema_init(&(spi_led_devp[i]->trigger_sem), 1);
        sema_init(&(spi_led_devp[i]->nlp_sem), 0);

        spin_lock_init(&(spi_led_devp[i]->spi_led_spinlock));
        }

    return rc;
}



void __exit SPI_SOCKET_EXIT(void)
{
    int rc = 0;
    int i = 0,j=0;

    genl_unregister_family(&spi_socket_genl_family);
    spi_unregister_driver(&spi_ledMatrix_driver);
    class_destroy(spi_class);
    unregister_chrdev(MAJOR_NUMBER, spi_ledMatrix_driver.driver.name);

    for (i = 0; i < number_of_sensors; i++)
    {
        printk("Freeing irq number %d\n", spi_led_devp[i]->irq);
        free_irq(spi_led_devp[i]->irq, (void *)(spi_led_devp[i]));
    }

    for (i = 0; i < number_of_sensors; i++)
    {
        for (j = 0; j < 4; j++)
        {
        if (spi_led_devp[i]->gpio_trigg[j] >= 0)
        {
            gpio_free(spi_led_devp[i]->gpio_trigg[j]);
        }

        if (spi_led_devp[i]->gpio_echo[j] >= 0)
        {
            gpio_free(spi_led_devp[i]->gpio_echo[j]);
        }
    }
        rc = misc_deregister(&miscdev[i]);
	kfree(spi_led_devp[i]);
    }
    if (spi_led_devp)
        kfree(spi_led_devp);

    return;
}



module_init(SPI_SOCKET_INIT);
module_exit(SPI_SOCKET_EXIT);

MODULE_LICENSE("GPL v2");
