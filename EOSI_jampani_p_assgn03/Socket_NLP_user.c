#define FILE_TO_OPEN "/dev/LedMatrix"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netlink/msg.h>
#include <netlink/attr.h>
#include "IOCTL.h"
#include <linux/netlink.h>

#ifndef __KERNEL__
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#else
#include <net/genetlink.h>
#endif

#ifndef GRADING
#define MAX7219_CS_PIN 12
#define HCSR04_TRIGGER_PIN 9
#define HCSR04_ECHO_PIN 10
#endif

#define DRIVER_NAME "HCSR_DRIVER"

#define spi_led_prd 10
#define spi_led_NS 5
// GENL Defines
#define spi_socket_genl_FAMILY_NAME "spi_socket_genl"
#define spi_socket_genl_MSG_MAX 256
#define spi_socket_genl_MCGROUP0_NAME "MCAST_0"
#define NUMBER_OF_TIMES_TO_MEASURE  15
#define PERIOD_TO_WAIT 3
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

// CMD enums
enum spi_socket_genl_cmd
{
    spi_socket_genl_CMD_UNSPEC,  // Unspecified CMD
    spi_socket_genl_CMD_PATTERN, // Pattern CMD
    spi_socket_genl_CMD_CONFIG,  // Config CMD
    spi_socket_genl_CMD_DIST,    // measured_distance Measurement Request CMD
};

// Multicast Group Enums
enum spi_socket_genl_mcgroups
{
    spi_socket_genl_MCGROUP0,
};

// GENL Policy structure
static struct nla_policy spi_socket_genl_policy[spi_socket_genl_ATTR_MAX] = {
    [spi_socket_genl_ATTR_MSG] = {
        .type = NLA_UNSPEC,
    },
};
// sequence buffer to carry the animation sequence for each pattern
int sequence[10];
// Animation sequence for Fish
int sequence1[10] = {0,200,1,200,0,200,1,200,0,0};
// Animation sequence for Frog
int sequence2[10] = {2,200,3,200,2,200,3,200,0,0};
// Animation sequence for Tortoise
int sequence3[10] = {4,200,5,200,4,200,5,200,0,0};
// Animation sequence for Bird
int sequence4[10] = {6,200,7,200,6,200,7,200,0,0};
// Animation sequence for Man Dancing
int sequence5[10] = {8,200,9,200,8,200,9,200,0,0};

struct cb_thread_args
{
    struct nl_sock *nlsock;
    struct nl_cb *cb;
};

int family_id = 0;
int g_measured_distance = -1; // measured_distance from kernel, global variable
int measured_distance_lock=0;

static int get_measured_distance_from_kernel(struct nl_msg *msg, void *arg)
{
    struct nlattr *attr[spi_socket_genl_ATTR_MAX];
    struct calc_dists_buf calc_dist;

    genlmsg_parse(nlmsg_hdr(msg), 0, attr, spi_socket_genl_ATTR_MSG, spi_socket_genl_policy);

    if (!attr[spi_socket_genl_ATTR_MSG])
    {
        printf("Empty message\n");
        return NL_OK;
    }

    calc_dist = *(struct calc_dists_buf *)nla_data(attr[spi_socket_genl_ATTR_MSG]);

    // Atomically udpate the global measured_distance variable
    if(measured_distance_lock!=1){
    measured_distance_lock=1;
    g_measured_distance = calc_dist.measured_distance;

    measured_distance_lock=0;
    }
    printf(" measured_distance = %d\n", calc_dist.measured_distance);

    return NL_OK;
}

int send_msg_to_kernel(struct nl_sock *nlsock, int cmd, int sequence[10])
{
    struct nl_msg *msg;
    struct nl_data *payload;
    struct pin_mux_buf cfg;
    struct calc_dists_buf request;
    struct pattern pattern;
    int j=0;
    int rc = 0;

    if (cmd == spi_socket_genl_CMD_CONFIG)
    {
        // Fill config buffer to send to kernel
        cfg.echo_pin = HCSR04_ECHO_PIN;
        cfg.trigger_pin = HCSR04_TRIGGER_PIN;
        cfg.chip_select = MAX7219_CS_PIN;
	if(cfg.chip_select!=12){
		printf("ERROR: USE Chip select as 12\n");
		return 1;
		}

        payload = nl_data_alloc(&cfg, sizeof(struct pin_mux_buf));
    }
    else if (cmd == spi_socket_genl_CMD_DIST)
    {
        //Send request to start measurement


        payload = nl_data_alloc(&request, sizeof(struct calc_dists_buf));
    }
    else if (cmd == spi_socket_genl_CMD_PATTERN)
    {
        for(j=0;j<10;j++){
          pattern.row[j]=sequence[j];
        }
        payload = nl_data_alloc(&pattern, sizeof(struct pattern));
    }

    msg = nlmsg_alloc();
    if (!msg)
    {
        printf("Error allocating nl message\n");
        return -1;
    }

    // Add the generic nl headers to the message
    if (!genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, family_id, 0, NLM_F_REQUEST, cmd, 0))
    {
        printf("Error adding headers to nl message\n");
        return -1;
    }

    // Add the message
    rc = nla_put_data(msg, spi_socket_genl_ATTR_MSG, payload);

    if (rc)
    {
        printf("Error adding cfg structure to message\n");
        return -1;
    }

    // Send the message
    rc = nl_send_auto(nlsock, msg);
    if (rc < 0)
    {
        printf("Error sending the message\n");
        return -1;
    }

    return 0;
}

void init_nlsock(struct nl_sock **nlsock)
{
    int grp_id;

    *nlsock = nl_socket_alloc();
    if (!*nlsock)
    {
        printf("Error allocating nl socket\n");
        exit(EXIT_FAILURE);
    }

    nl_socket_disable_seq_check(*nlsock);

    nl_socket_disable_auto_ack(*nlsock);


    if (genl_connect(*nlsock))
    {
        printf("Error connecting to generic netlink\n");
        exit(EXIT_FAILURE);
    }


    family_id = genl_ctrl_resolve(*nlsock, spi_socket_genl_FAMILY_NAME);
    if (family_id < 0)
    {
        printf("Error resolving family id\n");
        exit(EXIT_FAILURE);
    }


    grp_id = genl_ctrl_resolve_grp(*nlsock, spi_socket_genl_FAMILY_NAME, spi_socket_genl_MCGROUP0_NAME);
    if (grp_id < 0)
    {
        printf("Error resolving group id\n");
        exit(EXIT_FAILURE);
    }

    if (nl_socket_add_membership(*nlsock, grp_id))
    {
        printf("Error joining group\n");
        exit(EXIT_FAILURE);
    }

    printf("Succesfully initialised netlink socket\n");
}

static int skip_seq_check(struct nl_msg *msg, void *arg)
{
    return NL_OK;
}

void *doSPI(void *arg)
{
    int rc = 0, i = 0, measured_distance = 0;
    struct cb_thread_args cb_args;



    cb_args = *(struct cb_thread_args *)arg;
       for(i=0;i<NUMBER_OF_TIMES_TO_MEASURE;i++){
         rc = send_msg_to_kernel(cb_args.nlsock, spi_socket_genl_CMD_DIST, sequence1);
    	 rc = nl_recvmsgs(cb_args.nlsock, cb_args.cb);

   	/*Atomically getting distance*/
    	if(measured_distance_lock!=1){
    		measured_distance_lock=1;
    		measured_distance = g_measured_distance;
    		measured_distance_lock=0;
    		}

	 // For measured_distances above 100cm swimming fish is displayed
    	if(measured_distance>=100){
        	   rc = send_msg_to_kernel(cb_args.nlsock, spi_socket_genl_CMD_PATTERN,sequence1);
        }

        // For measured_distances between 70cm and 100cm, Jumping Frog is displayed
        else if(measured_distance>=70 && measured_distance<100){
	      rc = send_msg_to_kernel(cb_args.nlsock, spi_socket_genl_CMD_PATTERN,sequence2);  
        }

        // For measured_distances between 50cm and 70cm, moving Tortoise is displayed
       else if(measured_distance>=50 && measured_distance<70){
           rc = send_msg_to_kernel(cb_args.nlsock, spi_socket_genl_CMD_PATTERN,sequence3);   
        }

        // For measured_distances between 50cm and 25cm, flying bird is displayed
        else if(measured_distance>=25 && measured_distance<50){     
           rc = send_msg_to_kernel(cb_args.nlsock, spi_socket_genl_CMD_PATTERN,sequence4);    
        }

        // For measured_distances below 25cm, Dancing human is displayed
        else if(measured_distance<25){
           rc = send_msg_to_kernel(cb_args.nlsock, spi_socket_genl_CMD_PATTERN,sequence5); 
        }
	//sleeping for a specific period before starting new measurement
        sleep(PERIOD_TO_WAIT);
    }

    if (rc)
        printf("Error sending message to kernel. rc = %d\n", rc);

   return NULL;
}

int main()
{
    int rc = 0;
    struct nl_sock *nlsock = NULL;
    struct nl_cb *cb = NULL;
    pthread_t led_thread;
    struct cb_thread_args led_args;
/*Initializing the socket*/
    init_nlsock(&nlsock);

    cb = nl_cb_alloc(NL_CB_DEFAULT);
    nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, skip_seq_check, NULL);
    nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, get_measured_distance_from_kernel, NULL);
    led_args.nlsock = nlsock;
    led_args.cb = cb;

   /*Initialing the Spi thread and configuring the pins to measure distances and send appropriate patterns*/
    pthread_create(&led_thread,NULL,doSPI,(void *)&led_args);
    rc = send_msg_to_kernel(nlsock, spi_socket_genl_CMD_CONFIG, sequence1);



    if (rc)
    {
        printf("Error sending message to kernel\n");
        return rc;
    }

    pthread_join(led_thread, NULL);

    nl_cb_put(cb);
    nl_socket_free(nlsock); //Freeing the Socket

    return 0;
}
