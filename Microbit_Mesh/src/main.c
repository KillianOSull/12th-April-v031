#include <sys/printk.h>			//include lib for printk

#include <settings/settings.h>		//include lib for settings from settings.h

#include <bluetooth/bluetooth.h>	//include lib for bluetooth settings
#include <bluetooth/mesh.h>		//include lib for mesh settings
#include "buttons.h"			//include buttons.h file for buttons functionality
#include "matrix.h" 			//include matrix.h file for matrix functionality

#define MOD_LF 0x0000			

#define GROUP_ADDR 0xc000		//defines address for group address
#define PUBLISHER_ADDR  0x000f		//defines address for publisher (microbit)

#define OP_VENDOR_BUTTON BT_MESH_MODEL_OP_3(0x00, BT_COMP_ID_LF) 
#define OP_KOS_MESSAGE BT_MESH_MODEL_OP_3(0xD0, BT_COMP_ID_LF)

#define SYN 10				//SYN number for TCP simualtion
#define ACK 20				//ACK number for TCP simualtion

static const uint8_t net_key[16] = {	//NETWORK KEY
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const uint8_t dev_key[16] = {	//DEVELOPER KEY (I THINK) 
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const uint8_t app_key[16] = {	//APPLICATION KEY (I THINK)
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};					//ALL KEYS NEED TO MATCH IN THE NETWORK IN ORDER FOR MESH TO WORK
static const uint16_t net_idx;
static const uint16_t app_idx;
static const uint32_t iv_index;
static uint8_t flags;
static uint16_t addr = NODE_ADDR;	//address is equal to node address

static void heartbeat(const struct bt_mesh_hb_sub *sub, uint8_t hops,
		      uint16_t feat)
{
	printk("|   Microbit   | Got heartbeat\n");			//prints on terminal screen
}

static struct bt_mesh_cfg_cli cfg_cli = {
};

static void attention_on(struct bt_mesh_model *model)
{
	printk("|   Microbit   | attention_on\n");			//prints on terminal screen
}

static void attention_off(struct bt_mesh_model *model)
{
	printk("|   Microbit   | attention_off()\n");			//prints on terminal screen
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
};

uint32_t unlock_prbs()					//FUNCTION FOR PRBS GENERATOR
{
    static uint32_t shift_register=0xa551199; 		//"random" seed value... 2 billion different values, called 31 bit prbs
	int b1 = 0;					//bit 1 equals 0
	int b2 = 0;					//bit 2 equals 0
	if (shift_register & (1 << 30))
	{
		b1 = 1;
	}
	if (shift_register & (1 << 27))
	{
		b2 = 1;
	}
	
	shift_register=shift_register << 1;
	shift_register=shift_register | (b1 ^ b2); //xored = ^
	shift_register = shift_register & 0x7fffffff;
    return shift_register ; // return 31 LSB's 
}

static int kos_message_received(struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{
	static uint32_t unlock_code = 0;						//Declaring an unlock_code variable
	uint32_t data = net_buf_simple_pull_le32(buf);					//used for pulling data from the mesh network and puts it into data variable
	printk("|   Microbit   | Received KOS message from NRF58382: %d\n", data);	//prints on terminal screen
	printk("|   Microbit   | RSSI=%d\n",ctx->recv_rssi);				//prints on terminal screen
	if (data == ACK){								//if the data from the mesh network is equal to the Acknowldgement... (20)
		
		printk("|   Microbit   | Recieved an ACK\n");				//prints on terminal screen
		printk("|   Microbit   | Incrementing PRBS Generator\n");		//prints on terminal screen
		
		unlock_code = unlock_prbs();						//returns PRBS value from unlock_prbs() function and puts it in unlock_code vairiable
		unlock_code = unlock_code & 0x7fffffff;					//ands the unlock code with 0x7fffffff... this limits the size of the code so it can be sent over the network
		
		printk("|   Microbit   | Sening unlock code: %x\n", unlock_code);	//prints on terminal screen
		
		sendKOSMessage(unlock_code); 						//calls "sendKOSMessage" function and sends the uint32_t unlock_code value
	}
	return 0;									//returns nothing
}
static const struct bt_mesh_model_op vnd_ops[] = {	
	{ OP_KOS_MESSAGE, BT_MESH_LEN_EXACT(4), kos_message_received },			//BT_MESH_LEN_EXACT(4) = length of packet size
	  BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model vnd_models[] = {
	BT_MESH_MODEL_VND(BT_COMP_ID_LF, MOD_LF, vnd_ops, NULL, NULL),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, vnd_models),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static void configure(void)
{
	printk("Configuring...\n");	//prints on terminal screen

	/* Add Application Key */
	bt_mesh_cfg_app_key_add(net_idx, addr, net_idx, app_idx, app_key, NULL);

	/* Bind to vendor model */
	bt_mesh_cfg_mod_app_bind_vnd(net_idx, addr, addr, app_idx,
				     MOD_LF, BT_COMP_ID_LF, NULL);

	/* Bind to Health model */
	bt_mesh_cfg_mod_app_bind(net_idx, addr, addr, app_idx,
				 BT_MESH_MODEL_ID_HEALTH_SRV, NULL);

	/* Add model subscription */
	bt_mesh_cfg_mod_sub_add_vnd(net_idx, addr, addr, GROUP_ADDR,
				    MOD_LF, BT_COMP_ID_LF, NULL);

#if NODE_ADDR == PUBLISHER_ADDR
	{
		struct bt_mesh_cfg_hb_pub pub = {		//configuration for publisher
			.dst = GROUP_ADDR,			//desitination is the group address
			.count = 0xff,				//
			.period = 0x05,				//
			.ttl = 0x07,				//time to live
			.feat = 0,				//
			.net_idx = net_idx,			//network index (I THINK)
		};

		bt_mesh_cfg_hb_pub_set(net_idx, addr, &pub, NULL);
		printk("|   Microbit   | Publishing heartbeat messages\n");	//prints on terminal screen
	}
#endif
	printk("|   Microbit   | Configuration complete\n");			//prints on terminal screen
}

static const uint8_t dev_uuid[16] = { 0xdd, 0xdd };

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
};

BT_MESH_HB_CB_DEFINE(hb_cb) = {
	.recv = heartbeat,
};

static void bt_ready(int err) //initilising bluetooth functionality and checking for errors
{
	if (err) {
		printk("|   Microbit   | Bluetooth init failed (err %d)\n", err);	//prints on terminal screen
		return;
	}//error checking for Bluetooth

	printk("|   Microbit   | Bluetooth initialized\n");				//prints on terminal screen

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);			//prints on terminal screen
		return;
	}//error checking for Mesh

	printk("Mesh initialized\n");							//prints on terminal screen

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		printk("|   Microbit   | Loading stored settings\n");			//prints on terminal screen
		settings_load();							//loading settings for bluetooth
	}//loading settings

	err = bt_mesh_provision(net_key, net_idx, flags, iv_index, addr,
				dev_key);
	if (err == -EALREADY) {
		printk("|   Microbit   | Using stored settings\n");			//prints on terminal screen
	} else if (err) {
		printk("|   Microbit   | Provisioning failed (err %d)\n", err);		//prints on terminal screen
		return;
	} else {
		printk("|   Microbit   | Provisioning completed\n");			//prints on terminal screen
		configure();	
	}

#if NODE_ADDR != PUBLISHER_ADDR
	/* Heartbeat subcscription is a temporary state (due to there
	 * not being an "indefinite" value for the period, so it never
	 * gets stored persistently. Therefore, we always have to configure
	 * it explicitly.
	 */
	{
		struct bt_mesh_cfg_hb_sub sub = {
			.src = PUBLISHER_ADDR,
			.dst = GROUP_ADDR,
			.period = 0x10,
		};

		bt_mesh_cfg_hb_sub_set(net_idx, addr, &sub, NULL);
		printk("|   Microbit   | Subscribing to heartbeat messages\n");		//prints on terminal screen
	}
#endif
}
#if NODE_ADDR != PUBLISHER_ADDR

static uint16_t target = PUBLISHER_ADDR;

#else

static uint16_t target = 0x0001; 							//message is being targeted to the NRF

#endif

void mesh_send_start(uint16_t duration, int err, void *cb_data)
{
	printk("|   Microbit   | send_start duration = %d, err = %d\n",duration,err);	//prints on terminal screen
}
void mesh_send_end(int err, void *cb_data)
{
	printk("|   Microbit   | send_end err=%d\n\n\n",err);				//prints on terminal screen
};
const struct bt_mesh_send_cb kos_send_sb_s = { 						//for sending messages
	.start = mesh_send_start,							//start of message
	.end = mesh_send_end,								//end of message
};
void sendKOSMessage(uint32_t data) //function for sending messages
{
	int err;										//creates variable called err
	NET_BUF_SIMPLE_DEFINE(msg, 3 + 4 + 4);							//used for definining the type of message and its size
	struct bt_mesh_msg_ctx ctx = {								//message attributes
		.app_idx = app_idx,								//I DO NOT KNOW
		.addr = 1,									//address = 1 (I THINK)
		.send_ttl = BT_MESH_TTL_DEFAULT,						//sending time to live is set to default
	};

	bt_mesh_model_msg_init(&msg, OP_KOS_MESSAGE);						//message initilisation
	net_buf_simple_add_le32(&msg,data);							//adding the data to a message and sending message(I THINK)
	err = bt_mesh_model_send(&vnd_models[0], &ctx, &msg,&kos_send_sb_s, NULL);		//checking if there is an error sending message
	if (err) {										//if there is an error
		printk("|   Microbit   | Unable to send KOS message %d\n",err);			//prints on terminal screen
	}

	printk("|   Microbit   | KOS message sent with OpCode 0x%08x\n", OP_KOS_MESSAGE);	//prints on terminal screen
}
uint16_t board_set_target(void)
{
	switch (target) {
	case GROUP_ADDR:
		target = 1U;
		break;
	case 9:
		target = GROUP_ADDR;
		break;
	default:
		target++;
		break;
	}

	return target;
}



void button_a_callback() 						//for when button A is pressed
{
	printk("|   Microbit   | Button A pressed\n");			//prints on terminal screen
	printk("|   Microbit   | Sending SYN to NRF58382\n"); 		//prints on terminal screen
	sendKOSMessage(SYN); 						//Send a SYN
} 
void button_b_callback()						//for when button B is pressed
{
	printk("|   Microbit   | Button B pressed\n");			//prints on terminal screen
	printk("|   Microbit   | Sending Lock Code\n");			//prints on terminal screen
	sendKOSMessage(0xf109c5); 					//Send unlock code
}
void main(void)										//MAIN
{
	int err;									//creates variable called err
	uint8_t rows = 1;								//initilsing rows for led matrix 
	uint8_t cols = 1;								//initilsing cols for led matrix
	printk("|   Microbit   | Initializing...\n");					//prints on terminal screen
	if (err) {									//if there is an error
		printk("|   Microbit   | Board initialization failed\n");		//prints on terminal screen
		return;
	}										//error checking for board initilisation
	err = matrix_begin();								//calls matrix_begin from "matrix.c"... initilises the GPIO on the matrix and equals it to err
	if (err < 0)									//if there is an error
	{
		printf("\n|   Microbit   | Error initializing buttons.  Error code = %d\n",err);	//prints on terminal screen	
	 	while(1);
	}										//error checking for led matrix
	
	printk("|   Microbit   | Unicast address: 0x%04x\n", addr);			//prints on terminal screen

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);							//calls bt_ready(int err)... does various checks to ensure bluetooth is ready
	if (err) {									//if there is an error
		printk("|   Microbit   | Bluetooth init failed (err %d)\n", err);	//prints on terminal screen
		return;
	}										//error checking for bluetooth
	
	buttons_begin();								//initialize the user buttons
	attach_callback_to_button_a(button_a_callback);					//callback for button A
	attach_callback_to_button_b(button_b_callback);					//callback for button B

	while(1)
	{	
		if(get_buttonA() == 0)			//IF LOGIC FOR LED DISPLAY ON MICTOBIT WHEN BUTTON B IS PRESSED
		{
			//displaying led pattern
			rows =  0b11111;		// sets the rows to all on		
	   		cols =  0b10000; 		// sets the cols to all off
	   		matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
	   		k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds 
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b01000; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b00100; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b00010; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b00001; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b00010; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b00100; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b01000; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b10000; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b01000; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b00100; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b00010; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b00001; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b00010; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b00100; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b01000; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b11111;		// sets the rows to all on		
			cols =  0b10000; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds

			matrix_all_off();		// turns off all LED's
			k_sleep(K_SECONDS(1));          // pauses programe for 1 second
		}
		
		if(get_buttonB() == 0)			//IF LOGIC FOR LED DISPLAY ON MICTOBIT WHEN BUTTON B IS PRESSED
		{
			
			rows =  0b10000;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b01000;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b00100;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b00010;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b00001;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b00010;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b00100;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b01000;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b10000;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b01000;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b00100;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b00010;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b00001;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b00010;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b00100;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b01000;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds
			
			rows =  0b10000;		// sets the rows to all on		
			cols =  0b11111; 		// sets the cols to all off
			matrix_put_pattern(rows, ~cols);// put the pattern on the matrix (tilda, ~ inverts cols)
			k_sleep(K_SECONDS(0.05));	// pauses programe for 0.05 seconds

			matrix_all_off();		// turns off all LED's
			k_sleep(K_SECONDS(1));		// pauses programe for 1 second
		}
		
	}//end of while 1	

}//end of main
