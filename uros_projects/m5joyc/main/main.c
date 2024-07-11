// todo: add readme

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/joy.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "driver/i2c.h"
#include "driver/gpio.h"

#include <rosidl_runtime_c/string.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
sensor_msgs__msg__Joy joy_msg;

#define NUM_AXES 4
#define NUM_BUTTONS 2

#define BLINK_GPIO 10
static uint8_t s_led_state = 0;

float axes_data[NUM_AXES];
int32_t buttons_data[NUM_BUTTONS];

#define I2C_MASTER_SCL_IO           26      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           0      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define JOYC_SENSOR_ADDR                 0x38        /*!< Slave address of the JOYC sensor */

int32_t counter = 0;



static esp_err_t joyc_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, JOYC_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		// printf("Joy Publishing: %d\n", (int) ++counter);
   		uint8_t data1[5];
		joyc_register_read(0x60, data1, 5);
		// uint8_t data2[8];
		// joyc_register_read(0x70, data2, 8);


		axes_data[0] = ( 100-(float)data1[0])/100; // xo
		axes_data[1] = (-100+(float)data1[1])/100; // yo
		axes_data[2] = ( 100-(float)data1[2])/100; // x1
		axes_data[3] = (-100+(float)data1[3])/100; // y1
		buttons_data[0] = (data1[4] & 0x10) > 0;
		buttons_data[1] = (data1[4] & 0x01) > 0;

		// joy_msg.header.stamp.sec = esp_timer_get_time() / 1000000;
		joy_msg.header.stamp.nanosec = last_call_time;

		rosidl_runtime_c__String frame_id;
		frame_id.capacity=100;
		frame_id.data = "M5_JoyC";
		frame_id.size = strlen(frame_id.data);
		joy_msg.header.frame_id = frame_id;

		joy_msg.axes.data = axes_data;
		joy_msg.axes.size = NUM_AXES;

		joy_msg.buttons.data = buttons_data;
		joy_msg.buttons.size = NUM_BUTTONS;
		
		RCSOFTCHECK(rcl_publish(&publisher, &joy_msg, NULL));
	}
}

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}


void micro_ros_task(void * arg)
{
	i2c_master_init();

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "m5joyc", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
		"m5joyc_joy"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 100;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	
	/* Configure the peripheral according to the LED type */
    configure_led();

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
		++counter;
		
		if(counter%50==0){
			gpio_set_level(BLINK_GPIO, s_led_state);
			s_led_state = !s_led_state;
		}
			
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
