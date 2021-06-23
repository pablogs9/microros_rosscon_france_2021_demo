#include <unistd.h>

#include "led_strip.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/header.h>

static rclc_parameter_server_t param_server;
static rcl_publisher_t publisher;

std_msgs__msg__Int32 input_msg;
std_msgs__msg__Header output_msg;
uint8_t output_msg_buffer[60];

// ###############################
//      LED STRIP CONTROLLER
// ###############################

extern led_strip_t *strip;
extern led_strip_config_t strip_config;

void set_led_strip(size_t number, double red, double green, double blue)
{
	strip->clear(strip, 100);
	for (size_t i = 0; i < number; i++){
		strip->set_pixel(strip, i, red*255, green*255, blue*255);
	}
	strip->refresh(strip, 100);
}

void update_led_strip()
{
	bool toogle;
	rclc_parameter_get_bool(&param_server, "toogle", &toogle);

	if (toogle) {
		int number;
		double red, green, blue;
		rclc_parameter_get_int(&param_server,"number", &number);
		rclc_parameter_get_double(&param_server,"red", &red);
		rclc_parameter_get_double(&param_server,"green", &green);
		rclc_parameter_get_double(&param_server,"blue", &blue);

		set_led_strip(number, red, green, blue);
	} else {
		set_led_strip(8, 0.0, 0.0, 0.0);
	}
}


// ###############################
//      micro-ROS CALLBACKS
// ###############################

void subscription_callback(const void * msgin)
{
  	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("RECEIVED SUBSCRIPTION: %d\n", msg->data);

	// Change the frame id name
	char aux[50];
	snprintf(aux, sizeof(aux), "led_node_%d", msg->data);
	output_msg.frame_id = micro_ros_string_utilities_set(output_msg.frame_id, aux);
}

void on_parameter_changed(Parameter * param)
{
	printf("PARAMETER MODIFIED: %s\n", param->name.data);
	update_led_strip();
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	RCLC_UNUSED(timer);

	int64_t timestamp = rmw_uros_epoch_nanos();
	output_msg.stamp.sec = timestamp / 1000000000UL;
	output_msg.stamp.nanosec = timestamp % 1000000000UL;

	rcl_publish(&publisher, &output_msg, NULL);
}


// ###############################
//      micro-ROS TASK
// ###############################

void micro_ros_task(void)
{
    // Init micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	rcl_init_options_init(&init_options, allocator);
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);

	// Setup support structure.
	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	// Sync session for having a time&epoch reference
	rmw_uros_sync_session(100);

	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	rclc_node_init_default(&node, "led_node", "", &support);

	// Create publisher
	rclc_publisher_init_best_effort(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
		"led_publisher");

	// Create subscriber
	rcl_subscription_t subscriber;
	rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"led_subscriber");


	// Create parameter service
	rclc_parameter_options_t param_options = (rclc_parameter_options_t){.max_params = 5};
	rclc_parameter_server_init_with_option(&param_server, &node, &param_options);

	// Create timer
	rcl_timer_t timer;
	rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(100),
		timer_callback);

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER + 2, &allocator);
	rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);
  	rclc_executor_add_timer(&executor, &timer);
	rclc_executor_add_subscription(&executor, &subscriber, &input_msg, &subscription_callback, ON_NEW_DATA);

	// Create parameters
	rclc_add_parameter(&param_server, "toogle", RCLC_PARAMETER_BOOL);
  	rclc_add_parameter(&param_server, "number", RCLC_PARAMETER_INT);
  	rclc_add_parameter(&param_server, "red", RCLC_PARAMETER_DOUBLE);
  	rclc_add_parameter(&param_server, "green", RCLC_PARAMETER_DOUBLE);
  	rclc_add_parameter(&param_server, "blue", RCLC_PARAMETER_DOUBLE);

	rclc_parameter_set_bool(&param_server, "toogle", false);
	rclc_parameter_set_int(&param_server, "number", 8);
	rclc_parameter_set_double(&param_server, "red", 1.0);

	// Assing memory to message types
	micro_ros_utilities_create_static_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
		&output_msg,
		(micro_ros_utilities_memory_conf_t) {0},
		output_msg_buffer,
		sizeof(output_msg_buffer)
	);

	output_msg.frame_id = micro_ros_string_utilities_set(output_msg.frame_id, "led_node_x");
	int64_t timestamp = rmw_uros_epoch_nanos();
	output_msg.stamp.sec = timestamp / 1000000000UL;
	output_msg.stamp.nanosec = timestamp % 1000000000UL;

	// Spin micro-ROS node
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}
}
