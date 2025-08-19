#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "picow_udp_transports.h"
#include "Brazo.h"

rcl_publisher_t count_publisher;
rcl_subscription_t led_subscription;
rcl_subscription_t angle_subscription;
rcl_subscription_t angles_subscription;
std_msgs__msg__Int32 count_msg;
std_msgs__msg__Bool led_msg;
std_msgs__msg__Float32 angle_msg;
std_msgs__msg__Float32MultiArray angles_msg;

Brazo brazo = Brazo();

const char* SSID = "redpucp";
const char* PSWD = "C9AA28BA93";

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&count_publisher, &count_msg, NULL);

    count_msg.data++;
}

void led_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Bool * bmsg = (const std_msgs__msg__Bool *)msgin;
    if (bmsg->data) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    } else {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    }
    count_msg.data = 0;
}

void angle_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * amsg = (const std_msgs__msg__Float32 *)msgin;
    brazo.goDegree(1, amsg->data);
}

void angles_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Float32MultiArray * amsg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (amsg->data.size < 4) {
        printf("Received angles array with insufficient data: %zu elements\n", amsg->data.size);
        return;
    }
    float v0 = amsg->data.data[0];
    float v1 = amsg->data.data[1];
    float v2 = amsg->data.data[2];
    float v3 = amsg->data.data[3];
    brazo.goDegrees(v0, v1, v2, v3);
}

int main()
{
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_PERU)) {
        return 1;
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(SSID, PSWD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        return 1;
    }

    stdio_init_all();

    brazo.init();

    rmw_uros_set_custom_transport(
		false,
		&picow_params,
		picow_udp_transport_open,
		picow_udp_transport_close,
		picow_udp_transport_write,
		picow_udp_transport_read
	);
    
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &count_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_count_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_subscription_init_default(
        &led_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "pico_led_subscriber");

    rclc_subscription_init_default(
        &angle_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pico_angle_subscriber");

    rclc_subscription_init_default(
        &angles_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "pico_angles_subscriber");

    rclc_executor_init(&executor, &support.context, 5, &allocator);

    rclc_executor_add_subscription(
        &executor,
        &led_subscription,
        &led_msg,
        led_subscription_callback,
        ON_NEW_DATA);

    rclc_executor_add_subscription(
        &executor,
        &angle_subscription,
        &angle_msg,
        angle_subscription_callback,
        ON_NEW_DATA);

    rclc_executor_add_subscription(
        &executor,
        &angles_subscription,
        &angles_msg,
        angles_subscription_callback,
        ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &timer);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    count_msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
