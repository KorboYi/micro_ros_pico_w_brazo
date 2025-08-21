#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "picow_udp_transports.h"
#include "Brazo.h"
#include "ntp_client.h"

rcl_publisher_t angle_publisher;
rcl_subscription_t led_subscription;
rcl_subscription_t angle_subscription;
rcl_subscription_t angles_subscription;
std_msgs__msg__Bool led_msg;
sensor_msgs__msg__JointState angle_pub_msg;
sensor_msgs__msg__JointState angle_sub_msg;
sensor_msgs__msg__JointState angles_sub_msg;

Brazo brazo = Brazo();
absolute_time_t boot_time;
absolute_time_t ntp_time;

const char* SSID = "redpucp";
const char* PSWD = "C9AA28BA93";

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    float brazo_angles[4];
    brazo.getAngles(brazo_angles);
    for (size_t i = 0; i < 4; i++) {
        angle_pub_msg.position.data[i] = brazo_angles[i];
    }

    absolute_time_t current_time = ntp_time + (get_absolute_time() - boot_time);
    angle_pub_msg.header.stamp.sec = current_time / 1000000;
    angle_pub_msg.header.stamp.nanosec = (current_time % 1000000) * 1000;

    rcl_ret_t ret = rcl_publish(&angle_publisher, &angle_pub_msg, NULL);
}

void led_subscription_callback(const void* msgin)
{
    auto bmsg = (const std_msgs__msg__Bool*)msgin;
    if (bmsg->data) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
    else {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    }
}

void angle_subscription_callback(const void* msgin)
{
    auto jmsg = (const sensor_msgs__msg__JointState*)msgin;
    if (jmsg->name.size != 1 || jmsg->position.size != 1) {
        printf("Received angle message with unexpected size: %zu names, %zu positions\n", jmsg->name.size, jmsg->position.size);
        return;
    }
    uint8_t servo_index = jmsg->name.data->data[0] - '1';
    if (servo_index < 0 || servo_index > 3) {
        printf("Invalid servo index: %d\n", servo_index);
        return;
    }
    float angle = jmsg->position.data[0];
    brazo.goDegree(servo_index, angle);
}

void angles_subscription_callback(const void* msgin)
{
    auto jmsg = (const sensor_msgs__msg__JointState*)msgin;
    if (jmsg->position.size != 4) {
        printf("Received angles array with unexpected size: %zu elements\n", jmsg->name.size);
        return;
    }
    float d1 = jmsg->position.data[0]; 
    float d2 = jmsg->position.data[1];
    float d3 = jmsg->position.data[2];
    float d4 = jmsg->position.data[3];
    brazo.goDegrees(d1, d2, d3, d4);
}

int main()
{
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_PERU)) {
        printf("Failed to initialize CYW43 architecture\n");
        return 1;
    }

    stdio_init_all();

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(SSID, PSWD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Failed to connect to WiFi network %s\n", SSID);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        return 1;
    }

    if (!ntp_client_init()) {
        printf("Failed to initialize NTP client\n");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        return 1;
    }

    ntp_time = ntp_client_get_time(5000);
    boot_time = get_absolute_time();

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
        &angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "pico_count_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);

    rclc_subscription_init_default(
        &led_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "pico_led_subscriber");

    rclc_subscription_init_default(
        &angle_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "pico_angle_subscriber");

    rclc_subscription_init_default(
        &angles_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
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
        &angle_sub_msg,
        angle_subscription_callback,
        ON_NEW_DATA);

    rclc_executor_add_subscription(
        &executor,
        &angles_subscription,
        &angles_sub_msg,
        angles_subscription_callback,
        ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &timer);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    angle_pub_msg.name.size = 4;
    angle_pub_msg.name.capacity = 4;
    angle_pub_msg.name.data = (rosidl_runtime_c__String*)malloc(4 * sizeof(rosidl_runtime_c__String));
    angle_pub_msg.position.size = 4;
    angle_pub_msg.position.capacity = 4;
    angle_pub_msg.position.data = (double*)malloc(4 * sizeof(double));
    for (size_t i = 0; i < 4; i++) {
        angle_pub_msg.name.data[i].data = (char*)malloc(2);
        angle_pub_msg.name.data[i].data[0] = '1' + i;
        angle_pub_msg.name.data[i].data[1] = '\0';
        angle_pub_msg.name.data[i].size = 1;
        angle_pub_msg.name.data[i].capacity = 2;
    }

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
