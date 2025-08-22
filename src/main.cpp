// main.cpp
// Micro-ROS example for a Pico W that controls a 4-servo arm (Brazo).
// Added comments to explain initialization, WiFi/NTP handling, ROS setup,
// and the pub/sub/callback behavior.

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>

// micro-ROS and ROS2 C API headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rmw_microros/rmw_microros.h>

// Pico SDK and platform specific headers
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"

// Custom transport and project headers
#include "picow_udp_transports.h" // UDP transport for micro-ROS agent communication
#include "Brazo.h"               // Servo/arm control abstraction
#include "ntp_client.h"         // Simple NTP client helper

// -- ROS objects (global for simplicity in this small example) --
// Publisher that periodically publishes the 4 servo angles
rcl_publisher_t angle_publisher;

// Subscriptions for controlling LED and servo angles
rcl_subscription_t led_subscription;
rcl_subscription_t angle_subscription;   // single-servo command
rcl_subscription_t angles_subscription;  // 4-servo array command

// Message buffers used by the executor callbacks
std_msgs__msg__Bool led_msg;
sensor_msgs__msg__JointState angle_pub_msg;   // published message
sensor_msgs__msg__JointState angle_sub_msg;   // single-angle subscriber buffer
sensor_msgs__msg__JointState angles_sub_msg;  // multi-angle subscriber buffer

// Hardware/control objects
Brazo brazo = Brazo();               // 4-servo arm controller
absolute_time_t boot_time;           // pico absolute boot time (microseconds)
absolute_time_t ntp_time;            // last NTP-synchronized time (microseconds)

// WiFi credentials and retry configuration
const char* SSID = "redpucp";
const char* PSWD = "C9AA28BA93";
const int WIFI_RETRIES = 5;
const int WIFI_TIMEOUT_MS = 10000; // per-attempt timeout (ms)

// timer_callback: runs periodically and publishes current servo angles.
// - reads the angles from the Brazo object
// - writes them into a sensor_msgs/JointState message
// - computes a wall-clock timestamp by adding the elapsed time since
//   boot to the last NTP time sample
void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    // Read current servo angles (assumed 4 elements)
    float brazo_angles[4];
    brazo.getAngles(brazo_angles);
    for (size_t i = 0; i < 4; i++) {
        // JointState::position uses double values; assignment will convert
        angle_pub_msg.position.data[i] = brazo_angles[i];
    }

    // Compute current wall-clock time based on ntp_time and uptime.
    // The Pico provides only an uptime clock, so we offset it by the
    // last-known NTP time to obtain an approximate current timestamp.
    absolute_time_t current_time = ntp_time + (get_absolute_time() - boot_time);
    // Convert microseconds (absolute_time_t) to sec/nsec
    angle_pub_msg.header.stamp.sec = current_time / 1000000;
    angle_pub_msg.header.stamp.nanosec = (current_time % 1000000) * 1000;

    // Publish (error handling omitted here to keep example compact)
    rcl_ret_t ret = rcl_publish(&angle_publisher, &angle_pub_msg, NULL);
    (void)ret; // silence unused variable warning when debugging
}

// led_subscription_callback: turn the Pico W built-in LED on/off.
// Expects a std_msgs/Bool message where true => LED on.
void led_subscription_callback(const void* msgin)
{
    auto bmsg = (const std_msgs__msg__Bool*)msgin;
    if (bmsg->data) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        printf("LED ON\n");
    }
    else {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        printf("LED OFF\n");
    }
}

// angle_subscription_callback: expects a JointState message containing
// exactly one name and one position. The name is a single character
// '1'..'4' that identifies the servo index. The position is the angle.
// This is a compact command format used for single-servo control.
void angle_subscription_callback(const void* msgin)
{
    auto jmsg = (const sensor_msgs__msg__JointState*)msgin;
    if (jmsg->name.size != 1 || jmsg->position.size != 1) {
        printf("Received angle message with unexpected size: %zu names, %zu positions\n", jmsg->name.size, jmsg->position.size);
        return;
    }
    // The name is stored as a rosidl_runtime_c__String; here we pick the
    // first byte and interpret it as ASCII '1'..'4'.
    uint8_t servo_index = jmsg->name.data->data[0] - '1';
    if (servo_index > 3) {
        // servo_index is unsigned, so only upper bound needed
        printf("Invalid servo index: %d\n", servo_index);
        return;
    }
    float angle = jmsg->position.data[0];
    brazo.goDegree(servo_index, angle);
    printf("Set angle of servo %d to %f\n", servo_index, angle);
}

// angles_subscription_callback: expects a JointState where position.size == 4
// The four positions are applied to the four servos in order.
void angles_subscription_callback(const void* msgin)
{
    auto jmsg = (const sensor_msgs__msg__JointState*)msgin;
    if (jmsg->position.size != 4) {
        // print the unexpected size (use name.size for diagnostic clarity)
        printf("Received angles array with unexpected size: %zu elements\n", jmsg->name.size);
        return;
    }
    float d1 = jmsg->position.data[0]; 
    float d2 = jmsg->position.data[1];
    float d3 = jmsg->position.data[2];
    float d4 = jmsg->position.data[3];
    brazo.goDegrees(d1, d2, d3, d4);
    printf("Set angles of servos to: %f, %f, %f, %f\n", d1, d2, d3, d4);
}

int main()
{
    // Enable stdio over USB/serial for debugging output.
    stdio_init_all();

    // Initialize CYW43 (WiFi/LED) with country regulatory domain.
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_PERU)) {
        printf("Failed to initialize CYW43 architecture\n");
        return 1;
    }

    // Turn on the Pico W status LED while booting
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Enable station mode so the device can connect to an AP
    cyw43_arch_enable_sta_mode();

    // Try connecting to WiFi up to WIFI_RETRIES with a timeout per attempt.
    // This loop toggles the LED on failure as a visual indicator.
    bool wifi_ok = false;
    for (int attempt = 1; attempt <= WIFI_RETRIES; ++attempt) {
        printf("Connecting to WiFi network %s (attempt %d/%d)...\n", SSID, attempt, WIFI_RETRIES);
        if (!cyw43_arch_wifi_connect_timeout_ms(SSID, PSWD, CYW43_AUTH_WPA2_AES_PSK, WIFI_TIMEOUT_MS)) {
            // API returns 0 on success for this platform, so invert the check
            wifi_ok = true;
            break;
        }
        printf("Attempt %d failed to connect to WiFi network %s\n", attempt, SSID);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
    if (!wifi_ok) {
        printf("Failed to connect to WiFi network %s after %d attempts\n", SSID, WIFI_RETRIES);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        return 1;
    }
    printf("Connected to WiFi network %s with IP address %s\n", SSID, ipaddr_ntoa(&cyw43_state.netif->ip_addr));
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Initialize a lightweight NTP client to obtain a wall-clock time.
    if (!ntp_client_init()) {
        printf("Failed to initialize NTP client\n");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        return 1;
    }

    // Get an initial NTP time (blocking with timeout) and capture boot time
    ntp_time = ntp_client_get_time(5000);
    boot_time = get_absolute_time();

    // Print NTP time as human-readable string adjusted to local time (UTC-5)
    const int OFFSET_SECONDS = 5 * 3600; // UTC-5
    if (ntp_time != nil_time) {
        time_t ntp_sec = (time_t)((int64_t)ntp_time / 1000000LL);
        time_t ntp_local = ntp_sec - OFFSET_SECONDS;
        struct tm tm_local;
        char buf[64] = {0};
        if (gmtime_r(&ntp_local, &tm_local)) {
            strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC-5", &tm_local);
            printf("NTP time (local): %s\n", buf);
        } else {
            printf("NTP time: (invalid)\n");
        }
    } else {
        printf("NTP time: (nil)\n");
    }
    // Boot time (microseconds since boot)
    printf("Boot time (us since boot): %llu\n", boot_time);

    // Initialize servo controller (calibration, pin setup inside Brazo)
    brazo.init();

    // Configure micro-ROS to use a custom UDP transport tailored for Pico W
    rmw_uros_set_custom_transport(
        false,
        &picow_params,
        picow_udp_transport_open,
        picow_udp_transport_close,
        picow_udp_transport_write,
        picow_udp_transport_read
    );

    // rcl/rclc objects needed for node, timers, and executor
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for up to ~2 minutes. This blocks until
    // the micro-ROS agent is reachable so the node can communicate.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Agent unreachable; return error code from rmw layer
        return ret;
    }

    // Initialize rclc support/context
    rclc_support_init(&support, 0, NULL, &allocator);

    // Create node and a publisher that will announce joint states
    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "pico_count_publisher");

    // Create a periodic timer that calls timer_callback every 100 ms
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);

    // Initialize subscriptions for LED control and angle commands
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

    // Executor with capacity for 5 handles (3 subs + timer + spare)
    rclc_executor_init(&executor, &support.context, 5, &allocator);

    // Register subscriptions with the executor and provide the message buffers
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

    // Turn status LED on to indicate WiFi/NTP and micro-ROS initialization
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("micro-ROS node initialized and running.\n");

    // --- Prepare the JointState message used for publishing angles ---
    // We manually allocate the arrays here because rosidl generators are
    // not used to construct these messages on this constrained platform.
    angle_pub_msg.name.size = 4;
    angle_pub_msg.name.capacity = 4;
    angle_pub_msg.name.data = (rosidl_runtime_c__String*)malloc(4 * sizeof(rosidl_runtime_c__String));
    angle_pub_msg.position.size = 4;
    angle_pub_msg.position.capacity = 4;
    angle_pub_msg.position.data = (double*)malloc(4 * sizeof(double));
    for (size_t i = 0; i < 4; i++) {
        // Each name is a short string like "1", "2", ... used by the
        // single-servo command format elsewhere in the code.
        angle_pub_msg.name.data[i].data = (char*)malloc(2);
        angle_pub_msg.name.data[i].data[0] = '1' + i;
        angle_pub_msg.name.data[i].data[1] = '\0';
        angle_pub_msg.name.data[i].size = 1;
        angle_pub_msg.name.data[i].capacity = 2;
    }

    // Main loop: let the executor service callbacks and timers.
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
