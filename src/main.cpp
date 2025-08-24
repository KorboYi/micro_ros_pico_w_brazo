// main.cpp
// Micro-ROS example for a Pico W that controls a 4-servo arm (Brazo).
// Added comments to explain initialization, WiFi/NTP handling, ROS setup,
// and the pub/sub/callback behavior.

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <string>
#include <stdexcept>

// micro-ROS and ROS2 C API headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
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
rcl_publisher_t angles_publisher;

// Subscriptions for controlling LED and servo angles
rcl_subscription_t led_subscription;
rcl_subscription_t angles_subscription;  // 4-servo array command

// Message buffers used by the executor callbacks
std_msgs__msg__Bool led_msg;
sensor_msgs__msg__JointState angles_pub_msg;   // published message
std_msgs__msg__Float32MultiArray angles_sub_msg;  // multi-angle subscriber buffer

// Hardware/control objects
Brazo g_brazo = Brazo();               // 4-servo arm controller
absolute_time_t BOOT_TIME;           // pico absolute boot time (microseconds)
absolute_time_t NTP_TIME;            // last NTP-synchronized time (microseconds)
const int OFFSET_SECONDS = 5 * 3600; // UTC-5

// WiFi credentials and retry configuration
const char* SSID = "redpucp";
const char* PSWD = "C9AA28BA93";
const uint8_t WIFI_RETRIES = 5;
const uint32_t WIFI_TIMEOUT_MS = 10000; // per-attempt timeout (ms)

// IP address and port of the micro-ROS agent
// Change this to the agent IP and port on your network.
const char* ROS_AGENT_IP_ADDR = "10.101.13.109";
const int ROS_AGENT_UDP_PORT = 8888;

// Node and namespace identifiers
// Change this to match your robot's ID
const uint8_t BRAZO_ID = 1;
const std::string NODE_NAME = "pico_w_brazo_" + std::to_string(BRAZO_ID);
const std::string NAMESPACE = "brazo_" + std::to_string(BRAZO_ID);


// timer_callback: runs periodically and publishes current servo angles.
// - reads the angles from the Brazo object
// - writes them into a sensor_msgs/JointState message
// - computes a wall-clock timestamp by adding the elapsed time since
//   boot to the last NTP time sample
void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    // Read current servo angles (assumed 4 elements)
    float brazo_angles[4];
    g_brazo.getAngles(brazo_angles);
    for (size_t i = 0; i < 4; i++) {
        // JointState::position uses double values; assignment will convert
        angles_pub_msg.position.data[i] = brazo_angles[i];
    }

    // Compute current wall-clock time based on ntp_time and uptime.
    // The Pico provides only an uptime clock, so we offset it by the
    // last-known NTP time to obtain an approximate current timestamp.
    absolute_time_t current_time = NTP_TIME + (get_absolute_time() - BOOT_TIME);
    // Convert microseconds (absolute_time_t) to sec/nsec
    angles_pub_msg.header.stamp.sec = current_time / 1000000;
    angles_pub_msg.header.stamp.nanosec = (current_time % 1000000) * 1000;

    // Publish
    auto ret = rcl_publish(&angles_publisher, &angles_pub_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing angles: %s\n", rcl_get_error_string().str);
        rcl_reset_error();
    }
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

// angles_subscription_callback: expects a JointState where position.size == 4
// The four positions are applied to the four servos in order.
void angles_subscription_callback(const void* msgin)
{
    auto jmsg = (const std_msgs__msg__Float32MultiArray*)msgin;

    if (jmsg->data.size != 4) {
        printf("Invalid joint data size: %zu\n", jmsg->data.size);
        return;
    }

    for (size_t i = 0; i < 4; i++) {
        g_brazo.goDegree(i+1, jmsg->data.data[i]);
        printf("Set joint %zu to %f degrees\n", i+1, jmsg->data.data[i]);   
    }
}

int main()
{
    // Enable stdio over USB/serial for debugging output.
    stdio_init_all();
    sleep_ms(2000);
    printf("\n\nPico W micro-ROS Brazo starting up...\n");

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
    NTP_TIME = ntp_client_get_time(5000);
    BOOT_TIME = get_absolute_time();

    // Print NTP time as human-readable string adjusted to local time (UTC-5)
    if (NTP_TIME != nil_time) {
        time_t ntp_sec = (time_t)((int64_t)NTP_TIME / 1000000LL);
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
    printf("Boot time (us since boot): %llu\n", BOOT_TIME);

    // Initialize servo controller (calibration, pin setup inside Brazo)
    g_brazo.init();

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
    rclc_node_init_default(&node, NODE_NAME.c_str(), NAMESPACE.c_str(), &support);
    rclc_publisher_init_default(
        &angles_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states");

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
        "led_control");

    rclc_subscription_init_default(
        &angles_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "arm_controller/joint_commands");

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
    angles_pub_msg.name.size = 4;
    angles_pub_msg.name.capacity = 4;
    angles_pub_msg.name.data = (rosidl_runtime_c__String*)malloc(4 * sizeof(rosidl_runtime_c__String));
    angles_pub_msg.position.size = 4;
    angles_pub_msg.position.capacity = 4;
    angles_pub_msg.position.data = (double*)malloc(4 * sizeof(double));
    for (size_t i = 0; i < 4; i++) {
        angles_pub_msg.name.data[i].data = ("joint" + std::to_string(i + 1)).data();
        angles_pub_msg.name.data[i].size = strlen(angles_pub_msg.name.data[i].data);
        angles_pub_msg.name.data[i].capacity = angles_pub_msg.name.data[i].size + 1;
    }

    // Main loop: let the executor service callbacks and timers.
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
