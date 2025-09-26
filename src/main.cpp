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
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/range.h>
#include <rmw_microros/rmw_microros.h>

// Pico SDK and platform specific headers
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"

// Custom transport and project headers
#include "config.h"                 // Project-specific configuration (WiFi, ROS_DOMAIN_ID, etc)
#if USE_UART_TRANSPORT
#include "pico_uart_transports.h" // UART transport for micro-ROS agent communication
#else
#include "picow_udp_transports.h"   // UDP transport for micro-ROS agent communication
#endif
#include "Brazo.h"                  // Servo/arm control abstraction
#include "ntp_client.h"             // Simple NTP client helper

// -- ROS objects (global for simplicity in this small example) --
// Publisher that periodically publishes the 4 servo angles
rcl_publisher_t angles_publisher; // 4-servo angles
rcl_publisher_t ping_publisher;

// Subscriptions for controlling LED and servo angles
rcl_subscription_t led_subscription;    // LED control
rcl_subscription_t angles_subscription; // 4-servo array command
rcl_subscription_t speeds_subscription; // 4-servo speed command
rcl_subscription_t ping_subscriber;

// Message buffers used by the executor callbacks
std_msgs__msg__Bool led_msg;
std_msgs__msg__Header ping_pub_msg;
std_msgs__msg__Int16 ping_sub_msg;
sensor_msgs__msg__JointState angles_pub_msg;        // published message
std_msgs__msg__Float32MultiArray angles_sub_msg;    // multi-angle subscriber buffer
std_msgs__msg__Float32MultiArray speeds_sub_msg;    // multi-speed subscriber buffer

// Hardware/control objects
Brazo g_brazo = Brazo(27, 26, 22, 21);          // 4-servo arm controller
absolute_time_t BOOT_TIME;                      // pico absolute boot time (microseconds)
absolute_time_t NTP_TIME;                       // last NTP-synchronized time (microseconds)
const int OFFSET_SECONDS = 5 * 3600;            // UTC-5

// WiFi retry configuration
const uint8_t WIFI_RETRIES = 5;
const uint32_t WIFI_TIMEOUT_MS = 10000; // per-attempt timeout (ms)

volatile bool speed_control_enabled = false;

// timer_callback: runs periodically and publishes current servo angles.
// - reads the angles from the Brazo object
// - writes them into a sensor_msgs/JointState message
// - computes a wall-clock timestamp by adding the elapsed time since
//   boot to the last NTP time sample
void angles_pub_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    // Read current servo angles (assumed 4 elements)
    float brazo_angles[4];
    g_brazo.getAngles(brazo_angles);
    for (size_t i = 0; i < 4; i++) {
        // JointState::position uses double values; assignment will convert
        angles_pub_msg.position.data[i] = brazo_angles[i] + (((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f); // add +/-1.0 degree float noise
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
    printf("Received new joint angles command\n");
    auto jmsg = (const std_msgs__msg__Float32MultiArray*)msgin;

    if (jmsg->data.size != 4) {
        printf("Invalid joint data size: %zu\n", jmsg->data.size);
        return;
    }

    speed_control_enabled = false;
    for (size_t i = 0; i < 4; i++) {
        g_brazo.goDegree(i + 1, jmsg->data.data[i]);
        printf("Set joint %zu to %f degrees\n", i + 1, jmsg->data.data[i]);
    }
}

// speeds_subscription_callback: expects a Float32MultiArray where data.size == 4
// The four speeds (degrees per second) are applied to the four servos in order.
void speeds_subscription_callback(const void *msgin)
{
    auto smsg = (const std_msgs__msg__Float32MultiArray *)msgin;

    if (smsg->data.size != 4)
    {
        printf("Invalid speed data size: %zu\n", smsg->data.size);
        return;
    }

    speed_control_enabled = true;
    for (size_t i = 0; i < 4; i++)
    {
        g_brazo.setSpeed(i + 1, smsg->data.data[i]);
        printf("Set joint %zu speed to %f\n", i + 1, smsg->data.data[i]);
    }
}

// move_arm_timer_callback: if speed control is enabled, this timer callback
// runs periodically and updates the servo angles based on their speeds.
void move_arm_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    float angles[4];
    float speeds[4];

    if (!speed_control_enabled)
    {
        return;
    }
    g_brazo.getAngles(angles);
    g_brazo.getSpeeds(speeds);

    for (size_t i = 0; i < 4; i++)
    {
        if (speeds[i] == 0.0f)
        {
            continue;
        }
        float new_angle = angles[i] + speeds[i] * float(last_call_time) / 1e9f;
        if (new_angle > 180.0f)
        {
            new_angle = 180.0f;
        }
        if (new_angle < 0.0f)
        {
            new_angle = 0.0f;
        }
        g_brazo.goDegree(i + 1, new_angle);
    }
}

void ping_subscription_callback(const void* msgin){
    auto jmsg = (const std_msgs__msg__Int16*)msgin;
    int16_t ping_recivied;
    ping_recivied = jmsg->data;
    if (ping_recivied==1){
        absolute_time_t current_time = NTP_TIME + (get_absolute_time() - BOOT_TIME);
        // Convert microseconds (absolute_time_t) to sec/nsec
        ping_pub_msg.stamp.sec = current_time / 1000000;
        ping_pub_msg.stamp.nanosec = (current_time % 1000000) * 1000;
    }
    // Publish
    auto ret = rcl_publish(&ping_publisher, &ping_pub_msg, NULL);
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
    for (size_t attempt = 1; attempt <= WIFI_RETRIES; ++attempt) {
        printf("Connecting to WiFi network %s (attempt %zu/%zu)...\n", SSID, attempt, WIFI_RETRIES);
        if (!cyw43_arch_wifi_connect_timeout_ms(SSID, PSWD, CYW43_AUTH_WPA2_AES_PSK, WIFI_TIMEOUT_MS)) {
            // API returns 0 on success for this platform, so invert the check
            wifi_ok = true;
            break;
        }
        printf("Attempt %zu failed to connect to WiFi network %s\n", attempt, SSID);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
    if (!wifi_ok) {
        printf("Failed to connect to WiFi network %s after %zu attempts\n", SSID, WIFI_RETRIES);
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

#if USE_UART_TRANSPORT
    // Configure micro-ROS to use a custom UART transport
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);
#else
    // Configure micro-ROS to use a custom UDP transport tailored for Pico W
    rmw_uros_set_custom_transport(
        false,
        &picow_params,
        picow_udp_transport_open,
        picow_udp_transport_close,
        picow_udp_transport_write,
        picow_udp_transport_read);
#endif

    // rcl/rclc objects needed for node, timers, and executor
    rcl_timer_t angles_pub_timer;
    rcl_timer_t distance_pub_timer;
    rcl_timer_t move_arm_timer;
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

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, allocator);
    ret = rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);

    // Initialize rclc support/context
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // Create node and a publisher that will announce joint states
    // rclc_node_init_default(&node, NODE_NAME.c_str(), NAMESPACE.c_str(), &support);
    rclc_node_init_default(&node, "pico_w_arm", "", &support);
    rcl_publisher_options_t angle_pub_ops = rcl_publisher_get_default_options();
    angle_pub_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rclc_publisher_init(
        &angles_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states_arm",
        &angle_pub_ops.qos);

    // Create a periodic timer that calls timer_callback every 100 ms
    rclc_timer_init_default(
        &angles_pub_timer,
        &support,
        RCL_MS_TO_NS(100),
        angles_pub_timer_callback);

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

    rclc_subscription_init_default(
        &speeds_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "arm_controller/joint_speeds");

    rclc_timer_init_default(
        &move_arm_timer,
        &support,
        RCL_MS_TO_NS(100),
        move_arm_timer_callback);
    
    rclc_subscription_init_default(
        &ping_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "ping_subscriber");
    
    rclc_publisher_init_default(
        &ping_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
        "ping_publisher");
    // Executor with capacity for 7 handles (6 + 1 overhead)
    rclc_executor_init(&executor, &support.context, 7, &allocator);

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

    rclc_executor_add_subscription(
        &executor,
        &speeds_subscription,
        &speeds_sub_msg,
        speeds_subscription_callback,
        ON_NEW_DATA);
    
    rclc_executor_add_subscription(
        &executor,
        &ping_subscriber,
        &ping_sub_msg,
        ping_subscription_callback,
        ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &angles_pub_timer);
    rclc_executor_add_timer(&executor, &move_arm_timer);

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
        // Build the name in a std::string then allocate persistent storage
        // for the C string so it doesn't point into a temporary.
        std::string s = "joint_" + std::to_string(i + 1);
        angles_pub_msg.name.data[i].data = (char *)malloc(s.size() + 1);
        memcpy(angles_pub_msg.name.data[i].data, s.c_str(), s.size() + 1);
        angles_pub_msg.name.data[i].size = s.size();
        angles_pub_msg.name.data[i].capacity = s.size() + 1;
    }

    angles_sub_msg.data.data = (float *)malloc(4 * sizeof(float));
    angles_sub_msg.data.size = 0;
    angles_sub_msg.data.capacity = 4;

    speeds_sub_msg.data.data = (float *)malloc(4 * sizeof(float));
    speeds_sub_msg.data.size = 0;
    speeds_sub_msg.data.capacity = 4;

    ping_pub_msg = std_msgs__msg__Header();
    // Main loop: let the executor service callbacks and timers.
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    return 0;
}
