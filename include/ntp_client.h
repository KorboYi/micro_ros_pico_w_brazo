/**
 * @file ntp_client.h
 * @brief Simple NTP client for Raspberry Pi Pico W that returns time in absolute_time_t
 *
 * Requires:
 *  - pico/stdlib.h
 *  - pico/cyw43_arch.h
 *  - lwIP stack enabled
 */

#ifndef NTP_CLIENT_H
#define NTP_CLIENT_H

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/dns.h"

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief Initialize the NTP client.
     *
     * Must be called after Wi-Fi connection is established.
     * @return true if initialization succeeded, false otherwise
     */
    bool ntp_client_init(void);

    /**
     * @brief Query NTP server and return time as absolute_time_t (UTC).
     *
     * This function blocks until an NTP reply is received or timeout occurs.
     *
     * @param timeout_ms Maximum time to wait for a response (milliseconds).
     * @return absolute_time_t NTP time, or nil_time if failed.
     */
    absolute_time_t ntp_client_get_time(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // NTP_CLIENT_H
