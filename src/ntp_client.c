/**
 * @file ntp_client.c
 * @brief Implementation of NTP client returning absolute_time_t
 */

#include "ntp_client.h"
#include <string.h>
#include <time.h>
#include <stdlib.h>

#define NTP_SERVER       "pool.ntp.org"
#define NTP_MSG_LEN      48
#define NTP_PORT         123
#define NTP_DELTA        2208988800UL // seconds between 1900 and 1970

typedef struct {
    ip_addr_t ntp_server_address;
    struct udp_pcb* pcb;
    absolute_time_t ntp_time;
    bool response_received;
} ntp_state_t;

static ntp_state_t* state = NULL;

// Convert epoch seconds to absolute_time_t
static absolute_time_t ntp_to_absolute_time(uint32_t seconds_since_1970) {
    int64_t us_since_epoch = ((int64_t)seconds_since_1970) * 1000000LL;
    return us_since_epoch;
}

// Handle received NTP response
static void ntp_recv_cb(void* arg, struct udp_pcb* pcb,
    struct pbuf* p, const ip_addr_t* addr, u16_t port) {
    ntp_state_t* s = (ntp_state_t*)arg;

    if (p && p->tot_len == NTP_MSG_LEN && port == NTP_PORT) {
        uint8_t seconds_buf[4] = { 0 };
        pbuf_copy_partial(p, seconds_buf, sizeof(seconds_buf), 40);
        uint32_t seconds_since_1900 =
            (seconds_buf[0] << 24) | (seconds_buf[1] << 16) |
            (seconds_buf[2] << 8) | (seconds_buf[3]);
        uint32_t seconds_since_1970 = seconds_since_1900 - NTP_DELTA;

        s->ntp_time = ntp_to_absolute_time(seconds_since_1970);
        s->response_received = true;
    }

    if (p) pbuf_free(p);
}

// Send an NTP request
static void ntp_send_request(ntp_state_t* s) {
    struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    if (!p) return;

    uint8_t* req = (uint8_t*)p->payload;
    memset(req, 0, NTP_MSG_LEN);
    req[0] = 0x1B; // LI=0, VN=3, Mode=3 (client)

    udp_sendto(s->pcb, p, &s->ntp_server_address, NTP_PORT);
    pbuf_free(p);
}

// DNS callback
static void ntp_dns_cb(const char* hostname, const ip_addr_t* ipaddr, void* arg) {
    ntp_state_t* s = (ntp_state_t*)arg;
    if (ipaddr) {
        s->ntp_server_address = *ipaddr;
        ntp_send_request(s);
    }
}

bool ntp_client_init(void) {
    if (state) return true; // already initialized

    state = (ntp_state_t*)calloc(1, sizeof(ntp_state_t));
    if (!state) return false;

    state->pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!state->pcb) {
        free(state);
        state = NULL;
        return false;
    }

    udp_recv(state->pcb, ntp_recv_cb, state);
    return true;
}

absolute_time_t ntp_client_get_time(uint32_t timeout_ms) {
    if (!state) return nil_time;

    state->response_received = false;
    state->ntp_time = nil_time;

    int err = dns_gethostbyname(NTP_SERVER, &state->ntp_server_address,
        ntp_dns_cb, state);
    if (err == ERR_OK) {
        ntp_send_request(state); // cached DNS
    }
    else if (err != ERR_INPROGRESS) {
        return nil_time;
    }

    // Wait until response or timeout
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!state->response_received && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
#if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10));
#else
        sleep_ms(10);
#endif
    }

    return state->response_received ? state->ntp_time : nil_time;
}
