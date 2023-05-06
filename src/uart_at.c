#include "uart_at.h"
#include "ingsoc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "platform_api.h"
#include "bluetooth.h"
#include "btstack_defines.h"
#include "sm.h"
#include "gap.h"
#include "att_db.h"
#include "gatt_client.h"
#include "gatt_client_util.h"
#include "port_gen_os_driver.h"
#include "kv_storage.h"

#define INVALID_HANDLE              0xffff

#ifndef MAX_CONN_AS_SLAVE
#define MAX_CONN_AS_SLAVE           2
#endif

#ifndef MAX_CONN_AS_MASTER
#define MAX_CONN_AS_MASTER          8
#endif

#define TOTAL_CONN_NUM              (MAX_CONN_AS_MASTER + MAX_CONN_AS_SLAVE)

#if (TOTAL_CONN_NUM > 10) && (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
#error  ING916: 10 connection at most!
#endif

#define GEN_OS          ((const gen_os_driver_t *)platform_get_gen_os_driver())

typedef void (*f_cmd_handle_set)(int argc, const char *argv[]);
typedef void (*f_cmd_handle_get)(void);

#define MAX_ARG_C  10

enum
{
    KV_KEY_UART = KV_USER_KEY_START,
};

extern sm_persistent_t sm_persistent;

typedef struct notification_handler
{
    struct notification_handler *next;
    uint8_t registered;
    uint16_t value_handle;
    uint16_t desc_handle;
    uint16_t config;
    gatt_client_notification_t notification;
} notification_handler_t;

struct uart_settings
{
    uint32_t baud;
};

const struct uart_settings def_uart_settings =
{
    .baud = 115200,
};

struct write_char_info
{
    uint16_t value_handle;
    const uint8_t *data;
};

struct gatts_value_info
{
    uint16_t value_handle;
    const uint8_t *data;
};

typedef struct
{
    hci_con_handle_t handle;
    bd_addr_type_t peer_addr_type;
    bd_addr_t peer_addr;
    uint16_t min_interval, max_interval, cur_interval, latency, timeout;
    notification_handler_t *first_handler;
    struct gatt_client_discoverer *discoverer;

    struct write_char_info write_char_info;
    struct gatts_value_info gatts_value_info;
} conn_info_t;

// master role comes first; then slave role.
conn_info_t conn_infos[TOTAL_CONN_NUM] = {0};

// 26 is maximum number of connections supported by ING918.
uint8_t handle_2_id[26] = {0};

#define get_id_of_handle(handle)    (handle_2_id[handle])
#define get_handle_of_id(id)        (conn_infos[id].handle)

int initiating_index = -1;
int initiating_timeout = 30;
static uint8_t sec_auth_req = 0;

typedef struct
{
    const char *cmd;
    f_cmd_handle_set set;
    f_cmd_handle_get get;
} cmd_t;

struct
{
    const char *cmd;
    int argc;
    const char *argv[MAX_ARG_C];
} cmd_params = {0};

static char buffer[100] = {0};

extern sm_persistent_t sm_persistent;

void at_rx_data(const char *d, uint8_t len);
static void tx_data(const char *d, const uint16_t len);

void *at_alloc(size_t size)
{
    void *r = malloc(size);
    if (r == NULL) platform_raise_assertion("AT", __LINE__);
    return r;
}

void at_tx_ok(void)
{
    const static char ok[] = "OK\n";
    tx_data(ok, 4);
}

static void at_tx_error(void)
{
    const static char error[] = "ERROR\n";
    tx_data(error, 7);
}

void get_ble_init(void)
{
    tx_data("+BLEINIT:3\n", 11);
    at_tx_ok();
}

static void get_ble_addr(void)
{
    sprintf(buffer, "+BLEADDR:%d,%02X:%02X:%02X:%02X:%02X:%02X\n",
            sm_persistent.identity_addr_type,
            sm_persistent.identity_addr[0], sm_persistent.identity_addr[1], sm_persistent.identity_addr[2],
            sm_persistent.identity_addr[3], sm_persistent.identity_addr[4], sm_persistent.identity_addr[5]);
    tx_data(buffer, strlen(buffer) + 1);
    at_tx_ok();
}

static void update_addr(void *a, uint16_t b)
{
    gap_set_adv_set_random_addr(0, sm_persistent.identity_addr);
    gap_set_random_device_address(sm_persistent.identity_addr);
}

static int parse_addr(const char *str, uint8_t *adddress)
{
    unsigned int addr[6];
    int i = sscanf(str, "%2x:%2x:%2x:%2x:%2x:%2x", &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]);
    if (i != 6)
        return 1;

    for (i = 0; i < 6; i++) adddress[i] = (uint8_t)addr[i];
    return 0;
}

static void set_ble_addr(int argc, const char *argv[])
{
    if ((argc != 2) || (argv[0][0] != '1'))
        goto error;

    if (parse_addr(argv[1], sm_persistent.identity_addr) != 0)
        goto error;

    btstack_push_user_runnable(update_addr, NULL, 0);

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

static void set_uart(int argc, const char *argv[])
{
    if (argc < 1)
        goto error;

    struct uart_settings *p_uart = (struct uart_settings *)kv_get(KV_KEY_UART, NULL);
    uint32_t v = (uint32_t)atoi(argv[0]);

    if (v != p_uart->baud)
    {
        p_uart->baud = v;
        kv_commit(1);
        platform_reset();
    }
    else
        at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

struct
{
    uint8_t advertising;
    uint32_t adv_int_min, adv_int_max;
    uint8_t adv_type;
    uint8_t own_addr_type;
    uint8_t channel_map;
    uint8_t adv_filter_policy;
    uint8_t peer_addr_type;
    bd_addr_t peer_addr;
    int8_t tx_power;
} adv_param =
{
    .adv_int_min = 0x100,
    .adv_int_max = 0x100,
    .adv_type = CONNECTABLE_ADV_BIT | SCANNABLE_ADV_BIT | LEGACY_PDU_BIT,
    .own_addr_type = BD_ADDR_TYPE_LE_RANDOM,
    .channel_map = PRIMARY_ADV_ALL_CHANNELS,
    .adv_filter_policy = ADV_FILTER_ALLOW_ALL,
    .peer_addr_type = BD_ADDR_TYPE_LE_RANDOM,
    .tx_power = 3,
};

struct
{
    scan_type_t scan_type;
    bd_addr_type_t own_addr_type;
    scan_filter_policy_t filter_policy;
    uint16_t scan_interval;
    uint16_t scan_window;

    uint8_t scanning;
    uint16_t interval;
    uint8_t filter_type;
    bd_addr_t filter_addr;
} scan_param =
{
    .scan_type = SCAN_PASSIVE,
    .own_addr_type = BD_ADDR_TYPE_LE_RANDOM,
    .filter_policy = SCAN_ACCEPT_ALL_EXCEPT_NOT_DIRECTED,
    .scan_interval = 100,
    .scan_window = 50,
};

const static ext_adv_set_en_t adv_sets_en[] = { {.handle = 0, .duration = 0, .max_events = 0} };

static void get_ble_adv_param(void)
{
    sprintf(buffer, "+BLEADVPARAM:%u,%u,%u,%u,%u,%u,%u,%02X:%02X:%02X:%02X:%02X:%02X,%d\n",
            adv_param.adv_int_min, adv_param.adv_int_max,
            adv_param.adv_type,
            adv_param.own_addr_type,
            adv_param.channel_map,
            adv_param.adv_filter_policy,
            adv_param.peer_addr_type,
            adv_param.peer_addr[0], adv_param.peer_addr[1], adv_param.peer_addr[2],
            adv_param.peer_addr[3], adv_param.peer_addr[4], adv_param.peer_addr[5],
            adv_param.tx_power);
    tx_data(buffer, strlen(buffer) + 1);
    at_tx_ok();
}

static void set_ble_adv_param(int argc, const char *argv[])
{
    if (argc < 9) goto error;

    adv_param.adv_int_min   = (uint32_t)atoi(argv[0]);
    adv_param.adv_int_max   = (uint32_t)atoi(argv[1]);
    adv_param.adv_type      = (uint8_t)atoi(argv[2]);
    adv_param.own_addr_type = (uint8_t)atoi(argv[3]);
    adv_param.channel_map   = (uint8_t)atoi(argv[4]);
    adv_param.adv_filter_policy = (uint8_t)atoi(argv[5]);
    adv_param.peer_addr_type    = (uint8_t)atoi(argv[6]);
    if (parse_addr(argv[7], sm_persistent.identity_addr)) goto error;
    adv_param.tx_power          = (int8_t)atoi(argv[8]);

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

extern uint8_t g_adv_data[31];
extern uint8_t g_scan_data[31];
extern int g_adv_data_len;
extern int g_scan_data_len;

static char * append_hex_str(char *s, const uint8_t *data, int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        sprintf(s, "%02X", data[i]);
        s += 2;
    }
    return s;
}

static char *append_bd_addr(char *s, const uint8_t *addr)
{
    return s + sprintf(s, "%02X:%02X:%02X:%02X:%02X:%02X",
            addr[0], addr[1], addr[2],
            addr[3], addr[4], addr[5]);
}

void get_ble_adv_data(void)
{
    char *s = buffer;
    strcpy(s, "+BLEADVDATA:\"");
    s += strlen(s);
    s = append_hex_str(s, g_adv_data, g_adv_data_len);
    strcpy(s, "\"\n");
    tx_data(buffer, strlen(buffer) + 1);
}

uint8_t char_to_nibble(char c)
{
    if (('0' <= c) && (c <= '9'))
        return c - '0';
    else if (('a' <= c) && (c <= 'f'))
        return c - 'a' + 10;
    else if (('A' <= c) && (c <= 'F'))
        return c - 'A' + 10;
    else
        return 0;
}

static int load_hex_data(const char *s, uint8_t *data)
{
    int r = 0;
    while ((s[0] != 0) && (s[1] != 0))
    {
        data[r++] = (char_to_nibble(s[0]) << 4) | char_to_nibble(s[1]);
        s += 2;
    }
    return r;
}

static void set_ble_adv_data(int argc, const char *argv[])
{
    if (argc < 1) goto error;

    g_adv_data_len = load_hex_data(argv[0], g_adv_data);

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

void get_ble_scan_rsp_data(void)
{
    char *s = buffer;
    strcpy(s, "+BLESCANRSPDATA:\"");
    s += strlen(s);
    s = append_hex_str(s, g_scan_data, g_scan_data_len);
    strcpy(s, "\"\n");
    tx_data(buffer, strlen(buffer) + 1);
}

static void set_ble_scan_rsp_data(int argc, const char *argv[])
{
    if (argc < 1) goto error;

    g_scan_data_len = load_hex_data(argv[0], g_scan_data);

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

static void stack_set_ble_adv_start(void *a, uint16_t b)
{
    gap_set_ext_adv_para(adv_sets_en[0].handle,
                         adv_param.adv_type,
                         adv_param.adv_int_min, adv_param.adv_int_max,
                         adv_param.channel_map,
                         adv_param.own_addr_type,
                         adv_param.peer_addr_type,
                         adv_param.peer_addr,
                         adv_param.adv_filter_policy,
                         adv_param.tx_power,
                         PHY_1M,                    // Primary_Advertising_PHY
                         0,                         // Secondary_Advertising_Max_Skip
                         PHY_1M,                    // Secondary_Advertising_PHY
                         0x00,                      // Advertising_SID
                         0x00);                     // Scan_Request_Notification_Enable
    gap_set_ext_adv_data(adv_sets_en[0].handle, g_adv_data_len, g_adv_data);
    gap_set_ext_scan_response_data(adv_sets_en[0].handle, g_scan_data_len, g_scan_data);
    gap_set_ext_adv_enable(1, sizeof(adv_sets_en) / sizeof(adv_sets_en[0]), adv_sets_en);
}

static void stack_set_ble_adv_stop(void *a, uint16_t b)
{
    gap_set_ext_adv_enable(0, sizeof(adv_sets_en) / sizeof(adv_sets_en[0]), adv_sets_en);
}

static void get_ble_adv_start(void)
{
    adv_param.advertising = 1;
    btstack_push_user_runnable(stack_set_ble_adv_start, NULL, 0);
    at_tx_ok();
}

static void get_ble_adv_stop(void)
{
    adv_param.advertising = 0;
    btstack_push_user_runnable(stack_set_ble_adv_stop, NULL, 0);
    at_tx_ok();
}

void at_on_adv_set_terminated(void)
{
    if (adv_param.advertising)
        gap_set_ext_adv_enable(1, sizeof(adv_sets_en) / sizeof(adv_sets_en[0]), adv_sets_en);
}

static void stack_scan(void *a, uint16_t b)
{
    if (b)
    {
        scan_phy_config_t configs[2] =
        {
            {
                .phy = PHY_1M,
                .type = scan_param.scan_type,
                .interval = scan_param.scan_interval,
                .window = scan_param.scan_window,
            },
            {
                .phy = PHY_CODED,
                .type = scan_param.scan_type,
                .interval = scan_param.scan_interval,
                .window = scan_param.scan_window,
            }
        };
        gap_set_ext_scan_para(BD_ADDR_TYPE_LE_RANDOM, scan_param.filter_policy,
                              sizeof(configs) / sizeof(configs[0]), configs);
        gap_set_ext_scan_enable(1, 0, scan_param.interval * 100, 0);
    }
    else
    {
        gap_set_ext_scan_enable(0, 0, 0, 0);
    }
}

static void get_ble_scan_param(void)
{
    sprintf(buffer, "+BLESCANPARAM:%d,%d,%d,%d,%d\n",
            scan_param.scan_type,
            scan_param.own_addr_type,
            scan_param.filter_policy,
            scan_param.scan_interval,
            scan_param.scan_window);
    tx_data(buffer, strlen(buffer) + 1);
    at_tx_ok();
}

static void set_ble_scan_param(int argc, const char *argv[])
{
    if (argc < 5) goto error;

    scan_param.scan_type        = (scan_type_t)atoi(argv[0]);
    scan_param.own_addr_type    = (bd_addr_type_t)atoi(argv[1]);
    scan_param.filter_policy    = (scan_filter_policy_t)atoi(argv[2]);
    scan_param.scan_interval    = (uint16_t)atoi(argv[3]);
    scan_param.scan_interval    = (uint16_t)atoi(argv[4]);

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

void at_on_adv_report(const le_ext_adv_report_t *report)
{
    bd_addr_t addr;
    reverse_bd_addr(report->address, addr);
    if (scan_param.filter_type == 1)
    {
        if (memcmp(addr, scan_param.filter_addr, BD_ADDR_LEN))
            return;
    }

    strcpy(buffer, "+BLESCAN:");
    char *s = buffer + 9;
    s = append_bd_addr(s, addr);

    if (report->evt_type & HCI_EXT_ADV_PROP_SCAN_RSP)
    {
        s += sprintf(s, ",%d,,", report->rssi);
        s = append_hex_str(s, report->data, report->data_len);
        s += sprintf(s, ",%d", report->addr_type);
    }
    else
    {
        s += sprintf(s, ",%d,", report->rssi);
        s = append_hex_str(s, report->data, report->data_len);
        s += sprintf(s, ",,%d", report->addr_type);
    }

    tx_data(buffer, s - buffer + 1);
}

static void set_ble_scan(int argc, const char *argv[])
{
    if (argc < 1) goto error;
    uint8_t enable = atoi(argv[0]);

    if (argc >= 2)
        scan_param.interval        = atoi(argv[1]);
    if (argc >= 3)
    {
        scan_param.filter_type    = (bd_addr_type_t)atoi(argv[2]);
        if (scan_param.filter_type > 1)
            goto error;
    }
    if (argc >= 4)
        parse_addr(argv[3], scan_param.filter_addr);

    btstack_push_user_runnable(stack_scan, NULL, enable);

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

static void stack_gatts_read(void *user_data, uint16_t value_len)
{
    conn_info_t *p = (conn_info_t *)user_data;
    uint8_t r = att_server_deferred_read_response(p->handle,
                        p->gatts_value_info.value_handle,
                        p->gatts_value_info.data, value_len);
    if (0 == r) at_tx_ok(); else at_tx_error();
}

static void stack_gatts_notify(void *user_data, uint16_t value_len)
{
    conn_info_t *p = (conn_info_t *)user_data;
    uint8_t r = att_server_notify(p->handle,
                        p->gatts_value_info.value_handle,
                        p->gatts_value_info.data, value_len);
    if (0 == r) at_tx_ok(); else at_tx_error();
}

static void stack_gatts_indicate(void *user_data, uint16_t value_len)
{
    conn_info_t *p = (conn_info_t *)user_data;
    uint8_t r = att_server_indicate(p->handle,
                        p->gatts_value_info.value_handle,
                        p->gatts_value_info.data, value_len);
    if (0 == r) at_tx_ok(); else at_tx_error();
}

static void set_ble_gatts_read(int argc, const char *argv[])
{
    if (argc != 3) goto error;

    uint8_t id = (uint8_t)atoi(argv[0]);
    conn_info_t *p = &conn_infos[id];
    if (p->handle == INVALID_HANDLE) goto error;

    p->gatts_value_info.value_handle = (uint16_t)atoi(argv[1]);

    uint16_t len = (uint16_t)load_hex_data(argv[2], (uint8_t *)argv[2]);
    p->gatts_value_info.data = (uint8_t *)argv[2];

    btstack_push_user_runnable(stack_gatts_read, p, len);
    return;

error:
    at_tx_error();
    return;
}

static void set_ble_gatts_write(int argc, const char *argv[])
{
    if (argc != 4) goto error;

    uint8_t id = (uint8_t)atoi(argv[0]);
    conn_info_t *p = &conn_infos[id];
    if (p->handle == INVALID_HANDLE) goto error;

    p->gatts_value_info.value_handle = (uint16_t)atoi(argv[1]);
    uint8_t mode = (uint8_t)atoi(argv[2]);
    uint16_t len = (uint16_t)load_hex_data(argv[3], (uint8_t *)argv[3]);
    p->gatts_value_info.data = (uint8_t *)argv[3];

    btstack_push_user_runnable(mode == 0 ? stack_gatts_notify : stack_gatts_indicate, p, len);
    return;

error:
    at_tx_error();
    return;
}

static void get_ble_conn(void)
{
    int i;
    for (i = 0; i < TOTAL_CONN_NUM; i++)
    {
        conn_info_t *p = conn_infos + i;
        if (p->handle == INVALID_HANDLE) continue;
        char *s = buffer + sprintf(buffer, "+BLECONN:%d,", i);
        s = append_bd_addr(s, p->peer_addr);
        strcpy(s, "\n");
        s++;
        tx_data(buffer, s - buffer + 1);
    }
    at_tx_ok();
}

static void stack_cancel_create_conn(void *data, uint16_t index)
{
    gap_create_connection_cancel();
}

static void initiate_timeout(void)
{
    btstack_push_user_runnable(stack_cancel_create_conn, NULL, 0);
}

static void stack_initiate(void *data, uint16_t index)
{
    conn_info_t *p = conn_infos + index;
    initiating_phy_config_t phy_configs[] =
    {
        {
            .phy = PHY_1M,
            .conn_param =
            {
                .scan_int = 150,
                .scan_win = 100,
                .interval_min = p->min_interval,
                .interval_max = p->max_interval,
                .latency = p->latency,
                .supervision_timeout = p->timeout,
                .min_ce_len = 7,
                .max_ce_len = 7
            }
        }
    };

    if (gap_ext_create_connection(
                INITIATING_ADVERTISER_FROM_PARAM,
                BD_ADDR_TYPE_LE_RANDOM,
                p->peer_addr_type,
                p->peer_addr,
                sizeof(phy_configs) / sizeof(phy_configs[0]),
                phy_configs) == 0)
    {
        initiating_index = index;
        platform_set_timer(initiate_timeout, initiating_timeout * 1600);
    }
}

static void set_ble_conn(int argc, const char *argv[])
{
    if (initiating_index >= 0) goto error;

    if (argc < 2) goto error;

    int id = atoi(argv[0]);
    if (id > MAX_CONN_AS_MASTER) goto error;
    conn_info_t *p = conn_infos + id;
    parse_addr(argv[1], p->peer_addr);

    if (argc >= 3)
        p->peer_addr_type = (bd_addr_type_t)atoi(argv[2]);
    if (argc >= 4)
        initiating_timeout = atoi(argv[3]);

    btstack_push_user_runnable(stack_initiate, NULL, (uint16_t)id);

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

static void stack_disconn(void *data, uint16_t index)
{
    gap_disconnect(index);
}

static void set_ble_disconn(int argc, const char *argv[])
{
    if (argc < 1) goto error;

    int id = atoi(argv[0]);
    conn_info_t *p = conn_infos + id;
    if (p->handle == INVALID_HANDLE) goto error;

    btstack_push_user_runnable(stack_disconn, NULL, get_handle_of_id(id));

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

conn_info_t *get_conn_by_addr(bd_addr_type_t type, const uint8_t *addr)
{
    int i;
    for (i = 0; i < MAX_CONN_AS_MASTER; i++)
    {
        if ((conn_infos[i].peer_addr_type == type)
            && (memcmp(conn_infos[i].peer_addr, addr, sizeof(conn_infos[i].peer_addr)) == 0))
            return conn_infos + i;
    }
    return NULL;
}

static void get_ble_conn_param(void)
{
    int i;
    for (i = 0; i < TOTAL_CONN_NUM; i++)
    {
        conn_info_t *p = conn_infos + i;
        if (p->handle == INVALID_HANDLE) continue;
        int len = sprintf(buffer, "+BLECONNPARAM:%d,%d,%d,%d,%d,%d",
            i, p->min_interval, p->max_interval, p->cur_interval,
            p->latency, p->timeout);
        tx_data(buffer, len + 1);
    }
}

static void stack_update_conn(void *data, uint16_t index)
{
    conn_info_t *p = conn_infos + index;
    gap_update_connection_parameters(p->handle,
        p->min_interval, p->max_interval, p->latency, p->timeout, 7, 7);
}

static void set_ble_conn_param(int argc, const char *argv[])
{
    if (argc < 5) goto error;

    int id = atoi(argv[0]);
    conn_info_t *p = conn_infos + id;
    if (p->handle == INVALID_HANDLE) goto error;

    p->min_interval = (uint16_t)atoi(argv[1]);
    p->max_interval = (uint16_t)atoi(argv[2]);
    p->latency      = (uint16_t)atoi(argv[3]);
    p->timeout      = (uint16_t)atoi(argv[4]);

    btstack_push_user_runnable(stack_update_conn, NULL, get_handle_of_id(id));

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

static int print_uuid(char *s, const uint8_t *uuid)
{
    if (uuid_has_bluetooth_prefix(uuid))
    {
        return sprintf(s, "%04x", (uuid[2] << 8) | uuid[3]);
    }
    else
        return sprintf(s, "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
                        uuid[0], uuid[1], uuid[2], uuid[3],
                        uuid[4], uuid[5], uuid[6], uuid[7], uuid[8], uuid[9],
                        uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
}

static void gatt_client_dump_profile(service_node_t *first, void *user_data, int err_code)
{
    service_node_t *s = first;
    conn_info_t *p = (conn_info_t *)user_data;
    int id = p - conn_infos;

    while (s)
    {
        char_node_t *c = s->chars;
        char *str = buffer + sprintf(buffer, "+BLEGATTCPRIMSRV:%d,%d,%d,",
            id, s->service.start_group_handle, s->service.end_group_handle);
        str += print_uuid(str, s->service.uuid128);
        strcpy(str, "\n");
        tx_data(buffer, str - buffer + 2);

        while (c)
        {
            str = buffer + sprintf(buffer, "+BLEGATTCCHAR:%d,%d,%d,%d,%d,",
                id, c->chara.start_handle, c->chara.end_handle, c->chara.value_handle,
                c->chara.properties);
            str += print_uuid(str, c->chara.uuid128);
            strcpy(str, "\n");
            tx_data(buffer, str - buffer + 2);

            desc_node_t *d = c->descs;
            while (d)
            {
                str = buffer + sprintf(buffer, "+BLEGATTCDESC:%d,%d,",
                    id, d->desc.handle);
                str += print_uuid(str, d->desc.uuid128);
                strcpy(str, "\n");
                tx_data(buffer, str - buffer + 2);

                d = d->next;
            }
            c = c->next;
        }
        s = s->next;
    }

    int len = sprintf(buffer, "+BLEGATTCC:%d,%d\n", id, err_code);
    tx_data(buffer, len + 1);
    gatt_client_util_free(p->discoverer);
    p->discoverer = NULL;
}

static void stack_discover_all(void *p, uint16_t value)
{
    conn_info_t *conn = (conn_info_t *)p;
    conn->discoverer = gatt_client_util_discover_all(value, gatt_client_dump_profile, p);
}

static void set_ble_gattc(int argc, const char *argv[])
{
    if (argc < 1) goto error;

    int id = atoi(argv[0]);
    conn_info_t *p = conn_infos + id;
    if (p->handle == INVALID_HANDLE) goto error;

    if (p->discoverer) goto error;

    btstack_push_user_runnable(stack_discover_all, p, p->handle);

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

void read_characteristic_value_callback(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    switch (packet[0])
    {
    case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
        {
            uint16_t value_size;
            const gatt_event_value_packet_t *value =
                gatt_event_characteristic_value_query_result_parse(packet, size, &value_size);

            char *s = buffer + sprintf(buffer, "+BLEGATTCRD:%d,%d,0,", get_id_of_handle(channel), value->handle);
            s = append_hex_str(s, value->value, value_size);
            strcpy(s, "\n");
            tx_data(buffer, s - buffer + 2);
        }
        break;
    case GATT_EVENT_QUERY_COMPLETE:
        {
            const gatt_event_query_complete_t *complete = gatt_event_query_complete_parse(packet);
            if (complete->status != 0)
            {
                int len = sprintf(buffer, "+BLEGATTCRD:%d,%d,%d\n", get_id_of_handle(channel), complete->handle, complete->status);
                tx_data(buffer, len + 1);
            }
            else;
        }
        break;
    }
}

static void stack_read_char(void *p, uint16_t value_handle)
{
    uint8_t r = gatt_client_read_value_of_characteristic_using_value_handle(
                read_characteristic_value_callback,
                (uint16_t)(uintptr_t)p,
                value_handle);
    if (0 == r)
        at_tx_ok();
    else
        at_tx_error();
}

static void set_ble_gattc_read(int argc, const char *argv[])
{
    if (argc < 2) goto error;

    int id = atoi(argv[0]);
    uint16_t handle = (uint16_t)atoi(argv[1]);
    conn_info_t *p = conn_infos + id;
    if (p->handle == INVALID_HANDLE) goto error;

    btstack_push_user_runnable(stack_read_char, (void *)(uintptr_t)p->handle, handle);
    return;

error:
    at_tx_error();
    return;
}

void write_characteristic_value_callback(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    switch (packet[0])
    {
    case GATT_EVENT_QUERY_COMPLETE:
        {
            const gatt_event_query_complete_t *complete = gatt_event_query_complete_parse(packet);
            int len = sprintf(buffer, "+BLEGATTCWR:%d,%d,%d\n", get_id_of_handle(channel), complete->handle, complete->status);
            tx_data(buffer, len + 1);
        }
        break;
    }
}

static void stack_write_char(void *user_data, uint16_t value_len)
{
    conn_info_t *p = (conn_info_t *)user_data;

    uint8_t r = gatt_client_write_value_of_characteristic(
                write_characteristic_value_callback,
                p->handle,
                p->write_char_info.value_handle,
                value_len,
                p->write_char_info.data);
    if (0 == r)
        at_tx_ok();
    else
        at_tx_error();
}

static void set_ble_gattc_write(int argc, const char *argv[])
{
    if (argc < 3) goto error;

    uint8_t id = (uint8_t)atoi(argv[0]);

    conn_info_t *p = conn_infos + id;
    if (p->handle == INVALID_HANDLE) goto error;

    p->write_char_info.value_handle = (uint16_t)atoi(argv[1]);
    p->write_char_info.data = (uint8_t *)argv[2];
    uint16_t len = (uint16_t)load_hex_data(argv[2], (uint8_t *)argv[2]);

    btstack_push_user_runnable(stack_write_char, p, len);

    return;

error:
    at_tx_error();
    return;
}

static void output_notification_handler(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    const gatt_event_value_packet_t *value;
    uint16_t value_size = 0;
    int len = 0;
    switch (packet[0])
    {
    case GATT_EVENT_NOTIFICATION:
        value = gatt_event_notification_parse(packet, size, &value_size);
        strcpy(buffer, "+BLEGATTCNOTI");
        len = 13;
        break;
    case GATT_EVENT_INDICATION:
        value = gatt_event_indication_parse(packet, size, &value_size);
        strcpy(buffer, "+BLEGATTCIND");
        len = 12;
        break;
    }
    if (value_size < 1) return;
    char *s = buffer + len;
    s += sprintf(s, ":%d,%d,", get_id_of_handle(channel), value->handle);
    s = append_hex_str(s, value->value, value_size);
    strcpy(s, "\n");
    tx_data(buffer, s - buffer + 2);
}

static void write_characteristic_descriptor_callback(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    switch (packet[0])
    {
    case GATT_EVENT_QUERY_COMPLETE:
        {
            const gatt_event_query_complete_t *complete = gatt_event_query_complete_parse(packet);
            int len = sprintf(buffer, "+BLEGATTCSUB:%d,%d,%d\n", get_id_of_handle(channel), complete->handle, complete->status);
            tx_data(buffer, len + 1);
        }
        break;
    }
}

static void stack_sub_char(void *user_data, uint16_t value_handle)
{
    conn_info_t *p = (conn_info_t *)user_data;

    notification_handler_t *first = p->first_handler;

    while (first)
    {
        if (first->value_handle == value_handle) break;
        first = first->next;
    }

    if (NULL == first) return;

    if (first->registered == 0)
    {
        gatt_client_listen_for_characteristic_value_updates(
            &first->notification, output_notification_handler,
            p->handle, first->value_handle);
        first->registered = 1;
    }

    gatt_client_write_characteristic_descriptor_using_descriptor_handle(
        write_characteristic_descriptor_callback,
        p->handle,
        first->desc_handle,
        sizeof(first->config),
        (uint8_t *)&first->config);
}

static void set_ble_gattc_sub(int argc, const char *argv[])
{
    if (argc < 3) goto error;

    uint8_t id = (uint8_t)atoi(argv[0]);
    conn_info_t *p = conn_infos + id;
    if (p->handle == INVALID_HANDLE) goto error;

    uint16_t value_handle = (uint16_t)atoi(argv[1]);

    notification_handler_t *first = p->first_handler;

    while (first)
    {
        if (first->value_handle == value_handle) break;
        first = first->next;
    }

    if (NULL == first)
    {
        first = (notification_handler_t *)at_alloc(sizeof(notification_handler_t));
        first->next = p->first_handler;
        p->first_handler = first;

        first->registered = 0;
        first->value_handle = value_handle;
        if (argc < 4)
            first->desc_handle = value_handle + 1;
    }

    if (argc >= 4)
        first->desc_handle = (uint16_t)atoi(argv[3]);

    first->config = (uint16_t)atoi(argv[2]);

    btstack_push_user_runnable(stack_sub_char, p, value_handle);

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

static void stack_set_sec_param(void *user_data, uint16_t value)
{
    uintptr_t v = (uintptr_t)user_data;
    sm_config(v & 0xff, (v >> 8) & 0xff, 0, &sm_persistent);
    sm_set_authentication_requirements(sec_auth_req);
}

static void set_ble_sec_param(int argc, const char *argv[])
{
    if (argc < 3) goto error;

    uint32_t enable = (uint8_t)atoi(argv[0]);
    sec_auth_req = (uint8_t)atoi(argv[1]);
    uint32_t io_cap = (uint8_t)atoi(argv[2]);

    uintptr_t v = (io_cap << 8) | enable;

    btstack_push_user_runnable(stack_set_sec_param, (void *)v, 0);

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

static void reset_all(void)
{
    kv_remove_all();
    kv_commit(1);
    platform_reset();
}

static void set_power_saving(int argc, const char *argv[])
{
    if (argc < 1) goto error;

    platform_config(PLATFORM_CFG_POWER_SAVING, (uint8_t)atoi(argv[0]));

    at_tx_ok();
    return;

error:
    at_tx_error();
    return;
}

extern void config_wakeup_and_shutdown(void);

const static cmd_t cmds[] =
{
    {
        // AT+RESET
        .cmd = "+RESET",
        .get = reset_all
    },
    {
        // AT+SHUTDOWN
        .cmd = "+SHUTDOWN",
        .get = config_wakeup_and_shutdown,
    },
    {
        // AT+POWERSAVING=<enable>
        .cmd = "+POWERSAVING",
        .set = set_power_saving,
    },
    {
        // AT+UART=<baud>
        .cmd = "+UART",
        .set = set_uart,
    },
    {
        // AT+BLEINIT?
        .cmd = "+BLEINIT",
        .get = get_ble_init
    },
    {
        // AT+BLEADDR=<addr_type>,<random_addr>
        .cmd = "+BLEADDR",
        .get = get_ble_addr,
        .set = set_ble_addr,
    },
    {
        // +BLEADVPARAM:<adv_int_min>,<adv_int_max>,<adv_type>,<own_addr_type>,<channel_map>,<adv_filter_policy>,<peer_addr_type>,<peer_addr>,<tx_power>
        .cmd = "+BLEADVPARAM",
        .get = get_ble_adv_param,
        .set = set_ble_adv_param,
    },
    {
        // AT+BLEADVDATA=<adv_data>
        // AT+BLEADVDATA="1122334455"
        .cmd = "+BLEADVDATA",
        .get = get_ble_adv_data,
        .set = set_ble_adv_data,
    },
    {
        // AT+BLESCANRSPDATA=<scan_rsp_data>
        .cmd = "+BLESCANRSPDATA",
        .get = get_ble_scan_rsp_data,
        .set = set_ble_scan_rsp_data,
    },
    {
        // AT+BLEADVSTART
        .cmd = "+BLEADVSTART",
        .get = get_ble_adv_start,
    },
    {
        // AT+BLEADVSTOP
        .cmd = "+BLEADVSTOP",
        .get = get_ble_adv_stop,
    },
    {
        // +BLESCANPARAM:<scan_type>,<own_addr_type>,<filter_policy>,<scan_interval>,<scan_window>
        .cmd = "+BLESCANPARAM",
        .get = get_ble_scan_param,
        .set = set_ble_scan_param,
    },
    {
        // AT+BLESCAN=<enable>[[,<interval>],<filter_type>,<filter_param>]
        .cmd = "+BLESCAN",
        .set = set_ble_scan,
    },
    {
        // AT+BLECONN=<conn_index>,<remote_address>,<addr_type>[,<timeout>]
        .cmd = "+BLECONN",
        .get = get_ble_conn,
        .set = set_ble_conn,
    },
    {
        // +BLECONNPARAM:<conn_index>,<min_interval>,<max_interval>,<interval>,<latency>,<timeout>
        .cmd = "+BLECONNPARAM",
        .get = get_ble_conn_param,
        .set = set_ble_conn_param,
    },
    {
        // AT+BLEDISCONN=<conn_index>
        .cmd = "+BLEDISCONN",
        .set = set_ble_disconn,
    },
    {
        // AT+BLEGATTC=<conn_index>
        .cmd = "+BLEGATTC",
        .set = set_ble_gattc,
    },
    {
        // AT+BLEGATTCRD=<conn_index>,<handle>
        .cmd = "+BLEGATTCRD",
        .set = set_ble_gattc_read,
    },
    {
        // AT+BLEGATTCWR=<conn_index>,<handle>,<value>
        .cmd = "+BLEGATTCWR",
        .set = set_ble_gattc_write,
    },
    {
        // AT+BLEGATTCSUB=<conn_index>,<handle>,<config>[,<desc_handle>]
        .cmd = "+BLEGATTCSUB",
        .set = set_ble_gattc_sub,
    },
    {
        // +BLEGATTSRD=<conn_index>,<att_handle>,<hex_data>
        .cmd = "+BLEGATTSRD",
        .set = set_ble_gatts_read,
    },
    {
        // +BLEGATTSWR=<conn_index>,<att_handle>,<mode>,<hex_data>
        .cmd = "+BLEGATTSWR",
        .set = set_ble_gatts_write,
    },
    {
        // +BLESECPARAM:<enable>,<auth_req>,<io_cap>
        .cmd = "+BLESECPARAM",
        .set = set_ble_sec_param,
    },
};

static void handle_command(char *cmd_line)
{
    static const char unknow_cmd[] =  "ERROR: UNKNOWN\n";
    char *param = cmd_line;
    int i;
    uint8_t is_query = 1;

    if ((param[0] != 'A') || (param[1] != 'T'))
        goto show_help;

    param += 2;
    cmd_params.cmd = param;
    cmd_params.argc = 0;

    if (param[0] == '\0')
    {
        at_tx_ok();
        return;
    }

    while (*param)
    {
        if (*param == '=')
        {
            is_query = 0;
            break;
        }
        else if (*param == '?')
            break;
        else
            param++;
    }

    *param++ = '\0';

    if (is_query == 0)
    {
        while (*param)
        {
            uint8_t is_quote = param[0] == '"';
            if (is_quote) param++;
            cmd_params.argv[cmd_params.argc++] = param;

            if (is_quote)
            {
                while (*param && (*param != '"')) param++;
                if (*param) *param++ = '\0';
            }

            while (*param && (*param != ',')) param++;
            if (*param) *param++ = '\0';
        }
    }

    for (i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
    {
        if (strcasecmp(cmds[i].cmd, cmd_params.cmd) == 0)
            break;
    }

    if (i >= sizeof(cmds) / sizeof(cmds[0]))
        goto show_help;

    if (is_query)
    {
        if (cmds[i].get == NULL)
            goto show_help;

        cmds[i].get();
    }
    else
    {
        if (cmds[i].set == NULL)
            goto show_help;
        cmds[i].set(cmd_params.argc, cmd_params.argv);
    }

    return;

show_help:
    tx_data(unknow_cmd, strlen(unknow_cmd) + 1);
}

typedef struct
{
    uint8_t busy;
    uint16_t size;
    char buf[256];
} str_buf_t;

str_buf_t input = {0};
str_buf_t output = {0};

static void append_data(str_buf_t *buf, const char *d, const uint16_t len)
{
    if (buf->size + len > sizeof(buf->buf))
        buf->size = 0;

    if (buf->size + len <= sizeof(buf->buf))
    {
        memcpy(buf->buf + buf->size, d, len);
        buf->size += len;
    }
}

static gen_handle_t cmd_event = NULL;

static void at_task_entry(void *_)
{
    while (1)
    {
        GEN_OS->event_wait(cmd_event);

        handle_command(input.buf);
        input.size = 0;
        input.busy = 0;
    }
}

void uart_at_start(void)
{
    extern void update_baud(uint32_t baud);

    cmd_event = GEN_OS->event_create();
    GEN_OS->task_create("AT",
        at_task_entry,
        NULL,
        1024,
        GEN_TASK_PRIORITY_LOW);

    ll_set_max_conn_number(TOTAL_CONN_NUM);

    int i;
    for (i = 0; i < TOTAL_CONN_NUM; i++)
    {
        conn_info_t *p = conn_infos + i;
        p->handle = INVALID_HANDLE;
        p->peer_addr_type = BD_ADDR_TYPE_LE_RANDOM;
        p->min_interval = 350;
        p->max_interval = 350;
        p->timeout = 800;
    }

    const struct uart_settings *p_uart = (const struct uart_settings *)kv_get(KV_KEY_UART, NULL);
    if (p_uart == NULL)
    {
        kv_put(KV_KEY_UART, (const uint8_t *)&def_uart_settings, sizeof(def_uart_settings));
        p_uart = (const struct uart_settings *)kv_get(KV_KEY_UART, NULL);
    }

    update_baud(p_uart->baud);

    at_tx_ok();
}

void at_rx_data(const char *d, uint8_t len)
{
    if (input.busy)
    {
        return;
    }

    if (0 == input.size)
    {
        while ((len > 0) && ((*d == '\r') || (*d == '\n')))
        {
            d++;
            len--;
        }
    }
    if (len == 0) return;

    append_data(&input, d, len);

    if ((input.size > 0) &&
        ((input.buf[input.size - 1] == '\r') || (input.buf[input.size - 1] == '\n')))
    {
        int16_t t = input.size - 2;
        while ((t > 0) && ((input.buf[t] == '\r') || (input.buf[t] == '\n'))) t--;
        input.buf[t + 1] = '\0';
        input.busy = 1;
        GEN_OS->event_set(cmd_event);
    }
}

extern void stack_notify_tx_data(void);

static void tx_data(const char *d, const uint16_t len)
{
    GEN_OS->enter_critical();

    if ((output.size == 0) && (d[len - 1] == '\0'))
    {
        puts(d);
        goto exit;
    }

    append_data(&output, d, len);

    if ((output.size > 0) && (output.buf[output.size - 1] == '\0'))
    {
        puts(output.buf);
        output.size = 0;
    }

exit:
    GEN_OS->leave_critical();
}

uint8_t *at_clear_tx_data(uint16_t *len)
{
    *len = output.size;
    output.size = 0;
    return (uint8_t *)output.buf;
}

int at_att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode,
                              uint16_t offset, const uint8_t *att_buffer, uint16_t buffer_size)
{
    char *s = buffer + sprintf(buffer, "+BLEGATTSWR:%d,%d,\"", get_id_of_handle(connection_handle), att_handle);
    s = append_hex_str(s, att_buffer, buffer_size);
    s = s + sprintf(s, "\"\n");
    tx_data(buffer, s - buffer + 1);
    return 0;
}

uint16_t at_att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset,
                                  uint8_t * att_buffer, uint16_t buffer_size)
{
    sprintf(buffer, "+BLEGATTSRD:%d,%d\n", get_id_of_handle(connection_handle), att_handle);
    return ATT_DEFERRED_READ;
}

static void report_connected(uint8_t id)
{
    char *s  = buffer + sprintf(buffer, "+BLECONN:%d,", id);
    s = append_bd_addr(s, conn_infos[id].peer_addr);
    strcpy(s, "\n");
    s++;
    tx_data(buffer, s - buffer + 1);
}

void at_on_connection_complete(const le_meta_event_enh_create_conn_complete_t *complete)
{
    conn_info_t *p = NULL;
    if (complete->role == HCI_ROLE_SLAVE)
    {
        if (complete->status != 0) return;

        int i;
        for (i = MAX_CONN_AS_MASTER; i < TOTAL_CONN_NUM; i++)
        {
            if (conn_infos[i].handle == INVALID_HANDLE)
            {
                p = conn_infos + i;
                break;
            }
        }
        if (p)
        {
            p->handle = complete->handle;
            handle_2_id[complete->handle] = p - conn_infos;

            p->peer_addr_type = complete->peer_addr_type;
            reverse_bd_addr(complete->peer_addr, p->peer_addr);
        }
        else
        {
            // too many connections
            gap_disconnect(complete->handle);
        }
    }
    else
    {
        platform_set_timer(initiate_timeout, 0);

        if (complete->status == 0)
        {
            bd_addr_t rev;
            reverse_bd_addr(complete->peer_addr, rev);
            p = get_conn_by_addr(complete->peer_addr_type, rev);
            if (p)
            {
                p->handle = complete->handle;
                handle_2_id[complete->handle] = p - conn_infos;

                if (sec_auth_req & SM_AUTHREQ_BONDING)
                    sm_request_pairing(complete->handle);
            }
            else
                gap_disconnect(complete->handle);
        }
        else
        {
            int len = sprintf(buffer, "+BLECONN:%d,-1\n", initiating_index);
            tx_data(buffer, len + 1);
        }
        initiating_index = -1;
    }

    if (p)
    {
        p->cur_interval = complete->interval;
        p->latency = complete->latency;
        p->timeout = complete->sup_timeout;

        report_connected(get_id_of_handle(complete->handle));
    }
}

void at_on_disconnect(const event_disconn_complete_t *complete)
{
    int id = get_id_of_handle(complete->conn_handle);
    int len = sprintf(buffer, "+BLEDISCONN:%d,%d\n", id, complete->status);
    tx_data(buffer, len + 1);
    conn_infos[id].handle = INVALID_HANDLE;
    get_id_of_handle(complete->conn_handle) = 0;

    conn_info_t *p = conn_infos + id;
    notification_handler_t *first = p->first_handler;
    while (first)
    {
        p->first_handler = first->next;
        free(first);
        first = p->first_handler;
    }
}

void at_on_sm_state_changed(uint8_t reason)
{
    switch (reason)
    {
    case SM_STARTED:
        break;
    case SM_FINAL_PAIRED:
        tx_data("+SEC:PAIRED\n", 13);
        break;
    case SM_FINAL_REESTABLISHED:
        tx_data("+SEC:RESUMED\n", 14);
        break;
    default:
        tx_data("+SEC:FAILED\n", 13);
        break;
    }
}
