#include <stdio.h>
#include "platform_api.h"
#include "att_db.h"
#include "gap.h"
#include "btstack_event.h"
#include "btstack_defines.h"
#include "ota_service.h"
#include "sm.h"
#include "uart_at.h"

sm_persistent_t sm_persistent =
{
    .er = { 0xD0, 0x62, 0x8B, 0x28, 0x49, 0x6E, 0x38, 0x63,
            0xDC, 0xCE, 0xDA, 0xCB, 0x35, 0x52, 0x8E, 0x42 },
    .ir = { 0xE6, 0xB9, 0xC7, 0x92, 0x43, 0x1D, 0x27, 0x73,
            0x5D, 0xD7, 0xB3, 0xD3, 0x13, 0xB8, 0x38, 0x03 },
    .identity_addr_type = BD_ADDR_TYPE_LE_RANDOM,
    .identity_addr      = { 0xFF, 0x4E, 0x40, 0x61, 0x5A, 0xCA }
};

#define SECURITY_PERSISTENT_DATA    (&sm_persistent)
#define PRIVATE_ADDR_MODE           GAP_RANDOM_ADDRESS_OFF

// GATT characteristic handles
#include "../data/gatt.const"

uint8_t g_adv_data[31] = {
    #include "../data/advertising.adv"
};

#include "../data/advertising.const"

uint8_t g_scan_data[31] = {
    #include "../data/scan_response.adv"
};

int g_adv_data_len = 0;
int g_scan_data_len = 0;

#include "../data/scan_response.const"

const static uint8_t profile_data[] = {
    #include "../data/gatt.profile"
};

#include "../data/gatt.const"

extern void at_on_adv_set_terminated(void);
extern void at_on_adv_report(const le_ext_adv_report_t *report);
extern int at_att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode,
                              uint16_t offset, const uint8_t *buffer, uint16_t buffer_size);
extern uint16_t at_att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset,
                                  uint8_t * buffer, uint16_t buffer_size);
extern void at_on_connection_complete(const le_meta_event_enh_create_conn_complete_t *complete);
extern void at_on_disconnect(const event_disconn_complete_t *complete);
extern void at_on_sm_state_changed(uint8_t reason);

static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset,
                                  uint8_t * buffer, uint16_t buffer_size)
{
    switch (att_handle)
    {
    case HANDLE_FOTA_VERSION:
    case HANDLE_FOTA_DATA:
    case HANDLE_FOTA_CONTROL:
        return ota_read_callback(att_handle, offset, buffer, buffer_size);
    default:

        return at_att_read_callback(connection_handle, att_handle, offset, buffer, buffer_size);
    }
}

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode,
                              uint16_t offset, const uint8_t *buffer, uint16_t buffer_size)
{
    switch (att_handle)
    {
    case HANDLE_FOTA_DATA:
    case HANDLE_FOTA_CONTROL:
        return ota_write_callback(att_handle, transaction_mode, offset, buffer, buffer_size);
    default:
        return at_att_write_callback(connection_handle, att_handle, transaction_mode, offset, buffer, buffer_size);
    }
}

static void user_msg_handler(uint32_t msg_id, void *data, uint16_t size)
{
    switch (msg_id)
    {
        // add your code
    //case MY_MESSAGE_ID:
    //    break;
    default:
        ;
    }
}

static void user_packet_handler(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    uint8_t event = hci_event_packet_get_type(packet);
    const btstack_user_msg_t *p_user_msg;
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (event)
    {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING)
            break;
        gap_set_adv_set_random_addr(0, SECURITY_PERSISTENT_DATA->identity_addr);
        gap_set_random_device_address(SECURITY_PERSISTENT_DATA->identity_addr);
        g_adv_data_len = ADVERTISING_ITEM_OFFSET_COMPLETE_LOCAL_NAME +
                         g_adv_data[ADVERTISING_ITEM_OFFSET_COMPLETE_LOCAL_NAME - 2] - 1;
        uart_at_start();
        break;

    case HCI_EVENT_COMMAND_COMPLETE:
        switch (hci_event_command_complete_get_command_opcode(packet))
        {
        // add your code to check command complete response
        // case :
        //     break;
        default:
            break;
        }
        break;

    case HCI_EVENT_LE_META:
        switch (hci_event_le_meta_get_subevent_code(packet))
        {
        case HCI_SUBEVENT_LE_ENHANCED_CONNECTION_COMPLETE:
            {
                const le_meta_event_enh_create_conn_complete_t *complete =
                    decode_hci_le_meta_event(packet, le_meta_event_enh_create_conn_complete_t);
                if ((complete->role == HCI_ROLE_SLAVE) && (complete->status == 0))
                {
                    att_set_db(complete->handle, profile_data);
                }
                if (complete->status == 0)
                    gap_set_phy(complete->handle, 0, PHY_2M_BIT, PHY_2M_BIT, HOST_PREFER_S2_CODING);
                at_on_connection_complete(complete);
            }
            break;
        case HCI_SUBEVENT_LE_EXTENDED_ADVERTISING_REPORT:
            {
                const le_ext_adv_report_t *report = decode_hci_le_meta_event(packet, le_meta_event_ext_adv_report_t)->reports;
                at_on_adv_report(report);
            }
            break;
        case HCI_SUBEVENT_LE_ADVERTISING_SET_TERMINATED:
            at_on_adv_set_terminated();
            break;
        default:
            break;
        }

        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        at_on_disconnect(decode_hci_event_disconn_complete(packet));
        break;

    case ATT_EVENT_CAN_SEND_NOW:
        // add your code
        break;

    case BTSTACK_EVENT_USER_MSG:
        p_user_msg = hci_event_packet_get_user_msg(packet);
        user_msg_handler(p_user_msg->msg_id, p_user_msg->data, p_user_msg->len);
        break;

    default:
        break;
    }
}

static btstack_packet_callback_registration_t hci_event_callback_registration;

static void sm_packet_handler(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    uint8_t event = hci_event_packet_get_type(packet);

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (event)
    {
    case SM_EVENT_JUST_WORKS_REQUEST:
        sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
        break;
    case SM_EVENT_STATE_CHANGED:
        {
            const sm_event_state_changed_t *state_changed = decode_hci_event(packet, sm_event_state_changed_t);
            at_on_sm_state_changed(state_changed->reason);
        }
        break;
    default:
        break;
    }
}

static btstack_packet_callback_registration_t sm_event_callback_registration  = {.callback = &sm_packet_handler};

uint32_t setup_profile(void *data, void *user_data)
{
    sm_add_event_handler(&sm_event_callback_registration);
    sm_config(0, IO_CAPABILITY_NO_INPUT_NO_OUTPUT, 0, SECURITY_PERSISTENT_DATA);
    sm_add_event_handler(&sm_event_callback_registration);
    att_server_init(att_read_callback, att_write_callback);
    hci_event_callback_registration.callback = &user_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(&user_packet_handler);
    return 0;
}

