/*
* Copyright (C) INGCHIPS. All rights reserved.
* This code is INGCHIPS proprietary and confidential.
* Any use of the code for whatever purpose is subject to
* specific written permission of INGCHIPS.
*/

#include <stdio.h>
#include <string.h>
#include "ingsoc.h"
#include "platform_api.h"
#include "att_db.h"
#include "ota_service.h"
#include "rom_tools.h"
#include "eflash.h"

prog_ver_t prog_ver = { .major = 1, .minor = 0, .patch = 0 };

#define PAGE_SIZE (EFLASH_ERASABLE_SIZE)

#include "../data/gatt.const"
#define ATT_OTA_HANDLE_VER          HANDLE_FOTA_VERSION
#define ATT_OTA_HANDLE_DATA         HANDLE_FOTA_DATA
#define ATT_OTA_HANDLE_CTRL         HANDLE_FOTA_CONTROL

static uint8_t  ota_ctrl[] = {OTA_STATUS_DISABLED};
static uint8_t  ota_downloading = 0;
static uint32_t ota_start_addr = 0;
static uint32_t ota_page_offset = 0;
static uint8_t  page_buffer[PAGE_SIZE];


int ota_write_callback(uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, const uint8_t *buffer, uint16_t buffer_size)
{
    if (transaction_mode != ATT_TRANSACTION_MODE_NONE)
        return 0;

    if (att_handle == ATT_OTA_HANDLE_CTRL)
    {
        if (OTA_CTRL_START == buffer[0])
        {
            ota_ctrl[0] = OTA_STATUS_OK;
            ota_start_addr = 0;
            ota_downloading = 0;
            return 0;
        }

        switch (buffer[0])
        {
        case OTA_CTRL_PAGE_BEGIN:
            ota_start_addr = *(uint32_t *)(buffer + 1);
            if (ota_start_addr & 0x3)
            {
                ota_ctrl[0] = OTA_STATUS_ERROR;
                return 0;
            }
            else
                ota_ctrl[0] = OTA_STATUS_OK;
            ota_downloading = 1;
            ota_page_offset = 0;
            break;
        case OTA_CTRL_PAGE_END:
            program_flash(ota_start_addr, page_buffer, ota_page_offset);

            ota_downloading = 0;
            {
                uint16_t len = *(uint16_t *)(buffer + 1);
                uint16_t crc_value = *(uint16_t *)(buffer + 3);
                if (ota_page_offset < len)
                {
                    ota_ctrl[0] = OTA_STATUS_WAIT_DATA;
                    break;
                }

                if (crc((uint8_t *)ota_start_addr, len) != crc_value)
                    ota_ctrl[0] = OTA_STATUS_ERROR;
                else
                    ota_ctrl[0] = OTA_STATUS_OK;
            }
            break;
        case OTA_CTRL_READ_PAGE:
            if (ota_downloading)
                ota_ctrl[0] = OTA_STATUS_ERROR;
            else
            {
                ota_start_addr = *(uint32_t *)(buffer + 1);
                ota_ctrl[0] = OTA_STATUS_OK;
            }
            break;
        case OTA_CTRL_METADATA:
            if (OTA_STATUS_OK != ota_ctrl[0])
                break;
            if ((0 == ota_downloading) || (buffer_size < 1 + sizeof(ota_meta_t)))
            {
                const ota_meta_t  *meta = (const ota_meta_t *)(buffer + 1);
                int s = buffer_size - 1;
                if (crc((uint8_t *)&meta->entry, s - sizeof(meta->crc_value)) != meta->crc_value)
                {
                    ota_ctrl[0] = OTA_STATUS_ERROR;
                    break;
                }
                flash_do_update((s - sizeof(ota_meta_t)) / sizeof(meta->blocks[0]),
                                meta->blocks,
                                page_buffer);
            }
            else
            {
                ota_ctrl[0] = OTA_STATUS_ERROR;
            }
            break;
        case OTA_CTRL_REBOOT:
            if (OTA_STATUS_OK == ota_ctrl[0])
            {
                if (ota_downloading)
                    ota_ctrl[0] = OTA_STATUS_ERROR;
                else
                    platform_reset();
            }
            break;
        default:
            ota_ctrl[0] = OTA_STATUS_ERROR;
        }
    }
    else if (att_handle == ATT_OTA_HANDLE_DATA)
    {
        if (OTA_STATUS_OK == ota_ctrl[0])
        {
            if (   (buffer_size & 0x3) || (0 == ota_downloading)
                || (ota_page_offset + buffer_size > PAGE_SIZE))
            {
                ota_ctrl[0] = OTA_STATUS_ERROR;
                return 0;
            }

            memcpy(page_buffer + ota_page_offset,
                   buffer, buffer_size);
            ota_page_offset += buffer_size;
        }
    }
    else;

    return 0;
}

int ota_read_callback(uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size)
{
    if (buffer == NULL)
    {
        if (att_handle == ATT_OTA_HANDLE_CTRL)
            return 1;
        else if (att_handle == ATT_OTA_HANDLE_VER)
            return sizeof(ota_ver_t);
        else
            return 0;
    }

    if (att_handle == ATT_OTA_HANDLE_CTRL)
    {
        buffer[0] = ota_ctrl[0];
    }
    else if (att_handle == ATT_OTA_HANDLE_VER)
    {
        ota_ver_t *this_version = (ota_ver_t *)buffer;
        const platform_ver_t * v = platform_get_version();

        this_version->platform.major = v->major;
        this_version->platform.minor = v->minor;
        this_version->platform.patch = v->patch;
        this_version->app = prog_ver;
    }

    return buffer_size;
}

