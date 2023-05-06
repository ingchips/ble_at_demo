#include <stdio.h>
#include <string.h>
#include "profile.h"
#include "ingsoc.h"
#include "platform_api.h"
#include "kv_storage.h"
#include "FreeRTOS.h"
#include "task.h"
#include "eflash.h"
#include "trace.h"
#include "uart_at.h"

static uint32_t uart0_isr(void *data);

#define CMD_PORT  APB_UART0

#include "../data/setup_soc.cgen"

static uint32_t cb_hard_fault(hard_fault_info_t *info, void *_)
{
    platform_printf("HARDFAULT:\nPC : 0x%08X\nLR : 0x%08X\nPSR: 0x%08X\n"
                    "R0 : 0x%08X\nR1 : 0x%08X\nR2 : 0x%08X\nP3 : 0x%08X\n"
                    "R12: 0x%08X\n",
                    info->pc, info->lr, info->psr,
                    info->r0, info->r1, info->r2, info->r3, info->r12);
    for (;;);
}

static uint32_t cb_assertion(assertion_info_t *info, void *_)
{
    platform_printf("[ASSERTION] @ %s:%d\n",
                    info->file_name,
                    info->line_no);
    for (;;);
}

static uint32_t cb_heap_out_of_mem(uint32_t tag, void *_)
{
    platform_printf("[OOM] @ %d\n", tag);
    for (;;);
}

#define PRINT_PORT    APB_UART0

uint32_t cb_putc(char *c, void *dummy)
{
    while (apUART_Check_TXFIFO_FULL(PRINT_PORT) == 1);
    UART_SendData(PRINT_PORT, (uint8_t)*c);
    return 0;
}

int fputc(int ch, FILE *f)
{
    cb_putc((char *)&ch, NULL);
    return ch;
}

void update_baud(uint32_t baud)
{
    apUART_BaudRateSet(CMD_PORT, SYSCTRL_GetClk(SYSCTRL_ITEM_APB_UART0), baud);
}

void setup_peripherals(void)
{
    GIO_EnableRetentionGroupA(0);
    cube_setup_peripherals();
    platform_enable_irq(PLATFORM_CB_IRQ_UART0, 1);
}

void config_wakeup_and_shutdown(void)
{
#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
    // type A excluding GPIO0
    #define WAKEUP_PIN  GIO_GPIO_6

    SYSCTRL_ClearClkGate((WAKEUP_PIN >= GIO_GPIO_NUMBER / 2) ? SYSCTRL_ITEM_APB_GPIO1 : SYSCTRL_ITEM_APB_GPIO0);
    GIO_SetDirection(WAKEUP_PIN, GIO_DIR_INPUT);
    PINCTRL_Pull(WAKEUP_PIN, PINCTRL_PULL_DOWN);
    GIO_EnableDeeperSleepWakeupSourceGroupA(1, 1);
    GIO_EnableRetentionGroupA(1);
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
    // use EXT_INT to power on again
#endif
    platform_shutdown(0, NULL, 0);
}

uint32_t on_deep_sleep_wakeup(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    setup_peripherals();
    return 0;
}

uint32_t query_deep_sleep_allowed(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    return 1;
}

static void watchdog_task(void *pdata)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        TMR_WatchDogRestart();
    }
}

#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
    #define DB_FLASH_ADDRESS  0x2040000
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
    #define DB_FLASH_ADDRESS  0x40000
#endif
int db_write_to_flash(const void *db, const int size)
{
    program_flash(DB_FLASH_ADDRESS, (const uint8_t *)db, size);
    return KV_OK;
}

int db_read_from_flash(void *db, const int max_size)
{
    memcpy(db, (void *)DB_FLASH_ADDRESS, max_size);
    return KV_OK;
}

trace_rtt_t trace_ctx = {0};

uint32_t uart0_isr(void *user_data)
{
    uint32_t status;

    while(1)
    {
        status = apUART_Get_all_raw_int_stat(APB_UART0);
        if (status == 0)
            break;

        APB_UART0->IntClear = status;

        // rx int
        if (status & (1 << bsUART_RECEIVE_INTENAB))
        {
            while (apUART_Check_RXFIFO_EMPTY(APB_UART0) != 1)
            {
                char c = APB_UART0->DataRead;
                at_rx_data(&c, 1);
            }
        }
    }
    return 0;
}

const platform_evt_cb_table_t evt_cb_table =
{
    .callbacks = {
        [PLATFORM_CB_EVT_HARD_FAULT] = {
            .f = (f_platform_evt_cb)cb_hard_fault
        },
        [PLATFORM_CB_EVT_ASSERTION] = {
            .f = (f_platform_evt_cb)cb_assertion
        },
        [PLATFORM_CB_EVT_HEAP_OOM] = {
            .f = (f_platform_evt_cb)cb_heap_out_of_mem
        },
        [PLATFORM_CB_EVT_PUTC] = {
            .f = (f_platform_evt_cb)cb_putc
        },
        [PLATFORM_CB_EVT_ON_DEEP_SLEEP_WAKEUP] = {
            .f = (f_platform_evt_cb)on_deep_sleep_wakeup
        },
        [PLATFORM_CB_EVT_QUERY_DEEP_SLEEP_ALLOWED] = {
            .f = query_deep_sleep_allowed
        },
        [PLATFORM_CB_EVT_PROFILE_INIT] = {
            .f = setup_profile
        },
        [PLATFORM_CB_EVT_TRACE] = {
            .f = (f_platform_evt_cb)cb_trace_rtt,
            .user_data = &trace_ctx
        },
    }
};

const platform_irq_cb_table_t irq_cb_table =
{
    .callbacks = {
        [PLATFORM_CB_IRQ_UART0] = {
            .f = (f_platform_irq_cb)uart0_isr
        },
    }
};

int app_main()
{
    cube_soc_init();

    // setup handlers
    platform_set_evt_callback_table(&evt_cb_table);
    platform_set_irq_callback_table(&irq_cb_table);

    setup_peripherals();

    xTaskCreate(watchdog_task,
           "w",
           configMINIMAL_STACK_SIZE,
           NULL,
           (configMAX_PRIORITIES - 1),
           NULL);
    kv_init(db_write_to_flash, db_read_from_flash);

    trace_rtt_init(&trace_ctx);

    platform_config(PLATFORM_CFG_TRACE_MASK, 0xff);
    return 0;
}

