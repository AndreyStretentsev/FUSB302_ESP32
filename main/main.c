#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"

#include "driver/i2c.h"
#include "driver/timer.h"

#include "core.h"
#include "Port.h"

#include "esp_timer.h"
#include "defines.h"

static DevicePolicyPtr_t dpm;
static Port_t ports[1]; 

volatile bool fusb_ready = false;
esp_timer_handle_t fusb_timer;

#define LOG_DATA_SIZE   (4096 * 6)
static FSC_U8 pd_log_data[LOG_DATA_SIZE] = {0};
static FSC_U8 tc_log_data[LOG_DATA_SIZE] = {0};
static FSC_U8 ic_log_data[LOG_DATA_SIZE] = {0};

static const char tc_state_str[IllegalCable + 1][100] = {
    "Disabled",
    "ErrorRecovery",
    "Unattached",
    "AttachWaitSink",
    "AttachedSink",
    "AttachWaitSource",
    "AttachedSource",
    "TrySource",
    "TryWaitSink",
    "TrySink",
    "TryWaitSource",
    "AudioAccessory",
    "DebugAccessorySource",
    "AttachWaitAccessory",
    "PoweredAccessory",
    "UnsupportedAccessory",
    "DelayUnattached",
    "UnattachedSource",
    "DebugAccessorySink",
    "AttachWaitDebSink",
    "AttachedDebSink",
    "AttachWaitDebSource",
    "AttachedDebSource",
    "TryDebSource",
    "TryWaitDebSink",
    "UnattachedDebSource",
    "IllegalCable",
};

static const char pd_state_str[peSendGenericData + 1][100] = {
    "peDisabled",             /* Policy engine is disabled */
    "peErrorRecovery",            /* Error recovery state */
    "peSourceHardReset",          /* Received a hard reset */
    "peSourceSendHardReset",      /* Source send a hard reset */
    "peSourceSoftReset",          /* Received a soft reset */
    "peSourceSendSoftReset",      /* Send a soft reset */
    "peSourceStartup",            /* Initial state */
    "peSourceSendCaps",           /* Send the source capabilities */
    "peSourceDiscovery",          /* Waiting to detect a USB PD sink */
    "peSourceDisabled",           /* Disabled state */
    /* 10 */
    "peSourceTransitionDefault",  /* Transition to default 5V state */
    "peSourceNegotiateCap",       /* Negotiate capability and PD contract */
    "peSourceCapabilityResponse", /* Respond to a request with a reject/wait */
    "peSourceWaitNewCapabilities", /* Wait for new Source Caps from DPM */
    "peSourceTransitionSupply",   /* Transition the power supply */
    "peSourceReady",              /* Contract is in place and voltage is stable */
    "peSourceGiveSourceCaps",     /* State to resend source capabilities */
    "peSourceGetSinkCaps",        /* State to request the sink capabilities */
    "peSourceSendPing",           /* State to send a ping message */
    "peSourceGotoMin",            /* State to send gotoMin and prep the supply */
    /* 20 */
    "peSourceGiveSinkCaps",       /* State to send sink caps if dual-role */
    "peSourceGetSourceCaps",      /* State to request source caps from the UFP */
    "peSourceSendDRSwap",         /* State to send a DR_Swap message */
    "peSourceEvaluateDRSwap",     /* Evaluate DR swap request */
    "peSourceAlertReceived",      /* Source has received an alert */

    "peSinkHardReset",            /* Received a hard reset */
    "peSinkSendHardReset",        /* Sink send hard reset */
    "peSinkSoftReset",            /* Sink soft reset */
    "peSinkSendSoftReset",        /* Sink send soft reset */
    "peSinkTransitionDefault",    /* Transition to the default state */
    /* 30 */
    "peSinkStartup",              /* Initial sink state */
    "peSinkDiscovery",            /* Sink discovery state */
    "peSinkWaitCaps",             /* Sink wait for capabilities state */
    "peSinkEvaluateCaps",         /* Sink evaluate the rx'd source caps */
    "peSinkSelectCapability",     /* Sink selecting a capability */
    "peSinkTransitionSink",       /* Sink transitioning the current power */
    "peSinkReady",                /* Sink ready state */
    "peSinkGiveSinkCap",          /* Sink send capabilities state */
    "peSinkGetSourceCap",         /* Sink get source capabilities state */
    "peSinkGetSinkCap",           /* Sink get the sink caps of the source */
    /* 40 */
    "peSinkGiveSourceCap",        /* Sink send the source caps if dual-role */
    "peSinkSendDRSwap",           /* State to send a DR_Swap message */
    "peSinkAlertReceived",        /* Sink received alert message */
    "peSinkEvaluateDRSwap",       /* Evaluate DR swap request */
    "peSourceSendVCONNSwap",      /* Initiate a VCONN swap sequence */
    "peSourceEvaluateVCONNSwap",  /* Evaluate VCONN swap request */
    "peSinkSendVCONNSwap",        /* Initiate a VCONN swap sequence */
    "peSinkEvaluateVCONNSwap",    /* Evaluate VCONN swap request */
    "peSourceSendPRSwap",         /* Initiate a PR Swap sequence */
    "peSourceEvaluatePRSwap",     /* Evaluate PR swap request */
    /* 50 */
    "peSinkSendPRSwap",           /* Initiate a PR Swap sequence */
    "peSinkEvaluatePRSwap",       /* Evaluate PR swap request */

    "peGetCountryCodes",          /* Send Get country code message */
    "peGiveCountryCodes",         /* Send country codes */
    "peNotSupported",             /* Send a reject/NS to unknown msg */
    "peGetPPSStatus",             /* Sink request PPS source status */
    "peGivePPSStatus",            /* Source provide PPS status */
    "peGiveCountryInfo",          /* Send country info */

    /* VDM states */
    "peGiveVdm",
    /* ---------- UFP VDM State Diagram ---------- */
    "peUfpVdmGetIdentity",        /* Requesting Identity information from DPM */
    /* 60 */
    "peUfpVdmSendIdentity",       /* Sending Discover Identity ACK */

    "peUfpVdmGetSvids",           /* Requesting SVID info from DPM */
    "peUfpVdmSendSvids",          /* Sending Discover SVIDs ACK */
    "peUfpVdmGetModes",           /* Requesting Mode info from DPM */
    "peUfpVdmSendModes",          /* Sending Discover Modes ACK */
    "peUfpVdmEvaluateModeEntry",  /* Evaluate request to enter a mode */
    "peUfpVdmModeEntryNak",       /* Sending Enter Mode NAK response */
    "peUfpVdmModeEntryAck",       /* Sending Enter Mode ACK response */
    "peUfpVdmModeExit",           /* Evalute request to exit mode */
    "peUfpVdmModeExitNak",        /* Sending Exit Mode NAK reponse */
    "peUfpVdmModeExitAck",        /* Sending Exit Mode ACK Response */

    /* ---------- UFP VDM Attention State Diagram ---------- */
    "peUfpVdmAttentionRequest",   /* Sending Attention Command */
    /* ---------- DFP to UFP VDM Discover Identity State Diagram ---------- */
    "peDfpUfpVdmIdentityRequest", /* Sending Identity Request */
    "peDfpUfpVdmIdentityAcked",   /* Inform DPM of Identity */
    "peDfpUfpVdmIdentityNaked",   /* Inform DPM of result */
    /* ---------- DFP to Cable Plug VDM Discover Identity State Diagram --- */
    "peDfpCblVdmIdentityRequest", /* Sending Identity Request */
    "peDfpCblVdmIdentityAcked",   /* Inform DPM */
    "peDfpCblVdmIdentityNaked",   /* Inform DPM */
    /* ---------- DFP VDM Discover SVIDs State Diagram ---------- */
    "peDfpVdmSvidsRequest",       /* Sending Discover SVIDs request */
    "peDfpVdmSvidsAcked",         /* Inform DPM */
    "peDfpVdmSvidsNaked",         /* Inform DPM */

    /* ---------- DFP VDM Discover Modes State Diagram ---------- */
    "peDfpVdmModesRequest",       /* Sending Discover Modes request */
    "peDfpVdmModesAcked",         /* Inform DPM */
    "peDfpVdmModesNaked",         /* Inform DPM */
    /* ---------- DFP VDM Enter Mode State Diagram ---------- */
    "peDfpVdmModeEntryRequest",   /* Sending Mode Entry request */
    "peDfpVdmModeEntryAcked",     /* Inform DPM */
    "peDfpVdmModeEntryNaked",     /* Inform DPM */
    /* ---------- DFP VDM Exit Mode State Diagram ---------- */
    "peDfpVdmModeExitRequest",    /* Sending Exit Mode request */
    "peDfpVdmExitModeAcked",      /* Inform DPM */
    /* if Exit Mode not Acked, go to Hard Reset state */
    /* ---------- Source Startup VDM Discover Identity State Diagram ------ */
    "peSrcVdmIdentityRequest",    /* sending Discover Identity request */
    "peSrcVdmIdentityAcked",      /* inform DPM */

    "peSrcVdmIdentityNaked",      /* inform DPM */
    /* ---------- DFP VDM Attention State Diagram ---------- */
    "peDfpVdmAttentionRequest",   /* Attention Request received */
    /* ---------- Cable Ready VDM State Diagram ---------- */
    "peCblReady",                 /* Cable power up state? */
    /* ---------- Cable Discover Identity VDM State Diagram ---------- */
    "peCblGetIdentity",           /* Discover Identity request received */
    "peCblGetIdentityNak",        /* Respond with NAK */
    "peCblSendIdentity",          /* Respond with Ack */
    /* ---------- Cable Discover SVIDs VDM State Diagram ---------- */
    "peCblGetSvids",              /* Discover SVIDs request received */
    "peCblGetSvidsNak",           /* Respond with NAK */
    "peCblSendSvids",             /* Respond with ACK */
    /* ---------- Cable Discover Modes VDM State Diagram ---------- */
    "peCblGetModes",              /* Discover Modes request received */

    "peCblGetModesNak",           /* Respond with NAK */
    "peCblSendModes",             /* Respond with ACK */
    /* ---------- Cable Enter Mode VDM State Diagram ---------- */
    "peCblEvaluateModeEntry",     /* Enter Mode request received */
    "peCblModeEntryAck",          /* Respond with NAK */
    "peCblModeEntryNak",          /* Respond with ACK */
    /* ---------- Cable Exit Mode VDM State Diagram ---------- */
    "peCblModeExit",              /* Exit Mode request received */
    "peCblModeExitAck",           /* Respond with NAK */
    "peCblModeExitNak",           /* Respond with ACK */
    /* ---------- DP States ---------- */
    "peDpRequestStatus",          /* Requesting PP Status */
    "peDpRequestStatusAck",
    "peDpRequestStatusNak",
    "peDpRequestConfig",         /* Request Port partner Config */
    "peDpRequestConfigAck",
    "peDpRequestConfigNak",
    /* ---------- BIST Receive Mode --------------------- */
    "PE_BIST_Receive_Mode",       /* Bist Receive Mode */
    "PE_BIST_Frame_Received",     /* Test Frame received by Protocol layer */
    /* ---------- BIST Carrier Mode and Eye Pattern ----- */
    "PE_BIST_Carrier_Mode_2",     /* BIST Carrier Mode 2 */
    "PE_BIST_Test_Data",          /* BIST Test Data State */
    "dbgGetRxPacket",             /* Debug point for Rx packet handling */
    "dbgSendTxPacket",            /* Debug point for Tx packet handling */
    "peSendCableReset",           /* State to send cable reset */
    "peSendGenericCommand",       /* Send an arbitrary command from the GUI */
    "peSendGenericData",
};

static void IRAM_ATTR fusb_isr_handler(void* arg)
{
    fusb_ready = true;
}

static void oneshot_timer_callback(void* arg)
{
    fusb_ready = true;
}


void app_main()
{
    i2c_config_t i2c_config = {
        .mode=I2C_MODE_MASTER,
        .sda_io_num=I2C_SDA,
        .scl_io_num=I2C_SCL,
        .sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,
        .master.clk_speed=400000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    gpio_set_direction(FUSB_INT, GPIO_MODE_INPUT);
    gpio_set_intr_type(FUSB_INT, GPIO_INTR_NEGEDGE);
    gpio_set_pull_mode(FUSB_INT, GPIO_PULLUP_ONLY);

    // Install gpio ISR
    gpio_install_isr_service(0);
    // Hook isr handler for specific gpio pin
    gpio_isr_handler_add(FUSB_INT, fusb_isr_handler, (void*) FUSB_INT);

    platform_log_init();

    DPM_Init(&dpm);

    ports[0].dpm = dpm;
    ports[0].PortID = 0;
    core_initialize(&ports[0], FUSB_ADDR);
    core_set_sink(&ports[0]);
    core_enable_typec(&ports[0], TRUE);
    core_enable_pd(&ports[0], TRUE);

    DPM_AddPort(dpm, &ports[0]);

    const esp_timer_create_args_t fusb_timer_args = {
        .callback = &oneshot_timer_callback,
        .name = "fusb-timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&fusb_timer_args, &fusb_timer));

    while(1)
    {
        if (fusb_ready)
        {
            esp_timer_stop(fusb_timer);
            core_state_machine(&ports[0]);

            fusb_ready = false;

            /*
             * It is possible for the state machine to go into idle mode with
             * the interrupt pin still low and as a result the edge-sensitive
             * IRQ won't get triggered.  Check here before returning to wait
             * on the IRQ.
             */
            if(platform_get_device_irq_state(ports[0].PortID))
            {
                fusb_ready = true;
            }
            else
            {
                /* If needed, enable timer interrupt before idling */
                uint32_t timer_value = core_get_next_timeout(&ports[0]);

                if (timer_value > 0)
                {
                    if (timer_value == 1)
                    {
                        /* A value of 1 indicates that a timer has expired
                         * or is about to expire and needs further processing.
                         */
                        fusb_ready = true;
                    }
                    else
                    {
                        /* A non-zero time requires a future timer interrupt */
                        ESP_ERROR_CHECK(esp_timer_start_once(fusb_timer, timer_value));
                    }
                }
                else
                {
                    /* Optional: Disable system timer(s) here to save power
                     * while in Idle mode.
                     *
                     * Note: this is the place that gets called when the FUSB
                     * finished the process for an interrupt. Here you will know
                     * for sure that you have the final state of USB
                     */
                    FSC_U32 pd_log_size =
                        GetStateLog(&ports[0].PDStateLog, pd_log_data, LOG_DATA_SIZE);
                    FSC_U32 tc_log_size =
                        GetStateLog(&ports[0].TypeCStateLog, tc_log_data, LOG_DATA_SIZE);
                    FSC_U32 ic_log_size =
                        platform_get_log(ic_log_data, LOG_DATA_SIZE);
                    printf("FUSB302B logs:\n");
                    for (int i = 0; i < ic_log_size; i += 6) {
                        uint8_t addr, val, w;
                        uint16_t tms; 
                        w = ic_log_data[i] & 0x80;
                        addr = ic_log_data[i] & 0x7F;
                        val = ic_log_data[i + 1];
                        memcpy(&tms, &ic_log_data[i + 2], 2);
                        printf("%s, addr 0x%02X, val 0x%02X : %d ms\n", w ? "r" : "w", addr, val, tms);
                    }
                    printf("\nType C logs:\n");
                    for (int i = 0; i < tc_log_size; i += 6) {
                        uint16_t state;
                        uint16_t tms; 
                        memcpy(&state, &tc_log_data[i], 2);
                        memcpy(&tms, &tc_log_data[i + 2], 2);
                        printf("%s : %d ms\n", tc_state_str[state], tms);
                    }
                    printf("\nPower delivery logs:\n");
                    for (int i = 0; i < pd_log_size; i += 6) {
                        uint16_t state;
                        uint16_t tms; 
                        memcpy(&state, &pd_log_data[i], 2);
                        memcpy(&tms, &pd_log_data[i + 2], 2);
                        printf("%s : %d ms\n", pd_state_str[state], tms);
                    }
                    printf("-------------------------------\n");
                }
            }
        }
    }
}
