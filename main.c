//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************


//*****************************************************************************
//
// Application Name     -   SSL Demo
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************

// Simplelink includes
#include "simplelink.h"

#include<stdio.h>
// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "interrupt.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "timer.h"
#include "rom.h"
#include "rom_map.h"
#include "pin.h"
#include "utils.h"
#include "uart.h"

//Common interface includes

#include "common.h"

#include "gpio_if.h"
#include "uart_if.h"
#include "pinmux.h"
#include "timer_if.h"
#include "string.h"
#include "udma_if.h"

#include "spi.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"

#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1


#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Spring2019"
#define SERVER_NAME                "translation.googleapis.com"
#define SERVER_NAME_IFTTT          "maker.ifttt.com"
#define GOOGLE_DST_PORT             443
#define TIMER_FREQ      80000000
#define SPI_IF_BIT_RATE  450000
#define NOISE_THRESHOLD 200000

#define PIN18_ADDR GPIOA3_BASE
#define PIN18_OFF 0x80
#define BASE_TIMER TIMERA2_BASE
#define SECONDARY_TIMER TIMERA0_BASE
#define THIRTIARY_TIMER TIMERA1_BASE
#define DC_BASE GPIOA0_BASE
#define DC_OFFSET 0x1
#define R_BASE GPIOA1_BASE
#define R_OFFSET 0x10
#define OLEDCS_BASE GPIOA3_BASE
#define OLEDCS_OFFSET 0x10
#define ADC_CS_BASE GPIOA3_BASE
#define ADC_CS_OFFSET 0x80

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

#define UART_BAUD_RATE  115200
#define SYSCLK          80000000
#define CONSOLE_1         UARTA1_BASE
#define CONSOLE_PERIPH_1  PRCM_UARTA1

#define SL_SSL_CA_CERT_IFTTT "/usercerts/ifttt_root.der"
#define SL_SSL_CA_CERT "/usercerts/google_root.der"

//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                2    /* Current Date */
#define MONTH               6     /* Month 1-12 */
#define YEAR                2019  /* Current year */
#define HOUR                00    /* Time - hours */
#define MINUTE              00    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define POSTHEADER "POST /trigger/tweet_put/with/key/gsRshHJNFNzUxb_WX-T-NhC_NI-EFTQ-6ZDsfSYyQM1 HTTP/1.1\r\n"
#define GETHEADER "GET /language/translate/v2?key=AIzaSyCRrF3EXWa-X04_Dv5-j70V8m-mYo4J79o&source=EN&target=FR&q="
#define GETHEADER2 " HTTP/1.1\r\n"
#define HOSTHEADER_GET "Host: translation.googleapis.com\r\n"
#define HOSTHEADER_POST "Host: maker.ifttt.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA_SPLIT1 "{\"value1\":\""
#define DATA_SPLIT2 "\",\"value2\":\"LAB5 A03\",\"value3\":\"EEC172\"}\n\r\n\r"

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

char array_of_char[10][6] = {
    " 0",
    "1",
    "ABC2",
    "DEF3",
    "GHI4",
    "JKL5",
    "MNO6",
    "PQRS7",
    "TUV8",
    "WXYZ9"
};

// String processing
char tx_string[128];
short string_index;
unsigned short btn_press_index;
short lastRowPressed = 10;
short lastColPressed = 10;
int sendDeletePressed;

// OLED
int txt_size = 1;
int pos_x = 6;
int l_pos_x;
int pos_y = 10;
int l_pos_y;
int x_ovrflw = 0;
int y_ovrflw = 0;

// ADC
int N = 410;       // block size
int samples[410];  // buffer to store N samples
int count;         // samples count
int flag;         // flag set when the samples buffer is full with N samples
int new_dig;      // flag set when inter-digit interval (pause) is detected
long int power_all[8];       // array to store calculated power of 8 frequencies
int coeff[8] = {31548, 31281, 30951, 30556, 29144, 28361, 27409, 26258};
int f_tone[8] = { 697, 770, 852, 941, 1209, 1336, 1477, 1633 }; // frequencies of rows & columns

// ADC processing
unsigned char c = 0; // dummy variable to send via SPI
unsigned short miso_data;
unsigned char miso_data0;
unsigned char miso_data1;

int ifttt;
char translated_buffer[128];

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int);
static int http_get(int);
long int goertzel (int sample[], long int coeff, int N);
void post_test (void);
void processString(int);

//*****************************************************************************
// Event Handlers -- Start
//*****************************************************************************
void TimerIntHandler(){
    // clear interrupt timer, other wise the program will be stuck in this function in a loop!
    TimerIntClear(BASE_TIMER,TIMER_TIMA_TIMEOUT);
    if (count < N) {
        SPICSEnable(GSPI_BASE);
        GPIOPinWrite(ADC_CS_BASE, ADC_CS_OFFSET, 0);
        SPIDataPut(GSPI_BASE, c);
        SPIDataGet(GSPI_BASE, (unsigned long *) &miso_data0);
        SPIDataPut(GSPI_BASE, c);
        SPIDataGet(GSPI_BASE, (unsigned long *) &miso_data1);

        miso_data0 &= 0x1f;
        miso_data1 >>= 3;
        miso_data = (miso_data0 << 5) | miso_data1;

        samples[count++] = (int) miso_data; // scale down ADC reading, store the value in the samples buffer & increment the count
        GPIOPinWrite(ADC_CS_BASE, ADC_CS_OFFSET, ADC_CS_OFFSET);
        SPICSDisable(GSPI_BASE);
    } else if (count == N) { // if the buffer is full with N samples
      flag = 1;
      // Samples are collected at this point. Disable the 16kHz timer.
      TimerIntClear(BASE_TIMER,TIMER_TIMA_TIMEOUT);
      TimerDisable(BASE_TIMER,TIMER_A);
    }
}
// reset same button pressed variables after timeout
void TimerIntHandler2(){
    TimerIntClear(SECONDARY_TIMER,TIMER_TIMA_TIMEOUT);
    TimerDisable(SECONDARY_TIMER,TIMER_A);
    lastColPressed = 10;
    lastRowPressed = 10;
    btn_press_index = 0;
    sendDeletePressed = 0;
}

void calculateAverageAndUpdateDCBias() {
    int total = 0;
    int i = 0;
    for(i = 0; i < N; i++){
        total = total + samples[i];
    }
    int average = total / N;
    for(i = 0; i < N; i++){
        samples[i] = samples[i] - average;
    }
}
//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE: 
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n", 
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default: 
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}

//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP,uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
    long lRetVal = -1;
    int iSockID;

    if(ifttt){
        lRetVal = sl_NetAppDnsGetHostByName(SERVER_NAME_IFTTT, strlen((const char *)SERVER_NAME_IFTTT),
                                            (unsigned long*)&uiIP, SL_AF_INET);
        Report("HOST NAME: %s\n\n\r", SERVER_NAME_IFTTT);

        lRetVal = sl_NetAppDnsGetHostByName(SERVER_NAME_IFTTT, strlen((const char *)SERVER_NAME_IFTTT),
                                           (unsigned long*)&uiIP, SL_AF_INET);

       if(lRetVal < 0) {
           return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
       }

    } else {
        lRetVal = sl_NetAppDnsGetHostByName(SERVER_NAME, strlen((const char *)SERVER_NAME),
                                                    (unsigned long*)&uiIP, SL_AF_INET);
        Report("HOST NAME: %s\n\n\r", SERVER_NAME);

        lRetVal = sl_NetAppDnsGetHostByName(SERVER_NAME, strlen((const char *)SERVER_NAME),
                                           (unsigned long*)&uiIP, SL_AF_INET);

       if(lRetVal < 0) {
           return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
       }
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket 
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //
    //configure the socket with CA certificate - for server verification
    //
    if(ifttt){
        lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                                   SL_SO_SECURE_FILES_CA_FILE_NAME, \
                                   SL_SSL_CA_CERT_IFTTT, \
                                   strlen(SL_SSL_CA_CERT_IFTTT));
    } else {
        lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                                   SL_SO_SECURE_FILES_CA_FILE_NAME, \
                                   SL_SSL_CA_CERT, \
                                   strlen(SL_SSL_CA_CERT));
    }

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal < 0) {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }
    else {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}



long printErrConvenience(char * msg, long retVal) {
    UART_PRINT(msg);
    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}



int connectToAccessPoint() {
    long lRetVal = -1;
    GPIO_IF_LedConfigure(LED1|LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
long lRetVal = -1;
void main() {
    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    InitTerm();
    ClearTerm();
    UART_PRINT("Hello world!\n\r");

    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }

    // Enable the SPI module clock
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    // Reset the peripheral
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    // Reset SPI - performs software reset
    SPIReset(GSPI_BASE);
    // Configure SPI interface
    SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                    SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                    (SPI_SW_CTRL_CS |
                    SPI_4PIN_MODE |
                    SPI_TURBO_OFF |
                    SPI_CS_ACTIVELOW |
                    SPI_WL_8));
    // Enable SPI channel on CC3200 board
    SPIEnable(GSPI_BASE);
    Adafruit_Init();

    // Set OLED position
    l_pos_x = pos_x;
    l_pos_y = pos_y;

    fillScreen(BLACK);
    memset(tx_string, 0, sizeof(tx_string));

    // Init BaseTimer - TIMERA2
    PRCMPeripheralClkEnable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);
    PRCMPeripheralReset(PRCM_TIMERA2);
    TimerIntRegister(BASE_TIMER,TIMER_A,TimerIntHandler);
    TimerConfigure(BASE_TIMER,TIMER_CFG_PERIODIC);
    TimerIntEnable(BASE_TIMER,TIMER_TIMA_TIMEOUT);
    TimerLoadSet(BASE_TIMER,TIMER_A,5000);
    TimerEnable(BASE_TIMER,TIMER_A);

    // Second timer configurations
    PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralReset(PRCM_TIMERA0);
    TimerIntRegister(SECONDARY_TIMER,TIMER_A,TimerIntHandler2);
    TimerConfigure(SECONDARY_TIMER,TIMER_CFG_PERIODIC);
    TimerIntEnable(SECONDARY_TIMER,TIMER_TIMA_TIMEOUT);
    TimerLoadSet(SECONDARY_TIMER,TIMER_A,MILLISECONDS_TO_TICKS(800));

    int i;

    while(1)
    {
        if(flag == 1){
            // remove the DC bias from the collected samples
            calculateAverageAndUpdateDCBias();

            for (i = 0; i < 8; i++) {
                power_all[i] = goertzel(samples, coeff[i], N);   // call goertzel to calculate the power at each frequency and store it in the power_all array
            }

            post_test(); // call post test function to validate the data and display the pressed digit if applicable
            count = 0; //rest count
            flag = 0;
            // Enable the 16kHz timer -> which enables collection again
            TimerIntClear(BASE_TIMER,TIMER_TIMA_TIMEOUT);
            TimerEnable(BASE_TIMER,TIMER_A);
        }
    }
}

long int goertzel (int sample[], long int coeff, int N) {
    int Q, Q_prev, Q_prev2, i;
    long prod1, prod2, prod3, power;

    Q_prev = 0;           //set delay element1 Q_prev as zero
    Q_prev2 = 0;          //set delay element2 Q_prev2 as zero
    power = 0;            //set power as zero

    for (i = 0; i < N; i++)   // loop N times and calculate Q, Q_prev, Q_prev2 at each iteration
    {
        Q = (sample[i]) + ((coeff * Q_prev) >> 14) - (Q_prev2);   // >>14 used as the coeff was used in Q15 format
        Q_prev2 = Q_prev;     // shuffle delay elements
        Q_prev = Q;
    }

    //calculate the three products used to calculate power
    prod1 = ((long) Q_prev * Q_prev);
    prod2 = ((long) Q_prev2 * Q_prev2);
    prod3 = ((long) Q_prev * coeff) >> 14;
    prod3 = (prod3 * Q_prev2);

    power = (prod1 + prod2 - prod3) >> 8; //calculate power using the three products and scale the result down

    // return the absolute value of the power since power can be negative with the ADC used
    return abs(power);
}

void processString(int i) {
    // loop through the array, and roll back if NULL char reached
    if(array_of_char[i][btn_press_index] == '\0'){
       btn_press_index = 0;
    }
    // draw the char on the OLED
    tx_string[string_index] = array_of_char[i][btn_press_index];
    drawChar(pos_x,pos_y,tx_string[string_index], BLACK, BLACK, txt_size);
    drawChar(pos_x,pos_y,tx_string[string_index], GREEN, BLACK, txt_size);
    pos_x += 6*txt_size;
    l_pos_x = pos_x;
    l_pos_y = pos_y;
    x_ovrflw++;
    if(x_ovrflw == 20){
        pos_x = 6*txt_size;
        pos_y += 10;
        x_ovrflw = 0;
    }
    // increase string index
    string_index++;
}

//-------Post-test function---------------------------------------//
void post_test (void) {
//initialize variables to be used in the function
    int i, row, col;
    long int max_power;

    // find the maximum power in the row frequencies and the row number

    max_power = 0;                //initialize max_power=0

    for (i = 0; i < 4; i++)       //loop 4 times from 0>3 (the indecies of the rows)
    {
        if (power_all[i] > max_power) //if power of the current row frequency > max_power
        {
            max_power = power_all[i]; //set max_power as the current row frequency
            row = i;      //update row number
        }
    }
    // find the maximum power in the column frequencies and the column number

    max_power = 0;        //initialize max_power=0

    for (i = 4; i < 8; i++)   //loop 4 times from 4>7 (the indecies of the columns)
    {
        if (power_all[i] > max_power) //if power of the current column frequency > max_power
        {
            max_power = power_all[i]; //set max_power as the current column frequency
            col = i;      //update column number
        }
    }

    //if the maximum powers equal zero > this means no signal or inter-digit pause
    if (power_all[col] <= NOISE_THRESHOLD && power_all[row] <= NOISE_THRESHOLD)
    new_dig = 1;        //set new_dig to 1 to display the next decoded digit

    // check if maximum powers of row & column exceed certain threshold AND new_dig flag is set to 1
    if ((power_all[col] > NOISE_THRESHOLD && power_all[row] > NOISE_THRESHOLD) && (new_dig == 1))
    {
        new_dig = 0; // set new_dig to 0 to avoid displaying the same digit again.
        // if the same button was pressed
        if(lastRowPressed == row && lastColPressed == col){
            if(!sendDeletePressed){
                // if the same button was pressed, then decrease the index and increase index
                btn_press_index++;
                string_index--;
                // give the user another 700ms to respond
                TimerLoadSet(SECONDARY_TIMER,TIMER_A,MILLISECONDS_TO_TICKS(700));
                TimerEnable(SECONDARY_TIMER, TIMER_A);
                // draw on the OLED
                pos_x -= 6*txt_size;
                l_pos_x = pos_x;
                l_pos_y = pos_y;
                x_ovrflw--;
                if(x_ovrflw == 20){
                    pos_x = l_pos_x;
                    pos_y -= 10;
                    x_ovrflw = 20;
                }
            }
        } else {
            // if a different button was pressed, then disable the timer
            TimerIntClear(SECONDARY_TIMER,TIMER_TIMA_TIMEOUT);
            TimerDisable(SECONDARY_TIMER, TIMER_A);
            // reset some variables that was set when the second timer was active
            btn_press_index = 0;
            lastRowPressed = 10;
            lastColPressed = 10;
            sendDeletePressed = 0;
        }

        // BELOW ARE THE ACTIONS FOR THE CORRESPONDING BUTTON PRESSES
        if(row == 3 && (col - 4) == 0){ // * <--- DELETE
            if(string_index < 1){
                string_index = 0;
            } else {
            sendDeletePressed = 1;
            pos_x -= 6;
            x_ovrflw--;
            // need to work on new line logic!
            if(x_ovrflw < 0 && pos_y > 20){
                pos_y -=10;
            }
            drawChar(pos_x,l_pos_y,tx_string[--string_index], BLACK, BLACK, txt_size);
            tx_string[string_index] = '\0';
            }
        } else if(row == 3 && (col - 4) == 2) { // # <--- SEND
            sendDeletePressed = 1;
            if(string_index < 1 || string_index > 128 ){
                // reset the rest other variables
                Report("Max string length: > 1 and < 128 \n\n\r");
            } else {
                //Connect to the website with TLS encryption
                lRetVal = tls_connect();
                if(lRetVal < 0) {
                    ERR_PRINT(lRetVal);
                    UART_PRINT("TLS_CONNECT_FAILED");
                    LOOP_FOREVER();
                }

                http_get(lRetVal);
                sl_Close(lRetVal);
                ifttt = 1;
                lRetVal = tls_connect();
                if(lRetVal < 0) {
                    ERR_PRINT(lRetVal);
                    UART_PRINT("TLS_CONNECT_FAILED");
                    LOOP_FOREVER();
                }
                http_post(lRetVal);
                //sl_Stop(SL_STOP_TIMEOUT);
                sl_Close(lRetVal);
                ifttt = 0;
                memset(tx_string, 0, sizeof(tx_string));
                string_index = 0;
                fillRect(0, 0, SSD1351WIDTH, 64, BLACK);
                pos_x = 6*txt_size;
                pos_y = 10;
                l_pos_x = pos_x;
                l_pos_y = pos_y;
                x_ovrflw = 0;
            }
        } else if (row == 3 && (col - 4) == 1){ // 0
            processString(0);
        } else if (row == 0 && (col - 4) == 0){ // 1
            processString(1);
        } else if (row == 0 && (col - 4) == 1){ // 2
            processString(2);
        } else if (row == 0 && (col - 4) == 2){ // 3
            processString(3);
        } else if (row == 1 && (col - 4) == 0){ // 4
            processString(4);
        } else if (row == 1 && (col - 4) == 1){ // 5
            processString(5);
        } else if (row == 1 && (col - 4) == 2){ // 6
            processString(6);
        } else if (row == 2 && (col - 4) == 0){ // 7
            processString(7);
        } else if (row == 2 && (col - 4) == 1){ // 8
            processString(8);
        } else if (row == 2 && (col - 4) == 2){ // 9
            processString(9);
        }
        Report("TX_STRING: %s\n\r", tx_string);
        // save the last button pressed
        lastRowPressed = row;
        lastColPressed = col;
    }
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char cCLLength[200];
    char acRecvbuff[2048];
    char* pcBufHeaders;
    int lRetVal = 0;
    char post_buff[64];

    strcpy(post_buff, translated_buffer);

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER_POST);
    pcBufHeaders += strlen(HOSTHEADER_POST);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(DATA_SPLIT1);
    dataLength += strlen(post_buff);
    dataLength += strlen(DATA_SPLIT2);


    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);

     strcpy(pcBufHeaders, CLHEADER1);

     pcBufHeaders += strlen(CLHEADER1);
     sprintf(cCLLength, "%d", dataLength);

     strcpy(pcBufHeaders, cCLLength);
     pcBufHeaders += strlen(cCLLength);
     strcpy(pcBufHeaders, CLHEADER2);
     pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA_SPLIT1);
    pcBufHeaders += strlen(DATA_SPLIT1);

    strcpy(pcBufHeaders, post_buff);
    pcBufHeaders += strlen(post_buff);

    strcpy(pcBufHeaders, DATA_SPLIT2);
    pcBufHeaders += strlen(DATA_SPLIT2);

    int testDataLength = strlen(acSendBuff);

    UART_PRINT("String length: %d \n\n\r", testDataLength);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    Report("SEND HAPPENS RIGHT HERE!\n\n\r");
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

static int http_get(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[2048];
    char* pcBufHeaders;
    int lRetVal = 0;
    char sample[64];

    strcpy(sample, tx_string);

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, sample);
    pcBufHeaders += strlen(sample);
    strcpy(pcBufHeaders, GETHEADER2);
    pcBufHeaders += strlen(GETHEADER2);
    strcpy(pcBufHeaders, HOSTHEADER_GET);
    pcBufHeaders += strlen(HOSTHEADER_GET);
    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);
    UART_PRINT("TX_SIZE: %d\n\n\r", strlen(tx_string));

    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("GET failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    } else {
        acRecvbuff[lRetVal+1] = '\0';
        Report("RECEIVE HAPPENS RIGHT HERE!\n\n\r");
        //UART_PRINT(acRecvbuff);
        //UART_PRINT("\n\r\n\r");

        int buff_len = strlen(acRecvbuff);
        int cnt_prn = 0;
        int cnt_aps = 0;
        int t_buf_idx = 0;
        int begin_cp = 0;

        memset(translated_buffer, 0, sizeof(translated_buffer));

        // Parse the JSON data
        int k = 0;
        while(k < buff_len){
            if(begin_cp == 1){
                if(acRecvbuff[k] == '"'){
                    break;
                }
                translated_buffer[t_buf_idx++] = acRecvbuff[k];
            }
            if(acRecvbuff[k] == '{'){
                if(cnt_prn < 3){
                    cnt_prn++;
                }
            }
            if(cnt_prn == 3){
                if(cnt_aps < 3 && acRecvbuff[k] == '"'){
                    cnt_aps++;
                }
            }
            if(cnt_aps == 3){
                begin_cp = 1;
            }
            k++;
        }

        UART_PRINT("TRANSLATED BUFFER: %s\n\r", translated_buffer);

        k = 0;
        int rx_x = 6;
        int rx_y = 65;
        fillRect(0, rx_y, SSD1351WIDTH, 64, BLACK);

        int len_str = strlen(translated_buffer);
        while(k < len_str){
            drawChar(rx_x,rx_y,translated_buffer[k++], RED, BLACK, 1);
            rx_x += 6;
        }

    }

    return 0;
}
