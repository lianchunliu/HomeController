/**************************************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 15 seconds.  The application will also
  receive "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#include "user_printf.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "OSAL_Nv.h"

/*********************************************************************
 * MACROS
 */
#define HAL_LED_OFF   1
#define HAL_LED_ON    0


#define DATA_INPUT P0_6

static uint16 lightShortAddr=0x046A;

// This is the max byte count per OTA message.
#if !defined( SERIAL_APP_TX_MAX )
#define SERIAL_APP_TX_MAX  80
#endif

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */


static uint16 lightOffTimeValue = 0;
static uint16 currentTime = 0;
static uint8 isLightController = 0;

// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;
byte RxBuf[SERIAL_APP_TX_MAX+1];
static uint8 SerialApp_TxLen;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void GenericApp_HandleKeys( byte shift, byte keys );
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void GenericApp_SendTheMessage( void );
void rxCB(uint8 port,uint8 event);
static uint16 strToUint16(uint8* str);
static void processLightToggle(afIncomingMSGPacket_t *inMsg);
static void sendSendCmdMsg(uint8* cmd);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( byte task_id )
{
  halUARTCfg_t uartConfig;
  
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  GenericApp_DstAddr.endPoint = 0;
  GenericApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );
  
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 64;   // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 6;    // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE; // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = rxCB;
  HalUARTOpen (0, &uartConfig); 
  
  
  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif
    
  //ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  //ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
    
    
  HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
  
  if (strStartWith((uint8*)getMyName(), "DoorSensor")) {
    isLightController = 0;
    lightShortAddr=0x046A;
  } else if (strStartWith((uint8*)getMyName(), "RestroomSensor")) {
    isLightController = 0;
    lightShortAddr=0x2DC0;
  } else {
    isLightController = 1;
  }
  
  printf("GenericApp_Init[%s] Done!\n", getMyName());
  
}

static void resetCurrentTime(void);

void resetCurrentTime(void)
{
  currentTime = 0;
  lightOffTimeValue = 0;
}
/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
          
        case KEY_CHANGE:
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD)
              || (GenericApp_NwkState == DEV_ROUTER)
              || (GenericApp_NwkState == DEV_END_DEVICE) )
          {
            if (isLightController) {
              HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
            }
          //  byte* extAddr = NLME_GetExtAddr();
            uint16 shortAddr = NLME_GetShortAddr();
            uint16 parentShortAddr = NLME_GetCoordShortAddr();
            
            printf("Addr=%04X, ParentAddr=%04X\n", shortAddr, parentShortAddr);
            
            // Start sending "the" message in a regular interval.
           // osal_start_timerEx( GenericApp_TaskID,
           //                     GENERICAPP_SEND_MSG_EVT,
           //                   GENERICAPP_SEND_MSG_TIMEOUT );
            osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_LIGHT_OFF_LATER_EVT,
                      GENERICAPP_LIGHT_OFF_LATER_TIMEOUT );
          }
          
          
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in GenericApp_Init()).
  if ( events & GENERICAPP_SEND_MSG_EVT )
  {
    // Send "the" message
    GenericApp_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_SEND_MSG_EVT,
                      GENERICAPP_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ GENERICAPP_SEND_MSG_EVT);
  }
  
   if ( events & GENERICAPP_LIGHT_OFF_LATER_EVT )
  {
    
    currentTime++;
    
    if (lightOffTimeValue > 0 && currentTime > lightOffTimeValue) {
      
      printf("current : %d, LightOffTimeValue: %d\n", currentTime, lightOffTimeValue);
      
      HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
      
      resetCurrentTime();
      
    }

    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_LIGHT_OFF_LATER_EVT,
                      GENERICAPP_LIGHT_OFF_LATER_TIMEOUT );
    
    // return unprocessed events
    return (events ^ GENERICAPP_LIGHT_OFF_LATER_EVT);
  }

  // Discard unknown events
  return 0;
}

void processLightToggle(afIncomingMSGPacket_t *inMsg)
{
  uint16 shortAddr = NLME_GetShortAddr();
  uint16 parentShortAddr = NLME_GetCoordShortAddr();
  char buf[48];
  int len = sprintf(buf, "LightToggleResp %s %04X %04X", getMyName(), shortAddr, parentShortAddr);
  
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = inMsg->srcAddr.addr.shortAddr;
  
  HalLedSet ( HAL_LED_1, HAL_LED_MODE_TOGGLE );
  
  // disable light auto off
  resetCurrentTime();
  
  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       osal_strlen(buf)+1,
                       (uint8*)buf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    printf("send ReportNamesResp error %s\n", buf);
  }
  
}


void processLightOn(afIncomingMSGPacket_t *inMsg)
{
  uint16 shortAddr = NLME_GetShortAddr();
  uint16 parentShortAddr = NLME_GetCoordShortAddr();
  char buf[48];
  int len = sprintf(buf, "LightOnResp %s %04X %04X", getMyName(), shortAddr, parentShortAddr);
  
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = inMsg->srcAddr.addr.shortAddr;
  
  HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
  
  // disable light auto off
  resetCurrentTime();
  
  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       osal_strlen(buf)+1,
                       (uint8*)buf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    printf("send ReportNamesResp error %s\n", buf);
  }
  
}

static void processLightOffLater(afIncomingMSGPacket_t *inMsg);

void processLightOffLater(afIncomingMSGPacket_t *pkt)
{
  uint16 shortAddr = NLME_GetShortAddr();
  uint16 parentShortAddr = NLME_GetCoordShortAddr();
  char buf[48];
  int len = sprintf(buf, "LightOffLaterResp %s %04X %04X", getMyName(), shortAddr, parentShortAddr);
  
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
  
  uint8* cmdData = pkt->cmd.Data;
  
  cmdData += osal_strlen("LightOffLater ");
  
  uint16 plusTime = strToUint16(cmdData);
  
  resetCurrentTime();
  
  lightOffTimeValue = currentTime + plusTime;
  
  printf("plusTime %d, LightOffTimeValue: %d, currentTime=%d\n", plusTime, lightOffTimeValue, currentTime);
  
  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       osal_strlen(buf)+1,
                       (uint8*)buf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    printf("send ReportNamesResp error %s\n", buf);
  }
  
}


static void processLightOff(afIncomingMSGPacket_t *inMsg);

void processLightOff(afIncomingMSGPacket_t *inMsg)
{
  uint16 shortAddr = NLME_GetShortAddr();
  uint16 parentShortAddr = NLME_GetCoordShortAddr();
  char buf[48];
  int len = sprintf(buf, "LightOffResp %s %04X %04X", getMyName(), shortAddr, parentShortAddr);
  
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = inMsg->srcAddr.addr.shortAddr;
  
  HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
  
  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       osal_strlen(buf)+1,
                       (uint8*)buf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    printf("send ReportNamesResp error %s\n", buf);
  }
  
}


void processReportNames(afIncomingMSGPacket_t *inMsg)
{
  uint16 shortAddr = NLME_GetShortAddr();
  uint16 parentShortAddr = NLME_GetCoordShortAddr();
  char buf[48];
  int len = sprintf(buf, "ReportNamesResp %s %04X %04X", getMyName(), shortAddr, parentShortAddr);
  
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = inMsg->srcAddr.addr.shortAddr;
  
    
  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       osal_strlen(buf)+1,
                       (uint8*)buf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    printf("send ReportNamesResp error %s\n", buf);
  }
  
}
/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

void turnOffLightLater(void);
void turnOnLight(void);

void turnOffLightLater(void)
{
        char buf[48];
        sprintf(buf, "%04X LightOffLater 0040", lightShortAddr);    
        sendSendCmdMsg((uint8*)buf);
  
}

void turnOnLight(void)
{
          char buf[48];
         sprintf(buf, "%04X LightOn", lightShortAddr);
    
         sendSendCmdMsg((uint8*)buf);
 
  
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void GenericApp_HandleKeys( byte shift, byte keys )
{
    if ( keys & HAL_KEY_SW_1 )
    {
      printf("S1 pressed\n");
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      printf("S2 pressed\n");
    }
    
    if (!isLightController) {
      
      uint8* myName = (uint8*)getMyName();
      
      if (keys & HAL_KEY_DATA_LOW) {
        printf("DataLow pressed\n");
        if (strStartWith(myName, "DoorSensor")) {
          turnOffLightLater();    
        }
        
        
        if (strStartWith(myName, "RestroomSensor")) {
          turnOnLight();
        }
      }
      
      if (keys & HAL_KEY_DATA_HIGH) {
         printf("DataHigh pressed\n");
        if (strStartWith(myName, "DoorSensor")) {
          turnOnLight();
        }
        
         if (strStartWith(myName, "RestroomSensor")) {
          turnOffLightLater();
        }
      }    
    }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8 strStartWith(uint8* str1, uint8* str2)
{
  while (*str2) {
    if (*str1 != *str2) {
      return 0;
    }
    str1++;
    str2++;
  }
  return 1;
}

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
     
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
      
      printf("process Cmd: %s\n", pkt->cmd.Data);
      
      uint8* cmdData = pkt->cmd.Data;
      
      if (strStartWith(cmdData, "ReportNamesResp")) {
        // nothing to do
        
      } else if (strStartWith(cmdData, "ReportNames")) {
         processReportNames(pkt);
      } else if (strStartWith(cmdData, "LightOnResp")) {
        // nothing to do
      } else if (strStartWith(cmdData, "LightOn")) {
         processLightOn(pkt);
      } else if (strStartWith(cmdData, "LightOffLaterResp")) {
        // nothing to do
      } else if (strStartWith(cmdData, "LightOffLater")) { // LightOffLater 0040
         processLightOffLater(pkt);
      } else if (strStartWith(cmdData, "LightToggleResp")) {
        // nothing to do
      } else if (strStartWith(cmdData, "LightToggle")) {
         processLightToggle(pkt);
      } else if (strStartWith(cmdData, "LightOffResp")) {
        // nothing to do
      } else if (strStartWith(cmdData, "LightOff")) {
         processLightOff(pkt);
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_SendTheMessage( void )
{
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = 0xFFFF; //终端短地址在LCD上有显示，此处换成终端短地址就可以点播了。

  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       1,
                       RxBuf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}

void sendReportNamesMsg(void)
{
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = 0xFFFF; //终端短地址在LCD上有显示，此处换成终端短地址就可以点播了。
  char cmd[] = "ReportNames";
    
  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       osal_strlen(cmd)+1,
                       (uint8*)cmd,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    printf("send ReportNames error\n");
  }
}

static uint16 findHexValue(uint8 ch);

uint16 findHexValue(uint8 ch)
{
  switch(ch) {
  case '0':
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
  case '7':
  case '8':
  case '9':
    return ch - '0';
  case 'A':
  case 'B':
  case 'C':
  case 'D':
  case 'E':
  case 'F':
    return ch - 'A' + 10;
  case 'a':
  case 'b':
  case 'c':
  case 'd':
  case 'e':
  case 'f':
    return ch - 'a' + 10;
  }
  return 0;
}



uint16 strToUint16(uint8* str)
{
    return findHexValue(str[0])*16*16*16 + findHexValue(str[1])*16*16 + findHexValue(str[2])*16 + findHexValue(str[3]);
}


void sendSendCmdMsg(uint8* cmd)
{
  
  uint16 shortAddr = strToUint16(cmd);
  cmd += 5;
  
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = shortAddr; //终端短地址在LCD上有显示，此处换成终端短地址就可以点播了。
  
  printf("sendCmd: addr=%04X, cmd=%s\n", shortAddr, cmd);
  
  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       osal_strlen((char*)cmd)+1,
                       (uint8*)cmd,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    printf("send ReportNames error\n");
  }
}


#define MY_UART_BUF_SIZE 64
#define MY_NAME_NV_ID 0x2701

char myName[MY_UART_BUF_SIZE];

static char* getMyName()
{
    if (osal_nv_read(MY_NAME_NV_ID, 0, MY_UART_BUF_SIZE, myName ) == ZSUCCESS) {
      
    } else {
      myName[0] = 0; 
    }
    
    return myName;
}

static void processUartCB(uint8* buf, uint8 len)
{
  uint16 id;
  uint16 itemLen;
  //TODO disable overflow
  buf[len] = 0;
  uint8 myBuf[MY_UART_BUF_SIZE];
  
  if (len >= MY_UART_BUF_SIZE || len < 1) {
    printf("invalid buf length: %d\n", len);
    return; 
  }
  
  printf("processUartCB: %s\n", buf);
  
  id = MY_NAME_NV_ID; // for name
  if (strStartWith(buf, "RNV Name")) {
    
    itemLen = osal_nv_item_len(id);
    if (itemLen > MY_UART_BUF_SIZE || itemLen == 0) {
      printf("invalid nv item len: %d\n", itemLen);
      return;
    }
    
    if (osal_nv_read(id, 0, itemLen, myBuf ) == ZSUCCESS) {
      myBuf[itemLen-1] = 0;
      printf("Name = %s\n", myBuf);
    }
    
  } else if (strStartWith(buf, "WNV Name ")) {
    
    osal_memcpy(myBuf, buf,  len);
    myBuf[len] = 0;
    osal_nv_item_init(id, MY_UART_BUF_SIZE, NULL);
    osal_nv_write(id, 0, len-8, myBuf+9);
    printf("receive cmd: %s\n", myBuf);
  } else if (strStartWith(buf, "ReportNames")) {
    
    sendReportNamesMsg();
  } else  if (strStartWith(buf, "sendCmd ")) {
    sendSendCmdMsg(buf + 8);
  } else {
    buf[len-1] = 0;
    printf("unkown cmd: %s\n", buf); 
    
  }
}


static void rxCB(uint8 port,uint8 event)
{
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SERIAL_APP_LOOPBACK
      (SerialApp_TxLen < SERIAL_APP_TX_MAX))
#else
      !SerialApp_TxLen)
#endif
  {
    SerialApp_TxLen = HalUARTRead(0, RxBuf, SERIAL_APP_TX_MAX);
    if (SerialApp_TxLen)
    {
      
      processUartCB(RxBuf, SerialApp_TxLen);
        
      //HalUARTWrite(0, RxBuf, SerialApp_TxLen);
      //GenericApp_SendTheMessage();
      
      SerialApp_TxLen=0;
    }
  }
}
/*********************************************************************
*********************************************************************/
