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
#define LIGHT_NUM 4


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


static uint16 lightOffTimeValue[LIGHT_NUM];
static uint16 currentTime[LIGHT_NUM];

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

static void sendSendCmdMsg(uint8* cmd);
uint16 strToDec(uint8 * str);

void turnOffLightLater(uint16 addr, uint8 lightNum);
void turnOnLight(uint16 addr, uint8 lightNum);
void lightToggle(uint16 addr, uint8 lightNum);
void processLightClick(afIncomingMSGPacket_t *pkt);

void setLightMode(uint8 lightNum, uint8 mode);

void processLightStatus(afIncomingMSGPacket_t *pkt);
void processLightOn(afIncomingMSGPacket_t *pkt);
void processLightOff(afIncomingMSGPacket_t *pkt);
void processLightToggle(afIncomingMSGPacket_t *inMsg);
uint16 strToDec(uint8 * str);

static void processLightOffLater(afIncomingMSGPacket_t *inMsg);


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
   
    for (int i = 0; i < LIGHT_NUM; i++) {
      
      lightOffTimeValue[i] = 0;
      currentTime[i] = 0;
    }
    
  // turn on all light by default
  HalLedSet( HAL_LED_ALL, HAL_LED_MODE_ON );
  
#if ZG_BUILD_ENDDEVICE_TYPE
  
  P0INP  &= ~0X10;   //设置P0口输入电路模式为上拉/ 下拉
        P0IEN |= 0X10;     //P01设置为中断方式 
        PICTL |= 0X10;     // 下降沿触发     
   IEN1 |= 0X20;      //  开P0口总中断 
        P0IFG |= 0x00;     //清中断标志
	EA = 1; 
#endif
        
  printf("GenericApp_Init[%s] Done!\n", getMyName());
  
}

static void resetCurrentTime(uint8 lightNumber);

void resetCurrentTime(uint8 lightNumber)
{
  currentTime[lightNumber] = 0;
  lightOffTimeValue[lightNumber] = 0;
}

void SysPowerMode(uint8 mode);

void SysPowerMode(uint8 mode) 
{ 
 uint8 i,j; 
 i = mode; 
 if(mode<4) 
 {  
    SLEEPCMD |= i;     // 设置系统睡眠模式 
  for(j=0;j<4;j++); 
    PCON = 0x01;         // 进入睡眠模式 ,通过中断打断
  } 
 else 
 { 
      PCON = 0x00;             // 系统唤醒 ，通过中断打断
 } 
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
              
            
            if (strEqual("AirCond", getMyName())) {
              HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF);
            } else {
              HalLedSet ( HAL_LED_ALL, HAL_LED_MODE_ON );
            }
           
          //  byte* extAddr = NLME_GetExtAddr();
            uint16 shortAddr = NLME_GetShortAddr();
            uint16 parentShortAddr = NLME_GetCoordShortAddr();
            
            printf("Addr=%04X, ParentAddr=%04X\n", shortAddr, parentShortAddr);
            
            osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_LIGHT_OFF_LATER_EVT,
                      GENERICAPP_LIGHT_OFF_LATER_TIMEOUT );
            
            if (GenericApp_NwkState == DEV_END_DEVICE) {
               osal_start_timerEx( GenericApp_TaskID,
                            GENERICAPP_SLEEP_LATER_EVT,
                          1000 );
            }
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
    
      for (uint8 i = 0; i < LIGHT_NUM; i++) {
        currentTime[i]++;
          
        if (lightOffTimeValue[i] > 0 && currentTime[i] > lightOffTimeValue[i]) {
          
          printf("current[%d] : %d, LightOffTimeValue[%d]: %d\n", i, currentTime[i], i, lightOffTimeValue[i]);
          
          setLightMode(i, HAL_LED_MODE_OFF);
          
          resetCurrentTime(i);
          
        }
      }
  
      osal_start_timerEx( GenericApp_TaskID,
                          GENERICAPP_LIGHT_OFF_LATER_EVT,
                        GENERICAPP_LIGHT_OFF_LATER_TIMEOUT );
      
      // return unprocessed events
      return (events ^ GENERICAPP_LIGHT_OFF_LATER_EVT);
  }
  
  if ( events & GENERICAPP_SLEEP_LATER_EVT )
  {
    
     static int n = 1;
     
     printf("awake[%d]...\n", n);
     
     if (n++ % 10 == 0) {
       printf("sleep now\n");
      SysPowerMode(3);
     }
  
      osal_start_timerEx( GenericApp_TaskID,
                          GENERICAPP_SLEEP_LATER_EVT,
                        1000 );
      
      // return unprocessed events
      return (events ^ GENERICAPP_SLEEP_LATER_EVT);
  }

  // Discard unknown events
  return 0;
}

void setLightMode(uint8 lightNum, uint8 mode)
{
  uint8 led = 0;
  switch(lightNum) {
  case 0: led = HAL_LED_1; break;
  case 1: led = HAL_LED_2; break;
  case 2: led = HAL_LED_3; break;
  case 3: led = HAL_LED_4; break;
  default:
    printf("invalid lightNumber: %d\n", lightNum);
    return;
  }
  
  if (mode == HAL_LED_MODE_BLINK) {
      //printf("start blink\n");
      
      HalLedBlink (led, 1, 50, 1000);
  } else {
    HalLedSet ( led, mode);
  }
}



char* getKeyStatus(uint8 keyNum);

char* getKeyStatus(uint8 keyNum)
{
  
  if (HalKeyRead() & HAL_KEY_3_HIGH) {
    return "Off";
  } else {
    return "On";
  }
   
}


char* getLightStatus(uint8 lightNum);

char* getLightStatus(uint8 lightNum)
{
  uint8 led = 0;
  switch(lightNum) {
  case 0: led = HAL_LED_1; break;
  case 1: led = HAL_LED_2; break;
  case 2: led = HAL_LED_3; break;
  case 3: led = HAL_LED_4; break;
  default:
    printf("invalid lightNumber: %d\n", lightNum);
    return "Off";
  }
  
  if (HalLedGetState() & led) {
    return "On";
  } else {
    return "Off";
  }
   
}

void processLightCmd(afIncomingMSGPacket_t *pkt, char* cmd, uint8 mode);

void processLightToggle(afIncomingMSGPacket_t *pkt)
{
  processLightCmd(pkt, "LightToggle", HAL_LED_MODE_TOGGLE);
}

void processLightOn(afIncomingMSGPacket_t *pkt)
{
  processLightCmd(pkt, "LightOn", HAL_LED_MODE_ON);
}

void processLightOff(afIncomingMSGPacket_t *pkt)
{
  processLightCmd(pkt, "LightOff", HAL_LED_MODE_OFF);
}

void processLightStatus(afIncomingMSGPacket_t *pkt)
{
  processLightCmd(pkt, "LightStatus", 0);
}

void processLightOffLater(afIncomingMSGPacket_t *pkt)
{
  processLightCmd(pkt, "LightOffLater", 0);  
}

void processLightClick(afIncomingMSGPacket_t *pkt)
{
  processLightCmd(pkt, "LightClick", HAL_LED_MODE_BLINK);  
}

void processLightCmd(afIncomingMSGPacket_t *pkt, char* cmd, uint8 mode)
{  
  uint8* cmdData = pkt->cmd.Data;
  if (osal_strlen((char*)cmdData) < osal_strlen(cmd) + 2) {// cmd lightNumber
    printf("invalid request: %s\n", cmdData);
    return;
  }
  
  uint8 lightNum = cmdData[osal_strlen(cmd) + 1] - '0';
  if (lightNum < 0 || lightNum >= LIGHT_NUM) {
    printf("invalid request: %s\n", cmdData);
    return;
  }
  
  
  char buf[48];
  
  uint16 shortAddr = NLME_GetShortAddr();
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
  
  
  
  if (strEqual((char*)cmd, "LightStatus")) {
    if (strEqual("AirCond", getMyName())) {
      
      
      sprintf(buf, "%sResp %d %s %04X %s", cmd, lightNum, getKeyStatus(HAL_KEY_3_HIGH), shortAddr, getMyName());
    } else {
      sprintf(buf, "%sResp %d %s %04X %s", cmd, lightNum, getLightStatus(lightNum), shortAddr, getMyName());  
    }
    
    
  } else if (strEqual((char*)cmd, "LightOffLater")) {
    
    uint16 delayTime = 0;
    delayTime = strToDec(cmdData + osal_strlen("LightOffLater 1 "));
    
    resetCurrentTime(lightNum);
  
    lightOffTimeValue[lightNum] = currentTime[lightNum] + delayTime;
  
   // printf("delayTime %d, LightOffTimeValue: %d, currentTime=%d, lightNum=%d\n", delayTime, lightOffTimeValue, currentTime, lightNum);
  
    sprintf(buf, "%sResp %d %04X %s", cmd, lightNum, shortAddr, getMyName());
    
  } else {
    setLightMode(lightNum, mode);
    // disable light auto off
    resetCurrentTime(lightNum);
    
    sprintf(buf, "%sResp %d %04X %s", cmd, lightNum, shortAddr, getMyName());
  }
  
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
    printf("send %sResp %d error %s\n", cmd, lightNum, buf);
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
//        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
 //       HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
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
//            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}


void turnOffLightLater(uint16 addr, uint8 lightNum)
{
  char buf[48];
  sprintf(buf, "%04X LightOffLater %d 60", addr, lightNum);
  sendSendCmdMsg((uint8*)buf);
}

void turnOnLight(uint16 addr, uint8 lightNum)
{
  char buf[48];
  sprintf(buf, "%04X LightOn %d", addr, lightNum);
  sendSendCmdMsg((uint8*)buf);
}

void lightToggle(uint16 addr, uint8 lightNum)
{
  char buf[48];
  sprintf(buf, "%04X LightToggle %d", addr, lightNum);
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
  /*************************** logger ***********************************/
  if (keys & HAL_KEY_1_LOW) printf("HandleKeys: HAL_KEY_1_LOW\n");
  if (keys & HAL_KEY_2_LOW) printf("HandleKeys: HAL_KEY_2_LOW\n");
  if (keys & HAL_KEY_3_LOW) printf("HandleKeys: HAL_KEY_3_LOW\n");
  if (keys & HAL_KEY_4_LOW) printf("HandleKeys: HAL_KEY_4_LOW\n");
  if (keys & HAL_KEY_1_HIGH) printf("HandleKeys: HAL_KEY_1_HIGH\n");
  if (keys & HAL_KEY_2_HIGH) printf("HandleKeys: HAL_KEY_2_HIGH\n");
  if (keys & HAL_KEY_3_HIGH) printf("HandleKeys: HAL_KEY_3_HIGH\n");
  if (keys & HAL_KEY_4_HIGH) printf("HandleKeys: HAL_KEY_4_HIGH\n");
  
   
 /************************* DoorSensor ***************************/
  uint8* myName = (uint8*)getMyName();
  if (strEqual((char*)myName, "DoorSensor")) {
    if (keys & HAL_KEY_3_LOW) {
      turnOffLightLater(0x046A, 0);
    }
    
    if (keys & HAL_KEY_3_HIGH) {
      turnOnLight(0x046A, 0);
    }
  } // 
  
/************************ RestroomSensor ****************************/
  if (strEqual((char*)myName, "RestroomSensor")) {
    if (keys & HAL_KEY_3_LOW) {
      turnOnLight(0x2DC0,0);
    }
    
    if (keys & HAL_KEY_3_HIGH) {
      turnOffLightLater(0x2DC0, 0);
    }
    
  } // 
/*********************** MainLight ******************************/
  
  if (strEqual((char*)myName, "MainLight")) {
    if (keys & HAL_KEY_1_LOW || keys & HAL_KEY_1_HIGH) {
      lightToggle(0x2DC0, 0);
    }
    
    if (keys & HAL_KEY_2_LOW || keys & HAL_KEY_2_HIGH) {
      lightToggle(0x2DC0, 1);
    }
    
    if (keys & HAL_KEY_3_LOW || keys & HAL_KEY_3_HIGH) {
      lightToggle(0x2DC0, 2);
    }
    
  }
  /************************ BedLight *******************************/
  if (strEqual((char*)myName, "BedLight")) {
    if (keys & HAL_KEY_1_LOW) {
      
      SysPowerMode(3);
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

uint8 strEqual(char* str1, char* str2)
{
  while (*str2 && *str1) {
    if (*str1 != *str2) {
      return 0;
    }
    str1++;
    str2++;
  }
  if (*str1 == *str2) {
    return 1;
  } else {
    return 0;
  }
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
      } else if (strStartWith(cmdData, "LightStatusResp")) {
        // nothing to do
      } else if (strStartWith(cmdData, "LightStatus")) {
         processLightStatus(pkt);
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
      } else if (strStartWith(cmdData, "LightClickResp")) {
        // nothing to do
      } else if (strStartWith(cmdData, "LightClick")) {
         processLightClick(pkt);
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

uint16 strToDec(uint8 * str)
{
  uint16 ret = 0;
  while (*str && (*str != ' ')) {
    ret = ret*10 + (*str - '0');
    str++;
  }
  return ret;
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

char* getMyName()
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
  
 // printf("processUartCB: %s\n", buf);
  
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
