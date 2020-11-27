/*
   Fantail Flybarless.
   Electronic stabilization for radio controlled helicopters.
   #Arduino #FantailFlybarless #KeepRcHelisAlive

   Version: 0.12.0
   Date: November 28, 2020
   Author: Cassandra "ZZ Cat" Robinson.

   Copyright © 2020, Cassandra "ZZ Cat" Robinson. All rights reserved.

   Fantail Flybarless is free software: You can redistribute &/or modify it under the terms of the GNU General Public License as published by
   The Free Software Foundation, either version 3 of the license, or any later version.

   Fantail Flybarless is distributed in the hope that it will be useful, but WITHOUT WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with Fantail Flybarless. If not, see https://www.gnu.org/licenses/.

   Follow the project on social media:
    • Facebook: https://www.facebook.com/hashtag/fantailflybarless
    • Instagram: [COMING SOON]
    • Twitter: [COMING SOON]
    • YouTube: [COMING SOON]

   Get the source code & build your own:
    • GitHub: https://bit.ly/FantailFlybarless
              https://bit.ly/FantailSource

*/

// Arduino API is required.
#if defined( Arduino_h )

// Project definitions - For data logging & debugging purposes.
#define __PROJECT_NAME__        "Fantail Flybarless"
#define __PROJECT_BUILD_DATE__  "November 28, 2020"
#define __PROJECT_VERSION__     "0.12.0"

// Compilation Target Dev Board: Adafruit Feather M0.
#if defined( ADAFRUIT_FEATHER_M0 )

#define REFRESH_INTERVAL 20000

// Include all mandatory APIs.
#include "FreeRTOS_Arduino.h" // ◄-- FreeRTOS version 10.4.1 API integration.
#include "ServoEasing.h"      // ◄-- Servo Easing version 2.3.2 API integration. This is a valid servo driver API.

// CCPM Servos' settings.
const uint16_t usCCPMPulseWidth   = 1500;
const uint16_t usCCPMMinPulse     = usCCPMPulseWidth / 2;
const uint16_t usCCPMMaxPulse     = usCCPMPulseWidth + usCCPMMinPulse;
const uint16_t usCCPMMinRange     = 0;
const uint16_t usCCPMMaxRange     = 145;
const int8_t   cCCPMMix           = 40;

// Rudder Servo settings.
const uint16_t usRudPulseWidth    = 760;
const uint16_t usRudMinPulse      = usRudPulseWidth / 2;
const uint16_t usRudMaxPulse      = usRudPulseWidth + usRudMinPulse;
const uint16_t usRudMinRange      = 0;
const uint16_t usRudMaxRange      = 180;
const int8_t   cRudMix            = 50;

// ESC Throttle settings.
const uint16_t usEscMinPulse      = 1100;
const uint16_t usEscMaxPulse      = 1940;
const uint16_t usEscMinRange      = 0;
const uint16_t usEscMaxRange      = 100;
const int8_t   cEscMix            = 100;

// Channel ID
enum {
  ePwmAil,
  ePwmEle,
  ePwmPit,
  ePwmRud,
  ePwmEsc,
  ePwmChannelsCount
} PwmChannels_t;

// Pin Connections - IE What servos are connected to what PCA9685 pins?
const int16_t psPwmPins[ ePwmChannelsCount ] = {
  4,  // ◄--Aileron Servo Pin.
  5,  // ◄--Elevator Servo Pin.
  6,  // ◄--Pitch Servo Pin.
  3,  // ◄--Rudder Servo Pin.
  7   // ◄--ESC Throttle Pin.
};

QueueHandle_t xPidDataQueue;
QueueHandle_t xServoDataQueue;
QueueHandle_t xExceptionDataQueue;
TaskHandle_t xPidControlLoopHandle;
TaskHandle_t xTaskManagerHandle;

typedef enum {
  eReceiver,
  eImu,
  ePidDataSource_count
} PidDataSource_t;

typedef struct {
  float pfBuffer[ 9 ];
  PidDataSource_t eDataSource;
} PidData_t;

typedef enum {
  ePIDControlLoop,
  eServoDataSource_count
} ServoDataSource_t;

typedef struct {
  int16_t psBuffer[ 5 ];
  ServoDataSource_t eDataSource;
} ServoData_t;

typedef struct {
  TaskHandle_t xTskHandle;
  const char *pcTaskName;
  uint32_t ulID;
  const char *pcMsg;
} ExceptionData_t;

// Firmware initializer.
int main( void ) {

  vBoardInit( 1 );

  Wire.begin();

  vFreeRtosInit();
  vFreeRtosRun();

}

/*----------------------------------------------------------------------------------------------------------------------------------------------------*/
/*                                                            F R E E R T O S    T A S K S                                                            */
/*----------------------------------------------------------------------------------------------------------------------------------------------------*/
#if tskKERNEL_VERSION_MAJOR < 10 || tskKERNEL_VERSION_MINOR < 4 || tskKERNEL_VERSION_BUILD < 1
#error "Fantail Flybarless requires FreeRTOS, version 10.4.1 or later. Update the FreeRTOS API to the latest version."
#endif
/*
    @Task:      vReceiverInterface( void *pvParameters )
    @Desc:      A provisional task, what will be used in future updates to read data packets from a receiver & convert packet data to control data.
    @Param:     None.
*/
void vReceiverInterface( void *pvParameters ) {

  const char *pcName = pcTaskGetName( NULL );

  vTaskSuspend( NULL );

  while ( 1 ) {

    /**/

  }
}

/*
    @Task:      vImu( void *pvParameters )
    @Desc:      A provisional task, what sends dummy IMU "Data" to a queue.
    @Param:     None.
*/
void vImu( void *pvParameters ) {

  const char *pcName = pcTaskGetName( NULL );

  PidData_t xImuData;
  const TickType_t xPollRate = pdMS_TO_TICKS( 10 );
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while ( 1 ) {

    // Polling rate.
    vTaskDelayUntil( &xLastWakeTime, xPollRate );

    // [TODO]: Get IMU data.

    // Fill the buffer with IMU data.
    xImuData.eDataSource = eImu;
    xImuData.pfBuffer[ 0 ] = 0.0;
    xImuData.pfBuffer[ 1 ] = 0.0;
    xImuData.pfBuffer[ 2 ] = 0.0;

    if ( uxQueueSpacesAvailable( xPidDataQueue ) > 0 ) {

      if ( xQueueSendToBack( xPidDataQueue, &xImuData, 0 ) != pdPASS ) {

        vPrint( ( String ) "[" + pcName + " | ERROR]: PID Data Queue is already full.\r\n" );

      }
    }
  }
}

/*
    @Task:      vPidControlLoop( void *pvParameters )
    @Desc:      A provisional task, what reads dummy setpoint & feedback "data" from one queue, & sends dummy PID process variable "data" to another
                queue. Also, if one item in the process variable's buffer is above a pre-defined value, an exception is triggered, which prints an error
                to the Terminal & stops this task from executing.
    @Param:     None.
*/
void vPidControlLoop( void *pvParameters ) {

  const char *pcName = pcTaskGetName( NULL );

  PidData_t xPidData;
  ServoData_t xControlData;

  while ( 1 ) {

    do {

      if ( xQueueReceive( xPidDataQueue, &xPidData, pdMS_TO_TICKS( 20 ) ) == pdPASS ) {

        switch ( xPidData.eDataSource ) {

          case eReceiver:
            // [TODO]: Get Control data from Receiver.
            break;

          case eImu:
            // [TODO]: Get raw Heading, Pitch & Bank data from the IMU Sensor.
            break;

        }
      }

      else {

        // [TODO]: Possible error handling & failsafe?
        vPrint( ( String ) "[" + pcName + " | ERROR]: No data was received.\r\n" );

      }
    } while ( uxQueueMessagesWaiting( xPidDataQueue ) > 0 );

    // [TODO]: Calculate PID algorithms here.

    // Fill the buffer up with Process Variable data.
    xControlData.eDataSource = ePIDControlLoop;
    xControlData.psBuffer[ 0 ] = 0; // Process Variable - Collective Pitch
    xControlData.psBuffer[ 1 ] = 0; // Process Variable - Tail/Counter-Torque Pitch
    xControlData.psBuffer[ 2 ] = 0; // Process Variable - Fore/Aft Cyclic Pitch
    xControlData.psBuffer[ 3 ] = 0; // Process Variable - Left/Right Cyclic Pitch

    // [WARNING]: ESC THROTTLE MUST BE LOCKED AT -2048, TO PREVENT THE HELICOPTER'S POWER TRAIN (& BY EXTENSION, MAIN ROTOR) FROM SPOOLING UP!!!
    // [WARNING]: ESC THROTTLE MUST BE LOCKED AT -2048, TO PREVENT THE HELICOPTER'S POWER TRAIN (& BY EXTENSION, MAIN ROTOR) FROM SPOOLING UP!!!
    // [WARNING]: ESC THROTTLE MUST BE LOCKED AT -2048, TO PREVENT THE HELICOPTER'S POWER TRAIN (& BY EXTENSION, MAIN ROTOR) FROM SPOOLING UP!!!
    xControlData.psBuffer[ 4 ] = -2048; // Process Variable - ESC Throttle

    // [WARNING]: THIS IS A SAFETY FEATURE, TO PREVENT THE HELICOPTER'S POWER TRAIN (& BY EXTENSION, MAIN ROTOR) FROM SPOOLING UP!!!
    // [WARNING]: THIS IS A SAFETY FEATURE, TO PREVENT THE HELICOPTER'S POWER TRAIN (& BY EXTENSION, MAIN ROTOR) FROM SPOOLING UP!!!
    // [WARNING]: THIS IS A SAFETY FEATURE, TO PREVENT THE HELICOPTER'S POWER TRAIN (& BY EXTENSION, MAIN ROTOR) FROM SPOOLING UP!!!
    try {

      ExceptionData_t xErrorData;

      if ( xControlData.psBuffer[ 4 ] > -2048 ) {

        xErrorData.ulID = 0x1;
        xErrorData.pcMsg = "ESC Throttle was above -2048.";
        xErrorData.xTskHandle = xTaskGetCurrentTaskHandle();
        xErrorData.pcTaskName = pcTaskGetName( xErrorData.xTskHandle );

        vWriteError( xErrorData );

        throw xErrorData.ulID;

      }
    }

    catch ( uint32_t ulError ) {

      /* Task execution stops here. */

    }

    if ( uxQueueSpacesAvailable( xServoDataQueue ) > 0 ) {

      if ( xQueueSendToBack( xServoDataQueue, &xControlData, 0 ) != pdPASS ) {

        vPrint( ( String ) "[" + pcName + " | ERROR]: Servo Data Queue is already full.\r\n" );

      }
    }
  }
}

/*
    @Task:    vServoFrontEnd( void *pvParameters )
    @Desc:    The only fully functional task in the firmware, so far.
              PID Process Variable data is received from a queue & is converted to servo position values & ESC throttle values.
              The Servo Front End applies 120° swashplate mixing to three servos - Aileron, Elevator & Pitch.
              All values are mixed, mapped & constrained to appropriate ranges. The servo driver API manages interpolation between different set values
              & how fast/slow those values should be interpolated.
    @Param:   None.
*/
void vServoFrontEnd( void *pvParameters ) {

  const char *pcName = pcTaskGetName( NULL );

  bool xError;
  TickType_t xLastTimeDataWasReceived = 0;
  TickType_t xTimeToSuspend = 0;

  int16_t sAilInput, sEleInput, sRudInput, sPitInput, sThrInput;
  int16_t sAilMix, sEleMix, sRudMix, sPitMix, sThrMix;
  int16_t sAilVal, sEleVal, sPitVal;
  int16_t psPwmBuffer[ ePwmChannelsCount ];
  ServoData_t xData;

#if defined( SERVOEASING_H_ )
  ServoEasing *pxServos;
#endif

  taskENTER_CRITICAL();
  vInitServos( ( void * )&pxServos );
  taskEXIT_CRITICAL();

  vTaskDelay( pdMS_TO_TICKS( 3000 ) );

  while ( 1 ) {

    if ( xQueueReceive( xServoDataQueue, &xData, pdMS_TO_TICKS( 20 ) ) == pdPASS ) {

      // No need to call these two lines in literally every switch case category.
      xLastTimeDataWasReceived = xTaskGetTickCount();
      xError = 0;

      switch ( xData.eDataSource ) {

        case ePIDControlLoop:
          sPitInput = xData.psBuffer[ 0 ];  // Pitch Input - Collective Pitch Process Variable from PID Control Loop
          sRudInput = xData.psBuffer[ 1 ];  // Rudder Input - Tail/Counter-Torqe Pitch Process Variable from PID Control Loop
          sEleInput = xData.psBuffer[ 2 ];  // Elevator Input - Fore/Aft Cyclic Pitch Process Variable from PID Control Loop
          sAilInput = xData.psBuffer[ 3 ];  // Aileron Input - Left/Right Cyclic Pitch Process Variable from PID Control Loop
          sThrInput = xData.psBuffer[ 4 ];  // Throttle Input - ESC Throttle Process Variable from PID Control Loop
          break;

      }

      // Limit control inputs.
      sAilMix = sMix( sAilInput, cCCPMMix );
      sEleMix = sMix( sEleInput, cCCPMMix );
      sPitMix = sMix( sPitInput, cCCPMMix );
      sRudMix = sMix( sRudInput, cRudMix );
      sThrMix = sMix( sThrInput, cEscMix );

      // Apply 120° swashplate mixing to the cyclic servos.
      sAilVal = sAilMix + sPitMix - ( sEleMix >> 1 );
      sEleVal = sEleMix + sPitMix;
      sPitVal = -sAilMix + sPitMix - ( sEleMix >> 1 );

      // Fill the PWM buffer with Normalized values mapped to servo values.
      psPwmBuffer[ ePwmAil ] = constrain( map( sAilVal, -2048, 2047, 0, 180 ), 0, 180 );
      psPwmBuffer[ ePwmEle ] = constrain( map( sEleVal, -2048, 2047, 0, 180 ), 0, 180 );
      psPwmBuffer[ ePwmPit ] = constrain( map( sPitVal, -2048, 2047, 0, 180 ), 0, 180 );
      psPwmBuffer[ ePwmRud ] = constrain( map( sRudMix, -2048, 2047, 0, 180 ), 0, 180 );
      psPwmBuffer[ ePwmEsc ] = constrain( map( sThrMix, -2048, 2047, 0, 100 ), 0, 100 );

      // Write buffer contents to the PWM Servo Driver.
      vUpdateServos( psPwmBuffer );

    }

    else {

      // Error handling - Reset servos to neutral positions; & suspend the Servo Front End after five second timeout.

      xTimeToSuspend = ( xTaskGetTickCount() - xLastTimeDataWasReceived );

      if ( xError == 0 ) {

        // Moved the error here. We get it! There's no data. STOP SPAMMING THE TERMINAL!!!
        vPrint( ( String ) "[" + pcName + " | ERROR]: No data was received.\r\n" );

        // Update the PWM/Servos buffer.
        for ( uint8_t i = 0; i < ePwmChannelsCount; i++ ) {

          if ( i == ePwmEsc ) psPwmBuffer[ i ] = 0;
          else psPwmBuffer[ i ] = 90;

        }

        // Reset the servos back to their neutral position.
        vUpdateServos( psPwmBuffer );
        vPrint( ( String ) "[" + pcName + " | NOTICE]: Servos reset to neutral.\r\n" );
        xError = 1;

      }

      // Suspend the Servo Front End (AKA This task) indefinitely, when the timeout expires.
      if ( xTimeToSuspend > pdMS_TO_TICKS( 5000 ) ) {

        vPrint( ( String ) "[" + pcName + " | NOTICE]: Suspended.\r\n" );
        vTaskSuspend( NULL );

      }
    }
  }
}

void vTaskManager( void *pvParameters ) {

  while ( 1 ) {

    ExceptionData_t xErrorData;
    xQueueReceive( xExceptionDataQueue, & xErrorData, portMAX_DELAY );

    vPrint( ( String )
            "Exception in thread: " + xErrorData.pcTaskName + "\r\n" +
            "Error ID: 0x" + String( xErrorData.ulID, HEX ) + "\r\n" +
            "Error: " + String( xErrorData.pcMsg ) + "\r\n"
          );

  }
}

int16_t sMix( int16_t p_value, int8_t p_mix ) {

  bool valneg = p_value < 0;

  uint16_t value = static_cast<uint16_t>( valneg ? -p_value : p_value );

  valneg ^= p_mix < 0;
  value = ( value * static_cast<uint16_t>( p_mix > 0 ? p_mix : -p_mix ) ) / 100;

  return valneg ? -static_cast<int16_t>( value ) : static_cast<int16_t>( value );

}

/*----------------------------------------------------------------------------------------------------------------------------------------------------*/
/*                                               H A R D W A R E    A B S T R A C T I O N    L A Y E R                                                */
/*----------------------------------------------------------------------------------------------------------------------------------------------------*/
/*
    @Func:      vBoardInit( bool bWaitForTerminal )
    @Desc:      Initializes the target development board.
    @Param:     'bWaitForTerminal' When set, the system will wait for the Terminal to open, if a valid USB connection has been established.
    @Returns:   Nothing.
*/
extern "C" void __libc_init_array( void );
void vBoardInit( bool bWaitForTerminal ) {

  init();
  __libc_init_array();

  static const uint8_t ucResetCause = PM->RCAUSE.reg & PM_RCAUSE_MASK;

#if defined( USE_TINYUSB )
  Adafruit_TinyUSB_Core_init();
#elif defined( USBCON )
  USBDevice.init();
  USBDevice.attach();
#endif

  if ( bWaitForTerminal > 0 ) {

    Serial.begin( 200000 );

    const uint32_t ulTimeoutMS = 1000;
    const uint32_t ulStartMS = millis();

    do {

#if defined( USE_TINYUSB )
      yield();
      if ( USBDevice.mounted() ) {

        while ( !Serial ) delay( 100 );
        break;

      }

#elif defined( USBCON )
      static const uint8_t ucFrameNumberAtStart = USB->DEVICE.FNUM.bit.FNUM;
      const uint8_t ucFrameNumber = USB->DEVICE.FNUM.bit.FNUM;

      if ( ucFrameNumber > ucFrameNumberAtStart ) {

        while ( !Serial );
        break;

      }
#endif
    } while ( millis() - ulStartMS <= ulTimeoutMS );

  }
}

/*
    @Func:      vFreeRtosInit( void )
    @Desc:      Initialized the RTOS & created all the necessary tasks, queues & semaphores that are required to run this firmware.
    @Param:     None.
    @Returns:   Nothing.
*/
void vFreeRtosInit( void ) {

  if ( xTaskCreate( vImu, "IMU", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL ) != pdPASS ) {

    vPrint( "[SYS | ERROR]: Unable to create IMU Task. Not enough memory.\r\n" );

  }

  if ( xTaskCreate( vReceiverInterface, "Rx", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL ) != pdPASS ) {

    vPrint( "[SYS | ERROR]: Unable to create Receiver Interface. Not enough memory.\r\n" );

  }

  if ( xTaskCreate( vPidControlLoop, "PID", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xPidControlLoopHandle ) != pdPASS ) {

    vPrint( "[SYS | ERROR]: Unable to create PID Control Loop. Not enough memory.\r\n" );

  }

  if ( xTaskCreate( vServoFrontEnd, "SERVOS", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL ) != pdPASS ) {

    vPrint( "[SYS | ERROR]: Unable to create Servo Front End. Not enough memory.\r\n" );

  }

  if ( xTaskCreate( vTaskManager, "Tsk Mngr", 256, NULL, tskIDLE_PRIORITY, &xTaskManagerHandle ) != pdPASS ) {

    vPrint( "[SYS | ERROR]: Unable to create Task Manager. Not enough memory.\r\n" );

  }

  xPidDataQueue = xQueueCreate( 2, sizeof( PidData_t ) );
  if ( xPidDataQueue == NULL ) {

    vPrint( "[SYS | ERROR]: Unable to create PID Data Queue. Not enough memory.\r\n" );

  }


  xServoDataQueue = xQueueCreate( 1, sizeof( ServoData_t ) );
  if ( xServoDataQueue == NULL ) {

    vPrint( "[SYS | ERROR]: Unable to create Servo Data Queue. Not enough memory.\r\n" );

  }

  xExceptionDataQueue = xQueueCreate( 5, sizeof( ExceptionData_t ) );
  if ( xExceptionDataQueue == NULL ) {

    vPrint( "[SYS | ERROR]: Unable to create Exception Data Queue. Not enough memory.\r\n" );

  }
}

/*
    @Func:      vFreeRtosRun( void )
    @Desc:      Starts the RTOS scheduler. This function will NEVER exit. If it does, this means that the RTOS couldn't allocate enough memory for the
                idle task.
    @Param:     None.
    @Returns:   Nothing.
*/
void vFreeRtosRun( void ) {

  // Bug in the microcontroller's bootloader doesn't switch the built-in LED off.
  digitalWrite( LED_BUILTIN, LOW );

  // Start the RTOS Scheduler.
  vTaskStartScheduler();

  vPrint( "[SYS | ERROR]: Unable to create Idle Task. Not enough memory.\r\n" );

}

/*
    @Func:      vApplicationIdleHook( void )
    @Desc:      RTOS idle task. This is executed whenever FreeRTOS is not executing any other tasks.
    @Param:     None.
    @Returns:   Nothing.
*/
extern "C" void vApplicationIdleHook( void ) {

  yield();

}

/*
    @Func:      yield( void )
    @Desc:      TinyUSB idle task. This needs to be called frequently, to ensure USB actually works.
    @Param:     None.
    @Returns:   Nothing.
*/
extern "C" void yield( void ) {

#if defined( USE_TINYUSB )
  tud_task();
  tud_cdc_write_flush();
#endif

}

/*
    @Func:      assert( bool x )
    @Desc:      Stops software execution when the condition 'x' is zero.
    @Param:     'x' The condition being evaluated.
    @Returns:   Nothing.
*/
void assert( bool x ) {

  // 'x' equals zero.
  if ( x == 0 ) {

    // Disable interrupts & halt.
    __asm volatile( " cpsid i " ::: "memory" );
    while ( 1 );

  }
}

/*
    @Func:      assert( bool x, const char *pcMsg )
    @Desc:      Stops software execution when the condition 'x' is zero; writes a message to the Terminal 100 mS before halting execution.
    @Param:     'x' The condition being evaluated.
    @Param:     'pcMsg' The message to write to the Terminal.
    @Returns:   Nothing.
*/
void assert( bool x, const char *pcMsg ) {

  // 'x' equals zero.
  if ( x == 0 ) {

    // Print a message to the terminal.
    Serial.print( pcMsg );
    delay( 100 );

    // Disable interrupts & halt.
    __asm volatile( " cpsid i " ::: "memory" );
    while ( 1 );

  }
}

/*
    @Func:      assert( bool x, String xMsg )
    @Desc:      Stops software execution when the condition 'x' is zero; writes a message to the Terminal 100 mS before halting execution.
    @Param:     'x' The condition being evaluated.
    @Param:     'xMsg' The message to write to the Terminal.
    @Returns:   Nothing.
*/
void assert( bool x, String xMsg ) {

  // 'x' equals zero.
  if ( x == 0 ) {

    // Print a message to the terminal.
    Serial.print( xMsg );
    delay( 100 );

    // Disable interrupts & halt.
    __asm volatile( " cpsid i " ::: "memory" );
    while ( 1 );

  }
}

/*
    @Func:      vPrint( const char *pcMsg )
    @Desc:      Writes a message to the Terminal.
    @Param:     '*pcMsg' The message to write to the Terminal.
    @Returns:   Nothing.
*/
void vPrint( const char *pcMsg ) {

  Serial.print( pcMsg );

}

/*
    @Func:      vPrint( String xMsg )
    @Desc:      Writes a message to the Terminal as a String object.
    @Param:     'xMsg' The message to write to the Terminal.
    @Returns:   Nothing.
*/
void vPrint( String xMsg ) {

  Serial.print( xMsg );

}

/*
    @Func:      vInitServos( void *pvServoObject )
    @Desc:      Sets up the necessary hardware required to drive the servos.
    @Param:     'pvServoObject' A pointer to a servo object.
    @Returns:   Nothing.
*/
void vInitServos( void *pvServoObject ) {

#if defined( SERVOEASING_H_ )

  // Version compatibility checking of the Servo Easing API.
#if VERSION_SERVO_EASING_MAJOR < 2 || VERSION_SERVO_EASING_MINOR < 3
#error "Fantail Flybarless requires Servo Easing, version 2.3.x or later. Update the Servo Easing API to the latest version."
#endif

#ifndef USE_PCA9685_SERVO_EXPANDER
#error "Enable 'USE_PCA9685_EXPANDER' in ServoEasing.h"
#endif

  ServoEasing *pxServos = ( ServoEasing * ) pvServoObject;

  // Aileron Servo - Front Right Swashplate Horn.
  pxServos = new ServoEasing( 0x40, &Wire );
  pxServos->attach( psPwmPins[ ePwmAil ], usCCPMMinPulse, usCCPMMaxPulse, usCCPMMinRange, usCCPMMaxRange);

  // Elevator Servo - Rear Swashplate Horn.
  pxServos = new ServoEasing( 0x40, &Wire );
  pxServos->attach( psPwmPins[ ePwmEle ], usCCPMMinPulse, usCCPMMaxPulse, usCCPMMinRange, usCCPMMaxRange);

  // Pitch Servo - Front Left Swashplate Horn.
  pxServos = new ServoEasing( 0x40, &Wire );
  pxServos->attach( psPwmPins[ ePwmPit ], usCCPMMinPulse, usCCPMMaxPulse, usCCPMMinRange, usCCPMMaxRange);

  // Rudder Servo - Tail Boom.
  pxServos = new ServoEasing( 0x40, &Wire );
  pxServos->attach( psPwmPins[ ePwmRud ], usRudMinPulse, usRudMaxPulse, usRudMinRange, usRudMaxRange );

  // ESC Throttle - Motor.
  pxServos = new ServoEasing( 0x40, &Wire );
  pxServos->attach( psPwmPins[ ePwmEsc ], usEscMinPulse, usEscMaxPulse, usEscMinRange, usEscMaxRange );

  // Center servos & set ESC throttle to zero.
  for ( uint8_t i = 0; i < ePwmChannelsCount; i++ ) {

    sServoArray[ i ]->setEasingType( EASE_LINEAR );

    switch ( i ) {

      case ePwmAil:
        sServoArray[ i ]->write( 90 );
        sServoArray[ i ]->setReverseOperation( 0 );
        sServoArray[ i ]->setTrim( 0, 1 );
        break;

      case ePwmEle:
        sServoArray[ i ]->write( 90 );
        sServoArray[ i ]->setReverseOperation( 1 );
        sServoArray[ i ]->setTrim( 0, 1 );
        break;

      case ePwmPit:
        sServoArray[ i ]->write( 90 );
        sServoArray[ i ]->setReverseOperation( 1 );
        sServoArray[ i ]->setTrim( 0, 1 );
        break;

      case ePwmRud:
        sServoArray[ i ]->write( 90 );
        sServoArray[ i ]->setReverseOperation( 0 );
        sServoArray[ i ]->setTrim( 0, 1 );
        break;

      case ePwmEsc:
        sServoArray[ i ]->write( 0 );
        sServoArray[ i ]->setReverseOperation( 0 );
        sServoArray[ i ]->setTrim( 0, 1 );
        break;

    }
  }

  synchronizeAllServosAndStartInterrupt( 1 );
#else
#error "Fantail Flybarless: No valid servo driver API was found. Either install an API or write your own."
#endif

}

/*
    @Func:      vUpdateServos( int16_t *psServoBuffer )
    @Desc:      Writes data to the servos.
    @Param:     'psServoBuffer' An array containing the servo data.
    @Returns:   Nothing.
*/
void vUpdateServos( int16_t *psServoBuffer ) {

#if defined( SERVOEASING_H_ )
  for ( uint8_t i = 0; i < sizeof( psServoBuffer ); i++ ) sServoNextPositionArray[ i ] = psServoBuffer[ i ];
  setEaseToForAllServosSynchronizeAndStartInterrupt( 100 );
  while ( areInterruptsActive() );
#endif

}

void vWriteError( ExceptionData_t xErrorData ) {

  xQueueSendToBack( xExceptionDataQueue, &xErrorData, 0 );
  vTaskPrioritySet( xErrorData.xTskHandle, tskIDLE_PRIORITY );

}
/*-----------------------------------------------------------------------------------------------------------------------------------------------------*/
/*                                        E N D    O F    H A R D W A R E    A B S T R A C T I O N    L A Y E R                                        */
/*-----------------------------------------------------------------------------------------------------------------------------------------------------*/

#else
#warning "Fantail Flybarless: Unsupported hardware; firmware may be non-functional."
void setup( void ) {

#if defined( LED_BUILTIN )
  pinMode( LED_BUILTIN, OUTPUT );
  digitalWrite( LED_BUILTIN, HIGH );
#endif

  const char *pcMsg = "[SYS | ERROR]: Unsupported hardware. Fantail Flybarless firmware is non-functional.\r\n";
  const uint32_t ulTimeoutMS = 1000;
  const uint32_t ulStartMS = millis();

  // TinyUSB.
#if defined( USE_TINYUSB )
  do {

    if ( USBDevice.mounted() ) {

      while ( !Serial ) delay( 50 );
      break;

    }
  } while ( millis() - ulStartMS <= ulTimeoutMS );

  Serial.print( pcMsg );

  // Arduino USB.
#elif defined( USBCON )
  do {

    // AVR-specific USB Frame Number counting.
#if defined( ARDUINO_ARCH_AVR )
    static const uint8_t ucFrameNumberAtStart = UDFNUML;
    const uint8_t ucFrameNumber = UDFNUML;

    // SAMD-specific USB Frame Number counting.
#elif defined( ARDUINO_ARCH_SAMD )
    static const uint8_t ucFrameNumberAtStart = USB->DEVICE.FNUM.bit.FNUM;
    const uint8_t ucFrameNumber = USB->DEVICE.FNUM.bit.FNUM;

    // Non-specified. USB Frame Number counting is not available.
#else
    static const uint8_t ucFrameNumberAtStart = 0;
    const uint8_t ucFrameNumber = 1;
#endif

    if ( ucFrameNumber > ucFrameNumberAtStart ) {

#if defined( SerialUSB )
      while ( !SerialUSB );
#else
      while ( !Serial );
#endif
      break;

    }

  } while ( millis() - ulStartMS <= ulTimeoutMS );

#if defined( SerialUSB )
  SerialUSB.print( pcMsg );
#else
  Serial.print( pcMsg );
#endif

  // Arduino UART
#else
  Serial.begin( 9600 );
  delay( 20000 );
  Serial.print( pcMsg );
#endif

}

void loop( void ) {

  // Blink the built-in LED.
#if defined( LED_BUILTIN )
  delay( 500 );
  digitalWrite( LED_BUILTIN, LOW );
  delay( 500 );
  digitalWrite( LED_BUILTIN, HIGH );
#endif

}
#endif
#else
#error "Fantail Flybarless: Arduino API was not found."
#endif
