/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

// Added for RTbot control
#define RTBOT_READ_ODOMETRY      'o'
#define RTBOT_SET_MOTORS         'm'
//#define RTBOT_DRIVE              'v'

// Added for ARbot control
#define ARBOT_READ_ODOMETRY      'o'
#define ARBOT_SET_ADVANCE_LED    'f'
#define ARBOT_SET_VELOCITY       'v'

// Added for Pixy blob fetching
#define GET_PIXY_BLOB_COUNT      'g'
#define GET_BLOB_AT_INDEX        'h'

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define LEFT            0
#define RIGHT           1


#endif



