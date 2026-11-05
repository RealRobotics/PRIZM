#ifndef HITEC_CAN_PUBLIC_REGISTER_H
#define HITEC_CAN_PUBLIC_REGISTER_H
/**
 **************************************************************************
 * Copyright 2017-2023 HITEC RCD Korea, INC ( http://www.hitecrcd.co.kr/ )
 * File Name    : hitec-can-servo-reg-public.h
 * Description  : HITEC CAN Servo Register Header file
 * Date         : 2023-04-18
 * Version      : Ver 2.0
 **************************************************************************

이 파일은 BSD license 를 따릅니다.

이 파일은 하이텍 CAN 서보의 작동에 필요한 선언과 매크로 함수를 제공합니다.
이 파일을 자신의 시스템에 적용할때에는 전체 동작에 문제가 없는지 철저히 확인해야 합니다.
하이텍은 사전 알림 없이 수정판을 신규 배포할수 있습니다.

수정시 컴파일에 혼란이나 문제가 발생하지 않도록 주의해야 합니다.
재배포시 파일의 첫 부분에 이 글을 그대로 포함해야 합니다.
수정된 내용에 대한 설명은 이 글의 다음에 추가하기 바랍니다.

This file is under the BSD license.

This file provides the declarations and macro functions necessary for the operation of HITEC CAN servos.
When applying this file to your own system,
you should thoroughly check that there are no problems with the entire operation.
HITEC may distribute new modified versions without prior notice.

Care must be taken when modifying to avoid confusion or problems with compilation.
When redistributing, you must include this text verbatim at the beginning of the file.
Please add an explanation of the modified contents at the end of this article.

New Download:
http://support.hitecrcd.net:7700/index.php/CanServo/0

ST Version : FW Ver < 1.7
AT Version : FW Ver >= 1.7

NOTE: Reserved value must be NOT changed
 */
typedef enum
{
	e_PRODUCT_NO_0 = 0x00, // -> 0x74, 116
	e_PRODUCT_VER_0 = 0x02, // -> 0x76, 118
	// 0x04, 4, Reserved
	e_STATUS = 0x06, // 6
		#define BIT_STATUS_ENABLED 0
		#define BIT_STATUS_OVER_CURRENT 1
		#define BIT_STATUS_OVER_LOADED 2
	e_FUNC_LIST = 0x08, // 8
		#define BIT_FUNC_ENABLE 0
		#define BIT_FUNC_EDIT_TURN_COUNT 1
		#define BIT_FUNC_I_GAIN_x1000 2
		#define BIT_FUNC_OVERVOLT_BRAKE 3
		#define BIT_FUNC_OVERVOLT_BRAKE_TIME 4
		#define BIT_FUNC_SELECT_PAD 5
		#define BIT_FUNC_VOLTAGE_PEAK 6
		#define BIT_FUNC_FAST_VOLTAGE 7
		#define BIT_FUNC_WIDE_VOLTAGE 8
		#define BIT_FUNC_CURRENT_CUT 9
		#define BIT_FUNC_OVERLOADED 10
		#define BIT_FUNC_STREAM_CAN_ID 11
	// 0x0A, 10, Reserved
	e_POSITION = 0x0C, // 12, 0~16383, 16384 = 360 deg
	e_VELOCITY = 0x0E, // 14, pos/100msec = Vp, Vt = Ts * 60 * 16384 / 360 / Vp / 10, Vp = posdiff/100msec, PT = Reg 0xA4, Ts = PT / 10, Vt = sec/60deg
#define VelocityConv(V,PT) ((PT) * 60 * 16384 / 360 / (V) / 10 / 10)
	e_TORQUE = 0x10, // 16, 0~4095, motor pwm duty, 4095 = 100%
	e_VOLTAGE = 0x12, // 18, 0.01V, 100=1.00V
	e_TEMPER = 0x14, // 20, deg, MCU Temper.
	e_CURRENT = 0x16, // 22, mA
	e_TURN_COUNT = 0x18, // 24, signed int 16 bits
	e_32BITS_POSITION = 0x1A, // 26, 360deg=16384
	e_32BITS_POSITION_H = 0x1C, // 28
	e_POSITION_NEW = 0x1E, // 30
	e_TMP_CONFIG = 0x20, // 32, reset all bits to 0 on servo reset
		#define BIT_PAUSE_STREAM 0
	// 0x22, 34, Reserved
	e_TURN_NEW = 0x24, // 36
	e_SPEC_TORQUE = 0x26, // 38, Watt, 38, 9000=90.00W
	// 0x28, Reserved
	// 0x2A, Reserved
	e_UNITLESS_RAD_MODE = 0x2C, // 44, 0=deg, 1=radian
	e_STREAM_TIME = 0x2E, // 46, ms
	e_STREAM_MODE = 0x30, // 48, 0=off, 1=on
	e_ID1 = 0x32, // 50, Servo ID in Data, 0~255, (0~127 in DroneCAN)
	// 0x34, 52 // Reserved
	// 0x36, 54 // Reserved, System Configuration Value
	e_CAN_BAUDRATE = 0x38, // 56, REG_CAN_BAUDRATE, 0~
	e_STREAM_ID_H = 0x3A, // 58
	e_ID2_H = 0x3C, // 60, CAN_BUS_ID_H
	e_ID2_L = 0x3E, // 62, CAN_BUS_ID_L
	e_SAMPLE_POINT = 0x40, // 64, REG_SAMPLE_POINT, 0=50%, 1=87.5%
	e_STREAM_ID_L = 0x42, // 66
	e_SERVO_MODE = 0x44, // 68
	// 1.6(2): 0=turn, 1=servo
	// 2.0: 0=turn, 1=servo, 2=cr, 3=speed

	e_POWER = 0x46, // 70
		// bit 0 = reset mode 0
		// bit 1 = reset mode 1
		// bit 10:9 = emg status, 0=normal, 1=emg free, 2=emg hold, 3=emg speed down
		// other bits: Reserved, keep 0
	#define EMG_MODE() ((reg_val(e_POWER) >> 9) & 3)
		s_EMG_FREE       = 1, // 0x0200 - 01
		s_EMG_SPEED_DOWN = 2, // 0x0400 - 10
		s_EMG_HOLD       = 3, // 0x0600 - 11
	e_EMERGENCY_STOP = 0x48, // 72, Emergency Status Bits
	e_PARAM_VERSION_1 = 0x4A, // 0x4A, // 74 Year + Revision of 10th, YYYYH of YYYY-MM-DD-Hh, 2021-11-28-01 -> 20210
	e_PARAM_VERSION_2 = 0x4C, // 0x4C, // 76 MonthDay + Revision of 1th, MMDDh of YYYY-MM-DD-Hh, 2021-11-28-01 -> 11281
	e_DEADBAND = 0x4E, // 78, Output Position Deadzone, range 4096 = 90 deg
	e_POS_MAX = 0x50, // 80, SERVO MODE, EMG STOP RANGE
	e_POS_MIN = 0x52, // 82, SERVO MODE, EMG STOP RANGE
	e_VELOCITY_MAX = 0x54, // 84, check VELOCITY_MAX format
	e_TORQUE_MAX = 0x56, // 86, Motor PWM duty max: 4095=100%
	e_VOLTAGE_MAX = 0x58, // 88, Max Voltage Conf, 100=1.00V, same range in e_VOLTAGE
	e_VOLTAGE_MIN = 0x5A, // 90, Min Voltage Conf, 100=1.00V, same range in e_VOLTAGE
	e_TEMPER_MAX = 0x5C, // 92, mcu temper. maximum, deg
	e_PRODUCT_INFO = 0x5E, // 94, Product Infomation
	
	// 0x60, 96, System Reserved
	e_MOTOR_PWM_MINIMUM = 0x62, // 98, Product configuration value, DO NOT CHANGE
	
	e_INERTIA_RANGE = 0x64, // 100, REG_INERTIA_RANGE, 0=smart sense disable, 1=smart sense on, 2+ = manual mode
	e_INERTIA_CONF = 0x66, // 102, REG_INERTIA_CONF

	e_PEAK_POWER_RATE = 0x68, // 104 // 1000 = 100%
	e_CAN_MODE = 0x6A, // 106, REG_CAN_MODE, 0=2.0A, 1=2.0B, 2=UAVCAN
	e_TEMPER_MIN = 0x6C, // 108, mcu temper. minimum, deg
	e_FACTORY_DEFAULT = 0x6E, // 110, REG_FACTORY_DEFAULT, default command
	e_CONFIG_SAVE = 0x70, // 112, REG_CONFIG_SAVE, SAVE command
	e_CONFIG_LOCK = 0x72, // 114, REG_CONFIG_LOCK

	e_PRODUCT_NO = 0x74, // 116, Product No
	e_PRODUCT_VER = 0x76, // 118, Product Version
	e_PARAM_VER = 0x78, // 120, Param Version Old
	e_STARTUP_POSITION_NEW = 0x7A, // 122
	e_VOLT_R_H = 0x7C, // 124
	e_VOLT_R_L = 0x7E, // 126
	e_BRAKE_VOLT = 0x80, // 128, 1000 -> 10.00V
	e_BRAKE_TIME = 0x82, // 130, msec
	
	e_SETUP_4 = 0x84, // 132
		#define BIT_SETUP_4_MOTOR_DIRECTION 0	// 0:CW,1:CCW, Servo reset needed // FW Ver >= 2.0
	e_MOTOR_PWM_PERIOD = 0x86, // 134
	e_MOTOR_PWM_DEADTIME= 0x88, // 136

	e_P_GAIN = 0x8A, // 138, PID P gain value
	e_D_GAIN = 0x8C, // 140, PID D gain value
	e_I_GAIN = 0x8E, // 142, PID I gain value
	e_C_GAIN = 0x90, // 144, PID C gain value
	
	e_ACC = 0x92, // 146, acceleration, diff. of positions
	
	e_FAIL_SAFE_POSITION_NEW = 0x94, // 148

	e_POS_LOCK_LIMIT = 0x98, // 152, e_POS_LOCK_LIMIT // FW Ver >= 2.0
	e_POS_LOCK_TIME = 0x9A, // 154, REG_POS_LOCK_TIME, OLP TIME, second
	e_POS_LOCK_TORQUE_RATIO = 0x9C, // 156, Output rate in OLP, 0=zero, 100=100%
	
	// 0x9E, 158 // Reserved, DO NOT CHANGE BITS
	// 0xA0, 160 // Reserved, System Settings
	
	e_SETUP_1 = 0xA2, // 162, option 3 page
		#define BIT_SELECT_PAD 0
		#define BIT_STARTUP_POSITION_NEW 1
		#define BIT_USE_BRAKE_INSTEAD_FREE 2		// BLDC Motor
		#define BRAKE_MODE() getbit(reg_val(e_SETUP_1),BIT_USE_BRAKE_INSTEAD_FREE)
		#define BIT_USE_HIGH_VOLTAGE_BRAKE 3
		#define BIT_USE_WIDE_VOLTAGE 5				// FW Ver >= 2.0
		#define BIT_USE_CURRENT_CUT 6
		#define BIT_USE_STREAM_CAN_ID 7
		#define BIT_USE_VOLTAGE_MIN_REAL_TIME 8
		#define BIT_USE_SPEED_UP_or_DN_AS_REVERSE 9
		#define BIT_USE_FAIL_SAFE_POSITION 10
		#define BIT_USE_2PHASE_BRAKE 11				// FW Ver >= 2.0
		#define BIT_USE_ID_REALTIME 12				// FW Ver >= 2.0
		#define BIT_USE_RETURN_CAN_ID_OR_1 13       // FW Ver >= 2.0
		
	e_PID_TIME = 0xA4, // 164, PID sampling time, Ts, 0.1msec
	e_REF_1 = 0xA6, // 166
		// bit 0 = e_PAD_VOLT[0xA8] is real !
	e_PAD_VOLT = 0xA8, 	// 168,
		// Low Byte = Voltage by PAD logic Low
		// High Byte = Voltage by PAD logic High

	// 0xAA // Reserved, DO NOT CHANGE BITS
	// 0xAC // Reserved
	
	e_ZERO_OFFSET = 0xAE, // 174
	e_POSITION_MAX = 0xB0, // 176
	e_POSITION_MIN = 0xB2, // 178
	e_FAIL_SAFE_TIME = 0xB4, // 180, ms, BIT_USE_FAIL_SAFE_POSITION=1

	e_CURRENT_VALUE_ADC_0 = 0xB6, // 182, signed int 16
	e_CURRENT_VALUE_ADC_4095 = 0xB8, // 184, unsigned int 16
	
	// 0xBA, System Reserved, DO NOT CHANGE BITS
	// 0xBC, System Reserved, DO NOT CHANGE BITS

	e_MOTOR_PSC = 0xBE, // 190, Motor PWM Prescaler, System Configuration Value. DO NOT CHANGE.

	// 0xC0, Product Reserved, DO NOT CHANGE BITS

	e_POSITION_MID = 0xC2, // 194, Zero Point, 8192 recommanded.
	
	e_CURRENT_K2 = 0xC4, // 196, Current Dec. delay // FW Ver >= 2.0
	
	e_ECHO = 0xC6, // 198, Reset 0 on servo reset.
	e_TIME_L = 0xC8, // 200, Low 16 bits of Servo running time in seconds 32 bits value
	e_TIME_H = 0xCA, // 202, High 16 bits of Servo running time in seconds 32 bits value
	e_USER_1 = 0xCC, // 204, User Value 1
	e_USER_2 = 0xCE, // 206, User Value 2
	
	e_MOTOR_TEMP = 0xD0, // 208, Motor Temper., ADC
	e_MOTOR_TEMP_C = 0xD1, // 209, Motor Temper., deg // FW Ver >= 2.0
	e_TEMP = 0xD2, // 210, Temper. of HUM sensor, deg
	e_HUM = 0xD4, // 212, HUM value
	
	e_CURRENT_K = 0xD6, // 214
	// Recommand default value = 3267 for ST Version
	// Recommand default value = 500 for AT Version
	
	e_CURRENT_MAX = 0xD8, // 216, mA
	
	e_SPEED_VOLTAGE = 0xDA, // 218, Ref Voltage for Velocity Max, 180 -> 18.0V
	e_SPEED_UP = 0xDC, // 220, msec, Time for speed 0 to max
	e_SPEED_DN = 0xDE, // 222, msec, Time for speed max to 0
	e_SPEED_ES = 0xE0, // 224, msec, Time for speed max to 0 in emergency

	e_STREAM_0 = 0xE2, // 226, STREAM ADDR 0 = Stream 1,2
	e_STREAM_1 = 0xE4, // 228, STREAM ADDR 1 = Stream 3,4
	e_STREAM_2 = 0xE6, // 230, STREAM ADDR 2 = Stream 5,6
	e_STREAM_3 = 0xE8, // 232, STREAM ADDR 3 = Stream 7,8
} HITEC_CAN_PUBLIC_REGISTER_t;

#endif
