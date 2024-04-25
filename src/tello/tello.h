/*
 * File name: tello.h
 * Date:      2018/10/24 20:51
 * Author:    Jan Chudoba
 */

#ifndef __TELLO_H__
#define __TELLO_H__

#include <stdint.h>
#include <pthread.h>
#include <netinet/in.h>
#include <mutex>
#include <iostream>
#include <fstream>
#include <atomic>

using namespace std;

#ifndef _mypoint_t_
#define _mypoint_t_

struct mypoint_t{
	double x;
	double y;
	double z;
};

#endif


class CTello
{
public:
	CTello();
	~CTello();

	bool init(const int port = 8889, const char* net_interface = NULL);
	void close();

	void setStickData(int fast, double roll, double pitch, double throttle, double yaw);
	bool takeOff();
	bool land();
	bool setSmartVideoShot(int mode, bool isStart);
	bool bounce(bool isStart);
        void sendTextCommand(char command[]);
//        void set_goal(float x, float y, float z);
//        void set_goal(Target* target);
//        void set_goal_offset(float x, float y, float z);
//        void assign_own_target(Target* target);
//        void stop_navigating();

//        void start_target_navigating();
//        void start_position_navigating();
//        void start_trajectory_navigating();
//        void start_vicon_navigating();
//        void start_localize_vicon();
//        void start_localize_dwloc();

        
//        void link_vicon_master(Vicon_master* master);
//        void set_goal_vicon(int ID);
//        void set_own_vicon(int ID);
//        mypoint_t get_trajectory_point(uint16_t ms);
//        void get_position(mypoint_t* position);

protected:
	uint16_t calcCRC16(uint8_t * buf, int size);
	uint8_t calcCRC8(uint8_t * buf, int size);
	int buildPacket(uint8_t * outPacket, unsigned int & outPacketLength, uint8_t pacType, uint16_t cmdID, uint16_t seqID, uint8_t * data, int dataLength);
	uint16_t parsePacket(uint8_t * buf, int bufLen);
	bool sendCmd(uint8_t pacType, uint16_t cmdID, uint8_t * data = NULL, int dataLength = 0);

	static void * cmd_rx_thread_body(void * arg) { ((CTello*)arg)->cmdRxThreadBody(); return 0; }
	void cmdRxThreadBody();
	static void * timer_thread_body(void * arg) { ((CTello*)arg)->timerThreadBody(); return 0; }
	void timerThreadBody();

	// TODO: video rx
	// TODO: timer task

private:
	pthread_t cmdRxThread;
	bool cmdRxThreadStarted;
	pthread_t timerThread;
	bool timerThreadStarted;

        mutex send_command_mtx;
        mutex stick_data_mtx;
                
        ofstream log_file;

        bool navigate_dwloc = false;
        bool navigate_position = false;
        bool navigate_trajectory = false;
        bool navigate_vicon = false;
        bool localize_vicon = false;
        bool localize_dwloc = false;

        uint64_t trajectory_timestamp;
        mypoint_t trajectory_origin;

	bool threadInterrupt;
	int fdSockCmd;
	int cmdPort;
	struct sockaddr_in cmdAddr;
        
//        Target* target_goal = 0;
//        Target* target_own = 0;
//        Vicon_master* vicon_master = NULL;
//        
//        mypoint_t goal;
//        mypoint_t goal_offset = {0,0,0};
//        int vicon_id_target = -1;
//        int vicon_id_own = -1;
//        
//        double x_i = 0;
//        double y_i = 0;
//        double x_d = 0;
//        double y_d = 0;
//        double dx_last = 0;
//        double dy_last = 0;
        
//        void regulate();
        void limit(double &input, double min, double max);
        void normalize_sticks_input(double &u_x, double &u_y);
        double max_abs(double x, double y);

	atomic<uint64_t> stickData;
	uint16_t seqID;

	enum TTelloCmd {
		TELLO_CMD_CONN                      = 1     ,
		TELLO_CMD_CONN_ACK                  = 2     ,
		TELLO_CMD_SSID                      = 17    , //pt48
		TELLO_CMD_SET_SSID                  = 18    , //pt68
		TELLO_CMD_SSID_PASS                 = 19    , //pt48
		TELLO_CMD_SET_SSID_PASS             = 20    , //pt68
		TELLO_CMD_REGION                    = 21    , //pt48
		TELLO_CMD_SET_REGION                = 22    , //pt68
		TELLO_CMD_REQ_VIDEO_SPS_PPS         = 37    , //pt60
		TELLO_CMD_TAKE_PICTURE              = 48    , //pt68
		TELLO_CMD_SWITCH_PICTURE_VIDEO      = 49    , //pt68
		TELLO_CMD_START_RECORDING           = 50    , //pt68
		TELLO_CMD_SET_EV                    = 52    , //pt48
		TELLO_CMD_DATE_TIME                 = 70    , //pt50
		TELLO_CMD_STICK                     = 80    , //pt60
		TELLO_CMD_LOG_HEADER_WRITE          = 4176  , //pt50
		TELLO_CMD_LOG_DATA_WRITE            = 4177  , //RX_O
		TELLO_CMD_LOG_CONFIGURATION         = 4178  , //pt50
		TELLO_CMD_WIFI_SIGNAL               = 26    , //RX_O
		TELLO_CMD_VIDEO_BIT_RATE            = 40    , //pt48
		TELLO_CMD_LIGHT_STRENGTH            = 53    , //RX_O
		TELLO_CMD_VERSION_STRING            = 69    , //pt48
		TELLO_CMD_ACTIVATION_TIME           = 71    , //pt48
		TELLO_CMD_LOADER_VERSION            = 73    , //pt48
		TELLO_CMD_STATUS                    = 86    , //RX_O
		TELLO_CMD_ALT_LIMIT                 = 4182  , //pt48
		TELLO_CMD_LOW_BATT_THRESHOLD        = 4183  , //pt48
		TELLO_CMD_ATT_ANGLE                 = 4185  , //pt48
		TELLO_CMD_SET_JPEG_QUALITY          = 55    , //pt68
		TELLO_CMD_TAKEOFF                   = 84    , //pt68
		TELLO_CMD_LANDING                   = 85    , //pt68
		TELLO_CMD_SET_ALT_LIMIT             = 88    , //pt68
		TELLO_CMD_FLIP                      = 92    , //pt70
		TELLO_CMD_THROW_FLY                 = 93    , //pt48
		TELLO_CMD_PALM_LANDING              = 94    , //pt48
		TELLO_CMD_PLANE_CALIBRATION         = 4180  , //pt68
		TELLO_CMD_SET_LOW_BATTERY_THRESHOLD = 4181  , //pt68
		TELLO_CMD_SET_ATTITUDE_ANGLE        = 4184  , //pt68
		TELLO_CMD_ERROR1                    = 67    , //RX_O
		TELLO_CMD_ERROR2                    = 68    , //RX_O
		TELLO_CMD_FILE_SIZE                 = 98    , //pt50
		TELLO_CMD_FILE_DATA                 = 99    , //pt50
		TELLO_CMD_FILE_COMPLETE             = 100   , //pt48
		TELLO_CMD_HANDLE_IMU_ANGLE          = 90    , //pt48
		TELLO_CMD_SET_VIDEO_BIT_RATE        = 32    , //pt68
		TELLO_CMD_SET_DYN_ADJ_RATE          = 33    , //pt68
		TELLO_CMD_SET_EIS                   = 36    , //pt68
		TELLO_CMD_SMART_VIDEO_START         = 128   , //pt68
		TELLO_CMD_SMART_VIDEO_STATUS        = 129   , //pt50
		TELLO_CMD_BOUNCE                    = 4179  , //pt68
	};
	enum TSmartVideo {
		TELLO_SMART_VIDEO_STOP              = 0x00,
		TELLO_SMART_VIDEO_START             = 0x01,
		TELLO_SMART_VIDEO_360               = 0x01,
		TELLO_SMART_VIDEO_CIRCLE            = 0x02,
		TELLO_SMART_VIDEO_UP_OUT            = 0x03
	};

};

#endif

/* end of tello.h */
