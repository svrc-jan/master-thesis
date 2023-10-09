/*
 * File name: tello.cc
 * Date:      2018/10/26 19:31
 * Author:    Jan Chudoba
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <cmath>

#include "tello.h"
#include "timeutils.h"

#define TELLO_PORT_VIDEO 6037
#define TELLO_PORT 8889
static const char * TELLO_IP = "192.168.10.1";

#define NEW_ALT_LIMIT 30
#define STICKS_LOOP 5e3 

CTello::CTello() :
	cmdRxThreadStarted(false),
	timerThreadStarted(false),
	fdSockCmd(-1),
	cmdPort(8889),
	stickData(0),
	seqID(0)
{
}

CTello::~CTello()
{
        land();
	close();
}

bool CTello::init(int port, char* net_interface)
{
	//close();

	fdSockCmd = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (fdSockCmd < 0) return false;

	int so_reuse_addr_value = 1;
	setsockopt(fdSockCmd, SOL_SOCKET, SO_REUSEADDR, &so_reuse_addr_value, sizeof(so_reuse_addr_value));
        if (net_interface != 0){
            struct ifreq ifr;
            memset(&ifr, 0, sizeof(ifr));
            snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", net_interface);
            if (setsockopt(fdSockCmd, SOL_SOCKET, SO_BINDTODEVICE, (void *)&ifr, sizeof(ifr)) < 0) {
                printf("cant bind socket to network interface\n");
                return false;
            }
        }

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof addr);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(cmdPort);
	addr.sin_addr.s_addr = INADDR_ANY;

	memset(&cmdAddr, 0, sizeof(cmdAddr));
	cmdAddr.sin_family = AF_INET;
	cmdAddr.sin_port = htons(TELLO_PORT);
	inet_pton(AF_INET, TELLO_IP, &cmdAddr.sin_addr.s_addr); // TODO: ip

	if (bind(fdSockCmd, (struct sockaddr *)&addr, sizeof addr) < 0) {
		::close(fdSockCmd);
		fdSockCmd = -1;
		return false;
	}
        

	threadInterrupt = false;
	if (pthread_create(&cmdRxThread, 0, cmd_rx_thread_body, this) != 0) {
		fprintf(stderr, "Failed to start cmd rx thread\n");
		close();
		return false;
	}
	cmdRxThreadStarted = true;

	if (pthread_create(&timerThread, 0, timer_thread_body, this) != 0) {
		threadInterrupt = true;
		fprintf(stderr, "Failed to start timer thread\n");
		close();
		return false;
	}
	timerThreadStarted = true;

	// TODO: start video thread

	sendCmd(0x00, TELLO_CMD_CONN, NULL);
        
        usleep(100e3);
        setStickData(false, 0, 0, 0, 0);
        usleep(100e3);
        char cmd[10];
        strcpy(cmd, "battery?");
        sendTextCommand(cmd);
        
        //printf("Tello connected\n");
              
        
        log_file.open ("regulator_out.txt");
        log_file << "Log init\n";

	return true;
}

void CTello::close()
{
	if (cmdRxThreadStarted) {
		threadInterrupt = true;
		cmdRxThreadStarted = false;
		pthread_join(cmdRxThread, NULL);
		timerThreadStarted = false;
		pthread_join(timerThread, NULL);
	}
	::close(fdSockCmd);
	fdSockCmd = -1;
        log_file.close();
        printf("Tello successfully disconnected\n");
}


void CTello::limit(double &input, double min, double max){
    if (input > max)
        input = max;
    if (input < min)
        input = min;
    
    return;
}

void CTello::normalize_sticks_input(double &u_x, double &u_y){
    double k = 1;
    
    k = max_abs(k, u_x);
    k = max_abs(k, u_y);
    
    u_x /=k;
    u_y /=k;
    
    return;
}

double CTello::max_abs(double x, double y){
    if (abs(x) > abs(y)){
        return abs(x);
    } else {
        return abs(y);
    }
}

const int RC_VAL_MIN     = 364;
const int RC_VAL_MID     = 1024;
const int RC_VAL_MAX     = 1684;

void CTello::setStickData(int fast, double roll, double pitch, double throttle, double yaw)
{
	uint64_t iRoll = RC_VAL_MID + (int) (roll * (RC_VAL_MAX - RC_VAL_MID));
	if (iRoll < RC_VAL_MIN) iRoll = RC_VAL_MIN;
	if (iRoll > RC_VAL_MAX) iRoll = RC_VAL_MAX;
	uint64_t iPitch = RC_VAL_MID + (int) (pitch * (RC_VAL_MAX - RC_VAL_MID));
	if (iPitch < RC_VAL_MIN) iPitch = RC_VAL_MIN;
	if (iPitch > RC_VAL_MAX) iPitch = RC_VAL_MAX;
	uint64_t iThrottle = RC_VAL_MID + (int) (throttle * (RC_VAL_MAX - RC_VAL_MID));
	if (iThrottle < RC_VAL_MIN) iThrottle = RC_VAL_MIN;
	if (iThrottle > RC_VAL_MAX) iThrottle = RC_VAL_MAX;
	uint64_t iYaw = RC_VAL_MID - (int) (yaw * (RC_VAL_MAX - RC_VAL_MID));
	if (iYaw < RC_VAL_MIN) iYaw = RC_VAL_MIN;
	if (iYaw > RC_VAL_MAX) iYaw = RC_VAL_MAX;
	stickData = ((uint64_t)fast << 44) | (iYaw << 33) | (iThrottle << 22) | (iPitch << 11) | (iRoll);
	//fprintf(stderr, "DEBUG: set stick data %016lX = %lu | %lu | %lu | %lu\n", stickData, iYaw, iThrottle, iPitch, iRoll);
}

bool CTello::takeOff()
{       printf("TAKEEEEE\n");
        setStickData(0, 0, 0, 0, 0);
        sendCmd(0x68, TELLO_CMD_TAKEOFF);
        usleep(100e3);
        return sendCmd(0x68, TELLO_CMD_TAKEOFF);
}

bool CTello::land()
{   
    sendCmd(0x68, TELLO_CMD_LANDING, (uint8_t*) "\0", 1);
    usleep(100e3);
    sendCmd(0x68, TELLO_CMD_LANDING, (uint8_t*) "\0", 1);
    usleep(100e3);
    sendCmd(0x68, TELLO_CMD_LANDING, (uint8_t*) "\0", 1);
    usleep(1000e3);
    return sendCmd(0x68, TELLO_CMD_LANDING, (uint8_t*) "\0", 1);
}


bool CTello::setSmartVideoShot(int mode, bool isStart)
{
	uint8_t data = (isStart) ? TELLO_SMART_VIDEO_START : TELLO_SMART_VIDEO_STOP;
	data = data | (mode << 2);
	return sendCmd(0x68, TELLO_CMD_SMART_VIDEO_START, &data, 1);
}

bool CTello::bounce(bool isStart)
{
	uint8_t data = (isStart) ? 0x30 : 0x31;
	return sendCmd(0x68, TELLO_CMD_BOUNCE, &data, 1);
}

// CRC TABLES
static const uint16_t TBL_CRC16[] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static const uint8_t TBL_CRC8[] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
 
uint16_t CTello::calcCRC16(uint8_t * buf, int size)
{
	unsigned int i = 0;
	uint16_t seed = 0x3692;
	while (size > 0) {
		seed = TBL_CRC16[(seed ^ buf[i]) & 0xff] ^ (seed >> 8);
		i++;
		size--;
	}
	return seed;
}

uint8_t CTello::calcCRC8(uint8_t * buf, int size)
{
	unsigned int i = 0;
	uint8_t seed = 0x77;
	while (size > 0) {
		seed = TBL_CRC8[(seed ^ buf[i]) & 0xff];
		i++;
		size--;
	}
	return seed;
}

int CTello::buildPacket(uint8_t * outPacket, unsigned int & outPacketLength, uint8_t pacType, uint16_t cmdID, uint16_t seqID, uint8_t * data, int dataLength)
{
	unsigned int size = dataLength + 11;
	outPacketLength = size;
	outPacket[0] = 0xCC;
	outPacket[1] = (size << 3) & 0xFF;
	outPacket[2] = ((size << 3) >> 8) & 0xFF;
	uint8_t crc8 = calcCRC8(outPacket, 3);
	outPacket[3] = crc8;
	outPacket[4] = pacType;
	outPacket[5] = (cmdID) & 0xFF;
	outPacket[6] = (cmdID >> 8) & 0xFF;
	outPacket[7] = (seqID) & 0xFF;
	outPacket[8] = (seqID >> 8) & 0xFF;
	if (data) {
		memcpy(outPacket+9, data, dataLength);
	}
	uint16_t crc16 = calcCRC16(outPacket, size - 2);
	outPacket[9+dataLength] = crc16 & 0xFF;
	outPacket[10+dataLength] = crc16 >> 8;
	return outPacketLength;
}

uint16_t CTello::parsePacket(uint8_t * buf, int bufLen)
{
	//int dataSize = 0;
	uint16_t cmdID = 0;

	if (bufLen >= 11) {
		int pos = 0;
		uint8_t mark = buf[pos++];
		if (mark == 0xCC) {
			uint16_t size = ((((uint16_t)buf[pos+1]) << 8) | (buf[pos+0])) >> 3; // LSB MSB
			pos += 2;
			uint8_t crc8 = buf[pos++];
			uint8_t calcCrc8 = calcCRC8(buf, 3);
			if (crc8 != calcCrc8) {
				fprintf(stderr, "wrong CRC8 %02X / %02X\n", crc8, calcCrc8);
			}
			pos++; // uint8_t pacType  = buf[pos++];
			cmdID   = ((((uint16_t)buf[pos+1]) << 8) | (buf[pos+0]));
			pos += 2;
			//uint16_t seqID   = ((((uint16_t)buf[pos+1]) << 8) | (buf[pos]));
			pos += 2;
			//dataSize = size - 11;
			//data     = NULL;
			//if dataSize > 0:
			//    data = bytearray(dataSize)
			//    bb.get(data)
			//fprintf(stderr, "parsePacket(len=%d) mark=%02X size=%d dataSize=%d cmdId=%u\n", bufLen, mark, size, dataSize, cmdID);
			uint16_t crc16    = ((((uint16_t)buf[size-1]) << 8) | (buf[size-2]));
			uint16_t calcCrc16 = calcCRC16(buf, size - 2);
			if (crc16 != calcCrc16) {
				fprintf(stderr, "wrong CRC16 %04X / %04X\n", crc16, calcCrc16);
				cmdID = 0;
			}
			//print 'pt:{0:02x}, cmd:{1:4d}={2:04x}, seq:{3:04x}, data_sz:{4:d} - '.format(pacType, cmdID, cmdID, seqID, dataSize)
		} else
		if (mark == 0x63) {
			uint8_t ack[11];
			strcpy((char*)ack, "conn_ack:");
			pos = strlen((char*)ack);
			ack[pos++] = TELLO_PORT_VIDEO & 0xFF;
			ack[pos++] = TELLO_PORT_VIDEO >> 8;
			cmdID = TELLO_CMD_CONN_ACK;
		} else {
			fprintf(stderr, "wrong mark !! %02X\n", mark);
		}
	} else if (buf) {
		fprintf(stderr, "wrong packet length=%d, 1st byte=%02X\n", bufLen, buf[0]);
	}
	return cmdID;
}

bool CTello::sendCmd(uint8_t pacType, uint16_t cmdID, uint8_t * data, int dataLength)
{
	uint8_t buf[128]; // TODO: max size
	uint8_t payload[128]; // TODO: max size
	uint16_t seq = 0;
	unsigned int pos = 0;
	unsigned int payloadLength = 0;
        
        send_command_mtx.lock();

	if (cmdID == TELLO_CMD_CONN) {
		strcpy((char*)buf, "conn_req:");
		pos = strlen((char*)buf);
		buf[pos++] = TELLO_PORT_VIDEO >> 8;
		buf[pos++] = TELLO_PORT_VIDEO & 0xFF;
		seqID++;
	} else if (cmdID == TELLO_CMD_STICK) {
		CRealTime now;
		now.now();

		// put 64bit stick data (only 48bits)
#if 0
		//payload[payloadLength++] = (stickData >> 56) & 0xFF;
		//payload[payloadLength++] = (stickData >> 48) & 0xFF;
		payload[payloadLength++] = (stickData >> 40) & 0xFF;
		payload[payloadLength++] = (stickData >> 32) & 0xFF;
		payload[payloadLength++] = (stickData >> 24) & 0xFF;
		payload[payloadLength++] = (stickData >> 16) & 0xFF;
		payload[payloadLength++] = (stickData >> 8) & 0xFF;
		payload[payloadLength++] = stickData & 0xFF;
#else
		payload[payloadLength++] = stickData & 0xFF;
		payload[payloadLength++] = (stickData >> 8) & 0xFF;
		payload[payloadLength++] = (stickData >> 16) & 0xFF;
		payload[payloadLength++] = (stickData >> 24) & 0xFF;
		payload[payloadLength++] = (stickData >> 32) & 0xFF;
		payload[payloadLength++] = (stickData >> 40) & 0xFF;
#endif

		payload[payloadLength++] = now.hour();
		payload[payloadLength++] = now.min();
		payload[payloadLength++] = now.sec();
		//payload[payloadLength++] = now.microsecond() & 0xFF;
		//payload[payloadLength++] = (now.microsecond() >> 8) & 0xFF;
		payload[payloadLength++] = 0;
		payload[payloadLength++] = 0;
		data = payload;
		dataLength = payloadLength;
		seq = 0;
	} else if (cmdID == TELLO_CMD_DATE_TIME) {
		seq = seqID;
		CRealTime now;
		now.now();
		payload[payloadLength++] = 0x0;
		payload[payloadLength++] = now.year() & 0xFF;
		payload[payloadLength++] = (now.year() >> 8) & 0xFF;
		payload[payloadLength++] = now.month() & 0xFF;
		payload[payloadLength++] = (now.month() >> 8) & 0xFF;
		payload[payloadLength++] = now.day() & 0xFF;
		payload[payloadLength++] = (now.day() >> 8) & 0xFF;
		payload[payloadLength++] = now.hour() & 0xFF;
		payload[payloadLength++] = (now.hour() >> 8) & 0xFF;
		payload[payloadLength++] = now.min() & 0xFF;
		payload[payloadLength++] = (now.min() >> 8) & 0xFF;
		payload[payloadLength++] = now.sec() & 0xFF;
		payload[payloadLength++] = (now.sec() >> 8) & 0xFF;
		//payload[payloadLength++] = now.microsecond & 0xFF;
		//payload[payloadLength++] = (now.microsecond >> 8) & 0xFF;
		payload[payloadLength++] = 0;
		payload[payloadLength++] = 0;
		data = payload;
		dataLength = payloadLength;
		seqID++;
	} else if (cmdID == TELLO_CMD_REQ_VIDEO_SPS_PPS) {
		seq = 0;
	} else {
		seq = seqID;
		seqID++;
	}

	if (pos == 0) {
		pos = buildPacket(buf, pos, pacType, cmdID, seq, data, dataLength);
	}
	//char aux[32];
	//fprintf(stderr, "sendCmd(pacType=%02X, cmdId=%04X, data length=%d) send %d bytes to %s:%d\n", pacType, cmdID, dataLength, pos, inet_ntop(AF_INET, &cmdAddr.sin_addr.s_addr, aux, sizeof(aux)), ntohs(cmdAddr.sin_port));
	//for (unsigned int i = 0; i<pos; i++) fprintf(stderr, " %02X", buf[i]); fprintf(stderr, "\n");
	::sendto(fdSockCmd, buf, pos, 0, (struct sockaddr *) &cmdAddr, sizeof(cmdAddr));
        send_command_mtx.unlock();
	return true;
}

void CTello::sendTextCommand(char command[]){
    ::sendto(fdSockCmd, command, strlen(command), 0, (struct sockaddr *) &cmdAddr, sizeof(cmdAddr));
}

void CTello::cmdRxThreadBody()
{
	//fprintf(stderr, "rx thread started\n");
	int statusCtr = 0;
	uint8_t data[1024];

	while (!threadInterrupt) {
		struct sockaddr_in senderAddr;
		socklen_t senderLen = sizeof(senderAddr);
		//fprintf(stderr, "rx thread recv ...\n");
		int r = ::recvfrom(fdSockCmd, data, sizeof(data), 0, (struct sockaddr*) &senderAddr, &senderLen); 
		//fprintf(stderr, "rx thread received %d\n", r);
		if (r < 0) {
			fprintf(stderr, "RX ERROR: %s\n", strerror(errno));
			break;
		}
		uint16_t cmdID = parsePacket(data, r);
		uint8_t * payload = data + 9;
		int payloadLength = r - 11;

		if (cmdID == TELLO_CMD_CONN_ACK) {
			fprintf(stderr, "connection successful !\n");
		}
		else if (cmdID == TELLO_CMD_DATE_TIME) {
			sendCmd(0x50, cmdID, NULL);
		}

		else if (cmdID == TELLO_CMD_STATUS) {
			if (statusCtr == 3) {
				sendCmd(0x60, TELLO_CMD_REQ_VIDEO_SPS_PPS, NULL);
				sendCmd(0x48, TELLO_CMD_VERSION_STRING, NULL);
				sendCmd(0x48, TELLO_CMD_SET_VIDEO_BIT_RATE, NULL);
				sendCmd(0x48, TELLO_CMD_ALT_LIMIT, NULL);
				sendCmd(0x48, TELLO_CMD_LOW_BATT_THRESHOLD, NULL);
				sendCmd(0x48, TELLO_CMD_ATT_ANGLE, NULL);
				sendCmd(0x48, TELLO_CMD_REGION, NULL);
				sendCmd(0x48, TELLO_CMD_SET_EV, (uint8_t*) "\0", 1);
			}
			statusCtr = statusCtr + 1;
		}

		else if (cmdID == TELLO_CMD_VERSION_STRING) {
			if (r >= 42) {
				data[30] = 0;
				fprintf(stderr, "Version: '%s'\n", data+10);
			}
		}

		else if (cmdID == TELLO_CMD_SMART_VIDEO_START) {
			if (payloadLength > 0) {
				fprintf(stderr, "smart video start\n");
			}
		}

		else if (cmdID == TELLO_CMD_ALT_LIMIT) {
			if (payloadLength > 0) {
				//payload.get_ULInt8(); // 0x00
				uint16_t height = (((uint16_t)payload[2]) << 8) | payload[1];
				fprintf(stderr, "alt limit : %2d meter\n", height);

				if (height != NEW_ALT_LIMIT) {
					fprintf (stderr, "set new alt limit : %2d meter\n", NEW_ALT_LIMIT);
					uint8_t msg[2];
					msg[1] = NEW_ALT_LIMIT & 0xFF;
					msg[0] = (NEW_ALT_LIMIT >> 8) & 0xFF;
					sendCmd(0x68, TELLO_CMD_SET_ALT_LIMIT, msg, 2);
				}
			}
		}

		else if (cmdID == TELLO_CMD_SMART_VIDEO_STATUS) {
			if (payloadLength > 0) {
				uint8_t resp = payload[0];
				//uint8_t dummy = resp & 0x07;
				uint8_t start = (resp >> 3) & 0x03;
				uint8_t mode  = (resp >> 5) & 0x07;
				fprintf(stderr, "smart video status - mode:%d, start:%d\n", mode, start);
				sendCmd(0x50, TELLO_CMD_SMART_VIDEO_STATUS, (uint8_t*) "\0", 1);
			}
		} else if (data[0] >= '0' && data[0] <= 'z'){
                    data[r] = 0;
                    printf("received text: %s\n", data);
                }
	}
}

void CTello::timerThreadBody()
{
	fprintf(stderr, "timer thread started\n");
	int counter = 0;
	while (!threadInterrupt) {
                                
            sendCmd(0x60, TELLO_CMD_STICK);

            counter++;
            if (counter % 50 == 0) {
                    //sendCmd(0x60, TELLO_CMD_REQ_VIDEO_SPS_PPS);
            }

            usleep(STICKS_LOOP);
	}
}

/* end of tello.cc */
