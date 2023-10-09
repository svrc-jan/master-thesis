/*
 * File name: vicon_udp_client.h
 * Date:      2018/12/06 18:38
 * Author:    Jan Chudoba
 */

#ifndef __VICON_UDP_CLIENT_H__
#define __VICON_UDP_CLIENT_H__

#define VICON_UDP_DATA_NAME_LENGTH 24

#include <stdint.h>
#include <pthread.h>

class CViconUdpData
{
public:
	uint32_t frameNumber;
	char name[1+VICON_UDP_DATA_NAME_LENGTH];
	double x, y, z;
	double rotX, rotY, rotZ;
};

typedef void (*TViconUdpDataCallback)(CViconUdpData&, void*);

class CViconUdpClient
{
public:
	CViconUdpClient();
	~CViconUdpClient();

	void setDataCallback(TViconUdpDataCallback cb, void * context);
	bool start(int port);
	void stop();

protected:
	bool startThread();
	void threadBody();
	static void * thread_body(void * arg) { ((CViconUdpClient*)arg)->threadBody(); return 0; }

private:
	int sockfd;
	bool threadStrarted;
	bool threadInterrupt;
	pthread_t thread;
	TViconUdpDataCallback dataCallback;
	void * dataCallbackContext;
};

#endif

/* end of vicon_udp_client.h */
