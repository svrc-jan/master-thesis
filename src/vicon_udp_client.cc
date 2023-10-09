/*
 * File name: vicon_udp_client.cc
 * Date:      2018/12/06 18:44
 * Author:    Jan Chudoba
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <sys/socket.h>
#include <arpa/inet.h>

#include "vicon_udp_client.h"

#define RECEIVE_BUFFER_SIZE 512

CViconUdpClient::CViconUdpClient() :
	sockfd(-1),
	threadStrarted(false),
	dataCallback(NULL)
{
}

CViconUdpClient::~CViconUdpClient()
{
	stop();
}

void CViconUdpClient::setDataCallback(TViconUdpDataCallback cb, void * context)
{
	dataCallback = cb;
	dataCallbackContext = context;
}

bool CViconUdpClient::start(int port)
{
	stop();

	sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		fprintf(stderr, "Failed to create socket: %s\n", strerror(errno));
		return false;
	}

	int so_reuse_addr_value = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &so_reuse_addr_value, sizeof(so_reuse_addr_value));

	struct sockaddr_in localAddr;
	memset(&localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET;
	localAddr.sin_port = htons(port);
	localAddr.sin_addr.s_addr = INADDR_ANY;

	if (::bind(sockfd, (struct sockaddr*) &localAddr, sizeof(localAddr)) < 0) {
		fprintf(stderr, "Failed to bind socket: %s\n", strerror(errno));
		::close(sockfd);
		sockfd = -1;
		return false;
	}

	if (!startThread()) {
		::close(sockfd);
		sockfd = -1;
		return false;
	}

	return true;
}

void CViconUdpClient::stop()
{
	threadInterrupt = true;
	if (sockfd >= 0) {
		::shutdown(sockfd, SHUT_RDWR);
		::close(sockfd);
		sockfd = -1;
	}
	if (threadStrarted) {
		pthread_join(thread, 0);
		threadStrarted = false;
	}
}

bool CViconUdpClient::startThread()
{
	threadStrarted = false;
	threadInterrupt = false;
	if (pthread_create(&thread, 0, thread_body, this) == 0) {
		threadStrarted = true;
		return true;
	}
	fprintf(stderr, "ERROR: Failed to start thread: %s\n", strerror(errno));
	return false;
}

void CViconUdpClient::threadBody()
{
	while (!threadInterrupt) {
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));
		socklen_t addrSize = sizeof(addr);
		char buffer[RECEIVE_BUFFER_SIZE];
		//fprintf(stderr, "DEBUG: recv\n");
		int r = ::recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*) &addr, &addrSize);
		if (r < 0) {
			fprintf(stderr, "receive error: %s\n", strerror(errno));
			break;
		}
		uint32_t frameNumber = (*((uint32_t*)buffer));
		int pos = 4;
		uint8_t itemsInBlock = buffer[pos++];
		//fprintf(stderr, "DEBUG: received udp packet data length %d, frame %u, items %u\n", r, frameNumber, itemsInBlock);
		do {
			r -= pos;
			for (uint8_t i = 0; i < itemsInBlock; i++) {
				if (r < 2) break;
				uint8_t itemID = buffer[pos++];
				if (itemID != 0) break;
				uint8_t itemDataSize = buffer[pos++];
				if (itemDataSize != 72) break;
				pos++;
				r -= 3;
				if ((unsigned) r < itemDataSize) break;
				CViconUdpData data;
				data.frameNumber = frameNumber;
				memcpy(data.name, buffer + pos, VICON_UDP_DATA_NAME_LENGTH);
				data.name[VICON_UDP_DATA_NAME_LENGTH] = 0;
				pos += VICON_UDP_DATA_NAME_LENGTH;
				data.x = 1e-3 * (*((double*)(buffer+pos))); pos += 8;
				data.y = 1e-3 * (*((double*)(buffer+pos))); pos += 8;
				data.z = 1e-3 * (*((double*)(buffer+pos))); pos += 8;
				data.rotX = (*((double*)(buffer+pos))); pos += 8;
				data.rotY = (*((double*)(buffer+pos))); pos += 8;
				data.rotZ = (*((double*)(buffer+pos))); pos += 8;
				//fprintf(stderr, "FRAME %u OBJECT '%s' %.3f, %.3f, %.3f / %.3f, %.3f, %.3f\n", frameNumber, data.name, data.x, data.y, data.z, data.rotX, data.rotY, data.rotZ);
				if (dataCallback) {
					dataCallback(data, dataCallbackContext);
				}
			}
		} while (false);
	}
}

/* end of vicon_udp_client.cc */
