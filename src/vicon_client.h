/*
 * File name: vicon_client.h
 * Date:      2018/10/26 16:02
 * Author:    Jan Chudoba
 */

#ifndef __VICON_CLIENT_H__
#define __VICON_CLIENT_H__

#define VICON_CLIENT_INPUT_BUFFER_LENGTH 1024

#include <string>

#include <sys/socket.h>

class CViconObject
{
public:
	int frameNumber;
	double frameRate;
#if 0
	int timestampHours;
	int timestampMinutes;
	int timestampSeconds;
	int timestampFrames;
#endif
	std::string name;
	bool occluded;
	double translation[3];
	double rotation[9];
	int rotationComponents; // 3 for euler, 4 for quaterion, 9 for matrix
	// TODO: occluded, quality
};

class CViconClient
{
public:
	CViconClient();
	~CViconClient();

	//void registerObjectCallback(void (*cb)(CViconObject & o));

	bool connect(const char * hostName, int port);
	void close();

	bool connected() { return (fd >= 0); }

	bool waitForData(CViconObject & object);

protected:
	bool internalConnect();
	int send(char * data, int length) { return ::send(fd, data, length, 0); }
	int recv(char * buffer, int bufferLength) { return ::recv(fd, buffer, bufferLength, 0); }

	int parseInt(char * line, char * & pos, bool & result);
	double parseDouble(char * line, char *& pos, bool & result);
	void parseString(char * line, char *& pos, std::string & value, bool & result);
private:
	int fd;
	//void (*object_callback)(CViconObject & o);
	char line[VICON_CLIENT_INPUT_BUFFER_LENGTH];
	unsigned int lineLength;

	char serverHostName[128];
	int serverHostPort;
};

#endif

/* end of vicon_client.h */
