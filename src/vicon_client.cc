/*
 * File name: vicon_client.cc
 * Date:      2018/10/26 16:11
 * Author:    Jan Chudoba
 */

#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include <netinet/in.h>
#include <arpa/inet.h>


#include "vicon_client.h"

CViconClient::CViconClient() :
	fd(-1),
	lineLength(0)
{
}

CViconClient::~CViconClient()
{
	close();
}

bool CViconClient::connect(const char * hostName, int port)
{
	strncpy(serverHostName, hostName, sizeof(serverHostName));
	serverHostPort = port;
	return internalConnect();
}


bool CViconClient::internalConnect()
{
	struct sockaddr_in addr;
	int res;

	fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (fd < 0){
		return false;
	}

	memset(&addr, 0, sizeof addr);

	addr.sin_family = AF_INET;
	addr.sin_port = htons(serverHostPort);
	res = inet_pton(AF_INET, serverHostName, &addr.sin_addr); // TODO: DNS

	if (res <= 0) {
		::close(fd);
		fd = -1;
		return false;
	}

	if (::connect(fd, (struct sockaddr *)&addr, sizeof addr) < 0){
		::close(fd);
		fd = -1;
		return false;
	}

	return true;
}

void CViconClient::close()
{
	if (fd >= 0) {
		::shutdown(fd, SHUT_RDWR);
		::close(fd);
		fd = -1;
	}
}

bool CViconClient::waitForData(CViconObject & object)
{
	bool result = false;
	while (true) {
		if (!connected()) {
			// check connection and re-connect if necessary
			usleep(5000000);
			if (!internalConnect()) continue;
			fprintf(stderr, "CViconClient re-connected\n");
		}
		char c;
		int r = recv(&c, 1); // TODO: ineffective
		if (r < 0) {
			fprintf(stderr, "CViconClient read error: %s\n", strerror(errno));
			close();
			break;
		}
		if (c == '\n') {
			if (lineLength > 0) {
				line[lineLength] = 0;
				lineLength = 0;
				result = true;
			}
			break;
		}
		if (lineLength < sizeof(line)) {
			line[lineLength++] = c;
		}
	}
	if (result) {
		char * pos = NULL;
		object.frameNumber = parseInt(line, pos, result);
		object.frameRate = parseDouble(line, pos, result);
		//object.timestampHours = parseInt(line, pos, result);
		//object.timestampMinutes = parseInt(line, pos, result);
		//object.timestampSeconds = parseInt(line, pos, result);
		//object.timestampFrames = parseInt(line, pos, result);
		parseString(line, pos, object.name, result);
		object.occluded = (parseInt(line, pos, result) > 0);
		object.translation[0] = parseDouble(line, pos, result);
		object.translation[1] = parseDouble(line, pos, result);
		object.translation[2] = parseDouble(line, pos, result);
		if (result) {
			for (int i = 0; i < 9; i++) {
				object.rotation[i] = parseDouble(line, pos, result);
				if (!result) {
					if (i >= 2) {
						result = true;
					}
					break;
				}
				object.rotationComponents = i + 1;
			}
		}
	}
	return result;
}

int CViconClient::parseInt(char * line, char *& pos, bool & result)
{
	int value;
	char * token = strtok_r(pos ? NULL : line, " ", &pos);
	result = (token != NULL);
	if (token) {
		value = atoi(token);
	} else value = 0;
	return value;
}

double CViconClient::parseDouble(char * line, char *& pos, bool & result)
{
	double value;
	char * token = strtok_r(pos ? NULL : line, " ", &pos);
	result = (token != NULL);
	if (token) {
		value = atof(token);
	} else value = 0;
	return value;
}

void CViconClient::parseString(char * line, char *& pos, std::string & value, bool & result)
{
	char * token = strtok_r(pos ? NULL : line, " ", &pos);
	result = (token != NULL);
	if (token) {
		value = token;
	}
}

/* end of vicon_client.cc */
