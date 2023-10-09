#include <cstdio>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "joystick.h"

#if ! defined __CYGWIN__ && ! defined WIN32

CJoystick::CJoystick(void) :
	eventCallback(NULL),
	errorCallback(NULL)
{
	active = false;
	quit = false;
	version = axes = buttons = 0;
	joystick_fd = -1;
	joystick_ev = new js_event();
	joystick_st = new joystick_state();

	strcpy(deviceName, "/dev/input/js0");
	verbose = true;
	//initialize();
}

CJoystick::~CJoystick(void)
{
	close();
	delete joystick_ev;
	delete joystick_st;
}

void CJoystick::close()
{
	if (joystick_fd > 0) {
		active = false;
		quit = true;
		interrupt();
		pthread_join(thread, 0);
		::close(joystick_fd);
		joystick_fd = -1;
	}
	joystick_fd = 0;
	if (intPipe[0] > 0) ::close(intPipe[0]);
	if (intPipe[1] > 0) ::close(intPipe[1]);
}

bool CJoystick::initialize(const char * device)
{
	if (device != NULL) {
		strcpy(deviceName, device);
	}
	if (initialized()) return true;
	if (internalInitialize()) {
		if (pipe(intPipe) != 0) {
			fprintf(stderr, "CJoystick: ERROR: can not create interrupt pipe: %s\n", strerror(errno));
			intPipe[0] = -1;
			intPipe[1] = -1;
		}
		usleep(1000000);
		pthread_create(&thread, 0, &CJoystick::loop, this);

		return true;
	}
	return false;
}

bool CJoystick::internalInitialize(void)
{
	bool result = false;
	if (joystick_fd > 0) {
		::close(joystick_fd);
	}
	joystick_fd = open(deviceName, O_RDONLY);
	if (joystick_fd == -1) {
		if (verbose) fprintf(stderr, "Cannot open device %s: %s\n", deviceName, strerror(errno));
	}
	version = axes = buttons = 0;
	if (joystick_fd > 0) {
		memset(name, 0, sizeof(name));
		ioctl(joystick_fd, JSIOCGNAME(256), name);
		ioctl(joystick_fd, JSIOCGVERSION, &version);
		ioctl(joystick_fd, JSIOCGAXES, &axes);
		ioctl(joystick_fd, JSIOCGBUTTONS, &buttons);
		if (verbose) {
			fprintf (stderr, "JOY    Name: %s\n", name);
			fprintf (stderr, "JOY Version: %d\n", version);
			fprintf (stderr, "JOY    Axes: %d\n", (int)axes);
			fprintf (stderr, "JOY Buttons: %d\n", (int)buttons);
		}
		joystick_st->axis.reserve(axes);
		joystick_st->button.reserve(buttons);

		active = true;
		result = true;
	}
	return result;
}

void CJoystick::interrupt()
{
	char c = 1;
	int unused __attribute__((unused));
	unused = write(intPipe[1], &c, 1);
}

void * CJoystick::loop(void * obj) {
	CJoystick * j = reinterpret_cast<CJoystick *>(obj);
	while(!j->quit) {
		if (!j->readEv()) {
			if (j->verbose) fprintf(stderr, "CJoystick: READ ERROR\n");
			//fprintf(stderr, "CJoystick TERMINATING\n");
			//break;
			// try to re-connect joystick
			j->active = false;
			if (j->errorCallback) {
				j->errorCallback(j);
			}
			while (!j->active && !j->quit) {
				usleep(1000000);
				if (j->internalInitialize()) {
					if (j->verbose) fprintf(stderr, "CJoystick RE-INITIALIZED\n");
				}
			}
		} else {
			//fprintf(stderr, "CJoystick data read\n");
			if (j->eventCallback) {
				j->eventCallback(j);
			}
		}
	}
	j->active = false;
	return 0;
}

static int max(int a, int b) { return (a>b)?a:b; }

bool CJoystick::readEv(void)
{
	bool result;
	fd_set rfds;
	//struct timeval tv;
	int retval;

	FD_ZERO(&rfds);
	FD_SET(joystick_fd, &rfds);
	FD_SET(intPipe[0], &rfds);

	//tv.tv_sec = 5;
	//tv.tv_usec = 0;

	retval = select(1+max(joystick_fd, intPipe[0]), &rfds, NULL, NULL, NULL);

	if (retval > 0 && FD_ISSET(joystick_fd, &rfds)) {
		int bytes = read(joystick_fd, joystick_ev, sizeof(*joystick_ev));
		result = (bytes > 0);
		if (bytes > 0) {
			joystick_ev->type &= ~JS_EVENT_INIT;
			if (joystick_ev->type & JS_EVENT_BUTTON)
				joystick_st->button[joystick_ev->number] = joystick_ev->value;

			if (joystick_ev->type & JS_EVENT_AXIS)
				joystick_st->axis[joystick_ev->number] = joystick_ev->value;
		}
	} else {
		result = true;
	}
	return result;
}

int CJoystick::axis(unsigned short n) const
{
	return (n < axes) ? joystick_st->axis[n] : 0;
}

bool CJoystick::buttonPressed(unsigned short n) const
{
	return n < buttons ? joystick_st->button[n] : false;
}

#else // ! defined __CYGWIN__ && ! defined WIN32
CJoystick::CJoystick(void) {}
CJoystick::~CJoystick(void) {}
bool CJoystick::initialize(void) { return false;}
void CJoystick::interrupt() {}
void * CJoystick::loop(void * obj) { return NULL; }
bool CJoystick::readEv(void) {return false;}
int CJoystick::axis(unsigned short n) const { return 0; }
bool CJoystick::buttonPressed(unsigned short n) const { return false; }

#endif
