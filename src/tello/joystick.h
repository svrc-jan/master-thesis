#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include <pthread.h>
#include <vector>
#if ! defined __CYGWIN__ && ! defined WIN32
#include <linux/joystick.h>
#else
// TODO: windows joystick support
	#include <stdint.h>
	typedef void js_event;
	typedef uint8_t __u8;
	typedef uint32_t __u32;
#endif

class CJoystick {
private:
	struct joystick_state {
		std::vector<signed short> button;
		std::vector<signed short> axis;
	};

	char deviceName[256];

	pthread_t thread;
	bool active;
	bool quit;
	int joystick_fd;
	joystick_state * joystick_st;
	js_event * joystick_ev;
	__u32 version;
	__u8 axes;
	__u8 buttons;
	char name[256];
	bool verbose;

	int intPipe[2]; // [read, write]
	void (*eventCallback)(CJoystick*);
	void (*errorCallback)(CJoystick*);
public:
	CJoystick(void);
	~CJoystick(void);

	bool initialize(const char * device = NULL);

	bool initialized() { return active; }

	void close();

	void setVerbose(bool enableVerbose) { verbose = enableVerbose; }

protected:
	bool internalInitialize(void);
	void interrupt();

	static void * loop(void * obj);
	bool readEv(void);

public:
	const char * getName() { return name; }

	int axis(unsigned short n) const;
	bool buttonPressed(unsigned short n) const;
	bool status() { return active; }

	int getAxisCount() { return axes; }
	int getButtonCount() { return buttons; }

	void registerJoystickCallback(void (*update_callback)(CJoystick*)) { eventCallback = update_callback; }
	void registerErrorCallback(void (*error_callback)(CJoystick*)) { errorCallback = error_callback; }
};

#endif

