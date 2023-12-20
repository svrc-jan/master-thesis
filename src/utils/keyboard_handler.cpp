#include "keyboard_handler.hpp"


void turn_off_console_echo()
{
	struct termios term;
    tcgetattr(fileno(stdin), &term);

    term.c_lflag &= ~ECHO;
    tcsetattr(fileno(stdin), 0, &term);
}

void turn_on_console_echo()
{
	struct termios term;
	tcgetattr(fileno(stdin), &term);

    term.c_lflag |= ECHO;
    tcsetattr(fileno(stdin), 0, &term);
}

void keyboard_handler_func(Keyboard_handler *hndl)
{
	int fd;
	struct input_event ie;

	if ((fd = open(hndl->device_name.c_str(), O_RDONLY)) == -1) {
		cerr << "Cannot find keyboard device: \"" << hndl->device_name << "\"" << endl;
		exit(EXIT_FAILURE);
	}

	turn_off_console_echo();

	while(read(fd,&ie,sizeof(struct input_event)) && !hndl->done) {
		if (ie.type = EV_KEY) {
			hndl->array_mtx.lock();
			hndl->array[ie.code] = ie.value;
			hndl->array_mtx.unlock();
		}
	}
	
	turn_on_console_echo();
}

bool Keyboard_handler::get_val(size_t code)
{
	bool rv;

	this->array_mtx.lock();
	rv = array[code];
	this->array_mtx.unlock();

	return rv;
}

Keyboard_handler::Keyboard_handler(string device)
{
	this->device_name = device;
	this->handler_thread = thread(keyboard_handler_func, this);
}

Keyboard_handler::~Keyboard_handler()
{
	this->done = true;
	this->handler_thread.join();
}

