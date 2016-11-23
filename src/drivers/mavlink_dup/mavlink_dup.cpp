/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_dup.cpp
 * NuttX driver to duplicate mavlink onto multiple serial devices, with a single
 * mavlink instance. It is implemented such that only a single serial device can
 * be connected at a time (receiving data). If nothing is read within 3 seconds,
 * the device is assumed to be disconnected (mavlink sends heartbeats every second),
 * and a write from mavlink will be output to all serial devices.
 */

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <px4_sem.hpp>
#include <px4_getopt.h>

#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdint>
#include <termios.h>
#include <errno.h>

#include <v2.0/mavlink_types.h>
#include <v2.0/standard/mavlink.h>

#include <nuttx/serial/tioctl.h> // for ioctl definitions


extern "C" __EXPORT int mavlink_dup_main(int argc, char *argv[]);

class MavlinkDuplicator;

namespace
{
static MavlinkDuplicator *object = nullptr;
}



/**
 * class MavlinkDuplicator
 * This is the frontend device that mavlink can connect to. The class contains & manages
 * the backend devices.
 * read() & write() can be called concurrently
 *
 * Assumptions:
 * - poll() is always used before a read
 * - no exotic ioctl()'s are used (in fact, they will just be ignored)
 */
class MavlinkDuplicator : public device::CDev
{
public:
	MavlinkDuplicator(const char *device_name, const char *device_path);
	virtual ~MavlinkDuplicator();

	/**
	 * Add a new backend serial device
	 * @param device_path serial device path
	 * @param baudrate
	 * @param preferred whether this should be the peferred device (there can be at most one).
	 *        If two backends have data to read and one is the preferred, the connection is switched
	 *        to the preferred.
	 * @return
	 */
	int add_backend_device(const char *device_path, int baudrate, bool preferred);

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int	open(file *filp);
	virtual int	close(file *filp);

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);

protected:

	int set_baudrate(int fd, int baudrate);

	/** try to write to fd, if there is enough write space left and return number of bytes written,
	 *  otherwise fail and return 0 */
	inline ssize_t try_write(int fd, const char *buffer, size_t buflen);

	/** switch to connected state if backend data read or disconnect on backend timeout */
	inline void update_connection_state();

	virtual pollevent_t poll_state(struct file *filp);


	void lock()
	{
		while (sem_wait(&_lock) != 0) {
			/* The only case that an error should occur here is if
			 * the wait was awakened by a signal.
			 */
			ASSERT(get_errno() == EINTR);
		}
	}

	void unlock()
	{
		sem_post(&_lock);
	}

	static const int max_num_backends = 3;

	struct BackendData {
		int fd;
		char *device_path;
		int baudrate;
	};
	BackendData _backends[max_num_backends];
	int _num_backends = 0;

	/* packet parsing state */
	uint16_t _packet_len;
	enum class ParserState : uint8_t {
		Idle = 0,
		GotLength
	};
	ParserState _parser_state = ParserState::Idle;

	volatile int8_t _connected_backend_reader = -1; /**< currently connected backend. if -1, not connected */
	volatile int8_t _connected_backend_writer = -1; /**< we need two for multi-threading */
	volatile int8_t _polling_backend = -1; /**< if not connected, backend which has data available to read */
	volatile hrt_abstime _last_got_data_timestamp = 0;

	int8_t _preferred_backend = -1;

	sem_t _lock;

private:
};

MavlinkDuplicator::MavlinkDuplicator(const char *device_name, const char *device_path)
	: CDev(device_name, device_path)
{
	for (int i = 0; i < max_num_backends; ++i) {
		_backends[i].fd = -1;
		_backends[i].device_path = nullptr;
		_backends[i].baudrate = 0;
	}

	sem_init(&_lock, 1, 1);
}

MavlinkDuplicator::~MavlinkDuplicator()
{
	for (int i = 0; i < _num_backends; ++i) {
		if (_backends[i].fd >= 0) {
			::close(_backends[i].fd);
		}

		if (_backends[i].device_path) {
			free(_backends[i].device_path);
		}
	}

	sem_destroy(&_lock);
}

int MavlinkDuplicator::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	//pretend we have enough space left to write, so mavlink will not drop data and throw off
	//our parsing state
	if (cmd == FIONWRITE) {
		*(int *)arg = 1024;
		return 0;
	}

	// We must make sure the ioctl's for setting the baudrate are not passed through,
	// because different backends can have different baudrates.
	// mavlink currently only uses these 2 ioctl's so just warn if we get a new one.
	if (cmd != TCGETS && cmd != TCSETS) {
		PX4_WARN("Got unknown ioctl %i, %lu", cmd, arg);
	}

	return 0;
}

int MavlinkDuplicator::add_backend_device(const char *device_path, int baudrate, bool preferred)
{
	if (_num_backends >= max_num_backends) {
		return -ENOMEM;
	}

	if (_num_backends > 0 && _backends[0].fd >= 0) {
		/* cannot add more if already opened (ie mavlink already started) */
		return -EADDRINUSE;
	}

	//we cannot open the device_path here, because read() & write() will be called from a different context
	_backends[_num_backends].device_path = strdup(device_path);
	_backends[_num_backends].baudrate = baudrate;

	if (preferred) {
		_preferred_backend = _num_backends;
	}

	++_num_backends;
	return 0;
}

int MavlinkDuplicator::open(file *filp)
{
	bool success = true;

	for (int i = 0; i < _num_backends; ++i) {
		if (_backends[i].fd == -1) {
			_backends[i].fd = ::open(_backends[i].device_path, O_RDWR | O_NOCTTY);

			if (_backends[i].fd == -1) {
				PX4_ERR("Failed to open %s (%i)", _backends[i].device_path, errno);
				success = false;

			} else {
				int ret = set_baudrate(_backends[i].fd, _backends[i].baudrate);

				if (ret < 0) {
					PX4_ERR("Failed to set baudrate (%i) for %s", ret, _backends[i].device_path);
				}
			}
		}
	}

	CDev::open(filp);
	return success ? 0 : -1;
}

int MavlinkDuplicator::set_baudrate(int fd, int baudrate)
{
	int speed;

	switch (baudrate) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 500000: speed = B500000; break;

#ifdef __PX4_NUTTX

	case 750000: speed = 750000; break;
#endif

	case 921600: speed = B921600; break;

	case 1000000: speed = B1000000; break;

	case 1500000: speed = B1500000; break;

	default:
		PX4_ERR("Unsupported baudrate: %d", baudrate);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* Initialize the uart config */
	if ((termios_state = tcgetattr(fd, &uart_config)) < 0) {
		return -errno;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		return -errno;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		return -errno;
	}

	return 0;
}

int MavlinkDuplicator::close(file *filp)
{
	/* calling ::close(_backends[i].fd) here results in a dead-lock, because MavlinkDuplicator::close()
	 * is called from within another close(), and NuttX seems to hold a semaphore at this point.
	 * solution: let NuttX cleanup open fd's
	 */

	CDev::close(filp);
	return 0;
}

pollevent_t MavlinkDuplicator::poll_state(struct file *filp)
{
	/* Here we should just check the poll state (which is called before an actual poll waiting).
	 * Instead we poll on the fd with some timeout, and then pretend that there is data.
	 * This will let the calling poll return immediately (there's still no busy loop since
	 * we do actually poll here).
	 * We do this because there is no simple way with the given interface to poll on
	 * the fd in here or by overriding some other method.
	 */

	static_assert(max_num_backends >= 2, "Need at least two backends");

	pollfd fds[max_num_backends];
	int num_polling_fd;

	if (_connected_backend_reader >= 0) {
		fds[0].fd = _backends[_connected_backend_reader].fd;
		fds[0].events = POLLIN;
		num_polling_fd = 1;

		if (_preferred_backend >= 0 && _preferred_backend != _connected_backend_reader) {
			fds[1].fd = _backends[_preferred_backend].fd;
			fds[1].events = POLLIN;
			++num_polling_fd;
		}

	} else {
		for (int i = 0; i < _num_backends; ++i) {
			fds[i].fd = _backends[i].fd;
			fds[i].events = POLLIN;
		}

		num_polling_fd = _num_backends;
	}

	::poll(fds, num_polling_fd, 100);

	// update state
	if (_connected_backend_reader >= 0) {
		if (fds[0].revents & POLLIN) {
			_polling_backend = _connected_backend_reader;

		} else if (num_polling_fd == 2 && (fds[1].revents & POLLIN)) {
			_polling_backend = _preferred_backend; //switch to preferred

		} else {
			_polling_backend = -1;
		}

	} else {
		_polling_backend = -1;

		for (int i = 0; i < _num_backends; ++i) {
			if (fds[i].revents & POLLIN) {
				_polling_backend = i;
				break;
			}
		}
	}

	return POLLIN;
}


ssize_t MavlinkDuplicator::read(struct file *filp, char *buffer, size_t buflen)
{
	if (_num_backends == 0) {
		return 0;
	}

	ssize_t ret = 0;

	if (_polling_backend >= 0) {
		ret = ::read(_backends[_polling_backend].fd, buffer, buflen);
	}

	lock();

	if (ret > 0) {
		_last_got_data_timestamp = hrt_absolute_time();
	}

	//at this point we can synchronize the state
	if (_connected_backend_reader != _connected_backend_writer) {
		_connected_backend_reader = _connected_backend_writer;
	}

	unlock();

	return ret;
}

ssize_t MavlinkDuplicator::write(struct file *filp, const char *buffer, size_t buflen)
{
	ssize_t ret = 0;

	switch (_parser_state) {
	case ParserState::Idle:
		ASSERT(buflen >= 3);

		if ((unsigned char)buffer[0] == MAVLINK_STX) { // mavlink 2
			uint8_t payload_len = buffer[1];
			uint8_t incompat_flags = buffer[2];
			_packet_len = payload_len + 12;

			if (incompat_flags & 0x1) { //signing
				_packet_len += 13;
			}

			_parser_state = ParserState::GotLength;

		} else if ((unsigned char)buffer[0] == MAVLINK_STX_MAVLINK1) { // mavlink 1
			uint8_t payload_len = buffer[1];
			_packet_len = payload_len + 8;

			_parser_state = ParserState::GotLength;

		} else {
			PX4_ERR("parser error"); /* this should really be an assertion, ie if this fails, the implementation is wrong */
			errno = EINVAL;
			return -1;
		}

	//no break
	case ParserState::GotLength: {
			_packet_len -= buflen;

			/*
			// if _preferred_backend == -1, we could only write to the connected backend with the following code.
			// But if there is a preferred backend, we always need to write to the preferred as well.
			if (_connected_backend_writer >= 0) {
				ret = try_write(_backends[_connected_backend_writer].fd, buffer, buflen);
			} else {
			*/
			ret = buflen;

			for (int i = 0; i < _num_backends; ++i) {
				ssize_t ret_cur = try_write(_backends[i].fd, buffer, buflen);

				if (ret_cur < ret) { //use the minimum (mavlink uses it only for error statistics)
					ret = ret_cur;
				}
			}

			if (_packet_len == 0) {
				_parser_state = ParserState::Idle;
				update_connection_state();
			}
		}

		break;
	}

	return ret;
}

ssize_t MavlinkDuplicator::try_write(int fd, const char *buffer, size_t buflen)
{

	int buf_free;
	::ioctl(fd, FIONWRITE, (unsigned long)&buf_free);

	if (buf_free < buflen) {
		return 0;
	}

	return ::write(fd, buffer, buflen);
}

void MavlinkDuplicator::update_connection_state()
{
	lock();

	if (_connected_backend_writer >= 0) {
		if (hrt_elapsed_time(&_last_got_data_timestamp) > 3000 * 1000) {
			PX4_INFO("backend %i disconnected", (int)_connected_backend_writer);
			_connected_backend_writer = -1;

		} else if (_polling_backend != -1 && _polling_backend != _connected_backend_writer) {
			_connected_backend_writer = _polling_backend;
			PX4_INFO("Switching to backend %i", (int)_connected_backend_writer);
		}

	} else {
		if (_polling_backend >= 0) {
			_connected_backend_writer = _polling_backend;
			PX4_INFO("backend %i connected", (int)_connected_backend_writer);
		}
	}

	unlock();
}


int mavlink_dup_main(int argc, char *argv[])
{
	const char *device = "/dev/mavlink";
	int baudrate = 57600;
	int myoptind = 1;
	int ch;
	const char *myoptarg = NULL;
	bool preferred = false;

	if (argc < 2) {
		goto out;
	}

	if (!strcmp(argv[1], "start")) {
		if (object) {
			PX4_ERR("already running");
			return -1;
		}

		while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'd':
				device = myoptarg;
				break;

			default:
				goto out;
			}
		}

		PX4_INFO("Starting on %s", device);

		object = new MavlinkDuplicator("MavlinkDup", device);

		if (!object) {
			PX4_ERR("alloc failed");
			return -1;
		}

		object->init();
	}

	if (!strcmp(argv[1], "add")) {
		if (!object) {
			PX4_ERR("not started");
			return -1;
		}

		while ((ch = px4_getopt(argc, argv, "b:d:p", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'b':
				baudrate = strtoul(myoptarg, nullptr, 10);
				break;

			case 'd':
				device = myoptarg;
				break;

			case 'p':
				preferred = true;
				break;

			default:
				goto out;
			}
		}

		PX4_INFO("Adding %s, baud=%i %s", device, baudrate, preferred ? "(preferred)" : "");
		int ret = object->add_backend_device(device, baudrate, preferred);

		if (ret != 0) {
			PX4_ERR("Failed to add %s (%i)", device, baudrate);
		}
	}

	if (!strcmp(argv[1], "stop")) {
		if (object) {
			delete object;
			object = nullptr;
		}
	}

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		if (object) {
			PX4_INFO("running");

		} else {
			PX4_INFO("not running");
		}
	}

	return 0;

out:
	PX4_ERR("unrecognized command, try 'start -d <device>', 'add -d <device> -b <baud> [-p]', 'stop', 'status'");
	PX4_ERR("     -p:  if set, this is the preferred backend");
	return 1;
}

