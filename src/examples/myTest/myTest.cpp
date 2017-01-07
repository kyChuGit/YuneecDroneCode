#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <drivers/device/device.h>

class B;

class A : public device::CDev
{
public:
	A();
	~A();
	virtual int init();
	void cycle(void);
protected:
	friend class B;
private:
	int _uart_fd;
	int write_data(uint8_t *data, uint8_t len);
};

class B : public device::CDev
{
public:
	B(A *pA);
	~B();
	virtual int init();
	int send(uint8_t *buf, uint8_t size);
private:
	A *_pA;
};

A::A() :
	CDev("TestA", "/dev/TestA"),
	_uart_fd(-1)
{
	_uart_fd = ::open("/dev/ttyS4", O_RDWR | O_NOCTTY | O_NONBLOCK);
	PX4_INFO("fd: %d", _uart_fd);
}

A::~A()
{
//	::close(_uart_fd);
}

void A::cycle(void)
{
	while(1) {
		usleep(1000000);
	}
}

int A::init() {
	int ret = CDev::init();
	if(ret != OK) {
		DEVICE_DEBUG("A device init failed");
	}
	return ret;
}

B::B(A *pA) :
		CDev("TestB", "/dev/TestB"),
		_pA(pA)
{

}

B::~B() {}

int B::init() {
	int ret = CDev::init();
	if(ret != OK) {
		DEVICE_DEBUG("B device init failed");
	}
	return ret;
}

int B::send(uint8_t *buf, uint8_t size) {

	int ret = _pA->write_data(buf, size);
	if (ret < 1) {
		PX4_WARN("send failed");
	}
	return ret;
}

int A::write_data(uint8_t *data, uint8_t len)
{
	_uart_fd = ::open("/dev/ttyS4", O_RDWR | O_NOCTTY | O_NONBLOCK);
	int ret = ::write(_uart_fd, data, 9);
	::close(_uart_fd);
	if(ret < 1) {
		PX4_WARN("write uart error");
	}
	return ret;
}

namespace g_test{
A *a;
B *b;
};

extern "C" __EXPORT int myTest_main(int argc, char *argv[]);

int myTest_main(int argc, char *argv[]) {

	uint8_t TestData[9] = {0xfe, 0x05, 0x03, 0x97, 0x04, 0x7d, 0x00, 0x1e, 0x2f};

	if(g_test::a == nullptr || g_test::b == nullptr)
	{
		g_test::a = new A();
		g_test::b = new B(g_test::a);

		if(g_test::a->init() != OK || g_test::b->init() != OK) {
			PX4_WARN("device A & B init failed");
			return 1;
		}
	}

	int ret = g_test::b->send(TestData, 9);
	if(ret < 1) {
		PX4_WARN("B send error");
	}
	else {
		PX4_INFO("send %d", ret);
	}

	return 0;
}
