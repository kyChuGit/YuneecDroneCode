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

#include <px4_config.h>
#include <debug.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <termios.h>

#include <drivers/device/device.h>
#include <drivers/drv_tone_alarm.h>
#include <uORB/topics/actuator_armed.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include "../drv_tap_esc.h"

class ESC_Tune : public device::CDev
{
public:
	ESC_Tune(const char *path, const char *device, uint8_t power);
	~ESC_Tune();

	virtual int		init();
//	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

private:
	int _fd;
	const char *_device;
	static const uint8_t crcTable[256];
	static const unsigned	_tune_max = 1024 * 8; // be reasonable about user tunes
	const char		 *_default_tunes[TONE_NUMBER_OF_TUNES];
	const char		 *_tune_names[TONE_NUMBER_OF_TUNES];
	static const uint8_t	_note_tab[];

	int				_armed_sub;
	static actuator_armed_s	_armed;

	uint8_t			_power;
	unsigned		_default_tune_number; // number of currently playing default tune (0 for none)

	const char		*_user_tune;

	const char		*_tune;		// current tune string
	const char		*_next;		// next note in the string

	unsigned		_tempo;
	unsigned		_note_length;
	enum { MODE_NORMAL, MODE_LEGATO, MODE_STACCATO} _note_mode;
	unsigned		_octave;
	unsigned		_silence_length; // if nonzero, silence before next note
	bool			_repeat;	// if true, tune restarts at end

	hrt_call		_note_call;	// HRT callout for note completion

	uint8_t crc8_cal(uint8_t *p, uint8_t len);
	uint8_t crc_packet(EscPacket &p);
	int send_packet(EscPacket &p);
	void send_tone(uint16_t freq, uint16_t len, uint8_t pwr);
	float 			note_to_frequency(unsigned note);

	// Calculate the duration in microseconds of play and silence for a
	// note given the current tempo, length and mode and the number of
	// dots following in the play string.
	//
	unsigned		note_duration(unsigned &silence, unsigned note_length, unsigned dots);

	// Calculate the duration in microseconds of a rest corresponding to
	// a given note length.
	//
	unsigned		rest_duration(unsigned rest_length, unsigned dots);

	// Start playing the note
	//
	void			play_note(unsigned note, unsigned duration);

	// Start playing the tune
	//
	void			start_tune(const char *tune);

	// Parse the next note out of the string and play it
	//
	void			next_note();

	// Find the next character in the string, discard any whitespace and
	// return the canonical (uppercase) version.
	//
	int				next_char();

	// Extract a number from the string, consuming all the digit characters.
	//
	unsigned		next_number();

	// Consume dot characters from the string, returning the number consumed.
	//
	unsigned		next_dots();

	// hrt_call trampoline for next_note
	//
	static void		next_trampoline(void *arg);
};

const uint8_t ESC_Tune::crcTable[256] = TAP_ESC_CRC;
// semitone offsets from C for the characters 'A'-'G'
const uint8_t ESC_Tune::_note_tab[] = {9, 11, 0, 2, 4, 5, 7};
actuator_armed_s ESC_Tune::_armed = {};

#define TONEALARM1_DEVICE_PATH "/dev/esc_tone0"

ESC_Tune::ESC_Tune(const char *path, const char *device, uint8_t power) :
	CDev("esc_tune", path),
	_fd(-1),
	_device(device),
	_armed_sub(-1),
	_power(power),
	_default_tune_number(0),
	_user_tune(nullptr),
	_tune(nullptr),
	_next(nullptr),
	_tempo(120),
	_note_length(4),
	_note_mode(MODE_NORMAL),
	_octave(4),
	_silence_length(0),
	_repeat(false)
{
	_default_tunes[TONE_STARTUP_TUNE] = "MFT240L8 O4aO5dc O4aO5dc O4aO5dc L16dcdcdcdc";		// startup tune
	_default_tunes[TONE_ERROR_TUNE] = "MBT200a8a8a8PaaaP";						// ERROR tone
	_default_tunes[TONE_NOTIFY_POSITIVE_TUNE] = "MFT200e8a8a";					// Notify Positive tone
	_default_tunes[TONE_NOTIFY_NEUTRAL_TUNE] = "MFT200e8e";						// Notify Neutral tone
	_default_tunes[TONE_NOTIFY_NEGATIVE_TUNE] = "MFT200e8c8e8c8e8c8";				// Notify Negative tone
	_default_tunes[TONE_ARMING_WARNING_TUNE] = "MNT75L1O2G";					//arming warning
	_default_tunes[TONE_BATTERY_WARNING_SLOW_TUNE] = "MBNT100a8";					//battery warning slow
	_default_tunes[TONE_BATTERY_WARNING_FAST_TUNE] = "MBNT255a8a8a8a8a8a8a8a8a8a8a8a8a8a8a8a8";	//battery warning fast
	_default_tunes[TONE_GPS_WARNING_TUNE] = "MFT255L4AAAL1F#";					//gps warning slow
	_default_tunes[TONE_ARMING_FAILURE_TUNE] = "MFT255L4<<<BAP";
	_default_tunes[TONE_PARACHUTE_RELEASE_TUNE] = "MFT255L16agagagag";			// parachute release
	_default_tunes[TONE_EKF_WARNING_TUNE] = "MFT255L8ddd#d#eeff";				// ekf warning
	_default_tunes[TONE_BARO_WARNING_TUNE] = "MFT255L4gf#fed#d";				// baro warning
	_default_tunes[TONE_SINGLE_BEEP_TUNE] = "MFT100a8";                             // single beep
	_default_tunes[TONE_HOME_SET] = "MFT100L4>G#6A#6B#4";

	_tune_names[TONE_STARTUP_TUNE] = "startup";			// startup tune
	_tune_names[TONE_ERROR_TUNE] = "error";				// ERROR tone
	_tune_names[TONE_NOTIFY_POSITIVE_TUNE] = "positive";		// Notify Positive tone
	_tune_names[TONE_NOTIFY_NEUTRAL_TUNE] = "neutral";		// Notify Neutral tone
	_tune_names[TONE_NOTIFY_NEGATIVE_TUNE] = "negative";		// Notify Negative tone
	_tune_names[TONE_ARMING_WARNING_TUNE] = "arming";		// arming warning
	_tune_names[TONE_BATTERY_WARNING_SLOW_TUNE] = "slow_bat";	// battery warning slow
	_tune_names[TONE_BATTERY_WARNING_FAST_TUNE] = "fast_bat";	// battery warning fast
	_tune_names[TONE_GPS_WARNING_TUNE] = "gps_warning";	            // gps warning
	_tune_names[TONE_ARMING_FAILURE_TUNE] = "arming_failure";            //fail to arm
	_tune_names[TONE_PARACHUTE_RELEASE_TUNE] = "parachute_release";	// parachute release
	_tune_names[TONE_EKF_WARNING_TUNE] = "ekf_warning";				// ekf warning
	_tune_names[TONE_BARO_WARNING_TUNE] = "baro_warning";			// baro warning
	_tune_names[TONE_SINGLE_BEEP_TUNE] = "beep";                    // single beep
	_tune_names[TONE_HOME_SET] = "home_set";
}

ESC_Tune::~ESC_Tune()
{

}

int
ESC_Tune::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int result = OK;

	DEVICE_DEBUG("ioctl %i %u", cmd, arg);

//	irqstate_t flags = px4_enter_critical_section();

	/* decide whether to increase the alarm level to cmd or leave it alone */
	switch (cmd) {
	case TONE_SET_ALARM:
		DEVICE_DEBUG("TONE_SET_ALARM %u", arg);

		if (arg < TONE_NUMBER_OF_TUNES) {
			if (arg == TONE_STOP_TUNE) {
				// stop the tune
				_tune = nullptr;
				_next = nullptr;
				_repeat = false;
				_default_tune_number = 0;

			} else {
				/* always interrupt alarms, unless they are repeating and already playing */
				if (!(_repeat && _default_tune_number == arg)) {
					/* play the selected tune */
					_default_tune_number = arg;
					start_tune(_default_tunes[arg]);
				}
			}

		} else {
			result = -EINVAL;
		}

		break;

	default:
		result = -ENOTTY;
		break;
	}

//	px4_leave_critical_section(flags);

	/* give it to the superclass if we didn't like it */
	if (result == -ENOTTY) {
		result = CDev::ioctl(filp, cmd, arg);
	}

	return result;
}

int
ESC_Tune::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("esc tone init failed");
		return ret;
	}

	 _fd = ::open(_device, O_WRONLY);
	 if(_fd < 0) {
		 PX4_ERR("failed to open uart device");
		 return -1;
	 }

	// set baud rate
	int speed = 250000;
	struct termios uart_config;
	tcgetattr(_fd, &uart_config);

	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// set baud rate
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("failed to set baudrate for %s", _device);
		::close(_fd);
		return -1;
	}

	if (tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed for %s", _device);
		::close(_fd);
		return -1;
	}
	::close(_fd);

	return ret;
}

uint8_t ESC_Tune::crc8_cal(uint8_t *p, uint8_t len)
{
	uint8_t crc = 0;

	for (uint8_t i = 0; i < len; i++) {
		crc = crcTable[crc^*p++];
	}

	return crc;
}

uint8_t ESC_Tune::crc_packet(EscPacket &p)
{
	/* Calculate the crc over Len,ID,data */
	p.d.bytes[p.len] = crc8_cal(&p.len, p.len + 2);
	return p.len + offsetof(EscPacket, d) + 1;
}

int ESC_Tune::send_packet(EscPacket &packet)
{
	int packet_len = crc_packet(packet);
	_fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(_fd < 0) {
		PX4_WARN("Tune Open failed");
		return -1;
	}
	int ret = ::write(_fd, &packet.head, packet_len);
	if (ret < 1) {
		PX4_WARN("Tune ERR: ret: %d, errno: %d, fd: %d", ret, errno, _fd);
	} else {
		PX4_WARN("-------------------");
	}
	::close(_fd);
	return ret;
}

void ESC_Tune::send_tone(uint16_t freq, uint16_t len, uint8_t pwr)
{
	EscPacket packet = {0xfe, 5, ESCBUS_MSG_ID_TUNE};
	bool updated;

	if(_armed_sub < 0)
		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	orb_check(_armed_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
	if(!_armed.armed)
	{
		packet.d.bytes[0] = freq & 0x00FF;
		packet.d.bytes[1] = freq >> 8;
		packet.d.bytes[2] = len & 0x00FF;
		packet.d.bytes[3] = len >> 8;
		packet.d.bytes[4] = pwr;

		send_packet(packet);
	}
}

void
ESC_Tune::next_trampoline(void *arg)
{
	ESC_Tune *ta = (ESC_Tune *)arg;

	ta->next_note();
}

float
ESC_Tune::note_to_frequency(unsigned note)
{
	// compute the frequency (in Hz)
	float freq = 880.0f * expf(logf(2.0f) * ((int)note - 46) / 12.0f);
	return freq;
}

unsigned
ESC_Tune::note_duration(unsigned &silence, unsigned note_length, unsigned dots)
{
	unsigned whole_note_period = (60 * 1000000 * 4) / _tempo;

	if (note_length == 0) {
		note_length = 1;
	}

	unsigned note_period = whole_note_period / note_length;

	switch (_note_mode) {
	case MODE_NORMAL:
		silence = note_period / 8;
		break;

	case MODE_STACCATO:
		silence = note_period / 4;
		break;

	default:
	case MODE_LEGATO:
		silence = 0;
		break;
	}

	note_period -= silence;

	unsigned dot_extension = note_period / 2;

	while (dots--) {
		note_period += dot_extension;
		dot_extension /= 2;
	}

	return note_period;
}

unsigned
ESC_Tune::rest_duration(unsigned rest_length, unsigned dots)
{
	unsigned whole_note_period = (60 * 1000000 * 4) / _tempo;

	if (rest_length == 0) {
		rest_length = 1;
	}

	unsigned rest_period = whole_note_period / rest_length;

	unsigned dot_extension = rest_period / 2;

	while (dots--) {
		rest_period += dot_extension;
		dot_extension /= 2;
	}

	return rest_period;
}

void
ESC_Tune::play_note(unsigned note, unsigned duration)
{
	uint16_t Freq = (uint16_t)note_to_frequency(note);
	send_tone(Freq, (uint16_t)duration / 1000, _power);
}

void
ESC_Tune::start_tune(const char *tune)
{
	// kill any current playback
	hrt_cancel(&_note_call);

	// record the tune
	_tune = tune;
	_next = tune;

	// initialise player state
	_tempo = 120;
	_note_length = 4;
	_note_mode = MODE_NORMAL;
	_octave = 4;
	_silence_length = 0;
	_repeat = false;		// otherwise command-line tunes repeat forever...

	// schedule a callback to start playing
	hrt_call_after(&_note_call, 0, (hrt_callout)next_trampoline, this);
}

void
ESC_Tune::next_note()
{
	// do we have an inter-note gap to wait for?
	if (_silence_length > 0) {
		hrt_call_after(&_note_call, (hrt_abstime)_silence_length, (hrt_callout)next_trampoline, this);
		_silence_length = 0;
		return;
	}

	// make sure we still have a tune - may be removed by the write / ioctl handler
	if ((_next == nullptr) || (_tune == nullptr)) {
		return;
	}

	// parse characters out of the string until we have resolved a note
	unsigned note = 0;
	unsigned note_length = _note_length;
	unsigned duration;

	while (note == 0) {
		// we always need at least one character from the string
		int c = next_char();

		if (c == 0) {
			goto tune_end;
		}

		_next++;

		switch (c) {
		case 'L':	// select note length
			_note_length = next_number();

			if (_note_length < 1) {
				goto tune_error;
			}

			break;

		case 'O':	// select octave
			_octave = next_number();

			if (_octave > 6) {
				_octave = 6;
			}

			break;

		case '<':	// decrease octave
			if (_octave > 0) {
				_octave--;
			}

			break;

		case '>':	// increase octave
			if (_octave < 6) {
				_octave++;
			}

			break;

		case 'M':	// select inter-note gap
			c = next_char();

			if (c == 0) {
				goto tune_error;
			}

			_next++;

			switch (c) {
			case 'N':
				_note_mode = MODE_NORMAL;
				break;

			case 'L':
				_note_mode = MODE_LEGATO;
				break;

			case 'S':
				_note_mode = MODE_STACCATO;
				break;

			case 'F':
				_repeat = false;
				break;

			case 'B':
				_repeat = true;
				break;

			default:
				goto tune_error;
			}

			break;

		case 'P':	// pause for a note length
			hrt_call_after(&_note_call,
				       (hrt_abstime)rest_duration(next_number(), next_dots()),
				       (hrt_callout)next_trampoline,
				       this);
			return;

		case 'T': {	// change tempo
				unsigned nt = next_number();

				if ((nt >= 32) && (nt <= 255)) {
					_tempo = nt;

				} else {
					goto tune_error;
				}

				break;
			}

		case 'N':	// play an arbitrary note
			note = next_number();

			if (note > 84) {
				goto tune_error;
			}

			if (note == 0) {
				// this is a rest - pause for the current note length
				hrt_call_after(&_note_call,
					       (hrt_abstime)rest_duration(_note_length, next_dots()),
					       (hrt_callout)next_trampoline,
					       this);
				return;
			}

			break;

		case 'A'...'G':	// play a note in the current octave
			note = _note_tab[c - 'A'] + (_octave * 12) + 1;
			c = next_char();

			switch (c) {
			case '#':	// up a semitone
			case '+':
				if (note < 84) {
					note++;
				}

				_next++;
				break;

			case '-':	// down a semitone
				if (note > 1) {
					note--;
				}

				_next++;
				break;

			default:
				// 0 / no next char here is OK
				break;
			}

			// shorthand length notation
			note_length = next_number();

			if (note_length == 0) {
				note_length = _note_length;
			}

			break;

		default:
			goto tune_error;
		}
	}

	// compute the duration of the note and the following silence (if any)
	duration = note_duration(_silence_length, note_length, next_dots());

	// start playing the note
	play_note(note, duration);

	// and arrange a callback when the note should stop
	hrt_call_after(&_note_call, (hrt_abstime)duration, (hrt_callout)next_trampoline, this);
	return;

	// tune looks bad (unexpected EOF, bad character, etc.)
tune_error:
	syslog(LOG_ERR, "tune error\n");
	_repeat = false;		// don't loop on error

	// stop (and potentially restart) the tune
tune_end:

	if (_repeat) {
		start_tune(_tune);

	} else {
		_tune = nullptr;
		_default_tune_number = 0;
	}

	return;
}

int
ESC_Tune::next_char()
{
	while (isspace(*_next)) {
		_next++;
	}

	return toupper(*_next);
}

unsigned
ESC_Tune::next_number()
{
	unsigned number = 0;
	int c;

	for (;;) {
		c = next_char();

		if (!isdigit(c)) {
			return number;
		}

		_next++;
		number = (number * 10) + (c - '0');
	}
}

unsigned
ESC_Tune::next_dots()
{
	unsigned dots = 0;

	while (next_char() == '.') {
		_next++;
		dots++;
	}

	return dots;
}

int
ESC_Tune::write(file *filp, const char *buffer, size_t len)
{
	// sanity-check the buffer for length and nul-termination
	if (len > _tune_max) {
		return -EFBIG;
	}

	// if we have an existing user tune, free it
	if (_user_tune != nullptr) {

		// if we are playing the user tune, stop
		if (_tune == _user_tune) {
			_tune = nullptr;
			_next = nullptr;
		}

		// free the old user tune
		free((void *)_user_tune);
		_user_tune = nullptr;
	}

	// if the new tune is empty, we're done
	if (buffer[0] == '\0') {
		return OK;
	}

	// allocate a copy of the new tune
	_user_tune = strndup(buffer, len);

	if (_user_tune == nullptr) {
		return -ENOMEM;
	}

	// and play it
	start_tune(_user_tune);

	return len;
}

namespace esc_tune_drv {
static ESC_Tune *g_esc_tune = nullptr;

static char _device[16] = {};
static uint8_t _power = 30;
static bool _is_started = false;

void usage();
int start();
int tune_init();

int play_tune(unsigned tune);
int play_string(const char *str, bool free_buffer);

int play_tune(unsigned tune)
{
	int	fd, ret;

	fd = open(TONEALARM1_DEVICE_PATH, 0);

	if (fd < 0) {
		err(1, TONEALARM1_DEVICE_PATH);
	}

	ret = ioctl(fd, TONE_SET_ALARM, tune);
	close(fd);

	if (ret != 0) {
		err(1, "TONE_SET_ALARM");
	}

	exit(0);
}

int play_string(const char *str, bool free_buffer)
{
	int	fd, ret;

	fd = open(TONEALARM1_DEVICE_PATH, O_WRONLY);

	if (fd < 0) {
		err(1, TONEALARM1_DEVICE_PATH);
	}

	ret = write(fd, str, strlen(str) + 1);
	close(fd);

	if (free_buffer) {
		free((void *)str);
	}

	if (ret < 0) {
		err(1, "play tune");
	}

	exit(0);
}

int tune_init() {
	int ret = OK;

	if (g_esc_tune == nullptr) {
		g_esc_tune = new ESC_Tune(TONEALARM1_DEVICE_PATH, _device, _power);
		if (g_esc_tune == nullptr) {
			PX4_ERR("failed to allocate esc_tune driver");
			ret =  -ENOMEM;
			return ret;
		} else {
			ret = g_esc_tune->init();

			if (ret != OK) {
				PX4_ERR("failed to initialize esc_tune (%i)", ret);
				delete g_esc_tune;
				g_esc_tune = nullptr;

				return ret;
			}
		}
	}
	return ret;
}

int start() {
	if(tune_init() != OK) {
		_is_started = false;
		errx(1, "tune init failed");
	} else {
		_is_started = true;
	}
//	play_tune(TONE_STARTUP_TUNE);
	return OK;
}

void usage() {
	PX4_INFO("usage: esc_tone start -d /dev/ttyS4 -p 30");
	PX4_INFO("       esc_tone play");
//	PX4_INFO("       esc_tone status");
}
};

extern "C" __EXPORT int esc_tone_main(int argc, char *argv[]);

int esc_tone_main(int argc, char *argv[]) {

	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	char *verb = nullptr;

	if(argc >= 2) {
		verb = argv[1];
	} else {
		esc_tune_drv::usage();
		errx(1, "invalid parameters");
	}

	while ((ch = px4_getopt(argc, argv, "d:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			strncpy(esc_tune_drv::_device, device, strlen(device));
			break;

		case 'p':
			esc_tune_drv::_power = atoi(myoptarg);
			break;
		}
	}

	// Start/load the driver.
	if (!strcmp(verb, "start")) {
		if (esc_tune_drv::_is_started) {
			PX4_WARN("esc_tune already start");
			return 1;
		}

		// Check on required arguments
		if (device == nullptr || strlen(device) == 0) {
			esc_tune_drv::usage();
			return 1;
		}

		esc_tune_drv::start();
		return 0;
	}
	else if (!strcmp(verb, "play")) {
		if (!esc_tune_drv::_is_started) {
			PX4_WARN("esc_tune is not start");
			return 1;
		}
		esc_tune_drv::play_tune(TONE_STARTUP_TUNE);
	}

	return 0;
}
