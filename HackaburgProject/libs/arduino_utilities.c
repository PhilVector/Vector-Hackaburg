/*
 * arduino_utilities.c
 *
 *  Created on: 02.02.2018
 *      Author: WagPhi
 */

#include "arduino_utilities.h"

#ifndef MATLAB_MEX_FILE
//Open TTY with settings and PArse and mapp
int open_ttyACM(const char *filename)
{
	int fd;
	int err;
	struct termios term_attr;

	fd = open(filename, O_RDWR | O_NOCTTY | O_SYNC);

	assert(fd >= 0);
	err = tcgetattr(fd, &term_attr);
	assert(err >= 0);

	cfsetispeed(&term_attr, B115200);
	assert(err >= 0);
	cfsetospeed(&term_attr, B115200);
	assert(err >= 0);
	term_attr.c_cflag &= ~PARENB;
	term_attr.c_cflag &= ~CSTOPB;
	term_attr.c_cflag &= ~CSIZE;
	term_attr.c_cflag |= CS8;
	term_attr.c_cflag |= (CLOCAL | CREAD);
	term_attr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	term_attr.c_iflag &= ~(IXON | IXOFF | IXANY);
	term_attr.c_oflag &= ~OPOST;


	  cfmakeraw(&term_attr);

	tcflush(fd, TCIOFLUSH);

	err = tcsetattr(fd, TCSANOW, &term_attr);
	assert(err >= 0);

	return fd;
}


int map_arduino_id(uint8_t id)
{
	int returnvalue = -1;
	switch (id) {
	case ID_ARD_SENSOR_INFO:                    //0
		returnvalue = -2;
		break;
	case ID_ARD_SENS_ERROR:                    //1
		returnvalue = -3;
		break;
	case ID_ARD_SENS_US_FRONT_LEFT:                    //2
		returnvalue = ARD_FRONT;
		break;
	case ID_ARD_SENS_US_FRONT_CENTER_LEFT:                    //3
		returnvalue = ARD_FRONT;
		break;
	case ID_ARD_SENS_US_FRONT_CENTER:                    //4
		returnvalue = ARD_FRONT;
		break;
	case ID_ARD_SENS_US_FRONT_CENTER_RIGHT:                    //5
		returnvalue = ARD_FRONT;
		break;
	case ID_ARD_SENS_US_FRONT_RIGHT:                    //6
		returnvalue = ARD_FRONT;
		break;
	case ID_ARD_SENS_US_REAR_RIGHT:                    //7
		returnvalue = ARD_REAR_US;
		break;
	case ID_ARD_SENS_US_REAR_CENTER_RIGHT:                    //8
		returnvalue = ARD_REAR_US;
		break;
	case ID_ARD_SENS_US_REAR_CENTER:                    //9
		returnvalue = ARD_REAR_US;
		break;
	case ID_ARD_SENS_US_REAR_CENTER_LEFT:                    //10
		returnvalue = ARD_REAR_US;
		break;
	case ID_ARD_SENS_US_REAR_LEFT:                    //11
		returnvalue = ARD_REAR_US;
		break;
	case ID_ARD_SENS_US_SIDE_LEFT:                    //12
		returnvalue = ARD_REAR_US;
		break;
	case ID_ARD_SENS_US_SIDE_RIGHT:                    //13
		returnvalue = ARD_REAR_US;
		break;
	case ID_ARD_SENS_WHEEL_RIGHT:                    //14
		returnvalue = ARD_REAR_IMU;
		break;
	case ID_ARD_SENS_WHEEL_LEFT:                    //15
		returnvalue = ARD_REAR_IMU;
		break;
	case ID_ARD_SENS_IMU:                    //16
		returnvalue = ARD_REAR_IMU;
		break;
	case ID_ARD_SENS_VOLT_ACTUATOR:                    //17
		returnvalue = ARD_CENTER;
		break;
	case ID_ARD_SENS_VOLT_ACTUATOR_CELL1:                    //18
		returnvalue = ARD_CENTER;
		break;
	case ID_ARD_SENS_VOLT_ACTUATOR_CELL2:                    //19
		returnvalue = ARD_CENTER;
		break;
	case ID_ARD_SENS_VOLT_SENSORS:                    //20
		returnvalue = ARD_CENTER;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL1:                    //21
		returnvalue = ARD_CENTER;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL2:                    //22
		returnvalue = ARD_CENTER;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL3:                    //23
		returnvalue = ARD_CENTER;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL4:                    //24
		returnvalue = ARD_CENTER;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL5:                    //25
		returnvalue = ARD_CENTER;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL6:                    //26
		returnvalue = ARD_CENTER;
		break;
	case ID_ARD_SENS_REMOTE:                    //26
		returnvalue = ARD_REMOTE;
		break;
	default:
		break;
	}
	return returnvalue;

}

uint8_t get_id(char buf[], int length)
{
	char cID = buf[0];
	uint8_t id;
	memcpy(&id, &cID, sizeof(uint8_t));

	return id;
}

void hexdump(char buf[], int length)
{
	printf("Hexdump: Length: %d\n", length);
	bool breaking = false;
	for (int i = 0; i < length - 1; i++) {
		printf("%u\t", (uint8_t) buf[i]);
		if (i > 20) {
			printf("...\n");
			breaking = true;
			break;
		}
	}
	if (!breaking) {
		printf("%u\n", (uint8_t) buf[length - 1]);
	}
	//printf ("PA:%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
	//	  buf[8], buf[9], buf[10]);
	printf("Hexdump\n");

}

int read_packet(int fd, char *buf)
{
	int bytes;
	int ret = ioctl(fd, FIONREAD, &bytes);
	if (ret == -1) {
		perror("OCTL");
		errno = ret;
	}
	if (bytes > 0) {
		char cSTX;
		char cLastC = 1;
		//sync to start of frame (STX)
		do {
			ret = read(fd, &cSTX, sizeof(uint8_t));
			cLastC = cSTX;
		} while (!(cSTX == START_BYTE && cLastC != ESCAPE_BYTE)); // until we reach STX

		//Read frame
		int bytes_read = -1;
		while (true) {
			char cbuf;
			ret = read(fd, &cbuf, sizeof(uint8_t));
			if (ret == -1) {
				perror("Read_Frame");
				break;
			}
			bytes_read++;

			if (cbuf == END_BYTE)
				break;
			if (cbuf == ESCAPE_BYTE) {
				ret = read(fd, &cbuf, sizeof(uint8_t));
				if (ret == -1) {
					perror("Read_Frame");
					break;
				}
				buf[bytes_read] = cbuf;
			} else {
				buf[bytes_read] = cbuf;
			}
			if (bytes_read > MAX_MSG_LENGTH_TILL_TIMEOUT)
				return -1;
		}
		//hexdump(buf, bytes_read);
		return bytes_read;
	}
	return errno;
}

int get_Frame(char buf[], int fd)
{
	int ret = read_packet(fd, buf);
	if (ret != -1) {

		return ret;
	}
	if (ret != 0) {
		return -3;
	}

	return -1;
}

uint8_t
parse_info (char buf[], int length)
{
	uint8_t returnvalue = 200;
	tInfoData infodata;
	if (length != 11) {
		return returnvalue;
	} else {
		int bytepointer = 1;
		if (buf[bytepointer] != 3) {
			errno = 155;
			hexdump(buf, length);
			printf("Error: %d, parse_info\n\n", errno);
			perror("parse_info Packet");
		}
		bytepointer++;
		//TIMESTAMP
		uint32_t timestamp;
		char ctimestamp[sizeof(uint32_t)];
		for (int i = 0; i < sizeof(uint32_t); i++) {
			ctimestamp[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint32_t);
		memcpy(&timestamp, &ctimestamp, sizeof(uint32_t));

		//Address
		char cui8ArduinoAddress;
		cui8ArduinoAddress = buf[bytepointer];
		bytepointer = bytepointer + sizeof(uint8_t);
		uint8_t ui8ArduinoAddress;
		memcpy(&ui8ArduinoAddress, &cui8ArduinoAddress, sizeof(uint8_t));
		infodata.ui8ArduinoAddress = ui8ArduinoAddress;

		//ArduinoVersion
		char cui16ArduinoVersion[sizeof(uint16_t)];
		for (int i = 0; i < sizeof(uint16_t); i++) {
			cui16ArduinoVersion[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint16_t);
		uint16_t ui16ArduinoVersion;
		memcpy(&ui16ArduinoVersion, &cui16ArduinoVersion, sizeof(uint16_t));
		infodata.ui16ArduinoVersion = ui16ArduinoVersion;
		returnvalue = infodata.ui8ArduinoAddress;

	}
	return returnvalue;
}
uint16_t
fletcher16 (uint8_t const *data, uint8_t bytes)
{

	uint16_t sum1 = 0xff, sum2 = 0xff;

	while (bytes) {
		uint8_t tlen = bytes > 20 ? 20 : bytes;
		bytes -= tlen;
		do {
			sum2 += sum1 += *data++;
		} while (--tlen);
		sum1 = (sum1 & 0xff) + (sum1 >> 8);
		sum2 = (sum2 & 0xff) + (sum2 >> 8);
	}
	/* Second reduction step to reduce sums to 8 bits */
	sum1 = (sum1 & 0xff) + (sum1 >> 8);
	sum2 = (sum2 & 0xff) + (sum2 >> 8);
	return sum2 << 8 | sum1;
}

int8_t
parse_error (char buf[], int length)
{
	int8_t errorcode = -100;
	if (length != 9) {
		return errorcode;
	} else {
		int bytepointer = 1;
		if (buf[bytepointer] != 1) {
			errno = 155;
			hexdump(buf, length);
			printf("Error: %d, parse_info\n\n", errno);
			perror("parse_info Packet");
			return errorcode;
		}
		bytepointer++;
		//TIMESTAMP
		uint32_t timestamp;
		char ctimestamp[sizeof(uint32_t)];
		for (int i = 0; i < sizeof(uint32_t); i++) {
			ctimestamp[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint32_t) + 1;
		memcpy(&timestamp, &ctimestamp, sizeof(uint32_t));

		//Address
		char cerrorcode;
		cerrorcode = buf[bytepointer];
		bytepointer = bytepointer + sizeof(uint8_t);
		memcpy(&errorcode, &cerrorcode, sizeof(int8_t));
		printf(KMAG "Error %d\t", errorcode);
		switch (errorcode) {
		case ERROR_STX_NOT_FOUND:
			printf("ERROR_STX_NOT_FOUND");
			break;
		case ERROR_ESC_BYTE_BROKEN:
			printf("ERROR_ESC_BYTE_BROKEN");
			break;
		case ERROR_NO_BYTES_AVAILABLE:
			printf("ERROR_NO_BYTES_AVAILABLE");
			break;
		case ERROR_CRC_INVALID:
			printf("ERROR_CRC_INVALID");
			break;
		case ERROR_NO_FRAME_DATA:
			printf("ERROR_NO_FRAME_DATA");
			break;
		case ERROR_FRAME_DROPPED:
			printf("ERROR_FRAME_DROPPED");
			break;
		case ERROR_REMOTE_DETECTED:
			printf("ERROR_REMOTE_DETECTED");
			break;
		case ERROR_NO_GYRO_DETECTED:
			printf("ERROR_NO_GYRO_DETECTED");
			break;
		case ERROR_INVALID_ACTUATOR_HEADER:
			printf("ERROR_INVALID_ACTUATOR_HEADER");
			break;
		case ERROR_INITIALIZATION_FAILED:
			printf("ERROR_INITIALIZATION_FAILED");
			break;
		case ERROR_FRAME_NOT_WRITTEN:
			printf("ERROR_FRAME_NOT_WRITTEN");
			break;
		default:
			printf("Unkown Errorcode");
			break;
		}
		printf(RESET "\n");
	}
	return errorcode;
}

int
_write_serial (int fd, char data[], size_t length)
{
	int bytes = 0;
	ioctl(fd, TIOCOUTQ, &bytes);
//	printf("Bytes_pre: %d\n", bytes);

//	int n_written = 0, spot = 0;
//
//	do {
//		n_written = write(fd, &data[spot], 1);
//		spot += n_written;
//	} while (spot <= (int) length);
	int ret = write(fd, data, length);
//	printf("Bytes written: %d\n", ret);
	bytes = 0;
	ioctl(fd, TIOCOUTQ, &bytes);
//	printf("Bytes_pos: %d\n", bytes);
	//Write Frame
//	tcflush(fd, TCIOFLUSH);
	//	fsync(fd);
	return ret;
}


int
send_actor_frame (int fd, uint8_t ui8ID, uint8_t ui8DataLength,
		uint32_t ui32Timestamp, uint8_t data)
{
	int ret = -1;
	char buf[SIZEACTOR]; // Length 20

	//Create Data Frame
	char daten[10];
	//  printf ("dataLength: %u\n", ui8DataLength);


	memcpy(daten, &ui8ID, sizeof(uint8_t));
	memcpy(daten + sizeof(uint8_t), &ui8DataLength, sizeof(uint8_t));
	memcpy(daten + sizeof(uint8_t) + sizeof(uint8_t), &ui32Timestamp,
			sizeof(uint32_t));
	memcpy(daten + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint32_t), &data,
			sizeof(uint8_t));

	//CRC Data Frame
	uint16_t crc = fletcher16((uint8_t const*) daten,
			sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint32_t)
					+ sizeof(uint8_t));

	//Create Sending Frame
	buf[0] = START_BYTE;
	memcpy(buf + 1, daten,
			sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint32_t)
					+ sizeof(uint8_t));
	memcpy(buf + 8, &crc, sizeof(uint16_t));
	buf[10] = END_BYTE;

	char buf1[SIZEACTOR * 2]; // Length 20
	int d = 0;

	for (int i = 1; i < SIZEACTOR - 1; i++) {
		if (buf[i] == START_BYTE || buf[i] == END_BYTE) {
			buf1[i + d] = ESCAPE_BYTE;
			buf1[i + d + 1] = buf[i];
			d++;
		} else {
			buf1[i + d] = buf[i];
		}
	}
	buf1[0] = START_BYTE;
	buf1[SIZEACTOR + d - 1] = END_BYTE;
	buf1[SIZEACTOR + d] = (uint8_t) 0;
	buf1[SIZEACTOR + d + 1] = 4;


//	hexdump(buf1, SIZEACTOR + d);
	ret = _write_serial(fd, buf1, SIZEACTOR + d);
	return ret;
}

void
printName (uint8_t info)
{
	printf(KYEL "USB detected: ");
	switch (info) {
	case ARD_FRONT:
		printf(KGRN "ARD_FRONT");
		break;
	case ARD_REAR_US:
		printf(KGRN "ARD_SIDE");
		break;
	case ARD_REAR_IMU:
		printf(KGRN "ARD_REAR");
		break;
	case ARD_ACTOR:
		printf(KGRN "ARD_ACTOR");
		break;
	case ARD_REMOTE:
		printf(KGRN "ARD_REMOTE");
		break;
	case ARD_CENTER:
		printf(KGRN "ARD_CENTER");
		break;
	default:
		break;
	}
	printf(RESET "\n");

}

int get_sensor_arduino(int fd, uint8_t address)
{
	char buf[CHAR_PACKET_BUF];
	uint8_t info = 200;
	int bytes_read = get_Frame(buf, fd);
	if (bytes_read > 0) {
		int ret = -1;
		uint8_t id = get_id(buf, bytes_read);

		if (id == ID_ARD_SENSOR_INFO) {
			info = parse_info(buf, bytes_read);
			if (info == 200) {
				errno = ret;
				return -1;
			}
			if (address == info) {
				printName(info);
				return fd;
			}
		}

		if (id == ID_ARD_SENS_ERROR) {
			int8_t errorcode = parse_error(buf, bytes_read);
			return errorcode;
		}

		if (info == 200) {
			ret = map_arduino_id(id);
			if (ret == -1) {
				errno = ret;
				return ret;
			}
			info = (uint8_t) ret;
		}
		if (address == info) {
			printName(info);
			return fd;
		} else {
			return -1;
		}
	}
	return -1;
}

int
get_actor_arduino (int fd, uint8_t address)
{
	int ret = -1;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	unsigned long micros = 1000000 * tv.tv_sec + tv.tv_usec;
	uint32_t send_micros = micros;
	uint8_t one = 1;
	ret = send_actor_frame(fd, ID_ARD_ACT_REQUEST, one, send_micros, one);
	if (ret == -1) {
		perror("Sending Fail");
		return ret;
	}
	for (int j = 0; j < SENDREQUEST; j++) {
		ret = get_sensor_arduino(fd, address);
		if (ret > -1) {
			return ret;
		}
		gettimeofday(&tv, NULL);
		micros = 1000000 * tv.tv_sec + tv.tv_usec;
		while (true) {
			gettimeofday(&tv, NULL);
			unsigned long micros_2 = 1000000 * tv.tv_sec + tv.tv_usec;
			if ((micros_2 - micros) > 80000) {
				break;
			}
		}
	}
	return -1;
}

int scan_ttyACM(uint8_t address)
{
	const char *path;
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *bus, *name;
	errno = -1;
	udev = udev_new();
	assert(udev != NULL);

	/* Create a list of the devices in the 'tty' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "tty");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);




	udev_list_entry_foreach(dev_list_entry, devices)
	{
		path = udev_list_entry_get_name(dev_list_entry);
		name = udev_device_new_from_syspath(udev, path);
		bus = udev_device_get_parent_with_subsystem_devtype(name, "usb",
				"usb_device");
		if (!bus) {
			continue;
		}
		struct timeval tv;
		gettimeofday(&tv, NULL);
		unsigned long start_micros = 1000000 * tv.tv_sec + tv.tv_usec;
		int fd = open_ttyACM(udev_device_get_devnode(name));
		if (isatty(fd)) {
			while (true) {
				if (address == ARD_ACTOR) {
					int ret = get_actor_arduino(fd, address);
					if (ret != -1) {
						return ret;
					}
				} else {
					int ret = get_sensor_arduino(fd, address);
					if (ret != -1) {
						return ret;
					}
				}

				gettimeofday(&tv, NULL);
				unsigned long runtime_in_micros = 1000000 * tv.tv_sec
						+ tv.tv_usec - start_micros;
				if (runtime_in_micros >= TIMEOUT_MICROS) {
					//	  printf (KYEL "USB detected: ");
					//	  printf (RESET "%s, ", udev_device_get_devnode (name));
					//	  printf (KRED "Timeout");
					//	  printf (RESET "\n");
					errno = -1;
					break;
				}
			}
		}

	}
	return errno;
}



tUsData parse_us(char buf[], int length)
{
	tUsData usdata;
	usdata.ui16Distance = -1;
	if (length != 10) {
		return usdata;
	} else {
		int bytepointer = 1;
		if (buf[bytepointer] != 2) {
			errno = 155;
			hexdump(buf, length);
			printf("Error: %d, parse_us\n\n", errno);
			perror("parse_us Packet");
		}
		bytepointer++;
		//TIMESTAMP
		uint32_t timestamp;
		char ctimestamp[sizeof(uint32_t)];
		for (int i = 0; i < sizeof(uint32_t); i++) {
			ctimestamp[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint32_t);
		memcpy(&timestamp, &ctimestamp, sizeof(uint32_t));

		//TACHO
		char ci16Distance[sizeof(int16_t)];
		for (int i = 0; i < sizeof(int16_t); i++) {
			ci16Distance[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(int16_t);
		uint16_t ui16Distance;
		memcpy(&ui16Distance, &ci16Distance, sizeof(int16_t));
		usdata.ui16Distance = ui16Distance;

	}
	return usdata;
}

tFrontPacket read_front_frame(int fd,    tFrontPacket packet)
{
	errno = -1;
	packet.us_front_right_update = false;
	packet.us_front_center_right_update = false;
	packet.us_front_center_update = false;
	packet.us_front_center_left_update = false;
	packet.us_front_left_update = false;

	char buf[CHAR_PACKET_BUF];
	int bytes_read = get_Frame(buf, fd);
	if (bytes_read == -1) {
		errno = bytes_read;
		perror("Read Front Packet");
	}
	if (bytes_read != 10) {
		errno = bytes_read;
		packet.error = errno;
		return packet;
	}
	uint8_t id = get_id(buf, bytes_read);
	switch (id) {
	case ID_ARD_SENS_US_FRONT_LEFT:                    //2
		packet.us_front_left = parse_us(buf, bytes_read);
		packet.us_front_left_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_US_FRONT_CENTER_LEFT:                    //3
		packet.us_front_center_left = parse_us(buf, bytes_read);
		packet.us_front_center_left_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_US_FRONT_CENTER:                    //4
		packet.us_front_center = parse_us(buf, bytes_read);
		packet.us_front_center_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_US_FRONT_CENTER_RIGHT:                    //5
		packet.us_front_center_right = parse_us(buf, bytes_read);
		packet.us_front_center_right_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_US_FRONT_RIGHT:                    //6
		packet.us_front_right = parse_us(buf, bytes_read);
		packet.us_front_left_update = true;
		errno = 1;
		break;
	default:
		printf("front id not found %u\n", id);
		errno = EFAULT;
		break;
	}
	packet.error = errno;
	return packet;
}

tVoltageData parse_volt(char buf[], int length)
{
	tVoltageData voltdata;
	voltdata.ui16VoltageData = -1;

	if (length != 10) {
		return voltdata;
	} else {
		int bytepointer = 1;
		if (buf[bytepointer] != 2) {
			errno = 155;
			hexdump(buf, length);
			printf("Error: %d, parse_volt\n\n", errno);
			perror("parse_volt Packet");
		}
		bytepointer++;
		//TIMESTAMP
		uint32_t timestamp;
		char ctimestamp[sizeof(uint32_t)];
		for (int i = 0; i < sizeof(uint32_t); i++) {
			ctimestamp[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint32_t);
		memcpy(&timestamp, &ctimestamp, sizeof(uint32_t));

		//Volt
		char cui16VoltageData[sizeof(uint16_t)];
		for (int i = 0; i < sizeof(uint16_t); i++) {
			cui16VoltageData[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint16_t);
		uint16_t ui16VoltageData;
		memcpy(&ui16VoltageData, &cui16VoltageData, sizeof(uint16_t));
		voltdata.ui16VoltageData = ui16VoltageData;

	}
	return voltdata;
}

tCentMeasPacket read_center_frame(int fd,    tCentMeasPacket packet)
{
	errno = -1;
	packet.volt_actuator_update = false;
	packet.volt_actuator_update = false;
	packet.volt_actuator_cell1_update = false;
	packet.volt_actuator_cell2_update = false;
	packet.volt_sensors_update = false;
	packet.volt_sensors_cell1_update = false;
	packet.volt_sensors_cell2_update = false;
	packet.volt_sensors_cell3_update = false;
	packet.volt_sensors_cell4_update = false;
	packet.volt_sensors_cell5_update = false;
	packet.volt_sensors_cell6_update = false;

	char buf[CHAR_PACKET_BUF];
	int bytes_read = get_Frame(buf, fd);
	if (bytes_read == -1) {
		errno = bytes_read;
		perror("Read Center Meas Packet");
	}
	if (bytes_read != 10) {
		errno = bytes_read;
		packet.error = errno;
		return packet;
	}
	uint8_t id = get_id(buf, bytes_read);
	switch (id) {
	case ID_ARD_SENS_VOLT_ACTUATOR:                    //2
		packet.volt_actuator = parse_volt(buf, bytes_read);
		packet.volt_actuator_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_VOLT_ACTUATOR_CELL1:                    //3
		packet.volt_actuator_cell1 = parse_volt(buf, bytes_read);
		packet.volt_actuator_cell1_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_VOLT_ACTUATOR_CELL2:                    //4
		packet.volt_actuator_cell2 = parse_volt(buf, bytes_read);
		packet.volt_actuator_cell2_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_VOLT_SENSORS:                    //5
		packet.volt_sensors = parse_volt(buf, bytes_read);
		packet.volt_sensors_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL1:                    //6
		packet.volt_sensors_cell1 = parse_volt(buf, bytes_read);
		packet.volt_sensors_cell1_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL2:                    //6
		packet.volt_sensors_cell2 = parse_volt(buf, bytes_read);
		packet.volt_sensors_cell2_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL3:                    //6
		packet.volt_sensors_cell3 = parse_volt(buf, bytes_read);
		packet.volt_sensors_cell3_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL4:                    //6
		packet.volt_sensors_cell4 = parse_volt(buf, bytes_read);
		packet.volt_sensors_cell4_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL5:                    //6
		packet.volt_sensors_cell5 = parse_volt(buf, bytes_read);
		packet.volt_sensors_cell5_update = true;
		errno = 1;
		break;
	case ID_ARD_SENS_VOLT_SENSORS_CELL6:                    //6
		packet.volt_sensors_cell6 = parse_volt(buf, bytes_read);
		packet.volt_sensors_cell6_update = true;
		errno = 1;
		break;
	default:
		printf("center id not found %u\n", id);
		errno = EFAULT;
		break;
	}
	packet.error = errno;
	return packet;
}

tSensWheelData parse_wheel(char buf[], int length)
{
	tSensWheelData wheeldata;
	wheeldata.ui32WheelTach = -1;
	wheeldata.i8WheelDir = -1;
	if (length != 13) {
		return wheeldata;
	} else {
		int bytepointer = 1;
		if (buf[bytepointer] != 5) {
			errno = 155;
			hexdump(buf, length);
			printf("Error: %d, parse_wheel\n\n", errno);
			perror("parse_wheel Packet");
		}
		bytepointer++;
		//TIMESTAMP
		uint32_t timestamp;
		char ctimestamp[sizeof(uint32_t)];
		for (int i = 0; i < sizeof(uint32_t); i++) {
			ctimestamp[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint32_t);
		memcpy(&timestamp, &ctimestamp, sizeof(uint32_t));

		//Tacho
		char cui32WheelTach[sizeof(uint32_t)];
		for (int i = 0; i < sizeof(uint32_t); i++) {
			cui32WheelTach[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint32_t);
		uint32_t ui32WheelTach;
		memcpy(&ui32WheelTach, &cui32WheelTach, sizeof(uint32_t));
		wheeldata.ui32WheelTach = ui32WheelTach;

		//Direction
		char ci8WheelDir;
		ci8WheelDir = buf[bytepointer];
		bytepointer = bytepointer + sizeof(int8_t);
		int8_t i8WheelDir;
		memcpy(&i8WheelDir, &ci8WheelDir, sizeof(int8_t));
		wheeldata.i8WheelDir = i8WheelDir;
	}
	return wheeldata;
}

tImuData parse_imu(char buf[], int length)
{
	tImuData imudata;
	imudata.f32ax = -1;
	imudata.f32ay = -1;
	imudata.f32az = -1;
	imudata.f32gx = -1;
	imudata.f32gy = -1;
	imudata.f32gz = -1;
	imudata.f32mx = -1;
	imudata.f32my = -1;
	imudata.f32mz = -1;
	imudata.f32roll = -1;
	imudata.f32pitch = -1;
	imudata.f32yaw = -1;

	if (length != 56) {
		return imudata;
	} else {
		int bytepointer = 1;
		if (buf[bytepointer] != 48) {
			errno = 155;
			hexdump(buf, length);
			printf("Error: %d, parse_imu\n\n", errno);
			perror("parse_imu Packet");
		}
		bytepointer++;
		//TIMESTAMP
		uint32_t timestamp;
		char ctimestamp[sizeof(uint32_t)];
		for (int i = 0; i < sizeof(uint32_t); i++) {
			ctimestamp[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint32_t);
		memcpy(&timestamp, &ctimestamp, sizeof(uint32_t));

		//f32ax
		char cf32ax[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32ax[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32ax;
		memcpy(&f32ax, &cf32ax, sizeof(float));
		imudata.f32ax = f32ax;

		//f32ay
		char cf32ay[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32ay[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32ay;
		memcpy(&f32ay, &cf32ay, sizeof(float));
		imudata.f32ay = f32ay;

		//f32az
		char cf32az[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32az[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32az;
		memcpy(&f32az, &cf32az, sizeof(float));
		imudata.f32az = f32az;

		//f32gx
		char cf32gx[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32gx[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32gx;
		memcpy(&f32gx, &cf32gx, sizeof(float));
		imudata.f32gx = f32gx;

		//f32gy
		char cf32gy[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32gy[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32gy;
		memcpy(&f32gy, &cf32gy, sizeof(float));
		imudata.f32gy = f32gy;

		//f32gz
		char cf32gz[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32gz[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32gz;
		memcpy(&f32gz, &cf32gz, sizeof(float));
		imudata.f32gz = f32gz;

		//f32mx
		char cf32mx[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32mx[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32mx;
		memcpy(&f32mx, &cf32mx, sizeof(float));
		imudata.f32mx = f32mx;

		//f32my
		char cf32my[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32my[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32my;
		memcpy(&f32my, &cf32my, sizeof(float));
		imudata.f32my = f32my;

		//f32mz
		char cf32mz[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32mz[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32mz;
		memcpy(&f32mz, &cf32mz, sizeof(float));
		imudata.f32mz = f32mz;


		//f32roll
		char cf32roll[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32roll[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32roll;
		memcpy(&f32roll, &cf32roll, sizeof(float));
		imudata.f32roll = f32roll;

		//f32pitch
		char cf32pitch[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32pitch[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32pitch;
		memcpy(&f32pitch, &cf32pitch, sizeof(float));
		imudata.f32pitch = f32pitch;

		//f32yaw
		char cf32yaw[sizeof(float)];
		for (int i = 0; i < sizeof(float); i++) {
			cf32yaw[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(float);
		float f32yaw;
		memcpy(&f32yaw, &cf32yaw, sizeof(float));
		imudata.f32yaw = f32yaw;
	}
	return imudata;
}

tRearImuPacket
read_rear_imu_frame (int fd, tRearImuPacket packet)
{
	errno = -1;
	char buf[CHAR_PACKET_BUF];
	int bytes_read = get_Frame(buf, fd);
	if (bytes_read == -1) {
		errno = bytes_read;
		perror("Read Rear Imu Packet");
	}
	if ((bytes_read != 13) && (bytes_read != 56)) {
		errno = bytes_read;
		packet.error = errno;
		return packet;
	}
	uint8_t id = get_id(buf, bytes_read);
	switch (id) {
	case ID_ARD_SENS_WHEEL_RIGHT:                    //2
		packet.wheel_right = parse_wheel(buf, bytes_read);
		errno = 1;
		break;
	case ID_ARD_SENS_WHEEL_LEFT:                    //3
		packet.wheel_left = parse_wheel(buf, bytes_read);
		errno = 1;
		break;
	case ID_ARD_SENS_IMU:                    //4
		packet.imu = parse_imu(buf, bytes_read);
		errno = 1;
		break;
	default:
		printf("rear id not found %u\n", id);
		errno = EFAULT;
		break;
	}
	packet.error = errno;
	return packet;
}

tRemoteData
parse_remote (char buf[], int length)
{
	tRemoteData remotedata;
	remotedata.ui16Speed = 1500;
	remotedata.ui16Steer = 1500;

	if (length != 12) {
		return remotedata;
	} else {
		int bytepointer = 1;
		if (buf[bytepointer] != 4) {
			errno = 155;
			hexdump(buf, length);
			printf("Error: %d, parse_remote\n\n", errno);
			perror("parse_remote Packet");
		}
		bytepointer++;
		//TIMESTAMP
		uint32_t timestamp;
		char ctimestamp[sizeof(uint32_t)];
		for (int i = 0; i < sizeof(uint32_t); i++) {
			ctimestamp[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint32_t);
		memcpy(&timestamp, &ctimestamp, sizeof(uint32_t));

		//Speed
		char cui16Speed[sizeof(uint16_t)];
		for (int i = 0; i < sizeof(uint16_t); i++) {
			cui16Speed[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint16_t);
		uint16_t ui16Speed;
		memcpy(&ui16Speed, &cui16Speed, sizeof(uint16_t));
		remotedata.ui16Speed = ui16Speed;

		//Speed
		char cui16Steer[sizeof(uint16_t)];
		for (int i = 0; i < sizeof(uint16_t); i++) {
			cui16Steer[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint16_t);
		uint16_t ui16Steer;
		memcpy(&ui16Steer, &cui16Steer, sizeof(uint16_t));
		remotedata.ui16Steer = ui16Steer;

		//Speed
		char cui16CRC[sizeof(uint16_t)];
		for (int i = 0; i < sizeof(uint16_t); i++) {
			cui16CRC[i] = buf[bytepointer + i];
		}
		bytepointer = bytepointer + sizeof(uint16_t);
		uint16_t ui16CRC;
		memcpy(&ui16CRC, &cui16CRC, sizeof(uint16_t));

	}
	return remotedata;
}



tRemotePacket
read_remote_frame (int fd, tRemotePacket packet)
{
	errno = -1;
	char buf[CHAR_PACKET_BUF];
	int bytes_read = get_Frame(buf, fd);

	if (bytes_read == -1) {
		errno = bytes_read;
		perror("Read Remote Packet");
	}
	if ((bytes_read != 12)) {
		errno = bytes_read;
		packet.error = errno;
		return packet;
	}
	uint8_t id = get_id(buf, bytes_read);
	switch (id) {
	case ID_ARD_SENS_REMOTE:                    //2
		packet.remote = parse_remote(buf, bytes_read);
		errno = 1;
		break;
	default:
		printf("remote id not found %u\n", id);
		errno = EFAULT;
		break;
	}
	packet.error = errno;
	return packet;
}

#endif
