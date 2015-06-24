#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <linux/types.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "iflytype.h"
#include "LocalSdk.h"


#define GPS_POWER				4

#define ERROR_OPEN_TTY			1
#define ERROR_INIT_DVRSDK		2
#define ERROR_GPS_COMM			3
#define ERROR_CELLULAR_COMM		4
#define ERROR_RS232_PORT		5
#define ERROR_RS485_PORT		6
#define ERROR_CAN_COMM			7
#define ERROR_INPUT_SIG			8


struct hw_test_item {
	char *name;
	int (* test_func)(int);
	int param;
};


int test_gps(int pm);
int test_cellular(int pm);
int test_rs232_port(int pm);
int test_rs485_port(int pm);
int test_can_adapter(int pm);
int test_input_signal(int pm);


struct hw_test_item hwti[] = {
		{.name = "gps", 		.test_func = test_gps, 			.param = 0},
		{.name = "cellular", 	.test_func = test_cellular, 	.param = 0},
		{.name = "serial1", 	.test_func = test_rs232_port, 	.param = 0},
		{.name = "serial2", 	.test_func = test_rs232_port, 	.param = 1},
		{.name = "serial3", 	.test_func = test_rs485_port, 	.param = 0},
		{.name = "serial4", 	.test_func = test_rs485_port, 	.param = 1},
		{.name = "can", 		.test_func = test_can_adapter, 	.param = 0},
		{.name = "rev", 		.test_func = test_input_signal, .param = 4},
		{.name = "alarm", 		.test_func = test_input_signal, .param = 2},
};


int open_tty(char *device, speed_t baudrate, int timeout)
{
    int fd;
    struct termios options;

    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
    	//perror("open tty");
        return -1;
    } else
        fcntl(fd, F_SETFL, 0);

    tcgetattr(fd, &options);

    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;         /* no parity */
    options.c_cflag &= ~CSTOPB;         /* one stop bit */
    options.c_cflag &= ~CSIZE;          /* mask the character size bits */
    options.c_cflag |= CS8;             /* select 8 data bits */
    options.c_cflag &= ~CRTSCTS;        /* disable hardware flow control */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* choosing raw input */
    options.c_iflag &= ~(IXON | IXOFF | IXANY);          /* disable software flow control */
    options.c_oflag &= ~OPOST;          /* choosing raw output */
    options.c_cc[VTIME] = timeout;
    options.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

int init_dvrsdk(void)
{
    int ret, hdev;

    ret = DVRSDK_startup(&hdev);
	if (ret != NETDVR_SUCCESS)
        return -1;

    return hdev;
}

int test_gps(int pm)
{
	int h_dvr, tty, count = 20, ret = -ERROR_GPS_COMM;
	FILE *fp;
	char gps_dat[200];
	struct NETDVR_AlarmVal_t op;

	h_dvr = init_dvrsdk();
	if (h_dvr < 0)
		return -ERROR_INIT_DVRSDK;

	tty = open_tty("/dev/ttyAMA2", B9600, 10);
	if (tty < 0)
		return -ERROR_OPEN_TTY;

	fp = fdopen(tty, "r");

	/*open gps power*/
	op.alarmid = GPS_POWER;
	op.val = 1 << GPS_POWER;
	NETDVR_setAlarmOutVal(h_dvr, &op);

	while(count--) {
		if (fgets(gps_dat, 200, fp) != NULL) {
			if (strncmp(gps_dat, "$GP", 3) == 0) {
				ret = 0;
				break;
			}
		}
	}

	/*close gps power*/
	op.alarmid = GPS_POWER;
	op.val = 0;
	NETDVR_setAlarmOutVal(h_dvr, &op);

	fclose(fp);
	close(tty);
	DVRSDK_cleanup(h_dvr);
	return ret;
}

int test_cellular(int pm)
{
	int tty, retry, ret = -ERROR_CELLULAR_COMM;
	char *atcmd = "at\r\n", c;

	retry = 0;
	while(1) {
		tty = open_tty("/dev/ttyUSB0", B115200, 10);
		if (tty > 0)
			break;

		if (++retry == 60)
			return -ERROR_OPEN_TTY;

		sleep(1);
	}

	retry = 10;
	while(retry--) {
		write(tty, atcmd, strlen(atcmd));

		if (read(tty, &c, 1) == 1) {
			ret = 0;
			break;
		}
	}

	close(tty);
	return ret;
}

int test_rs232_port(int pm)
{
	int tty, retry = 5, ret = -ERROR_RS232_PORT;
	char *dat = "1234567890", *devname[2] = {"/dev/ttyS0", "/dev/ttyS1"},
		*pt[2] = {"serial1", "serial2"}, buf[50];
	FILE *fp;

	tty = open_tty(devname[pm], B9600, 10);
	if (tty < 0)
		return -ERROR_OPEN_TTY;

	fp = fdopen(tty, "r");

	while(retry--) {
		write(tty, dat, strlen(dat));

		if (fgets(buf, 50, fp) != NULL) {
			//printf("%s", buf);
			if (strncmp(buf, pt[pm], strlen(pt[pm])) == 0) {
				ret = 0;
				break;
			}
		}
	}

	fclose(fp);
	close(tty);
	return ret;
}

char * get_line(int tty, char *buf, int len)
{
	char c;
	int idx = 0, ret;

	while (1) {
		ret = read(tty, &c, 1);
		if (ret <= 0)
			return NULL;

		buf[idx++] = c;

		if (idx == len-1)
			return NULL;

		if (c == '\n')
			break;
	}

	buf[idx] = '\0';
	return buf;
}

int test_rs485_port(int pm)
{
	int tty, retry = 20, ret = -ERROR_RS485_PORT;;
	char *devname[2] = {"/dev/ttyAMA3", "/dev/ttyS2"},
		*pt[2] = {"PORT1", "PORT2"}, buf[50];
	unsigned char dat = pm == 0 ? 0x55 : 0xaa;
	int h_dvr;

	tty = open_tty(devname[pm], B9600, 2);
	if (tty < 0)
		return -ERROR_OPEN_TTY;

	if (pm == 0) {
		h_dvr = init_dvrsdk();
		if (h_dvr < 0)
			return -ERROR_INIT_DVRSDK;
	}

	while (retry--) {
		if (pm == 0) {
			NETDVR_enable485SerOut(h_dvr, 1);
			usleep(10000);
		}

		write(tty, &dat, 1);
		usleep(10000);

		if (pm == 0)
			NETDVR_enable485SerIn(h_dvr, 1);

		if (get_line(tty, buf, 50) != NULL) {
			printf("%s", buf);
			if (strncmp(buf, pt[pm], strlen(pt[pm])) == 0) {
				ret = 0;
				break;
			}
		}
	}

	if (pm == 0)
		DVRSDK_cleanup(h_dvr);

	close(tty);
	return ret;
}

int get_can_data(int tty, unsigned char *buf, int len)
{
	int ret, flen;

	do {
		ret = read(tty, buf, 1);
		if (ret <= 0)
			return -1;
	} while (buf[0] != 0x7e);

	if (read(tty, buf+1, 1) <= 0 || buf[1] > len)
		return -1;

	flen = buf[1] - 2;

	if (read(tty, buf+2, flen) != flen)
		return -1;

	return 0;
}

int test_can_adapter(int pm)
{
	int tty, retry = 5, ret = -ERROR_CAN_COMM;
	unsigned char buf[20];

	tty = open_tty("/dev/ttyS3", B115200, 10);
	if (tty < 0)
		return -ERROR_OPEN_TTY;

	while (retry--) {
		if (get_can_data(tty, buf, 20) == 0) {
			ret = 0;
			break;
		}
	}

	close(tty);
	return ret;
}

int test_input_signal(int pm)
{
	int h_dvr, loop = 20, ret = -ERROR_INPUT_SIG, hi_lvl = 0, low_lvl = 0;
	struct NETDVR_AlarmVal_t input_val;

	h_dvr = init_dvrsdk();
	if (h_dvr < 0)
		return -ERROR_INIT_DVRSDK;

	while (loop--) {
		NETDVR_getAlarmInVal(h_dvr, pm, &input_val);
		if (input_val.val == 1)
			hi_lvl = 1;
		if (input_val.val == 0)
			low_lvl = 1;

		if (hi_lvl && low_lvl) {
			ret = 0;
			break;
		}

		usleep(500000);
	}

	DVRSDK_cleanup(h_dvr);
	return ret;
}

int main(int argc, char **argv)
{
	int i;

	if (argc != 2)
		return -1;

	for (i = 0; i < sizeof(hwti)/sizeof(struct hw_test_item); i++) {
		if (strcmp(argv[1], hwti[i].name) != 0)
			continue;

		return hwti[i].test_func(hwti[i].param);
	}

	return -1;
}

