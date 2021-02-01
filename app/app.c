#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
//#include <sqlite3.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
//sqlite3 *db;
static pthread_mutex_t db_mutex = PTHREAD_MUTEX_INITIALIZER;
//int rc;
//char *err_msg = 0;
//char *sql = "DROP  TABLE IF  EXISTS  Samples;"
//	"CREATE  TABLE  Samples(Id INT , Time INT , Data  REAL);"
//	"DROP  TABLE IF  EXISTS  Temperature;"
//	"CREATE  TABLE  Temperature(Id INT , Time INT , Data  REAL);";

//static void *mms_function(void *ptr)
//{
//	for (;;) {}
//}
static int make_periodic(int sec)
{
	int fd, ret;
	struct itimerspec itval;
	
	fd = timerfd_create(CLOCK_MONOTONIC,0);
	if (fd < 0)
		return fd;
	itval.it_interval.tv_sec = sec;
        itval.it_interval.tv_nsec = 0;
        itval.it_value.tv_sec = sec;
        itval.it_value.tv_nsec = 0;
	ret = timerfd_settime(fd, 0, &itval, NULL);
	if (ret < 0)
		return ret;
	return fd;
}

static void *i2c_function(void *arg)
{
	int tfd, sfd, count = 0;
	uint64_t miss = 0;
	uint8_t buff[2] = {};
	tfd = make_periodic(2);
	sfd = open("/dev/i2c-2", O_RDWR);
	ioctl(sfd, I2C_SLAVE, 0x38);
	buff[0] = 0x00;
	buff[1] = 0x02;
	if (write(sfd, buff, 2) != 2)
		printf("I2C ctrl reg write error.\n");
	while (1) {
		read(tfd, &miss, sizeof(miss));
		buff[0] = 0x02;
		write(sfd, buff, 1);
		read(sfd, buff, 2);
		printf("DATA_LO: %u\n", buff[0] >> 4);
		printf("DATA_HI: %hhd\n", buff[1]);
		count++;
		printf("i2c count:%d\n", count);
	}
}

int main(int argc, char *argv[])
{
	pthread_t i2c_thread;
	pthread_create(&i2c_thread, NULL, i2c_function, NULL);
	pthread_exit(NULL);
	return 0;
}

