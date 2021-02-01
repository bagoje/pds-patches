#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sqlite3.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <poll.h>

static int rc;
static sqlite3 *db;
static pthread_mutex_t db_mutex = PTHREAD_MUTEX_INITIALIZER;
char *err_msg;
char *sql = "DROP  TABLE IF  EXISTS  Samples;"
	"CREATE  TABLE  Samples(Id INT , Time INT , Data  REAL);"
	"DROP  TABLE IF  EXISTS  Temperature;"
	"CREATE  TABLE  Temperature(Id INT , Time INT , Data  REAL);";


static void make_timer(int sec, int *tfd)
{
	int fd, ret;
	struct itimerspec itval;
	
	fd = timerfd_create(CLOCK_MONOTONIC,0);
	if (fd < 0)
		perror("timerfd_create failed\n");
	itval.it_interval.tv_sec = sec;
	itval.it_interval.tv_nsec = 0;
        itval.it_value.tv_sec = sec;
        itval.it_value.tv_nsec = 0;
	ret = timerfd_settime(fd, 0, &itval, NULL);
	if (ret < 0)
		perror("timerfd_settime failed\n");
	*tfd = fd;
}

static void *i2c_function(void *arg)
{
	int tfd, sfd, data, count = 0;
	uint64_t miss = 0;
	uint8_t buff[2] = {};
	float temp = 0;
	char query[100];
	make_timer(2, &tfd);
	if(tfd < 0)
		perror("Timer initialization error\n");
	sfd = open("/dev/i2c-2", O_RDWR);
	ioctl(sfd, I2C_SLAVE, 0x38);
	buff[0] = 0x00;
	buff[1] = 0x02;
	write(sfd, buff, 2);
	while (1) {
		read(tfd, &miss, sizeof(miss));
		buff[0] = 0x02;
		write(sfd, buff, 1);
		read(sfd, buff, 2);
		data = ((int8_t)buff[1] << 4);
		data = data | (buff[0] >> 4);
		temp = (float)data/16;
		printf("temp: %f\n", temp);
		sprintf(query , "INSERT  INTO  Temperature(Time , Data) VALUES  (%u,%f);",(unsigned)time(NULL), temp);
		pthread_mutex_lock(&db_mutex);
		rc = sqlite3_exec(db, query , 0, 0, &err_msg);
		if (rc != SQLITE_OK)
			printf("%s\n", sqlite3_errmsg(db));
		pthread_mutex_unlock(&db_mutex);
	}
}

static void *mms_function(void *arg)
{
	int fd, ret;
	struct pollfd pfd;
	uint8_t value[10] = {0};
	float volt = 0;
	char query[100];

	fd = open("/sys/class/pds28/pds28mms0/ctrl_en", O_RDWR);
	ret = write(fd, "1", 1);
	lseek(fd, 0, SEEK_SET);
	close(fd);
	fd = open("/sys/class/pds28/pds28mms0/ctrl_ien", O_RDWR);
	ret = write(fd, "1", 1);
	lseek(fd, 0, SEEK_SET);
	close(fd);
	fd = open("/sys/class/pds28/pds28mms0/data", O_RDONLY);
	read(fd, value, sizeof(value));
	lseek(fd, 0, SEEK_SET);
	pfd.fd = fd;
	pfd.events = POLLPRI | POLLERR;
	while (1) {
		ret = poll(&pfd, 1, -1);
		if (ret > 0) {
			ret = read(pfd.fd, value, sizeof(value));
			volt = (float)strtol(value, NULL, 10)/1000;
			printf("samp: %f\n", volt);
			lseek(pfd.fd, 0, SEEK_SET);
			sprintf(query , "INSERT  INTO  Samples(Time , Data) VALUES  (%u,%f);",(unsigned)time(NULL), volt);
			pthread_mutex_lock(&db_mutex);
			rc = sqlite3_exec(db, query , 0, 0, &err_msg);
			if (rc != SQLITE_OK)
				printf("%s\n", sqlite3_errmsg(db));
			pthread_mutex_unlock(&db_mutex);
		}
	}
	close(fd);
}

int main(int argc, char *argv[])
{
	int rc;
	pthread_t i2c_thread, mms_thread;
	rc = sqlite3_open("/www/test.db", &db);
	if (rc != SQLITE_OK)
		printf("%s\n", sqlite3_errmsg(db));
	rc = sqlite3_exec(db, sql , 0, 0, &err_msg);
	if (rc != SQLITE_OK)
		printf("%s\n", sqlite3_errmsg(db));
	pthread_create(&i2c_thread, NULL, i2c_function, NULL);
	pthread_create(&mms_thread, NULL, mms_function, NULL);
	pthread_exit(NULL);
	return 0;
}

