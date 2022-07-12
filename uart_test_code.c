#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <limits.h>
#include <sched.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <pthread.h>
#include <time.h>
#include <string.h>
#include <semaphore.h>
#include <termios.h>

#define COM_COUNTS	2

int uart_fd[COM_COUNTS];
unsigned long meduim_count = 0;

static int config_com_arg(int handle_fd, struct termios *old_opt)
{
	int ret = 0;
	struct termios Opt;
	
	tcgetattr(handle_fd, old_opt);
	memset(&Opt, 0, sizeof(struct termios));
	tcflush(handle_fd, TCIOFLUSH);

	/* Set the baud rate */
	cfsetispeed(&Opt, B115200);  
	cfsetospeed(&Opt, B115200);

	Opt.c_cflag |= (CREAD | CLOCAL);
	
	/*Set data bits */
	Opt.c_cflag &= ~CSIZE;
	Opt.c_cflag |= CS8;

	/* Stop bit (1 or 2) */
	Opt.c_cflag &= ~CSTOPB;
	/* PARENB   Enable parity bit
         * PARODD   Use odd parity instead of even 
	 */
	Opt.c_cflag &= ~PARENB; 	/* None */

	/* Raw input */
	Opt.c_lflag &=~(ICANON | ECHO | ECHOE | ISIG);
	/* None */
    	Opt.c_iflag &= ~INPCK;
	 
	/* Software flow control is disabled */ 
	Opt.c_iflag &=~(IXON | IXOFF | IXANY);
	/* Raw ouput */
	Opt.c_oflag &= ~OPOST;

	/* Unused because we use open with the NDELAY option */
    	Opt.c_cc[VMIN] = 0;
    	Opt.c_cc[VTIME] = 0;
	
	ret = tcsetattr(handle_fd, TCSANOW, &Opt);
        printf("c_iflag[%d] c_oflag[%d] c_cflag[%d] c_lflag[%d]\n"
               "c_line[%s] c_cc[%s] c_ispeed[%d] c_ospeed[%d]\n",
               Opt.c_iflag, Opt.c_oflag, Opt.c_cflag,
               Opt.c_lflag, Opt.c_line, Opt.c_cc,
               Opt.c_ospeed, Opt.c_ospeed);

	if (ret)
    	{        
        	perror("Set Com failed");  
        	return ret;
    	}    

	tcflush(handle_fd, TCIOFLUSH); 

	return ret;
}

void *com0_recv_task(void *arg)
{
	int rCnt = 0;
	FILE *stream;
	unsigned long rev_count = 0;
        unsigned char recv_buf[20] = {"\0"};

	stream = fopen("uart0_test.txt", "a+");
	while (1)
	{
		bzero(recv_buf, 20);
    		rCnt = read(uart_fd[0], recv_buf, 8);
		if (rCnt > 0) 
		{
			rev_count++;
			fprintf(stream, "num:%ld data:%s\n", rev_count, recv_buf);
			if (rev_count % 1000 == 0)
			{
				printf("com0 num:%ld  recv:%s\n",rev_count, recv_buf);
			}
		}
		usleep(2000);
	}

}

void *com1_recv_task(void *arg)
{
	int rCnt = 0;
	FILE *stream;
	unsigned long rev_count = 0;
        unsigned char recv_buf[20] = {"\0"};

	stream = fopen("uart1_test.txt", "a+");
	while (1)
	{
		bzero(recv_buf, 20);
    		rCnt = read(uart_fd[1], recv_buf, 8);
		if (rCnt > 0) {
			rev_count++;
			fprintf(stream, "num:%ld data:%s\n", rev_count, recv_buf);
			if (rev_count % 1000 == 0)
			{
                        	printf("com0 num:%ld  recv:%s\n",rev_count, recv_buf);
			}
		}

		usleep(2000);
	}

}

void *com0_send_task(void *arg)
{
	int rCnt = 0;
	char send_buf[] = "Testing0";
	
	while (1)
	{
		rCnt = write(uart_fd[0], send_buf, 8);
    		usleep(2000);	
	}

}

void *com1_send_task(void *arg)
{
	int rCnt = 0;
	char send_buf[] = "Testing1";
	
	while (1)
	{
		rCnt = write(uart_fd[1], send_buf, 8);
    		usleep(2000);	
	}

}

int main(int argc, char *argv[])
{
	int channel, ret = 0;
	struct sched_param param;
	pthread_attr_t attr;
	pthread_t com_send[COM_COUNTS];
	pthread_t com_recv[COM_COUNTS];
	char dev_name[16];
	struct termios Opt;

	/* Lock memory */
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {\
		printf("mlockall fialed:%m\n");
		exit(-2);
	}

	/* Initialize pthread attributes (default values) */
	ret = pthread_attr_init(&attr);
	if (ret) {
		printf("Init pthread attributes failed\n");
		goto out;
	}

	/* Set a specific stack size */
	ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
	if (ret) {
		printf("pthread setstacksize failed\n");
		goto out;
	}

	/* Set scheduler policy and priority of pthread */
	ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	if (ret) {
		printf("pthread setschedpolicy failed\n");
		goto out;
	}

//	param.__sched_priority = 80;
	param.sched_priority = 80;
	ret = pthread_attr_setschedparam(&attr, &param);
	if (ret) {
		printf(" pthread setschedparam failed\n");
		goto out;
	}

	/* Use scheduling parameters of attr */
	ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (ret) {
		printf("pthread setinheritsched failed\n");
		goto out;
	}

	for (channel = 0; channel < COM_COUNTS; channel++) {
		sprintf(dev_name,"/dev/ttySiasun%d", channel);
		uart_fd[channel] = open(dev_name, O_RDWR );
		if (uart_fd[channel] < 0)	
		{			
			printf("Open ttySiasun%d failed\n", channel);
			return -1;
		}

		ret = config_com_arg(uart_fd[channel], &Opt);
		if (ret) {
			printf("Config com%d failed\n", channel);
			return ret;
		}
	}

	ret = pthread_create(&com_recv[0], &attr, com0_recv_task, NULL);
	if (ret) {
		printf("can't create com0 recevie thread\n");
		return ret;
	}

	ret = pthread_create(&com_recv[1], &attr, com1_recv_task, NULL);
	if (ret) {
		printf("can't create com1 recevie thread\n");
		return ret;
	}

	ret = pthread_create(&com_send[0], &attr, com0_send_task, NULL);
	if (ret) {
		printf("can't create com0 send thread\n");
		return ret;
	}

	ret = pthread_create(&com_send[1], &attr, com1_send_task, NULL);
	if (ret) {
		printf("can't create com1 send thread\n");
		return ret;
	}

	pthread_join(com_recv[0], NULL);

	for (channel = 0; channel < COM_COUNTS; channel++)
	{
		tcsetattr(uart_fd[channel], TCSANOW, &Opt);
		close(uart_fd[channel]);
	}
out:	
	return ret;
}




