#include <sys/sysctl.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <linux/unistd.h>       /* for _syscallX macros/related stuff */
#include <linux/kernel.h>       /* for struct sysinfo */
#include <sys/sysinfo.h>
#include <time.h>
#include <unistd.h>

#include <string.h> /* for strncpy */
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <fcntl.h>
#include <termios.h>

#define DEFAULT_DELAY 5
#define SERIAL_SPEED B115200

typedef struct{
    uint32_t total;
    uint32_t cached;
    uint32_t buffered;
    uint32_t free;
    uint32_t used;
}ram_t;

typedef struct{
	int days;
	int hours;
	int mins;
	int secs;
}uptime_t;

typedef struct{
	uint32_t ramtotal;
	uint32_t ramfree;

	uint32_t upt_days;
	uint32_t upt_hours;
	uint32_t upt_mins;
	uint32_t upt_secs;

	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t month;
	uint8_t year; // till 2255

	uint16_t load[3];
	uint8_t ip[4];
}serial_pkt;	//64 bytes

ram_t get_ram(){
 // /proc/meminfo
 // (MemTotal - MemFree - (Buffers + Cached)) / 1024
	ram_t raminfo;
	FILE *meminfo = fopen("/proc/meminfo", "r");
	if(meminfo == NULL){
		exit(-1);
	}
	char buff[256];
	while(fgets(buff, sizeof(buff), meminfo)){
		uint32_t ramKB;
		if(sscanf(buff, "Buffers:	%d kB", &ramKB) == 1){
			raminfo.buffered = ramKB;
			ramKB = 0;
		}
		if(sscanf(buff, "MemFree: %d kB", &ramKB) == 1){
			raminfo.free = ramKB;
			ramKB = 0;
		}
		if(sscanf(buff, "MemTotal:	%d kB", &ramKB) == 1){
			raminfo.total = ramKB;
			ramKB = 0;

		}
		if(sscanf(buff, "Cached:	%d kB", &ramKB) == 1){
			raminfo.cached = ramKB;
			ramKB = 0;

		}
	}
	if(fclose(meminfo) != 0){
		exit(-1);
	}
	raminfo.used  = raminfo.total - raminfo.free - (raminfo.buffered + raminfo.cached);
	return raminfo;
}

uint8_t * in_addr_int_arr(struct in_addr ipadrr){
	static uint8_t iparr[4] = {0};
	char *ipstr = inet_ntoa(ipadrr);
	sscanf(ipstr, "%d.%d.%d.%d", &iparr[0], &iparr[1], &iparr[2], &iparr[3]);
	return iparr;
}
uptime_t get_uptime(){
	uptime_t upt;

	/*
	 * struct sysinfo {
    long uptime;             // Seconds since boot
    unsigned long loads[3];  // 1, 5, and 15 minute load averages
    unsigned long totalram;  // Total usable main memory size
    unsigned long freeram;   // Available memory size
    unsigned long sharedram; // Amount of shared memory
    unsigned long bufferram; // Memory used by buffers
    unsigned long totalswap; // Total swap space size
    unsigned long freeswap;  // swap space still available
    unsigned short procs;    // Number of current processes
    unsigned long totalhigh; // Total high memory size
    unsigned long freehigh;  // Available high memory size
    unsigned int mem_unit;   // Memory unit size in bytes
    char _f[20-2*sizeof(long)-sizeof(int)]; // Padding for libc5
	};
	*/
	struct sysinfo sys_info;
	int days, hours, mins, secs = 1;
	if(sysinfo(&sys_info) != 0)	perror("sysinfo");

	days = sys_info.uptime / 86400;
	hours = (sys_info.uptime / 3600) - (days * 24);
	mins = (sys_info.uptime / 60) - (days * 1440) - (hours * 60);
	secs = sys_info.uptime % 60;

	//printf("\033[1;33m  Load Avgs: \033[0;m 1min(%ld) 5min(%ld) 15min(%ld) \n",sys_info.loads[0], sys_info.loads[1], sys_info.loads[2]);
	//printf("\033[1;33m  Total Ram: \033[0;m %ldMb \t Free: %ldMb \n", sys_info.totalram/(1024*1024), sys_info.freeram /(1024*1024));
	//printf(" \033[1;33m Shared Ram: \033[0;m %ldMb ", sys_info.sharedram /(1024*1024) );
	//printf("  Buffered Ram: %ldMb \n", sys_info.bufferram / (1024*1024));
	//printf("\033[1;33m  Total Swap: \033[0;m %ldMb \t Free swap: %ldMb \n", sys_info.totalswap / (1024*1024), sys_info.freeswap / (1024*1024));
	//printf(" \n");
	//printf("\033[1;44m Total Number of processes: %d \033[0;m \n", sys_info.procs);
	upt.hours = hours;
	upt.days = days;
	upt.secs = secs;
	upt.mins = mins;
	return upt;
}
struct in_addr get_addr(char *dev){
	int fd;
	struct ifreq ifr;
	int out = 0;

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	ifr.ifr_addr.sa_family = AF_INET;

	/* I want IP address attached to "eth0" */
	strncpy(ifr.ifr_name, dev, IFNAMSIZ-1);
	out = ioctl(fd, SIOCGIFADDR, &ifr);
	close(fd);
	/* display result */
	//printf("%s\n", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
	if(out<0){
		struct in_addr addr = { 0 };
		return addr;
	}
	return ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr;
}

int set_interface_attribs (int fd, int speed, int parity){
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                perror("error %d from tcgetattr");
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                perror("error %d from tcsetattr");
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block){
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                perror ("error %d from tggetattr");
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                perror ("error %d setting term attributes");
}


void print_help(){
	printf("\n oled4linux: Send monitorization info to an external oled screen via serial\n");
		printf("\n OPTIONS:\n");
		printf("\t -h \tPrint this help\n");
		printf("\t -v \tShow info on stdout\n");
		printf("\t -i [IF]\tSpecify an interface to show its IP\n");
		printf("\t -t [TIME]\tSend time.\n");
		printf("\t -s [SERIAL_PATH]\tSpecify a serial port to send the data\n");
	exit(1);
}

int main(int argc, char *argv[]){
	unsigned int laptime = DEFAULT_DELAY;
	uint8_t verbose = 0;
	uint8_t serial = 0;
	uint8_t net = 0;
	int opt;
	char *dev;
	char *serialpath;
	int fd;
	while ((opt = getopt(argc, argv, "s:i:t:hv")) != -1) {
		switch(opt) {
			case 's':
				serial = 1;
				serialpath = optarg;
				break;
			case 't':
				laptime = atoi(optarg);
				break;
			case 'h':
				print_help();
				break;
			case 'v':
				verbose = 1;
				break;
			case 'i':
				dev = optarg;
				net = 1;
				break;
		}
	}
	if(serial){
		fd = open (serialpath, O_RDWR | O_NOCTTY | O_SYNC);
		if (fd < 0){
			perror("error opening serial conn");
			serial = 0;
		}

		set_interface_attribs (fd, SERIAL_SPEED, 0);  // set speed to 115,200 bps, 8n1 (no parity)
		set_blocking (fd, 0);                // set no blocking
	}

	//char buf [100];
	//int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
	while(1){
		double load[3];
		time_t rawtime;
		struct tm * timeinfo;

		ram_t raminfo = get_ram();
		uptime_t upt = get_uptime();
		if (getloadavg(load, 3) == -1){
			load[0] = 0;
			load[1] = 0;
			load[2] = 0;
		}
		time(&rawtime);
		timeinfo = localtime(&rawtime);
		struct in_addr addr = { 0 };

		if(net) addr = get_addr(dev);

		if(verbose){
			printf("%d-%d-%d ",timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
			printf("%d:%d:%d\t", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
			printf("%d days %d hrs %d mins %d secs\t",upt.days, upt.hours, upt.mins, upt.secs);
			printf("RAM %d/%d MB\t",raminfo.used/1024,raminfo.total/1024);
			printf("LOAD %0.2f %0.2f %0.2f\t",load[0],load[1],load[2]);
			printf("%s\t",inet_ntoa(addr));
			printf("%lu\n",sizeof(serial_pkt));

		}
		if(serial){
			serial_pkt serialbuff;
			serialbuff.ramtotal = raminfo.total/1024;
			serialbuff.ramfree = raminfo.used/1024;
			serialbuff.upt_days = upt.days;
			serialbuff.upt_hours = upt.hours;
			serialbuff.upt_mins = upt.mins;
			serialbuff.upt_secs = upt.secs;
			serialbuff.load[0] = load[0]*100;
			serialbuff.load[1] = load[1]*100;
			serialbuff.load[2] = load[2]*100;
			serialbuff.sec = timeinfo->tm_sec;
			serialbuff.min = timeinfo->tm_min;
			serialbuff.hour = timeinfo->tm_hour;
			serialbuff.day = timeinfo->tm_mday;
			serialbuff.month = timeinfo->tm_mon + 1;
			serialbuff.year = timeinfo->tm_year - 100;
			uint8_t *iparr = in_addr_int_arr(addr);
			serialbuff.ip[0] = iparr[0];
			serialbuff.ip[1] = iparr[1];
			serialbuff.ip[2] = iparr[2];
			serialbuff.ip[3] = iparr[3];
			write (fd, &serialbuff, sizeof(serial_pkt));           // send 7 character greeting
			//usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
			// receive 25:  approx 100 uS per char transmit
		}
		sleep(laptime);
	}
	return 0;

}
