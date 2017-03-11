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
#define DEFAULT_PORT 7

#define OUTPUT_CONSOLE 0
#define OUTPUT_SERIAL 1
#define OUTPUT_NETWORK 2

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

#define ANSI_COLOR_HEADER  "\033[95m"
#define ANSI_COLOR_OKBLUE  "\033[94m"
#define ANSI_COLOR_OKGREEN  "\033[92m"
#define ANSI_COLOR_WARNING  "\033[93m"
#define ANSI_COLOR_FAIL  "\033[91m"
#define ANSI_COLOR_BOLD  "\033[1m"
#define ANSI_COLOR_UNDERLINE  "\033[4m"

typedef struct{
    uint32_t total;
    uint32_t cached;
    uint32_t buffered;
    uint32_t free;
    uint32_t used;
}ram_t;

typedef struct{
	long long tx;
	long long rx;
	long long lasttx;
	long long lastrx;
}txrx_t;

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

uint8_t * addr2arr(struct in_addr ipadrr){
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
	if(out<0){
		struct in_addr addr = { 0 };
		return addr;
	}
	return ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr;
}

void gettxrx(char *dev){
	 FILE *netinfo = fopen("/proc/net/dev", "r");
	 if(netinfo == NULL){
		 exit(-1);
	 }
	 char buff[256];
	 while(fgets(buff, sizeof(buff), netinfo)){
		 long long tx = 0;
		 long long rx = 0;
		 char ifname[32];
		 int conv = sscanf( buff," %[^:]: %Lu %*u %*u %*u %*u %*u %*u %*u %Lu %*u %*u %*u %*u %*u %*u %*u",ifname, &rx, &tx );				 
			printf("%Lu %Lu tx/rx",tx,rx); 
	 }
	 if(fclose(netinfo) != 0){
		 exit(-1);
	 }	 
}


void clearscr(){
	printf("\033[2J\x1b[H");
}

void fill(int n){
	int i = 0;
	for(i=0; i < n; i++){
		printf("█");
		//printf("|");
	}
}

void empty(int n){
	int i = 0;
	for(i=0; i < n; i++){
		printf(" ");
	}
}

int config_serial (int fd, int speed, int parity){
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
		printf("\t -n [port]\tSend data using udp packets to the specified port.\n");
		printf("\t -c \tShow the data using ncurses style\n");


	exit(1);
}
uint8_t ndigits(int n){
		uint8_t count = 0;
    while(n != 0)
    {
        // n = n/10
        n /= 10;
        ++count;
    }
		return count;
}

float get_temp(){
	FILE *fp;
	char buff[5];
	float cpu_temp = 0;
	fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
	if(fp==NULL) return -1;
	fscanf(fp, "%s", buff);
	cpu_temp = (float)atoi(buff)/1000;
	fclose(fp);
	return cpu_temp;
}

uint32_t get_cpu(uint32_t *cpufull,uint32_t *cpuidle){
	// /proc/stat
	 FILE *cpuinfo = fopen("/proc/stat", "r");
	 if(cpuinfo == NULL){
		 exit(-1);
	 }
	 char buff[256];
	 uint32_t cores = 0;
	 while(fgets(buff, sizeof(buff), cpuinfo)){
		//char *c = strchr(buff, '\n');
		//if (c) *c = '\0';
		 uint32_t val[10];
		 uint32_t cpuid;
		 //     user    nice   system  idle  iowait irq   softirq  steal  guest  guest_nice
		 if(sscanf(buff, "cpu%d %d	%d	%d	%d	%d	%d	%d	%d	%d	%d\n",
			 &cpuid,&val[0],&val[1],&val[2],&val[3],&val[4],&val[5],&val[6],&val[7],&val[8],&val[9])){						 
					//int usertime = val[0] - val[8];                    
					//int nicetime = val[1] - val[9];                 
					int idlealltime = val[3] + val[4];                
					//int systemalltime = val[2] + val[5] + vak[4];
					//int virtalltime = val[8] + val[9];
					
					cpufull[cores] = val[0]+
						val[1]+val[2]+val[3]+val[4]+
						val[5]+val[6]+val[7]+val[0]+val[9];

					cpuidle[cores] = idlealltime;									
					 cores = cores + 1;
					 val[0] = 0;
					 val[1] = 0;
					 val[2] = 0;
					 val[3] = 0;
					 val[4] = 0;
					 val[5] = 0;
					 val[6] = 0;
					 val[7] = 0;
					 val[8] = 0;
					 val[9] = 0;					 
			 	}

	 }
	 //printf("%d cores",cores-1);
	 if(fclose(cpuinfo) != 0){
		 exit(-1);
	 }
	 return cores;
}

void cpu_bar(int cores, uint32_t *full,uint32_t *idle,uint32_t *lastfull,uint32_t *lastidle,uint32_t cols){
	int n = 1;
	for (n=1; n<cores;n++){
		uint32_t vtotal = full[n]-lastfull[n];
		uint32_t vidle = idle[n]-lastidle[n];
		float cpupcnt = (float)((vtotal - vidle)/(float)(vtotal))*100.0;
		//printf("%f\n",cpupcnt);
		printf(ANSI_COLOR_BOLD " CPU "ANSI_COLOR_RESET);
		//printf(" CPU ");
		//printf(ANSI_COLOR_BOLD "["ANSI_COLOR_RESET);
		int ldigits = ndigits(cpupcnt);
		if (ldigits==0) ldigits=1;
		int usedr = (float)(cols-6.0)*((float)cpupcnt/100.0);
		float freer = cols - usedr - 10 - ldigits;
		printf(ANSI_COLOR_OKGREEN);
		if (cpupcnt> 50) printf(ANSI_COLOR_WARNING);
		if (cpupcnt> 75) printf(ANSI_COLOR_FAIL);
		fill(usedr);
		printf(ANSI_COLOR_RESET);
		empty(freer);
		printf("%0.1f \% ",cpupcnt);
		//printf(ANSI_COLOR_BOLD "]"ANSI_COLOR_RESET);
		printf("\n");
	}
}


void status_bar(double *load,uint32_t cols){
	printf(ANSI_COLOR_BOLD" LOAD"ANSI_COLOR_RESET" %0.2f %0.2f %0.2f",load[0],load[1],load[2]);
	int temp = get_temp();
	cols = cols - 29 - ndigits(temp);
	empty(cols);
	printf(ANSI_COLOR_BOLD "TEMP"ANSI_COLOR_RESET" %d ºC\n",temp);
}

void ram_bar(ram_t ram, uint32_t cols){
	printf(ANSI_COLOR_BOLD" RAM " ANSI_COLOR_RESET);
	//printf(ANSI_COLOR_BOLD "["ANSI_COLOR_RESET);
	int ldigits = ndigits(ram.total/1024) + ndigits(ram.used/1024);
	
	float ramfull = ((float)ram.used/(float)ram.total);
	int usedr = (cols-6.0)*ramfull;
	if (usedr==0) usedr = 1;
	float freer = cols - usedr - 9 - ldigits;
	printf(ANSI_COLOR_OKGREEN);
	if (ramfull > 0.5) printf(ANSI_COLOR_WARNING);
	if (ramfull > 0.7) printf(ANSI_COLOR_FAIL);
	fill(usedr);
	printf(ANSI_COLOR_RESET);
	empty(freer);
	printf("%d/%d ",ram.used/1024,ram.total/1024);
	//printf(ANSI_COLOR_BOLD "]"ANSI_COLOR_RESET);
	
	printf("\n");
}

void time_bar(struct tm* timeinfo,uptime_t upt,uint32_t cols){
 		printf(ANSI_COLOR_BOLD" TIME"ANSI_COLOR_RESET" %02d-%02d-%04d",timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
		printf("  %02d:%02d:%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
		cols = cols - 18 - 40 - ndigits(upt.days);
		empty(cols);
		printf("  %d days %02d hrs %02d mins %02d secs\n",upt.days, upt.hours, upt.mins, upt.secs);
}

void ip_bar(char *dev, char*dev2,struct in_addr addr,struct in_addr addr2,int net){
	if(dev!=NULL && net==1){
		printf(ANSI_COLOR_BOLD" IFACE " ANSI_COLOR_RESET);
		printf("%s: %s\n",dev,inet_ntoa(addr));
	}
	if(dev2!=NULL && net==2){
		printf(ANSI_COLOR_BOLD" RAM " ANSI_COLOR_RESET);
		printf("%s: %s\n",dev2,inet_ntoa(addr2));
	}
}
int main(int argc, char *argv[]){
	unsigned int laptime = DEFAULT_DELAY;
	uint8_t verbose = 0;
	uint8_t net = 0;
	uint8_t outopt = 0;
	uint32_t port = DEFAULT_PORT;
	int opt;
	char *dev;
	char *dev2;
	char *serialpath;
	int fd;
	
	uint32_t cpufull[64];
	uint32_t cpuidle[64];
	uint32_t last_cpufull[64];
	uint32_t last_cpuidle[64];

	txrx_t if1stats;
	txrx_t if2stats;
	if1stats.tx = 0;
	if1stats.rx = 0;
	if1stats.lasttx = 0;
	if1stats.lastrx = 0;
	if2stats.tx = 0;
	if2stats.rx = 0;
	if2stats.lasttx = 0;
	if2stats.lastrx = 0;
	
	int i = 0;
	for (i=0; i<64; ++i){   
		cpufull[i] = 0;
		cpuidle[i] = 0;
		last_cpuidle[i] = 0;
		last_cpufull[i] = 0;        
	}
	while ((opt = getopt(argc, argv, "s:i:t:n:hvc")) != -1) {
		switch(opt) {
			case 's':
				outopt = OUTPUT_SERIAL;
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
			case 'w':
				dev2 = optarg;
				net = 2;
			case 'n':
				outopt= OUTPUT_NETWORK;
				port = atoi(optarg);
			case 'c':
				outopt = OUTPUT_CONSOLE;
		}
	}
	if(outopt == OUTPUT_SERIAL){
		fd = open (serialpath, O_RDWR | O_NOCTTY | O_SYNC);
		if (fd < 0){
			perror("error opening serial conn");
			outopt = OUTPUT_CONSOLE;
		}

		config_serial (fd, SERIAL_SPEED, 0);  // set speed to 115,200 bps, 8n1 (no parity)
		set_blocking (fd, 0);                // set no blocking
	}
	if (outopt == OUTPUT_NETWORK){

	}

	//char buf [100];
	//int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
	while(1){

		ram_t raminfo = get_ram();
		uptime_t upt = get_uptime();

		double load[3];
		if (getloadavg(load, 3) == -1){
			load[0] = 0;
			load[1] = 0;
			load[2] = 0;
		}

		time_t rawtime;
		struct tm * timeinfo;
		time(&rawtime);
		timeinfo = localtime(&rawtime);

		struct in_addr addr = { 0 };
		if(net==1) addr = get_addr(dev);

		struct in_addr addr2 = { 0 };
		if(net==2) addr2 = get_addr(dev2);
		
		if(verbose){
			printf("%02d-%02d-%04d ",timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
			printf("%02d:%02d:%02d\t", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
			printf("%d days %02d hrs %02d mins %02d secs\t",upt.days, upt.hours, upt.mins, upt.secs);
			printf("RAM %d/%d MB\t",raminfo.used/1024,raminfo.total/1024);
			printf("LOAD %0.2f %0.2f %0.2f\t",load[0],load[1],load[2]);
			printf(" %s\t",inet_ntoa(addr));
			printf(" %lu\n",sizeof(serial_pkt));

		}
		if(outopt == OUTPUT_SERIAL){
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
			uint8_t *iparr = addr2arr(addr);
			serialbuff.ip[0] = iparr[0];
			serialbuff.ip[1] = iparr[1];
			serialbuff.ip[2] = iparr[2];
			serialbuff.ip[3] = iparr[3];
			write (fd, &serialbuff, sizeof(serial_pkt));
			//usleep ((7 + 25) * 100);
			// receive 25:  approx 100 uS per char transmit
		}
		if(outopt == OUTPUT_NETWORK){

		}

		if(outopt == OUTPUT_CONSOLE){
			struct winsize w;
			ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
			// printf("%05d", zipCode);
			//printf ("lines %d columns %d\n", w.ws_row,  w.ws_col);

		// http://www.isthe.com/chongo/tech/comp/ansi_escapes.html
		clearscr();
		time_bar(timeinfo,upt,w.ws_col);
		status_bar(load,w.ws_col);
		ram_bar(raminfo,w.ws_col);
		
		memcpy(last_cpufull, cpufull, sizeof(cpufull));
		memcpy(last_cpuidle, cpuidle, sizeof(cpuidle));
		int cores = get_cpu(cpufull,cpuidle);
		
		cpu_bar(cores,cpufull,cpuidle,last_cpufull,last_cpuidle,w.ws_col);
		
		gettxrx(dev,if1stat);
		gettxrx(dev2,if2stat);
		
		ip_bar(dev,dev2,addr,addr2,net,if1stat,if2stat);
		
		
		}
		sleep(laptime);
	}
	return 0;

}
