#include "oled4linux.h"


uint8_t * addr2arr(struct in_addr ipadrr){
	static uint8_t iparr[4] = {0};
	char *ipstr = inet_ntoa(ipadrr);
	sscanf(ipstr, "%d.%d.%d.%d", &iparr[0], &iparr[1], &iparr[2], &iparr[3]);
	return iparr;
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
		printf("\t -v \tShow debug on stdout\n");
		printf("\t -t [TIME]\tSend time.\n");
		printf("\t -s [SERIAL_PATH]\tSpecify a serial port to send the data\n");
		printf("\t -n [port]\tSend data using udp to the specified port.\n");
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
    if(count==0) count = 1; 
		return count;
}


int main(int argc, char *argv[]){
	unsigned int laptime = DEFAULT_DELAY;
	uint8_t verbose = 0;
	uint8_t outopt = 0;
	uint32_t port = DEFAULT_PORT;
	int opt;
	char dev[32];
	char dev2[32];
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
	while ((opt = getopt(argc, argv, "s:t:n:hvc")) != -1) {
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
		int ifaces = get_ifname(dev,1);
		addr = get_addr(dev);

		struct in_addr addr2 = { 0 };
		if (ifaces > 1){
				get_ifname(dev2,2);
				addr2 = get_addr(dev2);
		}
		
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
		
		memcpy(last_cpufull, cpufull, sizeof(cpufull));
		memcpy(last_cpuidle, cpuidle, sizeof(cpuidle));
		int cores = get_cpu(cpufull,cpuidle);
		
		cpu_bar(cores,cpufull,cpuidle,last_cpufull,last_cpuidle,w.ws_col);
		
		ram_bar(raminfo,w.ws_col);
		swap_t swap = get_swap();
		swap_bar(swap,w.ws_col);
		
		gettxrx(dev,&if1stats);
		gettxrx(dev2,&if2stats);
		
		ip_bar(dev,dev2,addr,addr2,ifaces,&if1stats,&if2stats,w.ws_col);
		
		}
		sleep(laptime);
	}
	return 0;

}
