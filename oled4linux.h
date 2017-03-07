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
#include <ifaddrs.h>

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
uint32_t total;
uint32_t used;
}swap_t;

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


uint8_t * addr2arr(struct in_addr ipadrr);
void clearscr();
void fill(int n);
void empty(int n);
int config_serial (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
void print_help();
uint8_t ndigits(int n);


void ram_bar(ram_t ram, uint32_t cols);
void status_bar(double *load,uint32_t cols);
void ip_bar(char *dev, char*dev2,struct in_addr addr,struct in_addr addr2,int net, txrx_t *if1d, txrx_t *if2d,uint32_t cols);
void time_bar(struct tm* timeinfo,uptime_t upt,uint32_t cols);
void swap_bar(swap_t swap, uint32_t cols);
void cpu_bar(int cores, uint32_t *full,uint32_t *idle,uint32_t *lastfull,uint32_t *lastidle,uint32_t cols);

uint32_t get_cpu(uint32_t *cpufull,uint32_t *cpuidle);
float get_temp();
float get_h3temp();
void gettxrx(char *dev, txrx_t *if1);
struct in_addr get_addr(char *dev);
uptime_t get_uptime();
ram_t get_ram();
swap_t get_swap();
int get_ifname(char *iface,int index);
