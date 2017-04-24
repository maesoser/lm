#include "lm.h"


void cpu_bar(int cores, uint32_t *full,uint32_t *idle,uint32_t *lastfull,uint32_t *lastidle,uint32_t cols){
	int n = 1;
	for (n=1; n<cores;n++){
		uint32_t vtotal = full[n]-lastfull[n];
		uint32_t vidle = idle[n]-lastidle[n];
		float cpupcnt = (float)((vtotal - vidle)/(float)(vtotal))*100.0;

		printf(ANSI_COLOR_BOLD " CPU "ANSI_COLOR_RESET);
		int ldigits = ndigits(cpupcnt);
		if (ldigits==0) ldigits=1;
		float totalspace = cols-ldigits - 2 -8 ;
		int usedr = (float)totalspace*((float)cpupcnt/100.0);
		float freer = totalspace -usedr;
		printf(ANSI_COLOR_OKGREEN);
		if (cpupcnt> 50) printf(ANSI_COLOR_WARNING);
		if (cpupcnt> 75) printf(ANSI_COLOR_FAIL);
		fill(usedr);
		printf(ANSI_COLOR_RESET);
		empty(freer);
		printf("%0.1f %% ",cpupcnt);
		printf("\n");
	}
}

void swap_bar(swap_t swap, uint32_t cols){
	printf(ANSI_COLOR_BOLD" SWP " ANSI_COLOR_RESET);
	//printf(ANSI_COLOR_BOLD "["ANSI_COLOR_RESET);
	int ldigits = ndigits(swap.total/1024) + ndigits(swap.used/1024) +1 ;
	float swapfull = ((float)swap.used/(float)swap.total);
	float totalspace = (cols- 8.0 -ldigits);

	int usedr = totalspace*swapfull;

	float freer = totalspace - usedr;

	printf(ANSI_COLOR_OKGREEN);
	if (swapfull > 0.5) printf(ANSI_COLOR_WARNING);
	if (swapfull > 0.7) printf(ANSI_COLOR_FAIL);
	fill(usedr);
	printf(ANSI_COLOR_RESET);
	empty(freer);
	printf("%d/%d M ",swap.used/1024,swap.total/1024);
	printf("\n");
}

void time_bar(struct tm* timeinfo,uptime_t upt,uint32_t cols){
 		printf(ANSI_COLOR_BOLD" TIME"ANSI_COLOR_RESET" %02d-%02d-%04d",timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
		printf("  %02d:%02d:%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
		cols = cols - 18 - 40 - ndigits(upt.days);
		empty(cols);
		printf("  %d days %02d hrs %02d mins %02d secs\n",upt.days, upt.hours, upt.mins, upt.secs);
}

void ip_bar(char *dev, char*dev2,struct in_addr addr,struct in_addr addr2,int ifaces, txrx_t *if1d, txrx_t *if2d,uint32_t cols){
	long long tx,rx;
	char metric = 'B';

	if(ifaces>0){
		printf(ANSI_COLOR_BOLD" NET " ANSI_COLOR_RESET);
		printf("%s: %s",dev,inet_ntoa(addr));

		tx = (if1d->tx - if1d->lasttx);
		rx = (if1d->rx - if1d->lastrx);
		metric = 'B';

		if (rx>1024 || tx > 1024){
			rx = rx/1024;
			tx = tx/1024;
			metric = 'K';
		}

		if (rx>1024 || tx > 1024){
			rx = rx/1024;
			tx = tx/1024;
			metric = 'M';
		}

		empty(13 - addr2size(addr));
		//cols = cols - 45 - ndigits(tx) - ndigits(rx);
		//empty(cols);

		if(tx!=0)
        printf(ANSI_COLOR_BOLD"\tTX:\t"ANSI_COLOR_RESET ANSI_COLOR_GREEN"%Lu%c" ANSI_COLOR_RESET,tx,metric);
        else
        printf(ANSI_COLOR_BOLD"\tTX:\t"ANSI_COLOR_RESET"%Lu%c",tx,metric);

        if(rx!=0)
        printf(ANSI_COLOR_BOLD"\tRX:\t"ANSI_COLOR_RESET ANSI_COLOR_GREEN"%Lu%c"ANSI_COLOR_RESET"\n",rx,metric);
		else
		printf(ANSI_COLOR_BOLD"\tTX:\t"ANSI_COLOR_RESET"%Lu%c",tx,metric);

	}
	if(ifaces>1){
		printf(ANSI_COLOR_BOLD" NET " ANSI_COLOR_RESET);
		printf("%s: %s",dev2,inet_ntoa(addr2));

		tx = (if2d->tx - if2d->lasttx);
		rx = (if2d->rx - if2d->lastrx);
		metric = 'B';

		if (rx>2048 || tx > 2048){
			rx = rx/1024;
			tx = tx/1024;
			metric = 'K';
		}

		if (rx>2048 || tx > 2048){
			rx = rx/1024;
			tx = tx/1024;
			metric = 'M';
		}
		empty(13 - addr2size(addr2));

		if(tx!=0)
        printf(ANSI_COLOR_BOLD"\tTX:\t"ANSI_COLOR_RESET ANSI_COLOR_GREEN"%Lu%c" ANSI_COLOR_RESET,tx,metric);
        else
        printf(ANSI_COLOR_BOLD"\tTX:\t"ANSI_COLOR_RESET"%Lu%c",tx,metric);

        if(rx!=0)
        printf(ANSI_COLOR_BOLD"\tRX:\t"ANSI_COLOR_RESET ANSI_COLOR_GREEN"%Lu%c"ANSI_COLOR_RESET"\n",rx,metric);
		else
		printf(ANSI_COLOR_BOLD"\tTX:\t"ANSI_COLOR_RESET"%Lu%c",tx,metric);

	}
}

void status_bar(double *load,uint32_t cols){
	printf(ANSI_COLOR_BOLD" LOAD"ANSI_COLOR_RESET" %0.2f %0.2f %0.2f",load[0],load[1],load[2]);
	int temp = 0;
	temp = get_temp();
	if (temp<1) temp = get_h3temp();
	cols = cols - 29 - ndigits(temp) -2;
	empty(cols);
	printf(ANSI_COLOR_BOLD "TEMP"ANSI_COLOR_RESET" %d C\n",temp);
}

void ram_bar(ram_t ram, uint32_t cols){
	printf(ANSI_COLOR_BOLD" RAM " ANSI_COLOR_RESET);
	//printf(ANSI_COLOR_BOLD "["ANSI_COLOR_RESET);
	int ldigits = ndigits(ram.total/1024) + ndigits(ram.used/1024) +1 ;
	float ramfull = ((float)ram.used/(float)ram.total);
	float totalspace = (cols- 8.0 -ldigits);

	int usedr = totalspace*ramfull;
	if (usedr==0) usedr = 1;

	float freer = totalspace - usedr;

	printf(ANSI_COLOR_OKGREEN);
	if (ramfull > 0.5) printf(ANSI_COLOR_WARNING);
	if (ramfull > 0.7) printf(ANSI_COLOR_FAIL);
	fill(usedr);
	printf(ANSI_COLOR_RESET);
	empty(freer);
	printf("%d/%d M ",ram.used/1024,ram.total/1024);
	//printf(ANSI_COLOR_BOLD "]"ANSI_COLOR_RESET);

	printf("\n");
}

void storage_bar(swap_t ram, uint32_t cols){
	printf(ANSI_COLOR_BOLD" DSK " ANSI_COLOR_RESET);
	char unit = 'M';
	uint32_t total = ram.total/1024;
	uint32_t used = ram.used/1024;

	if (total > 1024){
		total = total/1024;
		used = used/1024;
		unit = 'G';
	}

	int ldigits = ndigits(total) + ndigits(used) +1 ;
	float ramfull = ((float)used/(float)total);
	float totalspace = (cols- 8.0 -ldigits);

	int usedr = totalspace*ramfull;
	if (usedr==0) usedr = 1;

	float freer = totalspace - usedr;

	printf(ANSI_COLOR_OKGREEN);
	if (ramfull > 0.7) printf(ANSI_COLOR_WARNING);
	if (ramfull > 0.9) printf(ANSI_COLOR_FAIL);
	fill(usedr);
	printf(ANSI_COLOR_RESET);
	empty(freer);

	printf("%d/%d %c ",used,total,unit);

	printf("\n");
}
