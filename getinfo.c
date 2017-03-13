#include "oled4linux.h"


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

float get_h3temp(){
        FILE *fp;
        int temp = 0;
        fp = fopen("/etc/armbianmonitor/datasources/soctemp", "r");
        if(fp==NULL) return -1;
        fscanf(fp, "%d", &temp);
        fclose(fp);
        return (float)temp;
}
void gettxrx(char *dev, txrx_t *if1){
	 FILE *netinfo = fopen("/proc/net/dev", "r");
	 if(netinfo == NULL){
		 exit(-1);
	 }
	 char buff[256];
	 while(fgets(buff, sizeof(buff), netinfo)){
		 long long tx = 0;
		 long long rx = 0;
		 char ifname[32];
		 sscanf( buff," %[^:]: %Lu %*u %*u %*u %*u %*u %*u %*u %Lu %*u %*u %*u %*u %*u %*u %*u",ifname, &rx, &tx );				 
			if(strstr(ifname,dev)!=NULL){
				//printf("%s %Lu %Lu tx/rx\n",ifname ,tx,rx); 
				if1->lasttx = if1->tx;
				if1->lastrx = if1->rx;
				if1->tx = tx;
				if1->rx = rx;
			}
	 }
	 if(fclose(netinfo) != 0){
		 exit(-1);
	 }	 
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

swap_t get_swap(){
	swap_t swap;
	FILE *meminfo = fopen("/proc/swaps", "r");
	if(meminfo == NULL){
		exit(-1);
	}
	char buff[256];
	while(fgets(buff, sizeof(buff), meminfo)){
		uint32_t total = 0;
		uint32_t used = 0;
		char name[64];
		sscanf(buff, "%s %*s %d %d", name, &total,&used) ;
			swap.total = total;
			swap.used = used;
	}
	if(fclose(meminfo) != 0){
		exit(-1);
	}
	return swap;

}

int get_ifname(char *iface,int index){
	struct ifaddrs *addrs, *tmp;
	getifaddrs(&addrs);
	tmp = addrs;
	int count = 0;
	while(tmp){
		if(tmp->ifa_addr && tmp->ifa_addr->sa_family == AF_PACKET){
			if(count==index)strcpy(iface,tmp->ifa_name);
			count ++;
		}
		tmp = tmp->ifa_next;
	}
	freeifaddrs(addrs);
	return count;
}

swap_t get_disk_byname(char *mntpoint){
	
	char *filename = "/etc/mtab";
	FILE *fp;
	struct mntent *fs;
	struct statfs vfs;
	
	swap_t disk;
	disk.total = 0;
	disk.used = 0;

	fp = setmntent(filename, "r");	/* read only */
	if (fp == NULL) {
		printf(" %s: could not open: %s\n", filename, strerror(errno));
		return disk;
	}
	while ((fs = getmntent(fp)) != NULL){
		 if(strcmp(fs->mnt_fsname, mntpoint)==0){
			if (statfs(fs->mnt_dir, & vfs) != 0) {
				printf(" %s: statfs failed: %s\n", fs->mnt_dir, strerror(errno));
				return disk;
			}
			//printf("%s, mounted on %s:", fs->mnt_dir, fs->mnt_fsname);
			//printf("\tf_bsize: %ld", vfs.f_blocks * vfs.f_bsize);
			//printf("\tf_bfree: %ld\n", vfs.f_bfree * vfs.f_bsize);
			//			printf("%d",strlen(fs->mnt_dir));
			//printf("f_namelen: %ld\n", vfs.f_namelen);
			disk.total = (vfs.f_blocks * vfs.f_bsize)/1024;
			disk.used = (vfs.f_bfree * vfs.f_bsize)/1024;
			
		}
	}

	endmntent(fp);
	return disk;
}

swap_t get_disk_bymnt(char *mntpoint){
	
	char *filename = "/etc/mtab";
	FILE *fp;
	struct mntent *fs;
	struct statfs vfs;
	
	swap_t disk;
	disk.total = 0;
	disk.used = 0;

	fp = setmntent(filename, "r");	/* read only */
	if (fp == NULL) {
		printf(" %s: could not open: %s\n", filename, strerror(errno));
		return disk;
	}
	while ((fs = getmntent(fp)) != NULL){
		 if(strcmp(fs->mnt_dir, mntpoint)==0){
			if (statfs(fs->mnt_dir, & vfs) != 0) {
				printf(" %s: statfs failed: %s\n", fs->mnt_dir, strerror(errno));
				return disk;
			}
			//printf("%s, mounted on %s:", fs->mnt_dir, fs->mnt_fsname);
			//printf("\tf_bsize: %ld", vfs.f_blocks * vfs.f_bsize);
			//printf("\tf_bfree: %ld\n", vfs.f_bfree * vfs.f_bsize);
			//			printf("%d",strlen(fs->mnt_dir));
			//printf("f_namelen: %ld\n", vfs.f_namelen);
			disk.total = (vfs.f_blocks * vfs.f_bsize)/1024;
			disk.used = (vfs.f_bfree * vfs.f_bsize)/1024;
			
		}
	}

	endmntent(fp);
	return disk;
}
