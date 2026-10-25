/* Native synchronous encoder sampler for wheel_odom.py (input GPIO only).
 * Avoids GPIO edge IRQ delivery to compare raw levels with event-path faults.
 * Sampling can miss transitions: invalid==0 alone does NOT prove no lost counts.
 */
#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <signal.h>
#include <sched.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
static volatile sig_atomic_t stopped;
static void stop(int sig) { (void)sig; stopped=1; }
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec/1e9; }
int main(int argc,char **argv) {
    char *end;
    if(argc<2 || argc>4)return 2;
    double duration=strtod(argv[1],&end);
    if(*end || !(duration>=0 && duration<=600))return 2;
    if(argc>=3) {
        long cpu=strtol(argv[2],&end,10);
        if(*end || cpu<0 || cpu>=CPU_SETSIZE)return 2;
        cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(cpu,&mask);
        if(sched_setaffinity(0,sizeof(mask),&mask)<0){perror("affinity");return 1;}
    }
    if(argc==4) {
        if(strcmp(argv[3],"rt"))return 2;
        struct sched_param priority={.sched_priority=10};
        if(sched_setscheduler(0,SCHED_FIFO,&priority)<0){perror("encoder FIFO permission");return 1;}
    }
    /* FIFO is used only when explicitly granted by the launcher. A bounded
     * 450us work / 50us sleep cycle stays below the board's RT CPU budget;
     * an unbounded FIFO spin would instead be throttled for tens of ms. */
    int realtime = sched_getscheduler(0)==SCHED_FIFO;
    int chip=open("/dev/gpiochip0",O_RDONLY|O_CLOEXEC);
    if(chip<0){perror("chip");return 1;}
    struct gpiohandle_request req={.lineoffsets={0,12,5,1},.flags=GPIOHANDLE_REQUEST_INPUT|GPIOHANDLE_REQUEST_BIAS_PULL_UP,.lines=4};
    snprintf(req.consumer_label,sizeof(req.consumer_label),"encoder_sampler_poll");
    if(ioctl(chip,GPIO_GET_LINEHANDLE_IOCTL,&req)<0){perror("input request");close(chip);return 1;}
    struct gpiohandle_data levels={0};
    if(ioctl(req.fd,GPIOHANDLE_GET_LINE_VALUES_IOCTL,&levels)<0){perror("initial read");return 1;}
    int prev[2]={levels.values[0]*2+levels.values[1],levels.values[2]*2+levels.values[3]};
    const int quad[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    long counts[2]={0},invalid[2]={0},changes[4]={0};
    unsigned char old[4];for(int i=0;i<4;i++)old[i]=levels.values[i];
    signal(SIGINT,stop);signal(SIGTERM,stop);
    double start=now(),last=start,printed=start,maxgap=0,windowgap=0;long samples=0,gaps=0;
    long forward[2]={0},reverse[2]={0};
    double next_yield=start+.00045;
    int status=0;
    while(!stopped && (duration==0 || now()-start<duration)) {
        if(ioctl(req.fd,GPIOHANDLE_GET_LINE_VALUES_IOCTL,&levels)<0){perror("read");status=1;break;}
        double t=now(),gap=t-last;last=t;if(gap>maxgap)maxgap=gap;
        if(gap>windowgap)windowgap=gap;
        gaps+=gap>.001;samples++;
        for(int i=0;i<4;i++){changes[i]+=levels.values[i]!=old[i];old[i]=levels.values[i];}
        for(int i=0;i<2;i++) {
            int cur=levels.values[i*2]*2+levels.values[i*2+1];
            invalid[i]+=(cur^prev[i])==3;
            int delta=quad[prev[i]*4+cur];
            counts[i]+=delta;forward[i]+=delta>0;reverse[i]+=delta<0;prev[i]=cur;
        }
        if(t-printed>=.02 || stopped) {
            printf("{\"elapsed\":%.6f,\"monotonic\":%.9f,\"counts\":[%ld,%ld],"
                   "\"invalid\":[%ld,%ld],\"changes\":[%ld,%ld,%ld,%ld],"
                   "\"samples\":%ld,\"max_gap_ms\":%.3f,\"window_gap_ms\":%.3f,"
                   "\"gaps_over_1ms\":%ld,\"forward\":[%ld,%ld],\"reverse\":[%ld,%ld],\"cpu\":%d}\n",
                   t-start,t,counts[0],counts[1],invalid[0],invalid[1],
                   changes[0],changes[1],changes[2],changes[3],samples,maxgap*1000,
                   windowgap*1000,gaps,forward[0],forward[1],reverse[0],reverse[1],sched_getcpu());
            fflush(stdout);printed=t;windowgap=0;
        }
        if(realtime && t>=next_yield) {
            struct timespec until;
            double deadline=t+.00005;
            until.tv_sec=(time_t)deadline;
            until.tv_nsec=(long)((deadline-until.tv_sec)*1e9);
            int rc;
            do { rc=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&until,NULL); }
            while(rc==EINTR && !stopped);
            if(rc && rc!=EINTR){errno=rc;perror("bounded RT sleep");status=1;break;}
            next_yield=now()+.00045;
        }
    }
    close(req.fd);close(chip);return status;
}
