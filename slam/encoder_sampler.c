/* Input-only simultaneous A/B sampling. Native encoder acquisition; requires independent calibration and health checks.
 * Avoids GPIO edge IRQ delivery to compare raw levels with event-path faults.
 * Sampling can miss transitions: invalid==0 alone does NOT prove no lost counts.
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <signal.h>
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
    double duration=argc>1?atof(argv[1]):60;
    if(!(duration>=0 && duration<=600))return 2;
    int chip=open("/dev/gpiochip0",O_RDONLY|O_CLOEXEC);
    if(chip<0){perror("chip");return 1;}
    struct gpiohandle_request req={.lineoffsets={0,12,5,1},.flags=GPIOHANDLE_REQUEST_INPUT|GPIOHANDLE_REQUEST_BIAS_PULL_UP,.lines=4};
    snprintf(req.consumer_label,sizeof(req.consumer_label),"encoder_sampler");
    if(ioctl(chip,GPIO_GET_LINEHANDLE_IOCTL,&req)<0){perror("input request");close(chip);return 1;}
    struct gpiohandle_data levels={0};
    if(ioctl(req.fd,GPIOHANDLE_GET_LINE_VALUES_IOCTL,&levels)<0){perror("initial read");return 1;}
    int prev[2]={levels.values[0]*2+levels.values[1],levels.values[2]*2+levels.values[3]};
    const int quad[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    long counts[2]={0},invalid[2]={0},changes[4]={0};
    unsigned char old[4];for(int i=0;i<4;i++)old[i]=levels.values[i];
    signal(SIGINT,stop);signal(SIGTERM,stop);
    double start=now(),last=start,printed=start,maxgap=0;long samples=0;
    int status=0;
    while(!stopped && (duration==0 || now()-start<duration)) {
        if(ioctl(req.fd,GPIOHANDLE_GET_LINE_VALUES_IOCTL,&levels)<0){perror("read");status=1;break;}
        double t=now(),gap=t-last;last=t;if(gap>maxgap)maxgap=gap;samples++;
        for(int i=0;i<4;i++){changes[i]+=levels.values[i]!=old[i];old[i]=levels.values[i];}
        for(int i=0;i<2;i++) {
            int cur=levels.values[i*2]*2+levels.values[i*2+1];
            invalid[i]+=(cur^prev[i])==3;
            counts[i]+=quad[prev[i]*4+cur];prev[i]=cur;
        }
        if(t-printed>=0.02 || stopped) {
            printf("{\"elapsed\":%.6f,\"monotonic\":%.9f,\"counts\":[%ld,%ld],\"invalid\":[%ld,%ld],\"changes\":[%ld,%ld,%ld,%ld],\"samples\":%ld,\"max_gap_ms\":%.3f}\n",t-start,t,counts[0],counts[1],invalid[0],invalid[1],changes[0],changes[1],changes[2],changes[3],samples,maxgap*1000);
            fflush(stdout);printed=t;
        }
        /* Optional fast mode is a bounded diagnostic on one CPU core. */
        /* Full-rate native sampling: one core, no Python GIL or edge IRQ queue. */
    }
    close(req.fd);close(chip);return status;
}
