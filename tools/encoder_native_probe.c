/* Read-only GPIO v1 event diagnostic. No motor outputs or ROS dependencies. */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/gpio.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stopped;
static void stop_signal(int signum) { (void)signum; stopped=1; }
static double now(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC,&t);
    return (double)t.tv_sec+(double)t.tv_nsec/1e9;
}
struct stats {
    uint64_t edges,rising,falling,repeated,nonmonotonic,minimum,last;
    uint32_t previous;
};
static void report(struct stats *s,double elapsed) {
    const char *names[]={"L_A","L_B","R_A","R_B"};
    printf("{\"elapsed_s\":%.2f,\"lines\":[",elapsed);
    for(int i=0;i<4;i++) {
        printf("%s{\"name\":\"%s\",\"edges\":%"PRIu64",\"rising\":%"PRIu64
               ",\"falling\":%"PRIu64",\"repeated_polarity\":%"PRIu64
               ",\"nonmonotonic\":%"PRIu64",\"min_interval_ns\":%"PRIu64"}",
               i?",":"",names[i],s[i].edges,s[i].rising,s[i].falling,
               s[i].repeated,s[i].nonmonotonic,s[i].minimum);
    }
    puts("]}");fflush(stdout);
}
int main(int argc,char **argv) {
    if(argc!=2) { fprintf(stderr,"Usage: %s seconds (0 waits until SIGINT; maximum timed duration 600)\n",argv[0]);return 2; }
    char *end;double duration=strtod(argv[1],&end);
    if(*end || !(duration>=0 && duration<=600))return 2;
    signal(SIGINT,stop_signal);signal(SIGTERM,stop_signal);
    int offsets[]={0,12,5,1};
    struct pollfd fds[4];
    for(int i=0;i<4;i++)fds[i]=(struct pollfd){.fd=-1,.events=POLLIN};
    int chip=open("/dev/gpiochip0",O_RDONLY|O_CLOEXEC);
    if(chip<0){perror("open gpiochip0");return 1;}
    int status=0;
    for(int i=0;i<4;i++) {
        struct gpioevent_request req={0};req.lineoffset=(unsigned)offsets[i];
        req.handleflags=GPIOHANDLE_REQUEST_INPUT|GPIOHANDLE_REQUEST_BIAS_PULL_UP;
        req.eventflags=GPIOEVENT_REQUEST_BOTH_EDGES;
        snprintf(req.consumer_label,sizeof(req.consumer_label),"encoder_native_probe");
        if(ioctl(chip,GPIO_GET_LINEEVENT_IOCTL,&req)<0){perror("request INPUT");status=1;goto cleanup;}
        fds[i].fd=req.fd;
        int flags=fcntl(req.fd,F_GETFL);
        if(flags<0 || fcntl(req.fd,F_SETFL,flags|O_NONBLOCK)<0){perror("nonblock");status=1;goto cleanup;}
    }
    struct stats stats[4]={{0}};
    double start=now(),printed=start;
    puts("INPUT ONLY: native GPIO v1 event reader ready");fflush(stdout);
    while(!stopped && (duration==0 || now()-start<duration)) {
        int ready=poll(fds,4,20);
        if(ready<0){if(errno==EINTR)continue;perror("poll");status=1;break;}
        for(int i=0;i<4;i++) {
            if(fds[i].revents&(POLLERR|POLLHUP|POLLNVAL)){status=1;stopped=1;break;}
            if(!(fds[i].revents&POLLIN))continue;
            for(int batch=0;batch<32;batch++) {
                struct gpioevent_data events[64];
                ssize_t n=read(fds[i].fd,events,sizeof(events));
                if(n<0){if(errno==EAGAIN||errno==EINTR)break;perror("read");status=1;stopped=1;break;}
                if(n==0 || n%(ssize_t)sizeof(events[0])){status=1;stopped=1;break;}
                for(size_t j=0;j<(size_t)n/sizeof(events[0]);j++) {
                    struct stats *s=&stats[i];struct gpioevent_data ev=events[j];
                    if(s->edges) {
                        if(ev.id==s->previous)s->repeated++;
                        if(ev.timestamp<=s->last)s->nonmonotonic++;
                        else if(!s->minimum||ev.timestamp-s->last<s->minimum)s->minimum=ev.timestamp-s->last;
                    }
                    s->edges++;s->rising+=ev.id==GPIOEVENT_EVENT_RISING_EDGE;
                    s->falling+=ev.id==GPIOEVENT_EVENT_FALLING_EDGE;
                    s->last=ev.timestamp;s->previous=ev.id;
                }
            }
        }
        if(now()-printed>=2){report(stats,now()-start);printed=now();}
    }
    report(stats,now()-start);
cleanup:
    for(int i=0;i<4;i++)if(fds[i].fd>=0)close(fds[i].fd);
    close(chip);return status;
}
