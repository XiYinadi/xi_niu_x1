/*
 * Event-driven quadrature sampler — replaces the busy-poll GPIOHANDLE loop.
 *
 * 2026-10-19: the old version polled GPIOHANDLE_GET_LINE_VALUES in a tight loop
 * (94% of one core, forever, even when the wheels were still). This one blocks
 * in poll() on four GPIO_GET_LINEEVENT fds and only wakes on an edge, so idle
 * CPU is ~0. Kernel queues can still lose edges under load; this implementation
 * does not claim that every physical transition is captured.
 *
 * Contract (unchanged): one JSON line every 20 ms on stdout with
 *   elapsed, monotonic, counts[2], invalid[2], changes[4], samples,
 *   max_gap_ms, window_gap_ms, gaps_over_1ms, forward[2], reverse[2], cpu, events
 *   counts[i] are signed x4 quadrature counts; invalid[i] counts dropped/
 *   inconsistent transitions (a phase-loss indicator, not proof of none).
 *
 * Usage: encoder_sampler_ev <duration_s> [cpu]
 *   duration 0 = run until SIGINT/SIGTERM.  cpu is an optional affinity mask.
 */
#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stopped;
static void stop(int sig) { (void)sig; stopped = 1; }
static double now(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

/* A(0),B(1) / A(2),B(3) — line order matches the wiring table */
static const unsigned int OFFS[4] = {0, 12, 5, 1};
static const int QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

/* Merge the heads of the four kernel queues by acquisition timestamp. Draining
 * A completely before B destroys quadrature chronology when both have backlog. */
static int earliest(const struct gpioevent_data head[4], const int ready[4]) {
    int best = -1;
    for (int i = 0; i < 4; i++)
        if (ready[i] && (best < 0 || head[i].timestamp < head[best].timestamp))
            best = i;
    return best;
}

static void decode(int line, unsigned int id, int prev[2], long counts[2],
                   long invalid[2], long forward[2], long reverse[2]) {
    int pair = line / 2, mask = line % 2 == 0 ? 2 : 1;
    int st = prev[pair];
    int nx = id == GPIOEVENT_EVENT_RISING_EDGE ? st | mask : st & ~mask;
    if (nx == st) { invalid[pair]++; return; }
    int d = QUAD[(st << 2) | nx];
    counts[pair] += d;
    forward[pair] += d > 0;
    reverse[pair] += d < 0;
    prev[pair] = nx;
}

static int self_test(void) {
    /* Four queued cycles, both wheels. Each line has its own FIFO, exactly as
     * the GPIO v1 interface does. Old per-line draining gives wrong counts. */
    struct gpioevent_data queues[4][8], head[4];
    int cursor[4] = {0}, ready[4] = {1,1,1,1}, prev[2] = {0};
    long counts[2] = {0}, invalid[2] = {0}, forward[2] = {0}, reverse[2] = {0};
    for (int line = 0; line < 4; line++) {
        for (int n = 0; n < 8; n++) {
            queues[line][n].timestamp = (uint64_t)(n * 4 + line % 2 + 1);
            queues[line][n].id = n % 2 == 0 ? GPIOEVENT_EVENT_RISING_EDGE : GPIOEVENT_EVENT_FALLING_EDGE;
        }
        head[line] = queues[line][0];
    }
    int i;
    while ((i = earliest(head, ready)) >= 0) {
        decode(i, head[i].id, prev, counts, invalid, forward, reverse);
        ready[i] = ++cursor[i] < 8;
        if (ready[i]) head[i] = queues[i][cursor[i]];
    }
    if (counts[0] != 16 || counts[1] != 16 || invalid[0] || invalid[1]) return 1;
    decode(0, GPIOEVENT_EVENT_FALLING_EDGE, prev, counts, invalid, forward, reverse);
    if (invalid[0] != 1) return 1; /* Real repeated edges must remain faults. */
    puts("chronological backlog + two wheels + repeated-edge detection: PASS");
    return 0;
}

static int read_level(int fd) {
    struct gpiohandle_data d;
    memset(&d, 0, sizeof d);
    if (ioctl(fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &d) < 0)
        return -1;
    return d.values[0] ? 1 : 0;
}

int main(int argc, char **argv) {
    if (argc == 2 && strcmp(argv[1], "--self-test") == 0) return self_test();
    char *end;
    if (argc < 2 || argc > 3)
        return 2;
    double duration = strtod(argv[1], &end);
    if (*end || !(duration >= 0 && duration <= 600))
        return 2;
    if (argc == 3) {
        long cpu = strtol(argv[2], &end, 10);
        if (*end || cpu < 0 || cpu >= CPU_SETSIZE)
            return 2;
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(cpu, &mask);
        if (sched_setaffinity(0, sizeof(mask), &mask) < 0) {
            perror("sampler CPU affinity");
            return 1;
        }
    }

    int chip = open("/dev/gpiochip0", O_RDONLY | O_CLOEXEC);
    if (chip < 0) {
        perror("chip");
        return 1;
    }

    int fds[4];
    struct pollfd pfd[4];
    for (int i = 0; i < 4; i++) {
        struct gpioevent_request req;
        memset(&req, 0, sizeof req);
        req.lineoffset = OFFS[i];
        req.handleflags = GPIOHANDLE_REQUEST_INPUT | GPIOHANDLE_REQUEST_BIAS_PULL_UP;
        req.eventflags = GPIOEVENT_REQUEST_BOTH_EDGES;
        snprintf(req.consumer_label, sizeof req.consumer_label, "encoder_sampler_ev");
        if (ioctl(chip, GPIO_GET_LINEEVENT_IOCTL, &req) < 0) {
            fprintf(stderr, "lineevent request line %u: %s\n", OFFS[i], strerror(errno));
            return 1;
        }
        fds[i] = req.fd;
        /* 必须非阻塞:排空事件时最后那次 read() 会一直等下一个边沿,
         * 加上 signal() 默认 SA_RESTART,SIGTERM 也打断不了 => 进程永不退出
         * (2026-10-19 实测:duration=3 跑了 3 分钟还不走)。 */
        int fl = fcntl(fds[i], F_GETFL, 0);
        if (fl < 0 || fcntl(fds[i], F_SETFL, fl | O_NONBLOCK) < 0) {
            perror("nonblocking event fd");
            return 1;
        }
        pfd[i].fd = req.fd;
        pfd[i].events = POLLIN;
    }

    /* 建 fd 时就可能已有排队事件(电平早就稳定了),先排空再读初始电平 ——
     * 否则第一个事件会被当成「重复边沿」记一次 invalid。 */
    for (int i = 0; i < 4; i++) {
        struct gpioevent_data ev;
        while (read(fds[i], &ev, sizeof ev) == (ssize_t)sizeof ev) { }
    }
    int level[4];
    for (int i = 0; i < 4; i++) {
        level[i] = read_level(fds[i]);
        if (level[i] < 0) {
            perror("initial read");
            return 1;
        }
    }
    int prev[2] = {level[0] * 2 + level[1], level[2] * 2 + level[3]};

    long counts[2] = {0}, invalid[2] = {0}, changes[4] = {0};
    long forward[2] = {0}, reverse[2] = {0}, events = 0;

    signal(SIGINT, stop);
    signal(SIGTERM, stop);

    double start = now(), printed = start, lastwake = start;
    double maxgap = 0, windowgap = 0;
    long samples = 0, gaps = 0;

    struct gpioevent_data head[4];
    int ready[4] = {0};
    while (!stopped && (duration == 0 || now() - start < duration)) {
        int r = poll(pfd, 4, 20);
        double t = now();
        double gap = t - lastwake;
        lastwake = t;
        if (gap > maxgap) maxgap = gap;
        if (gap > windowgap) windowgap = gap;
        if (gap > .001) gaps++;
        samples++;

        if (r < 0 && errno != EINTR) { perror("poll"); return 1; }
        for (int i = 0; i < 4; i++)
            if (pfd[i].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                fprintf(stderr, "encoder event fd failed line %d\n", i);
                return 1;
            }
        /* One lookahead per line, refill only the earliest head. Never decode
         * separate per-line batches. Bound work so reporting cannot starve. */
        for (int n = 0; n < 4096 && !stopped; n++) {
            for (int i = 0; i < 4; i++) {
                if (ready[i]) continue;
                ssize_t got = read(fds[i], &head[i], sizeof head[i]);
                if (got == (ssize_t)sizeof head[i]) ready[i] = 1;
                else if (got < 0 && (errno == EAGAIN || errno == EINTR)) continue;
                else { fprintf(stderr, "encoder event read failed line %d\n", i); return 1; }
            }
            int i = earliest(head, ready);
            if (i < 0) break;
            events++;
            changes[i]++;
            decode(i, head[i].id, prev, counts, invalid, forward, reverse);
            ready[i] = 0;
        }
        /* A live-level read races queued edges and is not a coherent snapshot.
         * Do not synthesize counts or resync state from those reads. Repeated
         * edges still flag phase loss; v1 cannot prove absence of full-cycle loss. */
        t = now();
        if (t - printed >= 0.02 || stopped) {
            printf("{\"elapsed\":%.6f,\"monotonic\":%.9f,\"counts\":[%ld,%ld],"
                   "\"invalid\":[%ld,%ld],\"changes\":[%ld,%ld,%ld,%ld],"
                   "\"samples\":%ld,\"max_gap_ms\":%.3f,\"window_gap_ms\":%.3f,"
                   "\"gaps_over_1ms\":%ld,\"forward\":[%ld,%ld],\"reverse\":[%ld,%ld],"
                   "\"cpu\":%d,\"events\":%ld}\n",
                   t - start, t, counts[0], counts[1], invalid[0], invalid[1],
                   changes[0], changes[1], changes[2], changes[3], samples,
                   maxgap * 1000, windowgap * 1000, gaps, forward[0], forward[1],
                   reverse[0], reverse[1], sched_getcpu(), events);
            fflush(stdout);
            printed = t;
            windowgap = 0;
        }
    }

    for (int i = 0; i < 4; i++) close(fds[i]);
    close(chip);
    return 0;
}
