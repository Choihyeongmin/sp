// rx_app.c
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>

#define DEV_RX   "/dev/speed_rx"

volatile sig_atomic_t stop = 0;
void sighandler(int sig){ stop = 1; }

int main(void) {
    int fd;
    unsigned char buf[2];

    signal(SIGINT,  sighandler);
    signal(SIGTERM, sighandler);

    fd = open(DEV_RX, O_RDONLY);
    if (fd < 0) {
        perror("open " DEV_RX);
        return 1;
    }

    printf("[RX_APP] listening on %s (Ctrl+C to quit)\n", DEV_RX);
    while (!stop) {
        int n = read(fd, buf, 2);
        if (n < 0) {
            if (stop) break;
            perror("read");
            break;
        } else if (n == 2) {
            printf("[RX_APP] speed=%3u km/h, state=%s\n",
                   buf[0],
                   buf[1]==1 ? "ACCEL" :
                   buf[1]==2 ? "LIMITED" : "IDLE");
        }
    }

    close(fd);
    printf("[RX_APP] exiting\n");
    return 0;
}
