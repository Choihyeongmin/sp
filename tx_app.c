// tx_app.c
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#define DEV_PATH       "/dev/speed_ctrl"
#define START_BYTE     0xA5
#define CMD_ACCEL      0x01
#define ACCEL_STEP     10
#define FRAME_LEN      4

volatile sig_atomic_t got_ack = 0;
void sigio_handler(int sig) { got_ack = 1; }

unsigned char checksum(const unsigned char *b, int len) {
    unsigned char cs = 0;
    for (int i = 0; i < len; i++) cs ^= b[i];
    return cs;
}

void build_frame(unsigned char *f) {
    f[0] = START_BYTE;
    f[1] = CMD_ACCEL;
    f[2] = ACCEL_STEP;
    f[3] = checksum(f, 3);
}

int main() {
    int fd = open(DEV_PATH, O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    // SIGIO 설정
    fcntl(fd, F_SETOWN, getpid());
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_ASYNC);
    signal(SIGIO, sigio_handler);

    unsigned char frame[FRAME_LEN], ack[FRAME_LEN];
    build_frame(frame);
    printf("Enter → +%dkm/h ACCEL (Ctrl+C to exit)\n", ACCEL_STEP);

    while (1) {
        if (getchar() == '\n') {
            if (write(fd, frame, FRAME_LEN) != FRAME_LEN)
                perror("write");
        }
        if (got_ack) {
            got_ack = 0;
            if (read(fd, ack, FRAME_LEN) == FRAME_LEN)
                printf("[ACK] speed=%d, state=%s\n",
                       ack[1],
                       ack[2]==1 ? "ACCEL" :
                       ack[2]==2 ? "LIMITED" : "IDLE");
        }
    }

    close(fd);
    return 0;
}
