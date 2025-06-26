/* tx_app.c - TX 애플리케이션 */
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#define DEV_PATH "/dev/speed_ctrl_tx"
#define GPIO_BTN "/dev/gpio20"
#define FRAME_SIZE 4

static int fd_dev = -1;

void sigio_handler(int signo) {
    unsigned char ack[FRAME_SIZE];
    lseek(fd_dev, 0, SEEK_SET);
    int n = read(fd_dev, ack, FRAME_SIZE);
    if (n == FRAME_SIZE) {
        printf("[APP][RECV] ACK: speed=%d, state=0x%02X\n", ack[1], ack[2]);
    } else {
        perror("read");
    }
}

unsigned char calc_checksum(unsigned char *frame) {
    return frame[0] ^ frame[1] ^ frame[2];
}

int main() {
    int fd_btn = open(GPIO_BTN, O_RDONLY);
    if (fd_btn < 0) {
        perror("open button");
        return 1;
    }

    fd_dev = open(DEV_PATH, O_RDWR | O_NONBLOCK);
    if (fd_dev < 0) {
        perror("open device");
        return 1;
    }

    signal(SIGIO, sigio_handler);
    fcntl(fd_dev, F_SETOWN, getpid());
    fcntl(fd_dev, F_SETFL, O_ASYNC | O_NONBLOCK);

    char prev = '0';
    while (1) {
        char buf[2] = {0};
        lseek(fd_btn, 0, SEEK_SET);
        read(fd_btn, buf, 1);

        if (buf[0] == '1' && prev == '0') {  // rising edge
            unsigned char frame[FRAME_SIZE] = {0xAA, 0x01, 10, 0};
            frame[3] = calc_checksum(frame);
            write(fd_dev, frame, FRAME_SIZE);
            printf("[APP][SEND] frame: %02X %02X %02X %02X\n", frame[0], frame[1], frame[2], frame[3]);
        }
        prev = buf[0];
        usleep(100000);
    }

    close(fd_btn);
    close(fd_dev);
    return 0;
}