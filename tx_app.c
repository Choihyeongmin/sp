/* tx_app.c - TX 응답 모니터링 애플리케이션 (버튼 처리 제거됨) */
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
#define FRAME_SIZE 4

static int fd_dev = -1;

void sigio_handler(int signo) {
    unsigned char ack[FRAME_SIZE];
    lseek(fd_dev, 0, SEEK_SET);
    int n = read(fd_dev, ack, FRAME_SIZE);
    if (n == FRAME_SIZE) {
        printf("[TX_APP][RECV] ACK: speed=%d, state=0x%02X\n", ack[1], ack[2]);
    } else {
        perror("read");
    }
}

int main() {
    fd_dev = open(DEV_PATH, O_RDWR | O_NONBLOCK);
    if (fd_dev < 0) {
        perror("open device");
        return 1;
    }

    signal(SIGIO, sigio_handler);
    fcntl(fd_dev, F_SETOWN, getpid());
    fcntl(fd_dev, F_SETFL, O_ASYNC | O_NONBLOCK);

    printf("[TX_APP] 응답 수신 대기 중 (SIGIO)...\n");
    while (1) pause();

    close(fd_dev);
    return 0;
}