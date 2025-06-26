/* rx_app.c - RX 상태 모니터링 애플리케이션 */
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

#define DEV_PATH "/dev/speed_ctrl_rx"

volatile int keep_running = 1;

void handle_sigint(int sig) {
    keep_running = 0;
}

int main() {
    printf("[RX_APP] RX 모니터링 시작\n");

    int fd = open(DEV_PATH, O_RDONLY);
    if (fd < 0) {
        perror("open /dev/speed_ctrl_rx");
        return 1;
    }

    signal(SIGINT, handle_sigint);

    printf("[RX_APP] 드라이버가 활성화되었습니다. FSM 및 감속 로직이 작동합니다.\n");
    printf("[RX_APP] 상태를 1초마다 확인합니다. (Ctrl+C로 종료)\n\n");

    while (keep_running) {
        char buf[128] = {0};
        ssize_t n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            printf("[RX_APP] %s", buf);
            fflush(stdout);
        } else {
            perror("read");
            break;
        }
        sleep(1);
    }

    close(fd);
    printf("\n[RX_APP] 종료되었습니다.\n");
    return 0;
}
