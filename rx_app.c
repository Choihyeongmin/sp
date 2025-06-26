/* rx_app.c - RX 상태 모니터링 애플리케이션 */
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#define DEV_PATH "/dev/speed_ctrl_rx"

int main() {
    printf("[RX_APP] RX 모니터링 시작\n");

    int fd = open(DEV_PATH, O_RDONLY);
    if (fd < 0) {
        perror("open /dev/speed_ctrl_rx");
        return 1;
    }

    printf("[RX_APP] 드라이버가 활성화되었습니다. FSM 및 감속 로직이 작동합니다.\n");
    printf("[RX_APP] 상태를 1초마다 확인합니다.\n\n");

    while (1) {
        char buf[128] = {0};
        lseek(fd, 0, SEEK_SET);
        read(fd, buf, sizeof(buf));
        printf("[RX_APP] %s", buf);
        sleep(1);
    }

    close(fd);
    return 0;
}