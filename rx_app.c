/* rx_app.c - RX 드라이버 작동 유지용 간단 앱 */
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#define DEV_PATH "/dev/speed_ctrl_rx"

void handle_sigint(int sig) {
    printf("\n[RX_APP] Ctrl+C 감지됨. 프로그램을 종료합니다.\n");
    // 이후 close() 및 exit(0)로 자연스럽게 종료됨
}

int main() {
    printf("[RX_APP] RX 드라이버 작동 유지 앱 실행\n");

    // 시그널 핸들러 등록 (Ctrl+C 처리)
    signal(SIGINT, handle_sigint);

    int fd = open(DEV_PATH, O_RDONLY);
    if (fd < 0) {
        perror("open /dev/speed_ctrl_rx");
        return 1;
    }

    printf("[RX_APP] 드라이버가 정상적으로 열렸습니다.\n");
    printf("[RX_APP] 감속 로직이 활성화됩니다. Ctrl+C로 종료하세요.\n");

    pause();  // 시그널이 오기 전까지 블로킹

    close(fd);
    return 0;
}
