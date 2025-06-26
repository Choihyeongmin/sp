// rx_drv.c
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fasync.h>
#include <linux/wait.h>

#define START_BYTE_RESP    0x5A
#define TIMEOUT_MS         1000    // 1초 무입력 시 감속
#define DECEL_STEP         5       // km/h 단위 감속
#define FRAME_LEN          4

// 하드코딩된 BCM 핀 번호
#define RX_CMD_DATA_PIN    17  // RX 수신 데이터
#define RX_CMD_CLK_PIN     22  // RX 수신 클럭
#define RX_ACK_DATA_PIN     5  // ACK 송신 데이터
#define RX_ACK_CLK_PIN     13  // ACK 송신 클럭

#define DEV_NAME_RX       "speed_rx"

static struct gpio_desc *g_rx_data, *g_rx_clk;
static struct gpio_desc *g_ack_data, *g_ack_clk;
static int              rx_irq;
static struct timer_list decel_timer;

static int   current_speed = 0;
static int   speed_state   = 0;   // 0=IDLE,1=ACCEL,2=LIMITED
static unsigned char rx_buf[FRAME_LEN];
static int   bit_cnt = 0, byte_cnt = 0;

// char device
static dev_t           rx_devt;
static struct cdev     rx_cdev;
static struct class   *rx_class;
static wait_queue_head_t rx_wq;
static struct fasync_struct *rx_async;
static unsigned char   event_buf[2];
static bool            event_pending = false;

static unsigned char checksum(const unsigned char *b, int len)
{
    unsigned char cs = 0;
    for (int i = 0; i < len; i++)
        cs ^= b[i];
    return cs;
}

// ACK 프레임 bit-bang 송신
static void send_ack_frame(void)
{
    unsigned char frame[FRAME_LEN] = {
        START_BYTE_RESP,
        (unsigned char)(current_speed & 0xFF),
        (unsigned char)(speed_state   & 0xFF),
        0
    };
    frame[3] = checksum(frame, 3);

    for (int i = 0; i < FRAME_LEN; i++) {
        for (int j = 7; j >= 0; j--) {
            gpiod_set_value(g_ack_data, (frame[i] >> j) & 1);
            gpiod_set_value(g_ack_clk,  1);
            udelay(1);
            gpiod_set_value(g_ack_clk,  0);
            udelay(1);
        }
    }
}

// FSM 업데이트 & 사용자 이벤트 큐잉
static void update_fsm_and_notify(const unsigned char *cmd)
{
    if (cmd[1] == 0x01) { // ACCEL
        current_speed += cmd[2];
        if (current_speed > 100) {
            current_speed = 100;
            speed_state   = 2; // LIMITED
        } else {
            speed_state = 1;   // ACCEL
        }
    }

    // 이벤트 버퍼에 저장
    event_buf[0] = (unsigned char)current_speed;
    event_buf[1] = (unsigned char)speed_state;
    event_pending = true;
    wake_up_interruptible(&rx_wq);
    if (rx_async)
        kill_fasync(&rx_async, SIGIO, POLL_IN);
}

// 타이머 콜백: 무입력 시 감속
static void decel_timer_cb(struct timer_list *t)
{
    if (current_speed <= 0)
        return;

    current_speed = max(0, current_speed - DECEL_STEP);
    if (current_speed == 0)
        speed_state = 0; // IDLE

    send_ack_frame();

    // 사용자 이벤트
    event_buf[0] = (unsigned char)current_speed;
    event_buf[1] = (unsigned char)speed_state;
    event_pending = true;
    wake_up_interruptible(&rx_wq);
    if (rx_async)
        kill_fasync(&rx_async, SIGIO, POLL_IN);

    if (current_speed > 0)
        mod_timer(&decel_timer,
                  jiffies + msecs_to_jiffies(TIMEOUT_MS));
}

// CLK 상승 엣지마다 비트 수신
static irqreturn_t rx_clk_irq(int irq, void *dev)
{
    int bit = gpiod_get_value(g_rx_data) & 1;
    rx_buf[byte_cnt] = (rx_buf[byte_cnt] << 1) | bit;
    bit_cnt++;
    if (bit_cnt == 8) {
        bit_cnt = 0;
        byte_cnt++;
        if (byte_cnt == FRAME_LEN) {
            if (rx_buf[3] == checksum(rx_buf, 3)) {
                // FSM 업데이트 + ACK + notify
                update_fsm_and_notify(rx_buf);
                send_ack_frame();
                mod_timer(&decel_timer,
                          jiffies + msecs_to_jiffies(TIMEOUT_MS));
            }
            byte_cnt = 0;
        }
    }
    return IRQ_HANDLED;
}

// file_operations: read, fasync
static ssize_t rx_read(struct file *filp, char __user *buf,
                       size_t cnt, loff_t *off)
{
    if (cnt < 2)
        return -EINVAL;

    // 이벤트 올 때까지 대기
    if (wait_event_interruptible(rx_wq, event_pending))
        return -ERESTARTSYS;

    // 사용자에게 전송
    if (copy_to_user(buf, event_buf, 2))
        return -EFAULT;

    event_pending = false;
    return 2;
}

static int rx_fasync(int fd, struct file *filp, int mode)
{
    return fasync_helper(fd, filp, mode, &rx_async);
}

static const struct file_operations rx_fops = {
    .owner   = THIS_MODULE,
    .read    = rx_read,
    .fasync  = rx_fasync,
};

static int __init rx_drv_init(void)
{
    int ret;

    // GPIO consumer API
    g_rx_data = gpiod_get_index(NULL, "gpio", RX_CMD_DATA_PIN, GPIOD_IN);
    g_rx_clk  = gpiod_get_index(NULL, "gpio", RX_CMD_CLK_PIN,  GPIOD_IN);
    g_ack_data= gpiod_get_index(NULL, "gpio", RX_ACK_DATA_PIN, GPIOD_OUT_LOW);
    g_ack_clk = gpiod_get_index(NULL, "gpio", RX_ACK_CLK_PIN,  GPIOD_OUT_LOW);
    if (IS_ERR(g_rx_data) || IS_ERR(g_rx_clk) ||
        IS_ERR(g_ack_data)|| IS_ERR(g_ack_clk)) {
        pr_err("rx_drv: GPIO get failed\n");
        return -ENODEV;
    }

    // IRQ 등록
    rx_irq = gpiod_to_irq(g_rx_clk);
    ret = request_irq(rx_irq, rx_clk_irq,
                      IRQF_TRIGGER_RISING, "rx_cmd_clk", NULL);
    if (ret) {
        pr_err("rx_drv: IRQ request failed\n");
        goto err_put;
    }

    // 타이머 초기화
    timer_setup(&decel_timer, decel_timer_cb, 0);

    // /dev/speed_rx
    ret = alloc_chrdev_region(&rx_devt, 0, 1, DEV_NAME_RX);
    if (ret) goto err_irq;
    cdev_init(&rx_cdev, &rx_fops);
    ret = cdev_add(&rx_cdev, rx_devt, 1);
    if (ret) goto err_chrdev;
    rx_class = class_create(THIS_MODULE, DEV_NAME_RX);
    device_create(rx_class, NULL, rx_devt, NULL, DEV_NAME_RX);
    init_waitqueue_head(&rx_wq);

    pr_info("rx_drv loaded, /dev/%s ready\n", DEV_NAME_RX);
    return 0;

err_chrdev:
    unregister_chrdev_region(rx_devt, 1);
err_irq:
    free_irq(rx_irq, NULL);
err_put:
    gpiod_put(g_ack_clk);
    gpiod_put(g_ack_data);
    gpiod_put(g_rx_clk);
    gpiod_put(g_rx_data);
    return ret;
}

static void __exit rx_drv_exit(void)
{
    device_destroy(rx_class, rx_devt);
    class_destroy(rx_class);
    cdev_del(&rx_cdev);
    unregister_chrdev_region(rx_devt, 1);
    del_timer_sync(&decel_timer);
    free_irq(rx_irq, NULL);
    gpiod_put(g_ack_clk);
    gpiod_put(g_ack_data);
    gpiod_put(g_rx_clk);
    gpiod_put(g_rx_data);
    pr_info("rx_drv unloaded\n");
}

module_init(rx_drv_init);
module_exit(rx_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("You");
MODULE_DESCRIPTION("ECU RX driver + event chardev (/dev/speed_rx)");
