// tx_drv.c
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fasync.h>
#include <linux/wait.h>
#include <linux/delay.h>

#define DEV_NAME          "speed_ctrl"
#define START_BYTE_CMD    0xA5
#define FRAME_LEN         4

// 하드코딩된 BCM 핀 번호
#define TX_CMD_DATA_PIN    4   // TX → RX 데이터
#define TX_CMD_CLK_PIN    27   // TX → RX 클럭
#define TX_ACK_DATA_PIN    6   // RX → TX 데이터
#define TX_ACK_CLK_PIN    19  // RX → TX 클럭

static struct gpio_desc *g_tx_data, *g_tx_clk;
static struct gpio_desc *g_ack_data, *g_ack_clk;
static int               ack_irq;

static dev_t             tx_devt;
static struct cdev       tx_cdev;
static struct class     *tx_class;

static wait_queue_head_t tx_wq;
static struct fasync_struct *tx_async;
static unsigned char     ack_buf[FRAME_LEN];
static int               ack_bit_cnt, ack_byte_cnt;
static bool              ack_pending = false;

// XOR checksum
static unsigned char checksum(const unsigned char *b, int len)
{
    unsigned char cs = 0;
    while (len--) cs ^= *b++;
    return cs;
}

// 명령 프레임 비트뱅잉 송신
static void send_cmd_frame(const unsigned char *frame)
{
    int i, j;
    for (i = 0; i < FRAME_LEN; i++) {
        for (j = 7; j >= 0; j--) {
            gpiod_set_value(g_tx_data, (frame[i] >> j) & 1);
            gpiod_set_value(g_tx_clk,  1);
            udelay(1);
            gpiod_set_value(g_tx_clk,  0);
            udelay(1);
        }
    }
}

// ACK 클럭 상승 엣지 IRQ 핸들러
static irqreturn_t ack_clk_irq(int irq, void *dev)
{
    int bit = gpiod_get_value(g_ack_data) & 1;
    ack_buf[ack_byte_cnt] = (ack_buf[ack_byte_cnt] << 1) | bit;
    if (++ack_bit_cnt == 8) {
        ack_bit_cnt = 0;
        if (++ack_byte_cnt == FRAME_LEN) {
            if (ack_buf[3] == checksum(ack_buf, 3)) {
                ack_pending = true;
                wake_up_interruptible(&tx_wq);
                if (tx_async)
                    kill_fasync(&tx_async, SIGIO, POLL_IN);
            }
            ack_byte_cnt = 0;
        }
    }
    return IRQ_HANDLED;
}

static ssize_t tx_write(struct file *filp,
                        const char __user *buf,
                        size_t len, loff_t *off)
{
    unsigned char frame[FRAME_LEN];
    if (len != FRAME_LEN) return -EINVAL;
    if (copy_from_user(frame, buf, FRAME_LEN)) return -EFAULT;
    if (frame[0] != START_BYTE_CMD ||
        frame[3] != checksum(frame, 3))
        return -EINVAL;
    send_cmd_frame(frame);
    return FRAME_LEN;
}

static ssize_t tx_read(struct file *filp,
                       char __user *buf,
                       size_t len, loff_t *off)
{
    if (wait_event_interruptible(tx_wq, ack_pending))
        return -ERESTARTSYS;
    if (copy_to_user(buf, ack_buf, FRAME_LEN))
        return -EFAULT;
    ack_pending = false;
    return FRAME_LEN;
}

static int tx_fasync(int fd, struct file *filp, int mode)
{
    return fasync_helper(fd, filp, mode, &tx_async);
}

static const struct file_operations tx_fops = {
    .owner   = THIS_MODULE,
    .write   = tx_write,
    .read    = tx_read,
    .fasync  = tx_fasync,
};

static int __init tx_drv_init(void)
{
    int ret;

    // GPIO consumer API로 획득
    g_tx_data = gpiod_get_index(NULL, "gpio", TX_CMD_DATA_PIN, GPIOD_OUT_LOW);
    g_tx_clk  = gpiod_get_index(NULL, "gpio", TX_CMD_CLK_PIN,  GPIOD_OUT_LOW);
    g_ack_data= gpiod_get_index(NULL, "gpio", TX_ACK_DATA_PIN, GPIOD_IN);
    g_ack_clk = gpiod_get_index(NULL, "gpio", TX_ACK_CLK_PIN,  GPIOD_IN);
    if (IS_ERR(g_tx_data)||IS_ERR(g_tx_clk)||
        IS_ERR(g_ack_data)||IS_ERR(g_ack_clk)) {
        pr_err("tx_drv: GPIO get failed\n");
        return -ENODEV;
    }

    // IRQ 등록
    ack_irq = gpiod_to_irq(g_ack_clk);
    ret = request_irq(ack_irq, ack_clk_irq,
                      IRQF_TRIGGER_RISING, "tx_ack_clk", NULL);
    if (ret) {
        pr_err("tx_drv: request_irq failed\n");
        goto err_gpio;
    }

    init_waitqueue_head(&tx_wq);

    // char device 등록
    ret = alloc_chrdev_region(&tx_devt, 0, 1, DEV_NAME);
    if (ret) goto err_irq;
    cdev_init(&tx_cdev, &tx_fops);
    tx_cdev.owner = THIS_MODULE;
    ret = cdev_add(&tx_cdev, tx_devt, 1);
    if (ret) goto err_chrdev;
    tx_class = class_create(THIS_MODULE, DEV_NAME);
    device_create(tx_class, NULL, tx_devt, NULL, DEV_NAME);

    pr_info("tx_drv loaded (BCM %d,%d→%d,%d), major=%d\n",
        TX_CMD_DATA_PIN, TX_CMD_CLK_PIN,
        TX_ACK_DATA_PIN, TX_ACK_CLK_PIN,
        MAJOR(tx_devt));
    return 0;

err_chrdev:
    unregister_chrdev_region(tx_devt,1);
err_irq:
    free_irq(ack_irq, NULL);
err_gpio:
    gpiod_put(g_ack_clk);
    gpiod_put(g_ack_data);
    gpiod_put(g_tx_clk);
    gpiod_put(g_tx_data);
    return ret;
}

static void __exit tx_drv_exit(void)
{
    device_destroy(tx_class, tx_devt);
    class_destroy(tx_class);
    cdev_del(&tx_cdev);
    unregister_chrdev_region(tx_devt,1);
    free_irq(ack_irq, NULL);
    gpiod_put(g_ack_clk);
    gpiod_put(g_ack_data);
    gpiod_put(g_tx_clk);
    gpiod_put(g_tx_data);
    pr_info("tx_drv unloaded\n");
}

module_init(tx_drv_init);
module_exit(tx_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("You");
MODULE_DESCRIPTION("ECU TX driver using gpio/consumer.h");
