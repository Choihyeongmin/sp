/* tx_drv.c - TX 측 드라이버 (ACK 수신 전용, SIGIO 제거) */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define DEVICE_NAME     "speed_ctrl_tx"
#define CLASS_NAME      "sysprog_tx"
#define FRAME_SIZE      4

#define BCM_DATA_TX_OUT 4
#define BCM_CLK_TX_OUT  27
#define BCM_DATA_RX_IN  6
#define BCM_CLK_RX_IN   20
#define GPIOCHIP_BASE   512

#ifdef DEBUG
# define DBG(fmt, ...) printk(KERN_INFO "[TX_DRV][DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
# define DBG(fmt, ...)
#endif

static dev_t dev_num;
static struct cdev tx_cdev;
static struct class *tx_class;

static struct gpio_desc *data_out, *clk_out;
static struct gpio_desc *data_in,  *clk_in;

static unsigned char rx_frame[FRAME_SIZE];
static int bit_pos;

/* IRQ 핸들러: ACK 비트 수신 */
static irqreturn_t clk_rx_irq_handler(int irq, void *dev_id)
{
    if (bit_pos >= FRAME_SIZE * 8) {
        bit_pos = 0;
        return IRQ_HANDLED;
    }

    /* 데이터 비트 수집 */
    int bit = gpiod_get_value(data_in);
    int idx = bit_pos / 8;
    rx_frame[idx] = (rx_frame[idx] << 1) | (bit & 1);
    bit_pos++;

    /* 전체 프레임 수신 시 reset */
    if (bit_pos == FRAME_SIZE * 8) {
        /* ACK 수신 완료 */
        bit_pos = 0;
    }
    return IRQ_HANDLED;
}

/* 프레임 전송 (blocking OK) */
static void send_frame(const unsigned char *frame)
{
    msleep(10);
    for (int i = 0; i < FRAME_SIZE; i++) {
        for (int b = 7; b >= 0; b--) {
            int bit = (frame[i] >> b) & 1;
            gpiod_set_value(data_out, bit);
            udelay(10);
            gpiod_set_value(clk_out, 1);
            udelay(200);
            gpiod_set_value(clk_out, 0);
            udelay(10);
        }
    }
}

static ssize_t tx_write(struct file *filp, const char __user *buf,
                        size_t len, loff_t *off)
{
    unsigned char frame[FRAME_SIZE];
    if (len < FRAME_SIZE)
        return -EINVAL;
    if (copy_from_user(frame, buf, FRAME_SIZE))
        return -EFAULT;
    send_frame(frame);
    return FRAME_SIZE;
}

static ssize_t tx_read(struct file *filp, char __user *buf,
                       size_t len, loff_t *off)
{
    if (len < FRAME_SIZE)
        return -EINVAL;
    if (copy_to_user(buf, rx_frame, FRAME_SIZE))
        return -EFAULT;
    return FRAME_SIZE;
}

static const struct file_operations tx_fops = {
    .owner   = THIS_MODULE,
    .read    = tx_read,
    .write   = tx_write,
};

static int __init tx_init(void)
{
    int ret;

    /* 문자 디바이스 등록 */
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0)
        return ret;
    cdev_init(&tx_cdev, &tx_fops);
    tx_cdev.owner = THIS_MODULE;
    ret = cdev_add(&tx_cdev, dev_num, 1);
    if (ret)
        return ret;

    /* 클래스 및 디바이스 생성 */
    tx_class = class_create(CLASS_NAME);
    if (IS_ERR(tx_class))
        return PTR_ERR(tx_class);
    device_create(tx_class, NULL, dev_num, NULL, DEVICE_NAME);

    /* GPIO 설정 */
    data_out = gpio_to_desc(GPIOCHIP_BASE + BCM_DATA_TX_OUT);
    clk_out  = gpio_to_desc(GPIOCHIP_BASE + BCM_CLK_TX_OUT);
    data_in  = gpio_to_desc(GPIOCHIP_BASE + BCM_DATA_RX_IN);
    clk_in   = gpio_to_desc(GPIOCHIP_BASE + BCM_CLK_RX_IN);
    gpiod_direction_output(data_out, 0);
    gpiod_direction_output(clk_out, 0);
    gpiod_direction_input(data_in);
    gpiod_direction_input(clk_in);

    /* IRQ 등록 */
    irq_clk_rx = gpiod_to_irq(clk_in);
    ret = request_irq(irq_clk_rx, clk_rx_irq_handler,
                      IRQF_TRIGGER_RISING, "clk_rx_irq", NULL);
    if (ret) {
        pr_err("clk_rx_irq request failed: %d\n", ret);
        return ret;
    }

    DBG("TX driver initialized");
    return 0;
}

static void __exit tx_exit(void)
{
    free_irq(irq_clk_rx, NULL);
    device_destroy(tx_class, dev_num);
    class_destroy(tx_class);
    cdev_del(&tx_cdev);
    unregister_chrdev_region(dev_num, 1);
    DBG("TX driver exited");
}

module_init(tx_init);
module_exit(tx_exit);
MODULE_LICENSE("GPL");
