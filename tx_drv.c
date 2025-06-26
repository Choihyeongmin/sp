/* tx_drv.c - TX 측 드라이버 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/signal.h>
#include <linux/poll.h>
#include <linux/delay.h>   // for udelay
#include <linux/workqueue.h>

#define DEVICE_NAME "speed_ctrl_tx"
#define CLASS_NAME  "sysprog_tx"

#define BCM_DATA_TX_OUT 4
#define BCM_CLK_TX_OUT  27
#define BCM_DATA_RX_IN  6
#define BCM_CLK_RX_IN   19
#define BCM_BUTTON      20

#define GPIOCHIP_BASE 512

#define DEBUG
#ifdef DEBUG
#define DBG(fmt, ...) printk(KERN_INFO "[TX_DRV][DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

static struct class *tx_class;
static struct gpio_desc *data_out, *clk_out;
static struct gpio_desc *data_in, *clk_in;
static struct gpio_desc *btn_in;
static int major;
static dev_t dev_num;
static struct cdev tx_cdev;

// ACK 수신용 인터럽트
static int irq_clk_rx = -1;
static struct fasync_struct *async_queue;

// 버튼 상태 확인용
static struct delayed_work button_poll_work;
static int last_btn = 0;

// 수신 버퍼
#define FRAME_SIZE 4
static unsigned char rx_frame[FRAME_SIZE];
static int bit_pos = 0;

static irqreturn_t clk_rx_irq_handler(int irq, void *dev_id) {
    int bit = gpiod_get_value(data_in);
    int byte_idx = bit_pos / 8;
    rx_frame[byte_idx] <<= 1;
    rx_frame[byte_idx] |= (bit & 0x1);
    bit_pos++;

    if (bit_pos == 32) {
        DBG("ACK received: %02X %02X %02X %02X", rx_frame[0], rx_frame[1], rx_frame[2], rx_frame[3]);
        if (async_queue)
            kill_fasync(&async_queue, SIGIO, POLL_IN);
        bit_pos = 0;
    }
    return IRQ_HANDLED;
}

static void send_frame(unsigned char *frame) {
    DBG("TX sending frame: %02X %02X %02X %02X", frame[0], frame[1], frame[2], frame[3]);
    for (int i = 0; i < FRAME_SIZE; i++) {
        unsigned char ch = frame[i];
        for (int b = 7; b >= 0; b--) {
            int bit = (ch >> b) & 1;
            gpiod_set_value(data_out, bit);
            udelay(10);
            gpiod_set_value(clk_out, 1);
            udelay(100);
            gpiod_set_value(clk_out, 0);
        }
    }
}

static void button_poll(struct work_struct *work) {
    int val = gpiod_get_value(btn_in);
    if (val == 1 && last_btn == 0) { // rising edge
        unsigned char frame[4] = {0xAA, 0x01, 10, 0};
        frame[3] = frame[0] ^ frame[1] ^ frame[2];
        send_frame(frame);
    }
    last_btn = val;
    schedule_delayed_work(&button_poll_work, msecs_to_jiffies(100));
}

static ssize_t tx_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    if (len < FRAME_SIZE)
        return -EINVAL;
    if (copy_to_user(buf, rx_frame, FRAME_SIZE))
        return -EFAULT;
    return FRAME_SIZE;
}

static int tx_open(struct inode *inode, struct file *filp) {
    return 0;
}

static int tx_release(struct inode *inode, struct file *filp) {
    return 0;
}

static int tx_fasync(int fd, struct file *filp, int mode) {
    return fasync_helper(fd, filp, mode, &async_queue);
}

static struct file_operations tx_fops = {
    .owner = THIS_MODULE,
    .read = tx_read,
    .open = tx_open,
    .release = tx_release,
    .fasync = tx_fasync,
};

static int __init tx_init(void) {
    int ret;
    
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) return ret;
    major = MAJOR(dev_num);

    cdev_init(&tx_cdev, &tx_fops);
    tx_cdev.owner = THIS_MODULE;
    ret = cdev_add(&tx_cdev, dev_num, 1);
    if (ret) return ret;

    tx_class = class_create(CLASS_NAME);
    device_create(tx_class, NULL, dev_num, NULL, DEVICE_NAME);

    data_out = gpio_to_desc(GPIOCHIP_BASE + BCM_DATA_TX_OUT);
    clk_out  = gpio_to_desc(GPIOCHIP_BASE + BCM_CLK_TX_OUT);
    data_in  = gpio_to_desc(GPIOCHIP_BASE + BCM_DATA_RX_IN);
    clk_in   = gpio_to_desc(GPIOCHIP_BASE + BCM_CLK_RX_IN);
    btn_in   = gpio_to_desc(GPIOCHIP_BASE + BCM_BUTTON);

    gpiod_direction_output(data_out, 0);
    gpiod_direction_output(clk_out, 0);
    gpiod_direction_input(data_in);
    gpiod_direction_input(clk_in);
    gpiod_direction_input(btn_in);

    irq_clk_rx = gpiod_to_irq(clk_in);
    request_irq(irq_clk_rx, clk_rx_irq_handler, IRQF_TRIGGER_RISING, "clk_rx_irq", NULL);

    INIT_DELAYED_WORK(&button_poll_work, button_poll);
    schedule_delayed_work(&button_poll_work, msecs_to_jiffies(100));

    DBG("TX driver initialized");
    return 0;
}

static void __exit tx_exit(void) {
    cancel_delayed_work_sync(&button_poll_work);
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