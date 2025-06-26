/* rx_drv.c - RX 측 드라이버 with IRQ 디버깅 및 타이밍 보정 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/delay.h>

#define DEVICE_NAME "speed_ctrl_rx"
#define CLASS_NAME  "sysprog_rx"

#define BCM_DATA_TX_IN   17
#define BCM_CLK_TX_IN    22
#define BCM_DATA_RX_OUT  5
#define BCM_CLK_RX_OUT   21

#define GPIOCHIP_BASE 512
#define FRAME_SIZE 4

#define DEBUG
#ifdef DEBUG
#define DBG(fmt, ...) printk(KERN_INFO "[RX_DRV][DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

enum state_code { IDLE = 0x00, ACCEL = 0x01, DECEL = 0x02, LIMITED = 0x03 };
static enum state_code current_state = IDLE;
static int current_speed = 0;
static int rx_active = 0;

static struct class *rx_class;
static struct gpio_desc *data_in, *clk_in;
static struct gpio_desc *data_out, *clk_out;
static int major;
static dev_t dev_num;
static struct cdev rx_cdev;

static int irq_clk_tx = -1;
static unsigned char rx_buf[FRAME_SIZE];
static int bit_pos = 0;
static unsigned long last_recv_time;

static struct delayed_work decay_work;

static void send_ack(void) {
    unsigned char ack[4];

    ack[0] = 0x55;
    ack[1] = (unsigned char)current_speed;
    ack[2] = (unsigned char)current_state;
    ack[3] = ack[0] ^ ack[1] ^ ack[2];

    for (int i = 0; i < 4; i++) {
        unsigned char ch = ack[i];
        for (int b = 7; b >= 0; b--) {
            int bit = (ch >> b) & 1;
            gpiod_set_value(data_out, bit);
            udelay(10);             // 데이터 세팅 안정화
            gpiod_set_value(clk_out, 1);
            udelay(200);            // 클럭 HIGH 유지 시간
            gpiod_set_value(clk_out, 0);
            udelay(10);             // 클럭 LOW 유지 시간
            DBG("asdfkjasdklfj CLK IRQ: asdf");

        }
    }
    DBG("ACK sent: %02X %02X %02X %02X", ack[0], ack[1], ack[2], ack[3]);
}

static void handle_frame(unsigned char *frame) {
    if ((frame[0] ^ frame[1] ^ frame[2]) != frame[3]) {
        DBG("Checksum error");
        return;
    }

    int cmd = frame[1];
    int value = frame[2];
    last_recv_time = jiffies;

    switch (cmd) {
        case 0x01: // ACCEL
            current_speed += value;
            current_state = (current_speed > 100) ? LIMITED : ACCEL;
            break;
        default:
            DBG("Unknown CMD: %02X", cmd);
            break;
    }

    DBG("FSM updated: speed=%d, state=%d", current_speed, current_state);
    send_ack();
}

static irqreturn_t clk_tx_irq_handler(int irq, void *dev_id) {
    if (!rx_active) return IRQ_HANDLED;

    DBG("First CLK IRQ: triggered");

    int bit = gpiod_get_value(data_in);
    int byte_idx = bit_pos / 8;
    rx_buf[byte_idx] <<= 1;
    rx_buf[byte_idx] |= (bit & 1);
    bit_pos++;

    if (bit_pos == 32) {
        DBG("CMD received: %02X %02X %02X %02X", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
        handle_frame(rx_buf);
        bit_pos = 0;
    }
    return IRQ_HANDLED;
}

static void decay_function(struct work_struct *work) {
    if (!rx_active) return;

    if (time_after(jiffies, last_recv_time + 2 * HZ)) {
        if (current_speed > 0) {
            current_speed -= 10;
            current_state = (current_speed == 0) ? IDLE : DECEL;
            DBG("Speed decayed: %d, state=%d", current_speed, current_state);
            send_ack();
        }
    }
    schedule_delayed_work(&decay_work, HZ);
}

static ssize_t rx_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    char msg[64];
    int n = snprintf(msg, sizeof(msg), "Speed: %d, State: %d\n", current_speed, current_state);
    return simple_read_from_buffer(buf, len, off, msg, n);
}

static int rx_open(struct inode *inode, struct file *filp) {
    rx_active = 1;
    DBG("RX opened, FSM activated");
    return 0;
}

static int rx_release(struct inode *inode, struct file *filp) {
    rx_active = 0;
    DBG("RX released, FSM deactivated");
    return 0;
}

static struct file_operations rx_fops = {
    .owner = THIS_MODULE,
    .open = rx_open,
    .release = rx_release,
    .read = rx_read,
};

static int __init rx_init(void) {
    int ret;
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) return ret;
    major = MAJOR(dev_num);

    cdev_init(&rx_cdev, &rx_fops);
    rx_cdev.owner = THIS_MODULE;
    ret = cdev_add(&rx_cdev, dev_num, 1);
    if (ret) return ret;

    rx_class = class_create(CLASS_NAME);
    device_create(rx_class, NULL, dev_num, NULL, DEVICE_NAME);

    data_in  = gpio_to_desc(GPIOCHIP_BASE + BCM_DATA_TX_IN);
    clk_in   = gpio_to_desc(GPIOCHIP_BASE + BCM_CLK_TX_IN);
    data_out = gpio_to_desc(GPIOCHIP_BASE + BCM_DATA_RX_OUT);
    clk_out  = gpio_to_desc(GPIOCHIP_BASE + BCM_CLK_RX_OUT);

    gpiod_direction_input(data_in);
    gpiod_direction_input(clk_in);
    gpiod_direction_output(data_out, 0);
    gpiod_direction_output(clk_out, 0);

    irq_clk_tx = gpiod_to_irq(clk_in);
    ret = request_irq(irq_clk_tx, clk_tx_irq_handler, IRQF_TRIGGER_RISING, "clk_tx_irq", NULL);
    if (ret) {
        pr_err("[RX_DRV][ERROR] IRQ request failed: %d\n", ret);
        return ret;
    }

    INIT_DELAYED_WORK(&decay_work, decay_function);
    schedule_delayed_work(&decay_work, HZ);
    last_recv_time = jiffies;

    DBG("RX driver initialized");
    return 0;
}

static void __exit rx_exit(void) {
    cancel_delayed_work_sync(&decay_work);
    free_irq(irq_clk_tx, NULL);
    device_destroy(rx_class, dev_num);
    class_destroy(rx_class);
    cdev_del(&rx_cdev);
    unregister_chrdev_region(dev_num, 1);
    DBG("RX driver exited");
}

module_init(rx_init);
module_exit(rx_exit);
MODULE_LICENSE("GPL");
