/* rx_drv.c - RX 측 드라이버 with IRQ 디버깅 및 타이밍 보정 (deferred ACK via workqueue, open/close 기반 decay) */
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

#define DEVICE_NAME    "speed_ctrl_rx"
#define CLASS_NAME     "sysprog_rx"

#define BCM_DATA_TX_IN   17
#define BCM_CLK_TX_IN    22
#define BCM_DATA_RX_OUT   5
#define BCM_CLK_RX_OUT   21

#define GPIOCHIP_BASE 512
#define FRAME_SIZE    4

#ifdef DEBUG
# define DBG(fmt, ...) printk(KERN_INFO "[RX_DRV][DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
# define DBG(fmt, ...)
#endif

enum state_code { IDLE = 0x00, ACCEL = 0x01, DECEL = 0x02, LIMITED = 0x03 };
static enum state_code  current_state = IDLE;
static int               current_speed = 0;
static int               rx_active     = 0;

static struct class     *rx_class;
static struct gpio_desc *data_in, *clk_in;
static struct gpio_desc *data_out, *clk_out;
static dev_t             dev_num;
static struct cdev       rx_cdev;

static int                        irq_clk_tx = -1;
static unsigned char             rx_buf[FRAME_SIZE];
static int                        bit_pos   = 0;
static unsigned long             last_recv_time;

/* deferred ACK via workqueue */
static struct work_struct ack_work;
static unsigned char      ack_buf[FRAME_SIZE];

/* decay delayed work */
static struct delayed_work decay_work;

/* workqueue handler: runs in process context, safe to sleep */
static void ack_work_fn(struct work_struct *work)
{
    msleep(10);  // TX 준비 대기

    for (int i = 0; i < FRAME_SIZE; i++) {
        unsigned char ch = ack_buf[i];
        for (int b = 7; b >= 0; b--) {
            int bit = (ch >> b) & 1;
            gpiod_set_value_cansleep(data_out, bit);
            udelay(10);
            gpiod_set_value_cansleep(clk_out, 1);
            udelay(200);
            gpiod_set_value_cansleep(clk_out, 0);
            udelay(10);
        }
    }
    DBG("ACK sent (deferred): %02X %02X %02X %02X",
        ack_buf[0], ack_buf[1], ack_buf[2], ack_buf[3]);
}

/* frame 처리만, ACK 스케줄은 IRQ 핸들러에서 */
static bool handle_frame(unsigned char *frame)
{
    if ((frame[0] ^ frame[1] ^ frame[2]) != frame[3]) {
        DBG("Checksum error");
        return false;
    }

    last_recv_time = jiffies;

    switch (frame[1]) {
    case 0x01: /* ACCEL */
        if (current_speed < 100) {
            current_speed += frame[2];
            if (current_speed > 100)
                current_speed = 100;
        }
        current_state = (current_speed == 100 ? LIMITED : ACCEL);
        break;
    default:
        DBG("Unknown CMD: %02X", frame[1]);
        break;
    }
    DBG("FSM updated: speed=%d, state=%d",
        current_speed, current_state);
    return true;
}

static irqreturn_t clk_tx_irq_handler(int irq, void *dev_id)
{
    if (!rx_active)
        return IRQ_HANDLED;

    DBG("First CLK IRQ: triggered");
    int bit = gpiod_get_value(data_in);
    int byte_idx = bit_pos / 8;
    rx_buf[byte_idx] <<= 1;
    rx_buf[byte_idx] |= (bit & 1);
    bit_pos++;

    if (bit_pos == FRAME_SIZE * 8) {
        DBG("CMD received: %02X %02X %02X %02X",
            rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
        if (handle_frame(rx_buf)) {
            ack_buf[0] = 0x55;
            ack_buf[1] = (unsigned char)current_speed;
            ack_buf[2] = (unsigned char)current_state;
            ack_buf[3] = ack_buf[0] ^ ack_buf[1] ^ ack_buf[2];
            schedule_work(&ack_work);
        }
        bit_pos = 0;
    }
    return IRQ_HANDLED;
}

/* decay: open 상태에서만 동작, 속도 감소 후 ACK (IDLE 제외) */
static void decay_function(struct work_struct *work)
{
    if (!rx_active)
        return;  // release 시 빠져나가 다음 스케줄 중지

    if (time_after(jiffies, last_recv_time + 2 * HZ) && current_speed > 0) {
        current_speed -= 10;
        if (current_speed < 0){
            current_speed = 0;
            return;
        }else if(current_speed==0){
            current_state=IDLE;
            ack_buf[0] = 0x55;
            ack_buf[1] = (unsigned char)current_speed;
            ack_buf[2] = (unsigned char)current_state;
            ack_buf[3] = ack_buf[0] ^ ack_buf[1] ^ ack_buf[2];
            schedule_work(&ack_work);
            return;
        }

        if (current_state != IDLE) {
            current_state=DECEL;
            ack_buf[0] = 0x55;
            ack_buf[1] = (unsigned char)current_speed;
            ack_buf[2] = (unsigned char)current_state;
            ack_buf[3] = ack_buf[0] ^ ack_buf[1] ^ ack_buf[2];
            schedule_work(&ack_work);
        }
    }

    /* 다음 decay 예약 (1초 뒤) */
    schedule_delayed_work(&decay_work, HZ);
}

static ssize_t rx_read(struct file *filp, char __user *buf,
                       size_t len, loff_t *off)
{
    char msg[64];
    int n = snprintf(msg, sizeof(msg),
                     "Speed: %d, State: %d\n",
                     current_speed, current_state);
    return simple_read_from_buffer(buf, len, off, msg, n);
}

static int rx_open(struct inode *inode, struct file *filp)
{
    rx_active = 1;
    DBG("RX opened, FSM activated");
    /* decay 루프 시작 */
    schedule_delayed_work(&decay_work, HZ);
    return 0;
}

static int rx_release(struct inode *inode, struct file *filp)
{
    rx_active = 0;
    DBG("RX released, FSM deactivated");
    /* decay 루프 중지 */
    cancel_delayed_work_sync(&decay_work);
    return 0;
}

static const struct file_operations rx_fops = {
    .owner   = THIS_MODULE,
    .open    = rx_open,
    .release = rx_release,
    .read    = rx_read,
};

static int __init rx_init(void)
{
    int ret;

    /* char device setup */
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) return ret;
    cdev_init(&rx_cdev, &rx_fops);
    rx_cdev.owner = THIS_MODULE;
    ret = cdev_add(&rx_cdev, dev_num, 1);
    if (ret) return ret;

    /* class/device */
    rx_class = class_create(CLASS_NAME);
    if (IS_ERR(rx_class)) return PTR_ERR(rx_class);
    device_create(rx_class, NULL, dev_num, NULL, DEVICE_NAME);

    /* GPIO setup */
    data_in  = gpio_to_desc(GPIOCHIP_BASE + BCM_DATA_TX_IN);
    clk_in   = gpio_to_desc(GPIOCHIP_BASE + BCM_CLK_TX_IN);
    data_out = gpio_to_desc(GPIOCHIP_BASE + BCM_DATA_RX_OUT);
    clk_out  = gpio_to_desc(GPIOCHIP_BASE + BCM_CLK_RX_OUT);
    gpiod_direction_input(data_in);
    gpiod_direction_input(clk_in);
    gpiod_direction_output(data_out, 0);
    gpiod_direction_output(clk_out, 0);

    /* IRQ */
    irq_clk_tx = gpiod_to_irq(clk_in);
    ret = request_irq(irq_clk_tx, clk_tx_irq_handler,
                      IRQF_TRIGGER_RISING, "clk_tx_irq", NULL);
    if (ret) {
        pr_err("[RX_DRV][ERROR] IRQ request failed: %d\n", ret);
        return ret;
    }

    /* init workqueue for ACK */
    INIT_WORK(&ack_work, ack_work_fn);

    /* init delayed work (but don't schedule here) */
    INIT_DELAYED_WORK(&decay_work, decay_function);

    last_recv_time = jiffies;
    DBG("RX driver initialized");
    return 0;
}

static void __exit rx_exit(void)
{
    /* cancel any pending work */
    cancel_work_sync(&ack_work);
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
