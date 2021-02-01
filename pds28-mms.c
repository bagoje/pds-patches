/* pds28-mms.c */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/poll.h>

#define DEVICE_FILE_NAME	"pds28mms"
#define DRIVER_NAME		"pds28mmsdrv"

#define MMS_CTRL_OFFSET		(0x00)
#define MMS_STATUS_OFFSET	(0x04)
#define MMS_DATA_OFFSET		(0x08)

#define CTRL_EN_MASK		(0x10000000)
#define CTRL_IEN_MASK		(0x20000000)
#define CTRL_RES_MASK		(0x40000000)
#define CTRL_FREQ_MASK		(0x80000000)

#define STATUS_IFG_MASK		(0x20000000)
#define DATA_MASK		(0x0000FFFF)

/**
 * struct pds28_mms - pds28 mms device private data structure
 * @base_addr:	base address of the device
 * @irq:	interrupt for the device
 * @dev:	struct device pointer
 * @cdev:	struct cdev
 * @devt:	dev_t member
 * @alarm:	set if alarm is active
 */
struct pds28_mms {
	void __iomem *base_addr;
	int irq;
	struct device *dev;
	struct device *parent;
	struct cdev cdev;
	dev_t devt;
	int alarm;
};

/* global so it can be destroyed when module is removed */
static struct class* pds28_mms_class;

/* declare wait queue */
static DECLARE_WAIT_QUEUE_HEAD(read_wq);

/* convert 11-bit pds28_DEMO register value to 1/10th of a Celsius */
static inline int pds28_mms_reg_to_mC(u16 val)
{
	return val * 10 / 16;
}

/* convert 1/10th of a Celsius to 11-bit ETF_DEMO register value */
static inline u16 pds28_mms_mC_to_reg(int val)
{
	return (val * 16) / 10;
}

static int pds28_mmsdev_open(struct inode *inode, struct file *filp)
{
	struct pds28_mms *mmsdev;

	/* store mmsdev pointer for read */
	mmsdev = container_of(inode->i_cdev, struct pds28_mms, cdev);
	filp->private_data = mmsdev;

	return 0;
}

static int pds28_mmsdev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t pds28_mmsdev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct pds28_mms *mmsdev;
	/* buffer */
	static char buffer[10];
	/* Number of bytes written to the buffer */
	ssize_t bytes_read = 0;
	/* data storage */
	int data_reg;

	if (*f_pos) {
		*f_pos = 0;
		return 0;
	}

	mmsdev = filp->private_data;

	/* read data from DATA register */
	data_reg = ioread32(mmsdev->base_addr+MMS_DATA_OFFSET);

	/* pack it into buffer */
	sprintf(buffer, "%d\n", data_reg);
	bytes_read = strlen(buffer);

	/* copy_to_user */
	if (copy_to_user(buf, buffer, bytes_read)) {
		return -EFAULT;
	}
	*f_pos += bytes_read;

	/* return number of bytes read */
	return bytes_read;
}

static ssize_t pds28_mmsdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	pr_alert("not implemented\n");
	return -EINVAL;
}


static unsigned int pds28_mmsdev_poll(struct file *filp, poll_table *wait) 
{
	struct pds28_mms *mmsdev;
	unsigned int retval_mask = 0;
	mmsdev = filp->private_data;
	poll_wait(filp, &read_wq, wait);
	if (mmsdev->alarm) {
		retval_mask = (POLLPRI | POLLRDNORM);
		mmsdev->alarm = 0;
	} else {
		retval_mask = 0;
	}
	return retval_mask;
}


static struct file_operations pds28_mmsdev_fops = {
	.owner = THIS_MODULE,
	.open = pds28_mmsdev_open,
	.release = pds28_mmsdev_release,
	.read = pds28_mmsdev_read,
	.write = pds28_mmsdev_write,
	.poll = pds28_mmsdev_poll,
};

/*static ssize_t ien_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct pds28_mms *mmsdev = dev_get_drvdata(child);

	u32 alarm = ioread32(mmsdev->base_addr + MMS_STATUS_OFFSET);
	alarm &= STATUS_ALARM_MASK;

	return sprintf(buf, "%d\n", !!alarm);
}
static DEVICE_ATTR_RO(alarm);*/

static ssize_t ctrl_ien_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct pds28_mms *mmsdev = dev_get_drvdata(child);

	u32 enable_alarm = ioread32(mmsdev->base_addr + MMS_CTRL_OFFSET);
	enable_alarm &= CTRL_IEN_MASK;

	return sprintf(buf, "%d\n", !!enable_alarm);
}

static ssize_t ctrl_ien_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pds28_mms *mmsdev = dev_get_drvdata(child);
	u32 ctrl = ioread32(mmsdev->base_addr + MMS_CTRL_OFFSET);
	int enable_alarm;

	sscanf(buf, "%d", &enable_alarm);
	if (!enable_alarm) {
		ctrl &= ~CTRL_IEN_MASK;
	} else {
		ctrl |= CTRL_IEN_MASK;
	}
	iowrite32(ctrl, mmsdev->base_addr + MMS_CTRL_OFFSET);
	return count;
}
static DEVICE_ATTR_RW(ctrl_ien);

static ssize_t ctrl_en_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct pds28_mms *mmsdev = dev_get_drvdata(child);
	u32 enable = ioread32(mmsdev->base_addr + MMS_CTRL_OFFSET);

	enable &= CTRL_EN_MASK;
	return sprintf(buf, "%d\n", !!enable);
}

static ssize_t ctrl_en_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pds28_mms *mmsdev = dev_get_drvdata(child);
	u32 ctrl = ioread32(mmsdev->base_addr + MMS_CTRL_OFFSET);
	int enable;

	sscanf(buf, "%d", &enable);
	if (!enable) {
		ctrl &= ~CTRL_EN_MASK;
	} else {
		ctrl |= CTRL_EN_MASK;
	}
	iowrite32(ctrl, mmsdev->base_addr + MMS_CTRL_OFFSET);
	return count;
}
static DEVICE_ATTR_RW(ctrl_en);

static ssize_t ctrl_res_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct pds28_mms *mmsdev = dev_get_drvdata(child);
	u32 ctrl = ioread32(mmsdev->base_addr + MMS_CTRL_OFFSET);

	if (ctrl & CTRL_RES_MASK) {
		return sprintf(buf, "coarse\n");
	} else {
		return sprintf(buf, "fine\n");
	}
}

static ssize_t ctrl_res_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pds28_mms *mmsdev = dev_get_drvdata(child);
	int ret;
        char resolution[16];
	u32 ctrl = ioread32(mmsdev->base_addr + MMS_CTRL_OFFSET);
	
	ret = sscanf(buf, "%15s", resolution);
	if (ret != 1)
                return -EINVAL;
	if (!strncasecmp(resolution, "fine", 5)) 
		ctrl &= ~CTRL_RES_MASK;
	else
		if (!strncasecmp(resolution, "coarse", 7))
			ctrl |= CTRL_RES_MASK;
	iowrite32(ctrl, mmsdev->base_addr + MMS_CTRL_OFFSET);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(ctrl_res);

static ssize_t available_res_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "fine coarse\n");
}
static DEVICE_ATTR_RO(available_res);

static ssize_t ctrl_freq_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct pds28_mms *mmsdev = dev_get_drvdata(child);
	u32 ctrl = ioread32(mmsdev->base_addr + MMS_CTRL_OFFSET);

	if (ctrl & CTRL_FREQ_MASK) {
		return sprintf(buf, "fast\n");
	} else {
		return sprintf(buf, "normal\n");
	}
}

static ssize_t ctrl_freq_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pds28_mms *mmsdev = dev_get_drvdata(child);
	int ret;
        char freq[16];
	u32 ctrl = ioread32(mmsdev->base_addr + MMS_CTRL_OFFSET);
	
	ret = sscanf(buf, "%15s", freq);
	if (ret != 1)
                return -EINVAL;
	if (!strncasecmp(freq, "normal", 7)) 
		ctrl &= ~CTRL_FREQ_MASK;
	else
		if (!strncasecmp(freq, "fast", 5))
			ctrl |= CTRL_FREQ_MASK;
	iowrite32(ctrl, mmsdev->base_addr + MMS_CTRL_OFFSET);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(ctrl_freq);

static ssize_t available_freq_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "normal fast\n");
}
static DEVICE_ATTR_RO(available_freq);
/*
static ssize_t data_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct pds28_mms *mmsdev = dev_get_drvdata(child);

	u32 data = ioread32(mmsdev->base_addr + MMS_DATA_OFFSET);
	data &= DATA_MASK;

	return sprintf(buf, "%d\n", pds28_mms_reg_to_mC(data));
}
static DEVICE_ATTR_RO(data);
*/
static struct attribute *pds28_mms_attrs[] = {
	//&dev_attr_alarm.attr,
	&dev_attr_ctrl_ien.attr,
	&dev_attr_ctrl_en.attr,
	&dev_attr_ctrl_res.attr,
	&dev_attr_ctrl_freq.attr,
	&dev_attr_available_res.attr,
	&dev_attr_available_freq.attr,
	//&dev_attr_data.attr,
	NULL,
};

ATTRIBUTE_GROUPS(pds28_mms);

static irqreturn_t pds28_mms_isr(int irq, void *data)
{
	struct pds28_mms *mmsdev = data;

	sysfs_notify(&mmsdev->dev->kobj, NULL, "data");
	wake_up_interruptible(&read_wq);
	mmsdev->alarm = 1;
	iowrite32(0, mmsdev->base_addr + MMS_STATUS_OFFSET);

	return IRQ_HANDLED;
}

static const struct of_device_id pds28_mms_of_match[] = {
	{ .compatible = "arm,pds28-mms", },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, pds28_mms_of_match);

static int pds28_mmsdev_setup(struct device *parent)
{
	int ret;
	dev_t devt;
	struct pds28_mms *mmsdev;

	mmsdev = dev_get_drvdata(parent);

	ret = alloc_chrdev_region(&devt, 0, 1, DEVICE_FILE_NAME);
	if (ret < 0) {
		dev_err(parent, "failed to alloc chrdev region\n");
		goto fail_alloc_chrdev_region;
	}
	mmsdev->devt = devt;

	cdev_init(&mmsdev->cdev, &pds28_mmsdev_fops);
	ret = cdev_add(&mmsdev->cdev, devt, 1);
	if (ret < 0) {
		dev_err(parent, "failed to add cdev\n");
		goto fail_add_cdev;
	}

	pds28_mms_class = class_create(THIS_MODULE, "pds28");
	if (!pds28_mms_class) {
		ret = -EEXIST;
		dev_err(parent, "failed to create class\n");
		goto fail_create_class;
	}

	mmsdev->dev = device_create_with_groups(pds28_mms_class, parent, devt, mmsdev, pds28_mms_groups, "%s%d", DEVICE_FILE_NAME, MINOR(devt));
	if (IS_ERR(mmsdev->dev)) {
		mmsdev->dev = NULL;
		ret = -EINVAL;
		dev_err(parent, "failed to create device\n");
		goto fail_create_device;
	}

	return 0;

fail_create_device:
	class_destroy(pds28_mms_class);
fail_create_class:
	cdev_del(&mmsdev->cdev);
fail_add_cdev:
	unregister_chrdev_region(devt, 1);
fail_alloc_chrdev_region:
	return ret;
}

static int pds28_mms_probe(struct platform_device *pdev)
{
	int ret;
	struct pds28_mms *mmsdev;
	struct resource *res;
	const struct of_device_id *match;

	mmsdev = devm_kzalloc(&pdev->dev, sizeof(*mmsdev), GFP_KERNEL);
	if (!mmsdev)
		return -ENOMEM;

	mmsdev->parent = &pdev->dev;

	match = of_match_node(pds28_mms_of_match, pdev->dev.of_node);
	if (!match) {
		dev_err(&pdev->dev, "of_match_node() failed\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, mmsdev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmsdev->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mmsdev->base_addr))
		return PTR_ERR(mmsdev->base_addr);

	mmsdev->irq = platform_get_irq(pdev, 0);
	if (mmsdev->irq < 0) {
		dev_err(&pdev->dev, "invalid IRQ\n");
		return mmsdev->irq;
	}
	ret = devm_request_irq(&pdev->dev, mmsdev->irq, pds28_mms_isr, 0, dev_name(&pdev->dev), mmsdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to request IRQ\n");
		return ret;
	}

	/* initialize device */
	ret = pds28_mmsdev_setup(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create device\n");
		return ret;
	}

	return 0;
}

static int pds28_mms_remove(struct platform_device *pdev)
{
	struct pds28_mms *mmsdev;

	mmsdev = dev_get_drvdata(&pdev->dev);

	device_destroy(pds28_mms_class, mmsdev->devt);
	class_destroy(pds28_mms_class);
	cdev_del(&mmsdev->cdev);
	unregister_chrdev_region(mmsdev->devt, 1);

	return 0;
}

static struct platform_driver pds28_mms_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = pds28_mms_of_match,
	},
	.probe = pds28_mms_probe,
	.remove = pds28_mms_remove,
};

static int __init pds28_mms_init(void)
{
	pr_alert("Hello, world!\n");
	return platform_driver_register(&pds28_mms_driver);
}

static void __exit pds28_mms_exit(void)
{
	pr_alert("Goodbye, world!\n");
	return platform_driver_unregister(&pds28_mms_driver);
}

module_init(pds28_mms_init);
module_exit(pds28_mms_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ETF Demo Driver and Device");
MODULE_AUTHOR("Strahinja Jankovic (ETF Belgrade)");
