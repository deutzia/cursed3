#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/anon_inodes.h>
#include <linux/file.h>

#include "fdoomdev.h"
#include "fdoomfw.h"
#include "fharddoom.h"

#define FHARDDOOM_MAX_DEVICES 256
#define FHARDDOOM_MAX_BUFFER_SIZE (4 * 1024 * 1024)
#define FHARDDOOM_MAX_ADDITIONAL_BUFFERS 60

MODULE_LICENSE("GPL");

struct fharddoom_device {
	struct pci_dev *pdev;
	struct cdev cdev;
	int idx;
	struct device *dev;
	void __iomem *bar;
	spinlock_t slock;
	wait_queue_head_t wq;
	bool broken;
	struct file *currently_running;
	bool currently_used;
};

struct fharddoom_context {
	struct fharddoom_device *dev;
	bool broken;
	spinlock_t slock;
	wait_queue_head_t wq;
	bool running;
};

struct fharddoom_buffer {
	struct list_head lh;
	struct fharddoom_device *dev;
	struct fharddoom_context *ctx;
	dma_addr_t page_table_dma;
	uint32_t *page_table_cpu; /* just sth 4 byte */
	size_t page_count;
	struct list_head pages;
	uint32_t pitch;
};

struct fharddoom_buffer_page {
	struct list_head lh;
	void *cpu;
	dma_addr_t dma;
};

static dev_t fharddoom_devno;
static struct fharddoom_device *fharddoom_devices[FHARDDOOM_MAX_DEVICES];
static DEFINE_MUTEX(fharddoom_devices_lock);
static struct class fharddoom_class = {
	.name = "fharddoom",
	.owner = THIS_MODULE,
};

/* Hardware handling. */

static inline void fharddoom_iow(struct fharddoom_device *dev, uint32_t reg,
				 uint32_t val)
{
	iowrite32(val, dev->bar + reg);
}

static inline uint32_t fharddoom_ior(struct fharddoom_device *dev, uint32_t reg)
{
	uint32_t res = ioread32(dev->bar + reg);
	return res;
}

static irqreturn_t fharddoom_isr(int irq, void *opaque)
{
	struct fharddoom_device *dev = opaque;
	unsigned long flags;
	uint32_t istatus;
	spin_lock_irqsave(&dev->slock, flags);
	istatus = fharddoom_ior(dev, FHARDDOOM_INTR);
	if (istatus) {
		struct fharddoom_context *ctx;
		fharddoom_iow(dev, FHARDDOOM_INTR, istatus);
		BUG_ON(!(dev->currently_running));
		ctx = dev->currently_running->private_data;
		if (istatus != FHARDDOOM_INTR_FENCE_WAIT) {
			ctx->broken = dev->broken = 1;
		}
		ctx->running = 0;
		wake_up(&ctx->wq);
	}
	spin_unlock_irqrestore(&dev->slock, flags);
	return IRQ_RETVAL(istatus);
}

/* assumes spinlock is held or not needed */
static void fharddoom_wipe(struct fharddoom_device *dev)
{
	int i;
	fharddoom_iow(dev, FHARDDOOM_ENABLE, 0);
	fharddoom_iow(dev, FHARDDOOM_INTR_ENABLE, 0);
	fharddoom_iow(dev, FHARDDOOM_FE_CODE_ADDR, 0);
	for (i = 0; i < (sizeof fdoomfw / sizeof fdoomfw[0]); i++)
		fharddoom_iow(dev, FHARDDOOM_FE_CODE_WINDOW, fdoomfw[i]);
	fharddoom_iow(dev, FHARDDOOM_RESET, 0x7f7ff3ff);
	fharddoom_iow(dev, FHARDDOOM_INTR, 0xff0f);
	fharddoom_iow(dev, FHARDDOOM_INTR_ENABLE, FHARDDOOM_INTR_MASK);
	fharddoom_iow(dev, FHARDDOOM_ENABLE, FHARDDOOM_ENABLE_ALL);
	fharddoom_iow(dev, FHARDDOOM_CMD_FENCE_LAST, 0);
	fharddoom_iow(dev, FHARDDOOM_CMD_FENCE_WAIT, 0);
	dev->broken = 0;
}

/* Main device node handling.  */

static int fharddoom_open(struct inode *inode, struct file *file)
{
	struct fharddoom_device *dev =
		container_of(inode->i_cdev, struct fharddoom_device, cdev);
	struct fharddoom_context *ctx = kzalloc(sizeof *ctx, GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dev = dev;
	init_waitqueue_head(&ctx->wq);
	spin_lock_init(&ctx->slock);
	file->private_data = ctx;
	return nonseekable_open(inode, file);
}

static int fharddoom_release(struct inode *inode, struct file *file)
{
	struct fharddoom_context *ctx = file->private_data;
	struct fharddoom_device *dev = ctx->dev;
	unsigned long flags, flags2;
	spin_lock_irqsave(&ctx->slock, flags);
	spin_lock_irqsave(&dev->slock, flags2);
	if (ctx->running) {
		spin_unlock_irqrestore(&dev->slock, flags);
		spin_unlock_irqrestore(&ctx->slock, flags);
		wait_event(ctx->wq, !ctx->running);
		spin_lock_irqsave(&ctx->slock, flags);
		spin_lock_irqsave(&dev->slock, flags);
	}
	spin_unlock_irqrestore(&dev->slock, flags2);
	spin_unlock_irqrestore(&ctx->slock, flags);
	kfree(ctx);
	return 0;
}

vm_fault_t fharddoom_page_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct file *file = vma->vm_file;
	struct fharddoom_buffer *buf = file->private_data;
	uint32_t desired_page_index = vmf->pgoff;
	if (desired_page_index < buf->page_count) {
		struct fharddoom_buffer_page *page;
		uint32_t i = 0;
		list_for_each_entry (page, &buf->pages, lh) {
			if (desired_page_index == i) {
				struct page *real_page =
					virt_to_page(page->cpu);
				get_page(real_page);
				vmf->page = real_page;
				return 0;
			}
			i++;
		}
	}
	return VM_FAULT_SIGBUS;
}

static const struct vm_operations_struct fharddoom_page_ops = {
	.fault = fharddoom_page_fault,
};

static int fharddoom_buffer_mmap(struct file *filp, struct vm_area_struct *vma)
{
	vma->vm_ops = &fharddoom_page_ops;
	return 0;
}

static int fharddoom_buffer_release(struct inode *inode, struct file *file)
{
	struct fharddoom_buffer_page *page, *page2;
	struct fharddoom_buffer *buf = file->private_data;
	struct fharddoom_device *dev = buf->dev;
	list_for_each_entry_safe (page, page2, &buf->pages, lh) {
		dma_free_coherent(&dev->pdev->dev, FHARDDOOM_PAGE_SIZE,
				  page->cpu, page->dma);
		list_del(&page->lh);
		kfree(page);
	}
	dma_free_coherent(&dev->pdev->dev, FHARDDOOM_PAGE_SIZE,
			  buf->page_table_cpu, buf->page_table_dma);
	kfree(buf);
	return 0;
}

static long fharddoom_buffer_ioctl(struct file *flip, unsigned int cmd,
				   unsigned long arg)
{
	void __user *p = (void *)arg;
	switch (cmd) {
	case FDOOMDEV_IOCTL_BUFFER_RESIZE: {
		struct fdoomdev_ioctl_buffer_resize req;
		size_t page_count;
		uint32_t i;
		struct fharddoom_buffer_page *page, *page2;
		unsigned long irq;
		struct fharddoom_buffer *buf = flip->private_data;
		struct device *dev = &buf->dev->pdev->dev;
		if (copy_from_user(&req, p, sizeof(req))) {
			return -EFAULT;
		}
		if (req.size > FHARDDOOM_MAX_BUFFER_SIZE) {
			return -EINVAL;
		}
		page_count = (req.size + FHARDDOOM_PAGE_SIZE - 1) /
			     FHARDDOOM_PAGE_SIZE;

		if (page_count <= buf->page_count) {
			/* resizing no-op can be immediate */
			return 0;
		}

		spin_lock_irqsave(&buf->dev->slock, irq);
		/* need to make sure device is not using this buffer */
		if (buf->dev->currently_used) {
			spin_unlock_irqrestore(&buf->dev->slock, irq);
			if (wait_event_interruptible(
				    buf->dev->wq, !buf->dev->currently_used)) {
				return -ERESTARTSYS;
			}
			spin_lock_irqsave(&buf->dev->slock, irq);
		}
		buf->dev->currently_used = 1;
		spin_unlock_irqrestore(&buf->dev->slock, irq);
		page2 = container_of(buf->pages.prev,
				     struct fharddoom_buffer_page, lh);

		for (i = buf->page_count; i < page_count; ++i) {
			page = kzalloc(sizeof(struct fharddoom_buffer_page),
				       GFP_KERNEL);
			if (!page)
				goto err_page;
			page->cpu = dma_alloc_coherent(dev, FHARDDOOM_PAGE_SIZE,
						       &page->dma, GFP_KERNEL);
			if (!page->cpu) {
				kfree(page);
				goto err_page;
			}
			list_add_tail(&page->lh, &buf->pages);
			buf->page_table_cpu[i] =
				FHARDDOOM_PTE_PRESENT |
				((page->dma) >> FHARDDOOM_PTE_PA_SHIFT);
		}
		buf->page_count = page_count;
		spin_lock_irqsave(&buf->dev->slock, irq);
		buf->dev->currently_used = 0;
		wake_up(&buf->dev->wq);
		spin_unlock_irqrestore(&buf->dev->slock, irq);
		return 0;
	err_page:
		page = page2;
		list_for_each_entry_safe_from (page, page2, &buf->pages, lh) {
			dma_free_coherent(dev, FHARDDOOM_PAGE_SIZE, page->cpu,
					  page->dma);
			list_del(&page->lh);
			kfree(page);
		}

		spin_lock_irqsave(&buf->dev->slock, irq);
		buf->dev->currently_used = 0;
		wake_up(&buf->dev->wq);
		spin_unlock_irqrestore(&buf->dev->slock, irq);
		return -ENOMEM;
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations fharddoom_buffer_ops = {
	.owner = THIS_MODULE,
	.mmap = fharddoom_buffer_mmap,
	.release = fharddoom_buffer_release,
	.unlocked_ioctl = fharddoom_buffer_ioctl,
	.compat_ioctl = fharddoom_buffer_ioctl,
};

static long fharddoom_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	struct fharddoom_context *ctx = filp->private_data;
	struct device *dev = &ctx->dev->pdev->dev;
	unsigned long irq, irq2;
	void __user *p = (void *)arg;
	switch (cmd) {
	case FDOOMDEV_IOCTL_CREATE_BUFFER: {
		struct fdoomdev_ioctl_create_buffer req;
		struct fharddoom_buffer *buf;
		struct fharddoom_buffer_page *page, *page2;
		long err;
		uint32_t i;
		if (copy_from_user(&req, p, sizeof(req))) {
			return -EFAULT;
		}
		if (req.size == 0 || req.size > FHARDDOOM_MAX_BUFFER_SIZE) {
			return -EINVAL;
		}
		if ((req.pitch & 63) || req.pitch > FHARDDOOM_MAX_BUFFER_SIZE) {
			return -EINVAL;
		}
		err = -ENOMEM;
		buf = kzalloc(sizeof(struct fharddoom_buffer), GFP_KERNEL);
		if (!buf) {
			goto create_err;
		}
		INIT_LIST_HEAD(&buf->pages);
		buf->dev = ctx->dev;
		buf->page_count = (req.size + FHARDDOOM_PAGE_SIZE - 1) /
				  FHARDDOOM_PAGE_SIZE;
		buf->page_table_cpu =
			dma_alloc_coherent(dev, FHARDDOOM_PAGE_SIZE,
					   &buf->page_table_dma, GFP_KERNEL);
		if (!buf->page_table_cpu) {
			goto create_err_page_table;
		}

		for (i = 0; i < buf->page_count; i++) {
			page = kzalloc(sizeof(struct fharddoom_buffer_page),
				       GFP_KERNEL);
			if (!page) {
				goto create_err_buf;
			}
			page->cpu = dma_alloc_coherent(dev, FHARDDOOM_PAGE_SIZE,
						       &page->dma, GFP_KERNEL);
			if (!page->cpu) {
				kfree(page);
				goto create_err_buf;
			}
			list_add_tail(&page->lh, &buf->pages);
			buf->page_table_cpu[i] =
				FHARDDOOM_PTE_PRESENT |
				((page->dma) >> FHARDDOOM_PTE_PA_SHIFT);
		}
		/* make sure all unused entries are zeroed */
		for (; i < 1024; ++i) {
			buf->page_table_cpu[i] = 0;
		}
		buf->pitch = req.pitch;

		err = anon_inode_getfd("fharddoom buffer",
				       &fharddoom_buffer_ops, buf, O_RDWR);
		if (err < 0) {
			goto create_err_buf;
		}
		return err;

	create_err_buf:
		list_for_each_entry_safe (page, page2, &buf->pages, lh) {
			dma_free_coherent(dev, FHARDDOOM_PAGE_SIZE, page->cpu,
					  page->dma);
			list_del(&page->lh);
			kfree(page);
		}
		dma_free_coherent(dev, FHARDDOOM_PAGE_SIZE, buf->page_table_cpu,
				  buf->page_table_dma);
	create_err_page_table:
		kfree(buf);
	create_err:
		return err;
	}
	case FDOOMDEV_IOCTL_RUN: {
		struct fdoomdev_ioctl_run req;
		long err;
		uint32_t i, j;
		struct file *main_buf_file;
		struct file
			*additional_buf_files[FHARDDOOM_MAX_ADDITIONAL_BUFFERS];
		struct fharddoom_buffer *main_buf;
		struct fharddoom_buffer
			*additional_bufs[FHARDDOOM_MAX_ADDITIONAL_BUFFERS];
		if (copy_from_user(&req, p, sizeof(req))) {
			return -EFAULT;
		}
		if (req.cmd_size > FHARDDOOM_MAX_BUFFER_SIZE ||
		    (req.cmd_addr & 3) || (req.cmd_size & 3) ||
		    req.buffers_num > FHARDDOOM_MAX_ADDITIONAL_BUFFERS) {
			return -EINVAL;
		}
		err = -EINVAL;
		main_buf_file = fget(req.cmd_fd);
		if (!main_buf_file) {
			return -EINVAL;
		}
		main_buf = main_buf_file->private_data;
		if (main_buf_file->f_op != &fharddoom_buffer_ops ||
		    ctx->dev != main_buf->dev) {
			goto main_buf_err;
		}
		i = 0;
		while (i < req.buffers_num) {
			additional_buf_files[i] = fget(req.buffer_fd[i]);
			if (!additional_buf_files[i]) {
				goto additional_buffers_err;
			}
			additional_bufs[i] =
				additional_buf_files[i]->private_data;
			if (additional_buf_files[i]->f_op !=
				    &fharddoom_buffer_ops ||
			    additional_bufs[i]->dev != ctx->dev) {
				i++;
				goto additional_buffers_err;
			}
			i++;
		}
		spin_lock_irqsave(&ctx->slock, irq);
		if (ctx->broken) {
			err = -EIO;
			spin_unlock_irqrestore(&ctx->slock, irq);
			goto additional_buffers_err;
		}
		if (ctx->running) {
			spin_unlock_irqrestore(&ctx->slock, irq);
			if (wait_event_interruptible(ctx->wq, !ctx->running)) {
				err = -ERESTARTSYS;
				goto additional_buffers_err;
			}

			spin_lock_irqsave(&ctx->slock, irq);
		}
		ctx->running = 1;

		spin_lock_irqsave(&ctx->dev->slock, irq2);
		if (ctx->dev->currently_used) {
			spin_unlock_irqrestore(&ctx->dev->slock, irq2);
			spin_unlock_irqrestore(&ctx->slock, irq);
			if (wait_event_interruptible(
				    ctx->dev->wq, !ctx->dev->currently_used)) {
				err = -ERESTARTSYS;
				goto ctx_running_err;
			}
			spin_lock_irqsave(&ctx->slock, irq);
			spin_lock_irqsave(&ctx->dev->slock, irq2);
		}
		ctx->dev->currently_used = 1;
		if (ctx->dev->broken) {
			fharddoom_wipe(ctx->dev);
		}
		ctx->dev->currently_running = get_file(filp);

		/* clear all slots */
		fharddoom_iow(ctx->dev, FHARDDOOM_CMD_MANUAL_FEED,
			      FHARDDOOM_USER_CLEAR_SLOTS_HEADER);
		fharddoom_iow(ctx->dev, FHARDDOOM_CMD_MANUAL_FEED,
			      (uint32_t)-1);
		fharddoom_iow(ctx->dev, FHARDDOOM_CMD_MANUAL_FEED,
			      (uint32_t)-1);
		fharddoom_iow(ctx->dev, FHARDDOOM_CMD_MANUAL_FEED,
			      FHARDDOOM_USER_BIND_SLOT_HEADER(60,
							      main_buf->pitch));
		fharddoom_iow(ctx->dev, FHARDDOOM_CMD_MANUAL_FEED,
			      FHARDDOOM_USER_BIND_SLOT_DATA(
				      1, 0, 0, main_buf->page_table_dma));
		for (j = 0; j < req.buffers_num; ++j) {
			fharddoom_iow(ctx->dev, FHARDDOOM_CMD_MANUAL_FEED,
				      FHARDDOOM_USER_BIND_SLOT_HEADER(
					      j, additional_bufs[j]->pitch));
			fharddoom_iow(
				ctx->dev, FHARDDOOM_CMD_MANUAL_FEED,
				FHARDDOOM_USER_BIND_SLOT_DATA(
					1, 1, 1,
					additional_bufs[j]->page_table_dma));
		}
		/* call */
		fharddoom_iow(ctx->dev, FHARDDOOM_CMD_MANUAL_FEED,
			      FHARDDOOM_USER_CALL_HEADER(60, req.cmd_addr));
		fharddoom_iow(ctx->dev, FHARDDOOM_CMD_MANUAL_FEED,
			      req.cmd_size);
		/* fence */
		fharddoom_iow(ctx->dev, FHARDDOOM_CMD_MANUAL_FEED,
			      FHARDDOOM_USER_FENCE_HEADER(0));

		spin_unlock_irqrestore(&ctx->dev->slock, irq2);
		spin_unlock_irqrestore(&ctx->slock, irq);
		if (wait_event_interruptible(ctx->wq, !ctx->running)) {
			err = -ERESTARTSYS;
			goto currently_running_err;
		}

		spin_lock_irqsave(&ctx->dev->slock, irq2);
		fput(ctx->dev->currently_running);
		ctx->dev->currently_running = NULL;
		ctx->dev->currently_used = 0;
		wake_up(&ctx->dev->wq);
		spin_unlock_irqrestore(&ctx->dev->slock, irq2);

		for (i = 0; i < req.buffers_num; ++i) {
			fput(additional_buf_files[i]);
		}
		fput(main_buf_file);

		return 0;
	currently_running_err:
		spin_lock_irqsave(&ctx->dev->slock, irq2);
		fput(ctx->dev->currently_running);
		ctx->dev->currently_running = NULL;
		ctx->dev->currently_used = 0;
		wake_up(&ctx->dev->wq);
		spin_unlock_irqrestore(&ctx->dev->slock, irq2);
	ctx_running_err:
		spin_lock_irqsave(&ctx->slock, irq);
		ctx->running = 0;
		spin_unlock_irqrestore(&ctx->slock, irq);
	additional_buffers_err:
		for (j = 0; j < i; ++j) {
			fput(additional_buf_files[j]);
		}
	main_buf_err:
		fput(main_buf_file);
		return err;
	}
	case FDOOMDEV_IOCTL_WAIT: {
		struct fdoomdev_ioctl_wait req;
		if (copy_from_user(&req, p, sizeof(req))) {
			return -EFAULT;
		}
		spin_lock_irqsave(&ctx->slock, irq);
		if (ctx->broken) {
			spin_unlock_irqrestore(&ctx->slock, irq);
			return -EIO;
		}
		if (ctx->running) {
			spin_unlock_irqrestore(&ctx->slock, irq);
			if (wait_event_interruptible(ctx->wq, !ctx->running)) {
				return -ERESTARTSYS;
			}
		} else {
			spin_unlock_irqrestore(&ctx->slock, irq);
		}
		return 0;
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations fharddoom_file_ops = {
	.owner = THIS_MODULE,
	.open = fharddoom_open,
	.release = fharddoom_release,
	.unlocked_ioctl = fharddoom_ioctl,
	.compat_ioctl = fharddoom_ioctl,
};

static int fharddoom_probe(struct pci_dev *pdev,
			   const struct pci_device_id *pci_id)
{
	int err, i;

	/* Allocate our structure.  */
	struct fharddoom_device *dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		goto out_alloc;
	}

	pci_set_drvdata(pdev, dev);
	dev->pdev = pdev;

	/* Locks etc.  */
	spin_lock_init(&dev->slock);
	init_waitqueue_head(&dev->wq);

	/* Allocate a free index.  */
	mutex_lock(&fharddoom_devices_lock);
	for (i = 0; i < FHARDDOOM_MAX_DEVICES; i++)
		if (!fharddoom_devices[i])
			break;
	if (i == FHARDDOOM_MAX_DEVICES) {
		err = -ENOSPC;
		mutex_unlock(&fharddoom_devices_lock);
		goto out_slot;
	}
	fharddoom_devices[i] = dev;
	dev->idx = i;
	mutex_unlock(&fharddoom_devices_lock);

	/* Enable hardware resources.  */
	if ((err = pci_enable_device(pdev)))
		goto out_enable;

	if ((err = pci_set_dma_mask(pdev, DMA_BIT_MASK(40))))
		goto out_mask;
	if ((err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(40))))
		goto out_mask;
	pci_set_master(pdev);

	if ((err = pci_request_regions(pdev, "fharddoom")))
		goto out_regions;

	/* Map the BAR.  */
	if (!(dev->bar = pci_iomap(pdev, 0, 0))) {
		err = -ENOMEM;
		goto out_bar;
	}

	/* Connect the IRQ line.  */
	if ((err = request_irq(pdev->irq, fharddoom_isr, IRQF_SHARED,
			       "fharddoom", dev)))
		goto out_irq;

	fharddoom_wipe(dev);

	/* We're live.  Let's export the cdev.  */
	cdev_init(&dev->cdev, &fharddoom_file_ops);
	if ((err = cdev_add(&dev->cdev, fharddoom_devno + dev->idx, 1)))
		goto out_cdev;

	/* And register it in sysfs.  */
	dev->dev = device_create(&fharddoom_class, &dev->pdev->dev,
				 fharddoom_devno + dev->idx, dev, "fdoom%d",
				 dev->idx);
	if (IS_ERR(dev->dev)) {
		printk(KERN_ERR "fharddoom: failed to register subdevice\n");
		/* too bad. */
		dev->dev = 0;
		goto out_devdev;
	}

	return 0;

out_devdev:
	cdev_del(&dev->cdev);
out_cdev:
	fharddoom_iow(dev, FHARDDOOM_INTR_ENABLE, 0);
	free_irq(pdev->irq, dev);
out_irq:
	pci_iounmap(pdev, dev->bar);
out_bar:
	pci_release_regions(pdev);
out_regions:
out_mask:
	pci_disable_device(pdev);
out_enable:
	mutex_lock(&fharddoom_devices_lock);
	fharddoom_devices[dev->idx] = 0;
	mutex_unlock(&fharddoom_devices_lock);
out_slot:
	kfree(dev);
out_alloc:
	return err;
}

static void fharddoom_remove(struct pci_dev *pdev)
{
	struct fharddoom_device *dev = pci_get_drvdata(pdev);
	if (dev->dev) {
		device_destroy(&fharddoom_class, fharddoom_devno + dev->idx);
	}
	cdev_del(&dev->cdev);
	fharddoom_iow(dev, FHARDDOOM_INTR_ENABLE, 0);
	fharddoom_iow(dev, FHARDDOOM_ENABLE, 0);
	fharddoom_ior(dev, FHARDDOOM_STATUS);
	free_irq(pdev->irq, dev);
	pci_iounmap(pdev, dev->bar);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	mutex_lock(&fharddoom_devices_lock);
	fharddoom_devices[dev->idx] = 0;
	mutex_unlock(&fharddoom_devices_lock);
	kfree(dev);
}

static int fharddoom_suspend(struct pci_dev *pdev, pm_message_t state)
{
	unsigned long flags;
	struct fharddoom_device *dev = pci_get_drvdata(pdev);
	spin_lock_irqsave(&dev->slock, flags);
	if (dev->currently_used) {
		spin_unlock_irqrestore(&dev->slock, flags);
		wait_event(dev->wq, !dev->currently_used);
		spin_lock_irqsave(&dev->slock, flags);
	}
	fharddoom_iow(dev, FHARDDOOM_INTR_ENABLE, 0);
	fharddoom_iow(dev, FHARDDOOM_ENABLE, 0);
	fharddoom_ior(dev, FHARDDOOM_INTR);
	spin_unlock_irqrestore(&dev->slock, flags);
	return 0;
}

static int fharddoom_resume(struct pci_dev *pdev)
{
	unsigned long flags;
	struct fharddoom_device *dev = pci_get_drvdata(pdev);
	spin_lock_irqsave(&dev->slock, flags);
	fharddoom_wipe(dev);
	spin_unlock_irqrestore(&dev->slock, flags);
	return 0;
}

static struct pci_device_id fharddoom_pciids[] = {
	{ PCI_DEVICE(FHARDDOOM_VENDOR_ID, FHARDDOOM_DEVICE_ID) },
	{ 0 }
};

static struct pci_driver fharddoom_pci_driver = {
	.name = "fharddoom",
	.id_table = fharddoom_pciids,
	.probe = fharddoom_probe,
	.remove = fharddoom_remove,
	.suspend = fharddoom_suspend,
	.resume = fharddoom_resume,
};

/* Init & exit.  */

static int fharddoom_init(void)
{
	int err;
	if ((err = alloc_chrdev_region(&fharddoom_devno, 0,
				       FHARDDOOM_MAX_DEVICES, "fhardoom")))
		goto err_chrdev;
	if ((err = class_register(&fharddoom_class)))
		goto err_class;
	if ((err = pci_register_driver(&fharddoom_pci_driver)))
		goto err_pci;
	return 0;

err_pci:
	class_unregister(&fharddoom_class);
err_class:
	unregister_chrdev_region(fharddoom_devno, FHARDDOOM_MAX_DEVICES);
err_chrdev:
	return err;
}

static void fharddoom_exit(void)
{
	pci_unregister_driver(&fharddoom_pci_driver);
	class_unregister(&fharddoom_class);
	unregister_chrdev_region(fharddoom_devno, FHARDDOOM_MAX_DEVICES);
}

module_init(fharddoom_init);
module_exit(fharddoom_exit);
