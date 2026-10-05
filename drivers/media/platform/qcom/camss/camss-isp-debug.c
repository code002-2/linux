// SPDX-License-Identifier: GPL-2.0
/*
 * camss-isp-debug.c
 *
 * Bring-up aid for the SM8750 TFE pixel path: register scripts applied when
 * the pixel path starts, and live register access while it streams.
 *
 * debugfs (camss_isp/):
 *   vfe_script, csid_script  write "OFFSET VALUE" lines (hex) to append,
 *                            "clear" to empty; read to list
 *   vfe_live, csid_live      write "OFFSET VALUE" to write now,
 *                            "r OFFSET COUNT" to read now; read to show
 *                            the last read
 *
 * Live access only touches a block between its pixel path start and stop,
 * i.e. while its clocks and power domains are known to be on.
 */
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "camss-isp-debug.h"

#define ISP_SCRIPT_MAX		2048
#define ISP_PEEK_MAX		256

struct isp_block {
	const char *name;
	u32 off[ISP_SCRIPT_MAX];
	u32 val[ISP_SCRIPT_MAX];
	unsigned int n;
	void __iomem *base;		/* set while live */
	u32 peek_off;
	u32 peek_val[ISP_PEEK_MAX];
	unsigned int peek_n;
};

static struct isp_block isp_blocks[CAMSS_ISP_BLOCKS] = {
	[CAMSS_ISP_VFE] = { .name = "vfe" },
	[CAMSS_ISP_CSID] = { .name = "csid" },
};

static DEFINE_MUTEX(isp_lock);
static struct dentry *isp_dir;

void camss_isp_apply_script(enum camss_isp_block which, void __iomem *base)
{
	struct isp_block *b = &isp_blocks[which];
	unsigned int i;

	mutex_lock(&isp_lock);
	for (i = 0; i < b->n; i++)
		writel(b->val[i], base + b->off[i]);
	mutex_unlock(&isp_lock);
}

void camss_isp_set_live(enum camss_isp_block which, void __iomem *base)
{
	mutex_lock(&isp_lock);
	isp_blocks[which].base = base;
	mutex_unlock(&isp_lock);
}

static int isp_script_show(struct seq_file *s, void *unused)
{
	struct isp_block *b = s->private;
	unsigned int i;

	mutex_lock(&isp_lock);
	for (i = 0; i < b->n; i++)
		seq_printf(s, "0x%05x 0x%08x\n", b->off[i], b->val[i]);
	mutex_unlock(&isp_lock);

	return 0;
}

static int isp_script_open(struct inode *inode, struct file *file)
{
	return single_open(file, isp_script_show, inode->i_private);
}

static ssize_t isp_script_write(struct file *file, const char __user *ubuf,
				size_t len, loff_t *ppos)
{
	struct isp_block *b = ((struct seq_file *)file->private_data)->private;
	char *buf, *line, *cur;
	u32 off, val;

	buf = memdup_user_nul(ubuf, len);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	mutex_lock(&isp_lock);
	cur = buf;
	while ((line = strsep(&cur, "\n"))) {
		if (!strncmp(line, "clear", 5)) {
			b->n = 0;
			continue;
		}
		if (sscanf(line, "%x %x", &off, &val) != 2)
			continue;
		if (b->n >= ISP_SCRIPT_MAX || off & 3)
			break;
		b->off[b->n] = off;
		b->val[b->n] = val;
		b->n++;
	}
	mutex_unlock(&isp_lock);
	kfree(buf);

	return len;
}

static const struct file_operations isp_script_fops = {
	.owner = THIS_MODULE,
	.open = isp_script_open,
	.read = seq_read,
	.write = isp_script_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int isp_live_show(struct seq_file *s, void *unused)
{
	struct isp_block *b = s->private;
	unsigned int i;

	mutex_lock(&isp_lock);
	for (i = 0; i < b->peek_n; i++)
		seq_printf(s, "0x%05x 0x%08x\n", b->peek_off + 4 * i,
			   b->peek_val[i]);
	mutex_unlock(&isp_lock);

	return 0;
}

static int isp_live_open(struct inode *inode, struct file *file)
{
	return single_open(file, isp_live_show, inode->i_private);
}

static ssize_t isp_live_write(struct file *file, const char __user *ubuf,
			      size_t len, loff_t *ppos)
{
	struct isp_block *b = ((struct seq_file *)file->private_data)->private;
	char *buf;
	u32 off, val;
	unsigned int i;
	int ret = len;

	buf = memdup_user_nul(ubuf, len);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	mutex_lock(&isp_lock);
	if (!b->base) {
		ret = -ENODEV;
	} else if (sscanf(buf, "r %x %x", &off, &val) == 2) {
		if (off & 3 || !val || val > ISP_PEEK_MAX) {
			ret = -EINVAL;
		} else {
			b->peek_off = off;
			b->peek_n = val;
			for (i = 0; i < val; i++)
				b->peek_val[i] = readl(b->base + off + 4 * i);
		}
	} else if (sscanf(buf, "%x %x", &off, &val) == 2 && !(off & 3)) {
		writel(val, b->base + off);
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&isp_lock);
	kfree(buf);

	return ret;
}

static const struct file_operations isp_live_fops = {
	.owner = THIS_MODULE,
	.open = isp_live_open,
	.read = seq_read,
	.write = isp_live_write,
	.llseek = seq_lseek,
	.release = single_release,
};

void camss_isp_debug_init(void)
{
	char name[16];
	int i;

	if (isp_dir)
		return;

	isp_dir = debugfs_create_dir("camss_isp", NULL);
	for (i = 0; i < CAMSS_ISP_BLOCKS; i++) {
		snprintf(name, sizeof(name), "%s_script", isp_blocks[i].name);
		debugfs_create_file(name, 0600, isp_dir, &isp_blocks[i],
				    &isp_script_fops);
		snprintf(name, sizeof(name), "%s_live", isp_blocks[i].name);
		debugfs_create_file(name, 0600, isp_dir, &isp_blocks[i],
				    &isp_live_fops);
	}
}

void camss_isp_debug_exit(void)
{
	debugfs_remove_recursive(isp_dir);
	isp_dir = NULL;
}
