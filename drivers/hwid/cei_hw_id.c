#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>

static char *hwid_string[] = {"EVT", "DVT1", "DVT2", "PVT", "MP"};
static char *hwid_invalid[] = {"HW_ID_INVALID"};
static char *prjtid_string[] = {"CT41", "CT42", "CT43", "CT44"};
static char *prjtid_invalid[] = {"PROJECT_ID_INVALID"};

int cei_hw_id;
int cei_prjt_id;
int cei_ftm_pin;

int get_cei_hw_id(void)
{
	return cei_hw_id;
}
EXPORT_SYMBOL(get_cei_hw_id);

int get_cei_prjt_id(void)
{
	return cei_prjt_id;
}
EXPORT_SYMBOL(get_cei_prjt_id);

static int __init cei_hw_id_from_cmdline(char *cei_hwid)
{
	cei_hw_id = 0;

	if (kstrtoint(cei_hwid, 0, &cei_hw_id))
		pr_debug("Read CEI HW ID =%d from cmdline!!\n", cei_hw_id);
	return 1;
}
__setup("CEI_HW_ID=", cei_hw_id_from_cmdline);

static int __init cei_prjt_id_from_cmdline(char *cei_prjtid)
{
	cei_prjt_id = 0;

	if (kstrtoint(cei_prjtid, 0, &cei_prjt_id))
		pr_debug("Read CEI Project ID =%d from cmdline!!\n"
			, cei_prjt_id);
	return 1;
}
__setup("CEI_PROJECT_ID=", cei_prjt_id_from_cmdline);

static int __init cei_ftm_pin_from_cmdline(char *cei_ftm)
{
	cei_ftm_pin = 0;

	if (kstrtoint(cei_ftm, 0, &cei_ftm_pin))
		pr_debug("Read CEI FTM Pin =%d from cmdline!!\n"
			, cei_ftm_pin);
	return 1;
}
__setup("CEI_FTM=", cei_ftm_pin_from_cmdline);

static const char *cei_prjt_id_read(void)
{
	if (cei_prjt_id >= 0 && cei_prjt_id < ARRAY_SIZE(prjtid_string))
		return prjtid_string[cei_prjt_id];
	else
		return prjtid_invalid[0];
}

static const char *cei_hw_id_read(void)
{
	if (cei_hw_id >= 0 && cei_hw_id < ARRAY_SIZE(hwid_string))
		return hwid_string[cei_hw_id];
	else
		return hwid_invalid[0];
}

static int prjtid_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", cei_prjt_id_read());
	return 0;
}

static int hwid_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", cei_hw_id_read());
	return 0;
}

static int ftm_dump_read(struct seq_file *m, void *v)
{
	if (cei_ftm_pin == 1)
		seq_puts(m, "1\n");
	else
		seq_puts(m, "0\n");
	return 0;
}

static int cei_proj_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, prjtid_dump_read, NULL);
}

static int cei_hw_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwid_dump_read, NULL);
}

static int cei_ftm_pin_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ftm_dump_read, NULL);
}

static const struct file_operations cei_proj_id_fops = {
	.open       = cei_proj_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static const struct file_operations cei_hw_id_fops = {
	.open       = cei_hw_id_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static const struct file_operations cei_ftm_pin_fops = {
	.open       = cei_ftm_pin_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

void cei_hwid_info_read(void)
{
	proc_create("cei_proj_id_type", 0, NULL, &cei_proj_id_fops);
	proc_create("cei_hw_id_type", 0, NULL, &cei_hw_id_fops);
	proc_create("cei_ftm_pin_type", 0, NULL, &cei_ftm_pin_fops);
}
EXPORT_SYMBOL(cei_hwid_info_read);

static int __init cei_hw_id_init(void)
{
	int err = 0;

	cei_hwid_info_read();
	return err;
}

static void __exit cei_hw_id_exit(void)
{
	pr_info("cei_hwid_exit enter\n");
}

arch_initcall(cei_hw_id_init);
module_exit(cei_hw_id_exit);

MODULE_DESCRIPTION("cei hardware ID driver");
MODULE_LICENSE("GPL");
