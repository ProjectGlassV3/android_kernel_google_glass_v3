#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <uapi/linux/stat.h>

/* Limiting by default for safety reasons. */
volatile int limit_fcc_ua_val = 1;

int step_chg_jeita_query_limit_fcc_ua(void)
{
	return !!limit_fcc_ua_val;
}
EXPORT_SYMBOL(step_chg_jeita_query_limit_fcc_ua);

static ssize_t step_chg_jeita_sysfs_limit_fcc_ua_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
		sscanf(buf, "%d", &limit_fcc_ua_val);
		return count;
}

static struct kobj_attribute limit_fcc_ua_attribute =
	__ATTR(limit_fcc_ua, S_IRUGO | S_IWUSR,
	NULL,
	step_chg_jeita_sysfs_limit_fcc_ua_store);

static struct attribute *attrs[] = {
	&limit_fcc_ua_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *kobj;

static int step_chg_jeita_sysfs_init(void)
{
	int ret;

	kobj = kobject_create_and_add("step_chg_jeita", kernel_kobj);
	if (!kobj)
		return -ENOMEM;
	ret = sysfs_create_group(kobj, &attr_group);
	if (ret)
		kobject_put(kobj);
	return ret;
}

static void step_chg_jeita_sysfs_exit(void)
{
	kobject_put(kobj);
}

module_init(step_chg_jeita_sysfs_init);
module_exit(step_chg_jeita_sysfs_exit);

MODULE_AUTHOR("Google");
MODULE_DESCRIPTION("Step charge jeita driver");
MODULE_LICENSE("GPL");
