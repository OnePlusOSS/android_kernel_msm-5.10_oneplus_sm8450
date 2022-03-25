#define pr_fmt(fmt) "[CORE]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include "oplus_chg_module.h"
#include "hal/oplus_chg_ic.h"
#include "mms/oplus_mms.h"

extern int oplus_mms_class_init(void);
extern void oplus_mms_class_exit(void);
extern int oplus_chg_ic_class_init(void);
extern void oplus_chg_ic_class_exit(void);

#ifdef MODULE
__attribute__((weak)) size_t __oplus_chg_module_start;
__attribute__((weak)) size_t __oplus_chg_module_end;

static inline struct oplus_chg_module *oplus_chg_find_first_module(void)
{
	return (struct oplus_chg_module *)&__oplus_chg_module_start;
}

static inline struct oplus_chg_module *oplus_chg_find_last_module(void)
{
	struct oplus_chg_module *oplus_module = (struct oplus_chg_module *)&__oplus_chg_module_end;
	return oplus_module--;
}
#endif /* MODULE */

static int __init oplus_chg_class_init(void)
{
	int rc;
	struct device_node *node;
#ifdef MODULE
	struct oplus_chg_module *oplus_module;
	struct oplus_chg_module *last_oplus_module;
#endif

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return -ENOTSUPP;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return -ENOTSUPP;

	rc = oplus_chg_ic_class_init();
	if (rc < 0) {
		chg_err("oplus_chg_ic_class init error, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_mms_class_init();
	if (rc < 0) {
		chg_err("oplus_mms_class init error, rc=%d\n", rc);
		goto mms_err;
	}

#ifdef MODULE
	oplus_module = oplus_chg_find_first_module();
	last_oplus_module = oplus_chg_find_last_module();
	while ((size_t)oplus_module <= (size_t)last_oplus_module) {
		if ((oplus_module->magic == OPLUS_CHG_MODEL_MAGIC) &&
		    (oplus_module->chg_module_init != NULL)) {
			rc = oplus_module->chg_module_init();
			if (rc < 0) {
				chg_err("%s init error, rc=%d\n", oplus_module->name, rc);
				oplus_module--;
				goto module_init_err;
			}
		}
		oplus_module++;
	}
#endif /* MODULE */

	return 0;

#ifdef MODULE
module_init_err:
	while ((size_t)oplus_module >= (size_t)&__oplus_chg_module_start) {
		if ((oplus_module->magic == OPLUS_CHG_MODEL_MAGIC) &&
		    (oplus_module->chg_module_exit != NULL))
			oplus_module->chg_module_exit();
		oplus_module--;
	}
	oplus_mms_class_exit();
#endif /* MODULE */
mms_err:
	oplus_chg_ic_class_exit();
	return rc;
}

static void __exit oplus_chg_class_exit(void)
{
#ifdef MODULE
	struct oplus_chg_module *oplus_module;

	oplus_module = oplus_chg_find_last_module();
	while ((size_t)oplus_module >= (size_t)&__oplus_chg_module_start) {
		if ((oplus_module->magic == OPLUS_CHG_MODEL_MAGIC) &&
		    (oplus_module->chg_module_exit != NULL))
			oplus_module->chg_module_exit();
		oplus_module--;
	}
#endif /* MODULE */
	oplus_mms_class_exit();
	oplus_chg_ic_class_exit();
}

subsys_initcall(oplus_chg_class_init);
module_exit(oplus_chg_class_exit);

MODULE_DESCRIPTION("oplus charge management subsystem");
MODULE_LICENSE("GPL");
