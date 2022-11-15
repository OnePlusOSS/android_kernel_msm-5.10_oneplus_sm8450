# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux Kernel SOC specific device drivers.
#

obj-$(CONFIG_ARCH_ACTIONS)	+= actions/
obj-$(CONFIG_SOC_ASPEED)	+= aspeed/
obj-$(CONFIG_ARCH_AT91)		+= atmel/
obj-y				+= bcm/
obj-$(CONFIG_ARCH_DOVE)		+= dove/
obj-$(CONFIG_MACH_DOVE)		+= dove/
obj-y				+= fsl/
obj-$(CONFIG_ARCH_GEMINI)	+= gemini/
obj-y				+= imx/
obj-$(CONFIG_ARCH_IXP4XX)	+= ixp4xx/
obj-$(CONFIG_SOC_XWAY)		+= lantiq/
obj-y				+= mediatek/
obj-y				+= amlogic/
obj-y				+= qcom/
obj-y				+= renesas/
obj-$(CONFIG_ARCH_ROCKCHIP)	+= rockchip/
obj-$(CONFIG_SOC_SAMSUNG)	+= samsung/
obj-$(CONFIG_SOC_SIFIVE)	+= sifive/
obj-y				+= sunxi/
obj-$(CONFIG_ARCH_TEGRA)	+= tegra/
obj-y				+= ti/
obj-$(CONFIG_ARCH_U8500)	+= ux500/
obj-$(CONFIG_PLAT_VERSATILE)	+= versatile/
obj-y				+= xilinx/
obj-$(CONFIG_ARCH_ZX)		+= zte/
obj-$(CONFIG_SOC_KENDRYTE)	+= kendryte/

#ifdef OPLUS_BUG_STABILITY
obj-$(CONFIG_OPLUS_SYSTEM_KERNEL)	+= oplus/system/
#endif /* OPLUS_BUG_STABILITY */

#ifdef OPLUS_FEATURE_MM_FEEDBACK
obj-$(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)	+= oplus/multimedia/
#endif /* OPLUS_FEATURE_MM_FEEDBACK */

#ifdef OPLUS_FEATURE_TP_BASIC
obj-$(CONFIG_OPLUS_DEVICE_IFNO)  +=  oplus/device_info/
#endif /* OPLUS_FEATURE_TP_BASIC */

#ifdef OPLUS_FEATURE_SENSOR_SMEM
obj-y                       += oplus/sensor/
#endif /* OPLUS_FEATURE_SENSOR_SMEM */

#ifdef OPLUS_DFR_FEATURE
obj-y += oplus/dfr/
#endif /* OPLUS_DFR_FEATURE */

#ifdef OPLUS_FEATURE_MIDAS_GKI
obj-$(CONFIG_OPLUS_FEATURE_MIDAS_GKI) += oplus/power/midas/
#endif

#ifdef OPLUS_FEATURE_DFT
obj-y += oplus/dft/
#endif /* ifdef OPLUS_FEATURE_DFT */

#ifdef OPLUS_FEATURE_BOOT
obj-y += oplus/boot/
#endif /* OPLUS_FEATURE_BOOT */

#ifdef OPLUS_FEATURE_MDMFEATURE
obj-$(CONFIG_OPLUS_FEATURE_MDMFEATURE)     += oplus/mdmfeature/
#endif /*OPLUS_FEATURE_MDMFEATURE*/
