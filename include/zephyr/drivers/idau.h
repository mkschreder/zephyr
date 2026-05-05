/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Generic IDAU (Implementation-Defined Attribution Unit) HAL.
 *
 * The ARM Security Attribution Unit (SAU) can be supplemented or replaced
 * by a vendor-specific IDAU.  Common examples include:
 *
 *  - Nordic nRF52840 / nRF5340 SPU (Security Protection Unit)
 *  - Infineon PSE84 extended SAU (pse84_s_sau.c)
 *  - STM32H5 GTZC (Global Trust-Zone Controller)
 *
 * This HAL abstracts those differences behind a uniform interface so that
 * board-agnostic Secure firmware code can configure memory security without
 * hard-coding SoC-specific register sequences.
 *
 * SoC BSPs implement the HAL by filling in an @c idau_driver instance and
 * registering it via @ref IDAU_DRIVER_REGISTER.
 *
 * Reference: ARM DDI 0553B §B3.4 (SAU/IDAU interaction).
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_IDAU_H_
#define ZEPHYR_INCLUDE_DRIVERS_IDAU_H_

#include <zephyr/types.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup idau_interface IDAU (Implementation-Defined Attribution Unit) HAL
 * @{
 */

/** Security attribute for an IDAU region. */
enum idau_security_attr {
	IDAU_ATTR_SECURE     = 0, /**< Region is Secure. */
	IDAU_ATTR_NON_SECURE = 1, /**< Region is Non-Secure. */
	IDAU_ATTR_NSC        = 2, /**< Region is Non-Secure Callable (NSC). */
};

/** IDAU region descriptor. */
struct idau_region {
	uintptr_t base;               /**< Region base address. */
	size_t    size;               /**< Region size in bytes. */
	enum idau_security_attr attr; /**< Security attribute. */
};

/**
 * @brief IDAU driver interface.
 *
 * SoC BSPs fill in this struct and register it with IDAU_DRIVER_REGISTER.
 */
struct idau_driver {
	/**
	 * @brief Initialize the IDAU hardware.
	 * @return 0 on success, negative errno on error.
	 */
	int (*init)(void);

	/**
	 * @brief Configure a single IDAU region.
	 *
	 * @param region_index  Index of the IDAU region to configure.
	 * @param region        Region descriptor.
	 * @return 0 on success, -EINVAL if parameters are invalid.
	 */
	int (*set_region)(uint32_t region_index, const struct idau_region *region);

	/**
	 * @brief Lock the IDAU configuration (prevent further changes).
	 *
	 * Some SoCs (e.g. nRF5340) support locking the SPU configuration.
	 * The default implementation does nothing.
	 *
	 * @return 0 on success, negative errno on error.
	 */
	int (*lock)(void);
};

/**
 * @brief Register a SoC IDAU driver.
 *
 * Each SoC that provides an IDAU uses this macro to publish its driver
 * struct so that the generic IDAU API can call it.
 *
 * @param _driver  Pointer to the @c struct idau_driver implementation.
 */
#define IDAU_DRIVER_REGISTER(_driver)                                         \
	const STRUCT_SECTION_ITERABLE(idau_driver_ptr, _idau_drv_##_driver) = { \
		.drv = (_driver),                                             \
	}

/** Pointer wrapper for linker-section iteration. */
struct idau_driver_ptr {
	const struct idau_driver *drv;
};

TYPE_SECTION_START_EXTERN(struct idau_driver_ptr, idau_drivers);
TYPE_SECTION_END_EXTERN(struct idau_driver_ptr, idau_drivers);

/**
 * @brief Initialize all registered IDAU drivers.
 *
 * Called by z_arm_sau_dt_init (or a dedicated SYS_INIT) to set up vendor
 * IDAU hardware.  Safe to call even if no drivers are registered.
 *
 * @return 0 on success, first negative errno encountered on error.
 */
static inline int idau_init_all(void)
{
	int rc = 0;

	STRUCT_SECTION_FOREACH(idau_driver_ptr, p) {
		if (p->drv && p->drv->init) {
			rc = p->drv->init();
			if (rc < 0) {
				return rc;
			}
		}
	}
	return rc;
}

/**
 * @brief Configure a single region on all registered IDAU drivers.
 *
 * @param region_index  Region index passed to each driver.
 * @param region        Region descriptor.
 * @return 0 on success, first negative errno encountered on error.
 */
static inline int idau_set_region(uint32_t region_index,
				  const struct idau_region *region)
{
	int rc = 0;

	STRUCT_SECTION_FOREACH(idau_driver_ptr, p) {
		if (p->drv && p->drv->set_region) {
			rc = p->drv->set_region(region_index, region);
			if (rc < 0) {
				return rc;
			}
		}
	}
	return rc;
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_IDAU_H_ */
