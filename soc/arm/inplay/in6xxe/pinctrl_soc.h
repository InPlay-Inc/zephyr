/*
 * Copyright 2023 InPlay Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM_INPLAY_IN6XXE_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_INPLAY_IN6XXE_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#include <soc.h>
#include <dt-bindings/pinctrl/in6xxe-pinctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t pinctrl_soc_pin_t;

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)			\
	(															\
		DT_PROP_BY_IDX(node_id, prop, idx)                      \
	),


#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			\
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop),		\
				DT_FOREACH_PROP_ELEM, pinmux,				\
				Z_PINCTRL_STATE_PIN_INIT)}


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_INPLAY_IN6XXE_PINCTRL_SOC_H_ */
