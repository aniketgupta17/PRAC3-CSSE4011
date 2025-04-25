/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include "ibeacon_node_manager.h"
// extern uint32_t central_gatt_write(uint32_t count);

int main(void)
{
	ibeacon_node_manager_init();
	(void)central_gatt_write(0U);
	printk("THis shit is done");
	return 0;
}
