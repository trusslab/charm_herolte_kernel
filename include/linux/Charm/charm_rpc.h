/* Copyright (C) 2016-2018 University of California, Irvine
 * 
 * Authors:
 * Seyed Mohammadjavad Seyed Talebi <mjavad@uci.edu>
 * Ardalan Amiri Sani <arrdalan@gmail.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define CLK_GET_RPC_CODE 1
#define CLK_ROUND_RATE_RPC_CODE 2
#define CLK_SET_RATE_RPC_CODE 3
#define CLK_GET_RATE_RPC_CODE 4
#define CLK_PREPARE_RPC_CODE 5
#define CLK_ENABLE_RPC_CODE 6
#define REGULATOR_GET_RPC_CODE 7
#define REGULATOR_ENABLE_RPC_CODE 8
#define REGULATOR_SET_VOLTAGE_RPC_CODE 9
#define REGULATOR_SET_OPT_RPC_CODE 10
#define REGULATOR_COUNT_VOLTAGES_RPC_CODE 11
#define PINCTRL_GET_RPC_CODE 12
#define PINCTRL_LOOKUP_STATE_RPC_CODE 13
#define PINCTRL_SELECT_STATE_RPC_CODE 14
#define GPIO_REQUEST_ONE_RPC_CODE 15
#define IS_ERR_RPC_CODE 16
#define PINCTRL_PUT_RPC_CODE 17
#define  IS_ERR_OR_NULL_RPC_CODE 18
#define PM_REQUEST_IDLE_RPC_CODE 19
#define PM_RUNTIME_BARRIER_RPC_CODE 20
#define OF_GET_GPIO_RPC_CODE 21
#define OF_GET_NAMED_GPIO_RPC_CODE 22
#define CLK_DISABLE_RPC_CODE 23
#define CLK_UNPREPARE_RPC_CODE 24
#define CLK_PUT_RPC_CODE 25
#define REGULATOR_DISABLE_RPC_CODE 26
#define REGULATOR_PUT_RPC_CODE 27
#define CLK_SET_PARENT_RPC_CODE 28
#define GPIO_SET_VALUE_CAN_SLEEP_RPC_CODE 29
#define RPM_REGULATOR_GET_RPC_CODE 30
#define RPM_REGULATOR_SET_MODE_RPC_CODE 31
#define RPM_REGULATOR_PUT_RPC_CODE 32
#define RPM_REGULATOR_ENABLE_RPC_CODE 33
#define RPM_REGULATOR_DISABLE_RPC_CODE 34
#define RPM_REGULATOR_SET_VOLTAGE_RPC_CODE 35
#define GPIO_REQUEST_RPC_CODE 36
#define GPIO_SET_VALUE_RPC_CODE 37
#define GPIO_GET_VALUE_RPC_CODE 38
#define GPIO_TO_IRQ_RPC_CODE 39
#define IRQ_TO_GPIO_RPC_CODE 40
#define GPIO_DIRECTION_OUTPUT_RPC_CODE 41
#define GPIO_DIRECTION_INPUT_RPC_CODE 42

#define CLK_RESET_RPC_CODE 43
#define MSM_IOMMU_GET_CTX_RPC_CODE 44
#define MSM_GET_IOMMU_DOMAIN_RPC_CODE 45
#define MSM_REGISTER_DOMAIN_RPC 46
#define IOMMU_ATTACH_DEVICE_RPC_CODE 47
#define IOMMU_DETACH_DEVICE_RPC_CODE 48
#define MSM_ION_CLIENT_CREATE_RPC_CODE 49
#define ION_CLIENT_DESTROY_RPC_CODE 50
#define ION_FREE_RPC_CODE 51
#define ION_MAP_IOMMU_RPC_CODE 52
#define ION_UNMAP_IOMMU_RPC_CODE 53
#define ION_IMPORT_DMA_RPC_CODE 54
#define GPIO_FREE_RPC_CODE 55
#define PTR_ERR_RPC_CODE 56

#define CLK_OPS_LIST_RATE_RPC_CODE 61
#define REGULATOR_IS_ENABLED_RPC_CODE 62
#define CLK_GET_PARENT_RPC_CODE 63
#define CLK_GET_SYS_RPC_CODE 64
#define GPIO_EXPORT_RPC_CODE 65
#define GPIO_IS_VALID_RPC_CODE 66
              
/* Exynos */
#define EXYNOS_UPDATE_IP_IDLE_STATUS 67
#define EXYNOS_GET_IDLE_IP_INDEX 68
#define EXYNOS_SMC 69

#define REGULATOR_GET_VOLTAGE_RPC_CODE 70
#define REBOOT_RPC_CODE 100

uint64_t rpc_callback(void *);
int dev_name_transform(char *, char*);
