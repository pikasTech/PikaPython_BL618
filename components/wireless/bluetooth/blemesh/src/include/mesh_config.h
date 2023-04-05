#ifndef _MESH_CONFIG_H_
#define _MESH_CONFIG_H_

#include "FreeRTOSConfig.h"

#define CONFIG_BT_MESH_ADV_PRIO (configMAX_PRIORITIES - 4)

#ifndef CONFIG_BT_MESH_NODE_COUNT
#define CONFIG_BT_MESH_NODE_COUNT (128)
#endif

#ifndef CONFIG_BT_MESH_MODEL_KEY_COUNT
#define CONFIG_BT_MESH_MODEL_KEY_COUNT 2
#endif

#ifndef CONFIG_BT_MESH_MODEL_GROUP_COUNT
#define CONFIG_BT_MESH_MODEL_GROUP_COUNT 2
#endif

#ifndef CONFIG_BT_MESH_APP_KEY_COUNT
#define CONFIG_BT_MESH_APP_KEY_COUNT 2
#endif

#ifndef CONFIG_BT_MESH_SUBNET_COUNT
#define CONFIG_BT_MESH_SUBNET_COUNT 2
#endif

#ifndef CONFIG_BT_MESH_CRPL
#define CONFIG_BT_MESH_CRPL (CONFIG_BT_MESH_NODE_COUNT)
#endif

#ifndef CONFIG_BT_MESH_ADV_BUF_COUNT
#define CONFIG_BT_MESH_ADV_BUF_COUNT 60
#endif

#ifndef CONFIG_BT_MESH_LABEL_COUNT
#if defined(CONFIG_AUTO_PTS)
#define CONFIG_BT_MESH_LABEL_COUNT (CONFIG_BT_MESH_MODEL_GROUP_COUNT)
#else
#define CONFIG_BT_MESH_LABEL_COUNT 1
#endif /* CONFIG_AUTO_PTS */
#endif

#ifndef CONFIG_BT_MESH_TX_SEG_MAX
#if defined(CONFIG_AUTO_PTS)
#define CONFIG_BT_MESH_TX_SEG_MAX  13
#else
#define CONFIG_BT_MESH_TX_SEG_MAX  6
#endif /* CONFIG_AUTO_PTS */
#endif

#ifndef CONFIG_BT_MESH_RX_SEG_MAX
#define CONFIG_BT_MESH_RX_SEG_MAX CONFIG_BT_MESH_TX_SEG_MAX
#endif /* CONFIG_BT_MESH_RX_SEG_MAX */


#ifndef CONFIG_BT_MESH_MSG_CACHE_SIZE
#if defined(CONFIG_AUTO_PTS)
#define CONFIG_BT_MESH_MSG_CACHE_SIZE 10
#else
#define CONFIG_BT_MESH_MSG_CACHE_SIZE (2*CONFIG_BT_MESH_TX_SEG_MAX+CONFIG_BT_MESH_NODE_COUNT)
#endif
#endif

#ifndef CONFIG_BT_MESH_TX_SEG_MSG_COUNT
#if defined(CONFIG_AUTO_PTS)
#define CONFIG_BT_MESH_TX_SEG_MSG_COUNT 4
#else
#define CONFIG_BT_MESH_TX_SEG_MSG_COUNT 2
#endif
#endif

#ifndef CONFIG_BT_MESH_RX_SDU_MAX
#define CONFIG_BT_MESH_RX_SDU_MAX 108
#endif
#ifndef CONFIG_BT_MESH_SEG_BUFS
#define CONFIG_BT_MESH_SEG_BUFS (CONFIG_BT_MESH_TX_SEG_MAX*2)
#endif


#ifndef CONFIG_BT_MESH_RX_SEG_MSG_COUNT
#define CONFIG_BT_MESH_RX_SEG_MSG_COUNT 2
#endif

#ifndef CONFIG_MESH_ADV_STACK_SIZE
#define CONFIG_MESH_ADV_STACK_SIZE (1024+512)
#endif

#ifndef CONFIG_BT_MESH_PROXY_FILTER_SIZE
#define CONFIG_BT_MESH_PROXY_FILTER_SIZE 1
#endif

#ifndef CONFIG_BT_MESH_NODE_ID_TIMEOUT
#define CONFIG_BT_MESH_NODE_ID_TIMEOUT 60
#endif

#ifdef CONFIG_BT_MESH_FRIEND

#ifndef CONFIG_BT_MESH_FRIEND_RECV_WIN
#define CONFIG_BT_MESH_FRIEND_RECV_WIN 255
#endif

#ifndef CONFIG_BT_MESH_FRIEND_QUEUE_SIZE
#define CONFIG_BT_MESH_FRIEND_QUEUE_SIZE 16
#endif

#ifndef CONFIG_BT_MESH_FRIEND_SUB_LIST_SIZE
#define CONFIG_BT_MESH_FRIEND_SUB_LIST_SIZE 3
#endif

#ifndef CONFIG_BT_MESH_FRIEND_LPN_COUNT
#define CONFIG_BT_MESH_FRIEND_LPN_COUNT 2
#endif

#ifndef CONFIG_BT_MESH_FRIEND_SEG_RX
#define CONFIG_BT_MESH_FRIEND_SEG_RX 1
#endif

#endif // CONFIG_BT_MESH_FRIEND

#ifdef CONFIG_BT_MESH_LOW_POWER

//#ifndef CONFIG_BT_MESH_LPN_ESTABLISHMENT
//#define CONFIG_BT_MESH_LPN_ESTABLISHMENT
//#endif

//#ifndef CONFIG_BT_MESH_LPN_AUTO
//#define CONFIG_BT_MESH_LPN_AUTO
//#endif

#ifndef CONFIG_BT_MESH_LPN_AUTO_TIMEOUT
#define CONFIG_BT_MESH_LPN_AUTO_TIMEOUT 15
#endif

#ifndef CONFIG_BT_MESH_LPN_RETRY_TIMEOUT
#define CONFIG_BT_MESH_LPN_RETRY_TIMEOUT 8
#endif

#ifndef CONFIG_BT_MESH_LPN_RSSI_FACTOR
#define CONFIG_BT_MESH_LPN_RSSI_FACTOR 0
#endif

#ifndef CONFIG_BT_MESH_LPN_RECV_WIN_FACTOR
#define CONFIG_BT_MESH_LPN_RECV_WIN_FACTOR 0
#endif

#ifndef CONFIG_BT_MESH_LPN_MIN_QUEUE_SIZE
#define CONFIG_BT_MESH_LPN_MIN_QUEUE_SIZE 1
#endif

#ifndef CONFIG_BT_MESH_LPN_RECV_DELAY
#define CONFIG_BT_MESH_LPN_RECV_DELAY 100
#endif

#ifndef CONFIG_BT_MESH_LPN_POLL_TIMEOUT
#define CONFIG_BT_MESH_LPN_POLL_TIMEOUT 300
#endif

#ifndef CONFIG_BT_MESH_LPN_INIT_POLL_TIMEOUT
#define CONFIG_BT_MESH_LPN_INIT_POLL_TIMEOUT CONFIG_BT_MESH_LPN_POLL_TIMEOUT
#endif

#ifndef CONFIG_BT_MESH_LPN_SCAN_LATENCY
#define CONFIG_BT_MESH_LPN_SCAN_LATENCY 10
#endif

#ifndef CONFIG_BT_MESH_LPN_GROUPS
#define CONFIG_BT_MESH_LPN_GROUPS 8
#endif

#endif // CONFIG_BT_MESH_LOW_POWER

#ifdef CONFIG_BT_MESH_PROXY
#ifndef CONFIG_BT_MESH_PROXY_FILTER_SIZE
#define CONFIG_BT_MESH_PROXY_FILTER_SIZE 1
#endif
#endif // BT_MESH_PROXY

#ifndef CONFIG_BT_MESH_IVU_DIVIDER
#define CONFIG_BT_MESH_IVU_DIVIDER 2
#endif

#ifndef CONFIG_BT_MESH_SEQ_STORE_RATE
#define CONFIG_BT_MESH_SEQ_STORE_RATE (100) //tmp value, need to check
#endif

#ifndef CONFIG_BT_MESH_STORE_TIMEOUT
#define CONFIG_BT_MESH_STORE_TIMEOUT 3
#endif

#ifndef CONFIG_BT_MESH_RPL_STORE_TIMEOUT
#define CONFIG_BT_MESH_RPL_STORE_TIMEOUT 4 //tmp value, need to check
#endif

#ifndef CONFIG_BT_DEVICE_NAME
#define CONFIG_BT_DEVICE_NAME "bl_mesh"
#endif

#ifndef CONFIG_BT_MESH_LOOPBACK_BUFS
#define CONFIG_BT_MESH_LOOPBACK_BUFS (CONFIG_BT_MESH_TX_SEG_MAX \
								+ CONFIG_BT_MESH_RX_SEG_MAX) //temp value, need to check
#endif

#define BL_COMP_ID                           0x07AF

/********************************BFLB_BLE patch to fix mesh bug**************************************/
#define BFLB_BLE_MESH_PATCH_DEL_APPKEY
#define BFLB_BLE_MESH_PATCH_NET_REVOKE_KEYS
#define BFLB_BLE_MESH_PATCH_NET_DECODE
#define BFLB_BLE_MESH_PATCH_IGNORE_SECURE_BEACON
#define BFLB_BLE_MESH_PATCH_MOD_PUB_VA_SET
#define BFLB_BLE_MESH_PATCH_HEARTBEAT_SUB_GET
#define BFLB_BLE_MESH_PATCH_HEARTBEAT_SUB_SET
#endif
