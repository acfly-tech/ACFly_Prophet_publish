/*
 * Copyright (c) 2022-2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "diskio.h"
#include "ff.h"
#include "task.h"
#include "usb_osal.h"
#include "usbd_core.h"
#include "usbd_msc.h"
#include "usbd_cdc.h"
#include "usbd_hid.h"
#include "diskio.h"
#include "sd_diskio.h"

// 磁盘分区定义
#define MSC_DEV_DISKS_CNT 2
static const uint8_t MSC_DEV_DISKS[MSC_DEV_DISKS_CNT] = {DEV_SD_USR, DEV_SD_EXT};
static bool MSC_DEV_DISKS_READONLY[MSC_DEV_DISKS_CNT] = {false, true};
uint8_t get_MSC_DEV_DISKS_CNT(void)
{
	return MSC_DEV_DISKS_CNT;
}
const uint8_t* get_MSC_DEV_DISKS(void)
{
	return MSC_DEV_DISKS;
}
const bool* get_MSC_DEV_DISKS_READONLY(void)
{
	return MSC_DEV_DISKS_READONLY;
}


#if defined(BOARD_BUTTON_PRESSED_VALUE)
#define APP_BUTTON_PRESSED_VALUE BOARD_BUTTON_PRESSED_VALUE
#else
#define APP_BUTTON_PRESSED_VALUE 0
#endif

#define USB_BUS_ID 0

/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define MSC_IN_EP  0x84
#define MSC_OUT_EP 0x05

/*!< endpoint address */
#define HID_INT_EP          0x86
#define HID_INT_EP_SIZE     4
#define HID_INT_EP_INTERVAL 10

/*!< report descriptor size */
#define HID_MOUSE_REPORT_DESC_SIZE 74

#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN + MSC_DESCRIPTOR_LEN /*+ 25*/)

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#define MSC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#define MSC_MAX_MPS 64
#endif

static uint8_t cdc_acm_hid_msc_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0200, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x03, 0x01, USB_CONFIG_BUS_POWERED | USB_CONFIG_REMOTE_WAKEUP, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, 0x02),
    MSC_DESCRIPTOR_INIT(0x02, MSC_OUT_EP, MSC_IN_EP, MSC_MAX_MPS, 0x02),
//    /************** Descriptor of Joystick Mouse interface ****************/
//    /* 09 */
//    0x09,                          /* bLength: Interface Descriptor size */
//    USB_DESCRIPTOR_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
//    0x03,                          /* bInterfaceNumber: Number of Interface */
//    0x00,                          /* bAlternateSetting: Alternate setting */
//    0x01,                          /* bNumEndpoints */
//    0x03,                          /* bInterfaceClass: HID */
//    0x01,                          /* bInterfaceSubClass : 1=BOOT, 0=no boot */
//    0x02,                          /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
//    0,                             /* iInterface: Index of string descriptor */
//    /******************** Descriptor of Joystick Mouse HID ********************/
//    /* 18 */
//    0x09,                    /* bLength: HID Descriptor size */
//    HID_DESCRIPTOR_TYPE_HID, /* bDescriptorType: HID */
//    0x11,                    /* bcdHID: HID Class Spec release number */
//    0x01,
//    0x00,                       /* bCountryCode: Hardware target country */
//    0x01,                       /* bNumDescriptors: Number of HID class descriptors to follow */
//    0x22,                       /* bDescriptorType */
//    HID_MOUSE_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
//    0x00,
//    /******************** Descriptor of Mouse endpoint ********************/
//    /* 27 */
//    0x07,                         /* bLength: Endpoint Descriptor size */
//    USB_DESCRIPTOR_TYPE_ENDPOINT, /* bDescriptorType: */
//    HID_INT_EP,                   /* bEndpointAddress: Endpoint Address (IN) */
//    0x03,                         /* bmAttributes: Interrupt endpoint */
//    HID_INT_EP_SIZE,              /* wMaxPacketSize: 4 Byte max */
//    0x00,
//    HID_INT_EP_INTERVAL, /* bInterval: Polling Interval */
    /*
     * string0 descriptor
     */
    USB_LANGID_INIT(USBD_LANGID_STRING),
    /*
     * string1 descriptor
     */
    0x14,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'A', 0x00,                  /* wcChar0 */
    'C', 0x00,                  /* wcChar1 */
    'F', 0x00,                  /* wcChar2 */
    'L', 0x00,                  /* wcChar3 */
    'Y', 0x00,                  /* wcChar4 */
    '2', 0x00,                  /* wcChar5 */
    '0', 0x00,                  /* wcChar6 */
    '2', 0x00,                  /* wcChar7 */
    '5', 0x00,                  /* wcChar8 */
    /*
     * string2 descriptor
     */
    0x20,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'A', 0x00,                  /* wcChar0 */
    'C', 0x00,                  /* wcChar1 */
    'F', 0x00,                  /* wcChar2 */
    'L', 0x00,                  /* wcChar3 */
    'Y', 0x00,                  /* wcChar4 */
    ' ', 0x00,                  /* wcChar5 */
    'K', 0x00,                  /* wcChar6 */
    '9', 0x00,                  /* wcChar7 */
    ' ', 0x00,                  /* wcChar8 */
    '2', 0x00,                  /* wcChar9 */
    '0', 0x00,                  /* wcChar10 */
    '2', 0x00,                  /* wcChar11 */
    '5', 0x00,                  /* wcChar12 */
    '0', 0x00,                  /* wcChar13 */
    '8', 0x00,                  /* wcChar14 */
    /*
     * string3 descriptor
     */
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'A', 0x00,                  /* wcChar0 */
    'C', 0x00,                  /* wcChar1 */
    'F', 0x00,                  /* wcChar2 */
    'L', 0x00,                  /* wcChar3 */
    'Y', 0x00,                  /* wcChar4 */
    'F', 0x00,                  /* wcChar5 */
    'C', 0x00,                  /* wcChar6 */
    '0', 0x00,                  /* wcChar7 */
    '0', 0x00,                  /* wcChar8 */
    '1', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    /*
     * device qualifier descriptor
     */
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};

void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
	DSTATUS stat = SD_initialize(DEV_SD);
	if(stat==RES_OK){
		if(lun<MSC_DEV_DISKS_CNT)
	  {
			uint8_t MSC_DEV_DISK = MSC_DEV_DISKS[lun];
			disk_ioctl(MSC_DEV_DISK, GET_SECTOR_COUNT, block_num);
			disk_ioctl(MSC_DEV_DISK, GET_SECTOR_SIZE, block_size);
		}
	}
}

int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    DRESULT res = RES_ERROR;
		if(lun<MSC_DEV_DISKS_CNT)
		{
			uint8_t MSC_DEV_DISK = MSC_DEV_DISKS[lun];
    
			uint32_t block_size = 0;
			if (disk_ioctl(MSC_DEV_DISK, GET_SECTOR_SIZE, &block_size) != RES_OK || block_size == 0)
				return -1;
			res = SD_read(MSC_DEV_DISK, buffer, sector, length/block_size);
    }
		if(res==RES_OK){
      return 0;
    }
    else{
      return -1;
    }
}

int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    DRESULT res = RES_ERROR;
		if(lun<MSC_DEV_DISKS_CNT)
		{
			uint8_t MSC_DEV_DISK = MSC_DEV_DISKS[lun];
    
			uint32_t block_size = 0;
			if (disk_ioctl(MSC_DEV_DISK, GET_SECTOR_SIZE, &block_size) != RES_OK || block_size == 0)
				return -1;
			res = SD_write(MSC_DEV_DISK, buffer, sector, length/block_size);
    
			void mscDiskWritten(uint8_t lun);
			mscDiskWritten(lun);
		}
    if(res==RES_OK)
      return 0;
    else
      return -1;
}

/*!< hid mouse report descriptor */
static const uint8_t hid_mouse_report_desc[HID_MOUSE_REPORT_DESC_SIZE] = {
    0x05, 0x01, /* USAGE_PAGE (Generic Desktop) */
    0x09, 0x02, /* USAGE (Mouse) */
    0xA1, 0x01, /* COLLECTION (Application) */
    0x09, 0x01, /*   USAGE (Pointer) */

    0xA1, 0x00, /*   COLLECTION (Physical) */
    0x05, 0x09, /*     USAGE_PAGE (Button) */
    0x19, 0x01, /*     USAGE_MINIMUM (Button 1) */
    0x29, 0x03, /*     USAGE_MAXIMUM (Button 3) */

    0x15, 0x00, /*     LOGICAL_MINIMUM (0) */
    0x25, 0x01, /*     LOGICAL_MAXIMUM (1) */
    0x95, 0x03, /*     REPORT_COUNT (3) */
    0x75, 0x01, /*     REPORT_SIZE (1) */

    0x81, 0x02, /*     INPUT (Data,Var,Abs) */
    0x95, 0x01, /*     REPORT_COUNT (1) */
    0x75, 0x05, /*     REPORT_SIZE (5) */
    0x81, 0x01, /*     INPUT (Cnst,Var,Abs) */

    0x05, 0x01, /*     USAGE_PAGE (Generic Desk*/
    0x09, 0x30, /*     USAGE (X) */
    0x09, 0x31, /*     USAGE (Y) */
    0x09, 0x38,

    0x15, 0x81, /*     LOGICAL_MINIMUM (-127) */
    0x25, 0x7F, /*     LOGICAL_MAXIMUM (127) */
    0x75, 0x08, /*     REPORT_SIZE (8) */
    0x95, 0x03, /*     REPORT_COUNT (2) */

    0x81, 0x06, /*     INPUT (Data,Var,Rel) */
    0xC0, 0x09,
    0x3c, 0x05,
    0xff, 0x09,

    0x01, 0x15,
    0x00, 0x25,
    0x01, 0x75,
    0x01, 0x95,

    0x02, 0xb1,
    0x22, 0x75,
    0x06, 0x95,
    0x01, 0xb1,

    0x01, 0xc0 /*   END_COLLECTION */
};

/*!< mouse report struct */
struct hid_mouse {
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheel;
};

/*!< mouse report */
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX struct hid_mouse mouse_cfg;

#define HID_STATE_IDLE 0
#define HID_STATE_BUSY 1
usb_osal_sem_t semaphore_hid_tx_done;

/* function ------------------------------------------------------------------*/
static void usbd_hid_int_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;

    if (semaphore_hid_tx_done != NULL) {
        if (usb_osal_sem_give(semaphore_hid_tx_done) != 0) {
            CONFIG_USB_PRINTF("%s usb_osal_sem_give error\n", __func__);
        }
    }
}

/*!< endpoint call back */
static struct usbd_endpoint hid_in_ep = {
    .ep_cb = usbd_hid_int_callback,
    .ep_addr = HID_INT_EP
};

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[CDC_MAX_MPS];
volatile bool dtr_enable;

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
    case USBD_EVENT_RESET:
	{
		void usbd_vcom_resetCallBack();
		usbd_vcom_resetCallBack();
        break;
	}
    case USBD_EVENT_CONNECTED:
        break;
    case USBD_EVENT_DISCONNECTED:
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
		{
				void usbd_vcom_configuredCallBack(void);
				usbd_vcom_configuredCallBack();
        /* setup first out ep read transfer */
        usbd_ep_start_read(busid, CDC_OUT_EP, cdc_read_buffer, CDC_MAX_MPS);
        break;
		}
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual out len:%d\r\n", nbytes);
    void usbd_vcom_rxCallBack(uint8_t* buffer, uint32_t nbytes);
    usbd_vcom_rxCallBack(cdc_read_buffer, nbytes);

    /* setup next out ep read transfer */
    usbd_ep_start_read(busid, ep, cdc_read_buffer, CDC_MAX_MPS);
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual in len:%d\r\n", nbytes);

    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(busid, ep, NULL, 0);
    } else {  
        void usbd_vcom_txCallBack();
        usbd_vcom_txCallBack();
    }
}

/*!< endpoint call back */
struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

/* function ------------------------------------------------------------------*/
/**
 * @brief            msc ram init
 * @pre              none
 * @param[in]        none
 * @retval           none
 */
struct usbd_interface intf0;
struct usbd_interface intf1;
struct usbd_interface intf2;
struct usbd_interface intf3;

void cdc_acm_hid_msc_descriptor_init(uint8_t busid, uint32_t reg_base, uint8_t id)
{
	for(uint8_t i=0; i<MSC_DEV_DISKS_CNT; ++i)
	{
		if( MSC_DEV_DISKS[i] == DEV_SD_EXT )
		{
			MSC_DEV_DISKS_READONLY[i] = SDLOG_Wrt_isEnabled() ? false : true;
			break;
		}
	}

	uint8_t *p = (uint8_t *)cdc_acm_hid_msc_descriptor;
	for (int i = 0; i < sizeof(cdc_acm_hid_msc_descriptor) - 20; i++) {
        if (p[i+0] == 0x16 && p[i+1] == 0x03 && p[i+2] == 'A') {
            char id_str[4];
			sprintf(id_str, "%03u", id);   // 强制 3 位，如 001、123、255
			p[i + 16] = id_str[0];   // p[i+16]  → 百位
            p[i + 18] = id_str[1];   // p[i+18]  → 十位
            p[i + 20] = id_str[2];   // p[i+20]  → 个位
        }
    }

    semaphore_hid_tx_done = usb_osal_sem_create(0);
    //assert(semaphore_hid_tx_done != NULL);

    usbd_desc_register(busid, cdc_acm_hid_msc_descriptor);

    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);

    usbd_add_interface(busid, usbd_msc_init_intf(busid, &intf2, MSC_OUT_EP, MSC_IN_EP));

//    usbd_add_interface(busid, usbd_hid_init_intf(busid, &intf3, hid_mouse_report_desc, HID_MOUSE_REPORT_DESC_SIZE));
//    usbd_add_endpoint(busid, &hid_in_ep);

    /*!< init mouse report data */
    mouse_cfg.buttons = 0;
    mouse_cfg.wheel = 0;
    mouse_cfg.x = 0;
    mouse_cfg.y = 0;

    usbd_initialize(busid, reg_base, usbd_event_handler);
}

/**
 * @brief            hid mouse test
 * @pre              none
 * @param[in]        none
 * @retval           none
 */
void hid_mouse_test(uint8_t busid)
{
   
}

int cdc_startSend(const uint8_t* write_buffer, uint32_t len)
{
    return usbd_ep_start_write(USB_BUS_ID, CDC_IN_EP, write_buffer, len);
}
