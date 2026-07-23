/*!
    \file    usbd_conf.h
    \brief   general low level driver configuration

    Adapted from GigaDevice GD32F4xx USB library V3.0.0 usb_conf.h.
*/

#pragma once

#include "gd32f4xx.h"
#include <stddef.h>

/****************** USBFS/USBHS PHY CONFIGURATION *******************************
 *  The USB OTG FS Core supports one on-chip Full Speed PHY.
*******************************************************************************/

#define USE_USB_FS

#ifdef USE_USB_FS
    #define USB_FS_CORE
#endif /* USE_USB_FS */

#ifdef USE_USB_HS
    #define USB_HS_CORE
#endif /* USE_USB_HS */

/****************** USBFS/USBHS CONFIGURATION **********************************/
#ifdef USB_FS_CORE
    #define USB_RX_FIFO_FS_SIZE                            128
    #define USB_HTX_NPFIFO_FS_SIZE                         96
    #define USB_HTX_PFIFO_FS_SIZE                          96

    /* FIFO sizes expected by GD32 USB library V3.0.x */
    #define RX_FIFO_FS_SIZE                                USB_RX_FIFO_FS_SIZE
    #define TX0_FIFO_FS_SIZE                               64
    #define TX1_FIFO_FS_SIZE                               64
    #define TX2_FIFO_FS_SIZE                               64
    #define TX3_FIFO_FS_SIZE                               64

    #define USBFS_SOF_OUTPUT                               0
    #define USBFS_LOW_POWER                                0

    /* V3.0.x library uses non-prefixed names */
    #define USB_SOF_OUTPUT                                 USBFS_SOF_OUTPUT
    #define USB_LOW_POWER                                  USBFS_LOW_POWER
#endif

#ifdef USB_HS_CORE
    #define USB_RX_FIFO_HS_SIZE                            512
    #define USB_HTX_NPFIFO_HS_SIZE                         256
    #define USB_HTX_PFIFO_HS_SIZE                          256

    #ifdef USE_ULPI_PHY
        #define USB_ULPI_PHY_ENABLED
    #endif

    #ifdef USE_EMBEDDED_PHY
        #define USB_EMBEDDED_PHY_ENABLED
    #endif

//    #define USB_HS_INTERNAL_DMA_ENABLED

    #define USBHS_SOF_OUTPUT                               0
    #define USBHS_LOW_POWER                                0
#endif

/****************** USB MODE CONFIGURATION ********************************/
#define USE_DEVICE_MODE

#ifndef USB_FS_CORE
    #ifndef USB_HS_CORE
        #error "USB_HS_CORE or USB_FS_CORE should be defined"
    #endif
#endif

#ifndef USE_DEVICE_MODE
    #ifndef USE_HOST_MODE
        #error "USE_DEVICE_MODE or USE_HOST_MODE should be defined"
    #endif
#endif

#ifndef USE_USB_HS
    #ifndef USE_USB_FS
        #error "USE_USB_HS or USE_USB_FS should be defined"
    #endif
#endif

/****************** C Compilers dependent keywords ****************************/
/* In HS mode and when the DMA is used, all variables and data structures dealing
   with the DMA during the transaction process should be 4-bytes aligned */
#ifdef USB_HS_INTERNAL_DMA_ENABLED
    #if defined   (__GNUC__)            /* GNU Compiler */
        #define __ALIGN_END __attribute__ ((aligned(4)))
        #define __ALIGN_BEGIN
    #else
        #define __ALIGN_END
        #if defined   (__CC_ARM)        /* ARM Compiler */
            #define __ALIGN_BEGIN __align(4)
        #elif defined (__ICCARM__)      /* IAR Compiler */
            #define __ALIGN_BEGIN
        #elif defined  (__TASKING__)    /* TASKING Compiler */
            #define __ALIGN_BEGIN __align(4)
        #endif                          /* __CC_ARM */
    #endif                              /* __GNUC__ */
#else
    #define __ALIGN_BEGIN
    #define __ALIGN_END
#endif /* USB_HS_INTERNAL_DMA_ENABLED */

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined   ( __GNUC__ )   /* GNU Compiler */
    #ifndef __packed
        #define __packed    __attribute__ ((__packed__))
    #endif
#elif defined   (__TASKING__)  /* TASKING Compiler */
    #define __packed __unaligned
#endif /* __GNUC__ */

/****************** USB DEVICE CONFIGURATION *********************************/
/* device interface/configuration counts */
#define USBD_ITF_MAX_NUM    2U
#define USBD_CFG_MAX_NUM    1U

/* CDC ACM endpoint/interface definitions required by cdc_acm_core.c */
#define CDC_COM_INTERFACE           0U
#define CDC_CMD_EP                  0x82U      /* interrupt IN */
#define CDC_DATA_IN_EP              0x81U      /* bulk IN */
#define CDC_DATA_OUT_EP             0x01U      /* bulk OUT */
#define USB_CDC_DATA_PACKET_SIZE    64U
#define USB_CDC_CMD_PACKET_SIZE     8U
