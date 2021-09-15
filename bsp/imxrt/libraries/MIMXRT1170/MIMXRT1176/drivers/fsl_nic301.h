/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_NIC301_H_
#define _FSL_NIC301_H_

#define GPV0_BASE (0x41000000UL)
#define GPV1_BASE (0x41100000UL)
#define GPV4_BASE (0x41400000UL)

#define FN_MOD_AHB_OFFSET  (0x028UL)
#define WR_TIDEMARK_OFFSET (0x040UL)
#define READ_QOS_OFFSET    (0x100UL)
#define WRITE_QOS_OFFSET   (0x104UL)
#define FN_MOD_OFFSET      (0x108UL)

#define NIC_GC355_BASE   (GPV0_BASE + 0x42000)
#define NIC_PXP_BASE     (GPV0_BASE + 0x43000)
#define NIC_LCDIF_BASE   (GPV0_BASE + 0x44000)
#define NIC_LCDIFV2_BASE (GPV0_BASE + 0x45000)
#define NIC_CSI_BASE     (GPV0_BASE + 0x46000)

#define NIC_CAAM_BASE      (GPV1_BASE + 0x42000)
#define NIC_ENET1G_RX_BASE (GPV1_BASE + 0x43000)
#define NIC_ENET1G_TX_BASE (GPV1_BASE + 0x44000)
#define NIC_ENET_BASE      (GPV1_BASE + 0x45000)
#define NIC_USBO2_BASE     (GPV1_BASE + 0x46000)
#define NIC_USDHC1_BASE    (GPV1_BASE + 0x47000)
#define NIC_USDHC2_BASE    (GPV1_BASE + 0x48000)
#define NIC_ENET_QOS_BASE  (GPV1_BASE + 0x4A000)

#define NIC_CM7_BASE       (GPV4_BASE + 0x42000)
#define NIC_LPSRMIX_M_BASE (GPV4_BASE + 0x46000)
#define NIC_DMA_BASE       (GPV4_BASE + 0x47000)
#define NIC_IEE_BASE       (GPV4_BASE + 0x48000)

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief Set read_qos Value
 *
 * @param base Base address of GPV address
 * @param value Target value
 */
static void inline NIC_SetReadQos(uint32_t base, uint32_t value)
{
    *(volatile uint32_t *)(base + READ_QOS_OFFSET) = value;
    __DSB();
}

/*!
 * @brief Get read_qos Value
 *
 * @param base Base address of GPV address
 * @return Current value configured
 */
static inline uint32_t NIC_GetReadQos(uint32_t base)
{
    return *(volatile uint32_t *)(base + READ_QOS_OFFSET);
}

/*!
 * @brief Set write_qos Value
 *
 * @param base Base address of GPV address
 * @param value Target value
 */
static void inline NIC_SetWriteQos(uint32_t base, uint32_t value)
{
    *(volatile uint32_t *)(base + WRITE_QOS_OFFSET) = value;
    __DSB();
}

/*!
 * @brief Get write_qos Value
 *
 * @param base Base address of GPV address
 * @return Current value configured
 */
static inline uint32_t NIC_GetWriteQos(uint32_t base)
{
    return *(volatile uint32_t *)(base + WRITE_QOS_OFFSET);
}

/*!
 * @brief Set fn_mod_ahb Value
 *
 * @param base Base address of GPV address
 * @param value Target value
 */
static void inline NIC_SetFnModAhb(uint32_t base, uint32_t value)
{
    *(volatile uint32_t *)(base + FN_MOD_AHB_OFFSET) = value;
    __DSB();
}

/*!
 * @brief Get fn_mod_ahb Value
 *
 * @param base Base address of GPV address
 * @return Current value configured
 */
static inline uint32_t NIC_GetFnModAhb(uint32_t base)
{
    return *(volatile uint32_t *)(base + FN_MOD_AHB_OFFSET);
}

/*!
 * @brief Set wr_tidemark Value
 *
 * @param base Base address of GPV address
 * @param value Target value
 */
static inline void NIC_SetWrTideMark(uint32_t base, uint32_t value)
{
    *(volatile uint32_t *)(base + WR_TIDEMARK_OFFSET) = value;
    __DSB();
}

/*!
 * @brief Get wr_tidemark Value
 *
 * @param base Base address of GPV address
 * @return Current value configured
 */
static inline uint32_t NIC_GetWrTideMark(uint32_t base)
{
    return *(volatile uint32_t *)(base + WR_TIDEMARK_OFFSET);
}

/*!
 * @brief Set fn_mod Value
 *
 * @param base Base address of GPV address
 * @param value Target value
 */
static inline void NIC_SetFnMod(uint32_t base, uint32_t value)
{
    *(volatile uint32_t *)(base + FN_MOD_OFFSET) = value;
    __DSB();
}

/*!
 * @brief Get fn_mod Value
 *
 * @param base Base address of GPV address
 * @return Current value configured
 */
static inline uint32_t NIC_GetFnMod(uint32_t base)
{
    return *(volatile uint32_t *)(base + FN_MOD_OFFSET);
}

#if defined(__cplusplus)
}
#endif /* __cplusplus */
#endif /* _FSL_NIC301_H_ */
