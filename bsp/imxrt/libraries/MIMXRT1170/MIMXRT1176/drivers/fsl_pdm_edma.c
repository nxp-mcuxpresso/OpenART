/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_pdm_edma.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.pdm_edma"
#endif

/*******************************************************************************
 * Definitations
 ******************************************************************************/
/* Used for 32byte aligned */
#define STCD_ADDR(address) (edma_tcd_t *)(((uint32_t)(address) + 32) & ~0x1FU)

/*<! Structure definition for pdm_edma_private_handle_t. The structure is private. */
typedef struct _pdm_edma_private_handle
{
    PDM_Type *base;
    pdm_edma_handle_t *handle;
} pdm_edma_private_handle_t;

/*! @brief pdm transfer state */
enum _pdm_edma_transfer_state
{
    kPDM_Busy = 0x0U, /*!< PDM is busy */
    kPDM_Idle,        /*!< Transfer is done. */
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Get the instance number for PDM.
 *
 * @param base PDM base pointer.
 */
static uint32_t PDM_GetInstance(PDM_Type *base);

/*!
 * @brief PDM EDMA callback for receive.
 *
 * @param handle pointer to pdm_edma_handle_t structure which stores the transfer state.
 * @param userData Parameter for user callback.
 * @param done If the DMA transfer finished.
 * @param tcds The TCD index.
 */
static void PDM_EDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief pdm base address pointer */
static PDM_Type *const s_pdmBases[] = PDM_BASE_PTRS;
/*<! Private handle only used for internally. */
static pdm_edma_private_handle_t s_edmaPrivateHandle[ARRAY_SIZE(s_pdmBases)];
/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t PDM_GetInstance(PDM_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_pdmBases); instance++)
    {
        if (s_pdmBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_pdmBases));

    return instance;
}

static void PDM_EDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds)
{
    pdm_edma_private_handle_t *privHandle = (pdm_edma_private_handle_t *)userData;
    pdm_edma_handle_t *pdmHandle          = privHandle->handle;

    if (!(pdmHandle->isLoopTransfer))
    {
        memset(&pdmHandle->tcd[pdmHandle->tcdDriver], 0, sizeof(edma_tcd_t));
        pdmHandle->tcdDriver = (pdmHandle->tcdDriver + 1) % pdmHandle->tcdNum;
    }

    pdmHandle->receivedBytes +=
        pdmHandle->tcd[pdmHandle->tcdDriver].BITER * (pdmHandle->tcd[pdmHandle->tcdDriver].NBYTES & 0x3FF);

    /* If finished a block, call the callback function */
    if (pdmHandle->callback)
    {
        (pdmHandle->callback)(privHandle->base, pdmHandle, kStatus_PDM_Idle, pdmHandle->userData);
    }

    pdmHandle->tcdUsedNum--;
    /* If all data finished, just stop the transfer */
    if ((pdmHandle->tcdUsedNum == 0U) && !(pdmHandle->isLoopTransfer))
    {
        /* Disable DMA enable bit */
        PDM_EnableDMA(privHandle->base, false);
        EDMA_AbortTransfer(handle);
    }
}

/*!
 * brief Initializes the PDM Rx eDMA handle.
 *
 * This function initializes the PDM slave DMA handle, which can be used for other PDM master transactional APIs.
 * Usually, for a specified PDM instance, call this API once to get the initialized handle.
 *
 * param base PDM base pointer.
 * param handle PDM eDMA handle pointer.
 * param base PDM peripheral base address.
 * param callback Pointer to user callback function.
 * param userData User parameter passed to the callback function.
 * param dmaHandle eDMA handle pointer, this handle shall be static allocated by users.
 */
void PDM_TransferCreateHandleEDMA(
    PDM_Type *base, pdm_edma_handle_t *handle, pdm_edma_callback_t callback, void *userData, edma_handle_t *dmaHandle)
{
    assert(handle && dmaHandle);

    uint32_t instance = PDM_GetInstance(base);

    /* Zero the handle */
    memset(handle, 0, sizeof(*handle));

    /* Set pdm base to handle */
    handle->dmaHandle = dmaHandle;
    handle->callback  = callback;
    handle->userData  = userData;

    /* Set PDM state to idle */
    handle->state = kPDM_Idle;

    s_edmaPrivateHandle[instance].base   = base;
    s_edmaPrivateHandle[instance].handle = handle;

    /* Install callback for Tx dma channel */
    EDMA_SetCallback(dmaHandle, PDM_EDMACallback, &s_edmaPrivateHandle[instance]);
}

/*!
 * brief Install EDMA descriptor memory.
 *
 * param handle Pointer to EDMA channel transfer handle.
 * param tcdAddr EDMA head descriptor address.
 * param tcdNum EDMA link descriptor address.
 */
void PDM_TransferInstallEDMATCDMemory(pdm_edma_handle_t *handle, void *tcdAddr, size_t tcdNum)
{
    assert(handle != NULL);

    handle->tcd    = (edma_tcd_t *)tcdAddr;
    handle->tcdNum = tcdNum;
}

/*!
 * brief Configures the PDM channel.
 *
 * param base PDM base pointer.
 * param handle PDM eDMA handle pointer.
 * param channel channel index.
 * param pdmConfig pdm channel configurations.
 */
void PDM_TransferSetChannelConfigEDMA(PDM_Type *base,
                                      pdm_edma_handle_t *handle,
                                      uint32_t channel,
                                      const pdm_channel_config_t *config)
{
    assert(handle && config);
    assert(channel < FSL_FEATURE_PDM_CHANNEL_NUM);

    /* Configure the PDM channel */
    PDM_SetChannelConfig(base, channel, config);

    /* record end channel number */
    handle->endChannel = channel;
    /* increase totoal enabled channel number */
    handle->channelNums++;
    /* increase count pre channel numbers */
    handle->count = base->FIFO_CTRL & PDM_FIFO_CTRL_FIFOWMK_MASK;
}

/*!
 * brief Performs a non-blocking PDM receive using eDMA.
 *
 * note This interface returns immediately after the transfer initiates. Call
 * the PDM_GetReceiveRemainingBytes to poll the transfer status and check whether the PDM transfer is finished.
 *
 * 1. Scatter gather case:
 * This functio support dynamic scatter gather and staic scatter gather,
 * a. for the dynamic scatter gather case:
 * Application should call PDM_TransferReceiveEDMA function continuously to make sure new receive request is submit
 *before the previous one finish. b. for the static scatter gather case: Application should use the link transfer
 *feature and make sure a loop link transfer is provided, such as: code pdm_edma_transfer_t pdmXfer[2] =
 *   {
 *       {
 *       .data  = s_buffer,
 *       .dataSize = BUFFER_SIZE,
 *       .linkTransfer = &pdmXfer[1],
 *       },
 *
 *       {
 *       .data  = &s_buffer[BUFFER_SIZE],
 *       .dataSize = BUFFER_SIZE,
 *       .linkTransfer = &pdmXfer[0]
 *       },
 *   };
 *endcode
 *
 * 2. Multi channel case:
 * This function support receive multi pdm channel data, for example, if two channel is requested,
 * code
 * PDM_TransferSetChannelConfigEDMA(DEMO_PDM, &s_pdmRxHandle_0, DEMO_PDM_ENABLE_CHANNEL_0, &channelConfig);
 * PDM_TransferSetChannelConfigEDMA(DEMO_PDM, &s_pdmRxHandle_0, DEMO_PDM_ENABLE_CHANNEL_1, &channelConfig);
 * PDM_TransferReceiveEDMA(DEMO_PDM, &s_pdmRxHandle_0, pdmXfer);
 * endcode
 *Then the output data will be formatted as:
 * -------------------------------------------------------------------------
 * |CHANNEL0 | CHANNEL1 | CHANNEL0 | CHANNEL1 | CHANNEL0 | CHANNEL 1 | ....|
 * -------------------------------------------------------------------------
 *
 * param base PDM base pointer
 * param handle PDM eDMA handle pointer.
 * param xfer Pointer to DMA transfer structure.
 * retval kStatus_Success Start a PDM eDMA receive successfully.
 * retval kStatus_InvalidArgument The input argument is invalid.
 * retval kStatus_RxBusy PDM is busy receiving data.
 */
status_t PDM_TransferReceiveEDMA(PDM_Type *base, pdm_edma_handle_t *handle, pdm_edma_transfer_t *xfer)
{
    assert(handle && xfer);

    edma_transfer_config_t config = {0};
    uint32_t startAddr            = PDM_GetDataRegisterAddress(base, handle->endChannel - (handle->channelNums - 1U));
    pdm_edma_transfer_t *currentTransfer = xfer;
    uint32_t nextTcdIndex = 0U, tcdIndex = handle->tcdUser;
    edma_minor_offset_config_t minorOffset = {
        .enableSrcMinorOffset  = true,
        .enableDestMinorOffset = false,
        .minorOffset           = 0xFFFFF - handle->channelNums * FSL_FEATURE_PDM_FIFO_OFFSET + 1U};

    /* Check if input parameter invalid */
    if ((xfer->data == NULL) || (xfer->dataSize == 0U))
    {
        return kStatus_InvalidArgument;
    }

    while (currentTransfer != NULL)
    {
        if (handle->tcdUsedNum >= handle->tcdNum)
        {
            return kStatus_PDM_QueueFull;
        }
        else
        {
            uint32_t primask = DisableGlobalIRQ();
            handle->tcdUsedNum++;
            EnableGlobalIRQ(primask);
        }

        nextTcdIndex = (handle->tcdUser + 1) % handle->tcdNum;

        if (handle->channelNums == 1U)
        {
            EDMA_PrepareTransferConfig(&config, (void *)startAddr, FSL_FEATURE_PDM_FIFO_WIDTH, 0U,
                                       (void *)currentTransfer->data, FSL_FEATURE_PDM_FIFO_WIDTH,
                                       FSL_FEATURE_PDM_FIFO_WIDTH, handle->count * FSL_FEATURE_PDM_FIFO_WIDTH,
                                       currentTransfer->dataSize);
        }
        else
        {
            EDMA_PrepareTransferConfig(&config, (void *)startAddr, FSL_FEATURE_PDM_FIFO_WIDTH,
                                       FSL_FEATURE_PDM_FIFO_OFFSET, (void *)currentTransfer->data,
                                       FSL_FEATURE_PDM_FIFO_WIDTH, FSL_FEATURE_PDM_FIFO_WIDTH,
                                       handle->channelNums * FSL_FEATURE_PDM_FIFO_WIDTH, currentTransfer->dataSize);
        }

        EDMA_TcdSetTransferConfig((edma_tcd_t *)&handle->tcd[handle->tcdUser], &config,
                                  (edma_tcd_t *)&handle->tcd[nextTcdIndex]);

        if (handle->channelNums > 1U)
        {
            EDMA_TcdSetMinorOffsetConfig((edma_tcd_t *)&handle->tcd[handle->tcdUser], &minorOffset);
        }

        EDMA_TcdEnableInterrupts((edma_tcd_t *)&handle->tcd[handle->tcdUser], kEDMA_MajorInterruptEnable);

        handle->tcdUser = nextTcdIndex;

        currentTransfer = currentTransfer->linkTransfer;

        if (currentTransfer == xfer)
        {
            handle->isLoopTransfer = true;
            break;
        }
    }

    if (handle->state != kPDM_Busy)
    {
        EDMA_InstallTCD(handle->dmaHandle->base, handle->dmaHandle->channel, (edma_tcd_t *)&handle->tcd[tcdIndex]);
        /* Start DMA transfer */
        EDMA_StartTransfer(handle->dmaHandle);

        /* Enable DMA enable bit */
        PDM_EnableDMA(base, true);
        /* enable PDM */
        PDM_Enable(base, true);

        handle->state = kPDM_Busy;
    }

    return kStatus_Success;
}

/*!
 * brief Aborts a PDM receive using eDMA.
 *
 * This function only aborts the current transfer slots, the other transfer slots' information still kept
 * in the handler. If users want to terminate all transfer slots, just call PDM_TransferTerminateReceiveEDMA.
 *
 * param base PDM base pointer
 * param handle PDM eDMA handle pointer.
 */
void PDM_TransferAbortReceiveEDMA(PDM_Type *base, pdm_edma_handle_t *handle)
{
    assert(handle);

    /* Disable dma */
    EDMA_AbortTransfer(handle->dmaHandle);

    /* Disable DMA enable bit */
    PDM_EnableDMA(base, false);

    /* Disable PDM */
    PDM_Enable(base, false);

    /* Handle the queue index */
    handle->tcdUsedNum--;

    /* Set the handle state */
    handle->state = kPDM_Idle;
}

/*!
 * brief Terminate all PDM receive.
 *
 * This function will clear all transfer slots buffered in the pdm queue. If users only want to abort the
 * current transfer slot, please call PDM_TransferAbortReceiveEDMA.
 *
 * param base PDM base pointer.
 * param handle PDM eDMA handle pointer.
 */
void PDM_TransferTerminateReceiveEDMA(PDM_Type *base, pdm_edma_handle_t *handle)
{
    assert(handle);

    /* Abort the current transfer */
    PDM_TransferAbortReceiveEDMA(base, handle);

    /* Clear all the internal information */
    memset(handle->tcd, 0U, sizeof(edma_tcd_t) * handle->tcdNum);
    handle->tcdUser    = 0U;
    handle->tcdUsedNum = 0U;
}

/*!
 * brief Gets byte count received by PDM.
 *
 * param base PDM base pointer
 * param handle PDM eDMA handle pointer.
 * param count Bytes count received by PDM.
 * retval kStatus_Success Succeed get the transfer count.
 * retval kStatus_NoTransferInProgress There is no non-blocking transaction in progress.
 */
status_t PDM_TransferGetReceiveCountEDMA(PDM_Type *base, pdm_edma_handle_t *handle, size_t *count)
{
    assert(handle);

    *count = handle->receivedBytes;

    return kStatus_Success;
}
