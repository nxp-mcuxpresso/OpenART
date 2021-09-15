/*
 * Copyright 2020, NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_key_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.key_manager"
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * brief Configures Master key settings.
 *
 * This function configures Key Manager's setting for Master key.
 *
 * param base Key Manager peripheral address.
 * param select select source for Master key.
 * param lock setting for lock Master key.
 * return  status of Master key control operation
 */
status_t KEYMGR_MasterKeyControll(KEY_MANAGER_Type *base, keymgr_select_t select, keymgr_lock_t lock)
{
    if (select != (KEYMGR_SEL_UDF || KEYMGR_SEL_PUF))
    {
        return kStatus_InvalidArgument;
    }

    base->CSR_MASTER_KEY_CTRL |=
        KEY_MANAGER_CSR_MASTER_KEY_CTRL_KEY_SELECT(select) | KEY_MANAGER_CSR_MASTER_KEY_CTRL_KEY_LOCK(lock);

    return kStatus_Success;
}

/*!
 * brief Configures OTFAD1 key settings.
 *
 * This function configures Key Manager's setting for OTFAD1 key.
 *
 * param base Key Manager peripheral address.
 * param select select source for OTFAD1 key.
 * param lock setting for lock OTFAD1 key.
 * return  status of OTFAD1 key control operation
 */
status_t KEYMGR_OTFAD1KeyControll(KEY_MANAGER_Type *base, keymgr_select_t select, keymgr_lock_t lock)
{
    if (select != (KEYMGR_SEL_OCOTP || KEYMGR_SEL_PUF))
    {
        return kStatus_InvalidArgument;
    }

    base->CSR_OTFAD1_KEY_CTRL |=
        KEY_MANAGER_CSR_OTFAD1_KEY_CTRL_KEY_SELECT(select) | KEY_MANAGER_CSR_OTFAD1_KEY_CTRL_KEY_LOCK(lock);

    return kStatus_Success;
}

/*!
 * brief Configures OTFAD2 key settings.
 *
 * This function configures Key Manager's setting for OTFAD2 key.
 *
 * param base Key Manager peripheral address.
 * param select select source for OTFAD2 key.
 * param lock setting for lock OTFAD2 key.
 * return  status of OTFAD2 key control operation
 */
status_t KEYMGR_OTFAD2KeyControll(KEY_MANAGER_Type *base, keymgr_select_t select, keymgr_lock_t lock)
{
    if (select != (KEYMGR_SEL_OCOTP || KEYMGR_SEL_PUF))
    {
        return kStatus_InvalidArgument;
    }

    base->CSR_OTFAD2_KEY_CTRL |=
        KEY_MANAGER_CSR_OTFAD2_KEY_CTRL_key_select(select) | KEY_MANAGER_CSR_OTFAD2_KEY_CTRL_KEY_LOCK(lock);

    return kStatus_Success;
}

/*!
 * brief Restart load key signal for IEE.
 *
 * This function genrates Key Manager's restart signal for IEE key.
 *
 * param base Key Manager peripheral address.
 */
void KEYMGR_IEEKeyReload(KEY_MANAGER_Type *base)
{
    base->CSR_IEE_KEY_CTRL |= KEYMGR_IEE_RELOAD;
}

/*!
 * brief Lock the key select from PUF.
 *
 * This function locks selection of key for PUF.
 *
 * param base Key Manager peripheral address.
 */
void KEYMGR_PUFKeyLock(KEY_MANAGER_Type *base, keymgr_lock_t lock)
{
    base->CSR_PUF_KEY_CTRL |= KEY_MANAGER_CSR_PUF_KEY_CTRL_PUF_KEY_LOCK(lock);
}

/*!
 * brief Sets the default configuration of Key manager slot.
 *
 * This function initialize Key Manager slot config structure to default values.
 *
 * param config Pointer to slot configuration structure.
 */
status_t KEYMGR_GetDefaultConfig(domain_slot_config_t *config)
{
    if (config == NULL)
    {
        return kStatus_InvalidArgument;
    }

    config->lockControl    = kKEYMGR_Unlock;
    config->allowUser      = kKEYMGR_Allow;
    config->allowNonSecure = kKEYMGR_Allow;
    config->domainLock     = kKEYMGR_Unlock;
    config->domainMapID    = 0u;

    return kStatus_Success;
}

/*!
 * brief Configures Slot Domain control.
 *
 * This function configures domain slot control which locks and allows writes.
 *
 * param base Key Manager peripheral address.
 * param config Pointer to slot configuration structure.
 * param slot Select slot to be configured.
 */
status_t KEYMGR_SlotControl(KEY_MANAGER_Type *base, domain_slot_config_t *config, keymgr_slot_t slot)
{
    if (slot == kKEYMGR_Slot0)
    {
        base->SLOT0_CTRL |= KEY_MANAGER_SLOT0_CTRL_LOCKED_DOMAIN_ID(config->domainMapID) |
                            KEY_MANAGER_SLOT0_CTRL_DOMAIN_LOCK(config->domainLock) |
                            KEY_MANAGER_SLOT0_CTRL_ALLOW_NONSECURE(config->allowNonSecure) |
                            KEY_MANAGER_SLOT0_CTRL_ALLOW_USER(config->allowUser) |
                            KEY_MANAGER_SLOT0_CTRL_LOCK_CONTROL(config->lockControl);
    }
    else if (slot == kKEYMGR_Slot1)
    {
        base->SLOT1_CTRL |= KEY_MANAGER_SLOT1_CTRL_LOCKED_DOMAIN_ID(config->domainMapID) |
                            KEY_MANAGER_SLOT1_CTRL_DOMAIN_LOCK(config->domainLock) |
                            KEY_MANAGER_SLOT1_CTRL_ALLOW_NONSECURE(config->allowNonSecure) |
                            KEY_MANAGER_SLOT1_CTRL_ALLOW_USER(config->allowUser) |
                            KEY_MANAGER_SLOT1_CTRL_LOCK_CONTROL(config->lockControl);
    }
    else if (slot == kKEYMGR_Slot2)
    {
        base->SLOT2_CTRL |= KEY_MANAGER_SLOT2_CTRL_LOCKED_DOMAIN_ID(config->domainMapID) |
                            KEY_MANAGER_SLOT2_CTRL_DOMAIN_LOCK(config->domainLock) |
                            KEY_MANAGER_SLOT2_CTRL_ALLOW_NONSECURE(config->allowNonSecure) |
                            KEY_MANAGER_SLOT2_CTRL_ALLOW_USER(config->allowUser) |
                            KEY_MANAGER_SLOT2_CTRL_LOCK_CONTROL(config->lockControl);
    }
    else if (slot == kKEYMGR_Slot3)
    {
        base->SLOT3_CTRL |= KEY_MANAGER_SLOT3_CTRL_LOCKED_DOMAIN_ID(config->domainMapID) |
                            KEY_MANAGER_SLOT3_CTRL_DOMAIN_LOCK(config->domainLock) |
                            KEY_MANAGER_SLOT3_CTRL_ALLOW_NONSECURE(config->allowNonSecure) |
                            KEY_MANAGER_SLOT3_CTRL_ALLOW_USER(config->allowUser) |
                            KEY_MANAGER_SLOT3_CTRL_LOCK_CONTROL(config->lockControl);
    }
    else if (slot == kKEYMGR_Slot4)
    {
        base->SLOT4_CTRL |= KEY_MANAGER_SLOT4_CTRL_LOCKED_DOMAIN_ID(config->domainMapID) |
                            KEY_MANAGER_SLOT4_CTRL_DOMAIN_LOCK(config->domainLock) |
                            KEY_MANAGER_SLOT4_CTRL_ALLOW_NONSECURE(config->allowNonSecure) |
                            KEY_MANAGER_SLOT4_CTRL_ALLOW_USER(config->allowUser) |
                            KEY_MANAGER_SLOT4_CTRL_LOCK_CONTROL(config->lockControl);
    }
    else
    {
        return kStatus_InvalidArgument;
    }

    return kStatus_Success;
}

/*!
 * brief Enables clock for Key Manager module.
 *
 * This function enables clocks for Key Manager module.
 *
 * param base Key Manager peripheral address.
 */
void KEYMGR_Init(KEY_MANAGER_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_EnableClock(kCLOCK_Key_Manager);
#endif
}
