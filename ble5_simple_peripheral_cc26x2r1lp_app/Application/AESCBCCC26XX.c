/*
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/dpl/DebugP.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/AESCBC.h>
#include <ti/drivers/aescbc/AESCBCCC26XX.h>
#include <ti/drivers/cryptoutils/sharedresources/CryptoResourceCC26XX.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKey.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_crypto.h)
#include DeviceFamily_constructPath(driverlib/aes.h)
#include DeviceFamily_constructPath(driverlib/interrupt.h)

/* Forward declarations */
static void AESCBC_hwiFxn (uintptr_t arg0);
static void AESCBC_swiFxn (uintptr_t arg0, uintptr_t arg1);
static inline int_fast16_t AESCBC_waitForAccess(AESCBC_Handle handle);
static int_fast16_t AESCBC_startOperation(AESCBC_Handle handle,
                                          AESCBC_Operation *operation,
                                          AESCBC_OperationType operationType);
static int_fast16_t AESCBC_waitForResult(AESCBC_Handle handle);
static void AESCBC_cleanup(AESCBC_Handle handle);

/* Extern globals */
extern const AESCBC_Config AESCBC_config[];
extern const uint_least8_t AESCBC_count;

/* Static globals */
static bool isInitialized = false;

/*
 *  ======== AESCBC_swiFxn ========
 */
static void AESCBC_swiFxn (uintptr_t arg0, uintptr_t arg1) {
    AESCBCCC26XX_Object *object = ((AESCBC_Handle)arg0)->object;

    /* Handle cleaning up of the operation. Invalidate the key,
     * release the Power constraints, and post the access semaphore.
     */
    AESCBC_cleanup((AESCBC_Handle)arg0);

    if (object->returnBehavior == AESCBC_RETURN_BEHAVIOR_BLOCKING) {
        /* Unblock the pending task to signal that the operation is complete. */
        SemaphoreP_post(&CryptoResourceCC26XX_operationSemaphore);
    }
    else {
        /* Call the callback function provided by the application.
         */
        object->callbackFxn((AESCBC_Handle)arg0,
                            object->returnStatus,
                            object->operation,
                            object->operationType);
    }
}

/*
 *  ======== AESCBC_hwiFxn ========
 */
static void AESCBC_hwiFxn (uintptr_t arg0) {
    AESCBCCC26XX_Object *object = ((AESCBC_Handle)arg0)->object;

    /* Propagate the DMA error from driverlib to the application */
    if (AESIntStatusRaw() & AES_DMA_BUS_ERR) {
        object->returnStatus = AESCBC_STATUS_ERROR;
    }

    AESIntClear(AES_RESULT_RDY | AES_DMA_IN_DONE | AES_DMA_BUS_ERR);

    SwiP_post(&(object->callbackSwi));
}

/*
 *  ======== AESCBC_cleanup ========
 */
static void AESCBC_cleanup(AESCBC_Handle handle) {
    /* Since plaintext keys use two reserved (by convention) slots in the keystore,
     * the slots must be invalidated to prevent its re-use without reloading
     * the key material again.
     */
    AESInvalidateKey(AES_KEY_AREA_6);
    AESInvalidateKey(AES_KEY_AREA_7);

    /*  This powers down all sub-modules of the crypto module until needed.
     *  It does not power down the crypto module at PRCM level and provides small
     *  power savings.
     */
    AESSelectAlgorithm(0x00);

    Power_releaseConstraint(PowerCC26XX_DISALLOW_STANDBY);

    /*  Grant access for other threads to use the crypto module.
     *  The semaphore must be posted before the callbackFxn to allow the chaining
     *  of operations.
     */
    SemaphoreP_post(&CryptoResourceCC26XX_accessSemaphore);
}

/*
 *  ======== AESCBC_init ========
 */
void AESCBC_init(void) {
    uint_least8_t i;
    uint_fast8_t key;

    key = HwiP_disable();

    if (!isInitialized) {
        /* Initialize objects */
        for (i = 0; i < AESCBC_count; i++) {
            AESCBC_Handle handle = (AESCBC_Handle)&(AESCBC_config[i]);
            AESCBCCC26XX_Object *object = (AESCBCCC26XX_Object *)handle->object;
            object->isOpen = false;
        }

        isInitialized = true;
    }

    HwiP_restore(key);
}

/*
 *  ======== AESCBC_open ========
 */
AESCBC_Handle AESCBC_open(uint_least8_t index, AESCBC_Params *params) {
    SwiP_Params                 swiParams;
    AESCBC_Handle               handle;
    AESCBCCC26XX_Object        *object;
    AESCBCCC26XX_HWAttrs const *hwAttrs;
    uint_fast8_t                key;

    handle = (AESCBC_Handle)&(AESCBC_config[index]);
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    DebugP_assert(index >= AESCBC_count);

    key = HwiP_disable();

    if (!isInitialized ||  object->isOpen) {
        HwiP_restore(key);
        return NULL;
    }

    object->isOpen = true;

    HwiP_restore(key);

    /* If params are NULL, use defaults */
    if (params == NULL) {
        params = (AESCBC_Params *)&AESCBC_defaultParams;
    }

    /* This is currently not supported. */
    DebugP_assert(!params->nonceInternallyGenerated);
    DebugP_assert(params->returnBehavior == AESCBC_RETURN_BEHAVIOR_CALLBACK ? params->callbackFxn : true);

    object->returnBehavior = params->returnBehavior;
    object->callbackFxn = params->callbackFxn;
    object->semaphoreTimeout = params->returnBehavior == AESCBC_RETURN_BEHAVIOR_BLOCKING ? params->timeout : SemaphoreP_NO_WAIT;

    /* Create Swi object for this AESCBC peripheral */
    SwiP_Params_init(&swiParams);
    swiParams.arg0 = (uintptr_t)handle;
    swiParams.priority = hwAttrs->swiPriority;
    SwiP_construct(&(object->callbackSwi), AESCBC_swiFxn, &swiParams);

    CryptoResourceCC26XX_constructRTOSObjects();

    /* Set power dependency - i.e. power up and enable clock for Crypto (CryptoResourceCC26XX) module. */
    Power_setDependency(PowerCC26XX_PERIPH_CRYPTO);

    return handle;
}

/*
 *  ======== AESCBC_close ========
 */
void AESCBC_close(AESCBC_Handle handle) {
    AESCBCCC26XX_Object         *object;

    DebugP_assert(handle);

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;

    CryptoResourceCC26XX_destructRTOSObjects();

    /* Destroy the Swi */
    SwiP_destruct(&(object->callbackSwi));

    /* Release power dependency on Crypto Module. */
    Power_releaseDependency(PowerCC26XX_PERIPH_CRYPTO);

    /* Mark the module as available */
    object->isOpen = false;
}

/*
 *  ======== AESCBC_startOperation ========
 */
static int_fast16_t AESCBC_startOperation(AESCBC_Handle handle,
                                          AESCBC_Operation *operation,
                                          AESCBC_OperationType operationType) {
    AESCBCCC26XX_Object *object = handle->object;
    AESCBCCC26XX_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Only plaintext CryptoKeys are supported for now */
    uint16_t keyLength = operation->key->u.plaintext.keyLength;
    uint8_t *keyingMaterial = operation->key->u.plaintext.keyMaterial;

    DebugP_assert(handle);

    /* Try and obtain access to the crypto module */
    if (AESCBC_waitForAccess(handle) != SemaphoreP_OK) {
        return AESCBC_STATUS_RESOURCE_UNAVAILABLE;
    }

    object->operationType = operationType;
    object->operation = operation;
    /* We will only change the returnStatus if there is an error */
    object->returnStatus = AESCBC_STATUS_SUCCESS;


    /* We need to set the HWI function and priority since the same physical interrupt is shared by multiple
     * drivers and they all need to coexist. Whenever a driver starts an operation, it
     * registers its HWI callback with the OS.
     */
    HwiP_setFunc(&CryptoResourceCC26XX_hwi, AESCBC_hwiFxn, (uintptr_t)handle);
    HwiP_setPriority(INT_CRYPTO_RESULT_AVAIL_IRQ, hwAttrs->intPriority);

    /* Load the key from RAM or flash into the key store at a hardcoded and reserved location */
    if (AESWriteToKeyStore(keyingMaterial, keyLength, AES_KEY_AREA_6) != AES_SUCCESS) {
        return AESCBC_STATUS_ERROR;
    }


    /* If we are in AESCBC_RETURN_BEHAVIOR_POLLING, we do not want an interrupt to trigger.
     * AESWriteToKeyStore() disables and then re-enables the CRYPTO IRQ in the NVIC so we
     * need to disable it before kicking off the operation.
     */
    if (object->returnBehavior == AESCBC_RETURN_BEHAVIOR_POLLING)  {
        IntDisable(INT_CRYPTO_RESULT_AVAIL_IRQ);
    }

    /* Power the AES sub-module of the crypto module */
    AESSelectAlgorithm(AES_ALGSEL_AES);

    /* Load the key from the key store into the internal register banks of the AES sub-module */
    if (AESReadFromKeyStore(AES_KEY_AREA_6) != AES_SUCCESS) {
        /* Since plaintext keys use two reserved (by convention) slots in the keystore,
         * the slots must be invalidated to prevent its re-use without reloading
         * the key material again.
         */
        AESInvalidateKey(AES_KEY_AREA_6);
        AESInvalidateKey(AES_KEY_AREA_7);

        return AESCBC_STATUS_ERROR;
    }

    /* Disallow standby. We are about to configure and start the accelerator.
     * Setting the constraint should happen after all opportunities to fail out of the
     * function. This way, we do not need to undo it each time we exit with a failure.
     */
    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);

    AESSetInitializationVector((uint32_t *)operation->iv);

    AESSetCtrl(CRYPTO_AESCTL_CBC |
               (operationType == AESCBC_OPERATION_TYPE_ENCRYPT ? CRYPTO_AESCTL_DIR : 0));

    AESSetDataLength(operation->inputLength);
    AESSetAuthLength(0);

    AESStartDMAOperation(operation->input, operation->inputLength, operation->output, operation->inputLength);

    return AESCBC_waitForResult(handle);
}

/*
 *  ======== AESCBC_waitForAccess ========
 */
static inline int_fast16_t AESCBC_waitForAccess(AESCBC_Handle handle) {
    AESCBCCC26XX_Object *object = handle->object;

    return SemaphoreP_pend(&CryptoResourceCC26XX_accessSemaphore, object->semaphoreTimeout);
}

/*
 *  ======== AESCBC_waitForResult ========
 */
static int_fast16_t AESCBC_waitForResult(AESCBC_Handle handle){
    AESCBCCC26XX_Object *object = handle->object;

    if (object->returnBehavior == AESCBC_RETURN_BEHAVIOR_POLLING) {
        /* Wait until the operation is complete and check for DMA errors. */
        if(AESWaitForIRQFlags(AES_RESULT_RDY | AES_DMA_BUS_ERR) & AES_DMA_BUS_ERR){
            object->returnStatus = AESCBC_STATUS_ERROR;
        }

        /* Make sure to also clear DMA_IN_DONE as it is not cleared above
         * but will be set none-the-less.
         */
        AESIntClear(AES_RESULT_RDY | AES_DMA_IN_DONE | AES_DMA_BUS_ERR);

        /* Instead of posting the swi to handle cleanup, we will execute
         * the core of the function here
         */
        AESCBC_cleanup(handle);

        return object->returnStatus;
    }
    else if (object->returnBehavior == AESCBC_RETURN_BEHAVIOR_BLOCKING) {
        SemaphoreP_pend(&CryptoResourceCC26XX_operationSemaphore, SemaphoreP_WAIT_FOREVER);

        return object->returnStatus;
    }
    else {
        return AESCBC_STATUS_SUCCESS;
    }
}

/*
 *  ======== AESCBC_oneStepEncrypt ========
 */
int_fast16_t AESCBC_oneStepEncrypt(AESCBC_Handle handle, AESCBC_Operation *operationStruct) {

    return AESCBC_startOperation(handle, operationStruct, AESCBC_OPERATION_TYPE_ENCRYPT);
}

/*
 *  ======== AESCBC_oneStepDecrypt ========
 */
int_fast16_t AESCBC_oneStepDecrypt(AESCBC_Handle handle, AESCBC_Operation *operationStruct) {

    return AESCBC_startOperation(handle, operationStruct, AESCBC_OPERATION_TYPE_DECRYPT);
}
