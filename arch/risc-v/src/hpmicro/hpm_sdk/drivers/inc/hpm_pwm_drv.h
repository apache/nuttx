/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef HPM_PWM_DRV_H
#define HPM_PWM_DRV_H

#include "hpm_common.h"
#include "hpm_pwm_regs.h"
/**
 * @brief PWM driver APIs
 * @defgroup pwm_interface PWM driver APIs
 * @ingroup io_interfaces
 * @{
 *
 */
#define PWM_UNLOCK_KEY (0xB0382607UL)

/* IRQ enable bit mask */
#define PWM_IRQ_FAULT PWM_IRQEN_FAULTIRQE_MASK
#define PWM_IRQ_EX_RELOAD PWM_IRQEN_XRLDIRQE_MASK
#define PWM_IRQ_HALF_RELOAD PWM_IRQEN_HALFRLDIRQE_MASK
#define PWM_IRQ_RELOAD PWM_IRQEN_RLDIRQE_MASK
#define PWM_IRQ_CMP(x) PWM_IRQEN_CMPIRQEX_SET((1 << x))

/* PWM force output mask */
#define PWM_FORCE_OUTPUT(pwm_index, force_output) \
    (force_output << (pwm_index << 1))

#define PWM_DUTY_CYCLE_FP_MAX ((1U << 24) - 1)

/**
 * @brief pwm trigger mode
 *
 */
typedef enum pwm_counter_type {
    pwm_counter_type_capture_rising_edge,   /**< rising edge trigger*/
    pwm_counter_type_capture_falling_edge,  /**< falling edge trigger*/
} pwm_counter_type_t;

/**
 * @brief pwm cmp mode
 *
 */
typedef enum pwm_cmp_mode {
    pwm_cmp_mode_output_compare = 0,    /**< output compare*/
    pwm_cmp_mode_input_capture = 1,     /**< input compare*/
} pwm_cmp_mode_t;

/**
 * @brief update time of the shadow register
 *
 */
typedef enum pwm_register_update {
    pwm_shadow_register_update_on_shlk = 0,     /**< after software set shlk bit of shlk register*/
    pwm_shadow_register_update_on_modify = 1,   /**<  immediately after the register being modified*/
    pwm_shadow_register_update_on_hw_event = 2, /**<  after hardware event assert*/
    pwm_shadow_register_update_on_sh_synci = 3, /**< after SHSYNCI assert */
} pwm_shadow_register_update_trigger_t;

/**
 * @brief configure the state of channel 0∼7 outputs when the forced output is in effect
 *
 */
typedef enum pwm_fault_mode {
    pwm_fault_mode_force_output_0 = 0, /**< fault forced output logic 0 */
    pwm_fault_mode_force_output_1 = 1,  /**< fault forced output logic 1 */
    pwm_fault_mode_force_output_highz = 2,  /**< turn off output, pin becomes high resistance */
} pwm_fault_mode_t;

/**
 * @brief  select when to recover PWM output after fault
 *
 */
typedef enum pwm_fault_recovery_trigger {
    pwm_fault_recovery_immediately = 0, /**< immediately*/
    pwm_fault_recovery_on_reload = 1,   /**<  after pwm timer counter reload time*/
    pwm_fault_recovery_on_hw_event = 2, /**< after hardware event assert*/
    pwm_fault_recovery_on_fault_clear = 3,  /**<  after software write faultclr bit in GCR register*/
} pwm_fault_recovery_trigger_t;

/**
 * @brief fault input signal
 *
 */
typedef enum pwm_fault_source {
    pwm_fault_source_internal_0 = PWM_GCR_FAULTI0EN_MASK, /**< FAULTI0 */
    pwm_fault_source_internal_1 = PWM_GCR_FAULTI1EN_MASK, /**< FAULTI1 */
    pwm_fault_source_internal_2 = PWM_GCR_FAULTI2EN_MASK, /**< FAULTI2 */
    pwm_fault_source_internal_3 = PWM_GCR_FAULTI3EN_MASK, /**< FAULTI3 */
    pwm_fault_source_external_0 = PWM_GCR_FAULTE0EN_MASK, /**< EXFAULTI0 */
    pwm_fault_source_external_1 = PWM_GCR_FAULTE1EN_MASK, /**< EXFAULTI1 */
} pwm_fault_source_t;

/**
 * @brief Select sources for force output
 *
 */
typedef enum pwm_force_source {
    pwm_force_source_force_input = 0, /**< force output is enabled when FRCI assert */
    pwm_force_source_software = 1,  /**<  force output is enabled by software write swfrc to 1 */
} pwm_force_source_t;
/**
 * @brief select when the FRCMD shadow register will be loaded to its work register
 *
 */
typedef enum pwm_force_cmd_timing {
    pwm_force_immediately = 0,  /**< after software set shlk bit of shlk register */
    pwm_force_at_reload = 1,    /**< immediately after the register being modified */
    pwm_force_at_synci = 2,     /**< after hardware event assert */
    pwm_force_none = 3,         /**< after SHSYNCI assert */
} pwm_force_cmd_timing_t;

/**
 * @brief pwm output type
 *
 */
typedef enum pwm_output_type {
    pwm_output_0 = 0, /**< output 0 */
    pwm_output_1 = 1, /**< output 1 */
    pwm_output_high_z = 2, /**< output */
    pwm_output_no_force = 3,
} pwm_output_type_t;

/**
 * @brief pwm compare config
 *
 */
typedef struct pwm_cmp_config {
    uint32_t cmp;         /**< compare value */
    bool enable_ex_cmp;   /**< enable extended compare value */
    bool enable_hr_cmp;     /**< enable high precision pwm */
    uint8_t mode;         /**< compare work mode: pwm_cmp_mode_output_compare or pwm_cmp_mode_input_capture */
    uint8_t update_trigger;  /**< compare configuration update trigger */
    uint8_t ex_cmp;       /**< extended compare value */
    uint8_t half_clock_cmp; /**< half clock compare value*/
    uint8_t jitter_cmp;     /**< jitter compare value */
    uint8_t hr_cmp;         /**< high precision pwm */
} pwm_cmp_config_t;

/**
 * @brief pwm output channel config
 *
 */
typedef struct pwm_output_channel {
    uint8_t cmp_start_index; /**< output channel compare start index */
    uint8_t cmp_end_index;   /**< output channel compare end index */
    bool invert_output;      /**< invert output */
} pwm_output_channel_t;
/**
 * @brief pwm fault source config
 *
 */
typedef struct pwm_fault_source_config {
    uint32_t source_mask;               /**< fault source mask*/
    bool fault_recover_at_rising_edge;  /**< recover fault at rising edge */
    bool external_fault_active_low;     /**< active external fault by low */
    uint8_t fault_output_recovery_trigger; /**< fault output recoverty trigger */
} pwm_fault_source_config_t;

/**
 * @brief pwm config data
 *
 */
typedef struct pwm_config {
    bool enable_output;                 /**< enable pwm output */
    bool invert_output;                 /**< invert pwm output level */
    uint8_t update_trigger;             /**< pwm config update trigger */
    uint8_t fault_mode;                 /**< fault mode */
    uint8_t fault_recovery_trigger;     /**< fault recoverty trigger */
    uint8_t force_source;               /**< fault source */
    uint32_t dead_zone_in_half_cycle;   /**< dead zone in half cycle*/
} pwm_config_t;

/**
 * @brief pair pwm config
 *
 */
typedef struct pwm_pair_config {
    pwm_config_t pwm[2];    /**< pwm config data */
} pwm_pair_config_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief issue all shawdow register
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 */
static inline void pwm_issue_shadow_register_lock_event(PWM_Type *pwm_x)
{
    if (pwm_x->SHCR & PWM_SHCR_SHLKEN_MASK) {
        /*
         * if lock shadow register has been enabled in SHCR, it has to set
         * the lock bit twice to issue shadow register lock event.
         */
        pwm_x->SHLK = PWM_SHLK_SHLK_MASK;
    }
    pwm_x->SHLK = PWM_SHLK_SHLK_MASK;
}

/**
 * @brief lock all shawdow register
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 */
static inline void pwm_shadow_register_lock(PWM_Type *pwm_x)
{
    pwm_x->SHCR |= PWM_SHCR_SHLKEN_MASK;
    pwm_x->SHLK = PWM_SHLK_SHLK_MASK;
}

/**
 * @brief unlock all shadow register
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 */
static inline void pwm_shadow_register_unlock(PWM_Type *pwm_x)
{
    pwm_x->UNLK = PWM_UNLOCK_KEY;
}

/**
 * @brief set counter start value and extended start value
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] ex_start pwm timer counter extended start value
 * @param[in] start pwm timer counter start value
 */
static inline void pwm_set_start_count(PWM_Type *pwm_x,
                                        uint8_t ex_start,
                                        uint32_t start)
{
    pwm_x->STA = PWM_STA_XSTA_SET(ex_start)
        | PWM_STA_STA_SET(start);
}

/**
 * @brief set the reload value
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] ex_reload pwm timer counter extended reload value
 * @param[in] reload pwm timer counter reload value
 */
static inline void pwm_set_reload(PWM_Type *pwm_x,
                                   uint8_t ex_reload,
                                   uint32_t reload)
{
    pwm_shadow_register_unlock(pwm_x);
    pwm_x->RLD = PWM_RLD_XRLD_SET(ex_reload)
        | PWM_RLD_RLD_SET(reload);
}

/**
 * @brief clear pwm status register
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] mask :
 *  @arg        PWM_IRQ_FAULT: fault condition flag
 *  @arg        PWM_IRQ_EX_RELOAD : extended reload flag, this flag set when xcnt count to xrld value or when SYNCI assert
 *  @arg        PWM_IRQ_HALF_RELOAD: half reload flag, this flag set when cnt count to rld/2
 *  @arg        PWM_IRQ_RELOAD: reload flag, this flag set when cnt count to rld value or when SYNCI assert
 *  @arg        PWM_IRQ_CMP(x)(x=0...n): comparator output compare or input capture flag
 */
static inline void pwm_clear_status(PWM_Type *pwm_x, uint32_t mask)
{
    pwm_x->SR |= mask;
}

/**
 * @brief get pwm status register
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @retval uint32_t SR register value
 */
static inline uint32_t pwm_get_status(PWM_Type *pwm_x)
{
    return pwm_x->SR;
}

/**
 * @brief disable pwm irq
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] mask :
 *  @arg        PWM_IRQ_FAULT: fault condition interrupt enable
 *  @arg        PWM_IRQ_EX_RELOAD: extended reload flag interrupt enable
 *  @arg        PWM_IRQ_HALF_RELOAD: half reload flag interrupt enable
 *  @arg        PWM_IRQ_RELOAD: reload flag interrupt enable
 *  @arg        PWM_IRQ_CMP(x)(x=0...n): comparator output compare or input capture flag interrupt enable
 */
static inline void pwm_disable_irq(PWM_Type *pwm_x, uint32_t mask)
{
    pwm_x->IRQEN &= ~mask;
}

/**
 * @brief enable pwm irq
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] mask :
 *  @arg        PWM_IRQ_FAULT: fault condition interrupt enable
 *  @arg        PWM_IRQ_EX_RELOAD: extended reload flag interrupt enable
 *  @arg        PWM_IRQ_HALF_RELOAD: half reload flag interrupt enable
 *  @arg        PWM_IRQ_RELOAD: reload flag interrupt enable
 *  @arg        PWM_IRQ_CMP(x)(x=0...n): comparator output compare or input capture flag interrupt enable
 */
static inline void pwm_enable_irq(PWM_Type *pwm_x, uint32_t mask)
{
    pwm_x->IRQEN |= mask;
}

/**
 * @brief disable pwm dma request
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] mask :
 *  @arg        PWM_IRQ_FAULT: fault condition DMA request enable
 *  @arg        PWM_IRQ_EX_RELOAD: extended reload flag DMA request enable
 *  @arg        PWM_IRQ_HALF_RELOAD: half reload flag DMA request enable
 *  @arg        PWM_IRQ_RELOAD: reload flag DMA request enable
 *  @arg        PWM_IRQ_CMP(x)(x=0...n): comparator output compare or input capture flag DMA request enable
 *
 */
static inline void pwm_disable_dma_request(PWM_Type *pwm_x, uint32_t mask)
{
    pwm_x->DMAEN &= ~mask;
}

/**
 * @brief enable pwm dma request
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] mask :
 *  @arg        PWM_IRQ_FAULT: fault condition DMA request enable
 *  @arg        PWM_IRQ_EX_RELOAD: extended reload flag DMA request enable
 *  @arg        PWM_IRQ_HALF_RELOAD: half reload flag DMA request enable
 *  @arg        PWM_IRQ_RELOAD: reload flag DMA request enable
 *  @arg        PWM_IRQ_CMP(x)(x=0...n): comparator output compare or input capture flag DMA request enable
 *
 */
static inline void pwm_enable_dma_request(PWM_Type *pwm_x, uint32_t mask)
{
    pwm_x->DMAEN |= mask;
}

/**
 * @brief set target cmp as hardware event to trigger force cmd output
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] target_cmp_index cmp index  select one of the cmp as hardware event time to load FRCMD shadow registers [0-23]
 */
static inline void pwm_set_force_cmd_shadow_register_hwevent(PWM_Type *pwm_x,
                                                    uint8_t target_cmp_index)
{
    pwm_x->SHCR = ((pwm_x->SHCR & ~(PWM_SHCR_FRCSHDWSEL_MASK))
                    | PWM_SHCR_FRCSHDWSEL_SET(target_cmp_index));
}
/**
 * @note if trigger is not set to hardware event, target_cmp_index can be
 * passed with any value
 *
 */

/**
 * @brief set shadow register control register
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] trigger  select when the counter related shadow registers @ref pwm_shadow_register_update_trigger_t
 * @param[in] target_cmp_index  select one of the comparators as hardware event (0..(PWM_SOC_CMP_MAX_COUNT-1))
 */
static inline void pwm_set_load_counter_shadow_register_trigger(PWM_Type *pwm_x,
                                pwm_shadow_register_update_trigger_t trigger,
                                uint8_t target_cmp_index)
{
    if (trigger == pwm_shadow_register_update_on_hw_event) {
        pwm_x->SHCR = ((pwm_x->SHCR & ~(PWM_SHCR_CNTSHDWSEL_MASK
                        | PWM_SHCR_CNTSHDWUPT_MASK))
                | PWM_SHCR_CNTSHDWSEL_SET(target_cmp_index)
                | PWM_SHCR_CNTSHDWUPT_SET(trigger));
    } else {
        pwm_x->SHCR = ((pwm_x->SHCR & ~(PWM_SHCR_CNTSHDWUPT_MASK))
                | PWM_SHCR_CNTSHDWUPT_SET(trigger));
    }
}

/**
 * @brief configure the cmp shadow on capture mode
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 * @param[in] edge  which edge is used as shadow register hardware load event
 *  @arg 1- falling edge
 *  @arg 0- rising edge
 */
static inline void pwm_load_cmp_shadow_on_capture(PWM_Type *pwm_x,
                                                   uint8_t index,
                                                   bool edge)
{
    pwm_x->CMPCFG[index] |= PWM_CMPCFG_CMPMODE_MASK;
    pwm_x->GCR = ((pwm_x->GCR & ~(PWM_GCR_CMPSHDWSEL_MASK | PWM_GCR_HWSHDWEDG_MASK))
            | PWM_GCR_CMPSHDWSEL_SET(index)
            | PWM_GCR_HWSHDWEDG_SET(edge));
}

/**
 * @brief disable pwn cmp half clock
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 */
static inline void pwm_cmp_disable_half_clock(PWM_Type *pwm_x, uint8_t index)
{
    pwm_x->CMP[index] &= ~PWM_CMP_CMPHLF_MASK;
}

/**
 * @brief enable pwm cmp half clock
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 */
static inline void pwm_cmp_enable_half_clock(PWM_Type *pwm_x, uint8_t index)
{
    pwm_x->CMP[index] |= PWM_CMP_CMPHLF_MASK;
}

/**
 * @brief update pwm cmp jitter counter compare value
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 * @param[in] jitter jitter value
 */
static inline void pwm_cmp_update_jitter_value(PWM_Type *pwm_x, uint8_t index, uint8_t jitter)
{
    pwm_x->CMP[index] = (pwm_x->CMP[index] & ~PWM_CMP_CMPJIT_MASK) | PWM_CMP_CMPJIT_SET(jitter);
}

/**
 * @brief update pwm cmp value
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 * @param[in] cmp clock counter compare value
 * @param[in] ex_cmp extended counter compare value
 */
static inline void pwm_cmp_update_cmp_value(PWM_Type *pwm_x, uint8_t index,
                                            uint32_t cmp, uint16_t ex_cmp)
{
    pwm_x->CMP[index] = (pwm_x->CMP[index] & ~(PWM_CMP_CMP_MASK | PWM_CMP_XCMP_MASK))
        | PWM_CMP_CMP_SET(cmp) | PWM_CMP_XCMP_SET(ex_cmp);
}

/**
 * @brief update high-precision cmp value
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 * @param[in] cmp clock counter compare value
 * @param[in] hr_cmp high-precision pwm
 */
static inline void pwm_cmp_update_hr_cmp_value(PWM_Type *pwm_x, uint8_t index,
                                            uint32_t cmp, uint16_t hr_cmp)
{
    pwm_x->CMP[index] = (pwm_x->CMP[index] & ~(PWM_CMP_HRPWM_CMP_MASK | PWM_CMP_HRPWM_CMP_HR_MASK))
        | PWM_CMP_HRPWM_CMP_SET(cmp) | PWM_CMP_HRPWM_CMP_HR_SET(hr_cmp);
}

/**
 * @brief Forced update of pwm cmp register value, cmp content guaranteed accurate by user
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 * @param[in] cmp cmp register data
 */
static inline void pwm_cmp_force_value(PWM_Type *pwm_x, uint8_t index, uint32_t cmp)
{
    pwm_x->CMP[index] = cmp;
}

/**
 * @brief config pwm cmp
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 * @param[in] config @ref pwm_cmp_config_t
 */
static inline void pwm_config_cmp(PWM_Type *pwm_x, uint8_t index, pwm_cmp_config_t *config)
{
    pwm_shadow_register_unlock(pwm_x);
    if (config->mode == pwm_cmp_mode_output_compare) {
        if (config->enable_hr_cmp) {
            pwm_x->CMPCFG[index] = PWM_CMPCFG_CMPSHDWUPT_SET(config->update_trigger);
            pwm_x->CMP[index] = PWM_CMP_HRPWM_CMP_SET(config->cmp)
                        | PWM_CMP_HRPWM_CMP_HR_SET(config->hr_cmp);
        } else {
            pwm_x->CMPCFG[index] = PWM_CMPCFG_XCNTCMPEN_SET(config->enable_ex_cmp)
                        | PWM_CMPCFG_CMPSHDWUPT_SET(config->update_trigger);
            pwm_x->CMP[index] = PWM_CMP_CMP_SET(config->cmp)
                        | PWM_CMP_XCMP_SET(config->ex_cmp)
                        | PWM_CMP_CMPHLF_SET(config->half_clock_cmp)
                        | PWM_CMP_CMPJIT_SET(config->jitter_cmp);
        }
    } else {
        pwm_x->CMPCFG[index] |= PWM_CMPCFG_CMPMODE_MASK;
    }
}

/**
 * @brief config pwm output channel
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index channel index (0..(PWM_SOC_PWM_MAX_COUNT-1))
 * @param[in] config @ref pwm_output_channel_t
 */
static inline void pwm_config_output_channel(PWM_Type *pwm_x, uint8_t index, pwm_output_channel_t *config)
{
    pwm_x->CHCFG[index] = PWM_CHCFG_CMPSELBEG_SET(config->cmp_start_index)
                            | PWM_CHCFG_CMPSELEND_SET(config->cmp_end_index)
                            | PWM_CHCFG_OUTPOL_SET(config->invert_output);
}

/**
 * @brief config pwm fault source
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] config @ref pwm_fault_source_config_t
 */
static inline void pwm_config_fault_source(PWM_Type *pwm_x, pwm_fault_source_config_t *config)
{
    pwm_x->GCR = (pwm_x->GCR & ~(PWM_GCR_FAULTI0EN_MASK | PWM_GCR_FAULTI1EN_MASK
                | PWM_GCR_FAULTI2EN_MASK | PWM_GCR_FAULTI3EN_MASK
                | PWM_GCR_FAULTE0EN_MASK | PWM_GCR_FAULTE1EN_MASK
                | PWM_GCR_FAULTRECEDG_MASK | PWM_GCR_FAULTEXPOL_MASK
                | PWM_GCR_FAULTRECHWSEL_MASK))
        | config->source_mask
        | PWM_GCR_FAULTEXPOL_SET(config->external_fault_active_low)
        | PWM_GCR_FAULTRECEDG_SET(config->fault_recover_at_rising_edge)
        | PWM_GCR_FAULTRECHWSEL_SET(config->fault_output_recovery_trigger);
}

/**
 * @brief clear pwm fault status
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 */
static inline void pwm_clear_fault(PWM_Type *pwm_x)
{
    pwm_x->GCR |= PWM_GCR_FAULTCLR_MASK;
}

/**
 * @brief stop the pwm timer counter
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 */
static inline void pwm_stop_counter(PWM_Type *pwm_x)
{
    pwm_x->GCR &= ~PWM_GCR_CEN_MASK;
}

/**
 * @brief start pwm timer counter
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 */
static inline void pwm_start_counter(PWM_Type *pwm_x)
{
    pwm_x->GCR |= PWM_GCR_CEN_MASK;
}

/**
 * @brief enable software force
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 */
static inline void pwm_enable_sw_force(PWM_Type *pwm_x)
{
    pwm_x->GCR |= PWM_GCR_SWFRC_MASK;
}

/**
 * @brief disable software force , force will take effect
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 */
static inline void pwm_disable_sw_force(PWM_Type *pwm_x)
{
    pwm_x->GCR &= ~PWM_GCR_SWFRC_MASK;
}

/**
 * @brief enable pwm reload value by synci
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 */
static inline void pwm_enable_reload_at_synci(PWM_Type *pwm_x)
{
    pwm_x->GCR |= PWM_GCR_XRLDSYNCEN_MASK | PWM_GCR_RLDSYNCEN_MASK;
}

/**
 * @brief disable pwm output
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index pwm index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 */
static inline void pwm_disable_output(PWM_Type *pwm_x, uint8_t index)
{
    pwm_x->PWMCFG[index] &= ~PWM_PWMCFG_OEN_MASK;
}

/**
 * @brief enable pwm output
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index pwm index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 */
static inline void pwm_enable_output(PWM_Type *pwm_x, uint8_t index)
{
    pwm_x->PWMCFG[index] |= PWM_PWMCFG_OEN_MASK;
}

/**
 * @brief config pwm force output level per output channel
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] output_mask PWM output channel force level, set it using the macro
 *  @arg PWM_FORCE_OUTPUT(pwm_index, force_output)
 */
static inline void pwm_set_force_output(PWM_Type *pwm_x, uint32_t output_mask)
{
    pwm_x->FRCMD = PWM_FRCMD_FRCMD_SET(output_mask);
}

/**
 * @brief config pwm force polarity
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] polarity polarity of input pwm_force
 *  @arg 1- active low
 *  @arg 0- active high
 */
static inline void pwm_config_force_polarity(PWM_Type *pwm_x, bool polarity)
{
    pwm_x->GCR = (pwm_x->GCR & ~(PWM_GCR_FRCPOL_MASK)) | PWM_GCR_FRCPOL_SET(polarity);
}

/**
 * @brief config the force effective time
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] timing @ref pwm_force_cmd_timing_t
 */
static inline void pwm_config_force_cmd_timing(PWM_Type *pwm_x, pwm_force_cmd_timing_t timing)
{
    pwm_x->GCR = (pwm_x->GCR & ~(PWM_GCR_FRCTIME_MASK)) | PWM_GCR_FRCTIME_SET(timing);
}

/**
 * @brief enable pwm sw force output
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index pwm cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 */
static inline void pwm_enable_pwm_sw_force_output(PWM_Type *pwm_x, uint8_t index)
{
    pwm_x->PWMCFG[index] |= PWM_PWMCFG_OEN_MASK | PWM_PWMCFG_FRCSRCSEL_MASK
        | PWM_PWMCFG_FRCSHDWUPT_SET(pwm_shadow_register_update_on_modify);
}

/**
 * @brief disable pwm sw force output
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index pwm cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 */
static inline void pwm_disable_pwm_sw_force_output(PWM_Type *pwm_x, uint8_t index)
{
    pwm_x->PWMCFG[index] &= ~PWM_PWMCFG_FRCSRCSEL_MASK;
}

/**
 * @brief config PWM channel configure registe
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index pwm channel index (0..(PWM_SOC_PWM_MAX_COUNT-1))
 * @param[in] config @ref pwm_config_t
 * @param[in] enable_pair_mode enable pair mode
 *  @arg 1- PWM output is in pair mode
 *  @arg 0- PWM output is in indepandent mode
 */
static inline void pwm_config_pwm(PWM_Type *pwm_x, uint8_t index,
                                  pwm_config_t *config, bool enable_pair_mode)
{
    pwm_x->PWMCFG[index] = PWM_PWMCFG_OEN_SET(config->enable_output)
        | PWM_PWMCFG_FRCSHDWUPT_SET(config->update_trigger)
        | PWM_PWMCFG_FAULTMODE_SET(config->fault_mode)
        | PWM_PWMCFG_FAULTRECTIME_SET(config->fault_recovery_trigger)
        | PWM_PWMCFG_FRCSRCSEL_SET(config->force_source)
        | PWM_PWMCFG_PAIR_SET(enable_pair_mode)
        | PWM_PWMCFG_DEADAREA_SET(config->dead_zone_in_half_cycle);
}

/**
 * @brief pwm load cmp shadow on match
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] index cmp index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 * @param[in] config @ref pwm_cmp_config_t
 * @retval status_invalid_argument or status_success
 */
hpm_stat_t pwm_load_cmp_shadow_on_match(PWM_Type *pwm_x,
                                         uint8_t index,
                                         pwm_cmp_config_t *config);

/**
 * @brief pwm get captured count
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[out] buf count value
 * @param[in] counter @ref pwm_counter_type_t
 * @param[in] start_index start capture index (0..(PWM_SOC_CMP_MAX_COUNT-1))
 * @param[in] num capture num (1..PWM_SOC_CMP_MAX_COUNT)
 */
void pwm_get_captured_count(PWM_Type *pwm_x, uint32_t *buf, pwm_counter_type_t counter, uint8_t start_index, uint8_t num);

/**
 * @brief get default cmp config
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[out] config @ref pwm_cmp_config_t
 */
void pwm_get_default_cmp_config(PWM_Type *pwm_x, pwm_cmp_config_t *config);

/**
 * @brief get default output channel config
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[out] config @ref pwm_output_channel_t
 */
void pwm_get_default_output_channel_config(PWM_Type *pwm_x, pwm_output_channel_t *config);

/**
 * @brief get default pwm config
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[out] config @ref pwm_config_t
 */
void pwm_get_default_pwm_config(PWM_Type *pwm_x, pwm_config_t *config);

/**
 * @brief get default pwm pair config
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[out] config @ref pwm_pair_config_t
 */
void pwm_get_default_pwm_pair_config(PWM_Type *pwm_x, pwm_pair_config_t *config);

/**
 * @brief setup waveform
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] pwm_index pwm channel index (0..(PWM_SOC_PWM_MAX_COUNT-1))
 * @param[in] pwm_config @ref pwm_config_t
 * @param[in] cmp_start_index pwm cmp index (0..(PWM_SOC_PWM_MAX_COUNT-1))
 * @param[in] cmp @ref pwm_cmp_config_t
 * @param[in] cmp_num cmp num (1..PWM_SOC_CMP_MAX_COUNT), cmp[cmp_num-1] must not overflow
 * @retval hpm_stat_t
 */
hpm_stat_t pwm_setup_waveform(PWM_Type *pwm_x,
                        uint8_t pwm_index, pwm_config_t *pwm_config,
                        uint8_t cmp_start_index, pwm_cmp_config_t *cmp, uint8_t cmp_num);
/**
 * @brief setup pwm waveform in pair
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] pwm_index pwm channel index (0..(PWM_SOC_PWM_MAX_COUNT-1))
 * @param[in] pwm_pair_config @ref pwm_pair_config_t
 * @param[in] cmp_start_index pwm cmp index (0..(PWM_SOC_PWM_MAX_COUNT-1))
 * @param[in] cmp @ref pwm_cmp_config_t
 * @param[in] cmp_num cmp num (1..PWM_SOC_CMP_MAX_COUNT), cmp[cmp_num-1] must not overflow
 * @retval hpm_stat_t @ref status_invalid_argument or @ref status_success
 */
hpm_stat_t pwm_setup_waveform_in_pair(PWM_Type *pwm_x,
                        uint8_t pwm_index, pwm_pair_config_t *pwm_pair_config,
                        uint8_t cmp_start_index, pwm_cmp_config_t *cmp, uint8_t cmp_num);

/**
 * @brief update raw compare value for edge aligned waveform
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] cmp_index index of cmp to be adjusted (0..(PWM_SOC_PWM_MAX_COUNT-1))
 * @param[in] target_cmp target compare value
 * @retval hpm_stat_t @ref status_invalid_argument or @ref status_success
 */
hpm_stat_t pwm_update_raw_cmp_edge_aligned(PWM_Type *pwm_x, uint8_t cmp_index,
                                uint32_t target_cmp);

/**
 * @brief update raw compare value for central aligned waveform
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] cmp1_index index of cmp1 to be adjusted (cmp1_index must be even number)
 * @param[in] cmp2_index index of cmp2 to be adjusted (cmp2_index must be odd number)
 * @param[in] target_cmp1 target compare value for cmp1
 * @param[in] target_cmp2 target compare value for cmp2
 * @retval hpm_stat_t @ref status_invalid_argument or @ref status_success cmp1_index
 */
hpm_stat_t pwm_update_raw_cmp_central_aligned(PWM_Type *pwm_x, uint8_t cmp1_index,
                                       uint8_t cmp2_index, uint32_t target_cmp1, uint32_t target_cmp2);

/**
 * @brief Enable high-precision pwm
 *
 * @param pwm_x @ref PWM_Type PWM base address
 */
static inline void pwm_enable_hr_pwm(PWM_Type *pwm_x)
{
    pwm_x->GCR = (pwm_x->GCR & ~(PWM_GCR_HR_PWM_EN_MASK)) | PWM_GCR_HR_PWM_EN_SET(1);
}

/**
 * @brief Disable high-precision pwm
 *
 * @param pwm_x @ref PWM_Type PWM base address
 */
static inline void pwm_disable_hr_pwm(PWM_Type *pwm_x)
{
    pwm_x->GCR = pwm_x->GCR & ~(PWM_GCR_HR_PWM_EN_MASK);
}

/**
 * @brief update raw high-precision compare value for edge aligned waveform
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] cmp_index index of cmp to be adjusted (0..(PWM_SOC_PWM_MAX_COUNT-1))
 * @param[in] target_cmp target compare value
 * @param[in] target_hr_cmp target high-precision compare value
 * @return hpm_stat_t
 */
hpm_stat_t pwm_update_raw_hr_cmp_edge_aligned(PWM_Type *pwm_x, uint8_t cmp_index, uint32_t target_cmp,
            uint16_t target_hr_cmp);

/**
 * @brief update raw high-precision compare value for central aligned waveform
 *
 * @param[in] pwm_x PWM base address, HPM_PWMx(x=0..n)
 * @param[in] cmp1_index index of cmp1 to be adjusted (cmp1_index must be even number)
 * @param[in] cmp2_index index of cmp2 to be adjusted (cmp2_index must be odd number)
 * @param[in] target_cmp1 target compare value for cmp1
 * @param[in] target_cmp2 target compare value for cmp2
 * @param[in] target_hr_cmp1 target high-precision compare value for cmp1
 * @param[in] target_hr_cmp2 target high-precision compare value for cmp2
 * @return hpm_stat_t
 */
hpm_stat_t pwm_update_raw_hr_cmp_central_aligned(PWM_Type *pwm_x, uint8_t cmp1_index,
                                       uint8_t cmp2_index, uint32_t target_cmp1, uint32_t target_cmp2,
                                        uint16_t target_hr_cmp1, uint16_t target_hr_cmp2);

#ifdef __cplusplus
}
#endif
/**
 * @}
 */
#endif /* HPM_PWM_DRV_H */
