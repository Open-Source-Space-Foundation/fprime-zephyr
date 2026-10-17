// Link stubs for usp entry points UspRadio never reaches (it uses only the
// LOCK_RADIO_ACCESS path; packet-mode RAC transactions are not used).
#include <stdint.h>
#include "smtc_rac_api.h"
void smtc_duty_cycle_sum(uint32_t freq_hz, uint32_t toa_ms) { (void)freq_hz; (void)toa_ms; }
smtc_rac_return_code_t smtc_rac_lora(uint8_t id) { (void)id; return SMTC_RAC_ERROR; }
smtc_rac_return_code_t smtc_rac_fsk(uint8_t id) { (void)id; return SMTC_RAC_ERROR; }
smtc_rac_return_code_t smtc_rac_flrc(uint8_t id) { (void)id; return SMTC_RAC_ERROR; }
smtc_rac_return_code_t smtc_rac_lrfhss(uint8_t id) { (void)id; return SMTC_RAC_ERROR; }
__attribute__((weak)) smtc_rac_return_code_t smtc_rac_flrc_burst(uint8_t id) { (void)id; return SMTC_RAC_ERROR; }
