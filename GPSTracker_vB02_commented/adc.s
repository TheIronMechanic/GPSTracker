#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

.set adc_channel, 6  //pin 34

.set adc_oversampling_factor_log, 1
.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

.bss

.global low_threshold
low_threshold: .long 0

.global high_threshold
high_threshold: .long 0

.global sample_counter
sample_counter:
.long 0

.global ADC_reading
ADC_reading:
.long 0

.text
.global entry
entry:

/* increment sample counter */
move r3, sample_counter
ld R2, r3, 0
add R2, R2, 1
st R2, r3, 0

/* do measurements using ADC */
/* r0 will be used as accumulator */
move r0, 0
/* initialize the loop counter */
stage_rst

measure:
/* measure and add value to accumulator */
adc r1, 0, adc_channel + 1
add r0, r0, r1
/* increment loop counter and check exit condition */
stage_inc 1
jumps measure, adc_oversampling_factor, lt

/* divide accumulator by adc_oversampling_factor.
   Since it is chosen as a power of two, use right shift */
rsh r0, r0, adc_oversampling_factor_log
/* averaged value is now in r0; store it into ADC_reading */
move r3, ADC_reading
st r0, r3, 0

/* compare with high_threshold; wake up if value > high_threshold */
move r3, high_threshold
ld r3, r3, 0
sub r3, r3, r0
jump wake_up, ov

/* value within range, end the program */
.global exit
exit:

halt

.global wake_up
wake_up:
/* Check if the system can be woken up */
READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
and r0, r0, 1
jump exit, eq

/* Wake up the SoC, end program */
wake
WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
jump exit
