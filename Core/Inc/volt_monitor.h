#ifndef VOLT_MONITOR_H
#define VOLT_MONITOR_H
/* Pure-C voltage state machine (no HAL). Call volt_monitor_update() every 100 ms
 * with averaged raw ADC counts. Uses VoltState from app.h. */
#include <stdint.h>
#include <stdbool.h>
#include "app.h"

/* ---- Tunables (trim these once a multimeter is available) ---- */
#define VM_VREFINT_V        1.20f    /* real part is 1.16..1.24 V */
#define VM_VDDA_FAIL_V      3.00f
#define VM_GAIN_6V          2.000f   /* divider ratio, CH1 */
#define VM_GAIN_BEC         1.833f   /* divider ratio, CH5 */
#define VM_TRIM_6V          1.000f   /* multiply to match a meter reading */
#define VM_TRIM_BEC         1.000f
#define VM_STARTUP_GRACE_MS 3000u    /* ignore rails right after boot */
#define VM_BEC_WARN_V       4.8f
#define VM_BEC_CRIT_V       4.6f
#define VM_6V_WARN_V        5.0f
#define VM_6V_CRIT_V        4.8f
#define VM_WARN_SAMPLES     5        /* x100 ms */
#define VM_CRIT_SAMPLES     10       /* consecutive samples below crit */
#define VM_RECOVER_SAMPLES  10

typedef struct { uint16_t warn_cnt, crit_cnt, rec_cnt; VoltState state; } VmRail;

typedef struct {
    uint32_t start_ms;
    VmRail   r6, bec;
    uint16_t vdda_fail_cnt, vdda_rec_cnt;
    bool     vdda_fail;
    float    vdda, v6, vbec;          /* last computed values (watch these in the debugger) */
} VoltMonitor;

void volt_monitor_init(VoltMonitor *m, uint32_t now_ms);
void volt_monitor_update(VoltMonitor *m, uint32_t now_ms,
                         uint32_t raw_6v, uint32_t raw_bec, uint32_t raw_vref);
VoltState volt_monitor_6v(const VoltMonitor *m);
VoltState volt_monitor_bec(const VoltMonitor *m);
bool volt_monitor_critical(const VoltMonitor *m);
#endif
