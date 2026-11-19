#include "volt_monitor.h"
#include <string.h>

static void rail_reset(VmRail *r) { memset(r, 0, sizeof *r); r->state = VOLT_STATE_NORMAL; }

static void rail_update(VmRail *r, float v, float warn_v, float crit_v)
{
    /* Separate counters: "critical" must be CONSECUTIVE samples below crit_v. */
    if (v < crit_v)      { if (r->crit_cnt < 1000) r->crit_cnt++; if (r->warn_cnt < 1000) r->warn_cnt++; }
    else if (v < warn_v) { r->crit_cnt = 0;                       if (r->warn_cnt < 1000) r->warn_cnt++; }
    else                 { r->crit_cnt = 0; r->warn_cnt = 0; }

    if (r->state == VOLT_STATE_CRITICAL) {
        if (v >= warn_v) {
            if (++r->rec_cnt >= VM_RECOVER_SAMPLES) rail_reset(r);
        } else r->rec_cnt = 0;
    } else if (r->crit_cnt >= VM_CRIT_SAMPLES) { r->state = VOLT_STATE_CRITICAL; r->rec_cnt = 0; }
    else if (r->warn_cnt >= VM_WARN_SAMPLES)   { r->state = VOLT_STATE_WARN; }
    else if (v >= warn_v)                      { r->state = VOLT_STATE_NORMAL; }
}

void volt_monitor_init(VoltMonitor *m, uint32_t now_ms)
{
    memset(m, 0, sizeof *m);
    m->start_ms = now_ms; m->vdda = 3.3f;
    rail_reset(&m->r6); rail_reset(&m->bec);
}

void volt_monitor_update(VoltMonitor *m, uint32_t now_ms,
                         uint32_t raw_6v, uint32_t raw_bec, uint32_t raw_vref)
{
    m->vdda = raw_vref ? (VM_VREFINT_V * 4095.0f) / (float)raw_vref : 0.0f;

    if (m->vdda < VM_VDDA_FAIL_V) {
        m->vdda_rec_cnt = 0;
        if (++m->vdda_fail_cnt >= 5) m->vdda_fail = true;
    } else {
        m->vdda_fail_cnt = 0;
        if (m->vdda_fail && ++m->vdda_rec_cnt >= 10) {
            m->vdda_fail = false; rail_reset(&m->r6); rail_reset(&m->bec);
        }
    }
    if (m->vdda_fail) {
        m->r6.state = m->bec.state = VOLT_STATE_VDDA_FAIL;
        return;
    }
    if (m->vdda < VM_VDDA_FAIL_V) return;            /* hold state during short Vdda glitch */

    m->v6   = (float)raw_6v  * m->vdda / 4095.0f * VM_GAIN_6V  * VM_TRIM_6V;
    m->vbec = (float)raw_bec * m->vdda / 4095.0f * VM_GAIN_BEC * VM_TRIM_BEC;

    if ((uint32_t)(now_ms - m->start_ms) < VM_STARTUP_GRACE_MS) {   /* inrush / power-up */
        rail_reset(&m->r6); rail_reset(&m->bec);
        return;
    }
    rail_update(&m->r6,  m->v6,   VM_6V_WARN_V,  VM_6V_CRIT_V);
    rail_update(&m->bec, m->vbec, VM_BEC_WARN_V, VM_BEC_CRIT_V);
}

VoltState volt_monitor_6v(const VoltMonitor *m)  { return m->r6.state; }
VoltState volt_monitor_bec(const VoltMonitor *m) { return m->bec.state; }
bool volt_monitor_critical(const VoltMonitor *m)
{
    return m->r6.state == VOLT_STATE_CRITICAL || m->bec.state == VOLT_STATE_CRITICAL ||
           m->r6.state == VOLT_STATE_VDDA_FAIL;
}
