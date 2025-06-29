#include "defaults.h"
#include <string.h>

void ths8200_set_defaults(ths8200_regs_t *r)
{
    if (!r) {
        return;
    }

    memset(r, 0, sizeof(*r));

    /*
     * Populate registers with the non-zero power-on defaults from the
     * THS8200 datasheet. Fields not listed are left cleared.
     */

    /* System control: software reset bit defaults high */
    r->system.ctl.arst_func_n = true;

    /* Data path control: default input format selector */
    r->datapath.format = 0x03; /* data_dman_cntl = 011b */

    /* Color Space Conversion defaults (Q2.8 coefficients) */
    r->csc.r2r_int  = 0x00; r->csc.r2r_frac  = 0xDA;
    r->csc.r2g_int  = (int8_t)0x80; r->csc.r2g_frac  = 0x78;
    r->csc.r2b_int  = 0x02; r->csc.r2b_frac  = 0x0C;
    r->csc.g2r_int  = 0x02; r->csc.g2r_frac  = 0xDC;
    r->csc.g2g_int  = (int8_t)0x81; r->csc.g2g_frac  = 0x94;
    r->csc.g2b_int  = (int8_t)0x81; r->csc.g2b_frac  = 0xDC;
    r->csc.b2r_int  = 0x00; r->csc.b2r_frac  = 0x4A;
    r->csc.b2g_int  = 0x02; r->csc.b2g_frac  = 0x0C;
    r->csc.b2b_int  = (int8_t)0x80; r->csc.b2b_frac  = 0x30;
    r->csc.yoff_int = 0x00; r->csc.yoff_frac = 0x08;
    r->csc.cboff_int= 0x02; r->csc.cboff_frac= 0x00;

    r->csc.csc_bypass = true;

    /* Display Timing Generator defaults */
    r->dtg1.y_blank      = 0x200;
    r->dtg1.y_sync_hi    = 0x300;
    r->dtg1.cbcr_blank   = 0x200;
    r->dtg1.cbcr_sync_hi = 0x300;
    r->dtg1.cbar_size    = 0x80;

    r->dtg2.hlength   = 0x60;
    r->dtg2.hdly      = 0x20;
    r->dtg2.vlength1  = 0x03;
    r->dtg2.vdly2     = 0x3FF;
    r->dtg2.hsind     = 0x3D;
    r->dtg2.vsind     = 0x03;
    r->dtg2.ctrl.rgb_mode  = true;
    r->dtg2.ctrl.vs_out    = true;
    r->dtg2.ctrl.hs_out    = true;
    r->dtg2.ctrl.fid_pol   = true;
    r->dtg2.ctrl.vs_in     = true;
    r->dtg2.ctrl.hs_in     = true;
}
