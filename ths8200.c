#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "ths8200.h"

struct device { int x; };

int i2c_burst_write(const struct device *dev, uint8_t addr,
                    uint8_t reg, const uint8_t *buf, uint16_t size);
int i2c_burst_read(const struct device *dev, uint8_t addr,
                   uint8_t reg, uint8_t *buf, uint16_t size);

#define THS8200_REG_COUNT  (0x89 + 1)

/**
 * @brief Read all THS8200 registers into ths8200_regs_t.
 */
static inline int ths8200_read_regs(const struct device *dev,
                                    uint8_t addr,
                                    ths8200_regs_t *r)
{
    uint8_t buf[THS8200_REG_COUNT];
    int rc = i2c_burst_read(dev, addr, 0x00, buf, sizeof(buf));
    if (rc) return rc;
    uint8_t t;

    /* System */
    r->system.version = buf[0x02];
    t = buf[0x03];
    r->system.ctl.vesa_clk    = (t >> 7) & 1;
    r->system.ctl.dll_bypass  = (t >> 6) & 1;
    r->system.ctl.dac_pwdn    = (t >> 5) & 1;
    r->system.ctl.chip_pwdn   = (t >> 4) & 1;
    r->system.ctl.chip_msbars = (t >> 3) & 1;
    r->system.ctl.sel_func_n  = (t >> 2) & 1;
    r->system.ctl.arst_func_n = (t >> 1) & 1;
    
    /* CSC Q2.8 coefficients & offsets */
    #define RD_COEF(msb, lsb, i_part, f_part) \
        r->csc.i_part  = (int8_t)buf[msb];   \
        r->csc.f_part  = buf[lsb]
    RD_COEF(0x04,0x05, r2r_int, r2r_frac);
    RD_COEF(0x06,0x07, r2g_int, r2g_frac);
    RD_COEF(0x08,0x09, r2b_int, r2b_frac);
    RD_COEF(0x0A,0x0B, g2r_int, g2r_frac);
    RD_COEF(0x0C,0x0D, g2g_int, g2g_frac);
    RD_COEF(0x0E,0x0F, g2b_int, g2b_frac);
    RD_COEF(0x10,0x11, b2r_int, b2r_frac);
    RD_COEF(0x12,0x13, b2g_int, b2g_frac);
    RD_COEF(0x14,0x15, b2b_int, b2b_frac);
    RD_COEF(0x16,0x17, yoff_int, yoff_frac);
    RD_COEF(0x18,0x19, cboff_int, cboff_frac);
    #undef RD_COEF
    t = buf[0x19];
    r->csc.csc_bypass = (t >> 1) & 1;
    r->csc.csc_uof    =  t       & 1;

    /* Test Control */
    t = buf[0x1A];
    r->test.digbypass = (t >> 7) & 1;
    r->test.force_off = (t >> 6) & 1;
    t = buf[0x1B];
    r->test.ydelay    = (t >> 6) & 0x03;
    r->test.fastramp  = (t >> 1) & 1;
    r->test.slowramp  =  t       & 1;

    /* Data Path */
    t = buf[0x1C];
    r->datapath.clk656 = (t >> 7) & 1;
    r->datapath.fsadj  = (t >> 6) & 1;
    r->datapath.ifir12 = (t >> 5) & 1;
    r->datapath.ifir35 = (t >> 4) & 1;
    r->datapath.tri656 = (t >> 3) & 1;
    r->datapath.format =  t       & 0x07;

    /* DTG1 Part1 */
    #define RD_SPLIT(lsb, msb_reg, msb_shift, field) \
        r->dtg1.field = ((((buf[msb_reg] >> msb_shift)&0x03)<<8) | buf[lsb])
    RD_SPLIT(0x1D,0x23,4,y_blank);
    RD_SPLIT(0x1E,0x23,2,y_sync_lo);
    RD_SPLIT(0x1F,0x23,0,y_sync_hi);
    RD_SPLIT(0x20,0x24,4,cbcr_blank);
    RD_SPLIT(0x21,0x24,2,cbcr_sync_lo);
    RD_SPLIT(0x22,0x24,0,cbcr_sync_hi);
    #undef RD_SPLIT
    t = buf[0x38];
    r->dtg1.dtg1_on    = (t >> 7) & 1;
    r->dtg1.pass_thru  = (t >> 4) & 1;
    r->dtg1.mode       =  t       & 0x0F;
    r->dtg1.spec_a     = buf[0x25];
    r->dtg1.spec_b     = buf[0x26];
    r->dtg1.spec_c     = buf[0x27];
    r->dtg1.spec_d     = buf[0x28];
    r->dtg1.spec_d1    = buf[0x29];
    r->dtg1.spec_e     = buf[0x2A];
    r->dtg1.spec_h     = (((buf[0x2B]>>2)&0x03)<<8) | buf[0x2C];
    r->dtg1.spec_i     = (((buf[0x2D]&0x0F)<<8) | buf[0x2E]);
    r->dtg1.spec_k     = (((buf[0x30]&0x07)<<8) | buf[0x2F]);
    r->dtg1.spec_k1    = buf[0x31];
    r->dtg1.spec_g     = (((buf[0x33]&0x0F)<<8) | buf[0x32]);
    r->dtg1.total_pixels = (((buf[0x34]&0x1F)<<8)|buf[0x35]);
    t = buf[0x36];
    r->dtg1.field_flip = (t>>7)&1;
    r->dtg1.line_cnt   = (((t&0x07)<<8)|buf[0x37]);
    r->dtg1.frame_size= (((buf[0x39]>>4)&0x07)<<8)|buf[0x3A];
    r->dtg1.field_size= (((buf[0x39]&0x07)<<8)|buf[0x3B]);
    r->dtg1.cbar_size = buf[0x3C];

    /* DAC */
    t = buf[0x3D];
    r->dac.i2c_ctrl = (t>>6)&1;
    r->dac.dac1     = (((t>>4)&0x03)<<8)|buf[0x3E];
    r->dac.dac2     = (((t>>2)&0x03)<<8)|buf[0x3F];
    r->dac.dac3     = (((t>>0)&0x03)<<8)|buf[0x40];

    /* CSM */
    r->csm.clip_gy_lo = buf[0x41];
    r->csm.clip_cb_lo = buf[0x42];
    r->csm.clip_cr_lo = buf[0x43];
    r->csm.clip_gy_hi = buf[0x44];
    r->csm.clip_cb_hi = buf[0x45];
    r->csm.clip_cr_hi = buf[0x46];
    r->csm.shift_gy   = buf[0x47];
    r->csm.shift_cb   = buf[0x48];
    r->csm.shift_cr   = buf[0x49];
    r->csm.mult_gy_msb= (buf[0x4A]>>5)&0x07;
    r->csm.mult_cb_msb= (buf[0x4B]>>5)&0x07;
    r->csm.mult_cr_msb=  buf[0x4B]&0x07;
    r->csm.mult_gy_lsb= buf[0x4C];
    r->csm.mult_cb_lsb= buf[0x4D];
    r->csm.mult_cr_lsb= buf[0x4E];
    r->csm.csm_ctrl   = buf[0x4F];

    /* DTG2 */
    for(int i=0;i<16;i++){
        /* bp MSB at 0x50+i, LSB at 0x60+i */
        r->dtg2.bp[i] = (((buf[0x50+i]&0x03)<<8)|buf[0x60+i]);
    }
    for(int i=0;i<16;i++){
        /* linetype two per register 0x68+(i/2) */
        uint8_t v=buf[0x68+(i/2)];
        r->dtg2.linetype[i] = (i%2)?(v&0x0F):(v>>4);
    }
    /* Timing */
    r->dtg2.hlength   = (((buf[0x71]&0x03)<<8)|buf[0x70]);
    r->dtg2.hdly      = (((buf[0x71]&0x1F)<<8)|buf[0x72]);
    r->dtg2.vlength1 = (((buf[0x74]&0x03)<<8)|buf[0x73]);
    r->dtg2.vdly1     = (((buf[0x74]&0x07)<<8)|buf[0x75]);
    r->dtg2.vlength2 = (((buf[0x77]&0x03)<<8)|buf[0x76]);
    r->dtg2.vdly2     = (((buf[0x77]>>6)<<8)|buf[0x78]);
    r->dtg2.hsind    = (((buf[0x79]&0x1F)<<8)|buf[0x7A]);
    r->dtg2.vsind    = (((buf[0x7B]&0x07)<<8)|buf[0x7C]);
    r->dtg2.pixel_cnt= ((buf[0x7D]<<8)|buf[0x7E]);
    t=buf[0x7F];
    r->dtg2.ctrl.ip_fmt    = (t>>7)&1;
    r->dtg2.ctrl.line_cnt  = (((t&0x7F)<<8)|buf[0x80]);
    t=buf[0x82];
    r->dtg2.ctrl.fid_de    = (t>>7)&1;
    r->dtg2.ctrl.rgb_mode  = (t>>6)&1;
    r->dtg2.ctrl.emb_timing= (t>>5)&1;
    r->dtg2.ctrl.vs_out    = (t>>4)&1;
    r->dtg2.ctrl.hs_out    = (t>>3)&1;
    r->dtg2.ctrl.fid_pol   = (t>>2)&1;
    r->dtg2.ctrl.vs_in     = (t>>1)&1;
    r->dtg2.ctrl.hs_in     =  t    &1;

    /* CGMS */
    t = buf[0x83];
    r->cgms.enable = (t>>6)&1;
    r->cgms.header = t &0x3F;
    r->cgms.payload= ((buf[0x84]<<8)|buf[0x85]);

    /* Readback */
    r->readback.ppl = ((buf[0x86]<<8)|buf[0x87]);
    r->readback.lpf = ((buf[0x88]<<8)|buf[0x89]);

    return 0;
}

/**
 * @brief Write all THS8200 registers from ths8200_regs_t.
 * (Implementation analogous to read, omitted for brevity)
 */
static inline int ths8200_write_regs(const struct device *dev,
                                     uint8_t addr,
                                     const ths8200_regs_t *r)
{
    /* pack buf[...] from r-> fields */
    /* ... */
    return i2c_burst_write(dev, addr, 0x00, buf, sizeof(buf));
}

