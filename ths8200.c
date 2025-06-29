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
    uint8_t buf[THS8200_REG_COUNT] = {0};

    /* System control */
    buf[0x02] = r->system.version;
    buf[0x03] = (r->system.ctl.vesa_clk   ? 0x80 : 0) |
                (r->system.ctl.dll_bypass ? 0x40 : 0) |
                (r->system.ctl.dac_pwdn   ? 0x20 : 0) |
                (r->system.ctl.chip_pwdn  ? 0x10 : 0) |
                (r->system.ctl.chip_msbars? 0x08 : 0) |
                (r->system.ctl.sel_func_n ? 0x04 : 0) |
                (r->system.ctl.arst_func_n? 0x02 : 0);

    /* CSC coefficients and offsets (Q2.8) */
#define WR_COEF(msb, lsb, i_part, f_part) \
    buf[msb] = (uint8_t)r->csc.i_part; \
    buf[lsb] = r->csc.f_part
    WR_COEF(0x04,0x05, r2r_int, r2r_frac);
    WR_COEF(0x06,0x07, r2g_int, r2g_frac);
    WR_COEF(0x08,0x09, r2b_int, r2b_frac);
    WR_COEF(0x0A,0x0B, g2r_int, g2r_frac);
    WR_COEF(0x0C,0x0D, g2g_int, g2g_frac);
    WR_COEF(0x0E,0x0F, g2b_int, g2b_frac);
    WR_COEF(0x10,0x11, b2r_int, b2r_frac);
    WR_COEF(0x12,0x13, b2g_int, b2g_frac);
    WR_COEF(0x14,0x15, b2b_int, b2b_frac);
    WR_COEF(0x16,0x17, yoff_int, yoff_frac);
    WR_COEF(0x18,0x19, cboff_int, cboff_frac);
#undef WR_COEF
    buf[0x19] = (r->csc.csc_bypass ? 0x02 : 0) |
                (r->csc.csc_uof    ? 0x01 : 0);

    /* Test control */
    buf[0x1A] = (r->test.digbypass ? 0x80 : 0) |
                (r->test.force_off ? 0x40 : 0);
    buf[0x1B] = ((r->test.ydelay & 0x03) << 6) |
                (r->test.fastramp ? 0x02 : 0) |
                (r->test.slowramp ? 0x01 : 0);

    /* Data path */
    buf[0x1C] = (r->datapath.clk656 ? 0x80 : 0) |
                (r->datapath.fsadj  ? 0x40 : 0) |
                (r->datapath.ifir12 ? 0x20 : 0) |
                (r->datapath.ifir35 ? 0x10 : 0) |
                (r->datapath.tri656 ? 0x08 : 0) |
                (r->datapath.format & 0x07);

    /* DTG1 values */
    buf[0x1D] = r->dtg1.y_blank & 0xFF;
    buf[0x1E] = r->dtg1.y_sync_lo & 0xFF;
    buf[0x1F] = r->dtg1.y_sync_hi & 0xFF;
    buf[0x20] = r->dtg1.cbcr_blank & 0xFF;
    buf[0x21] = r->dtg1.cbcr_sync_lo & 0xFF;
    buf[0x22] = r->dtg1.cbcr_sync_hi & 0xFF;
    buf[0x23] = ((r->dtg1.y_blank>>8)&0x03)<<4 |
                ((r->dtg1.y_sync_lo>>8)&0x03)<<2 |
                ((r->dtg1.y_sync_hi>>8)&0x03);
    buf[0x24] = ((r->dtg1.cbcr_blank>>8)&0x03)<<4 |
                ((r->dtg1.cbcr_sync_lo>>8)&0x03)<<2 |
                ((r->dtg1.cbcr_sync_hi>>8)&0x03);

    buf[0x25] = r->dtg1.spec_a;
    buf[0x26] = r->dtg1.spec_b;
    buf[0x27] = r->dtg1.spec_c;
    buf[0x28] = r->dtg1.spec_d;
    buf[0x29] = r->dtg1.spec_d1;
    buf[0x2A] = r->dtg1.spec_e;
    buf[0x2B] = ((r->dtg1.spec_h>>8)&0x03)<<2;
    buf[0x2C] = r->dtg1.spec_h & 0xFF;
    buf[0x2D] = (r->dtg1.spec_i>>8)&0x0F;
    buf[0x2E] = r->dtg1.spec_i & 0xFF;
    buf[0x2F] = r->dtg1.spec_k & 0xFF;
    buf[0x30] = (r->dtg1.spec_k>>8)&0x07;
    buf[0x31] = r->dtg1.spec_k1;
    buf[0x32] = r->dtg1.spec_g & 0xFF;
    buf[0x33] = (r->dtg1.spec_g>>8)&0x0F;
    buf[0x34] = (r->dtg1.total_pixels>>8)&0x1F;
    buf[0x35] = r->dtg1.total_pixels & 0xFF;
    buf[0x36] = (r->dtg1.field_flip?0x80:0) | ((r->dtg1.line_cnt>>8)&0x07);
    buf[0x37] = r->dtg1.line_cnt & 0xFF;
    buf[0x38] = (r->dtg1.dtg1_on?0x80:0) | (r->dtg1.pass_thru?0x10:0) |
                (r->dtg1.mode & 0x0F);
    buf[0x39] = ((r->dtg1.frame_size>>8)&0x07)<<4 |
                ((r->dtg1.field_size>>8)&0x07);
    buf[0x3A] = r->dtg1.frame_size & 0xFF;
    buf[0x3B] = r->dtg1.field_size & 0xFF;
    buf[0x3C] = r->dtg1.cbar_size;

    /* DAC control */
    buf[0x3D] = (r->dac.i2c_ctrl?0x40:0) |
                ((r->dac.dac1>>8)&0x03)<<4 |
                ((r->dac.dac2>>8)&0x03)<<2 |
                ((r->dac.dac3>>8)&0x03);
    buf[0x3E] = r->dac.dac1 & 0xFF;
    buf[0x3F] = r->dac.dac2 & 0xFF;
    buf[0x40] = r->dac.dac3 & 0xFF;

    /* Clip/Scale/Multiplier */
    buf[0x41] = r->csm.clip_gy_lo;
    buf[0x42] = r->csm.clip_cb_lo;
    buf[0x43] = r->csm.clip_cr_lo;
    buf[0x44] = r->csm.clip_gy_hi;
    buf[0x45] = r->csm.clip_cb_hi;
    buf[0x46] = r->csm.clip_cr_hi;
    buf[0x47] = r->csm.shift_gy;
    buf[0x48] = r->csm.shift_cb;
    buf[0x49] = r->csm.shift_cr;
    buf[0x4A] = (r->csm.mult_gy_msb & 0x07) << 5;
    buf[0x4B] = ((r->csm.mult_cb_msb & 0x07) << 5) |
                (r->csm.mult_cr_msb & 0x07);
    buf[0x4C] = r->csm.mult_gy_lsb;
    buf[0x4D] = r->csm.mult_cb_lsb;
    buf[0x4E] = r->csm.mult_cr_lsb;
    buf[0x4F] = r->csm.csm_ctrl;

    /* DTG2 breakpoints */
    for (int i = 0; i < 16; i++) {
        buf[0x50 + i] = (r->dtg2.bp[i] >> 8) & 0x03;
        buf[0x60 + i] = r->dtg2.bp[i] & 0xFF;
    }
    for (int i = 0; i < 16; i++) {
        uint8_t idx = 0x68 + (i/2);
        if ((i & 1) == 0)
            buf[idx] = (r->dtg2.linetype[i] & 0x0F) << 4;
        else
            buf[idx] |= (r->dtg2.linetype[i] & 0x0F);
    }

    buf[0x70] = r->dtg2.hlength & 0xFF;
    buf[0x71] = ((r->dtg2.hdly>>8)&0x1F) |
                ((r->dtg2.hlength>>8)&0x03);
    buf[0x72] = r->dtg2.hdly & 0xFF;
    buf[0x73] = r->dtg2.vlength1 & 0xFF;
    buf[0x74] = ((r->dtg2.vdly1>>8)&0x07) |
                ((r->dtg2.vlength1>>8)&0x03);
    buf[0x75] = r->dtg2.vdly1 & 0xFF;
    buf[0x76] = r->dtg2.vlength2 & 0xFF;
    buf[0x77] = (((r->dtg2.vdly2>>8)&0x03) << 6) |
                ((r->dtg2.vlength2>>8)&0x03);
    buf[0x78] = r->dtg2.vdly2 & 0xFF;
    buf[0x79] = (r->dtg2.hsind>>8) & 0x1F;
    buf[0x7A] = r->dtg2.hsind & 0xFF;
    buf[0x7B] = (r->dtg2.vsind>>8) & 0x07;
    buf[0x7C] = r->dtg2.vsind & 0xFF;
    buf[0x7D] = (r->dtg2.pixel_cnt>>8) & 0xFF;
    buf[0x7E] = r->dtg2.pixel_cnt & 0xFF;
    buf[0x7F] = (r->dtg2.ctrl.ip_fmt ? 0x80 : 0) |
                ((r->dtg2.ctrl.line_cnt>>8) & 0x7F);
    buf[0x80] = r->dtg2.ctrl.line_cnt & 0xFF;
    buf[0x82] = (r->dtg2.ctrl.fid_de   ? 0x80 : 0) |
                (r->dtg2.ctrl.rgb_mode ? 0x40 : 0) |
                (r->dtg2.ctrl.emb_timing?0x20 : 0) |
                (r->dtg2.ctrl.vs_out   ? 0x10 : 0) |
                (r->dtg2.ctrl.hs_out   ? 0x08 : 0) |
                (r->dtg2.ctrl.fid_pol  ? 0x04 : 0) |
                (r->dtg2.ctrl.vs_in    ? 0x02 : 0) |
                (r->dtg2.ctrl.hs_in    ? 0x01 : 0);

    /* CGMS control */
    buf[0x83] = (r->cgms.enable ? 0x40 : 0) |
                (r->cgms.header & 0x3F);
    buf[0x84] = (r->cgms.payload>>8) & 0xFF;
    buf[0x85] = r->cgms.payload & 0xFF;

    /* Readback (write ignored) */
    buf[0x86] = (r->readback.ppl>>8) & 0xFF;
    buf[0x87] = r->readback.ppl & 0xFF;
    buf[0x88] = (r->readback.lpf>>8) & 0xFF;
    buf[0x89] = r->readback.lpf & 0xFF;

    return i2c_burst_write(dev, addr, 0x00, buf, sizeof(buf));
}

static inline const char *boolstr(bool v) { return v ? "true" : "false"; }

void ths8200_print_regs(const ths8200_regs_t *r)
{
    printf("System.version: 0x%02X\n", r->system.version);
    printf(" System ctl: vesa_clk=%s dll_bypass=%s dac_pwdn=%s chip_pwdn=%s chip_msbars=%s sel_func_n=%s arst_func_n=%s\n",
           boolstr(r->system.ctl.vesa_clk), boolstr(r->system.ctl.dll_bypass),
           boolstr(r->system.ctl.dac_pwdn), boolstr(r->system.ctl.chip_pwdn),
           boolstr(r->system.ctl.chip_msbars), boolstr(r->system.ctl.sel_func_n),
           boolstr(r->system.ctl.arst_func_n));

    printf("CSC:\n");
#define PR_COEF(name) \
    printf(" %s = %d + 0x%02X/256\n", #name, r->csc.name##_int, r->csc.name##_frac)
    PR_COEF(r2r); PR_COEF(r2g); PR_COEF(r2b);
    PR_COEF(g2r); PR_COEF(g2g); PR_COEF(g2b);
    PR_COEF(b2r); PR_COEF(b2g); PR_COEF(b2b);
    PR_COEF(yoff); PR_COEF(cboff);
#undef PR_COEF
    printf(" csc_bypass=%s csc_uof=%s\n",
           boolstr(r->csc.csc_bypass), boolstr(r->csc.csc_uof));

    printf("Test: digbypass=%s force_off=%s ydelay=%u fastramp=%s slowramp=%s\n",
           boolstr(r->test.digbypass), boolstr(r->test.force_off),
           r->test.ydelay, boolstr(r->test.fastramp), boolstr(r->test.slowramp));

    printf("Datapath: clk656=%s fsadj=%s ifir12=%s ifir35=%s tri656=%s format=0x%X\n",
           boolstr(r->datapath.clk656), boolstr(r->datapath.fsadj),
           boolstr(r->datapath.ifir12), boolstr(r->datapath.ifir35),
           boolstr(r->datapath.tri656), r->datapath.format);

    printf("DTG1:\n");
    printf(" y_blank=%u y_sync_lo=%u y_sync_hi=%u\n",
           r->dtg1.y_blank, r->dtg1.y_sync_lo, r->dtg1.y_sync_hi);
    printf(" cbcr_blank=%u cbcr_sync_lo=%u cbcr_sync_hi=%u\n",
           r->dtg1.cbcr_blank, r->dtg1.cbcr_sync_lo, r->dtg1.cbcr_sync_hi);
    printf(" dtg1_on=%s pass_thru=%s mode=0x%X\n",
           boolstr(r->dtg1.dtg1_on), boolstr(r->dtg1.pass_thru), r->dtg1.mode);
    printf(" spec_a=0x%02X spec_b=0x%02X spec_c=0x%02X spec_d=0x%02X spec_d1=0x%02X spec_e=0x%02X\n",
           r->dtg1.spec_a, r->dtg1.spec_b, r->dtg1.spec_c, r->dtg1.spec_d,
           r->dtg1.spec_d1, r->dtg1.spec_e);
    printf(" spec_h=%u spec_i=%u spec_k=%u spec_k1=0x%02X\n",
           r->dtg1.spec_h, r->dtg1.spec_i, r->dtg1.spec_k, r->dtg1.spec_k1);
    printf(" spec_g=%u total_pixels=%u field_flip=%s line_cnt=%u\n",
           r->dtg1.spec_g, r->dtg1.total_pixels, boolstr(r->dtg1.field_flip), r->dtg1.line_cnt);
    printf(" frame_size=%u field_size=%u cbar_size=%u\n",
           r->dtg1.frame_size, r->dtg1.field_size, r->dtg1.cbar_size);

    printf("DAC: i2c_ctrl=%s dac1=%u dac2=%u dac3=%u\n",
           boolstr(r->dac.i2c_ctrl), r->dac.dac1, r->dac.dac2, r->dac.dac3);

    printf("CSM:\n");
    printf(" clip_gy_lo=%u clip_cb_lo=%u clip_cr_lo=%u\n", r->csm.clip_gy_lo, r->csm.clip_cb_lo, r->csm.clip_cr_lo);
    printf(" clip_gy_hi=%u clip_cb_hi=%u clip_cr_hi=%u\n", r->csm.clip_gy_hi, r->csm.clip_cb_hi, r->csm.clip_cr_hi);
    printf(" shift_gy=%u shift_cb=%u shift_cr=%u\n", r->csm.shift_gy, r->csm.shift_cb, r->csm.shift_cr);
    printf(" mult_gy=%u.%u mult_cb=%u.%u mult_cr=%u.%u csm_ctrl=0x%02X\n",
           r->csm.mult_gy_msb, r->csm.mult_gy_lsb,
           r->csm.mult_cb_msb, r->csm.mult_cb_lsb,
           r->csm.mult_cr_msb, r->csm.mult_cr_lsb,
           r->csm.csm_ctrl);

    printf("DTG2 breakpoints:\n");
    for (int i = 0; i < 16; i++)
        printf("  bp[%d]=%u\n", i, r->dtg2.bp[i]);
    for (int i = 0; i < 16; i++)
        printf("  linetype[%d]=0x%X\n", i, r->dtg2.linetype[i]);

    printf("DTG2 timing: hlength=%u hdly=%u vlength1=%u vdly1=%u vlength2=%u vdly2=%u\n",
           r->dtg2.hlength, r->dtg2.hdly, r->dtg2.vlength1, r->dtg2.vdly1,
           r->dtg2.vlength2, r->dtg2.vdly2);
    printf(" hsind=%u vsind=%u pixel_cnt=%u\n",
           r->dtg2.hsind, r->dtg2.vsind, r->dtg2.pixel_cnt);
    printf(" ip_fmt=%s line_cnt=%u fid_de=%s rgb_mode=%s emb_timing=%s vs_out=%s hs_out=%s fid_pol=%s vs_in=%s hs_in=%s\n",
           boolstr(r->dtg2.ctrl.ip_fmt), r->dtg2.ctrl.line_cnt,
           boolstr(r->dtg2.ctrl.fid_de), boolstr(r->dtg2.ctrl.rgb_mode),
           boolstr(r->dtg2.ctrl.emb_timing), boolstr(r->dtg2.ctrl.vs_out),
           boolstr(r->dtg2.ctrl.hs_out), boolstr(r->dtg2.ctrl.fid_pol),
           boolstr(r->dtg2.ctrl.vs_in), boolstr(r->dtg2.ctrl.hs_in));

    printf("CGMS: enable=%s header=0x%02X payload=%u\n",
           boolstr(r->cgms.enable), r->cgms.header, r->cgms.payload);

    printf("Readback: ppl=%u lpf=%u\n",
           r->readback.ppl, r->readback.lpf);
}

