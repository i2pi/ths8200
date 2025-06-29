#include <stdio.h>
#include <stdlib.h>

#include "ths8200.h"

struct device { int x; };

int i2c_burst_write(const struct device *dev, uint8_t addr, uint8_t d, uint8_t *buf, uint16_t size) { return (0); }
int i2c_burst_read(const struct device *dev, uint8_t addr, uint8_t d, uint8_t *buf, uint16_t size) { return (0); }

#define THS8200_REG_COUNT 159

/**
 * @brief Read all THS8200 registers into ths8200_regs_t.
 */
static inline int ths8200_read_regs(const struct device *dev,
                                    uint8_t addr,
                                    ths8200_regs_t *r)
{
    uint8_t buf[THS8200_REG_COUNT];
    int rc = i2c_burst_read(dev, addr, 0x00, buf, sizeof(buf));
    if (rc) {
        return rc;
    }
    uint8_t t;

    /* System */
    r->reserved0 = buf[0];
    r->reserved1 = buf[1];
    r->system.version = buf[2];
    t = buf[3];
    r->system.ctl.vesa_clk    = (t >> 7) & 1;
    r->system.ctl.dll_bypass  = (t >> 6) & 1;
    r->system.ctl.dac_pwdn    = (t >> 5) & 1;
    r->system.ctl.chip_pwdn   = (t >> 4) & 1;
    r->system.ctl.chip_msbars = (t >> 3) & 1;
    r->system.ctl.sel_func_n  = (t >> 2) & 1;

    /* CSC */
    #define RD16(msb, lsb) ((int16_t)((buf[msb] << 8) | buf[lsb]))
    r->csc.csc_r11 = RD16(4,5);
    r->csc.csc_r21 = RD16(6,7);
    r->csc.csc_r31 = RD16(8,9);
    r->csc.csc_g11 = RD16(10,11);
    r->csc.csc_g21 = RD16(12,13);
    r->csc.csc_g31 = RD16(14,15);
    r->csc.csc_b11 = RD16(16,17);
    r->csc.csc_b21 = RD16(18,19);
    r->csc.offs1   = RD16(22,23);
    r->csc.offs2   = RD16(24,25);
    r->csc.offs3   = RD16(26,27);
    t = buf[27];
    r->csc.bypass  = (t >> 1) & 1;
    r->csc.uof     =  t       & 1;
    #undef RD16

    /* Test */
    t = buf[28];
    r->test.digbypass = (t >> 7) & 1;
    r->test.offset    = (t >> 6) & 1;
    t = buf[29];
    r->test.ydelay    = (t >> 6) & 3;
    r->test.fastramp  = (t >> 1) & 1;
    r->test.slowramp  =  t       & 1;

    /* DataPath */
    t = buf[30];
    r->datapath.clk656      = (t >> 7) & 1;
    r->datapath.fsadj       = (t >> 6) & 1;
    r->datapath.ifir12      = (t >> 5) & 1;
    r->datapath.ifir35      = (t >> 4) & 1;
    r->datapath.tristate656 = (t >> 3) & 1;
    r->datapath.dman        =  t       & 7;

    /* DTG1 */
    #define RD10(msb_reg, msb_bits, lsb_reg) \
        ((((buf[msb_reg] >> msb_bits) & 3) << 8) | buf[lsb_reg])
    r->dtg1.y_blank    = RD10(35,4,29);
    r->dtg1.y_sync_lo  = RD10(35,2,30);
    r->dtg1.y_sync_hi  = RD10(35,0,31);
    r->dtg1.cbc_blank  = RD10(36,4,32);
    r->dtg1.cbc_sync_lo= RD10(36,2,33);
    r->dtg1.cbc_sync_hi= RD10(36,0,34);
    #undef RD10
    r->dtg1.mode       = (buf[35] >> 4) & 0x0F;
    r->dtg1.frame_size = (buf[57] << 8) | buf[58];
    r->dtg1.field_size = (buf[59] << 8) | buf[60];
    r->dtg1.cbar_size  = buf[61];

    /* DAC */
    t = buf[62];
    r->dac.i2c_cntl = (t >> 6) & 1;
    r->dac.dac1    = (((t >> 4) & 3) << 8) | buf[63];
    r->dac.dac2    = (((t >> 2) & 3) << 8) | buf[64];
    r->dac.dac3    = (((t)      & 3) << 8) | buf[65];

    /* CSM */
    r->csm.clip_gy_lo   = buf[66];
    r->csm.clip_bcb_lo  = buf[67];
    r->csm.clip_rcr_lo  = buf[68];
    r->csm.clip_gy_hi   = buf[69];
    r->csm.clip_bcb_hi  = buf[70];
    r->csm.clip_rcr_hi  = buf[71];
    r->csm.shift_gy     = buf[72];
    r->csm.shift_bcb    = buf[73];
    r->csm.shift_rcr    = buf[74];
    r->csm.mult_gy_msb  = (buf[75] >> 5) & 7;
    r->csm.mult_bcb_msb = (buf[76] >> 3) & 7;
    r->csm.mult_rcr_msb =  buf[76]        & 7;
    r->csm.mult_gy_lsb  = buf[77];
    r->csm.mult_bcb_lsb = buf[78];
    r->csm.mult_rcr_lsb = buf[79];
    r->csm.csm_ctl      = buf[80];

    /* DTG2 */
    for (int i = 0; i < 16; i++) {
        int msb = 81 + i*2;
        int lsb = msb + 1;
        r->dtg2.bp[i]       = (buf[msb] << 8) | buf[lsb];
        r->dtg2.linetype[i] = buf[113 + i];
    }
    r->dtg2.hlength   = (buf[129] << 8) | buf[130];
    r->dtg2.hdly      = (buf[131] << 8) | buf[132];
    r->dtg2.vlength1 = (buf[133] << 8) | buf[134];
    r->dtg2.vdly1     = (buf[135] << 8) | buf[136];
    r->dtg2.vlength2 = (buf[137] << 8) | buf[138];
    r->dtg2.vdly2     = (buf[139] << 8) | buf[140];
    r->dtg2.hs_dly    = (buf[141] << 8) | buf[142];
    r->dtg2.vs_dly    = (buf[143] << 8) | buf[144];
    r->dtg2.pixel_cnt = (buf[145] << 8) | buf[146];
    t = buf[147];
    r->dtg2.cntl.ip_fmt    = (t >> 7) & 1;
    r->dtg2.cntl.line_cnt  = ((t & 0x7F) << 8) | buf[148];
    t = buf[150];
    r->dtg2.cntl.fid_de    = (t >> 7) & 1;
    r->dtg2.cntl.rgb_mode  = (t >> 6) & 1;
    r->dtg2.cntl.emb_timing= (t >> 5) & 1;
    r->dtg2.cntl.vsout_pol = (t >> 4) & 1;
    r->dtg2.cntl.hsout_pol = (t >> 3) & 1;
    r->dtg2.cntl.fid_pol   = (t >> 2) & 1;
    r->dtg2.cntl.vsin_pol  = (t >> 1) & 1;
    r->dtg2.cntl.hsin_pol  =  t       & 1;

    /* CGMS */
    t = buf[151];
    r->cgms.enable  = (t >> 6) & 1;
    r->cgms.header  =  t       & 0x3F;
    r->cgms.payload = (buf[152] << 8) | buf[153];

    /* Readback */
    r->readback.ppl = (buf[154] << 8) | buf[155];
    r->readback.lpf = (buf[156] << 8) | buf[157];

    return 0;
}

/**
 * @brief Write all THS8200 registers from ths8200_regs_t.
 */
static inline int ths8200_write_regs(const struct device *dev,
                                     uint8_t addr,
                                     const ths8200_regs_t *r)
{
    uint8_t buf[THS8200_REG_COUNT];
    int rc;
    uint8_t t;

    /* System */
    buf[0] = r->reserved0;
    buf[1] = r->reserved1;
    buf[2] = r->system.version;
    buf[3] = (r->system.ctl.vesa_clk    << 7) |
             (r->system.ctl.dll_bypass  << 6) |
             (r->system.ctl.dac_pwdn    << 5) |
             (r->system.ctl.chip_pwdn   << 4) |
             (r->system.ctl.chip_msbars << 3) |
             (r->system.ctl.sel_func_n  << 2);

    /* CSC */
    #define WR16(msb, lsb, v) \
        buf[msb] = (uint8_t)((v) >> 8); \
        buf[lsb] = (uint8_t)((v) & 0xFF)
    WR16(4,5,  r->csc.csc_r11);
    WR16(6,7,  r->csc.csc_r21);
    WR16(8,9,  r->csc.csc_r31);
    WR16(10,11,r->csc.csc_g11);
    WR16(12,13,r->csc.csc_g21);
    WR16(14,15,r->csc.csc_g31);
    WR16(16,17,r->csc.csc_b11);
    WR16(18,19,r->csc.csc_b21);
    WR16(22,23,r->csc.offs1);
    WR16(24,25,r->csc.offs2);
    WR16(26,27,r->csc.offs3);
    t = (r->csc.bypass << 1) | r->csc.uof;
    buf[27] |= t;
    #undef WR16

    /* Test */
    buf[28] = (r->test.digbypass << 7) |
             (r->test.offset    << 6);
    buf[29] = (r->test.ydelay   << 6) |
             (r->test.fastramp << 1) |
             (r->test.slowramp);

    /* DataPath */
    buf[30] = (r->datapath.clk656    << 7) |
             (r->datapath.fsadj     << 6) |
             (r->datapath.ifir12    << 5) |
             (r->datapath.ifir35    << 4) |
             (r->datapath.tristate656<< 3)|
             (r->datapath.dman & 7);

    /* DTG1 */
    #define WR10(msb_reg, msb_bits, lsb_reg, v) \
        buf[msb_reg] |= (((v) >> 8) & 3) << (msb_bits); \
        buf[lsb_reg] = (uint8_t)((v) & 0xFF)
    WR10(35,4,29, r->dtg1.y_blank);
    WR10(35,2,30, r->dtg1.y_sync_lo);
    WR10(35,0,31, r->dtg1.y_sync_hi);
    WR10(36,4,32, r->dtg1.cbc_blank);
    WR10(36,2,33, r->dtg1.cbc_sync_lo);
    WR10(36,0,34, r->dtg1.cbc_sync_hi);
    buf[35] |= (r->dtg1.mode & 0x0F) << 4;
    buf[57] = (uint8_t)(r->dtg1.frame_size >> 8);
    buf[58] = (uint8_t)(r->dtg1.frame_size & 0xFF);
    buf[59] = (uint8_t)(r->dtg1.field_size >> 8);
    buf[60] = (uint8_t)(r->dtg1.field_size & 0xFF);
    buf[61] = r->dtg1.cbar_size;
    #undef WR10

    /* DAC */
    t = (r->dac.i2c_cntl << 6) |
        (((r->dac.dac1 >> 8) & 3) << 4) |
        (((r->dac.dac2 >> 8) & 3) << 2) |
         ((r->dac.dac3 >> 8) & 3);
    buf[62] = t;
    buf[63] = r->dac.dac1 & 0xFF;
    buf[64] = r->dac.dac2 & 0xFF;
    buf[65] = r->dac.dac3 & 0xFF;

    /* CSM */
    buf[66] = r->csm.clip_gy_lo;
    buf[67] = r->csm.clip_bcb_lo;
    buf[68] = r->csm.clip_rcr_lo;
    buf[69] = r->csm.clip_gy_hi;
    buf[70] = r->csm.clip_bcb_hi;
    buf[71] = r->csm.clip_rcr_hi;
    buf[72] = r->csm.shift_gy;
    buf[73] = r->csm.shift_bcb;
    buf[74] = r->csm.shift_rcr;
    buf[75] = (r->csm.mult_gy_msb & 7) << 5;
    buf[76] = ((r->csm.mult_bcb_msb & 7) << 3) |
             (r->csm.mult_rcr_msb & 7);
    buf[77] = r->csm.mult_gy_lsb;
    buf[78] = r->csm.mult_bcb_lsb;
    buf[79] = r->csm.mult_rcr_lsb;
    buf[80] = r->csm.csm_ctl;

    /* DTG2 */
    for (int i = 0; i < 16; i++) {
        int msb = 81 + i*2;
        int lsb = msb + 1;
        buf[msb] = (uint8_t)(r->dtg2.bp[i] >> 8);
        buf[lsb] = (uint8_t)(r->dtg2.bp[i] & 0xFF);
        buf[113 + i] = r->dtg2.linetype[i];
    }
    buf[129] = (uint8_t)(r->dtg2.hlength >> 8);
    buf[130] = (uint8_t)(r->dtg2.hlength & 0xFF);
    buf[131] = (uint8_t)(r->dtg2.hdly >> 8);
    buf[132] = (uint8_t)(r->dtg2.hdly & 0xFF);
    buf[133] = (uint8_t)(r->dtg2.vlength1 >> 8);
    buf[134] = (uint8_t)(r->dtg2.vlength1 & 0xFF);
    buf[135] = (uint8_t)(r->dtg2.vdly1 >> 8);
    buf[136] = (uint8_t)(r->dtg2.vdly1 & 0xFF);
    buf[137] = (uint8_t)(r->dtg2.vlength2 >> 8);
    buf[138] = (uint8_t)(r->dtg2.vlength2 & 0xFF);
    buf[139] = (uint8_t)(r->dtg2.vdly2 >> 8);
    buf[140] = (uint8_t)(r->dtg2.vdly2 & 0xFF);
    buf[141] = (uint8_t)(r->dtg2.hs_dly >> 8);
    buf[142] = (uint8_t)(r->dtg2.hs_dly & 0xFF);
    buf[143] = (uint8_t)(r->dtg2.vs_dly >> 8);
    buf[144] = (uint8_t)(r->dtg2.vs_dly & 0xFF);
    buf[145] = (uint8_t)(r->dtg2.pixel_cnt >> 8);
    buf[146] = (uint8_t)(r->dtg2.pixel_cnt & 0xFF);
    buf[147] = (r->dtg2.cntl.ip_fmt << 7) |
              ((r->dtg2.cntl.line_cnt >> 8) & 0x7F);
    buf[148] = r->dtg2.cntl.line_cnt & 0xFF;
    buf[150] = (r->dtg2.cntl.fid_de    << 7) |
              (r->dtg2.cntl.rgb_mode  << 6) |
              (r->dtg2.cntl.emb_timing<< 5) |
              (r->dtg2.cntl.vsout_pol << 4) |
              (r->dtg2.cntl.hsout_pol << 3) |
              (r->dtg2.cntl.fid_pol   << 2) |
              (r->dtg2.cntl.vsin_pol  << 1) |
               r->dtg2.cntl.hsin_pol;

    /* CGMS */
    buf[151] = (r->cgms.enable << 6) | (r->cgms.header & 0x3F);
    buf[152] = (uint8_t)(r->cgms.payload >> 8);
    buf[153] = (uint8_t)(r->cgms.payload & 0xFF);

    /* Readback fields are read-only, skip 154-157 */

    /* Write all writable regs */
    rc = i2c_burst_write(dev, addr, 0x00, buf, sizeof(buf));
    return rc;
}
