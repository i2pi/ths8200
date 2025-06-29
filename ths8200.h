#ifndef THS8200_REGS_H
#define THS8200_REGS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file ths8200_regs.h
 * @brief Complete THS8200 DAC register map as nested C structures.
 *
 * Each register field is annotated with its default value and I2C sub-address & bit positions
 * according to the THS8200-EP datasheet.
 */

typedef struct {
    uint8_t reserved0; /**< Reserved @0x00 */
    uint8_t reserved1; /**< Reserved @0x01 */

    /** System Control (Sub-Addresses 0x02-0x03) */
    struct {
        uint8_t version;
        /**< ver(7:0): Device version [00000000]. @0x02 bits[7:0] */

        struct {
            bool vesa_clk;    /**< vesa_clk: Clock mode selection [0]. @0x03 bit7 */
            bool dll_bypass;  /**< dll_bypass: DLL bypass [0]. @0x03 bit6 */
            bool dac_pwdn;    /**< dac_pwdn: DAC power-down [0]. @0x03 bit5 */
            bool chip_pwdn;   /**< chip_pwdn: Chip power-down [0]. @0x03 bit4 */
            bool chip_msbars; /**< chip_msbars: Master/slave select [0]. @0x03 bit3 */
            bool sel_func_n;  /**< sel_func_n: Function select n [0]. @0x03 bit2 */
        } ctl;
    } system;

    /** Color Space Conversion (Sub-Addresses 0x04-0x19) */
    struct {
        /* R->R, R->G, R->B coefficients */
        int16_t csc_r11; /**< csc_r11: coefficient (signed Q2.8) [0]. @0x04-0x05 */
        int16_t csc_r12; /**< csc_r12: fractional part [0]. @0x05 bits */
        int16_t csc_r21; /**< csc_r21: coefficient (signed Q2.8) [0]. @0x06-0x07 */
        int16_t csc_r22; /**< csc_r22: fractional part [0]. @0x07 bits */
        int16_t csc_r31; /**< csc_r31: coefficient (signed Q2.8) [0]. @0x08-0x09 */
        int16_t csc_r32; /**< csc_r32: fractional part [0]. @0x09 bits */
        /* G->R, G->G, G->B coefficients */
        int16_t csc_g11; /**< csc_g11: coefficient (signed Q2.8) [0]. @0x0A-0x0B */
        int16_t csc_g12; /**< csc_g12: fractional part [0]. @0x0B bits */
        int16_t csc_g21; /**< csc_g21: coefficient (signed Q2.8) [0]. @0x0C-0x0D */
        int16_t csc_g22; /**< csc_g22: fractional part [0]. @0x0D bits */
        int16_t csc_g31; /**< csc_g31: coefficient (signed Q2.8) [0]. @0x0E-0x0F */
        int16_t csc_g32; /**< csc_g32: fractional part [0]. @0x0F bits */
        /* B->R, B->G, B->B coefficients */
        int16_t csc_b11; /**< csc_b11: coefficient (signed Q2.8) [0]. @0x10-0x11 */
        int16_t csc_b12; /**< csc_b12: fractional part [0]. @0x11 bits */
        int16_t csc_b21; /**< csc_b21: coefficient (signed Q2.8) [0]. @0x12-0x13 */
        int16_t csc_b22; /**< csc_b22: fractional part [0]. @0x13 bits */
        int16_t csc_b31; /**< csc_b31: coefficient (signed Q2.8) [0]. @0x14-0x15 */
        int16_t csc_b32; /**< csc_b32: fractional part [0]. @0x15 bits */

        /* CSC Offsets */
        int16_t offs1; /**< csc_offset1(9:0): DAC1 offset [0]. @0x16-0x17 */
        int16_t offs2; /**< csc_offset2(9:0): DAC2 offset [0]. @0x17-0x18 */
        int16_t offs3; /**< csc_offset3(9:0): DAC3 offset [0]. @0x18-0x19 */
        bool bypass;  /**< csc_bypass: CSC bypass [1]. @0x19 bit1 */
        bool uof;     /**< csc_uof_cntl: Under/overflow control [0]. @0x19 bit0 */
    } csc;

    /** Test Control (Sub-Addresses 0x1A-0x1B) */
    struct {
        bool digbypass; /**< tst_digbypass [0]. @0x1A bit7 */
        bool offset;    /**< tst_offset [0]. @0x1A bit6 */
        uint8_t ydelay; /**< tst_ydelay(1:0) [00]. @0x1B bits7..6 */
        bool fastramp;  /**< tst_fastramp [0]. @0x1B bit1 */
        bool slowramp;  /**< tst_slowramp [0]. @0x1B bit0 */
    } test;

    /** Data Path Control (Sub-Address 0x1C) */
    struct {
        bool clk656;      /**< data_clk656_on [0]. @0x1C bit7 */
        bool fsadj;       /**< data_fsadj [0]. @0x1C bit6 */
        bool ifir12;      /**< data_ifir12_bypass [0]. @0x1C bit5 */
        bool ifir35;      /**< data_ifir35_bypass [0]. @0x1C bit4 */
        bool tristate656; /**< data_tristate656 [0]. @0x1C bit3 */
        uint8_t dman;     /**< data_dman_cntl(2:0) [011]. @0x1C bits2..0 */
    } datapath;

    /** Display Timing Generator, Part 1 (Sub-Addresses 0x1D-0x3C) */
    struct {
        uint16_t y_blank;   /**< dtg1_y_blank(9:0) [0x200]. MSB@0x23[5:4], LSB@0x1D */
        uint16_t y_sync_lo; /**< dtg1_y_sync_low(9:0) [0]. MSB@0x23[3:2], LSB@0x1E */
        uint16_t y_sync_hi; /**< dtg1_y_sync_high(9:0) [0x300]. MSB@0x23[1:0], LSB@0x1F */
        uint16_t cbc_blank; /**< dtg1_cbcr_blank(9:0) [0x200]. MSB@0x24[5:4], LSB@0x20 */
        uint16_t cbc_sync_lo;/**< dtg1_cbcr_sync_low(9:0) [0]. MSB@0x24[3:2], LSB@0x21 */
        uint16_t cbc_sync_hi;/**< dtg1_cbcr_sync_high(9:0) [0x300]. MSB@0x24[1:0], LSB@0x22 */
        uint8_t mode;       /**< dtg1_mode [generic]. @0x23 bits7..4 */
        uint16_t frame_size;/**< dtg1_frame_size(10:0) [generic]. @0x39-0x3A */
        uint16_t field_size;/**< dtg1_field_size(10:0) [generic]. @0x39-0x3B */
        uint8_t cbar_size;  /**< dtg1_vesa_cbar_size [0x80]. @0x3C */
    } dtg1;

    /** DAC Control (Sub-Addresses 0x3D-0x40) */
    struct {
        bool i2c_cntl;    /**< dac_i2c_cntl [0]. @0x3D bit6 */
        uint16_t dac1;    /**< dac1_cntl(9:0) [0]. MSB@0x3D[5:4], LSB@0x3E */
        uint16_t dac2;    /**< dac2_cntl(9:0) [0]. MSB@0x3D[3:2], LSB@0x3F */
        uint16_t dac3;    /**< dac3_cntl(9:0) [0]. MSB@0x3D[1:0], LSB@0x40 */
    } dac;

    /** Clip/Scale/Multiplier Control (Sub-Addresses 0x41-0x4F) */
    struct {
        uint8_t clip_gy_lo; /**< csm_clip_gy_low(7:0) [0]. @0x41 */
        uint8_t clip_bcb_lo;/**< csm_clip_bcb_low(7:0) [0]. @0x42 */
        uint8_t clip_rcr_lo;/**< csm_clip_rcr_low(7:0) [0]. @0x43 */
        uint8_t clip_gy_hi; /**< csm_clip_gy_high(7:0) [0]. @0x44 */
        uint8_t clip_bcb_hi;/**< csm_clip_bcb_high(7:0) [0]. @0x45 */
        uint8_t clip_rcr_hi;/**< csm_clip_rcr_high(7:0) [0]. @0x46 */
        uint8_t shift_gy;   /**< csm_shift_gy(7:0) [0]. @0x47 */
        uint8_t shift_bcb;  /**< csm_shift_bcb(7:0) [0]. @0x48 */
        uint8_t shift_rcr;  /**< csm_shift_rcr(7:0) [0]. @0x49 */
        uint8_t mult_gy_msb;/**< csm_mult_gy(10:8) [0]. @0x4A bits7..5 */
        uint8_t mult_bcb_msb;/**< csm_mult_bcb(10:8) [0]. @0x4B bits5..3 */
        uint8_t mult_rcr_msb;/**< csm_mult_rcr(10:8) [0]. @0x4B bits2..0 */
        uint8_t mult_gy_lsb;/**< csm_mult_gy(7:0) [0]. @0x4C */
        uint8_t mult_bcb_lsb;/**< csm_mult_bcb(7:0) [0]. @0x4D */
        uint8_t mult_rcr_lsb;/**< csm_mult_rcr(7:0) [0]. @0x4E */
        uint8_t csm_ctl;    /**< csm control bits [0]. @0x4F */
    } csm;

    /** Display Timing Generator, Part 2 (Sub-Addresses 0x50-0x82) */
    struct {
        uint16_t bp[16];   /**< dtg2 breakpoints(10:0) [0]. @0x50-0x5F */
        uint8_t linetype[16]; /**< dtg2_linetype codes(3:0) [0]. @0x62,0x64-0x75 */
        uint16_t hlength;  /**< dtg2_hlength(9:0) [0x060]. @0x70-0x71 */
        uint16_t hdly;     /**< dtg2_hdly(12:0) [0x0020]. @0x71-0x72 */
        uint16_t vlength1; /**< dtg2_vlength1(9:0) [0x003]. @0x73-0x74 */
        uint16_t vdly1;    /**< dtg2_vdly1(10:0) [0]. @0x74-0x75 */
        uint16_t vlength2; /**< dtg2_vlength2(9:0)[0]. @0x76-0x77 */
        uint16_t vdly2;    /**< dtg2_vdly2(10:0)[0x3FF]. @0x77-0x78 */
        uint16_t hs_dly;   /**< dtg2_hs_in_dly(12:0)[0x03D]. @0x79-0x7A */
        uint16_t vs_dly;   /**< dtg2_vs_in_dly(10:0)[0x003]. @0x7B-0x7C */
        uint16_t pixel_cnt;/**< dtg2_pixel_cnt(15:0)[r/o]. @0x7D-0x7E */
        struct {
            bool ip_fmt;    /**< dtg2 interlaced/prog flag [r/o]. @0x7F bit7 */
            uint16_t line_cnt; /**< dtg2 line count(10:0) [r/o]. @0x7F-0x80 */
            bool fid_de;    /**< dtg2 fID/DE select [0]. @0x82 bit7 */
            bool rgb_mode;  /**< dtg2 rgb_mode_on [1]. @0x82 bit6 */
            bool emb_timing;/**< dtg2 embedded timing [0]. @0x82 bit5 */
            bool vsout_pol; /**< dtg2 vsout_pol [1]. @0x82 bit4 */
            bool hsout_pol; /**< dtg2 hsout_pol [1]. @0x82 bit3 */
            bool fid_pol;   /**< dtg2 fid_pol [1]. @0x82 bit2 */
            bool vsin_pol;  /**< dtg2 vsin_pol [1]. @0x82 bit1 */
            bool hsin_pol;  /**< dtg2 hsin_pol [1]. @0x82 bit0 */
        } cntl;
    } dtg2;

    /** CGMS Control (Sub-Addresses 0x83-0x85) */
    struct {
        bool enable;       /**< cgms enable [0]. @0x83 bit6 */
        uint8_t header;    /**< cgms_header(5:0) [0]. @0x83 bits5..0 */
        uint16_t payload;  /**< cgms_payload(13:0) [0]. @0x84-0x85 */
    } cgms;

    /** Readback (Sub-Addresses 0x86-0x89) */
    struct {
        uint16_t ppl; /**< HS high count(15:0) [r/o]. @0x87-0x86 */
        uint16_t lpf; /**< VS high count(15:0) [r/o]. @0x89-0x88 */
    } readback;
} ths8200_regs_t;

#endif // THS8200_REGS_H

