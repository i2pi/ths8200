#ifndef THS8200_REGS_H
#define THS8200_REGS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file ths8200_regs.h
 * @brief Complete THS8200 DAC register map as nested C structures.
 *
 * Each field abstracts MSB/LSB register pairs into single variables,
 * and splits integer and fractional parts for Q2.8 values.
 */
typedef struct {
    /** 0x02–0x03: System Control */
    struct {
        uint8_t version;         /**< @0x02 Device version */
        struct {
            bool vesa_clk;       /**< @0x03 bit7 */
            bool dll_bypass;     /**< @0x03 bit6 */
            bool dac_pwdn;       /**< @0x03 bit5 */
            bool chip_pwdn;      /**< @0x03 bit4 */
            bool chip_msbars;    /**< @0x03 bit3 */
            bool sel_func_n;     /**< @0x03 bit2 */
            bool arst_func_n;    /**< @0x03 bit1 software reset */
        } ctl;
    } system;

    /** 0x04–0x19: Color Space Conversion (Q2.8) */
    struct {
        /* coefficients: MSB = int part, frac = fractional byte */
        int8_t  r2r_int; uint8_t r2r_frac; /**< @0x04/05 R→R */
        int8_t  r2g_int; uint8_t r2g_frac; /**< @0x06/07 R→G */
        int8_t  r2b_int; uint8_t r2b_frac; /**< @0x08/09 R→B */
        int8_t  g2r_int; uint8_t g2r_frac; /**< @0x0A/0x0B G→R */
        int8_t  g2g_int; uint8_t g2g_frac; /**< @0x0C/0x0D G→G */
        int8_t  g2b_int; uint8_t g2b_frac; /**< @0x0E/0x0F G→B */
        int8_t  b2r_int; uint8_t b2r_frac; /**< @0x10/0x11 B→R */
        int8_t  b2g_int; uint8_t b2g_frac; /**< @0x12/0x13 B→G */
        int8_t  b2b_int; uint8_t b2b_frac; /**< @0x14/0x15 B→B */
        /* offsets: same Q2.8 format */
        int8_t  yoff_int; uint8_t yoff_frac; /**< @0x16/17 Y offset */
        int8_t  cboff_int; uint8_t cboff_frac; /**< @0x18/19 Cb/Cr offset */
        bool    csc_bypass; /**< @0x19 bit1 */
        bool    csc_uof;    /**< @0x19 bit0 under/overflow control */
    } csc;

    /** 0x1A–0x1B: Test Control */
    struct {
        bool    digbypass; /**< @0x1A bit7 */
        bool    force_off; /**< @0x1A bit6 */
        uint8_t ydelay;    /**< @0x1B bits7-6 */
        bool    fastramp;  /**< @0x1B bit1 */
        bool    slowramp;  /**< @0x1B bit0 */
    } test;

    /** 0x1C: Data Path Control */
    struct {
        bool    clk656;    /**< @0x1C bit7 */
        bool    fsadj;     /**< @0x1C bit6 */
        bool    ifir12;    /**< @0x1C bit5 */
        bool    ifir35;    /**< @0x1C bit4 */
        bool    tri656;    /**< @0x1C bit3 */
        uint8_t format;    /**< @0x1C bits2-0 */
    } datapath;

    /** 0x1D–0x3C: Display Timing Generator, Part 1 */
    struct {
        uint16_t y_blank;      /**< @0x1D LSB, 0x23[5:4] MSB */
        uint16_t y_sync_lo;    /**< @0x1E/0x23[3:2] */
        uint16_t y_sync_hi;    /**< @0x1F/0x23[1:0] */
        uint16_t cbcr_blank;   /**< @0x20/0x24[5:4] */
        uint16_t cbcr_sync_lo; /**< @0x21/0x24[3:2] */
        uint16_t cbcr_sync_hi; /**< @0x22/0x24[1:0] */
        bool     dtg1_on;      /**< @0x38 bit7 */
        bool     pass_thru;    /**< @0x38 bit4 */
        uint8_t  mode;         /**< @0x38 bits3-0 */
        uint8_t  spec_a;       /**< @0x25 */
        uint8_t  spec_b;       /**< @0x26 */
        uint8_t  spec_c;       /**< @0x27 */
        uint8_t  spec_d;       /**< @0x28 */
        uint8_t  spec_d1;      /**< @0x29 */
        uint8_t  spec_e;       /**< @0x2A */
        uint16_t spec_h;       /**< 0x2B[3:2] & 0x2C */
        uint16_t spec_i;       /**< 0x2D[3:0] & 0x2E */
        uint16_t spec_k;       /**< 0x2F & 0x30[2:0] */
        uint8_t  spec_k1;      /**< @0x31 */
        uint16_t spec_g;       /**< 0x32 & 0x33[3:0] */
        uint16_t total_pixels; /**< 0x34[4:0] & 0x35 */
        bool     field_flip;   /**< @0x36 bit7 */
        uint16_t line_cnt;     /**< 0x36[2:0] & 0x37 */
        uint16_t frame_size;   /**< 0x39[6:4] & 0x3A */
        uint16_t field_size;   /**< 0x39[2:0] & 0x3B */
        uint8_t  cbar_size;    /**< @0x3C */
    } dtg1;

    /** 0x3D–0x40: DAC Control */
    struct {
        bool     i2c_ctrl; /**< @0x3D bit6 */
        uint16_t dac1;     /**< 0x3D[5:4] & 0x3E */
        uint16_t dac2;     /**< 0x3D[3:2] & 0x3F */
        uint16_t dac3;     /**< 0x3D[1:0] & 0x40 */
    } dac;

    /** 0x41–0x4F: Clip/Scale/Multiplier */
    struct {
        uint8_t clip_gy_lo;   /**< @0x41 */
        uint8_t clip_cb_lo;   /**< @0x42 */
        uint8_t clip_cr_lo;   /**< @0x43 */
        uint8_t clip_gy_hi;   /**< @0x44 */
        uint8_t clip_cb_hi;   /**< @0x45 */
        uint8_t clip_cr_hi;   /**< @0x46 */
        uint8_t shift_gy;     /**< @0x47 */
        uint8_t shift_cb;     /**< @0x48 */
        uint8_t shift_cr;     /**< @0x49 */
        uint8_t mult_gy_msb;  /**< @0x4A bits7-5 */
        uint8_t mult_cb_msb;  /**< @0x4B bits5-3 */
        uint8_t mult_cr_msb;  /**< @0x4B bits2-0 */
        uint8_t mult_gy_lsb;  /**< @0x4C */
        uint8_t mult_cb_lsb;  /**< @0x4D */
        uint8_t mult_cr_lsb;  /**< @0x4E */
        uint8_t csm_ctrl;     /**< @0x4F */
    } csm;

    /** 0x50–0x82: Display Timing Generator, Part 2 */
    struct {
        uint16_t bp[16];      /**< dtg2 breakpoints (11-bit): 0x50–0x61 MSB & 0x58–0x69 LSB */
        uint8_t  linetype[16];/**< dtg2 line types: 0x68–0x6F hold two codes each */
        uint16_t hlength;     /**< dtg2_hlength (10-bit): 0x70–0x71 */
        uint16_t hdly;        /**< dtg2_hdly (13-bit): 0x71(4:0)&0x72 */
        uint16_t vlength1;    /**< dtg2_vlength1 (10-bit): 0x73–0x74 */
        uint16_t vdly1;       /**< dtg2_vdly1 (11-bit): 0x74(2:0)&0x75 */
        uint16_t vlength2;    /**< dtg2_vlength2 (10-bit): 0x76–0x77 */
        uint16_t vdly2;       /**< dtg2_vdly2 (11-bit): 0x77(7:6)&0x78 */
        uint16_t hsind;       /**< dtg2_hs_in_dly (13-bit): 0x79(4:0)&0x7A */
        uint16_t vsind;       /**< dtg2_vs_in_dly (11-bit): 0x7B(2:0)&0x7C */
        uint16_t pixel_cnt;   /**< dtg2_pixel_cnt (16-bit r/o): 0x7D–0x7E */
        struct {
            bool    ip_fmt;    /**< @0x7F bit7 */
            uint16_t line_cnt; /**< dtg2_line_cnt (11-bit): 0x7F[6:0]&0x80 */
            bool    fid_de;    /**< @0x82 bit7 */
            bool    rgb_mode;  /**< @0x82 bit6 */
            bool    emb_timing;/**< @0x82 bit5 */
            bool    vs_out;    /**< @0x82 bit4 */
            bool    hs_out;    /**< @0x82 bit3 */
            bool    fid_pol;   /**< @0x82 bit2 */
            bool    vs_in;     /**< @0x82 bit1 */
            bool    hs_in;     /**< @0x82 bit0 */
        } ctrl;
    } dtg2;

    /** 0x83–0x85: CGMS Control */
    struct {
        bool    enable;      /**< @0x83 bit6 */
        uint8_t header;      /**< @0x83 bits5-0 */
        uint16_t payload;    /**< @0x84–0x85 (14-bit) */
    } cgms;

    /** 0x86–0x89: Readback */
    struct {
        uint16_t ppl;        /**< @0x86–0x87 */
        uint16_t lpf;        /**< @0x88–0x89 */
    } readback;
} ths8200_regs_t;

#endif /* THS8200_REGS_H */

