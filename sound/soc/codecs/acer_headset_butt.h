/*
 * Acer Headset button detection driver.
 *
 * Copyright (C) 2012 Acer Corporation.
 *
 * Authors:
 *    Eric Cheng <Eric_Cheng@acer.com.tw>
 */
#ifndef __ACER_HEADSET_BUTT_H
#define __ACER_HEADSET_BUTT_H

extern int acer_hs_butt_init(void);
extern int acer_hs_butt_remove(void);
extern int get_mpp_hs_mic_adc(u64 *val);
extern void set_hs_state(bool state);

#endif
