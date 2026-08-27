#pragma once
#include <stdint.h>

// Open Miramar thermal camera control port (GenCP over serial).
// Default device in main.cpp is "/dev/ttyACM1" (you can change it there).
int miramar_open(const char* dev);

// Close control port.
void miramar_close();

// Set AGC / CLAHE enable bits in TM_CONTROL (0x2C000000).
// Only modifies bit0 (AGC) and bit1 (CLAHE), preserving all other bits.
int miramar_set_agc_clahe(int agc_on, int clahe_on);

// Read current AGC / CLAHE enable bits from TM_CONTROL (0x2C000000).
// Returns 0 on success; -1 on failure.
int miramar_get_agc_clahe(int *agc_on, int *clahe_on);

//mmlab 
int miramar_set_color(int *color);
int miramar_read_ctl_roi(int *ctl_roi);


// Return 1 if control port is open, else 0.
int miramar_is_open(void);
