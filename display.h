#pragma once

#include "config.h"

// Configure the desired display update rate. Use 120 for max performance/minimized latency, and 60/50/30/24 etc. for regular content, or to save battery.
#define TARGET_FRAME_RATE 60

#include "ili9486.h"

// The native display resolution is in portrait/landscape, but we want to display in the opposite landscape/portrait orientation?
// Compare DISPLAY_NATIVE_WIDTH <= DISPLAY_NATIVE_HEIGHT in the first test to let users toggle DISPLAY_OUTPUT_LANDSCAPE directive in config.h to flip orientation on square displays with width=height
#if ((DISPLAY_NATIVE_WIDTH <= DISPLAY_NATIVE_HEIGHT && defined(DISPLAY_OUTPUT_LANDSCAPE)) || (DISPLAY_NATIVE_WIDTH > DISPLAY_NATIVE_HEIGHT && !defined(DISPLAY_OUTPUT_LANDSCAPE)))
#define DISPLAY_SHOULD_FLIP_ORIENTATION
#endif

#if defined(DISPLAY_SHOULD_FLIP_ORIENTATION) && !defined(DISPLAY_FLIP_ORIENTATION_IN_SOFTWARE)
// Need to do orientation flip, but don't want to do it on the CPU, so pretend the display dimensions are of the flipped form,
// and use display controller initialization sequence to do the flipping
#define DISPLAY_WIDTH DISPLAY_NATIVE_HEIGHT
#define DISPLAY_HEIGHT DISPLAY_NATIVE_WIDTH
#define DISPLAY_FLIP_ORIENTATION_IN_HARDWARE
#else
#define DISPLAY_WIDTH DISPLAY_NATIVE_WIDTH
#define DISPLAY_HEIGHT DISPLAY_NATIVE_HEIGHT
#endif

#if !defined(DISPLAY_SHOULD_FLIP_ORIENTATION) && defined(DISPLAY_FLIP_ORIENTATION_IN_SOFTWARE)
#undef DISPLAY_FLIP_ORIENTATION_IN_SOFTWARE
#endif

#ifndef DISPLAY_NATIVE_COVERED_LEFT_SIDE
#define DISPLAY_NATIVE_COVERED_LEFT_SIDE 0
#endif

#ifndef DISPLAY_NATIVE_COVERED_TOP_SIDE
#define DISPLAY_NATIVE_COVERED_TOP_SIDE 0
#endif

#ifndef DISPLAY_NATIVE_COVERED_BOTTOM_SIDE
#define DISPLAY_NATIVE_COVERED_BOTTOM_SIDE 0
#endif

#ifndef DISPLAY_NATIVE_COVERED_RIGHT_SIDE
#define DISPLAY_NATIVE_COVERED_RIGHT_SIDE 0
#endif

#if defined(DISPLAY_FLIP_ORIENTATION_IN_SOFTWARE) || !defined(DISPLAY_SHOULD_FLIP_ORIENTATION)
#define DISPLAY_COVERED_TOP_SIDE DISPLAY_NATIVE_COVERED_TOP_SIDE
#define DISPLAY_COVERED_LEFT_SIDE DISPLAY_NATIVE_COVERED_LEFT_SIDE
#define DISPLAY_COVERED_RIGHT_SIDE DISPLAY_NATIVE_COVERED_RIGHT_SIDE
#define DISPLAY_COVERED_BOTTOM_SIDE DISPLAY_NATIVE_COVERED_BOTTOM_SIDE
#else
#define DISPLAY_COVERED_TOP_SIDE DISPLAY_NATIVE_COVERED_LEFT_SIDE
#define DISPLAY_COVERED_LEFT_SIDE DISPLAY_NATIVE_COVERED_TOP_SIDE
#define DISPLAY_COVERED_RIGHT_SIDE DISPLAY_NATIVE_COVERED_BOTTOM_SIDE
#define DISPLAY_COVERED_BOTTOM_SIDE DISPLAY_NATIVE_COVERED_RIGHT_SIDE
#endif

#define DISPLAY_DRAWABLE_WIDTH (DISPLAY_WIDTH-DISPLAY_COVERED_LEFT_SIDE-DISPLAY_COVERED_RIGHT_SIDE)
#define DISPLAY_DRAWABLE_HEIGHT (DISPLAY_HEIGHT-DISPLAY_COVERED_TOP_SIDE-DISPLAY_COVERED_BOTTOM_SIDE)

#ifndef DISPLAY_SPI_DRIVE_SETTINGS
#define DISPLAY_SPI_DRIVE_SETTINGS (0)
#endif

// 16 bits per pixel
#define SPI_BYTESPERPIXEL 2

void ClearScreen(void);

void RandomizeScreen(void);

void TurnBacklightOn(void);

void TurnBacklightOff(void);

void TurnDisplayOn(void);

void TurnDisplayOff(void);

void DeinitSPIDisplay(void);
