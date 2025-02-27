#pragma once

// Build options: Uncomment any of these, or set at the command line to configure:

// If defined, renders a performance overlay on top of the screen. This option is passed from CMake
// configuration script. If you are getting statistics printed on screen
// building with
//   'make VERBOSE=1'
// to see which config flags are coming from CMake to the build.

// How often the on-screen statistics is refreshed (in usecs)
// How many usecs worth of past frame rate data do we preserve in the history buffer. Higher values
// make the frame rate display counter smoother and respond to changes with a delay, whereas smaller
// values can make the display fluctuate a bit erratically.
#define FRAMERATE_HISTORY_LENGTH 400000

// If enabled, displays a visual graph of frame completion times

// If defined, no sleeps are specified and the code runs as fast as possible. This should not improve
// performance, as the code has been developed with the mindset that sleeping should only occur at
// times when there is no work to do, rather than sleeping to reduce power usage. The only expected
// effect of this is that CPU usage shoots to 200%, while display update FPS is the same. Present
// here to allow toggling to debug this assumption.
// #define NO_THROTTLING

// If defined, display updates are synced to the vsync signal provided by the VideoCore GPU. That seems
// to occur quite precisely at 60 Hz. Testing on PAL NES games that run at 50Hz, this will not work well,
// since they produce new frames at every 20msecs, and the VideoCore functions for snapshotting also will
// output new frames at this vsync-detached interval, so there's a 50 Hz vs 60 Hz mismatch that results
// in visible microstuttering. Still, providing this as an option, this might be good for content that
// is known to run at native 60Hz.
// #define USE_GPU_VSYNC

// Always enable GPU VSync on the Pi Zero. Even though it is suboptimal and can cause stuttering, it saves battery.

// If enabled, the source video frame is not scaled to fit to the screen, but instead if the source frame
// is bigger than the SPI display, then content is cropped away, i.e. the source is displayed "centered"
// on the SPI screen:
// #define DISPLAY_CROPPED_INSTEAD_OF_SCALING

// If enabled, the main thread and SPI thread are executed with realtime priority
// #define RUN_WITH_REALTIME_THREAD_PRIORITY

// If defined, progressive updating is always used (at the expense of slowing down refresh rate if it's
// too much for the display to handle)
// #define NO_INTERLACING

// If defined, all frames are always rendered as interlaced, and never use progressive rendering.
// #define ALWAYS_INTERLACING

// By default, if the SPI bus is idle after rendering an interlaced frame, but the GPU has not yet produced
// a new application frame to be displayed, the same frame will be rendered again for its other field.
// Define this option to disable this behavior, in which case when an interlaced frame is rendered, the 
// remaining other field half of the image will never be uploaded.
// #define THROTTLE_INTERLACING

// The ILI9486 has to resort to interlacing as a rule rather than exception, and it works much smoother
// when applying throttling to interlacing, so enable it by default there.
#if defined(ILI9486) || defined(HX8357D)
#define THROTTLE_INTERLACING
#endif

// If defined, DMA usage is foremost used to save power consumption and CPU usage. If not defined,
// DMA usage is tailored towards maximum performance.
// #define ALL_TASKS_SHOULD_DMA

// If defined, screen updates are performed in strictly one update rectangle per frame.
// This reduces CPU consumption at the expense of sending more pixels. You can try enabling this
// if your SPI display runs at a good high SPI bus MHz speed with respect to the screen resolution.
// Useful on Pi Zero W and ILI9341 to conserve CPU power. If this is not defined, the default much
// more powerful diffing algorithm is used, which sends far fewer pixels each frame, (but that diffing
// costs more CPU time). Enabling this requires that ALL_TASKS_SHOULD_DMA is also enabled.
// #define UPDATE_FRAMES_IN_SINGLE_RECTANGULAR_DIFF

// If UPDATE_FRAMES_IN_SINGLE_RECTANGULAR_DIFF is used, controls whether the generated tasks are aligned for
// ARMv6 cache lines. This is good to be enabled for ARMv6 Pis, doesn't make much difference on ARMv7 and ARMv8 Pis.
#define ALIGN_DIFF_TASKS_FOR_32B_CACHE_LINES

// If defined, screen updates are performend without performing diffing at all, i.e. by doing
// full updates. This is very lightweight on CPU, but excessive on the SPI bus. Enabling this
// requires that ALL_TASKS_SHOULD_DMA is also enabled.
// #define UPDATE_FRAMES_WITHOUT_DIFFING


// If per-pixel diffing is enabled (neither UPDATE_FRAMES_IN_SINGLE_RECTANGULAR_DIFF or UPDATE_FRAMES_WITHOUT_DIFFING
// are enabled), the following variable controls whether to lean towards more precise pixel diffing, or faster, but
// coarser pixel diffing. Coarse method is twice as fast than the precise method, but submits slightly more pixels.
// In most cases it is better to use the coarse method, since the increase in pixel counts is small (~5%-10%),
// so enabled by default. If your display is very constrained on SPI bus speed, and don't mind increased
// CPU consumption, comment this out to use the precise algorithm.
#define FAST_BUT_COARSE_PIXEL_DIFF

// If defined, the GPU polling thread will be put to sleep for 1/TARGET_FRAMERATE seconds after receiving
// each new GPU frame, to wait for the earliest moment that the next frame could arrive.
#define SAVE_BATTERY_BY_SLEEPING_UNTIL_TARGET_FRAME

// Detects when the activity on the screen is mostly idle, and goes to low power mode, in which new
// frames will be polled first at 10fps, and ultimately at only 2fps.
#define SAVE_BATTERY_BY_SLEEPING_WHEN_IDLE

// Builds a histogram of observed frame intervals and uses that to sync to a known update rate. This aims
// to detect if an application uses a non-60Hz update rate, and synchronizes to that instead.
#define SAVE_BATTERY_BY_PREDICTING_FRAME_ARRIVAL_TIMES

// If defined, rotates the display 180 degrees. This might not rotate the panel scan order though,
// so adding this can cause up to one vsync worth of extra display latency. It is best to avoid this and
// install the display in its natural rotation order, if possible.
// #define DISPLAY_ROTATE_180_DEGREES

// If defined, displays in landscape. Undefine to display in portrait.
#define DISPLAY_OUTPUT_LANDSCAPE

// If defined, the source video frame is scaled to fit the SPI display by stretching to fit, ignoring
// aspect ratio. Enabling this will cause e.g. 16:9 1080p source to be stretched to fully cover
// a 4:3 320x240 display. If disabled, scaling is performed preserving aspect ratio, so letterboxes or
// pillarboxes will be introduced if needed to retain proper width and height proportions.
// #define DISPLAY_BREAK_ASPECT_RATIO_WHEN_SCALING

// If defined, reverses RGB<->BGR color subpixel order. This is something that seems to be display panel
// specific, rather than display controller specific, and displays manufactured with the same controller
// can have different subpixel order (without the controller taking it automatically into account).
// If display colors come out reversed in blue vs red channels, define this to swap the two.
// #define DISPLAY_SWAP_BGR

// If defined, inverts display pixel colors (31=black, 0=white). Default is to have (0=black, 31=white)
// Pass this if the colors look like a photo negative of the actual image.
// #define DISPLAY_INVERT_COLORS

// If defined, flipping the display between portrait<->landscape is done in software, rather than
// asking the display controller to adjust its RAM write direction.
// Doing the flip in software reduces tearing, since neither the ILI9341 nor ILI9486 displays (and
// probably no other displays in existence?) allow one to adjust the direction that the scanline refresh
// cycle runs in, but the scanline refresh always runs in portrait mode in these displays. Not having
// this defined reduces CPU usage at the expense of more tearing, although it is debatable which
// effect is better - this can be subjective. Impact is around 0.5-1.0msec of extra CPU time.
// DISPLAY_FLIP_ORIENTATION_IN_SOFTWARE disabled: diagonal tearing
// DISPLAY_FLIP_ORIENTATION_IN_SOFTWARE enabled: traditional no-vsync tearing (tear line runs in portrait
// i.e. narrow direction)
#if !defined(SINGLE_CORE_BOARD)
#define DISPLAY_FLIP_ORIENTATION_IN_SOFTWARE
#endif

// If enabled, build to utilize DMA transfers to communicate with the SPI peripheral. Otherwise polling
// writes will be performed (possibly with interrupts, if using kernel side driver module)

// If defined, enables code to manage the backlight.
// #define BACKLIGHT_CONTROL

// If defined, enable a low battery icon triggered by a GPIO pin whose BCM number is given.
// #define LOW_BATTERY_PIN 19

// Which state of the LOW_BATTERY_PIN is considered to be low battery. Note that the GPIO pin must be
// in the correct state (input with pull-up/pull-down resistor) before the program is started.
#define LOW_BATTERY_IS_ACTIVE_HIGH 0

// Polling interval (in micro-second) for the low battery pin.
#define LOW_BATTERY_POLLING_INTERVAL 1000000

// If less than this much % of the screen changes per frame, the screen is considered to be inactive, and
// the display backlight can automatically turn off, if TURN_DISPLAY_OFF_AFTER_USECS_OF_INACTIVITY is 
// defined.
#define DISPLAY_CONSIDERED_INACTIVE_PERCENTAGE (5.0 / 100.0)

#ifndef KERNEL_MODULE

// Define this if building the client side program to run against the kernel driver module, rather than
// as a self-contained userland program.
// #define KERNEL_MODULE_CLIENT

#endif

// Experimental/debugging: If defined, let the userland side program create and run the SPI peripheral
// driving thread. Otherwise, let the kernel drive SPI (e.g. via interrupts or its own thread)
// This should be unset, only available for debugging.
// #define KERNEL_MODULE_CLIENT_DRIVES
