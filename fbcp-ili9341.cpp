#include <fcntl.h>
#include <linux/fb.h>
#include <linux/futex.h>
#include <linux/spi/spidev.h>
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <endian.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>

#include "config.h"
#include "spi.h"
#include "tick.h"
#include "display.h"
#include "util.h"
#include "mem_alloc.h"


volatile bool programRunning = true;

const char *SignalToString(int signal) {
    if (signal == SIGINT) return "SIGINT";
    if (signal == SIGQUIT) return "SIGQUIT";
    if (signal == SIGUSR1) return "SIGUSR1";
    if (signal == SIGUSR2) return "SIGUSR2";
    if (signal == SIGTERM) return "SIGTERM";
    return "?";
}

void MarkProgramQuitting() {
    programRunning = false;
}

void ProgramInterruptHandler(int signal) {
    printf("Signal %s(%d) received, quitting\n", SignalToString(signal), signal);
    static int quitHandlerCalled = 0;
    if (++quitHandlerCalled >= 5) {
        printf("Ctrl-C handler invoked five times, looks like fbcp-ili9341 is not gracefully quitting - performing a forcible shutdown!\n");
        exit(1);
    }
    MarkProgramQuitting();
    __sync_synchronize();
    // Wake the SPI thread if it was sleeping so that it can gracefully quit
    if (spiTaskMemory) {
        __atomic_fetch_add(&spiTaskMemory->queueHead, 1, __ATOMIC_SEQ_CST);
        __atomic_fetch_add(&spiTaskMemory->queueTail, 1, __ATOMIC_SEQ_CST);
        syscall(SYS_futex, &spiTaskMemory->queueTail, FUTEX_WAKE, 1, 0, 0,
                0); // Wake the SPI thread if it was sleeping to get new tasks
    }
}


void drawScreen(int z) {
    for (int y = 0; y < DISPLAY_HEIGHT; ++y) {
        SPI_TRANSFER(DISPLAY_SET_CURSOR_X, 0, 0, 0, 0, 0, (DISPLAY_WIDTH - 1) >> 8, 0, (DISPLAY_WIDTH - 1) & 0xFF);
        SPI_TRANSFER(DISPLAY_SET_CURSOR_Y, 0, (uint8_t) (y >> 8), 0, (uint8_t) (y & 0xFF), 0, (DISPLAY_HEIGHT - 1) >> 8,
                     0, (DISPLAY_HEIGHT - 1) & 0xFF);

        SPITask *clearLine = AllocTask(DISPLAY_WIDTH * SPI_BYTESPERPIXEL);
        clearLine->cmd = DISPLAY_WRITE_PIXELS;

        for (int i = 0; i < DISPLAY_WIDTH; ++i)
            clearLine->data[i] = z * y + i;

//    memset(clearLine->data, 0, clearLine->size);
        CommitTask(clearLine);
        RunSPITask(clearLine);
        DoneTask(clearLine);
    }
    SPI_TRANSFER(DISPLAY_SET_CURSOR_X, 0, 0, 0, 0, 0, (DISPLAY_WIDTH - 1) >> 8, 0, (DISPLAY_WIDTH - 1) & 0xFF);
    SPI_TRANSFER(DISPLAY_SET_CURSOR_Y, 0, 0, 0, 0, 0, (DISPLAY_HEIGHT - 1) >> 8, 0, (DISPLAY_HEIGHT - 1) & 0xFF);
}

int main() {
    signal(SIGINT, ProgramInterruptHandler);
    signal(SIGQUIT, ProgramInterruptHandler);
    signal(SIGUSR1, ProgramInterruptHandler);
    signal(SIGUSR2, ProgramInterruptHandler);
    signal(SIGTERM, ProgramInterruptHandler);

    InitSPI();
    usleep(3000 * 1000);
//    for (int z = 0; z < 5; z++) {
//        usleep(200 * 1000);
//        drawScreen(z);
//    }
    DeinitSPI();
    printf("Quit.\n");
}
