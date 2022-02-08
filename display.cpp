#include "config.h"
#include "display.h"
#include "spi.h"

#include <memory.h>

void ClearScreen()
{
  for(int y = 0; y < DISPLAY_HEIGHT; ++y)
  {
    SPI_TRANSFER(DISPLAY_SET_CURSOR_X, 0, 0, 0, 0, 0, (DISPLAY_WIDTH-1) >> 8, 0, (DISPLAY_WIDTH-1) & 0xFF);
    SPI_TRANSFER(DISPLAY_SET_CURSOR_Y, 0, (uint8_t)(y >> 8), 0, (uint8_t)(y & 0xFF), 0, (DISPLAY_HEIGHT-1) >> 8, 0, (DISPLAY_HEIGHT-1) & 0xFF);

    SPITask *clearLine = AllocTask(DISPLAY_WIDTH*SPI_BYTESPERPIXEL);
    clearLine->cmd = DISPLAY_WRITE_PIXELS;
    memset(clearLine->data, 0, clearLine->size);
    CommitTask(clearLine);
    RunSPITask(clearLine);
    DoneTask(clearLine);
  }
  SPI_TRANSFER(DISPLAY_SET_CURSOR_X, 0, 0, 0, 0, 0, (DISPLAY_WIDTH-1) >> 8, 0, (DISPLAY_WIDTH-1) & 0xFF);
  SPI_TRANSFER(DISPLAY_SET_CURSOR_Y, 0, 0, 0, 0, 0, (DISPLAY_HEIGHT-1) >> 8, 0, (DISPLAY_HEIGHT-1) & 0xFF);
}

void RandomizeScreen() {
    for (int y = 0; y < DISPLAY_HEIGHT; ++y) {
        SPI_TRANSFER(DISPLAY_SET_CURSOR_X, 0, 0, 0, 0, 0, (DISPLAY_WIDTH - 1) >> 8, 0, (DISPLAY_WIDTH - 1) & 0xFF);
        SPI_TRANSFER(DISPLAY_SET_CURSOR_Y, 0, (uint8_t) (y >> 8), 0, (uint8_t) (y & 0xFF), 0, (DISPLAY_HEIGHT - 1) >> 8,
                     0, (DISPLAY_HEIGHT - 1) & 0xFF);

        SPITask *clearLine = AllocTask(DISPLAY_WIDTH * SPI_BYTESPERPIXEL);
        clearLine->cmd = DISPLAY_WRITE_PIXELS;

        for (int i = 0; i < DISPLAY_WIDTH; ++i)
            clearLine->data[i] = tick() * y + i;

//    memset(clearLine->data, 0, clearLine->size);
        CommitTask(clearLine);
        RunSPITask(clearLine);
        DoneTask(clearLine);
    }
    SPI_TRANSFER(DISPLAY_SET_CURSOR_X, 0, 0, 0, 0, 0, (DISPLAY_WIDTH - 1) >> 8, 0, (DISPLAY_WIDTH - 1) & 0xFF);
    SPI_TRANSFER(DISPLAY_SET_CURSOR_Y, 0, 0, 0, 0, 0, (DISPLAY_HEIGHT - 1) >> 8, 0, (DISPLAY_HEIGHT - 1) & 0xFF);
}

