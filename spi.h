#pragma once

#include <inttypes.h>
#include <sys/syscall.h>

#include <linux/futex.h>

#include "display.h"
#include "tick.h"
#include "display.h"

#define BCM2835_GPIO_BASE                    0x200000   // Address to GPIO register file
#define BCM2835_SPI0_BASE                    0x204000   // Address to SPI0 register file
#define BCM2835_TIMER_BASE                   0x3000     // Address to System Timer register file

#define BCM2835_SPI0_CS_RXF                  0x00100000 // Receive FIFO is full
#define BCM2835_SPI0_CS_RXR                  0x00080000 // FIFO needs reading
#define BCM2835_SPI0_CS_TXD                  0x00040000 // TXD TX FIFO can accept Data
#define BCM2835_SPI0_CS_RXD                  0x00020000 // RXD RX FIFO contains Data
#define BCM2835_SPI0_CS_DONE                 0x00010000 // Done transfer Done
#define BCM2835_SPI0_CS_TA                   0x00000080 // Transfer Active
#define BCM2835_SPI0_CS_CLEAR                0x00000030 // Clear FIFO Clear RX and TX
#define BCM2835_SPI0_CS_CLEAR_RX             0x00000020 // Clear FIFO Clear RX

#define GPIO_SPI0_MOSI  10        // Pin P1-19, MOSI when SPI0 in use
#define GPIO_SPI0_MISO   9        // Pin P1-21, MISO when SPI0 in use
#define GPIO_SPI0_CLK   11        // Pin P1-23, CLK when SPI0 in use
#define GPIO_SPI0_CE0    8        // Pin P1-24, CE0 when SPI0 in use
#define GPIO_SPI0_CE1    7        // Pin P1-26, CE1 when SPI0 in use

extern volatile void *bcm2835;

typedef struct GPIORegisterFile {
    uint32_t gpfsel[6], reserved0; // GPIO Function Select registers, 3 bits per pin, 10 pins in an uint32_t
    uint32_t gpset[2], reserved1; // GPIO Pin Output Set registers, write a 1 to bit at index I to set the pin at index I high
    uint32_t gpclr[2], reserved2; // GPIO Pin Output Clear registers, write a 1 to bit at index I to set the pin at index I low
    uint32_t gplev[2];
} GPIORegisterFile;
extern volatile GPIORegisterFile *gpio;

#define SET_GPIO_MODE(pin, mode) gpio->gpfsel[(pin)/10] = (gpio->gpfsel[(pin)/10] & ~(0x7 << ((pin) % 10) * 3)) | ((mode) << ((pin) % 10) * 3)
#define SET_GPIO(pin) gpio->gpset[0] = 1 << (pin) // Pin must be (0-31)
#define CLEAR_GPIO(pin) gpio->gpclr[0] = 1 << (pin) // Pin must be (0-31)

typedef struct SPIRegisterFile {
    uint32_t cs;   // SPI Master Control and Status register
    uint32_t fifo; // SPI Master TX and RX FIFOs
    uint32_t clk;  // SPI Master Clock Divider
    uint32_t dlen; // SPI Master Number of DMA Bytes to Write
} SPIRegisterFile;
extern volatile SPIRegisterFile *spi;

// Defines the size of the SPI task memory buffer in bytes. This memory buffer can contain two frames worth of tasks at maximum,
// so for best performance, should be at least ~DISPLAY_WIDTH*DISPLAY_HEIGHT*BYTES_PER_PIXEL*2 bytes in size, plus some small
// amount for structuring each SPITask command. Technically this can be something very small, like 4096b, and not need to contain
// even a single full frame of data, but such small buffers can cause performance issues from threads starving.
#define SHARED_MEMORY_SIZE (DISPLAY_DRAWABLE_WIDTH*DISPLAY_DRAWABLE_HEIGHT*SPI_BYTESPERPIXEL*3)
#define SPI_QUEUE_SIZE (SHARED_MEMORY_SIZE - sizeof(SharedMemory))

typedef struct __attribute__((packed)) SPITask {
    uint32_t size; // Size, including both 8-bit and 9-bit tasks
    uint8_t cmd;
    uint8_t data[]; // Contains both 8-bit and 9-bit tasks back to back, 8-bit first, then 9-bit.

    inline uint8_t *PayloadStart() { return data; }

    inline uint8_t *PayloadEnd() { return data + size; }

    inline uint32_t PayloadSize() const { return size; }

} SPITask;

#define BEGIN_SPI_COMMUNICATION() do { spi->cs = BCM2835_SPI0_CS_TA | DISPLAY_SPI_DRIVE_SETTINGS; } while(0)
#define END_SPI_COMMUNICATION()  do { \
    uint32_t cs; \
    while (!(((cs = spi->cs) ^ BCM2835_SPI0_CS_TA) & (BCM2835_SPI0_CS_DONE | BCM2835_SPI0_CS_TA))) /* While TA=1 and DONE=0*/ \
    { \
      if ((cs & (BCM2835_SPI0_CS_RXR | BCM2835_SPI0_CS_RXF))) \
        spi->cs = BCM2835_SPI0_CS_CLEAR_RX | BCM2835_SPI0_CS_TA | DISPLAY_SPI_DRIVE_SETTINGS; \
    } \
    spi->cs = BCM2835_SPI0_CS_CLEAR_RX | DISPLAY_SPI_DRIVE_SETTINGS; /* Clear TA and any pending bytes */ \
  } while(0)

// A convenience for defining and dispatching SPI task bytes inline
#define SPI_TRANSFER(command, ...) do { \
    char data_buffer[] = { __VA_ARGS__ }; \
    SPITask *t = AllocTask(sizeof(data_buffer)); \
    t->cmd = (command); \
    memcpy(t->data, data_buffer, sizeof(data_buffer)); \
    CommitTask(t); \
    RunSPITask(t); \
    DoneTask(t); \
  } while(0)

#define QUEUE_SPI_TRANSFER(command, ...) do { \
    char data_buffer[] = { __VA_ARGS__ }; \
    SPITask *t = AllocTask(sizeof(data_buffer)); \
    t->cmd = (command); \
    memcpy(t->data, data_buffer, sizeof(data_buffer)); \
    CommitTask(t); \
  } while(0)

typedef struct SharedMemory {
    volatile uint32_t queueHead;
    volatile uint32_t queueTail;
    volatile uint32_t spiBytesQueued; // Number of actual payload bytes in the queue
    volatile uint32_t interruptsRaised;
    volatile uintptr_t sharedMemoryBaseInPhysMemory;
    volatile uint8_t buffer[];
} SharedMemory;

extern SharedMemory *spiTaskMemory;

extern int mem_fd;

static inline SPITask *AllocTask(uint32_t bytes) // Returns a pointer to a new SPI task block, called on main thread
{

    uint32_t bytesToAllocate = sizeof(SPITask) + bytes;// + totalBytesFor9BitTask;
    uint32_t tail = spiTaskMemory->queueTail;
    uint32_t newTail = tail + bytesToAllocate;
    // Is the new task too large to write contiguously into the ring buffer, that it's split into two parts? We never split,
    // but instead write a sentinel at the end of the ring buffer, and jump the tail back to the beginning of the buffer and
    // allocate the new task there. However in doing so, we must make sure that we don't write over the head marker.
    if (newTail + sizeof(SPITask)/*Add extra SPITask size so that there will always be room for eob marker*/ >=
        SPI_QUEUE_SIZE) {
        uint32_t head = spiTaskMemory->queueHead;
        // Write a sentinel, but wait for the head to advance first so that it is safe to write.
        while (head > tail || head == 0/*Head must move > 0 so that we don't stomp on it*/) {
            head = spiTaskMemory->queueHead;
        }
        SPITask *endOfBuffer = (SPITask *) (spiTaskMemory->buffer + tail);
        endOfBuffer->cmd = 0; // Use cmd=0x00 to denote "end of buffer, wrap to beginning"
        __sync_synchronize();
        spiTaskMemory->queueTail = 0;
        __sync_synchronize();
        if (spiTaskMemory->queueHead == tail)
            syscall(SYS_futex, &spiTaskMemory->queueTail, FUTEX_WAKE, 1, 0, 0,
                    0); // Wake the SPI thread if it was sleeping to get new tasks
        tail = 0;
        newTail = bytesToAllocate;
    }

    // If the SPI task queue is full, wait for the SPI thread to process some tasks. This throttles the main thread to not run too fast.
    uint32_t head = spiTaskMemory->queueHead;
    while (head > tail && head <= newTail) {
        usleep(100); // Since the SPI queue is full, we can afford to sleep a bit on the main thread without introducing lag.
        head = spiTaskMemory->queueHead;
    }

    SPITask *task = (SPITask *) (spiTaskMemory->buffer + tail);
    task->size = bytes;
    return task;
}

static inline void
CommitTask(SPITask *task) // Advertises the given SPI task from main thread to worker, called on main thread
{
    __sync_synchronize();
    uint32_t tail = spiTaskMemory->queueTail;
    spiTaskMemory->queueTail = (uint32_t) ((uint8_t *) task - spiTaskMemory->buffer) + sizeof(SPITask) + task->size;
    __atomic_fetch_add(&spiTaskMemory->spiBytesQueued, task->PayloadSize() + 1, __ATOMIC_RELAXED);
    __sync_synchronize();
    if (spiTaskMemory->queueHead == tail)
        syscall(SYS_futex, &spiTaskMemory->queueTail, FUTEX_WAKE, 1, 0, 0,
                0); // Wake the SPI thread if it was sleeping to get new tasks
}

int InitSPI(void);

void DeinitSPI(void);

void RunSPITask(SPITask *task);

void DoneTask(SPITask *task);
