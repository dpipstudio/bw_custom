/*
 * BotWave Custom - FM/RDS transmitter for the Raspberry Pi
 * 
 * Based on PiFmRds by Christophe Jacquet, F8FTK
 * 
 * Copyright (C) 2025, douxx@douxx.tech
 * Copyright (C) 2014, 2015 Christophe Jacquet, F8FTK
 * Copyright (C) 2012, 2015 Richard Hirst
 * Copyright (C) 2012 Oliver Mattos and Oskar Weigl
 *
 * See: https://github.com/dpipstudio/BWCustom
 * 
 * Original project: https://github.com/ChristopheJacquet/PiFmRds
 *
 * PI-FM-RDS: RaspberryPi FM transmitter, with RDS.
 *
 * This file contains the VHF FM modulator. All credit goes to the original
 * authors, Oliver Mattos and Oskar Weigl for the original idea, and to
 * Richard Hirst for using the Pi's DMA engine, which reduced CPU usage
 * dramatically.
 *
 * I (Christophe Jacquet) have adapted their idea to transmitting samples
 * at 228 kHz, allowing to build the 57 kHz subcarrier for RDS BPSK data.
 *
 * To make it work on the Raspberry Pi 2, I used a fix by Richard Hirst
 * (again) to request memory using Broadcom's mailbox interface. This fix
 * was published for ServoBlaster here:
 * https://www.raspberrypi.org/forums/viewtopic.php?p=699651#p699651
 *
 * Never use this to transmit VHF-FM data through an antenna, as it is
 * illegal in most countries. This code is for testing purposes only.
 * Always connect a shielded transmission line from the RaspberryPi directly
 * to a radio receiver, so as *not* to emit radio waves.
 *
 * ---------------------------------------------------------------------------
 * These are the comments from Richard Hirst's version:
 *
 * RaspberryPi based FM transmitter.  For the original idea, see:
 *
 * http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter
 *
 * All credit to Oliver Mattos and Oskar Weigl for creating the original code.
 *
 * I have taken their idea and reworked it to use the Pi DMA engine, so
 * reducing the CPU overhead for playing a .wav file from 100% to about 1.6%.
 *
 * I have implemented this in user space, using an idea I picked up from Joan
 * on the Raspberry Pi forums - credit to Joan for the DMA from user space
 * idea.
 *
 * The idea of feeding the PWM FIFO in order to pace DMA control blocks comes
 * from ServoBlaster, and I take credit for that :-)
 *
 * This code uses DMA channel 0 and the PWM hardware, with no regard for
 * whether something else might be trying to use it at the same time (such as
 * the 3.5mm jack audio driver).
 *
 * I know nothing much about sound, subsampling, or FM broadcasting, so it is
 * quite likely the sound quality produced by this code can be improved by
 * someone who knows what they are doing.  There may be issues realting to
 * caching, as the user space process just writes to its virtual address space,
 * and expects the DMA controller to see the data; it seems to work for me
 * though.
 *
 * NOTE: THIS CODE MAY WELL CRASH YOUR PI, TRASH YOUR FILE SYSTEMS, AND
 * POTENTIALLY EVEN DAMAGE YOUR HARDWARE.  THIS IS BECAUSE IT STARTS UP THE DMA
 * CONTROLLER USING MEMORY OWNED BY A USER PROCESS.  IF THAT USER PROCESS EXITS
 * WITHOUT STOPPING THE DMA CONTROLLER, ALL HELL COULD BREAK LOOSE AS THE
 * MEMORY GETS REALLOCATED TO OTHER PROCESSES WHILE THE DMA CONTROLLER IS STILL
 * USING IT.  I HAVE ATTEMPTED TO MINIMISE ANY RISK BY CATCHING SIGNALS AND
 * RESETTING THE DMA CONTROLLER BEFORE EXITING, BUT YOU HAVE BEEN WARNED.  I
 * ACCEPT NO LIABILITY OR RESPONSIBILITY FOR ANYTHING THAT HAPPENS AS A RESULT
 * OF YOU RUNNING THIS CODE.  IF IT BREAKS, YOU GET TO KEEP ALL THE PIECES.
 *
 * NOTE ALSO:  THIS MAY BE ILLEGAL IN YOUR COUNTRY.  HERE ARE SOME COMMENTS
 * FROM MORE KNOWLEDGEABLE PEOPLE ON THE FORUM:
 *
 * "Just be aware that in some countries FM broadcast and especially long
 * distance FM broadcast could get yourself into trouble with the law, stray FM
 * broadcasts over Airband aviation is also strictly forbidden."
 *
 * "A low pass filter is really really required for this as it has strong
 * harmonics at the 3rd, 5th 7th and 9th which sit in licensed and rather
 * essential bands, ie GSM, HAM, emergency services and others. Polluting these
 * frequencies is immoral and dangerous, whereas "breaking in" on FM bands is
 * just plain illegal."
 *
 * "Don't get caught, this GPIO use has the potential to exceed the legal
 * limits by about 2000% with a proper aerial."
 *
 *
 * As for the original code, this code is released under the GPL.
 *
 * Richard Hirst <richardghirst@gmail.com>  December 2012
 */

#include <locale.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sndfile.h>

#include "rds.h"
#include "fm_mpx.h"

#include "mailbox.h"
#define MBFILE            DEVICE_FILE_NAME    /* From mailbox.h */

// Hardware addresses differ between Raspberry Pi models
// These macros set the correct memory addresses based on Pi version
#if (RASPI)==1
#define PERIPH_VIRT_BASE 0x20000000  // Virtual base address for peripherals
#define PERIPH_PHYS_BASE 0x7e000000  // Physical base address
#define DRAM_PHYS_BASE 0x40000000    // RAM base address
#define MEM_FLAG 0x0c                // Memory allocation flags
#define PLLFREQ 500000000.           // PLL clock frequency (500 MHz)
#elif (RASPI)==2
#define PERIPH_VIRT_BASE 0x3f000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0xc0000000
#define MEM_FLAG 0x04
#define PLLFREQ 500000000.
#elif (RASPI)==4
#define PERIPH_VIRT_BASE 0xfe000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0xc0000000
#define MEM_FLAG 0x04
#define PLLFREQ 750000000.           // Pi 4 has faster PLL (750 MHz)
#else
#error Unknown Raspberry Pi version (variable RASPI)
#endif

// Number of frequency samples to generate
#define NUM_SAMPLES        50000
// Each sample needs 2 DMA control blocks (one for data, one for timing)
#define NUM_CBS            (NUM_SAMPLES * 2)

// DMA (Direct Memory Access) control flags
// DMA allows hardware to access memory without CPU involvement
#define BCM2708_DMA_NO_WIDE_BURSTS    (1<<26)  // Use single-word transfers
#define BCM2708_DMA_WAIT_RESP        (1<<3)    // Wait for write acknowledgment
#define BCM2708_DMA_D_DREQ        (1<<6)       // Use DREQ (data request) signal
#define BCM2708_DMA_PER_MAP(x)        ((x)<<16) // Map to peripheral
#define BCM2708_DMA_END            (1<<1)       // Transfer complete flag
#define BCM2708_DMA_RESET        (1<<31)       // Reset DMA channel
#define BCM2708_DMA_INT            (1<<2)       // Interrupt flag

// DMA register offsets (divided by 4 because they're 32-bit words)
#define DMA_CS            (0x00/4)  // Control and Status register
#define DMA_CONBLK_AD        (0x04/4)  // Control Block Address register
#define DMA_DEBUG        (0x20/4)  // Debug register

// Hardware module offsets from peripheral base
#define DMA_BASE_OFFSET        0x00007000
#define DMA_LEN            0x24
#define PWM_BASE_OFFSET        0x0020C000  // PWM for timing
#define PWM_LEN            0x28
#define CLK_BASE_OFFSET            0x00101000  // Clock generator
#define CLK_LEN            0xA8
#define GPIO_BASE_OFFSET    0x00200000  // GPIO pins
#define GPIO_LEN        0x100

// Calculate full virtual addresses for each hardware module
#define DMA_VIRT_BASE        (PERIPH_VIRT_BASE + DMA_BASE_OFFSET)
#define PWM_VIRT_BASE        (PERIPH_VIRT_BASE + PWM_BASE_OFFSET)
#define CLK_VIRT_BASE        (PERIPH_VIRT_BASE + CLK_BASE_OFFSET)
#define GPIO_VIRT_BASE        (PERIPH_VIRT_BASE + GPIO_BASE_OFFSET)
#define PCM_VIRT_BASE        (PERIPH_VIRT_BASE + PCM_BASE_OFFSET)

// Calculate physical addresses (used by DMA)
#define PWM_PHYS_BASE        (PERIPH_PHYS_BASE + PWM_BASE_OFFSET)
#define PCM_PHYS_BASE        (PERIPH_PHYS_BASE + PCM_BASE_OFFSET)
#define GPIO_PHYS_BASE        (PERIPH_PHYS_BASE + GPIO_BASE_OFFSET)

// PWM (Pulse Width Modulation) register offsets
#define PWM_CTL            (0x00/4)  // Control register
#define PWM_DMAC        (0x08/4)     // DMA configuration
#define PWM_RNG1        (0x10/4)     // Range register
#define PWM_FIFO        (0x18/4)     // FIFO data register

// PWM Clock registers
#define PWMCLK_CNTL        40
#define PWMCLK_DIV        41

// GPIO clock divider register address
#define CM_GP0DIV (0x7e101074)

// GPIO Clock register offsets
#define GPCLK_CNTL        (0x70/4)  // Control register
#define GPCLK_DIV        (0x74/4)   // Divider register

// PWM control flags
#define PWMCTL_MODE1        (1<<1)  // Use PWM mode
#define PWMCTL_PWEN1        (1<<0)  // Enable PWM channel 1
#define PWMCTL_CLRF        (1<<6)   // Clear FIFO
#define PWMCTL_USEF1        (1<<5)  // Use FIFO

// PWM DMA configuration
#define PWMDMAC_ENAB        (1<<31)  // Enable DMA
#define PWMDMAC_THRSHLD        ((15<<8)|(15<<0))  // FIFO threshold

// GPIO function select register
#define GPFSEL0            (0x00/4)

// Frequency deviation for FM modulation (25 kHz for broadcast FM)
#define DEVIATION        25.0

// DMA Control Block structure
// This tells the DMA controller what to transfer, where, and what to do next
typedef struct {
    uint32_t info;     // Transfer information (flags)
    uint32_t src;      // Source address (where to read from)
    uint32_t dst;      // Destination address (where to write to)
    uint32_t length;   // How many bytes to transfer
    uint32_t stride;   // 2D stride (not used here)
    uint32_t next;     // Address of next control block (for chaining)
    uint32_t pad[2];   // Padding for alignment
} dma_cb_t;

// Convert bus address to physical address (remove cache bits)
#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

// Structure to hold mailbox memory allocation info
// The mailbox is used to request special memory from the GPU
static struct {
    int handle;            // File handle to mailbox device
    unsigned mem_ref;      // Memory reference from allocation
    unsigned bus_addr;     // Bus address (used by DMA)
    uint8_t *virt_addr;    // Virtual address (used by CPU)
} mbox;

// Pointers to hardware registers (volatile = can change unexpectedly)
static volatile uint32_t *pwm_reg;   // PWM registers
static volatile uint32_t *clk_reg;   // Clock registers
static volatile uint32_t *dma_reg;   // DMA registers
static volatile uint32_t *gpio_reg;  // GPIO registers

// Main data structure: holds all DMA control blocks and frequency samples
struct control_data_s {
    dma_cb_t cb[NUM_CBS];           // Array of DMA control blocks
    uint32_t sample[NUM_SAMPLES];   // Array of frequency samples
};

// Memory management constants
#define PAGE_SIZE    4096
#define PAGE_SHIFT    12
#define NUM_PAGES    ((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)

// Pointer to our control data structure
static struct control_data_s *ctl;

// Sleep for specified microseconds
static void udelay(int us) {
    struct timespec ts = { 0, us * 1000 };
    nanosleep(&ts, NULL);
}

// Clean shutdown function - VERY IMPORTANT!
// This stops the DMA and turns off the transmitter
static void terminate(int num) {
    // Stop the clock generator on GPIO4
    if (clk_reg && gpio_reg && mbox.virt_addr) {
        // Change GPIO4 from clock function back to regular output
        gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (1 << 12);

        // Disable the clock (0x5A is the "password" required to modify)
        clk_reg[GPCLK_CNTL] = 0x5A;
    }

    // Stop the DMA engine
    if (dma_reg && mbox.virt_addr) {
        dma_reg[DMA_CS] = BCM2708_DMA_RESET;
        udelay(10);
    }

    // Clean up other resources
    fm_mpx_close();

    // Free the allocated memory
    if (mbox.virt_addr != NULL) {
        unmapmem(mbox.virt_addr, NUM_PAGES * 4096);
        mem_unlock(mbox.handle, mbox.mem_ref);
        mem_free(mbox.handle, mbox.mem_ref);
    }

    printf("Terminating: cleanly deactivated the DMA engine and killed the carrier.\n");
    exit(num);
}

// Print error message and terminate
static void fatal(char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    terminate(0);
}

// Convert virtual address to physical address
// (Virtual = what CPU sees, Physical = what DMA needs)
static size_t mem_virt_to_phys(void *virt) {
    size_t offset = (size_t)virt - (size_t)mbox.virt_addr;
    return mbox.bus_addr + offset;
}

// Convert physical address back to virtual address
static size_t mem_phys_to_virt(size_t phys) {
    return (size_t) (phys - mbox.bus_addr + mbox.virt_addr);
}

// Map a hardware peripheral into our process's memory space
// This lets us access hardware registers directly
static void *map_peripheral(uint32_t base, uint32_t len) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    void * vaddr;

    if (fd < 0)
        fatal("Failed to open /dev/mem: %m.\n");
    
    // mmap = memory map - maps physical memory to virtual memory
    vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
    if (vaddr == MAP_FAILED)
        fatal("Failed to map peripheral at 0x%08x: %m.\n", base);
    close(fd);

    return vaddr;
}

#define SUBSIZE 1
#define DATA_SIZE 5000

// Main transmission function
int tx(uint32_t carrier_freq, char *audio_file, uint16_t pi, char *ps, char *rt, float ppm, int loop_audio) {
    // Set up signal handlers to catch ALL signals
    // This ensures we always clean up properly, even if killed
    for (int i = 0; i < 64; i++) {
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = terminate;
        sigaction(i, &sa, NULL);
    }

    // Map all hardware peripherals into memory
    dma_reg = map_peripheral(DMA_VIRT_BASE, DMA_LEN);
    pwm_reg = map_peripheral(PWM_VIRT_BASE, PWM_LEN);
    clk_reg = map_peripheral(CLK_VIRT_BASE, CLK_LEN);
    gpio_reg = map_peripheral(GPIO_VIRT_BASE, GPIO_LEN);

    // Allocate DMA-capable memory using the mailbox interface
    // This is special memory that both CPU and DMA can access
    mbox.handle = mbox_open();
    if (mbox.handle < 0)
        fatal("Failed to open mailbox. Check kernel support for vcio / BCM2708 mailbox.\n");
    
    printf("Allocating physical memory: size = %zu     ", NUM_PAGES * 4096);
    if(! (mbox.mem_ref = mem_alloc(mbox.handle, NUM_PAGES * 4096, 4096, MEM_FLAG))) {
        fatal("Could not allocate memory.\n");
    }
    
    printf("mem_ref = %u     ", mbox.mem_ref);
    if(! (mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref))) {
        fatal("Could not lock memory.\n");
    }
    
    printf("bus_addr = %x     ", mbox.bus_addr);
    if(! (mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), NUM_PAGES * 4096))) {
        fatal("Could not map memory.\n");
    }
    printf("virt_addr = %p\n", mbox.virt_addr);

    // Configure GPIO4 to output the clock signal
    // ALT FUNC 0 means "use alternate function 0" which is GPCLK0
    gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (4 << 12);

    // Set up the GPIO clock with MASH (noise shaping) enabled
    // 0x5A is a "password" required to modify clock registers
    clk_reg[GPCLK_CNTL] = 0x5A << 24 | 6;  // Set source to PLLD
    udelay(100);
    clk_reg[GPCLK_CNTL] = 0x5A << 24 | 1 << 9 | 1 << 4 | 6;  // Enable MASH and clock

    // Get pointer to our control structure
    ctl = (struct control_data_s *) mbox.virt_addr;
    dma_cb_t *cbp = ctl->cb;  // Pointer to walk through control blocks
    
    // Physical addresses that DMA will write to
    uint32_t phys_sample_dst = CM_GP0DIV;  // Clock divider register
    uint32_t phys_pwm_fifo_addr = PWM_PHYS_BASE + 0x18;  // PWM FIFO

    // Calculate frequency control word
    // This is the base frequency value written to the clock divider
    // Lower 12 bits are fractional part
    uint32_t freq_ctl = ((float)(PLLFREQ / carrier_freq)) * ( 1 << 12 );

    // Build circular buffer of DMA control blocks
    // Each sample needs 2 control blocks:
    //   1. Write the frequency value to the clock divider
    //   2. Wait for PWM timing (this paces the DMA)
    for (int i = 0; i < NUM_SAMPLES; i++) {
        // Initialize sample with silence (just the base frequency)
        ctl->sample[i] = 0x5a << 24 | freq_ctl;
        
        // Control block 1: Write frequency sample to clock divider
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
        cbp->src = mem_virt_to_phys(ctl->sample + i);  // Read from our sample array
        cbp->dst = phys_sample_dst;                     // Write to clock divider
        cbp->length = 4;                                // Transfer 4 bytes (32 bits)
        cbp->stride = 0;                                // No 2D stride
        cbp->next = mem_virt_to_phys(cbp + 1);         // Next control block
        cbp++;
        
        // Control block 2: Delay/timing via PWM FIFO
        // This waits for PWM to request data, which provides precise timing
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | 
                    BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
        cbp->src = mem_virt_to_phys(mbox.virt_addr);   // Dummy read
        cbp->dst = phys_pwm_fifo_addr;                  // Write to PWM FIFO
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(cbp + 1);
        cbp++;
    }
    
    // Make the last control block point back to the first (circular buffer)
    cbp--;
    cbp->next = mem_virt_to_phys(mbox.virt_addr);

    // Configure PWM timing
    // PWM creates the precise 228 kHz sample rate needed for FM transmission
    // The divider calculation compensates for oscillator frequency error (ppm)
    float divider = (PLLFREQ/(2000*228*(1.+ppm/1.e6)));
    uint32_t idivider = (uint32_t) divider;              // Integer part
    uint32_t fdivider = (uint32_t) ((divider - idivider)*pow(2, 12));  // Fractional part

    printf("ppm corr is %.4f, divider is %.4f (%d + %d*2^-12) [nominal 1096.4912].\n",
                ppm, divider, idivider, fdivider);

    // Configure PWM
    pwm_reg[PWM_CTL] = 0;  // Stop PWM
    udelay(10);
    
    // Set PWM clock source to PLLD and disable
    clk_reg[PWMCLK_CNTL] = 0x5A000006;
    udelay(100);
    
    // Set the divider (determines sample rate)
    clk_reg[PWMCLK_DIV] = 0x5A000000 | (idivider<<12) | fdivider;
    udelay(100);
    
    // Enable PWM clock with MASH filter
    clk_reg[PWMCLK_CNTL] = 0x5A000216;
    udelay(100);
    
    // Set PWM range and enable DMA
    pwm_reg[PWM_RNG1] = 2;
    udelay(10);
    pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_CLRF;  // Clear FIFO
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;  // Enable PWM
    udelay(10);

    // Start the DMA engine!
    dma_reg[DMA_CS] = BCM2708_DMA_RESET;  // Reset first
    udelay(10);
    dma_reg[DMA_CS] = BCM2708_DMA_INT | BCM2708_DMA_END;  // Clear flags
    dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);   // Point to first control block
    dma_reg[DMA_DEBUG] = 7;  // Clear debug errors
    dma_reg[DMA_CS] = 0x10880001;  // GO! Start DMA transfer

    // Track which control block DMA is currently processing
    size_t last_cb = (size_t)ctl->cb;

    // Buffers for audio data
    float data[DATA_SIZE];
    int data_len = 0;
    int data_index = 0;

    // Initialize audio processing
    if(fm_mpx_open(audio_file, DATA_SIZE, loop_audio) < 0) return 1;

    // Initialize RDS (Radio Data System) - sends station info
    set_rds_pi(pi);  // Program Identifier
    set_rds_rt(rt);  // Radio Text

    
    set_rds_ps(ps);
    printf("PI: %04X, PS: \"%s\".\n", pi, ps);
    
    printf("RT: \"%s\"\n", rt);

    printf("Starting to transmit on %3.1f MHz.\n", carrier_freq/1e6);

    // Main transmission loop - runs forever until terminated
    for (;;) {
        usleep(5000);  // Sleep 5ms

        // Calculate how many samples the DMA has consumed
        // We can safely write to those slots
        size_t cur_cb = mem_phys_to_virt(dma_reg[DMA_CONBLK_AD]);
        int last_sample = (last_cb - (size_t)mbox.virt_addr) / (sizeof(dma_cb_t) * 2);
        int this_sample = (cur_cb - (size_t)mbox.virt_addr) / (sizeof(dma_cb_t) * 2);
        int free_slots = this_sample - last_sample;

        // Handle wraparound in circular buffer
        if (free_slots < 0)
            free_slots += NUM_SAMPLES;

        // Fill available slots with new samples
        while (free_slots >= SUBSIZE) {
            // Get more audio data if buffer is empty
            if(data_len == 0) {
                if( fm_mpx_get_samples(data) < 0 ) {
                    terminate(0);
                }
                data_len = DATA_SIZE;
                data_index = 0;
            }

            // Get next audio sample and convert to frequency deviation
            float dval = data[data_index] * (DEVIATION / 10.);
            data_index++;
            data_len--;

            // Convert to integer frequency offset
            int intval = (int)((floor)(dval));

            // Write new frequency sample
            // This is: base_frequency + audio_deviation
            ctl->sample[last_sample++] = (0x5A << 24 | freq_ctl) + intval;
            
            // Wrap around at end of buffer
            if (last_sample == NUM_SAMPLES)
                last_sample = 0;

            free_slots -= SUBSIZE;
        }
        
        // Update our position in the circular buffer
        last_cb = (size_t)(mbox.virt_addr + last_sample * sizeof(dma_cb_t) * 2);
    }

    return 0;
}

// Program entry point
int main(int argc, char **argv) {
    // Required parameters (must be provided by user)
    char *audio_file = NULL;
    uint32_t carrier_freq = 0;
    
    // Optional parameters with defaults
    char *ps = "BWC";  // Default station name
    char *rt = "BWC: FM transmission from RaspberryPi";  // Default radio text
    uint16_t pi = 0x1234;  // Default PI code
    int loop_audio = 0;

    // Parse command line arguments
    for(int i=1; i<argc; i++) {
        char *arg = argv[i];
        char *param = NULL;

        // Get parameter for flags that need one
        if(arg[0] == '-' && i+1 < argc) param = argv[i+1];

        if((strcmp("-wav", arg)==0 || strcmp("-audio", arg)==0) && param != NULL) {
            i++;
            audio_file = param;
        } else if(strcmp("-freq", arg)==0 && param != NULL) {
            i++;
            carrier_freq = 1e6 * atof(param);  // Convert MHz to Hz
            if(carrier_freq < 76e6 || carrier_freq > 108e6)
                fatal("Incorrect frequency specification. Must be in megahertz, of the form 107.9, between 76 and 108.\n");
        } else if(strcmp("-pi", arg)==0 && param != NULL) {
            i++;
            pi = (uint16_t) strtol(param, NULL, 16);  // Parse as hex
        } else if(strcmp("-ps", arg)==0 && param != NULL) {
            i++;
            ps = param;  // Program Service name (8 chars)
        } else if(strcmp("-rt", arg)==0 && param != NULL) {
            i++;
            rt = param;  // Radio Text (64 chars)
        }
        else if(strcmp("-loop", arg)==0) {
            loop_audio = 1;  // Enable looping
        } else {
            fatal("Unrecognised argument: %s.\n"
            "Syntax: pi_fm_rds -freq <freq> -audio <file> [-pi <code>] [-ps <text>] [-rt <text>] [-loop]\n", arg);
        }
    }
    
    // Validate required parameters only
    if(carrier_freq == 0) {
        fatal("Error: Frequency is required. Use -freq <MHz> (e.g., -freq 107.9)\n");
    }
        if(audio_file == NULL) {
        fatal("Error: Audio file is required. Use -audio <filename>\n");
    }
    
    // Show what we're using
    printf("Configuration:\n");
    printf("  Frequency: %.1f MHz\n", carrier_freq/1e6);
    printf("  Audio file: %s\n", audio_file);
    printf("  PI code: %04X\n", pi);
    printf("  PS (station name): %s\n", ps);
    printf("  RT (radio text): %s\n", rt);
    printf("\n");

    char* locale = setlocale(LC_ALL, "");
    printf("Locale set to %s.\n", locale);

    // Start transmitting!
    int errcode = tx(carrier_freq, audio_file, pi, ps, rt, 0, loop_audio);

    terminate(errcode);
}