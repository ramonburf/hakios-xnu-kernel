/*
 * The Lumia 950's GPU (talkman, MSM8992): an Adreno 418, the a4xx generation. Only with boot-arg gpu4=1: every
 * address below belongs to this phone. Bring-up, step by step; this is step 1's kernel half.
 *
 * The loader powers the GPU before the kernel starts (xnuloader, gpu_probe=2): the graphics clock from RPM over
 * SMD (clk2 #1, 300 MHz), the CX and GX power domains, the AHB, timer and core clocks. What the firmware maps does
 * not include the GPU's registers, so the loader cannot read them; here they are mapped and read, once the GX
 * domain says it is on (an access to a collapsed domain can hang the bus):
 *   RBBM_HW_VERSION, RBBM_HW_CONFIGURATION, RBBM_STATUS, CP_ME_CNTL, RBBM_INT_0_STATUS
 * (Boot 24: RBBM_HW_VERSION 0x51, HW_CONFIGURATION 0x229249a, RBBM_STATUS 1, CP_ME_CNTL 0xd0000000 -- the CP halted.)
 * Register indices: Mesa's freedreno a4xx.xml (dword indices; byte offset = index * 4). Addresses: Qualcomm's
 * msm8992-gpu.dtsi and msm-iommu-v1.dtsi (msm-3.10), and the 950 XL port's handset tests (UniXNU).
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

#define printf hakios_say      /* (into the loader's log too: the 950 boots without -v) */

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

#define MMSS_PHYS               0xfd8c0000UL
#define MMSS_SIZE               0x10000UL
#define GDSCR_GX                0x4024
#define GDSCR_CX                0x4034
#define GDSC_PWR_ON             (1u << 31)
#define CBCR_GPU_CORE           0x4028

#define GPU_PHYS                0xfdb00000UL
#define GPU_SIZE                0x10000UL
#define REG(index)              ((index) << 2)
#define RBBM_HW_VERSION         REG(0x0000)
#define RBBM_HW_CONFIGURATION   REG(0x0002)
#define RBBM_INT_0_STATUS       REG(0x017d)
#define RBBM_STATUS             REG(0x0191)
#define CP_ME_CNTL              REG(0x022d)


static vm_offset_t mmss, gpu;

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static char said[2048];
static size_t said_length;

/* said now, and kept: the kernel's log of a run keeps only its end, so it is said again later */
static void
say(const char *line)
{
	size_t n = strlen(line);

	printf("%s", line);
	if (said_length + n < sizeof(said)) {
		memcpy(said + said_length, line, n);
		said_length += n;
		said[said_length] = 0;
	}
}

static void
probe(void)
{
	char line[256];
	uint32_t gx, cx;

	mmss = ml_io_map(MMSS_PHYS, MMSS_SIZE);
	if (mmss == 0) {
		say("adreno4: the clock controller did not map\n");
		return;
	}
	gx = rd(mmss + GDSCR_GX);
	cx = rd(mmss + GDSCR_CX);
	snprintf(line, sizeof(line), "adreno4: GDSCR gx %#x cx %#x, core clock %#x\n", gx, cx, rd(mmss + CBCR_GPU_CORE));
	say(line);
	if (!(gx & GDSC_PWR_ON) || !(cx & GDSC_PWR_ON)) {
		say("adreno4: the GPU is not powered (the loader's gpu_probe=2 powers it): not touched\n");
		return;
	}
	gpu = ml_io_map(GPU_PHYS, GPU_SIZE);
	if (gpu == 0) {
		say("adreno4: the GPU's registers did not map\n");
		return;
	}
	snprintf(line, sizeof(line), "adreno4: RBBM_HW_VERSION %#x, HW_CONFIGURATION %#x, RBBM_STATUS %#x, CP_ME_CNTL %#x, "
	    "INT_0_STATUS %#x\n", rd(gpu + RBBM_HW_VERSION), rd(gpu + RBBM_HW_CONFIGURATION), rd(gpu + RBBM_STATUS),
	    rd(gpu + CP_ME_CNTL), rd(gpu + RBBM_INT_0_STATUS));
	say(line);
	/*
	 * Not the SMMU's context banks: reading SCTLR of bank 0 (0xfdb18000) took a kernel data abort on boot 24, as the
	 * global registers did for the 950 XL port -- the whole of the GPU's SMMU answers only the secure world (or wants a
	 * clock nobody has turned on). Its configuration goes through the secure monitor, as Qualcomm's kgsl does it.
	 */
}

static void
adreno4_thread(__unused void *parameter, __unused wait_result_t result)
{
	probe();
	for (int i = 0; i < 6; i++) {
		delay_for_interval(60, NSEC_PER_SEC);
		printf("adreno4: (again) %s", said);
	}
	thread_terminate(current_thread());
}

void
hakios_adreno4_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("gpu4", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	if (kernel_thread_start(adreno4_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
