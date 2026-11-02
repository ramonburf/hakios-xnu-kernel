/*
 * The Lumia 950's Wi-Fi (talkman, MSM8992): a Qualcomm QCA6174 behind PCIe root complex 0. Only with boot-arg
 * wifi950=1: every address below belongs to this phone. Bring-up, step by step; this is step 2.
 *
 * Step 1 (the loader's wifi_probe=1, boot 26) found what the firmware left: its PCIExpressConfiguratorDxe powered the
 * chip (GPIO 113 WLAN_EN, 35 PERST# high), trained the link (ELBI SYS_STTS 0x11401: up; Gen1 x1), and points iATU
 * region 1 at the endpoint's configuration space (0xff100000, bus 1). The endpoint answers 168c:003e, class 0x0280,
 * with a 64-bit memory BAR0 of 2 MB that nobody has given an address.
 *
 * Here: iATU region 2 for memory, CPU 0xff300000..0xff4fffff to the same PCI addresses (the "bars" window of the
 * device tree's qcom,pcie@fc520000, 0xff300000 + 0xd00000); BAR0 there; memory and bus mastering on; the chip woken
 * (PCIE_SOC_WAKE) and its SoC chip ID read through BAR0 (ath10k's hw.c, QCA6174: RTC_SOC_BASE 0x800, SOC_CHIP_ID
 * +0xf0; PCIE_LOCAL_BASE 0x80000, SOC_WAKE +0x4, RTC_STATE 0x...). Nothing of the radio is started.
 *
 * DesignWare iATU (the dm_core window, 0xff000000): viewport 0x900, then for the region it selects: CTRL1 0x904
 * (type: 0 memory, 4 config type 0, 5 config type 1), CTRL2 0x908 (bit 31 enable), LOWER_BASE 0x90c, UPPER_BASE
 * 0x910, LIMIT 0x914, LOWER_TARGET 0x918, UPPER_TARGET 0x91c -- the firmware's own ConfigSpace() programs it so.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

#define PCIE_DBI_PHYS           0xff000000UL
#define PCIE_DBI_SIZE           0x1000UL
#define PCIE_CONF_PHYS          0xff100000UL
#define PCIE_CONF_SIZE          0x1000UL
#define PCIE_BAR_PHYS           0xff300000UL    /* where BAR0 goes, CPU and PCI address alike */
#define PCIE_BAR_SIZE           0x200000UL      /* 2 MB, what BAR0 asks for */

#define ELBI_SYS_STTS           0xf28           /* in dm_core: bit 10, the link is up */
#define IATU_VIEWPORT           0x900
#define IATU_CTRL1              0x904
#define IATU_CTRL2              0x908
#define IATU_LOWER_BASE         0x90c
#define IATU_UPPER_BASE         0x910
#define IATU_LIMIT              0x914
#define IATU_LOWER_TARGET       0x918
#define IATU_UPPER_TARGET       0x91c
#define IATU_ENABLE             (1u << 31)

#define PCI_ID                  0x00
#define PCI_COMMAND             0x04
#define PCI_COMMAND_MEMORY      (1u << 1)
#define PCI_COMMAND_MASTER      (1u << 2)
#define PCI_CLASS               0x08
#define PCI_BAR0                0x10
#define PCI_BAR1                0x14

/* QCA6174 (ath10k hw.c, qca6174_regs; pci.h) */
#define PCIE_LOCAL_BASE         0x80000
#define SOC_WAKE                (PCIE_LOCAL_BASE + 0x0004)
#define RTC_STATE               (PCIE_LOCAL_BASE + 0x0000)
#define RTC_SOC_BASE            0x00800
#define SOC_CHIP_ID             (RTC_SOC_BASE + 0x00f0)
#define SOC_CORE_BASE           0x3a000
#define FW_INDICATOR            (SOC_CORE_BASE + 0x9030)

static vm_offset_t dbi, conf, bar;

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static char said[2048];
static size_t said_length;

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
	uint32_t id, link;

	dbi = ml_io_map(PCIE_DBI_PHYS, PCIE_DBI_SIZE);
	conf = ml_io_map(PCIE_CONF_PHYS, PCIE_CONF_SIZE);
	if (dbi == 0 || conf == 0) {
		say("qca6174: the PCIe controller did not map\n");
		return;
	}
	link = rd(dbi + ELBI_SYS_STTS);
	if (!(link & 0x400)) {
		snprintf(line, sizeof(line), "qca6174: the PCIe link is down (ELBI SYS_STTS %#x): not touched\n", link);
		say(line);
		return;
	}
	id = rd(conf + PCI_ID);
	snprintf(line, sizeof(line), "qca6174: endpoint id %#x class %#x command %#x BAR0 %#x BAR1 %#x\n", id,
	    rd(conf + PCI_CLASS), rd(conf + PCI_COMMAND), rd(conf + PCI_BAR0), rd(conf + PCI_BAR1));
	say(line);
	if (id != 0x003e168c) {
		say("qca6174: that is not a QCA6174: left alone\n");
		return;
	}

	/* iATU region 2: memory, 0xff300000 + 2 MB to the same PCI addresses */
	wr(dbi + IATU_VIEWPORT, 2);
	wr(dbi + IATU_CTRL1, 0);
	wr(dbi + IATU_LOWER_BASE, (uint32_t)PCIE_BAR_PHYS);
	wr(dbi + IATU_UPPER_BASE, 0);
	wr(dbi + IATU_LIMIT, (uint32_t)(PCIE_BAR_PHYS + PCIE_BAR_SIZE - 1));
	wr(dbi + IATU_LOWER_TARGET, (uint32_t)PCIE_BAR_PHYS);
	wr(dbi + IATU_UPPER_TARGET, 0);
	wr(dbi + IATU_CTRL2, IATU_ENABLE);
	snprintf(line, sizeof(line), "qca6174: iATU region 2: ctrl2 %#x base %#x limit %#x target %#x\n",
	    rd(dbi + IATU_CTRL2), rd(dbi + IATU_LOWER_BASE), rd(dbi + IATU_LIMIT), rd(dbi + IATU_LOWER_TARGET));
	say(line);
	/* (the configuration accesses go through region 1 again: the viewport only selects what CTRL* show) */
	wr(dbi + IATU_VIEWPORT, 1);

	wr(conf + PCI_BAR0, (uint32_t)PCIE_BAR_PHYS);
	wr(conf + PCI_BAR1, 0);
	wr(conf + PCI_COMMAND, rd(conf + PCI_COMMAND) | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
	snprintf(line, sizeof(line), "qca6174: BAR0 %#x BAR1 %#x command %#x\n", rd(conf + PCI_BAR0), rd(conf + PCI_BAR1),
	    rd(conf + PCI_COMMAND));
	say(line);

	bar = ml_io_map(PCIE_BAR_PHYS, PCIE_BAR_SIZE);
	if (bar == 0) {
		say("qca6174: BAR0 did not map\n");
		return;
	}
	snprintf(line, sizeof(line), "qca6174: before waking: RTC_STATE %#x SOC_CHIP_ID %#x\n", rd(bar + RTC_STATE),
	    rd(bar + SOC_CHIP_ID));
	say(line);
	wr(bar + SOC_WAKE, 1);
	delay_for_interval(20, NSEC_PER_MSEC);
	snprintf(line, sizeof(line), "qca6174: awake: RTC_STATE %#x SOC_CHIP_ID %#x FW_INDICATOR %#x\n", rd(bar + RTC_STATE),
	    rd(bar + SOC_CHIP_ID), rd(bar + FW_INDICATOR));
	say(line);
	wr(bar + SOC_WAKE, 0);
}

static void
qca6174_thread(__unused void *parameter, __unused wait_result_t result)
{
	probe();
	for (int i = 0; i < 6; i++) {
		delay_for_interval(60, NSEC_PER_SEC);
		printf("qca6174: (again) %s", said);
	}
	thread_terminate(current_thread());
}

void
hakios_qca6174_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("wifi950", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	if (kernel_thread_start(qca6174_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
