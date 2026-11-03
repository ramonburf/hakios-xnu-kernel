/*
 * The Lumia 950's battery and charger (talkman, MSM8992): the PMI8994's charger and fuel gauge, read over the SPMI
 * arbiter and handed to the battery service Darwin reads (hakios_battery_report: drivers/power/hakios.battery.c and
 * .cpp, the IOPMPowerSource powerd matches for the status bar and Settings > Battery). Only with boot-arg pmi8994=1:
 * every address below belongs to this phone.
 *
 * Read only but for one bit: charging commanded on (charging_on). Otherwise the charging itself needs nothing from the kernel: the PMI8994 charges with the setup the firmware gave
 * it (the 950 XL port saw a 1500 mA input limit, fast charge, then taper to full). What is read (PMI8994 is SPMI
 * slave 2; the cityman kernel's qpnp-smbcharger.c and qpnp-fg.c, as the 950 XL port's NativeBattery reads them):
 *   FG_SOC 0x4009            the fuel gauge's monotonic state of charge, 0..255
 *   USB_CHGPTH RT_STS 0x1310 bit 2: something on the USB port gives power
 *   CHGR_STS 0x100e          bits 2:1: 0 not charging, 1 pre-charge, 2 fast charge, 3 taper
 *   USB_CHGPTH IL_CFG 0x13f2 the input current limit, for the log
 * The arbiter (0xfc4c0000) is read through one command channel at 0xf800, the 950 XL port's; the kernel's other SPMI
 * users take hakios_spmi_lock (drivers/core/hakios.platform.c) for the same arbiter. Polled every 20 s.
 * The battery: BV-T5E, 3000 mAh.
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

#define SPMI_PHYS               0xfc4c0000UL
#define SPMI_SIZE               0x10000UL
#define SPMI_CMD                0xf800
#define SPMI_CONFIG             0xf804
#define SPMI_STATUS             0xf808
#define SPMI_WDATA0             0xf810
#define SPMI_RDATA0             0xf818

#define PMI                     2
#define FG_SOC                  0x4009
#define USB_RT_STS              0x1310
#define CHGR_STS                0x100e
#define USB_IL_CFG              0x13f2
/* for the charger's own state, logged when it changes (qpnp-smbcharger.c: CHGR 0x1000, BAT_IF 0x1200, USB_CHGPTH 0x1300) */
#define CHGR_RT_STS             0x1010
#define CHGR_CFG2               0x10fc          /* bit 7 CHG_EN_SRC (command register), bit 6 CHG_EN_POLARITY */
#define BAT_IF_RT_STS           0x1210
#define BAT_IF_CMD_CHG          0x1242          /* bit 1 EN_BAT_CHG (its sense set by CHGR_CFG2), bit 0 OTG */
#define USB_CMD_IL              0x1340          /* bit 0 USBIN_MODE_HC, bit 1 USB 9/1.5, bit 2 USBIN_SUSPEND */
#define USB_ICL_STS1            0x1307

#define DESIGN_UAH              3000000

static vm_offset_t spmi;

static const uint32_t usb_limit_ma[32] = { 300, 400, 450, 475, 500, 550, 600, 650, 700, 900, 950, 1000, 1100, 1200, 1400,
	                                   1450, 1500, 1600, 1800, 1850, 1880, 1910, 1930, 1950, 1970, 2000, 2050, 2100, 2300,
	                                   2400, 2500, 3000 };

static inline uint32_t
rd(uint32_t offset)
{
	return *(volatile uint32_t *)(spmi + offset);
}

static inline void
wr(uint32_t offset, uint32_t value)
{
	*(volatile uint32_t *)(spmi + offset) = value;
	__asm__ volatile ("dsb sy" ::: "memory");
}

/* one byte of a peripheral register: the command (read, slave, peripheral, offset), then status 1 = done */
static bool
spmi_read(uint8_t slave, uint16_t address, uint8_t *value)
{
	bool ok = false;

	hakios_spmi_lock();
	if (rd(SPMI_CONFIG) == 0) {             /* a channel somebody else configured is not ours */
		wr(SPMI_CMD, (1u << 27) | (uint32_t)slave << 20 | (uint32_t)(address >> 8) << 12 | (uint32_t)(address & 0xff) << 4);
		for (int i = 0; i < 1000; i++) {
			uint32_t status = rd(SPMI_STATUS);

			if (status != 0) {
				if (status == 1) {
					*value = (uint8_t)rd(SPMI_RDATA0);
					ok = true;
				}
				break;
			}
			delay_for_interval(1, NSEC_PER_USEC);
		}
	}
	hakios_spmi_unlock();
	return ok;
}

/* one byte into a peripheral register: the byte in WDATA0, then the command (opcode 0, extended write, one byte) */
static bool
spmi_write(uint8_t slave, uint16_t address, uint8_t value)
{
	bool ok = false;

	hakios_spmi_lock();
	if (rd(SPMI_CONFIG) == 0) {
		wr(SPMI_WDATA0, value);
		wr(SPMI_CMD, (uint32_t)slave << 20 | (uint32_t)(address >> 8) << 12 | (uint32_t)(address & 0xff) << 4);
		for (int i = 0; i < 1000; i++) {
			uint32_t status = rd(SPMI_STATUS);

			if (status != 0) {
				ok = status == 1;
				break;
			}
			delay_for_interval(1, NSEC_PER_USEC);
		}
	}
	hakios_spmi_unlock();
	return ok;
}

/*
 * Charging on (boot-arg pmi8994_charge=0 leaves it as the firmware did). Boot 31: USB power, no battery fault, input
 * not suspended, and yet not charging -- CHGR_CFG2 0x0b (CHG_EN_SRC 0: the command register decides; CHG_EN_POLARITY
 * 0: active high) with BAT_IF CMD_CHG 0, i.e. charging commanded off. EN_BAT_CHG (bit 1) set is what
 * smbchg_charging_en() of qpnp-smbcharger.c writes for that polarity; CMD_CHG is not a secured register.
 */
static void
charging_on(void)
{
	uint8_t cfg2 = 0, cmd = 0, after = 0;
	uint32_t enabled = 1;

	PE_parse_boot_argn("pmi8994_charge", &enabled, sizeof(enabled));
	if (!enabled || !spmi_read(PMI, CHGR_CFG2, &cfg2) || !spmi_read(PMI, BAT_IF_CMD_CHG, &cmd)) {
		return;
	}
	if (cfg2 & (1u << 7)) {
		printf("pmi8994: charging is enabled by a pin here (CHGR_CFG2 %#x): left as it is\n", cfg2);
		return;
	}
	if ((cfg2 & (1u << 6)) ? !(cmd & 2) : (cmd & 2)) {
		return;         /* commanded on already */
	}
	spmi_write(PMI, BAT_IF_CMD_CHG, (cfg2 & (1u << 6)) ? (uint8_t)(cmd & ~2u) : (uint8_t)(cmd | 2u));
	spmi_read(PMI, BAT_IF_CMD_CHG, &after);
	printf("pmi8994: charging commanded on: CMD_CHG %#x -> %#x (CHGR_CFG2 %#x)\n", cmd, after, cfg2);
}

static void
pmi8994_thread(__unused void *parameter, __unused wait_result_t result)
{
	struct hakios_battery_state last = { 0 };
	unsigned failures = 0;

	spmi = ml_io_map(SPMI_PHYS, SPMI_SIZE);
	charging_on();
	for (;;) {
		uint8_t soc = 0, rt = 0, chgr = 0, il = 0;
		struct hakios_battery_state b = { .size = sizeof(b) };

		if (!spmi_read(PMI, FG_SOC, &soc) || !spmi_read(PMI, USB_RT_STS, &rt) || !spmi_read(PMI, CHGR_STS, &chgr) ||
		    !spmi_read(PMI, USB_IL_CFG, &il)) {
			if (failures++ < 10) {
				printf("pmi8994: SPMI read failed (arbiter channel config %#x)\n", rd(SPMI_CONFIG));
			}
		} else {
			uint32_t state = chgr >> 1 & 3;

			b.percent_hundredths = (int32_t)((soc * 10000u + 127) / 255);
			b.present = 1;
			b.external = (rt & 4) != 0;
			b.status = b.external && state != 0 ? 1 : b.external && b.percent_hundredths >= 9900 ? 4 : b.external ? 3 : 2;
			b.temperature_hundredths = -27300;      /* not read yet: "not known" to the thermal and flashlight guards */
			b.design_uah = b.full_uah = DESIGN_UAH;
			b.charge_uah = (int32_t)((int64_t)DESIGN_UAH * b.percent_hundredths / 10000);
			b.usb_type = b.external ? 2 : 0;
			b.usb_current_ua = b.external ? (int32_t)usb_limit_ma[il & 31] * 1000 : 0;
			if (b.percent_hundredths / 100 != last.percent_hundredths / 100 || b.external != last.external ||
			    b.status != last.status || last.size == 0) {
				uint8_t chg_rt = 0, cfg2 = 0, bat_rt = 0, cmd_chg = 0, cmd_il = 0, icl = 0;

				spmi_read(PMI, CHGR_RT_STS, &chg_rt);
				spmi_read(PMI, CHGR_CFG2, &cfg2);
				spmi_read(PMI, BAT_IF_RT_STS, &bat_rt);
				spmi_read(PMI, BAT_IF_CMD_CHG, &cmd_chg);
				spmi_read(PMI, USB_CMD_IL, &cmd_il);
				spmi_read(PMI, USB_ICL_STS1, &icl);
				printf("pmi8994: charger: CHGR_STS %#x RT_STS %#x CFG2 %#x; BAT_IF RT_STS %#x CMD_CHG %#x; USB RT_STS %#x "
				    "CMD_IL %#x ICL_STS1 %#x IL_CFG %#x\n", chgr, chg_rt, cfg2, bat_rt, cmd_chg, rt, cmd_il, icl, il);
				printf("pmi8994: %d%%, %s, %s (charger state %u, input limit %u mA)\n", b.percent_hundredths / 100,
				    b.external ? "power on the USB port" : "on battery",
				    b.status == 1 ? "charging" : b.status == 4 ? "full" : b.status == 3 ? "not charging" : "discharging",
				    state, usb_limit_ma[il & 31]);
			}
			hakios_battery_report(&b);
			last = b;
		}
		delay_for_interval(20, NSEC_PER_SEC);
	}
}

void
hakios_pmi8994_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("pmi8994", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	if (kernel_thread_start(pmi8994_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
