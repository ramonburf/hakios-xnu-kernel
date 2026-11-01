/*
 * The touch screen of the Microsoft Lumia 950 (talkman, MSM8992): a Synaptics RMI4 controller at I2C
 * address 0x4b on BLSP1 QUP1 (0xf9923000), as a source of touches for /dev/hakios-touch
 * (sys/hakios.input.h). Only with boot-arg touchscreen=1: every address below belongs to this phone.
 *
 * Wiring (the phone's TWRP device tree, synaptics,rmi4 under i2c@f9923000; the cityman kernel's
 * arch/arm64/boot/dts/lumia/common/touch.dtsi is the same): SDA/SCL on TLMM GPIO 2/3 (function 3),
 * reset on GPIO 38 (low = in reset), attention on GPIO 77 (low = the controller has news). Its
 * supplies, PM8994 L22 (3.0 V) and LVS1 (1.8 V I2C pull-up), are RPM-managed and the Windows Phone
 * UEFI that boots this kernel leaves them on, so this driver writes no supply and only does the
 * least it takes to talk to the bus:
 *   clocks   the BLSP1 AHB and QUP1 I2C clocks, turned on only if they are off (branch vote, XO root)
 *   pins     GPIO 2/3 to the QUP if they are not, and the controller out of reset if GPIO 38 holds it
 *   I2C      the QUP in FIFO mode, polled, tags v2: 100 kHz to find the controller, then 400 kHz
 *   chip     RMI4: the page description table, F01 (device control, interrupt status) and F12 (2-D
 *            sensor: its finger objects, 8 bytes each)
 * No interrupt is used: a thread looks at the attention line every 12 ms (boot-arg touch_poll_us=N)
 * and reads F01's interrupt status, then F12's objects, when it is low.
 *
 * The UEFI keeps running on core 0 (it serves the kernel's storage), so a hang of the bus here would
 * take the whole phone with it: before the first write to the clock controller and before the first
 * access to the QUP the driver waits until the loader has saved the kernel's log (every 10 s), so
 * that \EFI\hakios\kernel.log says how far it got (boot-arg touch_checkpoint_ms=N, 0 = no wait).
 *
 * From UniXNU's NativeTouch for the Lumia 950 XL (the same SoC family and the same controller wiring);
 * register layouts: the cityman kernel's drivers/i2c/busses/i2c-msm-v2.[ch], clock-gcc-8994.c,
 * pinctrl-msm8994, and drivers/input/touchscreen/synaptics_i2c_rmi4.c (synaptics_rmi4_f12_init).
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

static vm_offset_t gcc, tlmm, qup;

/*
 * Everything the driver says is also kept, and said again once a minute for its first ten minutes:
 * the probe runs in the first half minute of a boot, and on the 950 the kernel's log buffer has been
 * overwritten by then (boot 14: nothing before 102 s was left).
 */
static char history[6144];
static size_t history_length;

static void
repeat_history(void)
{
	const char *line = history;

	printf("synaptics: --- what it said so far, again ---\n");
	while (line < history + history_length) {
		int length = 0;

		while (line + length < history + history_length && line[length] != '\n') {
			length++;
		}
		printf("%.*s (again)\n", length, line);
		line += length + 1;
	}
}

static int
say(const char *format, ...)
{
	char line[400];
	va_list list;
	int length;

	va_start(list, format);
	length = vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	printf("%s", line);
	if (length > 0) {
		size_t n = strnlen(line, sizeof(line));

		if (history_length + n < sizeof(history)) {
			memcpy(history + history_length, line, n);
			history_length += n;
		}
	}
	return length;
}
#define printf say

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

static void
sleep_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

/* Wait for a register to read `want` under `mask`, looking every 10 us; false after `tries` looks. */
static bool
wait_reg(vm_offset_t address, uint32_t mask, uint32_t want, uint32_t tries)
{
	while (tries--) {
		if ((rd(address) & mask) == want) {
			return true;
		}
		sleep_us(10);
	}
	return false;
}

static uint32_t checkpoint_ms = 11000;

/* Say what comes next, and give the loader time to save the log before it is tried. */
static void
checkpoint(const char *what)
{
	printf("synaptics: %s\n", what);
	if (checkpoint_ms) {
		sleep_ms(checkpoint_ms);
	}
}

/* ---- clocks (GCC, clock-gcc-8994.c) ---------------------------------------------------------------- */

#define GCC_PHYS                0xfc400000UL
#define GCC_SIZE                0x2000UL
#define GCC_BRANCH_VOTE         (gcc + 0x1484)
#define VOTE_BLSP1_AHB          (1u << 17)
#define GCC_BLSP1_AHB_CBCR      (gcc + 0x05c4)
#define GCC_QUP1_I2C_CBCR       (gcc + 0x0648)
#define GCC_QUP1_I2C_CMD_RCGR   (gcc + 0x0660)
#define GCC_QUP1_I2C_CFG_RCGR   (gcc + 0x0664)  /* [4:0] 2*divider-1, [10:8] source (0 = XO) */
#define CBCR_OFF                (1u << 31)
#define CBCR_ENABLE             (1u << 0)
#define RCG_ROOT_OFF            (1u << 31)
#define RCG_UPDATE              (1u << 0)

static void
show_clocks(const char *when)
{
	printf("synaptics: clocks %s: vote %#x (blsp1 ahb %s), blsp1 ahb cbcr %#x, qup1 i2c cbcr %#x, rcgr %#x/%#x\n", when,
	    rd(GCC_BRANCH_VOTE), (rd(GCC_BRANCH_VOTE) & VOTE_BLSP1_AHB) ? "voted" : "not voted", rd(GCC_BLSP1_AHB_CBCR),
	    rd(GCC_QUP1_I2C_CBCR), rd(GCC_QUP1_I2C_CMD_RCGR), rd(GCC_QUP1_I2C_CFG_RCGR));
}

static bool
clocks_needed(void)
{
	return (rd(GCC_BLSP1_AHB_CBCR) & CBCR_OFF) || (rd(GCC_QUP1_I2C_CBCR) & CBCR_OFF) ||
	       (rd(GCC_QUP1_I2C_CMD_RCGR) & RCG_ROOT_OFF);
}

static bool
clocks_on(void)
{
	if (rd(GCC_BLSP1_AHB_CBCR) & CBCR_OFF) {
		wr(GCC_BRANCH_VOTE, rd(GCC_BRANCH_VOTE) | VOTE_BLSP1_AHB);
		if (!wait_reg(GCC_BLSP1_AHB_CBCR, CBCR_OFF, 0, 10000)) {
			printf("synaptics: the blsp1 ahb clock stays off after the vote (%#x)\n", rd(GCC_BLSP1_AHB_CBCR));
			return false;
		}
	}
	if ((rd(GCC_QUP1_I2C_CMD_RCGR) & RCG_ROOT_OFF) || (rd(GCC_QUP1_I2C_CFG_RCGR) & 0x71f) != 0x001) {
		/* the crystal, undivided: the I2C dividers below are worked out for 19.2 MHz */
		wr(GCC_QUP1_I2C_CFG_RCGR, (rd(GCC_QUP1_I2C_CFG_RCGR) & ~0x71fu) | 0x001);
		wr(GCC_QUP1_I2C_CMD_RCGR, rd(GCC_QUP1_I2C_CMD_RCGR) | RCG_UPDATE);
		if (!wait_reg(GCC_QUP1_I2C_CMD_RCGR, RCG_UPDATE, 0, 10000)) {
			printf("synaptics: the qup1 i2c root does not take the XO configuration (%#x)\n", rd(GCC_QUP1_I2C_CMD_RCGR));
		}
	}
	if (rd(GCC_QUP1_I2C_CBCR) & CBCR_OFF) {
		wr(GCC_QUP1_I2C_CBCR, rd(GCC_QUP1_I2C_CBCR) | CBCR_ENABLE);
		if (!wait_reg(GCC_QUP1_I2C_CBCR, CBCR_OFF, 0, 10000)) {
			printf("synaptics: the qup1 i2c clock stays off (%#x)\n", rd(GCC_QUP1_I2C_CBCR));
			return false;
		}
	}
	return true;
}

/* ---- pins (TLMM, msm-tlmm-8974 layout) ------------------------------------------------------------- */

#define TLMM_PHYS               0xfd510000UL
#define TLMM_SIZE               0x4000UL
#define GPIO_CFG(n)             (tlmm + 0x1000UL + 0x10UL * (n))  /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(n)              (tlmm + 0x1004UL + 0x10UL * (n))  /* bit 0 in, bit 1 out */

#define PIN_SDA                 2
#define PIN_SCL                 3
#define PIN_RESET               38      /* low = in reset */
#define PIN_ATTN                77      /* low = the controller has news */
#define PIN_FUNCTION_QUP        3       /* "blsp_i2c1" */

static void
show_pin(const char *name, unsigned pin)
{
	uint32_t cfg = rd(GPIO_CFG(pin)), io = rd(GPIO_IO(pin));

	printf("synaptics: gpio %u (%s): cfg %#x function %u pull %u drive %u mA %s, in %u out %u\n", pin, name, cfg,
	    cfg >> 2 & 15, cfg & 3, (cfg >> 6 & 7) * 2 + 2, (cfg & (1u << 9)) ? "output" : "input", io & 1, io >> 1 & 1);
}

static void
show_pins(void)
{
	show_pin("sda", PIN_SDA);
	show_pin("scl", PIN_SCL);
	show_pin("reset", PIN_RESET);
	show_pin("attention", PIN_ATTN);
}

static void
pins_for_bus(void)
{
	const unsigned bus[] = { PIN_SDA, PIN_SCL };

	for (unsigned i = 0; i < 2; i++) {
		uint32_t cfg = rd(GPIO_CFG(bus[i]));

		if ((cfg >> 2 & 15) != PIN_FUNCTION_QUP) {
			wr(GPIO_CFG(bus[i]), (cfg & ~0x3ffu) | PIN_FUNCTION_QUP << 2);
		}
	}
	/* an output driven low holds the controller in reset: release it, with the Linux driver's power-on delay (66 ms) */
	if ((rd(GPIO_CFG(PIN_RESET)) & (1u << 9)) && !(rd(GPIO_IO(PIN_RESET)) >> 1 & 1)) {
		wr(GPIO_IO(PIN_RESET), 1u << 1);
		printf("synaptics: released the controller's reset (gpio %u was driven low)\n", PIN_RESET);
		sleep_ms(100);
	}
}

static bool
attention(void)
{
	return !(rd(GPIO_IO(PIN_ATTN)) & 1);
}

/* ---- I2C through the QUP (i2c-msm-v2.[ch]), FIFO mode, polled ------------------------------------- */

#define QUP_PHYS                0xf9923000UL
#define QUP_SIZE                0x1000UL
#define QUP_CONFIG              (qup + 0x000)
#define QUP_STATE               (qup + 0x004)
#define QUP_IO_MODES            (qup + 0x008)
#define QUP_SW_RESET            (qup + 0x00c)
#define QUP_OPERATIONAL         (qup + 0x018)
#define QUP_ERROR_FLAGS         (qup + 0x01c)
#define QUP_ERROR_FLAGS_EN      (qup + 0x020)
#define QUP_OPERATIONAL_MASK    (qup + 0x028)
#define QUP_HW_VERSION          (qup + 0x030)
#define QUP_MX_OUTPUT_COUNT     (qup + 0x100)
#define QUP_OUT_FIFO            (qup + 0x110)
#define QUP_MX_WRITE_COUNT      (qup + 0x150)
#define QUP_MX_INPUT_COUNT      (qup + 0x200)
#define QUP_MX_READ_COUNT       (qup + 0x208)
#define QUP_IN_FIFO             (qup + 0x218)
#define QUP_I2C_MASTER_CLK_CTL  (qup + 0x400)
#define QUP_I2C_STATUS          (qup + 0x404)
#define QUP_I2C_MASTER_CONFIG   (qup + 0x408)

#define STATE_RESET             0
#define STATE_RUN               1
#define STATE_PAUSE             3
#define STATE_MASK              3
#define STATE_VALID             (1u << 2)
#define STATE_MAST_GEN          (1u << 4)
#define CONFIG_I2C              (0x200u | 0x7)  /* mini-core I2C master, 8-bit words */
#define CONFIG_NO_OUTPUT        (1u << 6)
#define CONFIG_NO_INPUT         (1u << 7)
#define IO_MODE_BITS            ((3u << 10) | (3u << 12) | (1u << 16))
#define IO_PACK_UNPACK          ((1u << 14) | (1u << 15))
#define OP_MAX_OUTPUT_DONE      (1u << 10)
#define OP_MAX_INPUT_DONE       (1u << 11)
#define OP_SERVICE_FLAGS        ((1u << 8) | (1u << 9))
#define I2C_STATUS_ERRORS       0x380003cu
#define I2C_STATUS_NACK         (1u << 3)
#define I2C_STATUS_BUS_ACTIVE   (1u << 8)
#define CLK_100K                (62u << 16 | 124u)      /* high time and full divider of the 19.2 MHz core clock */
#define CLK_400K                (14u << 16 | 28u)
#define TAG_START               0x81
#define TAG_WRITE               0x82
#define TAG_WRITE_STOP          0x83
#define TAG_READ_STOP           0x87
#define MAX_READ                60      /* and two tag bytes: the 64-byte input FIFO */

#define TOUCH_ADDRESS           0x4b

static uint32_t bus_clock = CLK_100K;

static bool
qup_state(uint32_t state)
{
	if (!wait_reg(QUP_STATE, STATE_VALID, STATE_VALID, 20000)) {
		return false;
	}
	wr(QUP_STATE, state);
	return wait_reg(QUP_STATE, STATE_VALID | STATE_MASK, STATE_VALID | state, 20000);
}

static bool
qup_init(void)
{
	printf("synaptics: qup version %#x, state %#x, io modes %#x, i2c status %#x\n", rd(QUP_HW_VERSION), rd(QUP_STATE),
	    rd(QUP_IO_MODES), rd(QUP_I2C_STATUS));
	wr(QUP_SW_RESET, 1);
	if (!wait_reg(QUP_STATE, STATE_VALID | STATE_MASK, STATE_VALID | STATE_RESET, 20000) || !qup_state(STATE_RESET)) {
		printf("synaptics: the qup does not reach its reset state (%#x)\n", rd(QUP_STATE));
		return false;
	}
	wr(QUP_CONFIG, CONFIG_I2C);
	wr(QUP_ERROR_FLAGS_EN, 0x3c);
	wr(QUP_OPERATIONAL_MASK, OP_SERVICE_FLAGS);
	wr(QUP_I2C_MASTER_CONFIG, 1);   /* tags v2 */
	if (!(rd(QUP_STATE) & STATE_MAST_GEN)) {
		printf("synaptics: the qup reports no I2C master (state %#x)\n", rd(QUP_STATE));
		return false;
	}
	return true;
}

/* Put a transfer's bytes into the output FIFO, four to a word, and start it. */
static bool
qup_start(const uint8_t *tx, uint32_t length, uint32_t read_count, bool output_only)
{
	if (!qup_state(STATE_RESET)) {
		return false;
	}
	wr(QUP_MX_INPUT_COUNT, 0);
	wr(QUP_MX_OUTPUT_COUNT, 0);
	wr(QUP_MX_READ_COUNT, read_count);
	wr(QUP_MX_WRITE_COUNT, length);
	wr(QUP_CONFIG, (rd(QUP_CONFIG) & ~(CONFIG_NO_INPUT | CONFIG_NO_OUTPUT | 0xf1fu)) | CONFIG_I2C |
	    (output_only ? CONFIG_NO_INPUT : 0));
	wr(QUP_IO_MODES, (rd(QUP_IO_MODES) & ~(IO_MODE_BITS | IO_PACK_UNPACK)) | IO_PACK_UNPACK);
	wr(QUP_OPERATIONAL_MASK, 0);
	if (!qup_state(STATE_RUN)) {
		return false;
	}
	wr(QUP_I2C_MASTER_CLK_CTL, bus_clock);
	if (!qup_state(STATE_PAUSE)) {
		return false;
	}
	for (uint32_t i = 0; i < length; i += 4) {
		uint32_t word = 0;

		for (uint32_t b = 0; b < 4 && i + b < length; b++) {
			word |= (uint32_t)tx[i + b] << (8 * b);
		}
		wr(QUP_OUT_FIFO, word);
	}
	return qup_state(STATE_RUN);
}

/* Clear what the transfer left and go back to the reset state. */
static void
qup_finish(uint32_t status)
{
	uint32_t errors = rd(QUP_ERROR_FLAGS);

	wr(QUP_I2C_STATUS, status & I2C_STATUS_ERRORS);
	wr(QUP_ERROR_FLAGS, errors & 0x3c);
	wr(QUP_OPERATIONAL, rd(QUP_OPERATIONAL) & OP_SERVICE_FLAGS);
	wr(QUP_CONFIG, rd(QUP_CONFIG) & ~CONFIG_NO_INPUT);
	qup_state(STATE_RESET);
}

/* Write `reg`, repeated start, read `count` bytes. */
static bool
i2c_read(uint8_t reg, uint8_t *out, uint32_t count, uint32_t *status_out)
{
	const uint8_t tx[] = { TAG_START, TOUCH_ADDRESS << 1, TAG_WRITE, 1, reg,
		               TAG_START, TOUCH_ADDRESS << 1 | 1, TAG_READ_STOP, (uint8_t)count };
	uint32_t status = 0;
	bool done = false;

	*status_out = 0;
	if (count == 0 || count > MAX_READ || !qup_start(tx, sizeof(tx), count + 2, false)) {
		return false;
	}
	for (int i = 0; i < 5000 && !done; i++) {       /* 50 ms; 60 bytes at 100 kHz take ~7 ms */
		status = rd(QUP_I2C_STATUS);
		if (status & I2C_STATUS_ERRORS) {
			break;
		}
		done = rd(QUP_OPERATIONAL) & OP_MAX_INPUT_DONE;
		if (!done) {
			sleep_us(10);
		}
	}
	if (done) {
		uint8_t in[64];

		/* the input FIFO starts with the read's two tag bytes */
		for (uint32_t i = 0; i < count + 2; i += 4) {
			uint32_t word = rd(QUP_IN_FIFO);

			for (uint32_t b = 0; b < 4 && i + b < sizeof(in); b++) {
				in[i + b] = (uint8_t)(word >> (8 * b));
			}
		}
		memcpy(out, in + 2, count);
	}
	qup_finish(status);
	*status_out = status;
	return done && !(status & I2C_STATUS_ERRORS);
}

static bool
i2c_write(uint8_t reg, uint8_t value, uint32_t *status_out)
{
	const uint8_t tx[] = { TAG_START, TOUCH_ADDRESS << 1, TAG_WRITE_STOP, 2, reg, value };
	uint32_t status = 0;
	bool done = false;

	*status_out = 0;
	if (!qup_start(tx, sizeof(tx), 0, true)) {
		return false;
	}
	for (int i = 0; i < 5000 && !done; i++) {
		status = rd(QUP_I2C_STATUS);
		if (status & I2C_STATUS_ERRORS) {
			break;
		}
		/* MAX_OUTPUT_DONE can come before the bus is done (i2c-msm-v2.c): wait for it to go idle as well */
		done = (rd(QUP_OPERATIONAL) & OP_MAX_OUTPUT_DONE) && !(status & I2C_STATUS_BUS_ACTIVE);
		if (!done) {
			sleep_us(10);
		}
	}
	qup_finish(status);
	*status_out = status;
	return done && !(status & I2C_STATUS_ERRORS);
}

/* ---- the RMI4 controller ----------------------------------------------------------------------------- */

struct rmi4_function {
	uint8_t query, command, control, data, irq_shift, irq_count;
	bool found;
};
static struct rmi4_function f01, f12;

#define MAX_FINGERS             10      /* synaptics,num-of-fingers */
#define OBJECT_BYTES            8       /* F12 data 1: type, x low/high, y low/high, z, wx, wy */

static uint16_t max_x, max_y;           /* the sensor's, from F12 control 8 */
static uint8_t f12_irq_mask;
static bool use_attention;              /* F12 raises the attention line: read only when it is low */

static uint32_t
popcount8(uint8_t v)
{
	uint32_t n = 0;

	for (; v; v &= (uint8_t)(v - 1)) {
		n++;
	}
	return n;
}

/*
 * The page description table: 6-byte entries from 0xe9 down (query, command, control and data base,
 * interrupt sources, function number); function 0 ends it. Interrupt status bits are handed out in
 * the table's order.
 */
static bool
rmi4_scan(void)
{
	uint8_t irq_shift = 0;

	for (unsigned entry = 0xe9; entry >= 0x0a; entry -= 6) {
		uint8_t pdt[6];
		uint32_t status;

		if (!i2c_read((uint8_t)entry, pdt, sizeof(pdt), &status)) {
			printf("synaptics: page description entry %#x unreadable: i2c status %#x (%s%s%s), sda %u scl %u\n", entry,
			    status, (status & I2C_STATUS_NACK) ? "no acknowledge " : "", (status & (1u << 2)) ? "bus error " : "",
			    (status & (1u << 4)) ? "arbitration lost" : "", status >> 26 & 1, status >> 27 & 1);
			return false;
		}
		printf("synaptics: pdt %#x: function %#x query %#x command %#x control %#x data %#x info %#x\n", entry, pdt[5],
		    pdt[0], pdt[1], pdt[2], pdt[3], pdt[4]);
		if (pdt[5] == 0 || pdt[5] == 0xff) {
			break;
		}
		struct rmi4_function function = { pdt[0], pdt[1], pdt[2], pdt[3], irq_shift, (uint8_t)(pdt[4] & 7), true };

		irq_shift = (uint8_t)(irq_shift + (pdt[4] & 7));
		if (pdt[5] == 0x12) {
			f12 = function;
		} else if (pdt[5] == 0x01) {
			uint8_t product[10];

			f01 = function;
			if (i2c_read((uint8_t)(pdt[0] + 11), product, sizeof(product), &status)) {
				char id[11] = { 0 };

				for (int i = 0; i < 10; i++) {
					id[i] = product[i] >= 32 && product[i] < 127 ? (char)product[i] : 0;
				}
				printf("synaptics: F01 product id \"%s\"\n", id);
			}
		}
	}
	return f01.found && f12.found;
}

/*
 * F12's registers are described by presence maps (synaptics_rmi4_f12_init): query 5 = size and control
 * presence bits, query 8 = size and data presence bits. Data 1 (the objects) follows data 0 if that is
 * there; control 8 holds the sensor's maximum X and Y.
 */
static bool
f12_layout(void)
{
	uint8_t q5[5], q8[3], ctrl8[4] = { 0 }, f01_ctrl[2] = { 0 }, f01_status = 0;
	uint32_t status;

	if (!i2c_read((uint8_t)(f12.query + 5), q5, sizeof(q5), &status) ||
	    !i2c_read((uint8_t)(f12.query + 8), q8, sizeof(q8), &status)) {
		printf("synaptics: F12 presence maps unreadable (i2c status %#x)\n", status);
		return false;
	}
	if ((q5[2] & 1) && i2c_read((uint8_t)(f12.control + popcount8(q5[1])), ctrl8, sizeof(ctrl8), &status)) {
		max_x = (uint16_t)(ctrl8[0] | ctrl8[1] << 8);
		max_y = (uint16_t)(ctrl8[2] | ctrl8[3] << 8);
	}
	if (max_x == 0 || max_y == 0) {
		max_x = 1079;   /* the panel: synaptics,display-coords 0 0 1079 1919 */
		max_y = 1919;
	}
	if (q8[1] & 1) {
		f12.data++;     /* skip data 0 */
	}
	f12_irq_mask = (uint8_t)(((1u << f12.irq_count) - 1) << f12.irq_shift);
	i2c_read(f01.data, &f01_status, 1, &status);
	i2c_read(f01.control, f01_ctrl, sizeof(f01_ctrl), &status);
	printf("synaptics: F12 q5 %02x %02x %02x %02x %02x, q8 %02x %02x %02x, objects at %#x, sensor up to %u x %u; F01 status %#x"
	    " control %#x (sleep mode %u) interrupt enable %#x (F12's %#x)\n", q5[0], q5[1], q5[2], q5[3], q5[4], q8[0], q8[1],
	    q8[2], f12.data, max_x, max_y, f01_status, f01_ctrl[0], f01_ctrl[0] & 3, f01_ctrl[1], f12_irq_mask);
	if (f01_ctrl[0] & 3) {
		/* asleep: back to normal operation */
		i2c_write(f01.control, (uint8_t)(f01_ctrl[0] & ~3u), &status);
		printf("synaptics: F01 woken from sleep mode %u (i2c status %#x)\n", f01_ctrl[0] & 3, status);
	}
	if ((f01_ctrl[1] & f12_irq_mask) != f12_irq_mask) {
		i2c_write((uint8_t)(f01.control + 1), (uint8_t)(f01_ctrl[1] | f12_irq_mask), &status);
		i2c_read((uint8_t)(f01.control + 1), &f01_ctrl[1], 1, &status);
		printf("synaptics: F12's interrupts enabled: %#x\n", f01_ctrl[1]);
	}
	use_attention = (f01_ctrl[1] & f12_irq_mask) == f12_irq_mask;
	return true;
}

/* ---- touches ------------------------------------------------------------------------------------------ */

static struct {
	uint32_t looks, reads, touches, bad, last_x, last_y;
} seen;

static struct {
	bool down;
	uint32_t x, y;
} contact[MAX_FINGERS];
static uint32_t flip;           /* boot-arg touch_flip: 1 = mirror left-right, 2 = mirror top-bottom */

static void
f12_report(void)
{
	static unsigned shown, failed;
	uint8_t objects[MAX_FINGERS * OBJECT_BYTES];
	uint32_t status;

	seen.reads++;
	/* two reads: the input FIFO holds 60 bytes, ten objects are 80 */
	if (!i2c_read(f12.data, objects, 7 * OBJECT_BYTES, &status) ||
	    !i2c_read((uint8_t)(f12.data + 7 * OBJECT_BYTES), objects + 7 * OBJECT_BYTES, (MAX_FINGERS - 7) * OBJECT_BYTES,
	    &status)) {
		seen.bad++;
		if (failed++ < 20) {
			printf("synaptics: object read failed (i2c status %#x)\n", status);
		}
		return;
	}
	for (uint32_t id = 0; id < MAX_FINGERS; id++) {
		const uint8_t *o = objects + id * OBJECT_BYTES;
		/* object type: 0 none, 1 finger, 2 stylus, 3 palm, 5 glove, 6 hovering finger */
		bool touching = o[0] == 1 || o[0] == 2 || o[0] == 5;
		uint32_t x = (uint32_t)(o[1] | o[2] << 8), y = (uint32_t)(o[3] | o[4] << 8);

		if (!touching) {
			/* a lifted object reads as 0,0: it lifts where it last was */
			if (contact[id].down) {
				contact[id].down = false;
				hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
			}
			continue;
		}
		if (shown < 40 && !contact[id].down) {
			shown++;
			printf("synaptics: finger %u (type %u) at %u, %u of %u x %u, z %u\n", id, o[0], x, y, max_x, max_y, o[5]);
		}
		seen.touches++;
		seen.last_x = x;
		seen.last_y = y;
		x = x >= max_x ? 65535 : x * 65535 / max_x;
		y = y >= max_y ? 65535 : y * 65535 / max_y;
		if (flip & 1) {
			x = 65535 - x;
		}
		if (flip & 2) {
			y = 65535 - y;
		}
		if (!contact[id].down) {
			hakios_touch_post(HAKIOS_TOUCH_DOWN, id, x, y);
		} else if (contact[id].x != x || contact[id].y != y) {
			hakios_touch_post(HAKIOS_TOUCH_MOVE, id, x, y);
		}
		contact[id].down = true;
		contact[id].x = x;
		contact[id].y = y;
	}
}

static bool
fingers_down(void)
{
	for (int id = 0; id < MAX_FINGERS; id++) {
		if (contact[id].down) {
			return true;
		}
	}
	return false;
}

/* Reading F01's interrupt status clears it, and with it the attention line; F12's bits say there are new objects. */
static void
poll_once(void)
{
	uint8_t irq = 0;
	uint32_t status;

	if (!use_attention) {
		f12_report();
		return;
	}
	if (i2c_read((uint8_t)(f01.data + 1), &irq, 1, &status) && (irq & f12_irq_mask)) {
		f12_report();
	}
}

/* ---- all of it together ---------------------------------------------------------------------------- */

static bool
bring_up(void)
{
	show_clocks("as found");
	show_pins();
	if (clocks_needed()) {
		checkpoint("the qup's clocks are off: turning them on");
		if (!clocks_on()) {
			return false;
		}
		show_clocks("now");
	}
	pins_for_bus();
	show_pins();
	checkpoint("first access to the qup");
	if (!qup_init()) {
		return false;
	}
	printf("synaptics: reading the page description table at %#x\n", TOUCH_ADDRESS);
	if (!rmi4_scan()) {
		printf("synaptics: no F01 and F12\n");
		return false;
	}
	bus_clock = CLK_400K;
	return f12_layout();
}

static void
synaptics_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t poll_us = 12000, delay_s = 10, idle = 0, idle_looks;

	PE_parse_boot_argn("touch_poll_us", &poll_us, sizeof(poll_us));
	PE_parse_boot_argn("touch_delay_s", &delay_s, sizeof(delay_s));
	PE_parse_boot_argn("touch_checkpoint_ms", &checkpoint_ms, sizeof(checkpoint_ms));
	if (poll_us < 1000 || poll_us > 100000) {
		poll_us = 12000;
	}
	idle_looks = 96000 / poll_us;   /* with a finger down, the objects are read every ~100 ms even with the line high */

	gcc = ml_io_map(GCC_PHYS, GCC_SIZE);
	tlmm = ml_io_map(TLMM_PHYS, TLMM_SIZE);
	qup = ml_io_map(QUP_PHYS, QUP_SIZE);
	if (gcc == 0 || tlmm == 0 || qup == 0) {
		printf("synaptics: a register block did not map (gcc %#lx tlmm %#lx qup %#lx)\n", gcc, tlmm, qup);
		thread_terminate(current_thread());
		return;
	}
	if (delay_s > 600) {
		delay_s = 10;
	}
	sleep_ms(delay_s * 1000);

	printf("synaptics: probing (poll %u us, checkpoints %u ms)\n", poll_us, checkpoint_ms);
	/* three more tries a minute apart (the controller may not have been ready), then ten minutes of saying why not */
	for (unsigned attempt = 1; !bring_up(); attempt++) {
		printf("synaptics: no touch screen (attempt %u)\n", attempt);
		checkpoint_ms = 0;
		if (attempt == 4) {
			for (int i = 0; i < 10; i++) {
				sleep_ms(60000);
				repeat_history();
			}
			thread_terminate(current_thread());
			return;
		}
		sleep_ms(60000);
		repeat_history();
	}
	printf("synaptics: touch screen ready; %s every %u us\n",
	    use_attention ? "the attention line is looked at" : "the objects are read (no interrupt)", poll_us);
	for (unsigned tick = 1;; tick++) {
		seen.looks++;
		if (!use_attention || attention()) {
			idle = 0;
			poll_once();
		} else if (fingers_down() && ++idle >= idle_looks) {
			/* a lift whose news was missed */
			idle = 0;
			f12_report();
		}
		if (tick % (60 * (1000000 / poll_us)) == 0) {
			printf("synaptics: so far %u object reads in %u looks (%u failed), %u finger positions, the last at %u, %u;"
			    " attention line %s\n", seen.reads, seen.looks, seen.bad, seen.touches, seen.last_x, seen.last_y,
			    attention() ? "low" : "high");
			if (tick / (60 * (1000000 / poll_us)) <= 10) {
				repeat_history();
			}
		}
		sleep_us(poll_us);
	}
}

void
hakios_synaptics_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("touchscreen", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	PE_parse_boot_argn("touch_flip", &flip, sizeof(flip));
	if (kernel_thread_start(synaptics_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
