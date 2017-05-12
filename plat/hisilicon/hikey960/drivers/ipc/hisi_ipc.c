
#include <arch_helpers.h>
#include <assert.h>
#include <hi3660.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <hisi_ipc.h>
#include <debug.h>

#define writel(val, addr)		mmio_write_32((uintptr_t)addr, (uint32_t)val)
#define readl(addr)				mmio_read_32((uintptr_t)addr)

#define REG_MBX_SOURCE_OFFSET(m)		(((m) << 6))
#define REG_MBX_DSET_OFFSET(m)		(((m) << 6) + 0x04)
#define REG_MBX_DCLEAR_OFFSET(m)		(((m) << 6) + 0x08)
#define REG_MBX_DSTATUS_OFFSET(m)	(((m) << 6) + 0x0C)
#define REG_MBX_MODE_OFFSET(m)		(((m) << 6) + 0x10)
#define REG_MBX_IMASK_OFFSET(m)		(((m) << 6) + 0x14)
#define REG_MBX_ICLR_OFFSET(m)		(((m) << 6) + 0x18)
#define REG_MBX_SEND_OFFSET(m)		(((m) << 6) + 0x1C)
#define REG_MBX_DATA_OFFSET(m, d)		(((m) << 6) + 0x20 + ((d) * 4))
#define REG_CPU_IMST_OFFSET(m)		(((m) << 3))
#define REG_IPC_LOCK_OFFSET			(0xA00)
#define REG_IPC_ACK_BIT_SHIFT			(1 << 7)
#define IPC_UNLOCK_VALUE				(0x1ACCE551)
/*********************************************************
 *bit[31:24]:0~AP
 *bit[23:16]:0x1~A15, 0x2~A7
 *bit[15:8]:0~ON, 1~OFF
 *bit[7:0]:0x3 cpu power mode
 *********************************************************/
#define IPC_CMD_TYPE(src_obj, cluster_obj, is_off, mode) \
	((src_obj << 24) | (((cluster_obj) + 1) << 16) | (is_off << 8) |(mode))

/*********************************************************
 *bit[15:8]:0~no idle, 1~idle
 *bit[7:0]:cpux
 *********************************************************/

#define IPC_CMD_PARA(is_idle, cpu) \
	((is_idle << 8) | (cpu))

#define REG_IPC_BASE	REG_IPC_SEC_BASE

enum src_id {
	SRC_IDLE = 0,
	SRC_A15 = 1 << 0,
	SRC_A7 = 1 << 1,
	SRC_IOM3 = 1 << 2,
	SRC_LPM3 = 1 << 3
};

/*lpm3's mailboxs are 13~17*/
enum lpm3_mbox_id {
	LPM3_MBX0 = 13,
	LPM3_MBX1,
	LPM3_MBX2,
	LPM3_MBX3,
	LPM3_MBX4,
};

static void cpu_relax(void)
{
	volatile int i;

	for (i = 0; i < 10; i++) {
		__asm__ volatile("nop");
	}
}

static inline void hisi_ipc_clear_ack(enum src_id source, enum lpm3_mbox_id mbox)
{
	unsigned int int_status = 0;

	do {
		int_status = readl(IPC_BASE + REG_MBX_MODE_OFFSET(mbox)) & 0xF0;
		cpu_relax();
	} while (int_status != REG_IPC_ACK_BIT_SHIFT);

	writel(source, IPC_BASE + REG_MBX_ICLR_OFFSET(mbox));
}

#define IPC_STATE_IDLE			0x10
static void hisi_ipc_send_cmd_with_ack(enum src_id source,
		enum lpm3_mbox_id mbox, unsigned int cmdtype, unsigned int cmdpara)
{
	unsigned int regval;
	unsigned int mask;

	writel(IPC_UNLOCK_VALUE, (IPC_BASE + REG_IPC_LOCK_OFFSET));
	/*wait for idle and occupy*/
	do {
		if (IPC_STATE_IDLE == readl(IPC_BASE + REG_MBX_MODE_OFFSET(mbox))) {
			writel(source, IPC_BASE + REG_MBX_SOURCE_OFFSET(mbox));
			regval = readl(IPC_BASE + REG_MBX_SOURCE_OFFSET(mbox));
			if (regval == source)
				break;
		}
		cpu_relax();

	} while (1);

	/*auto answer*/
	writel(0x1, IPC_BASE + REG_MBX_MODE_OFFSET(mbox));

	mask = (~((int)source | SRC_LPM3) & 0x3F);
	/*mask the other cpus*/
	writel(mask, IPC_BASE + REG_MBX_IMASK_OFFSET(mbox));
	/*set data*/
	writel(cmdtype, IPC_BASE + REG_MBX_DATA_OFFSET(mbox, 0));
	writel(cmdpara, IPC_BASE + REG_MBX_DATA_OFFSET(mbox, 1));
	/*send cmd*/
	writel(source, IPC_BASE + REG_MBX_SEND_OFFSET(mbox));
	/*wait ack and clear*/
	hisi_ipc_clear_ack(source, mbox);

	/*release mailbox*/
	writel(source, IPC_BASE + REG_MBX_SOURCE_OFFSET(mbox));
}

void hisi_ipc_pm_on_off(unsigned int core, unsigned int cluster, enum pm_mode mode)
{
	unsigned int cmdtype = 0;
	unsigned int cmdpara = 0;
	enum src_id source = SRC_IDLE;
	enum lpm3_mbox_id mailbox = (enum lpm3_mbox_id)(LPM3_MBX0 + core);

	cmdtype = IPC_CMD_TYPE(0, cluster, mode, 0x3);
	cmdpara = IPC_CMD_PARA(0, core);
	source = cluster ? SRC_A7: SRC_A15;
	hisi_ipc_send_cmd_with_ack(source, mailbox, cmdtype, cmdpara);
}

void hisi_ipc_pm_suspend(unsigned int core, unsigned int cluster, unsigned int affinity_level)
{
	unsigned int cmdtype = 0;
	unsigned int cmdpara = 0;
	enum src_id source = SRC_IDLE;
	enum lpm3_mbox_id mailbox = (enum lpm3_mbox_id)(LPM3_MBX0 + core);

	if (affinity_level == 0x3)
		cmdtype = IPC_CMD_TYPE(0, -1, 0x1, 0x3 + affinity_level);
	else
		cmdtype = IPC_CMD_TYPE(0, cluster, 0x1, 0x3 + affinity_level);

	cmdpara = IPC_CMD_PARA(1, core);
	source = cluster ? SRC_A7: SRC_A15;
	hisi_ipc_send_cmd_with_ack(source, mailbox, cmdtype, cmdpara);
}

void hisi_ipc_psci_system_off(unsigned int core, unsigned int cluster)
{
    unsigned int cmdtype = 0;
    unsigned int cmdpara = 0;
    enum src_id source = SRC_IDLE;
    enum lpm3_mbox_id mailbox = (enum lpm3_mbox_id)(LPM3_MBX0 + core);

    cmdtype = IPC_CMD_TYPE(0, (0x10 - 1), 0x1, 0x0);
    cmdpara = IPC_CMD_PARA(0, 0);
    source = cluster ? SRC_A7: SRC_A15;
    hisi_ipc_send_cmd_with_ack(source, mailbox, cmdtype, cmdpara);
}

void hisi_ipc_psci_system_reset(unsigned int core, unsigned int cluster, unsigned int cmd_id)
{
    unsigned int cmdtype = 0;
    unsigned int cmdpara = 0;
    enum src_id source = SRC_IDLE;
    enum lpm3_mbox_id mailbox = (enum lpm3_mbox_id)(LPM3_MBX0 + core);

    cmdtype = IPC_CMD_TYPE(0, (0x10 - 1), 0x0, 0x0);
    cmdpara = cmd_id;
    source = cluster ? SRC_A7: SRC_A15;
    hisi_ipc_send_cmd_with_ack(source, mailbox, cmdtype, cmdpara);
}

int hisi_ipc_init(void)
{
	int ret = 0;
	enum lpm3_mbox_id  i = LPM3_MBX0;

	writel(IPC_UNLOCK_VALUE, (IPC_BASE + REG_IPC_LOCK_OFFSET));
	for (i = LPM3_MBX0; i <= LPM3_MBX4; i++) {
		writel(1, IPC_BASE + REG_MBX_MODE_OFFSET(i));
		writel(((int)SRC_IOM3 | (int)SRC_A15 | (int)SRC_A7),\
			IPC_BASE + REG_MBX_IMASK_OFFSET(i));
		writel(SRC_A7, IPC_BASE + REG_MBX_ICLR_OFFSET(i));
	}

	return ret;
}
