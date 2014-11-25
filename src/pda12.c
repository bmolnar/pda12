/*******************************************************************************

  Signatec PDA12 Driver
  Copyright(c) 2009 Brian Molnar

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Contact Information:
  Brian Molnar <brian.molnar@gmail.com>

*******************************************************************************/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <asm/unaligned.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>

#include "pda12.h"

#define DRV_NAME		"pda12"
#define DRV_VERSION		"1.0.0"
#define DRV_DESCRIPTION		"Signatec PDA12 Driver"
#define DRV_COPYRIGHT		"Copyright(c) 2009 Brian Molnar"
#define PFX			DRV_NAME ": "

#define PRINTK(level, fmt, args...) printk(level DRV_NAME ": " fmt, ##args)

MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_AUTHOR(DRV_COPYRIGHT);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

struct pda12_reg0 {
#if defined(__BIG_ENDIAN_BITFIELD)
	u32 OM:3;	// Operating Mode
	u32 BC:2;	// SAB Configuration
	u32 TR0:1;	// Trigger Mode
	u32 TR1:1;	// Trigger Mode
	u32 TRS:2;	// Trigger Source
	u32 TSL:1;	// Trigger Slope
	u32 ST:1;	// Software Trigger
	u32 CS:2;	// Clock Source
	u32 CD:3;	// Clock Divider
	u32 SS:4;	// Segment Size
	u32 VR:2;	// Voltage Range
	u32 TM2:1;
	u32 PIE:1;
	u32 DMA:1;	// PCI DMA Enable
	u32 IEN:3;	// SAB Interrupt
	u32 TSEN:1;	// Time Stamp Enable
	u32 TSD:2;	// Time Stamp Divider
	u32 SCS:1;	// Input Channel
#elif defined(__LITTLE_ENDIAN_BITFIELD)
	u32 SCS:1;	// Input Channel
	u32 TSD:2;	// Time Stamp Divider
	u32 TSEN:1;	// Time Stamp Enable
	u32 IEN:3;	// SAB Interrupt
	u32 DMA:1;	// PCI DMA Enable
	u32 PIE:1;
	u32 TM2:1;
	u32 VR:2;	// Voltage Range
	u32 SS:4;	// Segment Size
	u32 CD:3;	// Clock Divider
	u32 CS:2;	// Clock Source
	u32 ST:1;	// Software Trigger
	u32 TSL:1;	// Trigger Slope
	u32 TRS:2;	// Trigger Source
	u32 TR1:1;	// Trigger Mode
	u32 TR0:1;	// Trigger Mode
	u32 BC:2;	// SAB Configuration
	u32 OM:3;	// Operating Mode
#else
#error "Please fix <asm/byteorder.h>"
#endif
};

struct pda12_reg1 {
#if defined(__BIG_ENDIAN_BITFIELD)
	u32 OF1:8;	// Channel 1 Offset
	u32 UNUSED0:24;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
	u32 UNUSED0:24;
	u32 OF1:8;	// Channel 1 Offset
#else
#error "Please fix <asm/byteorder.h>"
#endif
};

struct pda12_reg2 {
#if defined(__BIG_ENDIAN_BITFIELD)
	u32 ACL:16;	// Low Address Counter
	u32 ACH:6;	// High Address Counter
	u32 UNUSED0:2;
	u32 ACRST:1;	// Auto Address Counter Reset
	u32 UNUSED1:2;
	u32 CONF:1;	// Configuration Flag
	u32 UNUSED2:1;
	u32 RO:1;	// Roll Over Flag
	u32 FF:1;	// FIFO Flag
	u32 MF:1;	// Memory Full Flag
#elif defined(__LITTLE_ENDIAN_BITFIELD)
	u32 MF:1;	// Memory Full Flag
	u32 FF:1;	// FIFO Flag
	u32 RO:1;	// Roll Over Flag
	u32 UNUSED2:1;
	u32 CONF:1;	// Configuration Flag
	u32 UNUSED1:2;
	u32 ACRST:1;	// Auto Address Counter Reset
	u32 UNUSED0:2;
	u32 ACH:6;	// High Address Counter
	u32 ACL:16;	// Low Address Counter
#else
#error "Please fix <asm/byteorder.h>"
#endif
};

struct pda12_reg3 {
#if defined(__BIG_ENDIAN_BITFIELD)
	u32 OF2:8;	// Channel 2 Offset
	u32 UNUSED0:24;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
	u32 UNUSED0:24;
	u32 OF2:8;	// Channel 2 Offset
#else
#error "Please fix <asm/byteorder.h>"
#endif
};

struct pda12_reg4 {
#if defined(__BIG_ENDIAN_BITFIELD)
	u32 EA:16;	// Ending Address
	u32 FR:1;	// Address Free Run
	u32 UNUSED0:15;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
	u32 UNUSED0:15;
	u32 FR:1;	// Address Free Run
	u32 EA:16;	// Ending Address
#else
#error "Please fix <asm/byteorder.h>"
#endif
};

struct pda12_reg5 {
#if defined(__BIG_ENDIAN_BITFIELD)
	u32 TL:8;	// Trigger Level
	u32 UNUSED0:24;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
	u32 UNUSED0:24;
	u32 TL:8;	// Trigger Level
#else
#error "Please fix <asm/byteorder.h>"
#endif
};

struct pda12_reg6 {
#if defined(__BIG_ENDIAN_BITFIELD)
	u32 TDORPT:16;	// Trigger Delay or Pre-Trig Samples
	u32 TDEN:1;	// Trigger Delay Enable
	u32 UNUSED0:3;
	u32 SC:2;	// SC0-SC1: SAB Clock selected bits to select clock source
	u32 SCDI:1;     // SC3: P Clock divide by 1 or 2
	u32 UNUSED1:9;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
	u32 UNUSED1:9;
	u32 SCDI:1;     // SC3: P Clock divide by 1 or 2
	u32 SC:2;	// SC0-SC1: SAB Clock selected bits to select clock source
	u32 UNUSED0:3;
	u32 TDEN:1;	// Trigger Delay Enable
	u32 TDORPT:16;	// Trigger Delay or Pre-Trig Samples
#else
#error "Please fix <asm/byteorder.h>"
#endif
};

struct pda12_reg7 {
#if defined(__BIG_ENDIAN_BITFIELD)
	u32 MD:8;
	u32 UNUSED0:24;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
	u32 UNUSED0:24;
	u32 MD:8;
#else
#error "Please fix <asm/byteorder.h>"
#endif
};

struct pda12_dev {
	struct pci_dev		*pci_dev;
	void __iomem		*iomem0;
	void __iomem		*iomem1;
	void __iomem		*iomem2;

	dma_addr_t		dma_addr;
	unsigned char		*dma_buf;
	int			dma_len;

	u8			*buffer;

	uint8_t			irq;

	u32			regs[8];
	struct pda12_reg0	*reg0;
	struct pda12_reg1	*reg1;
	struct pda12_reg2	*reg2;
	struct pda12_reg3	*reg3;
	struct pda12_reg4	*reg4;
	struct pda12_reg5	*reg5;
	struct pda12_reg6	*reg6;
	struct pda12_reg7	*reg7;
};

struct pda12_dev pda12dev;


/*
 * PCI Operation Register Offsets
 * for AMCC S5933 PCI Controller
 */
#define AMCC_OP_REG_OMB1         0x00
#define AMCC_OP_REG_OMB2         0x04
#define AMCC_OP_REG_OMB3         0x08
#define AMCC_OP_REG_OMB4         0x0C
#define AMCC_OP_REG_IMB1         0x11
#define AMCC_OP_REG_IMB2         0x14
#define AMCC_OP_REG_IMB3         0x18
#define AMCC_OP_REG_IMB4         0x1C
#define AMCC_OP_REG_FIFO         0x20
#define AMCC_OP_REG_MWAR         0x24
#define AMCC_OP_REG_MWTC         0x28
#define AMCC_OP_REG_MRAR         0x2C
#define AMCC_OP_REG_MRTC         0x30
#define AMCC_OP_REG_MBEF         0x34
#define AMCC_OP_REG_INTCSR       0x38
#define AMCC_OP_REG_MCSR         0x3C
#define AMCC_OP_REG_MCSR_NVDATA  (AMCC_OP_REG_MCSR + 2)
#define AMCC_OP_REG_MCSR_NVCMD   (AMCC_OP_REG_MCSR + 3)

/*
 * Interrupt Control/Status Register (INTCSR) flags
 */
#define AMCC_INTCSR_ANY            0x00800000
#define AMCC_INTCSR_TARGET_ABORT   0x00200000
#define AMCC_INTCSR_MASTER_ABORT   0x00100000
#define AMCC_INTCSR_READ_TC        0x00080000
#define AMCC_INTCSR_WRITE_TC       0x00040000
#define AMCC_INTCSR_IMB            0x00020000
#define AMCC_INTCSR_OMB            0x00010000

#define AMCC_INTCSR_EN_READ_TC     0x00008000
#define AMCC_INTCSR_EN_WRITE_TC    0x00004000
#define AMCC_INTCSR_EN_IMB         0x00001000
#define AMCC_INTCSR_EN_OMB         0x00000010

/*
 * Master Control/Status Register (MCSR) flags
 */
#define AMCC_MCSR_ADDPCI_FIFO_RST  (1 << 26)
#define AMCC_MCSR_PCIADD_FIFO_RST  (1 << 25)
#define AMCC_MCSR_EN_WRITE         (1 << 10)
#define AMCC_MCSR_WR_PRIO          (1 << 8)

#define NVRAM_BUSY               0x80
#define NVCMD_LOAD_LOW           0x80
#define NVCMD_LOAD_HIGH          0xA0
#define NVCMD_BEGIN_READ         0xE0


struct amcc_op_reg_intcsr {
#if defined(__BIG_ENDIAN_BITFIELD)
	u32 UNUSED0:8;
	u32 asserted:1;
	u32 UNUSED1:1;
	u32 tabrt:1;
	u32 mabrt:1;
	u32 rtcint:1;
	u32 wtcint:1;
	u32 imbint:1;
	u32 ombint:1;
	u32 rtcint_enable:1;
	u32 wtcint_enable:1;
	u32 UNUSED2:1;
	u32 imbint_enable:1;
	u32 imbint_sel:4;
	u32 UNUSED3:3;
	u32 ombint_enable:1;
	u32 ombint_sel:4;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
	u32 ombint_sel:4;
	u32 ombint_enable:1;
	u32 UNUSED3:3;
	u32 imbint_sel:4;
	u32 imbint_enable:1;
	u32 UNUSED2:1;
	u32 wtcint_enable:1;
	u32 rtcint_enable:1;
	u32 ombint:1;
	u32 imbint:1;
	u32 wtcint:1;
	u32 rtcint:1;
	u32 mabrt:1;
	u32 tabrt:1;
	u32 UNUSED1:1;
	u32 asserted:1;
	u32 UNUSED0:8;
#else
#error "Please fix <asm/byteorder.h>"
#endif
};


#define IOMEM_OFFSET(MEM, OFF) ((void __iomem *)(((unsigned long)(MEM)) + (OFF)))

#define DMA_SIZE 65536

int major;

static void printk_reg0(struct pda12_reg0 *reg)
{
	PRINTK(KERN_INFO, "reg0->OM: %d\n", reg->OM);
	PRINTK(KERN_INFO, "reg0->BC: %d\n", reg->BC);
	PRINTK(KERN_INFO, "reg0->TR0: %d\n", reg->TR0);
	PRINTK(KERN_INFO, "reg0->TR1: %d\n", reg->TR1);
	PRINTK(KERN_INFO, "reg0->TRS: %d\n", reg->TRS);
	PRINTK(KERN_INFO, "reg0->TSL: %d\n", reg->TSL);
	PRINTK(KERN_INFO, "reg0->ST: %d\n", reg->ST);
	PRINTK(KERN_INFO, "reg0->CS: %d\n", reg->CS);
	PRINTK(KERN_INFO, "reg0->CD: %d\n", reg->CD);
	PRINTK(KERN_INFO, "reg0->SS: %d\n", reg->SS);
	PRINTK(KERN_INFO, "reg0->VR: %d\n", reg->VR);
	PRINTK(KERN_INFO, "reg0->TM2: %d\n", reg->TM2);
	PRINTK(KERN_INFO, "reg0->PIE: %d\n", reg->PIE);
	PRINTK(KERN_INFO, "reg0->DMA: %d\n", reg->DMA);
	PRINTK(KERN_INFO, "reg0->IEN: %d\n", reg->IEN);
	PRINTK(KERN_INFO, "reg0->TSEN: %d\n", reg->TSEN);
	PRINTK(KERN_INFO, "reg0->TSD: %d\n", reg->TSD);
	PRINTK(KERN_INFO, "reg0->SCS: %d\n", reg->SCS);
}

static void printk_reg2(struct pda12_reg2 *reg)
{
	PRINTK(KERN_INFO, "reg2->ACL: %d\n", reg->ACL);
	PRINTK(KERN_INFO, "reg2->ACH: %d\n", reg->ACH);
	PRINTK(KERN_INFO, "reg2->UNUSED0: %d\n", reg->UNUSED0);
	PRINTK(KERN_INFO, "reg2->ACRST: %d\n", reg->ACRST);
	PRINTK(KERN_INFO, "reg2->UNUSED1: %d\n", reg->UNUSED1);
	PRINTK(KERN_INFO, "reg2->CONF: %d\n", reg->CONF);
	PRINTK(KERN_INFO, "reg2->UNUSED2: %d\n", reg->UNUSED2);
	PRINTK(KERN_INFO, "reg2->RO: %d\n", reg->RO);
	PRINTK(KERN_INFO, "reg2->FF: %d\n", reg->FF);
	PRINTK(KERN_INFO, "reg2->MF: %d\n", reg->MF);
}

static void printk_reg6(struct pda12_reg6 *reg)
{
	PRINTK(KERN_INFO, "reg6->TDORPT: %d\n", reg->TDORPT);
	PRINTK(KERN_INFO, "reg6->TDEN: %d\n", reg->TDEN);
	PRINTK(KERN_INFO, "reg6->UNUSED0: %d\n", reg->UNUSED0);
	PRINTK(KERN_INFO, "reg6->SC: %d\n", reg->SC);
}

static void pda12_init_registers(struct pda12_dev *dev)
{
	int idx;

	for (idx = 0; idx < 8; idx++)
		dev->regs[idx] = 0;

	dev->reg0 = (struct pda12_reg0 *) &dev->regs[0];
	dev->reg1 = (struct pda12_reg1 *) &dev->regs[1];
	dev->reg2 = (struct pda12_reg2 *) &dev->regs[2];
	dev->reg3 = (struct pda12_reg3 *) &dev->regs[3];
	dev->reg4 = (struct pda12_reg4 *) &dev->regs[4];
	dev->reg5 = (struct pda12_reg5 *) &dev->regs[5];
	dev->reg6 = (struct pda12_reg6 *) &dev->regs[6];
	dev->reg7 = (struct pda12_reg7 *) &dev->regs[7];
}

static u32 pda12_reg_read(struct pda12_dev *dev, int reg)
{
    return ioread32be(IOMEM_OFFSET(dev->iomem1, (reg << 2)));
    //return ioread32(IOMEM_OFFSET(dev->iomem1, (reg << 2)));
}

static void pda12_reg_write(struct pda12_dev *dev, int reg)
{
    iowrite32be(dev->regs[reg], IOMEM_OFFSET(dev->iomem1, (reg << 2)));
    //iowrite32(dev->regs[reg], IOMEM_OFFSET(dev->iomem1, (reg << 2)));
}

static u8 pda12_eprom_read(struct pda12_dev *dev, unsigned int parm)
{
	void __iomem *nvdata = IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_MCSR_NVDATA);
	void __iomem *nvcmd = IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_MCSR_NVCMD);
	int idx;

	idx = 1024;
	while (idx-- > 0) {
		if ((ioread8(nvcmd) & NVRAM_BUSY) != NVRAM_BUSY)
			break;
	}

	iowrite16((NVCMD_LOAD_LOW << 8) | (parm & 0xFF), nvdata);
	iowrite16((NVCMD_LOAD_HIGH << 8) | ((parm >> 8) & 0xFF), nvdata);
	iowrite8(NVCMD_BEGIN_READ, nvcmd);

	idx = 1024;
	while (idx-- > 0) {
		if ((ioread8(nvcmd) & NVRAM_BUSY) != NVRAM_BUSY)
			break;
	}

	return (u8) ioread8(nvdata);
}

#define PDA12_MODE_OFF      0
#define PDA12_MODE_ACQ_PCI  1
#define PDA12_MODE_ACQ_1    2
#define PDA12_MODE_ACQ_2    3
#define PDA12_MODE_PCI      4
#define PDA12_MODE_AUX      5
#define PDA12_MODE_WRAM     6
#define PDA12_MODE_STBY     7

static int pda12_set_mode(struct pda12_dev *dev, u8 mode)
{
	dev->reg0->OM = mode;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_set_start_addr(struct pda12_dev *dev, u32 addr)
{
	dev->reg2->ACL = (unsigned int)((addr << 2) & 0xFFFF);
	dev->reg2->ACH = (unsigned int)((addr >> 14) & 0x003F);
	dev->reg2->ACRST = 1;
	pda12_reg_write(dev, 2);
	return 0;
}

static int pda12_set_end_addr(struct pda12_dev *dev, u32 addr)
{
	dev->reg4->EA = addr >> 4;
	if (addr >= 1048576)
		dev->reg4->FR = 1;
	else
		dev->reg4->FR = 0;
	pda12_reg_write(dev, 4);
	return 0;
}

static int pda12_acq_pci_data(struct pda12_dev *dev, u32 addr, u32 count)
{
	struct pda12_reg0 *reg0 = (struct pda12_reg0 *) &dev->regs[0];
	u32 intcsr;
        u32 mcsr;

	pda12_set_start_addr(dev, 0);

	/* Set DMA address and length */
	iowrite32be(addr, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_MWAR));
	iowrite32be(count, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_MWTC));

	/* Clear WRITE_TC interrupt flag */
	intcsr = ioread32be(IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_INTCSR));
        intcsr |= AMCC_INTCSR_WRITE_TC;
	iowrite32be(intcsr, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_INTCSR));

	/* Enable WRITE_TC interrupt */
	intcsr = ioread32be(IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_INTCSR));
        intcsr |= AMCC_INTCSR_EN_WRITE_TC;
	iowrite32be(intcsr, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_INTCSR));

	pda12_set_mode(dev, PDA12_MODE_ACQ_PCI);

	/* Enable PCI bus mastering */
        mcsr = (AMCC_MCSR_ADDPCI_FIFO_RST | AMCC_MCSR_PCIADD_FIFO_RST | AMCC_MCSR_EN_WRITE | AMCC_MCSR_WR_PRIO);
	iowrite32be(mcsr, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_MCSR));

	return 0;
}

static int pda12_copy_ram_block(struct pda12_dev *dev, void *address, int block)
{
	//int idx;
	//char *ptr = (char *) address;

	pda12_set_start_addr(dev, block * (8192 >> 2));

	pda12_set_mode(dev, PDA12_MODE_PCI);
	//for (idx = 0; idx < 8192; idx++)
	//ptr[idx] = ioread8(IOMEM_OFFSET(dev->iomem2, idx));

	memcpy(address, dev->iomem2, 8192);
	return 0;
}

static int pda12_copy_ram_block_dma(struct pda12_dev *dev, dma_addr_t dma_addr, int block)
{
	u32 intcsr;
        u32 mcsr;

	pda12_set_start_addr(dev, block * (8192 >> 2));

	/* Set DMA address and length */
	iowrite32be(dma_addr, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_MWAR));
	iowrite32be(8192, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_MWTC));

	/* Clear WRITE_TC interrupt flag */
	intcsr = ioread32be(IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_INTCSR));
        intcsr |= AMCC_INTCSR_WRITE_TC;
	iowrite32be(intcsr, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_INTCSR));

	/* Enable WRITE_TC interrupt */
	intcsr = ioread32be(IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_INTCSR));
        intcsr |= AMCC_INTCSR_EN_WRITE_TC;
	iowrite32be(intcsr, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_INTCSR));

	pda12_set_mode(dev, PDA12_MODE_PCI);

	/* Enable PCI bus mastering */
        mcsr = (AMCC_MCSR_ADDPCI_FIFO_RST | AMCC_MCSR_PCIADD_FIFO_RST | AMCC_MCSR_EN_WRITE | AMCC_MCSR_WR_PRIO);
	iowrite32be(mcsr, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_MCSR));

	return 0;
}

static int pda12_print_dma_data(struct pda12_dev *dev)
{
	char hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
	char buffer[512];
	char *p;
	unsigned char byte;
	int idx;

	printk("DMA BUFFER:\n");

	p = &buffer[0];
	for (idx = 0; idx < 128; idx++) {
		byte = dev->dma_buf[idx];

		*(p++) = hex[(byte >> 4) & 0x0F];
		*(p++) = hex[byte & 0x0F];
		*(p++) = ((idx + 1) % 16) ? ' ' : '\n';
	}
	*(p++) = 0;

	printk("%s", buffer);

	return 0;
}

static int pda12_set_offset(struct pda12_dev *dev, unsigned int channel, unsigned int val)
{
	if (channel == 1) {
		dev->reg1->OF1 = val;
		pda12_reg_write(dev, 1);
	} else if (channel == 2) {
		dev->reg3->OF2 = val;
		pda12_reg_write(dev, 3);
	}

	return 0;
}

static int pda12_set_gain(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->VR = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_set_channel_mode(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->SCS = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_set_clk_divide(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->CD = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_set_clk_freq(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->CS = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_set_segment_size(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->SS = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_set_trigger_level(struct pda12_dev *dev, unsigned int val)
{
	dev->reg5->TL = val;
	pda12_reg_write(dev, 5);
	return 0;
}

static int pda12_set_trigger_mode(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->TR0 = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_set_trigger_source(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->TRS = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_set_trigger_slope(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->TSL = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_set_trigger_delay(struct pda12_dev *dev, unsigned int val)
{
	dev->reg6->TDORPT = val;
	pda12_reg_write(dev, 6);
	return 0;
}

static int pda12_set_trigger_pretrig(struct pda12_dev *dev, unsigned int val)
{
	dev->reg6->TDORPT = val;
	pda12_reg_write(dev, 6);
	return 0;
}

static int pda12_enable_trigger_delay(struct pda12_dev *dev, unsigned int val)
{
	dev->reg6->TDEN = val;
	pda12_reg_write(dev, 6);
	return 0;
}

static int pda12_enable_trigger_pretrig(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->TR0 = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_enable_trigger_gate(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->TM2 = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_software_trigger(struct pda12_dev *dev)
{
	dev->reg0->ST = 1;
	pda12_reg_write(dev, 0);
	dev->reg0->ST = 0;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_enable_timestamp(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->TSEN = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_enable_dma(struct pda12_dev *dev, unsigned int val)
{
	dev->reg0->DMA = val;
	pda12_reg_write(dev, 0);
	return 0;
}

static int pda12_SET_SETTINGS(struct pda12_dev *dev)
{

	pda12_set_start_addr(dev, 0);
	pda12_set_end_addr(dev, 1048576);

	/* 0 = Single Shot, 1 = Segmented Mode */
	pda12_set_trigger_mode(dev, 0);
	pda12_set_trigger_level(dev, 0);
	pda12_set_trigger_source(dev, 0);
	pda12_set_trigger_slope(dev, 0);


	pda12_enable_trigger_delay(dev, 0);
	pda12_set_trigger_delay(dev, 0);

	pda12_enable_trigger_pretrig(dev, 0);
	pda12_set_trigger_pretrig(dev, 0);


	pda12_enable_timestamp(dev, 0);


	/*
	 * 0 = 0, 1 = 64, 2 = 128, 3 = 192, 4 = 256, 5 = 320,
	 * 6 = 384, 7 = 448, 8 = 512, 9 = 1024, 10 = 2048, 11 = 4096,
	 * 12 = 8192, 13 = 16384, 14 = 32768, 15 = 65536
	 */
	pda12_set_segment_size(dev, 12);


	/* 0 = Dual Channel, 1 = Single Channel */
	pda12_set_channel_mode(dev, 1);
	pda12_set_offset(dev, 1, 128);
	pda12_set_offset(dev, 2, 128);

	/* 0 = 3V, 1 = 1V, 2 = 300mV, 3 = 100mV */
	pda12_set_gain(dev, 0);

	/* 0 = 50 MHz, 1 = 62.5 MHz */
	pda12_set_clk_freq(dev, 1);

	pda12_set_clk_divide(dev, 0);

	return 0;
}

static int pda12_PCIACQTEST_1(struct pda12_dev *dev)
{

	pda12_set_mode(dev, PDA12_MODE_STBY);

	pda12_set_offset(dev, 1, 128);
	pda12_set_offset(dev, 2, 128);
	pda12_set_trigger_level(dev, 128);
	pda12_set_trigger_mode(dev, 1);
	pda12_set_trigger_source(dev, 0);


	pda12_enable_trigger_delay(dev, 1);
	pda12_set_trigger_delay(dev, 0);

	pda12_enable_trigger_pretrig(dev, 0);
	pda12_set_trigger_pretrig(dev, 0);


	pda12_enable_timestamp(dev, 1);

	pda12_set_segment_size(dev, 1);

	pda12_set_gain(dev, 2);
	pda12_set_channel_mode(dev, 1);
	pda12_set_clk_divide(dev, 7);
	pda12_set_clk_freq(dev, 0);

	pda12_set_start_addr(dev, 0);
	//pda12_set_end_addr(dev, 1048576);
	pda12_set_end_addr(dev, DMA_SIZE);

	pda12_enable_dma(dev, 1);

	pci_dma_sync_single_for_device(dev->pci_dev, dev->dma_addr, DMA_SIZE, PCI_DMA_FROMDEVICE);

	pda12_acq_pci_data(dev, dev->dma_addr, dev->dma_len);
	return 0;
}

static int pda12_PCIACQTEST_2(struct pda12_dev *dev)
{
	uint8_t status;

	//pda12_set_mode(dev, PDA12_MODE_STBY);

	//status = ioread8(IOMEM_OFFSET(dev->iomem0, (AMCC_OP_REG_INTCSR + 2)));

	/* Disable bus mastering */
	//iowrite8(0x11, IOMEM_OFFSET(dev->iomem0, (AMCC_OP_REG_MCSR + 1)));

	//iowrite32(EN_PDAPC_TC_INT, IOMEM_OFFSET(dev->iomem0, AMCC_OP_REG_INTCSR));

	pda12_enable_dma(dev, 0);

	pci_dma_sync_single_for_cpu(dev->pci_dev, dev->dma_addr, DMA_SIZE, PCI_DMA_FROMDEVICE);

	pda12_print_dma_data(dev);

	return 0;
}

static int pda12_SPEEDTEST_1(struct pda12_dev *dev)
{
	int i, j;

	pda12_set_mode(dev, PDA12_MODE_STBY);

	/* Write pattern to signal RAM */
	for (i = 0; i < 16; i++) {
		pda12_set_start_addr(dev, i * (8192 >> 2));
		pda12_set_mode(dev, PDA12_MODE_WRAM);

		for (j = 0; j < 8192; j += 4) {
			iowrite16(0x5a5, IOMEM_OFFSET(dev->iomem2, j));
			iowrite16(0xa5a, IOMEM_OFFSET(dev->iomem2, (j + 2)));
		}
	}

	pda12_set_mode(dev, PDA12_MODE_STBY);

	pda12_set_start_addr(dev, 0);
	pda12_set_end_addr(dev, 1048576);

	pda12_enable_dma(dev, 1);

	pda12_copy_ram_block_dma(dev, dev->dma_addr, 0);

	return 0;
}

static long pda12_chrdev_unlocked_ioctl(struct file *filp, unsigned int cmd,
                              unsigned long arg)
{
	uint32_t *p32 = (uint32_t *) arg;
	struct pda12_dev *dev = &pda12dev;

	switch(cmd) {
	case IOCTL_GET_REG0:
		*p32 = dev->regs[0];
		return 0;
	case IOCTL_GET_REG1:
		*p32 = dev->regs[1];
		return 0;
	case IOCTL_GET_REG2:
		*p32 = dev->regs[2];
		return 0;
	case IOCTL_GET_REG3:
		*p32 = dev->regs[3];
		return 0;
	case IOCTL_GET_REG4:
		*p32 = dev->regs[4];
		return 0;
	case IOCTL_GET_REG5:
		*p32 = dev->regs[5];
		return 0;
	case IOCTL_GET_REG6:
		*p32 = dev->regs[6];
		return 0;
	case IOCTL_GET_REG7:
		*p32 = dev->regs[7];
		return 0;
	case IOCTL_SET_REG0:
		dev->regs[0] = *p32;
		pda12_reg_write(dev, 0);
		return 0;
	case IOCTL_SET_REG1:
		dev->regs[1] = *p32;
		pda12_reg_write(dev, 1);
		return 0;
	case IOCTL_SET_REG2:
		dev->regs[2] = *p32;
		pda12_reg_write(dev, 2);
		return 0;
	case IOCTL_SET_REG3:
		dev->regs[3] = *p32;
		pda12_reg_write(dev, 3);
		return 0;
	case IOCTL_SET_REG4:
		dev->regs[4] = *p32;
		pda12_reg_write(dev, 4);
		return 0;
	case IOCTL_SET_REG5:
		dev->regs[5] = *p32;
		pda12_reg_write(dev, 5);
		return 0;
	case IOCTL_SET_REG6:
		dev->regs[6] = *p32;
		pda12_reg_write(dev, 6);
		return 0;
	case IOCTL_SET_REG7:
		dev->regs[7] = *p32;
		pda12_reg_write(dev, 7);
		return 0;
	}

	return -EINVAL;
}

static int pda12_chrdev_open(struct inode *inode, struct file *filp)
{
	PRINTK(KERN_INFO, "%s\n", __FUNCTION__);
	return 0;
}

static ssize_t pda12_chrdev_read(struct file *filp, char *buf, size_t count, loff_t *offset)
{
	int len;
	int block;
	int boff;
	int bufpos;
	int rampos;

	//PRINTK(KERN_INFO, "%s: count=%u, offset=%u\n",
	//       __FUNCTION__, (unsigned int) count, (unsigned int) *offset);

	if (!pda12dev.buffer) {
		PRINTK(KERN_INFO, "No read buffer. Skipping...\n");
		return (ssize_t) -ENOMEM;
	}


	//pda12_set_mode(&pda12dev, PDA12_MODE_PCI);

	bufpos = 0;
	rampos = *offset;

	while (bufpos < count) {
		block = rampos / 8192;
		boff = rampos % 8192;

		len = 8192 - boff;
		len = (count - bufpos) < len ? (count - bufpos) : len;

		pda12_copy_ram_block(&pda12dev, pda12dev.buffer, block);

		copy_to_user((buf + bufpos), (pda12dev.buffer + boff), len);

		bufpos += len;
		rampos += len;
	}

	*offset += count;
	return (ssize_t) count;
}

static ssize_t pda12_chrdev_write(struct file *filp, const char *buf, size_t count, loff_t *offset)
{
	unsigned int call = (unsigned int) count;
	u32 _reg;
	u8 byte;
	int idx;
	u32 serial;


	PRINTK(KERN_INFO, "pda12_chrdev_write call=%d\n", call);

	switch (call) {
	case 1:
		for (idx = 0; idx < 8; idx++)
			pda12dev.regs[idx] = pda12_reg_read(&pda12dev, idx);
		break;

	case 2:
		for (idx = 0; idx < 8; idx++)
			pda12dev.regs[idx] = 0;
		break;

	case 3:
		for (idx = 0; idx < 8; idx++)
			pda12_reg_write(&pda12dev, idx);
		break;



	case 4:
		pda12_enable_dma(&pda12dev, 1);
		break;

	case 5:
		pda12_enable_dma(&pda12dev, 0);
		break;




	case 6:
		pda12_set_mode(&pda12dev, PDA12_MODE_STBY);
		iowrite8(0x11, IOMEM_OFFSET(pda12dev.iomem0, (AMCC_OP_REG_MCSR + 1)));
		iowrite32((_reg & ~AMCC_INTCSR_EN_WRITE_TC), IOMEM_OFFSET(pda12dev.iomem0, AMCC_OP_REG_INTCSR));
		break;

	case 7:
		pda12_copy_ram_block_dma(&pda12dev, pda12dev.dma_addr, 0);
		break;

	case 8:
		pda12_acq_pci_data(&pda12dev, pda12dev.dma_addr, pda12dev.dma_len);
		break;

	case 9:
		/* Disable bus mastering */
		//iowrite8(0x11, IOMEM_OFFSET(pda12dev.iomem0, (AMCC_OP_REG_MCSR + 1)));

		pda12_print_dma_data(&pda12dev);
		break;

	case 10:
		_reg = pda12dev.regs[0];
		PRINTK(KERN_INFO, "REG0: %08x\n", (unsigned int) _reg);
		printk_reg0((struct pda12_reg0 *) &_reg);
		break;

	case 12:
		_reg = pda12dev.regs[2];
		PRINTK(KERN_INFO, "REG2: %08x\n", (unsigned int) _reg);
		printk_reg2((struct pda12_reg2 *) &_reg);
		break;

	case 15:
		_reg = pda12dev.regs[5];
		PRINTK(KERN_INFO, "REG5: %08x\n", (unsigned int) _reg);
		break;

	case 16:
		_reg = pda12dev.regs[6];
		PRINTK(KERN_INFO, "REG6: %08x\n", (unsigned int) _reg);
		printk_reg6((struct pda12_reg6 *) &_reg);
		break;

	case 20:
		_reg = pda12_reg_read(&pda12dev, 0);
		PRINTK(KERN_INFO, "REG0: %08x\n", (unsigned int) _reg);
		printk_reg0((struct pda12_reg0 *) &_reg);
		break;

	case 22:
		_reg = pda12_reg_read(&pda12dev, 2);
		PRINTK(KERN_INFO, "REG2: %08x\n", (unsigned int) _reg);
		printk_reg2((struct pda12_reg2 *) &_reg);
		break;

	case 25:
		_reg = pda12_reg_read(&pda12dev, 5);
		PRINTK(KERN_INFO, "REG5: %08x\n", (unsigned int) _reg);
		break;

	case 26:
		_reg = pda12_reg_read(&pda12dev, 6);
		PRINTK(KERN_INFO, "REG6: %08x\n", (unsigned int) _reg);
		printk_reg6((struct pda12_reg6 *) &_reg);
		break;

	case 31:
		PRINTK(KERN_INFO, "IOMEM1: ");
		for (idx = 0; idx < 32; idx += 4)
			printk(" %08x", ioread32(IOMEM_OFFSET(pda12dev.iomem1, idx)));
		printk("\n");
		PRINTK(KERN_INFO, "REGS: ");
		for (idx = 0; idx < 8; idx++)
			printk(" %08x", pda12dev.regs[idx]);
		printk("\n");
		break;

	case 33:
		PRINTK(KERN_INFO, "EPROM:");
		for (idx = 0; idx < 0x100; idx++)
			printk(" %02x", pda12_eprom_read(&pda12dev, idx));
		printk("\n");
		break;

	case 40:
		pda12_set_mode(&pda12dev, PDA12_MODE_OFF);
		break;

	case 41:
		pda12_set_mode(&pda12dev, PDA12_MODE_ACQ_PCI);
		break;

	case 42:
		pda12_set_mode(&pda12dev, PDA12_MODE_ACQ_1);
		break;

	case 43:
		pda12_set_mode(&pda12dev, PDA12_MODE_ACQ_2);
		break;

	case 44:
		pda12_set_mode(&pda12dev, PDA12_MODE_PCI);
		break;

	case 45:
		pda12_set_mode(&pda12dev, PDA12_MODE_AUX);
		break;

	case 46:
		pda12_set_mode(&pda12dev, PDA12_MODE_WRAM);
		break;

	case 47:
		pda12_set_mode(&pda12dev, PDA12_MODE_STBY);
		break;

	case 50:
		if (!pda12dev.buffer) {
			PRINTK(KERN_INFO, "Buffer not allocated, skipping\n");
			break;
		}

		pda12_copy_ram_block(&pda12dev, pda12dev.buffer, 0);

		printk("RAM BUFFER:\n");
		for (idx = 0; idx < 256; idx++) {
			printk(" %02x", pda12dev.buffer[idx]);
			if ((idx % 32) == 31)
				printk("\n");
		}
		printk("\n");
		break;

	case 60:
		pda12_software_trigger(&pda12dev);
		break;

	case 61:
		pda12_set_end_addr(&pda12dev, 8192);
		pda12_set_start_addr(&pda12dev, 0);

	case 90:
		pda12_PCIACQTEST_1(&pda12dev);
		break;

	case 91:
		pda12_PCIACQTEST_2(&pda12dev);
		break;

	case 92:
		pda12_SPEEDTEST_1(&pda12dev);
		break;

	case 99:
		pda12_SET_SETTINGS(&pda12dev);
		break;

	}
	return count;
}

static int pda12_chrdev_release(struct inode *inode, struct file *filp)
{
	PRINTK(KERN_INFO, "%s\n", __FUNCTION__);
	return 0;
}

static struct file_operations chrdev_fops = {
	.open = pda12_chrdev_open,
	.read = pda12_chrdev_read,
	.write = pda12_chrdev_write,
	.release = pda12_chrdev_release,
	.unlocked_ioctl = pda12_chrdev_unlocked_ioctl,
};

static irqreturn_t pda12_irq_handler(int irq, void *data)
{
	u32 intcsr;
        u8 status;

	intcsr = ioread32be(IOMEM_OFFSET(pda12dev.iomem0, AMCC_OP_REG_INTCSR));

        status = (intcsr >> 2) & 0xff;

	PRINTK(KERN_INFO, "Got interrupt - status: %02x\n", status);

        if (!(intcsr & AMCC_INTCSR_ANY))
            return IRQ_NONE;

	pda12_set_mode(&pda12dev, PDA12_MODE_STBY);

	if ((intcsr & AMCC_INTCSR_IMB)) {
		PRINTK(KERN_INFO, "AMCC_INTCSR_IMB\n");

		/* disable the incoming mailbox interrupt */
		iowrite8(0, IOMEM_OFFSET(pda12dev.iomem0, (AMCC_OP_REG_INTCSR + 1)));

		/* clear the incoming mailbox interrupt */
		iowrite32be(AMCC_INTCSR_EN_IMB, IOMEM_OFFSET(pda12dev.iomem0, AMCC_OP_REG_INTCSR));
	} else {

		PRINTK(KERN_INFO, "MEM FULL IRQ\n");

		/* Disable bus mastering */
		iowrite8(0x11, IOMEM_OFFSET(pda12dev.iomem0, (AMCC_OP_REG_MCSR + 1)));

		iowrite32be(AMCC_INTCSR_EN_WRITE_TC, IOMEM_OFFSET(pda12dev.iomem0, AMCC_OP_REG_INTCSR));
	}

	return IRQ_HANDLED;
}

static struct pci_device_id pda12_id_table[] = {
	{0x10e8, 0x80d5, PCI_ANY_ID, PCI_ANY_ID, PCI_CLASS_NOT_DEFINED, 0x0000, 0},
	{0,}
};

static int __devinit pda12_probe(struct pci_dev *pdev,
				 const struct pci_device_id *ent)
{
	void __iomem *iomem0;
	void __iomem *iomem1;
	void __iomem *iomem2;
	void *buffer;
	void *dma_buf;
	dma_addr_t dma_addr;
	int rc;

	PRINTK(KERN_INFO, "%s\n", __FUNCTION__);

	if ((rc = pci_enable_device(pdev))) {
		PRINTK(KERN_ERR, "Cannot enable PCI device, aborting.\n");
		goto err_out_free_dev;
	}

	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_IO)) {
		PRINTK(KERN_ERR, "Cannot find proper PCI device "
		       "base address, aborting.\n");
		rc = -ENODEV;
		goto err_out_disable_pdev;
	}

	if ((rc = pci_request_regions(pdev, DRV_NAME))) {
		PRINTK(KERN_ERR, "Cannot obtain PCI resources, aborting.\n");
		goto err_out_disable_pdev;
	}

	if ((rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32)))) {
		PRINTK(KERN_ERR, "No usable DMA configuration, aborting.\n");
		goto err_out_free_res;
	}

	if (!(iomem0 = pci_iomap(pdev, 0, 64))) {
		PRINTK(KERN_ERR, "Cannot map device registers (0), aborting.\n");
		rc = -ENOMEM;
		goto err_out_free_res;
	}
	printk(KERN_INFO "pda12: iomem0: %016lx\n", (unsigned long) iomem0);

	if (!(iomem1 = pci_iomap(pdev, 1, 32))) {
		PRINTK(KERN_ERR, "Cannot map device registers (1), aborting.\n");
		rc = -ENOMEM;
		goto err_out_iounmap0;
	}
	printk(KERN_INFO "pda12: iomem1: %016lx\n", (unsigned long) iomem1);

	if (!(iomem2 = pci_iomap(pdev, 2, 8192))) {
		PRINTK(KERN_ERR, "Cannot map device registers (1), aborting.\n");
		rc = -ENOMEM;
		goto err_out_iounmap1;
	}
	printk(KERN_INFO "pda12: iomem2: %016lx\n", (unsigned long) iomem2);

#if 1
	rc = pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &pda12dev.irq);
	if (rc < 0) {
		PRINTK(KERN_ERR, "Failed to get interrupt line\n");
		rc = -EINVAL;
		goto err_out_iounmap1;
	}

        pdev->irq = pda12dev.irq;
#endif

	rc = request_irq(pdev->irq, pda12_irq_handler, IRQF_SHARED, "pda12_irq_handler", &pda12dev);
	if (rc < 0) {
		PRINTK(KERN_ERR, "Failed to set-up interrupt handler for IRQ: %d\n", pdev->irq);
		rc = -EINVAL;
		goto err_out_iounmap1;
	}

	//enable_irq(pdev->irq);

#ifdef CONSISTENT
	dma_buf = pci_alloc_consistent(pdev, DMA_SIZE, &dma_addr);
	if (!dma_buf) {
		PRINTK(KERN_ERR, "Failed to allocate DMA buffer\n");
	}
#else
	dma_buf = kmalloc(DMA_SIZE, GFP_DMA);
	if (!dma_buf) {
		PRINTK(KERN_ERR, "Failed to allocate DMA buffer\n");
	}

	dma_addr = pci_map_single(pdev, dma_buf, DMA_SIZE, PCI_DMA_FROMDEVICE);
#endif

	PRINTK(KERN_INFO, "Allocated DMA buffer: Phys: %016llx, Virt: %016llx\n",
	       (unsigned long long) virt_to_phys(dma_buf),
               (unsigned long long) dma_buf);

	pci_set_master(pdev);

	buffer = kmalloc(65536, GFP_KERNEL);
	if (!buffer) {
		PRINTK(KERN_ERR, "Failed to allocate read buffer\n");
	}

	memset(&pda12dev, 0, sizeof(pda12dev));
	pda12dev.pci_dev = pdev;
	pda12dev.iomem0 = iomem0;
	pda12dev.iomem1 = iomem1;
	pda12dev.iomem2 = iomem2;
	pda12dev.dma_buf = dma_buf;
	pda12dev.dma_addr = dma_addr;
	pda12dev.dma_len = DMA_SIZE;
	pda12dev.buffer = buffer;
	pda12dev.irq = pdev->irq;

	pda12_init_registers(&pda12dev);

	if ((major = register_chrdev(0, DRV_NAME, &chrdev_fops)) < 0) {
		PRINTK(KERN_ERR, "Failed to register char dev\n");
		goto err_out_iounmap2;
	}
	PRINTK(KERN_INFO, "Got major number: %d\n", major);

	return 0;

err_out_iounmap2:
	pci_iounmap(pdev, iomem2);
err_out_iounmap1:
	pci_iounmap(pdev, iomem1);
err_out_iounmap0:
	pci_iounmap(pdev, iomem0);
err_out_free_res:
	pci_release_regions(pdev);
err_out_disable_pdev:
	pci_disable_device(pdev);
err_out_free_dev:
	pci_set_drvdata(pdev, NULL);
	return rc;
}

static void __devexit pda12_remove(struct pci_dev *pdev)
{
	void *ptr = pci_get_drvdata(pdev);

	PRINTK(KERN_INFO, "%s\n", __FUNCTION__);

	free_irq(pda12dev.irq, &pda12dev);

	unregister_chrdev(major, DRV_NAME);

	if (pda12dev.buffer)
		kfree(pda12dev.buffer);

#ifdef CONSISTENT
	if (pda12dev.dma_buf)
		pci_free_consistent(pdev, DMA_SIZE, pda12dev.dma_buf, pda12dev.dma_addr);
#else
	if (pda12dev.dma_buf) {
		pci_unmap_single(pdev, pda12dev.dma_addr, DMA_SIZE, PCI_DMA_FROMDEVICE);
		kfree(pda12dev.dma_buf);
	}
#endif

	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}


#ifdef CONFIG_PM
static int pda12_suspend(struct pci_dev *pdev, pm_message_t state)
{
	PRINTK(KERN_INFO, "%s\n", __FUNCTION__);

#if 0
	pci_save_state(pdev);
	pci_enable_wake(pdev, PCI_D3hot, 1);
	pci_enable_wake(pdev, PCI_D3cold, 1);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);
#endif

	return 0;

}

static int pda12_resume(struct pci_dev *pdev)
{
	PRINTK(KERN_INFO, "%s\n", __FUNCTION__);

#if 0
	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	pci_enable_wake(pdev, 0, 0);
#endif

	return 0;
}
#endif /* CONFIG_PM */

static void pda12_shutdown(struct pci_dev *pdev)
{
	PRINTK(KERN_INFO, "%s\n", __FUNCTION__);

	pci_enable_wake(pdev, PCI_D3hot, 1);
	pci_enable_wake(pdev, PCI_D3cold, 1);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);
}

static pci_ers_result_t pda12_io_error_detected(struct pci_dev *pdev, pci_channel_state_t state)
{
	PRINTK(KERN_INFO, "%s\n", __FUNCTION__);
	pci_disable_device(pdev);
	return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t pda12_io_slot_reset(struct pci_dev *pdev)
{
	PRINTK(KERN_INFO, "%s\n", __FUNCTION__);

	if (pci_enable_device(pdev)) {
		PRINTK(KERN_ERR, "pda12: Cannot re-enable PCI device after reset.\n");
		return PCI_ERS_RESULT_DISCONNECT;
	}
	pci_set_master(pdev);

	/* Only one device per card can do a reset */
	if (0 != PCI_FUNC(pdev->devfn))
		return PCI_ERS_RESULT_RECOVERED;

	return PCI_ERS_RESULT_RECOVERED;
}

static void pda12_io_resume(struct pci_dev *pdev)
{
	PRINTK(KERN_INFO, "%s\n", __FUNCTION__);
}

static struct pci_error_handlers pda12_err_handler = {
	.error_detected	= pda12_io_error_detected,
	.slot_reset	= pda12_io_slot_reset,
	.resume		= pda12_io_resume,
};

static struct pci_driver pda12_driver = {
	.name		= DRV_NAME,
	.id_table	= pda12_id_table,
	.probe		= pda12_probe,
	.remove		= __devexit_p(pda12_remove),
#ifdef CONFIG_PM
	.suspend	= pda12_suspend,
	.resume		= pda12_resume,
#endif
	.shutdown	= pda12_shutdown,
	.err_handler	= &pda12_err_handler,
};

static int __init pda12_init_module(void)
{
	PRINTK(KERN_INFO, "%s, %s\n", DRV_DESCRIPTION, DRV_VERSION);
	PRINTK(KERN_INFO, "%s\n", DRV_COPYRIGHT);
	return pci_register_driver(&pda12_driver);
}

static void __exit pda12_cleanup_module(void)
{
	pci_unregister_driver(&pda12_driver);
}

module_init(pda12_init_module);
module_exit(pda12_cleanup_module);
