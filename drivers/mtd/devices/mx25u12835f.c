/*
 * MTD SPI driver for MACRONIX M25Uxx (and similar) serial flash chips
 *
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/hwid.h>

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR		0x05	/* Read status register */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte */
#define	OPCODE_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define	OPCODE_PP		0x02	/* Page program (up to 256 bytes) */
#define	OPCODE_BE_4K		0x20	/* Erase 4KiB block */
#define	OPCODE_BE_32K		0x52	/* Erase 32KiB block */
#define	OPCODE_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define	OPCODE_SE		0xd8	/* Sector erase (usually 64KiB) */
#define	OPCODE_RDID		0x9f	/* Read JEDEC ID */

/* Used for SST flashes only. */
#define	OPCODE_BP		0x02	/* Byte program */
#define	OPCODE_WRDI		0x04	/* Write disable */
#define	OPCODE_AAI_WP		0xad	/* Auto address increment word program */

/* Used for Macronix flashes only. (4 Byte Address Command Set) */
#define	OPCODE_READ4B		0x13	/* Normal Read */
#define	OPCODE_FAST_READ4B	0x0c	/* Fast Read */
#define OPCODE_PP4B			0x12	/* Page Program */
#define OPCODE_BE4B			0xdc	/* Block Erase */
#define OPCODE_SE4B			0x21	/* Sector Erase */

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4	/* Block protect 0 */
#define	SR_BP1			8	/* Block protect 1 */
#define	SR_BP2			0x10	/* Block protect 2 */
#define	SR_SRWD			0x80	/* SR write protect */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)	/* M25P16 specs 40s max chip erase */
#define	MAX_CMD_SIZE		5

#ifdef CONFIG_M25PXX_USE_FAST_READ
#define OPCODE_READ 	OPCODE_FAST_READ
#define FAST_READ_DUMMY_BYTE 1
#else
#define OPCODE_READ 	OPCODE_NORM_READ
#define FAST_READ_DUMMY_BYTE 0
#endif


/*
 * Debugging macro and defines
 */
#define MTD_DEBUG_LEVEL0	(0)	// Quiet   //
#define MTD_DEBUG_LEVEL1	(1)	// Audible //
#define MTD_DEBUG_LEVEL2	(2)	// Loud    //
#define MTD_DEBUG_LEVEL3	(3)	// Noisy   //

#define CONFIG_MTD_DEBUG
#define CONFIG_MTD_DEBUG_VERBOSE MTD_DEBUG_LEVEL2

#ifdef CONFIG_MTD_DEBUG
#define DEBUG(n, args...)				\
	do {						\
		if (n <= CONFIG_MTD_DEBUG_VERBOSE)	\
			printk(KERN_INFO args);		\
	} while(0)
#else // CONFIG_MTD_DEBUG //
#define DEBUG(n, args...) do { } while(0)
#endif // CONFIG_MTD_DEBUG //


/****************************************************************************/

struct m25p {
	struct spi_device	*spi;
	struct mutex		lock;
	struct mtd_info		mtd;
	unsigned		partitioned:1;
	u16			page_size;
	u16			addr_width;
	u8			erase_opcode;
	u8			*command;
};


struct mx25u12835f_dev_info {
	struct device	*dev;  
	int				fpga_creset;    
};


static inline struct m25p *mtd_to_m25p(struct mtd_info *mtd)
{
	return container_of(mtd, struct m25p, mtd);
}



struct mtd_info *__mtd_next_device(int i);
int add_mtd_device(struct mtd_info *mtd);
int del_mtd_device(struct mtd_info *mtd);
int add_mtd_partitions(struct mtd_info *, const struct mtd_partition *, int);
int del_mtd_partitions(struct mtd_info *);
int parse_mtd_partitions(struct mtd_info *master, const char * const *types,
			 struct mtd_partition **pparts,
			 struct mtd_part_parser_data *data);
			 
struct mx25u12835f_dev_info *g_bdi;
			 
			 
			 
#ifdef CONFIG_MTD_PARTITIONS
static inline int mtd_has_partitions(void) { return 1; }
#else
static inline int mtd_has_partitions(void) { return 0; }
#endif

#ifdef CONFIG_MTD_CMDLINE_PARTS
static inline int mtd_has_cmdlinepart(void) { return 1; }
#else
static inline int mtd_has_cmdlinepart(void) { return 0; }
#endif
			 
			 
			 
			 
/****************************************************************************/

/*
 * Internal helper functions
 */

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */

static int fpga_disable(struct mx25u12835f_dev_info *bdi)
{
	if (!gpio_request(bdi->fpga_creset, "fpga_creset")) {
		gpio_direction_output(bdi->fpga_creset,0);
		gpio_export(bdi->fpga_creset, 1);
		gpio_free(bdi->fpga_creset);
		dev_info(bdi->dev, "[%s] FPGA is in CRESET LOW state \n", __func__);
	}
	else
		goto gpio_err;

	return 1;

gpio_err:
	dev_err(bdi->dev, "[%s] GPIO error \n", __func__);
	if (bdi->fpga_creset)
		gpio_free(bdi->fpga_creset);
	return -1;
}

static int fpga_enable(struct mx25u12835f_dev_info *bdi)
{
	if (!gpio_request(bdi->fpga_creset, "fpga_creset")) {
		gpio_direction_output(bdi->fpga_creset,1);
		gpio_export(bdi->fpga_creset, 1);
		gpio_free(bdi->fpga_creset);
		dev_info(bdi->dev, "[%s] FPGA is in CRESET HIGH state \n", __func__);
	}
	else
		goto gpio_err;

	return 1;

gpio_err:
	dev_err(bdi->dev, "[%s] GPIO error \n", __func__);
	if (bdi->fpga_creset)
		gpio_free(bdi->fpga_creset);
	return -1;
}

static int mx25u_setup_dt(struct mx25u12835f_dev_info *bdi)
{
	bdi->fpga_creset = of_get_named_gpio(bdi->dev->of_node, "fpga,cresetb-gpio", 0);
	if (!gpio_is_valid(bdi->fpga_creset))
		return -1;

	dev_err(bdi->dev, "[%s] Get GPIO%d (fpga_creset)\n", __func__, bdi->fpga_creset);
	return 0;
}

static int read_sr(struct m25p *flash)
{
	ssize_t retval;
	u8 code = OPCODE_RDSR;
	u8 val;

	retval = spi_write_then_read(flash->spi, &code, 1, &val, 1);

	if (retval < 0) {
		dev_err(&flash->spi->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
/*
static int write_sr(struct m25p *flash, u8 val)
{
	flash->command[0] = OPCODE_WRSR;
	flash->command[1] = val;

	return spi_write(flash->spi, flash->command, 2);
}
*/

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct m25p *flash)
{
	u8	code = OPCODE_WREN;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/*
 * Send write disble instruction to the chip.
 */
static inline int write_disable(struct m25p *flash)
{
	u8	code = OPCODE_WRDI;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct m25p *flash)
{
	unsigned long deadline;
	int sr;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		if ((sr = read_sr(flash)) < 0)
			break;
		else if (!(sr & SR_WIP))
			return 0;

		cond_resched();

	} while (!time_after_eq(jiffies, deadline));

	return 1;
}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(struct m25p *flash)
{
	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %lldKiB\n",
	      dev_name(&flash->spi->dev), __func__,
	      (long long)(flash->mtd.size >> 10));

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = OPCODE_CHIP_ERASE;

	spi_write(flash->spi, flash->command, 1);

	return 0;
}

static void m25p_addr2cmd(struct m25p *flash, unsigned int addr, u8 *cmd)
{
	/* opcode is in cmd[0] */
	cmd[1] = addr >> (flash->addr_width * 8 -  8);
	cmd[2] = addr >> (flash->addr_width * 8 - 16);
	cmd[3] = addr >> (flash->addr_width * 8 - 24);
	cmd[4] = addr >> (flash->addr_width * 8 - 32);
}

static int m25p_cmdsz(struct m25p *flash)
{
	return 1 + flash->addr_width;
}

/*
 * Erase one sector of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_sector(struct m25p *flash, u32 offset)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s: %s %dKiB at 0x%08x\n",
			dev_name(&flash->spi->dev), __func__,
			flash->mtd.erasesize / 1024, offset);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = flash->erase_opcode;
	m25p_addr2cmd(flash, offset, flash->command);

	spi_write(flash->spi, flash->command, m25p_cmdsz(flash));

	return 0;
}

/****************************************************************************/

/*
 * MTD implementation
 */

/*
 * Erase an address range on the flash chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int m25p80_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct m25p *flash = mtd_to_m25p(mtd);
	u32 addr,len;
	uint32_t rem;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%llx, len %lld\n",
	      dev_name(&flash->spi->dev), __func__, "at",
	      (long long)instr->addr, (long long)instr->len);

	/* sanity checks */
	if (instr->addr + instr->len > flash->mtd.size)
		return -EINVAL;
	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem)
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;

	mutex_lock(&flash->lock);

	/* whole-chip erase? */
	if (len == flash->mtd.size) {
		if (erase_chip(flash)) {
			instr->state = MTD_ERASE_FAILED;
			mutex_unlock(&flash->lock);
			return -EIO;
		}

	/* REVISIT in some cases we could speed up erasing large regions
	 * by using OPCODE_SE instead of OPCODE_BE_4K.  We may have set up
	 * to use "small sector erase", but that's not always optimal.
	 */

	/* "sector"-at-a-time erase */
	} else {
		while (len) {
			if (erase_sector(flash, addr)) {
				instr->state = MTD_ERASE_FAILED;
				mutex_unlock(&flash->lock);
				return -EIO;
			}

			addr += mtd->erasesize;
			len -= mtd->erasesize;
		}
	}

	mutex_unlock(&flash->lock);

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}

/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int m25p80_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	struct m25p *flash = mtd_to_m25p(mtd);
	struct spi_transfer t[2];
	struct spi_message m;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
			dev_name(&flash->spi->dev), __func__, "from",
			(u32)from, len);

	/* sanity checks */
	if (!len)
		return 0;

	if (from + len > flash->mtd.size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	/* NOTE:
	 * OPCODE_FAST_READ (if available) is faster.
	 * Should add 1 byte DUMMY_BYTE.
	 */
	t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(flash) + FAST_READ_DUMMY_BYTE;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);

	/* Byte count starts at zero. */
	*retlen = 0;

	mutex_lock(&flash->lock);

	/* Wait till previous write/erase is done. */
	if (wait_till_ready(flash)) {
		/* REVISIT status return?? */
		mutex_unlock(&flash->lock);
		return 1;
	}

	/* FIXME switch to OPCODE_FAST_READ.  It's required for higher
	 * clocks; and at this writing, every chip this driver handles
	 * supports that opcode.
	 */

	/* Set up the write data buffer. */
	if (flash->addr_width == 4)
#ifdef CONFIG_M25PXX_USE_FAST_READ
		flash->command[0] = OPCODE_FAST_READ4B;
#else
		flash->command[0] = OPCODE_READ4B;
#endif
	else
		flash->command[0] = OPCODE_READ;
	m25p_addr2cmd(flash, from, flash->command);

	spi_sync(flash->spi, &m);

	*retlen = m.actual_length - m25p_cmdsz(flash) - FAST_READ_DUMMY_BYTE;

	mutex_unlock(&flash->lock);

	return 0;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int m25p80_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct m25p *flash = mtd_to_m25p(mtd);
	u32 page_offset, page_size;
	struct spi_transfer t[2];
	struct spi_message m;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
			dev_name(&flash->spi->dev), __func__, "to",
			(u32)to, len);

	*retlen = 0;

	/* sanity checks */
	if (!len)
		return(0);

	if (to + len > flash->mtd.size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(flash);
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&flash->lock);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash)) {
		mutex_unlock(&flash->lock);
		return 1;
	}

	write_enable(flash);

	/* Set up the opcode in the write buffer. */
	if (flash->addr_width == 4)
		flash->command[0] = OPCODE_PP4B;
	else
		flash->command[0] = OPCODE_PP;
	m25p_addr2cmd(flash, to, flash->command);

	page_offset = to & (flash->page_size - 1);

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= flash->page_size) {
		t[1].len = len;

		spi_sync(flash->spi, &m);

		*retlen = m.actual_length - m25p_cmdsz(flash);
	} else {
		u32 i;

		/* the size of data remaining on the first page */
		page_size = flash->page_size - page_offset;

		t[1].len = page_size;
		spi_sync(flash->spi, &m);

		*retlen = m.actual_length - m25p_cmdsz(flash);

		/* write everything in flash->page_size chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > flash->page_size)
				page_size = flash->page_size;

			/* write the next page to flash */
			m25p_addr2cmd(flash, to + i, flash->command);

			t[1].tx_buf = buf + i;
			t[1].len = page_size;

			wait_till_ready(flash);

			write_enable(flash);

			spi_sync(flash->spi, &m);

			*retlen += m.actual_length - m25p_cmdsz(flash);
		}
	}

	mutex_unlock(&flash->lock);
	
	return 0;
}

static ssize_t mx25u12835f_erase_all_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct m25p *flash = dev_get_drvdata(dev);
	struct erase_info *instr;

	mutex_lock(&flash->lock);

	if (erase_chip(flash)) {
		instr->state = MTD_ERASE_FAILED;
		printk(KERN_ERR "[%s] erase_chip error! \n", __func__);
		mutex_unlock(&flash->lock);
		return -EIO;
	}
	mutex_unlock(&flash->lock);

	return count;
}

static DEVICE_ATTR(erase_all,       0664, NULL, mx25u12835f_erase_all_store);

static ssize_t mx25u12835f_usecode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	struct m25p *flash = dev_get_drvdata(dev);
	struct spi_transfer t[2];
	struct spi_message m;

	size_t len = 24;
	loff_t from = 220;
	size_t retlen = 0;
	uint8_t array[24] = {0};
	u_char *spi_buf = (u_char *)array;
	//int i;

	printk(KERN_ERR "[%s] from 0x%llx\n", __func__, from);

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	/* NOTE:
	 * OPCODE_FAST_READ (if available) is faster.
	 * Should add 1 byte DUMMY_BYTE.
	 */
	t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(flash) + FAST_READ_DUMMY_BYTE;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = spi_buf;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&flash->lock);

	/* Wait till previous write/erase is done. */
	if (wait_till_ready(flash)) {
		/* REVISIT status return?? */
		mutex_unlock(&flash->lock);
		return 1;
	}

	/* FIXME switch to OPCODE_FAST_READ.  It's required for higher
	 * clocks; and at this writing, every chip this driver handles
	 * supports that opcode.
	 */

	/* Set up the write data buffer. */
	if (flash->addr_width == 4)
#ifdef CONFIG_M25PXX_USE_FAST_READ
		flash->command[0] = OPCODE_FAST_READ4B;
#else
		flash->command[0] = OPCODE_READ4B;
#endif
	else
		flash->command[0] = OPCODE_READ;

	m25p_addr2cmd(flash, from, flash->command);

	spi_sync(flash->spi, &m);

	retlen = m.actual_length - m25p_cmdsz(flash) - FAST_READ_DUMMY_BYTE;

	printk(KERN_ERR "[%s] retlen 0x%zx \n", __func__, retlen);
	mutex_unlock(&flash->lock);

	/*for ( i = 0; i < len; i++ ) {
		printk(KERN_ERR "[%s] array[%d] = 0x%x \n", __func__, i, array[i]);
	}*/

	printk("[%s] %s\n", __func__, spi_buf);
	return sprintf(buf, "%s\n", spi_buf);
}

static DEVICE_ATTR(usecode,       0664, mx25u12835f_usecode_show, NULL);

static struct attribute *mx25u12835f_attributes[] = {
    &dev_attr_erase_all.attr,
    &dev_attr_usecode.attr,
    NULL,
};

static struct attribute_group mx25u12835f_attr_group = {
    .attrs = mx25u12835f_attributes,
};


/****************************************************************************/

/*
 * SPI device driver setup and teardown
 */

struct flash_info {
	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32		jedec_id;
	u16             ext_id;

	/* The size listed here is what works with OPCODE_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned	sector_size;
	u16		n_sectors;

	u16		page_size;
	u16		addr_width;

	u16		flags;
#define	SECT_4K		0x01		/* OPCODE_BE_4K works uniformly */
#define	M25P_NO_ERASE	0x02		/* No erase command needed */
};

#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
	((kernel_ulong_t)&(struct flash_info) {				\
		.jedec_id = (_jedec_id),				\
		.ext_id = (_ext_id),					\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.addr_width = 3,					\
		.flags = (_flags),					\
	})

#define CAT25_INFO(_sector_size, _n_sectors, _page_size, _addr_width)	\
	((kernel_ulong_t)&(struct flash_info) {				\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = (_page_size),				\
		.addr_width = (_addr_width),				\
		.flags = M25P_NO_ERASE,					\
	})

/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */
static const struct spi_device_id m25p_ids[] = {
	/* Macronix -- mx25/mx66 "blocks" are 64K, "sectors" are 4K */
	{ "mx25u2033e",  INFO(0xc22532, 0, 64 * 1024,    4, 0) },
	{ "mx25u4033e",  INFO(0xc22533, 0, 64 * 1024,    8, 0) },
	{ "mx25u1635fz", INFO(0xc22535, 0, 64 * 1024,   32, 0) },
	{ "mx25u12835f", INFO(0xc22538, 0, 64 * 1024,  256, 0) },

	/* ST Microelectronics -- newer production may have feature updates */
	{ "m25p05",  INFO(0x202010,  0,  32 * 1024,   2, 0) },
	{ "m25p10",  INFO(0x202011,  0,  32 * 1024,   4, 0) },
	{ "m25p20",  INFO(0x202012,  0,  64 * 1024,   4, 0) },
	{ "m25p40",  INFO(0x202013,  0,  64 * 1024,   8, 0) },
	{ "m25p80",  INFO(0x202014,  0,  64 * 1024,  16, 0) },
	{ "m25p16",  INFO(0x202015,  0,  64 * 1024,  32, 0) },
	{ "m25p32",  INFO(0x202016,  0,  64 * 1024,  64, 0) },
	{ "m25p64",  INFO(0x202017,  0,  64 * 1024, 128, 0) },
	{ "m25p128", INFO(0x202018,  0, 256 * 1024,  64, 0) },
	{ },
};
MODULE_DEVICE_TABLE(spi, m25p_ids);

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "mx,mx25u12835f", },
	{},
};

MODULE_DEVICE_TABLE(of, spidev_dt_ids);

static const struct spi_device_id * jedec_probe(struct spi_device *spi)
{
	int			tmp;
	u8			code = OPCODE_RDID;
	u8			id[5];
	u32			jedec;
	u16                     ext_jedec;
	struct flash_info	*info;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 */
	tmp = spi_write_then_read(spi, &code, 1, id, 5);
	if (tmp < 0) {
		DEBUG(MTD_DEBUG_LEVEL0, "%s: error %d reading JEDEC ID\n",
			dev_name(&spi->dev), tmp);
		return ERR_PTR(tmp);
	}
	jedec = id[0];
	jedec = jedec << 8;
	jedec |= id[1];
	jedec = jedec << 8;
	jedec |= id[2];

	ext_jedec = id[3] << 8 | id[4];

	for (tmp = 0; tmp < ARRAY_SIZE(m25p_ids) - 1; tmp++) {
		info = (void *)m25p_ids[tmp].driver_data;
		if (info->jedec_id == jedec) {
			if (info->ext_id != 0 && info->ext_id != ext_jedec)
				continue;
			return &m25p_ids[tmp];
		}
	}
	return ERR_PTR(-ENODEV);
}


/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int m25p_probe(struct spi_device *spi)
{
	const struct spi_device_id	*id = spi_get_device_id(spi);
	struct flash_platform_data	*data;
	struct m25p			*flash;
	struct flash_info		*info;
	unsigned			i;
	int rc = 0;
	int ret2, hwid = 0;
	struct mx25u12835f_dev_info *bdi;

	
	dev_info(&spi->dev, "mx25u12835fz probe +++\n");
	bdi = devm_kzalloc(&spi->dev, sizeof(*bdi), GFP_KERNEL);
	if (!bdi) {
		dev_err(&spi->dev, "[%s] Can't alloc bdi struct\n",  __func__);
		return -ENOMEM;
	}
	bdi->dev = &spi->dev;
	g_bdi = bdi;
	
	if (bdi->dev->of_node) {
		if (mx25u_setup_dt(bdi))
			dev_err(&spi->dev, "[%s]Can't get GPIO info\n", __func__);
	} else {
		dev_err(&spi->dev, "[%s]Can't get device tree\n", __func__);
	}

	hwid = read_hw_id();
	if (hwid <= 1)    fpga_disable(bdi);
	
	dev_info(&spi->dev, "max_speed = %d, chip_select = %d, mode = %d", spi->max_speed_hz, spi->chip_select, spi->mode);
	
	/* Platform data helps sort out which chip type we have, as
	 * well as how this board partitions it.  If we don't have
	 * a chip ID, try the JEDEC id commands; they'll work for most
	 * newer chips, even if we don't recognize the particular chip.
	 */
	data = spi->dev.platform_data;
	if (data && data->type) {
		const struct spi_device_id *plat_id;

		for (i = 0; i < ARRAY_SIZE(m25p_ids) - 1; i++) {
			plat_id = &m25p_ids[i];
			if (strcmp(data->type, plat_id->name))
				continue;
			break;
		}

		if (i < ARRAY_SIZE(m25p_ids) - 1)
			id = plat_id;
		else
			dev_warn(&spi->dev, "unrecognized id %s\n", data->type);
	}

	info = (void *)id->driver_data;

	if (info->jedec_id) {
		const struct spi_device_id *jid;
		jid = jedec_probe(spi);
		
		if (IS_ERR(jid)) {
			return PTR_ERR(jid);
		} else if (jid != id) {
			/*
			 * JEDEC knows better, so overwrite platform ID. We
			 * can't trust partitions any longer, but we'll let
			 * mtd apply them anyway, since some partitions may be
			 * marked read-only, and we don't want to lose that
			 * information, even if it's not 100% accurate.
			 */
			dev_warn(&spi->dev, "found %s, expected %s\n",
				 jid->name, id->name);
			id = jid;
			info = (void *)jid->driver_data;
		}
	}

	flash = kzalloc(sizeof *flash, GFP_KERNEL);
	if (!flash)
		return -ENOMEM;
	flash->command = kmalloc(MAX_CMD_SIZE + FAST_READ_DUMMY_BYTE, GFP_KERNEL);
	if (!flash->command) {
		kfree(flash);
		return -ENOMEM;
	}

	flash->spi = spi;
	mutex_init(&flash->lock);
	dev_set_drvdata(&spi->dev, flash);

	if (data && data->name)
		flash->mtd.name = data->name;
	else
		flash->mtd.name = dev_name(&spi->dev);

	flash->mtd.type = MTD_NORFLASH;
	flash->mtd.writesize = 1;
	flash->mtd.flags = MTD_CAP_NORFLASH;
	flash->mtd.size = info->sector_size * info->n_sectors;
	flash->mtd._erase = m25p80_erase;
	flash->mtd._read = m25p80_read;
	flash->mtd._write = m25p80_write;

	/* enable 4 byte address command set if the device exceeds 128Mbit */
	if (flash->mtd.size > 0x1000000)
		flash->addr_width = 4;
	else
		flash->addr_width = 3;

	/* prefer "small sector" erase if possible */
	if (info->flags & SECT_4K) {
		if (flash->addr_width == 4)
			flash->erase_opcode = OPCODE_SE4B;
		else
			flash->erase_opcode = OPCODE_BE_4K;
		flash->mtd.erasesize = 4096;
	} else {
		if (flash->addr_width == 4)
			flash->erase_opcode = OPCODE_BE4B;
		else
			flash->erase_opcode = OPCODE_SE;
		flash->mtd.erasesize = info->sector_size;
	}

	if (info->flags & M25P_NO_ERASE)
		flash->mtd.flags |= MTD_NO_ERASE;

	flash->mtd.dev.parent = &spi->dev;
	flash->page_size = info->page_size;

	dev_info(&spi->dev, "%s (%lld Kbytes)\n", id->name,
			(long long)flash->mtd.size >> 10);

	dev_info(&spi->dev, "mtd .name = %s, .size = 0x%llx (%lldMiB) "
			".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		flash->mtd.name,
		(long long)flash->mtd.size, (long long)(flash->mtd.size >> 20),
		flash->mtd.erasesize, flash->mtd.erasesize / 1024,
		flash->mtd.numeraseregions);

	if (flash->mtd.numeraseregions)
		for (i = 0; i < flash->mtd.numeraseregions; i++)
			DEBUG(MTD_DEBUG_LEVEL2,
				"mtd.eraseregions[%d] = { .offset = 0x%llx, "
				".erasesize = 0x%.8x (%uKiB), "
				".numblocks = %d }\n",
				i, (long long)flash->mtd.eraseregions[i].offset,
				flash->mtd.eraseregions[i].erasesize,
				flash->mtd.eraseregions[i].erasesize / 1024,
				flash->mtd.eraseregions[i].numblocks);


	/* partitions should match sector boundaries; and it may be good to
	 * use readonly partitions for writeprotected sectors (BP2..BP0).
	 */
	if (mtd_has_partitions()) {
		struct mtd_partition	*parts = NULL;
		int			nr_parts = 0;

		if (mtd_has_cmdlinepart()) {
			static const char *part_probes[]
					= { "cmdlinepart", NULL, };

			nr_parts = parse_mtd_partitions(&flash->mtd,
					part_probes, &parts, 0);
		}

		if (nr_parts <= 0 && data && data->parts) {
			parts = data->parts;
			nr_parts = data->nr_parts;
		}

		if (nr_parts > 0) {
			for (i = 0; i < nr_parts; i++) {
				DEBUG(MTD_DEBUG_LEVEL2, "partitions[%d] = "
					"{.name = %s, .offset = 0x%llx, "
						".size = 0x%llx (%lldKiB) }\n",
					i, parts[i].name,
					(long long)parts[i].offset,
					(long long)parts[i].size,
					(long long)(parts[i].size >> 10));
			}
			flash->partitioned = 1;
			return add_mtd_partitions(&flash->mtd, parts, nr_parts);
		}
	} else if (data && data->nr_parts)
		dev_warn(&spi->dev, "ignoring %d default partitions on %s\n",
				data->nr_parts, data->name);

	// Initialize sysfs
	if((rc = sysfs_create_group(&spi->dev.kobj, &mx25u12835f_attr_group)) != 0){
		dev_err(&spi->dev, "\t Initialized sysfs enter faild %d\n", rc);
	}

	if (add_mtd_device(&flash->mtd)) {
		ret2 = -ENODEV;
		dev_info(&spi->dev, "[%s] add_mtd_device ...failed\n", __func__);
	}
	else {
		ret2 = 0;
		dev_info(&spi->dev, "[%s] add_mtd_device ...successful\n", __func__);
	}

	if (hwid <= 1)  fpga_enable(bdi);
	dev_info(&spi->dev, "mx25u12835fz probe ---\n");

	return ret2;
}


static int m25p_remove(struct spi_device *spi)
{
	struct m25p	*flash = dev_get_drvdata(&spi->dev);
	int		status;

	/* Clean up MTD stuff. */
	if (mtd_has_partitions() && flash->partitioned)
		status = del_mtd_partitions(&flash->mtd);
	else
		status = del_mtd_device(&flash->mtd);
	if (status == 0) {
		kfree(flash->command);
		kfree(flash);
	}
	return 0;
}


static struct spi_driver m25p80_driver = {
	.driver = {
		.name	= "mx25u12835f",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.id_table	= m25p_ids,
	.probe	= m25p_probe,
	.remove	= m25p_remove,

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};


static int m25p80_init(void)
{
	return spi_register_driver(&m25p80_driver);
}


static void m25p80_exit(void)
{
	spi_unregister_driver(&m25p80_driver);
}

module_init(m25p80_init);
//late_initcall(m25p80_init);
module_exit(m25p80_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Lavender");
MODULE_DESCRIPTION("MTD SPI driver for MACRONIX MX25xx flash chips");
