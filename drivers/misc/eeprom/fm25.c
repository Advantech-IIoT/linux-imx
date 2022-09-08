// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * fm25.c -- support SPI FRAMs, such as Cypress FM25 models
 *
 * Copyright (C) 2014 Jiri Prchal
 * Copyright (C) 2021 Chang.Qing
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>

#include <linux/nvmem-provider.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/of.h>

struct fm25_data {
	struct spi_device	*spi;
	struct mutex		lock;
	struct spi_eeprom	chip;
	unsigned		addrlen;
	int			has_sernum;
	struct nvmem_config	nvmem_config;
	struct nvmem_device	*nvmem;
};

#define	FM25_WREN	0x06		/* latch the write enable */
#define	FM25_WRDI	0x04		/* reset the write enable */
#define	FM25_RDSR	0x05		/* read status register */
#define	FM25_WRSR	0x01		/* write status register */
#define	FM25_READ	0x03		/* read byte(s) */
#define	FM25_WRITE	0x02		/* write byte(s)/sector */
#define	FM25_SLEEP	0xb9		/* enter sleep mode */
#define	FM25_RDID	0x9f		/* read device ID */
#define	FM25_RDSN	0xc3		/* read S/N */

#define	FM25_SR_WEN	0x02		/* write enable (latched) */
#define	FM25_SR_BP0	0x04		/* BP for software writeprotect */
#define	FM25_SR_BP1	0x08
#define	FM25_SR_WPEN	0x80		/* writeprotect enable */

#define	FM25_ID_LEN	9		/* ID lenght */
#define	FM25_SN_LEN	8		/* serial number lenght */

#define	FM25_MAXADDRLEN	3		/* 24 bit addresses */

#define	io_limit	PAGE_SIZE	/* bytes */

static int fm25_id_read(struct fm25_data *fm25, char *buf)
{
	u8			command = FM25_RDID;
	int			status;
	struct spi_transfer	t[2];
	struct spi_message	m;

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	t[0].tx_buf = &command;
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = FM25_ID_LEN;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&fm25->lock);

	status = spi_sync(fm25->spi, &m);
	dev_dbg(&fm25->spi->dev,
		"read %zu bytes of ID --> %zd\n",
	 FM25_ID_LEN, status);

	mutex_unlock(&fm25->lock);
	return status ? status : FM25_ID_LEN;
}

static int fm25_data_read(void *priv, unsigned int offset,
			void *val, size_t count)
{
	struct fm25_data *fm25 = priv;
	char *buf = val;
	u8			command[FM25_MAXADDRLEN + 1];
	u8			*cp;
	ssize_t			status;
	struct spi_transfer	t[2];
	struct spi_message	m;
	u8			instr;

	if (unlikely(offset >= fm25->chip.byte_len))
		return -EINVAL;
	if ((offset + count) > fm25->chip.byte_len)
		count = fm25->chip.byte_len - offset;
	if (unlikely(!count))
		return -EINVAL;

	cp = command;

	instr = FM25_READ;
	*cp++ = instr;

	/* 8/16/24-bit address is written MSB first */
	switch (fm25->addrlen) {
	default:	/* case 3 */
		*cp++ = offset >> 16;
	case 2:
		*cp++ = offset >> 8;
	case 1:
	case 0:	/* can't happen: for better codegen */
		*cp++ = offset >> 0;
	}

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	t[0].tx_buf = command;
	t[0].len = fm25->addrlen + 1;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&fm25->lock);

	/* Read it all at once.
	 *
	 * REVISIT that's potentially a problem with large chips, if
	 * other devices on the bus need to be accessed regularly or
	 * this chip is clocked very slowly
	 */
	status = spi_sync(fm25->spi, &m);
	dev_dbg(&fm25->spi->dev,
		"read %zu bytes at %d --> %zd\n",
		count, offset, status);

	mutex_unlock(&fm25->lock);
	return status;
}

static int fm25_data_write(void *priv, unsigned int off, void *val, size_t count)
{
	struct fm25_data 	*fm25 = priv;
	char 				*buf = val;
	int 				status = 0;
	unsigned			buf_size;
	u8					*bounce;

	if (unlikely(off >= fm25->chip.byte_len))
		return -EFBIG;
	if ((off + count) > fm25->chip.byte_len)
		count = fm25->chip.byte_len - off;
	if (unlikely(!count))
		return -EINVAL;

	/* Temp buffer starts with command and address */
	buf_size = io_limit;
	bounce = kmalloc(buf_size + fm25->addrlen + 1, GFP_KERNEL);
	if (!bounce)
		return -ENOMEM;

	/* For write, rollover is within the page ... so we write at
	 * most one page, then manually roll over to the next page.
	 */
	mutex_lock(&fm25->lock);
	do {
		unsigned	segment;
		unsigned	offset = (unsigned) off;
		u8		*cp = bounce;
		u8		instr;

		*cp = FM25_WREN;
		status = spi_write(fm25->spi, cp, 1);
		if (status < 0) {
			dev_dbg(&fm25->spi->dev, "WREN --> %d\n", status);
			break;
		}

		instr = FM25_WRITE;
		*cp++ = instr;

		/* 8/16/24-bit address is written MSB first */
		switch (fm25->addrlen) {
		default:	/* case 3 */
			*cp++ = offset >> 16;
		case 2:
			*cp++ = offset >> 8;
		case 1:
		case 0:	/* can't happen: for better codegen */
			*cp++ = offset >> 0;
		}

		/* Write as much of a page as we can */
		segment = buf_size - (offset % buf_size);
		if (segment > count)
			segment = count;
		memcpy(cp, buf, segment);
		status = spi_write(fm25->spi, bounce,
				segment + fm25->addrlen + 1);
		dev_dbg(&fm25->spi->dev,
				"write %u bytes at %u --> %d\n",
				segment, offset, status);
		if (status < 0)
			break;

		/* REVISIT this should detect (or prevent) failed writes
		 * to readonly sections of the EEPROM...
		 */

		off += segment;
		buf += segment;
		count -= segment;

	} while (count > 0);

	mutex_unlock(&fm25->lock);

	kfree(bounce);
	return status;
}

static int fm25_fw_to_chip(struct device *dev,
			   struct spi_eeprom *chip)
{
	memset(chip, 0, sizeof(*chip));
	strncpy(chip->name, "fm25", sizeof(chip->name));

	if (device_property_present(dev, "read-only"))
		chip->flags |= EE_READONLY;
	return 0;
}

static int fm25_probe(struct spi_device *spi)
{
	struct fm25_data	*fm25 = NULL;
	struct spi_eeprom	*chip;
	int					err;
	char				id[FM25_ID_LEN];

	fm25 = devm_kzalloc(&spi->dev, sizeof(*fm25), GFP_KERNEL);
	if (!fm25)
		return -ENOMEM;

	mutex_init(&fm25->lock);
	chip = &fm25->chip;
	fm25->spi = spi;
	spi_set_drvdata(spi, fm25);

	/* Chip description */
	if (!spi->dev.platform_data) {
		err = fm25_fw_to_chip(&spi->dev, chip);
		if (err)
			goto out_err;
	} else
		chip = (struct spi_eeprom *)spi->dev.platform_data;

	/* Get ID of chip */
	fm25_id_read(fm25, id);
	if (id[6] != 0xc2) {
		/*
		* Maybe this is a bug for other platform. since
		* fsl imx8 linux fucking kernel loading order
		* for spi subsystem, we must set the defer mechism.
		*/
		err = PTR_ERR(-EPROBE_DEFER);
		goto out_err;
	}

	/* set size found in ID */
	switch (id[7]) {
	case 0x21:
		chip->byte_len = 16 * 1024;
		break;
	case 0x22:
		chip->byte_len = 32 * 1024;
		break;
	case 0x23:
		chip->byte_len = 64 * 1024;
		break;
	case 0x24:
		chip->byte_len = 1024 * 1024;
		break;
	case 0x25:
		chip->byte_len = 256 * 1024;
		break;
	default:
		dev_err(&spi->dev, "Error: unsupported size (id %02x)\n", id[7]);
		err = -ENODEV;
		goto out_err;
		break;
	}

	if (chip->byte_len > 64 * 1024) {
		fm25->addrlen = 3;
		chip->flags |= EE_ADDR3;
	} else {
		fm25->addrlen = 2;
		chip->flags |= EE_ADDR2;
	}

	if (id[8])
		fm25->has_sernum = 1;
	else
		fm25->has_sernum = 0;

	chip->page_size = PAGE_SIZE;

	fm25->nvmem_config.name = chip->name;
	fm25->nvmem_config.dev = &spi->dev;
	fm25->nvmem_config.read_only = chip->flags & EE_READONLY;
	fm25->nvmem_config.root_only = true;
	fm25->nvmem_config.owner = THIS_MODULE;
	fm25->nvmem_config.compat = true;
	fm25->nvmem_config.base_dev = &spi->dev;
	fm25->nvmem_config.reg_read = fm25_data_read;
	fm25->nvmem_config.reg_write = fm25_data_write;
	fm25->nvmem_config.priv = fm25;
	fm25->nvmem_config.stride = 4;
	fm25->nvmem_config.word_size = 1;
	fm25->nvmem_config.size = chip->byte_len;

	fm25->nvmem = devm_nvmem_register(&spi->dev, &fm25->nvmem_config);
	if (IS_ERR(fm25->nvmem)){
		err = PTR_ERR(fm25->nvmem);
		goto out_err;
	}

	dev_info(&spi->dev, "%d %s %s eeprom%s, pagesize %u\n",
		(chip->byte_len < 1024) ? chip->byte_len : (chip->byte_len / 1024),
		(chip->byte_len < 1024) ? "Byte" : "KByte",
		chip->name,
		(chip->flags & EE_READONLY) ? " (readonly)" : "",
		chip->page_size);

	return 0;

out_err:
	return err;
}

/*-------------------------------------------------------------------------*/

static const struct of_device_id fm25_of_match[] = {
	{ .compatible = "cypress,fm25", },
	{ }
};
MODULE_DEVICE_TABLE(of, fm25_of_match);

static struct spi_driver fm25_driver = {
	.driver = {
		.name		= "fm25",
		.of_match_table = fm25_of_match,
	},
	.probe		= fm25_probe,
};

module_spi_driver(fm25_driver);

MODULE_DESCRIPTION("Driver for Cypress SPI FRAMs");
MODULE_AUTHOR("Jiri Prchal");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:fram");
