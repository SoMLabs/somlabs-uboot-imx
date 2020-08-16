 // SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 SOMLabs
 *
 */

#include <common.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <linux/err.h>


struct ph720128t003 {
	struct gpio_desc reset;
	unsigned int lanes;
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
};

struct cmd_set_entry {
	u8 cmd;
	u8 data;
};

#define CMD_INSTR(CMD, DATA)     {.cmd = CMD, .data = DATA}

static const struct cmd_set_entry ph720128t003_init_data[] = {
	CMD_INSTR(0xFF, 0x03),
	CMD_INSTR(0x01, 0x00),
	CMD_INSTR(0x02, 0x00),
	CMD_INSTR(0x03, 0x55),
	CMD_INSTR(0x04, 0x13),
	CMD_INSTR(0x05, 0x00),
	CMD_INSTR(0x06, 0x06),
	CMD_INSTR(0x07, 0x01),
	CMD_INSTR(0x08, 0x00),
	CMD_INSTR(0x09, 0x01),
	CMD_INSTR(0x0A, 0x01),
	CMD_INSTR(0x0B, 0x00),
	CMD_INSTR(0x0C, 0x00),
	CMD_INSTR(0x0D, 0x00),
	CMD_INSTR(0x0E, 0x00),
	CMD_INSTR(0x0F, 0x18),
	CMD_INSTR(0x10, 0x18),
	CMD_INSTR(0x11, 0x00),
	CMD_INSTR(0x12, 0x00),
	CMD_INSTR(0x13, 0x00),
	CMD_INSTR(0x14, 0x00),
	CMD_INSTR(0x15, 0x00),
	CMD_INSTR(0x16, 0x00),
	CMD_INSTR(0x17, 0x00),
	CMD_INSTR(0x18, 0x00),
	CMD_INSTR(0x19, 0x00),
	CMD_INSTR(0x1A, 0x00),
	CMD_INSTR(0x1B, 0x00),
	CMD_INSTR(0x1C, 0x00),
	CMD_INSTR(0x1D, 0x00),
	CMD_INSTR(0x1E, 0x44),
	CMD_INSTR(0x1F, 0x80),
	CMD_INSTR(0x20, 0x02),
	CMD_INSTR(0x21, 0x03),
	CMD_INSTR(0x22, 0x00),
	CMD_INSTR(0x23, 0x00),
	CMD_INSTR(0x24, 0x00),
	CMD_INSTR(0x25, 0x00),
	CMD_INSTR(0x26, 0x00),
	CMD_INSTR(0x27, 0x00),
	CMD_INSTR(0x28, 0x33),
	CMD_INSTR(0x29, 0x03),
	CMD_INSTR(0x2A, 0x00),
	CMD_INSTR(0x2B, 0x00),
	CMD_INSTR(0x2C, 0x00),
	CMD_INSTR(0x2D, 0x00),
	CMD_INSTR(0x2E, 0x00),
	CMD_INSTR(0x2F, 0x00),
	CMD_INSTR(0x30, 0x00),
	CMD_INSTR(0x31, 0x00),
	CMD_INSTR(0x32, 0x00),
	CMD_INSTR(0x33, 0x00),
	CMD_INSTR(0x34, 0x04),
	CMD_INSTR(0x35, 0x00),
	CMD_INSTR(0x36, 0x00),
	CMD_INSTR(0x37, 0x00),
	CMD_INSTR(0x38, 0x01),
	CMD_INSTR(0x39, 0x00),
	CMD_INSTR(0x3A, 0x00),
	CMD_INSTR(0x3B, 0x00),
	CMD_INSTR(0x3C, 0x00),
	CMD_INSTR(0x3D, 0x00),
	CMD_INSTR(0x3E, 0x00),
	CMD_INSTR(0x3F, 0x00),
	CMD_INSTR(0x40, 0x00),
	CMD_INSTR(0x41, 0x00),
	CMD_INSTR(0x42, 0x00),
	CMD_INSTR(0x43, 0x00),
	CMD_INSTR(0x44, 0x00),
	CMD_INSTR(0x50, 0x01),
	CMD_INSTR(0x51, 0x23),
	CMD_INSTR(0x52, 0x45),
	CMD_INSTR(0x53, 0x67),
	CMD_INSTR(0x54, 0x89),
	CMD_INSTR(0x55, 0xAB),
	CMD_INSTR(0x56, 0x01),
	CMD_INSTR(0x57, 0x23),
	CMD_INSTR(0x58, 0x45),
	CMD_INSTR(0x59, 0x67),
	CMD_INSTR(0x5A, 0x89),
	CMD_INSTR(0x5B, 0xAB),
	CMD_INSTR(0x5C, 0xCD),
	CMD_INSTR(0x5D, 0xEF),
	CMD_INSTR(0x5E, 0x11),
	CMD_INSTR(0x5F, 0x14),
	CMD_INSTR(0x60, 0x15),
	CMD_INSTR(0x61, 0x0F),
	CMD_INSTR(0x62, 0x0D),
	CMD_INSTR(0x63, 0x0E),
	CMD_INSTR(0x64, 0x0C),
	CMD_INSTR(0x65, 0x06),
	CMD_INSTR(0x66, 0x02),
	CMD_INSTR(0x67, 0x02),
	CMD_INSTR(0x68, 0x02),
	CMD_INSTR(0x69, 0x02),
	CMD_INSTR(0x6A, 0x02),
	CMD_INSTR(0x6B, 0x02),
	CMD_INSTR(0x6C, 0x02),
	CMD_INSTR(0x6D, 0x02),
	CMD_INSTR(0x6E, 0x02),
	CMD_INSTR(0x6F, 0x02),
	CMD_INSTR(0x70, 0x02),
	CMD_INSTR(0x71, 0x00),
	CMD_INSTR(0x72, 0x01),
	CMD_INSTR(0x73, 0x08),
	CMD_INSTR(0x74, 0x02),
	CMD_INSTR(0x75, 0x14),
	CMD_INSTR(0x76, 0x15),
	CMD_INSTR(0x77, 0x0F),
	CMD_INSTR(0x78, 0x0D),
	CMD_INSTR(0x79, 0x0E),
	CMD_INSTR(0x7A, 0x0C),
	CMD_INSTR(0x7B, 0x08),
	CMD_INSTR(0x7C, 0x02),
	CMD_INSTR(0x7D, 0x02),
	CMD_INSTR(0x7E, 0x02),
	CMD_INSTR(0x7F, 0x02),
	CMD_INSTR(0x80, 0x02),
	CMD_INSTR(0x81, 0x02),
	CMD_INSTR(0x82, 0x02),
	CMD_INSTR(0x83, 0x02),
	CMD_INSTR(0x84, 0x02),
	CMD_INSTR(0x85, 0x02),
	CMD_INSTR(0x86, 0x02),
	CMD_INSTR(0x87, 0x00),
	CMD_INSTR(0x88, 0x01),
	CMD_INSTR(0x89, 0x06),
	CMD_INSTR(0x8A, 0x02),
	CMD_INSTR(0xFF, 0x04),
	CMD_INSTR(0x6C, 0x15),
	CMD_INSTR(0x6E, 0x2A),
	CMD_INSTR(0x6F, 0x33),
	CMD_INSTR(0x3A, 0x24),
	CMD_INSTR(0x8D, 0x14),
	CMD_INSTR(0x87, 0xBA),
	CMD_INSTR(0x26, 0x76),
	CMD_INSTR(0xB2, 0xD1),
	CMD_INSTR(0xB5, 0xD7),
	CMD_INSTR(0x35, 0x1F),
	CMD_INSTR(0xFF, 0x01),
	CMD_INSTR(0x22, 0x0A),
	CMD_INSTR(0x53, 0x72),
	CMD_INSTR(0x55, 0x77),
	CMD_INSTR(0x50, 0xA6),
	CMD_INSTR(0x51, 0xA6),
	CMD_INSTR(0x31, 0x00),
	CMD_INSTR(0x60, 0x20),
	CMD_INSTR(0xA0, 0x08),
	CMD_INSTR(0xA1, 0x1A),
	CMD_INSTR(0xA2, 0x2A),
	CMD_INSTR(0xA3, 0x14),
	CMD_INSTR(0xA4, 0x17),
	CMD_INSTR(0xA5, 0x2B),
	CMD_INSTR(0xA6, 0x1D),
	CMD_INSTR(0xA7, 0x20),
	CMD_INSTR(0xA8, 0x9D),
	CMD_INSTR(0xA9, 0x1C),
	CMD_INSTR(0xAA, 0x29),
	CMD_INSTR(0xAB, 0x8F),
	CMD_INSTR(0xAC, 0x20),
	CMD_INSTR(0xAD, 0x1F),
	CMD_INSTR(0xAE, 0x4F),
	CMD_INSTR(0xAF, 0x23),
	CMD_INSTR(0xB0, 0x29),
	CMD_INSTR(0xB1, 0x56),
	CMD_INSTR(0xB2, 0x66),
	CMD_INSTR(0xB3, 0x39),
	CMD_INSTR(0xC0, 0x08),
	CMD_INSTR(0xC1, 0x1A),
	CMD_INSTR(0xC2, 0x2A),
	CMD_INSTR(0xC3, 0x15),
	CMD_INSTR(0xC4, 0x17),
	CMD_INSTR(0xC5, 0x2B),
	CMD_INSTR(0xC6, 0x1D),
	CMD_INSTR(0xC7, 0x20),
	CMD_INSTR(0xC8, 0x9D),
	CMD_INSTR(0xC9, 0x1D),
	CMD_INSTR(0xCA, 0x29),
	CMD_INSTR(0xCB, 0x8F),
	CMD_INSTR(0xCC, 0x20),
	CMD_INSTR(0xCD, 0x1F),
	CMD_INSTR(0xCE, 0x4F),
	CMD_INSTR(0xCF, 0x24),
	CMD_INSTR(0xD0, 0x29),
	CMD_INSTR(0xD1, 0x56),
	CMD_INSTR(0xD2, 0x66),
	CMD_INSTR(0xD3, 0x39),
	CMD_INSTR(0xFF, 0x00),
	CMD_INSTR(0x11, 0x00),
};

static const struct display_timing ph720128t003_default_timing = {
	.pixelclock.typ   = 54000000,
	.hactive.typ      = 720,
	.hfront_porch.typ = 20,
	.hsync_len.typ    = 60,
	.hback_porch.typ  = 20,
	.vactive.typ      = 1280,
	.vfront_porch.typ = 10,
	.vsync_len.typ    = 2,
	.vback_porch.typ  = 15,
	.flags = DISPLAY_FLAGS_HSYNC_LOW |
		 DISPLAY_FLAGS_VSYNC_LOW |
		 DISPLAY_FLAGS_DE_LOW |
		 DISPLAY_FLAGS_PIXDATA_NEGEDGE,
};

static int ph720128t003_switch_page(struct mipi_dsi_device *device, u8 page)
{
    u8 buf[4] = { 0xff, 0x98, 0x81, page };

	return mipi_dsi_dcs_write_buffer(device, buf, sizeof(buf));
}

static int ph720128t003_send_cmd_data(struct mipi_dsi_device *dsi, u8 cmd, u8 data)
{
	u8 buf[2] = { cmd, data };
	return mipi_dsi_dcs_write_buffer(dsi, buf, sizeof(buf));
}

static int ph720128t003_panel_push_cmd_list(struct mipi_dsi_device *device)
{
	size_t i;
	struct cmd_set_entry entry;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(ph720128t003_init_data); i++) {
		entry = ph720128t003_init_data[i];
        if(entry.cmd == 0xFF) {
            ret = ph720128t003_switch_page(device, entry.data);
        } else {
		    ret = ph720128t003_send_cmd_data(device, entry.cmd, entry.data);
        }
		if (ret < 0) {
            break;
        }
	}

	return ret;
};

static int ph720128t003_enable(struct udevice *dev)
{
	struct ph720128t003 *ctx = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	struct mipi_dsi_device *dsi = plat->device;
	u16 brightness;
	int ret;

	ret = ph720128t003_panel_push_cmd_list(dsi);
	if (ret < 0) {
		printf("Failed to send init sequence! (%d)\n", ret);
		return -EIO;
	}
    mdelay(20);

	ph720128t003_switch_page(dsi, 0);
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		printf("Failed to exit sleep mode (%d)\n", ret);
		return -EIO;
	}
	mdelay(120);
	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		printf("Failed to set display ON (%d)\n", ret);
		return -EIO;
	}

    return 0;
}

static int ph720128t003_panel_enable_backlight(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	struct mipi_dsi_device *device = plat->device;
	int ret;

	ret = mipi_dsi_attach(device);
	if (ret < 0)
		return ret;

	return ph720128t003_enable(dev);
}

static int ph720128t003_panel_get_display_timing(struct udevice *dev,
					    struct display_timing *timings)
{
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	struct mipi_dsi_device *device = plat->device;
	struct ph720128t003 *ctx = dev_get_priv(dev);

	memcpy(timings, &ph720128t003_default_timing, sizeof(*timings));

	/* fill characteristics of DSI data link */
	if (device) {
		device->lanes = ctx->lanes;
		device->format = ctx->format;
		device->mode_flags = ctx->mode_flags;
	}

	return 0;
}

static int ph720128t003_panel_probe(struct udevice *dev)
{
	struct ph720128t003 *ctx = dev_get_priv(dev);
	int ret;

	ctx->format = MIPI_DSI_FMT_RGB888;
	ctx->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE | MIPI_DSI_MODE_LPM;
    //ctx->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_VIDEO_HSE;
    ctx->lanes = 2;

	ret = gpio_request_by_name(dev, "reset-gpio", 0, &ctx->reset,
				   GPIOD_IS_OUT);
	if (ret) {
		printf("Warning: cannot get reset GPIO\n");
		if (ret != -ENOENT)
			return ret;
	}

	/* reset panel */
	ret = dm_gpio_set_value(&ctx->reset, true);
	if (ret)
		printf("reset gpio fails to set true\n");
	mdelay(20);
	ret = dm_gpio_set_value(&ctx->reset, false);
	if (ret)
		printf("reset gpio fails to set true\n");
	mdelay(20);

	return 0;
}

static int ph720128t003_panel_disable(struct udevice *dev)
{
	struct ph720128t003 *ctx = dev_get_priv(dev);

	dm_gpio_set_value(&ctx->reset, true);

	return 0;
}

static const struct panel_ops ph720128t003_panel_ops = {
	.enable_backlight = ph720128t003_panel_enable_backlight,
	.get_display_timing = ph720128t003_panel_get_display_timing,
};

static const struct udevice_id ph720128t003_panel_ids[] = {
	{ .compatible = "powertip,ph720128t003" },
	{ }
};

U_BOOT_DRIVER(ph720128t003_panel) = {
	.name			  = "ph720128t003_panel",
	.id			      = UCLASS_PANEL,
	.of_match		  = ph720128t003_panel_ids,
	.ops			  = &ph720128t003_panel_ops,
	.probe			  = ph720128t003_panel_probe,
	.remove			  = ph720128t003_panel_disable,
	.platdata_auto_alloc_size = sizeof(struct mipi_dsi_panel_plat),
	.priv_auto_alloc_size	= sizeof(struct ph720128t003),
};
