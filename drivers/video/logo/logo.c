// SPDX-License-Identifier: GPL-2.0-only

/*
 *  Linux logo to be displayed on boot
 *
 *  Copyright (C) 1996 Larry Ewing (lewing@isc.tamu.edu)
 *  Copyright (C) 1996,1998 Jakub Jelinek (jj@sunsite.mff.cuni.cz)
 *  Copyright (C) 2001 Greg Banks <gnb@alphalink.com.au>
 *  Copyright (C) 2001 Jan-Benedict Glaw <jbglaw@lug-owl.de>
 *  Copyright (C) 2003 Geert Uytterhoeven <geert@linux-m68k.org>
 */

#include <linux/linux_logo.h>
#include <linux/stddef.h>
#include <linux/module.h>

#ifdef CONFIG_M68K
#include <asm/setup.h>
#endif

static bool nologo;
module_param(nologo, bool, 0);
MODULE_PARM_DESC(nologo, "Disables startup logo");

/*
 * Logos are located in the initdata, and will be freed in kernel_init.
 * Use late_init to mark the logos as freed to prevent any further use.
 */

static bool logos_freed;

static int __init fb_logo_late_init(void)
{
	logos_freed = true;
	return 0;
}

late_initcall_sync(fb_logo_late_init);

/* logo's are marked __initdata. Use __ref to tell
 * modpost that it is intended that this function uses data
 * marked __initdata.
 */
#ifdef CONFIG_LOGO_ADV_CUSTOM

#include <linux/slab.h>
#include <linux/io.h>

#define __BMP_HEADER_SIZE               0x36 //54
#define __BMP_DIB_HEADER_SIZE           0x28 //40
// Correct values for the header
#define __BMP_MAGIC_VALUE               0x4d42
#define __BMP_NUM_PLANE                 0x1
#define __BMP_COMPRESSION               0x0
#define __BMP_NUM_COLORS                0x0
#define __BMP_IMPORTANT_COLORS          0x0
#define __BMP_BITS_PER_PIXEL            0x18
#define __BMP_BITS_PER_BYTE             0x8
#define __MAX_BMP_FILE_SIZE             (1920 * 1080 * 3) + 54  //max support 1920x1080 24bit bmp
#define __BMP_RESERVED_PIXEL            20       // valid resolution is less then (width - 20, height - 20) of device resolution.

#define MAX_LINUX_LOGO_COLORS	        224
#define LINUX_CLUT224_START_INDEX       0x20

/*
 *      it's define in imx8mm-advantech.dtsi [reserved-memory] -> [logo_reserved]
 *      logo_reserved: logo_reserved {
 *              reg = <0 0xb8400000 0 0x600000>;
 *      };
 */
#define __BMP_LOGO_RESERVED_MEM_SIZE    0x600000
#define __BMP_LOGO_RESERVED_MEM_ADDR    0xb8400000

typedef struct {
        uint16_t type;                  // Magic identifier: 0x4d42
        uint32_t size;                  // File size in bytes
        uint32_t reserved;              // Not used
        uint32_t offset;                // Offset to image data in bytes from beginning of file
        uint32_t dib_header_size;       // DIB Header size in bytes
        uint32_t  width_px;             // Width of the image
        uint32_t  height_px;            // Height of image
        uint16_t num_planes;            // Number of color planes
        uint16_t bits_per_pixel;        // Bits per pixel
        uint32_t compression;           // Compression type
        uint32_t image_size_bytes;      // Image size in bytes
        uint32_t x_resolution_ppm;      // Pixels per meter
        uint32_t  y_resolution_ppm;     // Pixels per meter
        uint32_t num_colors;            // Number of colors
        uint32_t important_colors;      // Important colors
} BMPHeader;

typedef struct {
        uint8_t red,green,blue;
} COLOR;

typedef struct {
        BMPHeader header;
        volatile uint8_t *data;
        uint8_t padding;
        COLOR *pixel_color;
} BMPImage;

bool color_equal(COLOR* c1, COLOR* c2)
{
        if (c1->red == c2->red) {
                if (c1->green == c2->green){
                        if (c1->blue == c2->blue) return true;
                }
        }
        return false;
}

/*
 * check bmp header is valid or not.
*/
bool check_bmp_header(BMPImage* bmp_image)
{
        uint32_t _t_width_bytes = (bmp_image->header.width_px * sizeof(COLOR));
        if (bmp_image->padding > 0) _t_width_bytes = _t_width_bytes + 4 - bmp_image->padding;
        return bmp_image->header.type == __BMP_MAGIC_VALUE
        && bmp_image->header.size < __MAX_BMP_FILE_SIZE
        && bmp_image->header.offset == __BMP_HEADER_SIZE
        && bmp_image->header.dib_header_size == __BMP_DIB_HEADER_SIZE
        && bmp_image->header.num_planes == __BMP_NUM_PLANE
        && bmp_image->header.compression == __BMP_COMPRESSION
        && bmp_image->header.num_colors == __BMP_NUM_COLORS && bmp_image->header.important_colors == __BMP_IMPORTANT_COLORS
        && bmp_image->header.bits_per_pixel == __BMP_BITS_PER_PIXEL
        && bmp_image->header.size == (_t_width_bytes * bmp_image->header.height_px) + __BMP_HEADER_SIZE
        ;
}

/*
 * load bmp data from specific memory address.
 * need to write bmp data before kernel boot.
 * linux logo only support 224 colors in bmp and valid resolution (less then device resolution).
*/
const struct linux_logo* load_bmp_from_mem(uint32_t valid_xres, uint32_t  valid_yres)
{
        struct linux_logo* logo = NULL;
        void __iomem *mapped_area = ioremap_cache (__BMP_LOGO_RESERVED_MEM_ADDR, __BMP_LOGO_RESERVED_MEM_SIZE);
        int i = 0, x = 0, y = 0;
        if(mapped_area == NULL)
        {
                printk(KERN_WARNING "cannot get ioremap from 0x%x size=0x%x", __BMP_LOGO_RESERVED_MEM_ADDR, __BMP_LOGO_RESERVED_MEM_SIZE);
                goto error;
        }

        BMPImage image = {
                .data = NULL,
                .padding = 0,
                .pixel_color = NULL
        };

        image.header.type = *((uint16_t*)mapped_area);
        image.header.size = *((uint32_t*)(mapped_area+2));
        image.header.reserved = *((uint32_t*)(mapped_area+6));
        image.header.offset = *((uint32_t*)(mapped_area+10));
        image.header.dib_header_size = *((uint32_t*)(mapped_area+14));
        image.header.width_px = *((uint32_t*)(mapped_area+18));
        image.header.height_px = *((uint32_t*)(mapped_area+22));
        image.header.num_planes = *((uint16_t*)(mapped_area+26));
        image.header.bits_per_pixel = *((uint16_t*)(mapped_area+28));
        image.header.compression = *((uint32_t*)(mapped_area+30));
        image.header.image_size_bytes = *((uint32_t*)(mapped_area+34));
        image.header.x_resolution_ppm = *((uint32_t*)(mapped_area+38));
        image.header. y_resolution_ppm = *((uint32_t*)(mapped_area+42));
        image.header.num_colors = *((uint32_t*)(mapped_area+46));
        image.header.important_colors = *((uint32_t*)(mapped_area+50));
        image.padding = (image.header.width_px * sizeof(COLOR)) % 4;    // each row is rounded up to a multiple of 4 bytes (a 32-bit DWORD) by padding

        if (check_bmp_header(&image) == true)
        {
                image.data = (volatile uint8_t*)(mapped_area + __BMP_HEADER_SIZE);
                uint8_t* memdata = image.data;
                COLOR *img_data_head = NULL;
                COLOR *img_color = NULL;
                uint8_t clut_cnt = 0;
                COLOR clut[MAX_LINUX_LOGO_COLORS] = {0};
                uint8_t *clut224_data = NULL;

                if ((image.header.width_px > valid_xres) || (image.header.height_px > valid_yres))
                {
                        printk(KERN_WARNING "Invalid bmp file (%u x %u). valid bmp resolution is %u x %u", image.header.width_px, image.header.height_px, valid_xres, valid_yres);
                        goto error;
                }

                image.pixel_color = kmalloc(image.header.width_px * image.header.height_px * 3, GFP_KERNEL);
                img_data_head = image.pixel_color;
                for( y = image.header.height_px - 1; y >= 0; y--)
                {
                        img_color = img_data_head + (image.header.width_px * y);
                        for( x = 0; x < image.header.width_px; x++)
                        {
                                img_color->blue = *memdata++;
                                img_color->green = *memdata++;
                                img_color->red = *memdata++;
                                i = 0;
                                for(i = 0; i < clut_cnt; i++)
                                {
                                        if(color_equal(img_color, clut+i)) break;
                                }
                                if (i == MAX_LINUX_LOGO_COLORS)
                                {
                                        printk(KERN_WARNING "Invalid bmp file. (only 224 colors)");
                                        goto error;
                                }
                                if(i == clut_cnt)
                                {
                                        clut[clut_cnt] = *img_color;
                                        clut_cnt++;
                                }
                                img_color++;
                        }
                        if(image.padding > 0) memdata = memdata + (4-image.padding);
                }

                clut224_data = kmalloc(image.header.width_px * image.header.height_px, GFP_KERNEL);
                for ( y = 0; y < image.header.height_px; y++)
                {
                        for( x = 0; x < image.header.width_px; x++)
                        {
                                img_color = img_data_head + (image.header.width_px * y) + x;
                                for( i = 0; i < clut_cnt; i++)
                                {
                                        if(color_equal(img_color, clut+i)==true)
                                        {
                                                clut224_data[(image.header.width_px * y)+x] = LINUX_CLUT224_START_INDEX + i;
                                        }
                                }
                        }
                }

                logo = kmalloc(sizeof(struct linux_logo), GFP_KERNEL);
                logo->type = LINUX_LOGO_CLUT224;
                logo->width = image.header.width_px;
                logo->height = image.header.height_px;
                logo->clutsize = clut_cnt;
                logo->clut = kmalloc(clut_cnt*sizeof(COLOR), GFP_KERNEL);
                memcpy(logo->clut, clut, clut_cnt*sizeof(COLOR));
                logo->data = clut224_data;
        }
        else
        {
                printk(KERN_WARNING "Invalid bmp file.");
        }

error:
        if (image.pixel_color != NULL) kfree(image.pixel_color);
        if (mapped_area != NULL) iounmap (mapped_area);

        return logo;
}

const struct linux_logo * __ref fb_find_logo(int depth , unsigned int xres, unsigned int  yres)
{
        const struct linux_logo *logo = NULL;

        if (nologo || logos_freed)
                return NULL;
        if (depth >= 1) {
#ifdef CONFIG_LOGO_LINUX_MONO
                /* Generic Linux logo */
                logo = &logo_linux_mono;
#endif
#ifdef CONFIG_LOGO_SUPERH_MONO
                /* SuperH Linux logo */
                logo = &logo_superh_mono;
#endif
        }

        if (depth >= 4) {
#ifdef CONFIG_LOGO_LINUX_VGA16
                /* Generic Linux logo */
                logo = &logo_linux_vga16;
#endif
#ifdef CONFIG_LOGO_SUPERH_VGA16
                /* SuperH Linux logo */
                logo = &logo_superh_vga16;
#endif
        }

        if (depth >= 8) {
#ifdef CONFIG_LOGO_LINUX_CLUT224
                /* Generic Linux logo */
                logo = &logo_linux_clut224;
#endif
#ifdef CONFIG_LOGO_DEC_CLUT224
                /* DEC Linux logo on MIPS/MIPS64 or ALPHA */
                logo = &logo_dec_clut224;
#endif
#ifdef CONFIG_LOGO_MAC_CLUT224
                /* Macintosh Linux logo on m68k */
                if (MACH_IS_MAC)
                        logo = &logo_mac_clut224;
#endif
#ifdef CONFIG_LOGO_PARISC_CLUT224
                /* PA-RISC Linux logo */
                logo = &logo_parisc_clut224;
#endif
#ifdef CONFIG_LOGO_SGI_CLUT224
                /* SGI Linux logo on MIPS/MIPS64 */
                logo = &logo_sgi_clut224;
#endif
#ifdef CONFIG_LOGO_SUN_CLUT224
                /* Sun Linux logo */
                logo = &logo_sun_clut224;
#endif
#ifdef CONFIG_LOGO_SUPERH_CLUT224
                /* SuperH Linux logo */
                logo = &logo_superh_clut224;
#endif
                // try load logo from reserved-memory written by uboot.
                // example: fatload mmc ${mmcdev}:${mmcpart} 0xb8400000 logo.bmp
                logo = load_bmp_from_mem(xres - __BMP_RESERVED_PIXEL, yres - __BMP_RESERVED_PIXEL);
                if (logo != NULL) return logo;
                
                if (xres <= 1024 && yres <= 600)
                        logo = &logo_adv_custom_1024_600_clut224;
                else if (xres <= 1280 && yres <= 800)
                        logo = &logo_adv_custom_1280_800_clut224;
                else if (xres <= 1388 && yres <= 768)
                        logo = &logo_adv_custom_1366_768_clut224;
                else if (xres <= 1920 && yres <= 1080)
                        logo = &logo_adv_custom_1920_1080_clut224;
                else
                        logo = &logo_linux_clut224;


        }
        return logo;
}
EXPORT_SYMBOL_GPL(fb_find_logo);
#else
const struct linux_logo * __ref fb_find_logo(int depth)
{
	const struct linux_logo *logo = NULL;

	if (nologo || logos_freed)
		return NULL;

	if (depth >= 1) {
#ifdef CONFIG_LOGO_LINUX_MONO
		/* Generic Linux logo */
		logo = &logo_linux_mono;
#endif
#ifdef CONFIG_LOGO_SUPERH_MONO
		/* SuperH Linux logo */
		logo = &logo_superh_mono;
#endif
	}
	
	if (depth >= 4) {
#ifdef CONFIG_LOGO_LINUX_VGA16
		/* Generic Linux logo */
		logo = &logo_linux_vga16;
#endif
#ifdef CONFIG_LOGO_SUPERH_VGA16
		/* SuperH Linux logo */
		logo = &logo_superh_vga16;
#endif
	}
	
	if (depth >= 8) {
#ifdef CONFIG_LOGO_LINUX_CLUT224
		/* Generic Linux logo */
		logo = &logo_linux_clut224;
#endif
#ifdef CONFIG_LOGO_DEC_CLUT224
		/* DEC Linux logo on MIPS/MIPS64 or ALPHA */
		logo = &logo_dec_clut224;
#endif
#ifdef CONFIG_LOGO_MAC_CLUT224
		/* Macintosh Linux logo on m68k */
		if (MACH_IS_MAC)
			logo = &logo_mac_clut224;
#endif
#ifdef CONFIG_LOGO_PARISC_CLUT224
		/* PA-RISC Linux logo */
		logo = &logo_parisc_clut224;
#endif
#ifdef CONFIG_LOGO_SGI_CLUT224
		/* SGI Linux logo on MIPS/MIPS64 */
		logo = &logo_sgi_clut224;
#endif
#ifdef CONFIG_LOGO_SUN_CLUT224
		/* Sun Linux logo */
		logo = &logo_sun_clut224;
#endif
#ifdef CONFIG_LOGO_SUPERH_CLUT224
		/* SuperH Linux logo */
		logo = &logo_superh_clut224;
#endif
	}
	return logo;
}
EXPORT_SYMBOL_GPL(fb_find_logo);
#endif
