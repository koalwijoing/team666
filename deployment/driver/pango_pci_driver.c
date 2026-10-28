#include "pango_pci_driver.h"
#include "id_config.h"
#include <linux/ktime.h>
#include <linux/math64.h>

//#define NDEBUG

#ifndef NDEBUG
#define LOG(msg ...) printk(KERN_ALERT msg)
#else
#define LOG(msg ...)
#endif

#define PCI_DRIVER_DEV_COUNT 1
#define PCI_DRIVER_DEV_NAME "pango_pci_driver"

struct file_operations pango_cdev_fops = {
	.owner   = THIS_MODULE,
	.llseek  = pango_cdev_llseek,
	.read    = pango_cdev_read,
	.write   = pango_cdev_write,
	.open    = pango_cdev_open,
	.unlocked_ioctl = pango_cdev_ioctl,
	.mmap    = pango_cdev_mmap,
	.release = pango_cdev_release,
};

struct PciDriverDevInfo pci_dev_info = {
	._dev            = 0,
	._dev_firstminor = 0,
	._dev_count      = PCI_DRIVER_DEV_COUNT,
	._dev_name       = PCI_DRIVER_DEV_NAME,
};

struct pci_device_id pci_pango_device_ids[] = {
	{ PCI_DEVICE(PCI_PANGO_DEFAULT_VENDOR_ID, PCI_PANGO_DEFAULT_DEVICE_ID) },
	{ 0, },
};

struct PciPango pci_info = {
	._pango_pci_driver = {
		._pci_bar     = 1,
		._pci_io_size = 0,
		._pci_io      = NULL,
		._pci_io_buff = NULL,
		._pci_status_bar = PANGO_DMA_DONE_BAR,
		._pci_status_io_size = 0,
		._pci_status_io = NULL,
		._pci_driver  = {
			.name     = PCI_DRIVER_DEV_NAME,
			.id_table = pci_pango_device_ids,
			.probe    = pci_driver_probe,
			.remove   = pci_driver_remove,
		},
	},
	._cdev_class = NULL,
};


struct pci_dev *op_dev;

static struct PangoFrameBuffers frame_buffers;

///////////////////////////////////////////////////////////////////////

loff_t pango_cdev_llseek(struct file *filp, loff_t off, int whence)
{
	struct PciPango *pci_pango = &pci_info;
	loff_t newpos = 0;
	
	//LOG("pango_cdev_llseek.\n");
	
	switch (whence)
	{
	case 0: /* SEEK_SET */
	{
		newpos = off;
	}
		break;
	case 1: /* SEEK_CUR */
	{
		newpos = filp->f_pos + off;
	}
		break;
	case 2: /* SEEK_END */
	{
		newpos = pci_pango->_pango_pci_driver._pci_io_size + off;
	}
		break;
	default: /* can't happen */
	{
		return -EINVAL;
	}	
		break;
	}
	
	if (newpos < 0)
	{
		return -EINVAL;
	}
	
	filp->f_pos = newpos;
	
	return newpos;
}

static void ReadConfig(struct pci_dev * pdev)
{
	int i;
	u8 valb;
	u16 valw;
	u32 valdw;
	u8 id;
	u8 next;
	int pos;
	unsigned long reg_base, reg_len;

	/* Read PCI configuration space 读取PCI配置空间*/
	printk(KERN_INFO "PCI Configuration Space:\n");
	for(i=0; i< 1024; i++)
	{
		pci_read_config_dword(pdev, i*4, &valdw);
		command_operation.get_pci_dev_info.data[i] = valdw;
	}

	/* Now read each element - one at a time 现在读取每个元素——一次读取一个*/

	/* Read Vendor ID 读取厂商ID*/
	pci_read_config_word(pdev, PCI_VENDOR_ID, &valw);
	printk("Vendor ID: 0x%x, \n", valw);
	command_operation.get_pci_dev_info.vendor_id = valw;

	/* Read Device ID 读取设备ID*/
	pci_read_config_word(pdev, PCI_DEVICE_ID, &valw);
	printk("Device ID: 0x%x, \n", valw);
	command_operation.get_pci_dev_info.device_id = valw;

	/* Read Command Register 读取命令寄存器*/
	pci_read_config_word(pdev, PCI_COMMAND, &valw);
	printk("Cmd Reg: 0x%x, \n", valw);
	command_operation.get_pci_dev_info.cmd_reg = valw;

	/* Read Status Register 读取状态寄存器*/
	pci_read_config_word(pdev, PCI_STATUS, &valw);
	printk("Stat Reg: 0x%x, \n", valw);
	command_operation.get_pci_dev_info.status_reg = valw;

	/* Read Revision ID 阅读修订ID*/
	pci_read_config_byte(pdev, PCI_REVISION_ID, &valb);
	printk("Revision ID: 0x%x, \n", valb);
	command_operation.get_pci_dev_info.revision_id = valb;

	/* Read Class Code */
	/*
	pci_read_config_dword(pdev, PCI_CLASS_PROG, &valdw);
	printk("Class Code: 0x%lx, ", valdw);
	valdw &= 0x00ffffff;
	printk("Class Code: 0x%lx, ", valdw);
	*/
	/* Read Reg-level Programming Interface 读取reg级编程接口*/
	pci_read_config_byte(pdev, PCI_CLASS_PROG, &valb);
	printk("Class Prog: 0x%x, \n", valb);
	command_operation.get_pci_dev_info.class_prog = valb;

	/* Read Device Class */
	pci_read_config_word(pdev, PCI_CLASS_DEVICE, &valw);
	printk("Device Class: 0x%x, \n", valw);
	command_operation.get_pci_dev_info.class_device = valw;

	  /* Read Cache Line 读缓存线*/
	  pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &valb);
	  printk("Cache Line Size: 0x%x, \n", valb);
	
	  /* Read Latency Timer 读延迟计时器*/
	  pci_read_config_byte(pdev, PCI_LATENCY_TIMER, &valb);
	  printk("Latency Timer: 0x%x, \n", valb);
	
	  /* Read Header Type 读头类型*/
	  pci_read_config_byte(pdev, PCI_HEADER_TYPE, &valb);
	  printk("Header Type: 0x%x, \n", valb);
	
	  /* Read BIST */
	  pci_read_config_byte(pdev, PCI_BIST, &valb);
	  printk("BIST: 0x%x\n", valb);

	/* Read all 6 BAR registers 读取所有6个BAR寄存器*/
	for(i=0; i<=5; i++)
	{
		/* Physical address & length 物理地址和长度*/
		reg_base = pci_resource_start(pdev, i);
		reg_len = pci_resource_len(pdev, i);
		printk("BAR%d: Addr:0x%lx Len:0x%lx,  ", i, reg_base, reg_len);
		command_operation.get_pci_dev_info.bar[i].bar_base = reg_base;
		command_operation.get_pci_dev_info.bar[i].bar_len = reg_len;

		/* Flags */
		if((pci_resource_flags(pdev, i) & IORESOURCE_MEM))
		  printk("Region is for memory\n");
		else if((pci_resource_flags(pdev, i) & IORESOURCE_IO))
		  printk("Region is for I/O\n");
	}
	printk("\n");

	  /* Read CIS Pointer 读CIS指针*/
	  pci_read_config_dword(pdev, PCI_CARDBUS_CIS, &valdw);
	  printk("CardBus CIS Pointer: 0x%x, \n", valdw);
	
	  /* Read Subsystem Vendor ID 读取子系统供应商ID*/
	  pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &valw);
	  printk("Subsystem Vendor ID: 0x%x, \n", valw);
	
	  /* Read Subsystem Device ID 读取子系统设备ID*/
	  pci_read_config_word(pdev, PCI_SUBSYSTEM_ID, &valw);
	  printk("Subsystem Device ID: 0x%x\n", valw);
	
	  /* Read Expansion ROM Base Address 读取扩展ROM的基地地址*/
	  pci_read_config_dword(pdev, PCI_ROM_ADDRESS, &valdw);
	  printk("Expansion ROM Base Address: 0x%x\n", valdw);
	
	  /* Read IRQ Line 读IRQ线*/
	  pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &valb);
	  printk("IRQ Line: 0x%x, ", valb);
	
	  /* Read IRQ Pin 读IRQ引脚*/
	  pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &valb);
	  printk("IRQ Pin: 0x%x, ", valb);

	/* Read Min Gnt */
	pci_read_config_byte(pdev, PCI_MIN_GNT, &valb);
	printk("Min Gnt: 0x%x, ", valb);
	command_operation.get_pci_dev_info.min_gnt = valb;

	/* Read Max Lat */
	pci_read_config_byte(pdev, PCI_MAX_LAT, &valb);
	printk("Max Lat: 0x%x\n", valb);
	command_operation.get_pci_dev_info.max_lat = valb;
	
	if(pci_find_capability(pdev, PCI_CAP_ID_EXP))
	{
		pos = pci_find_capability(pdev, PCI_CAP_ID_EXP);
		pci_read_config_word(pdev, pos + PCI_EXP_LNKSTA, &valw);
		command_operation.get_pci_dev_info.link_speed = (valw & 0x0003);
		command_operation.get_pci_dev_info.link_width = (valw & 0x03f0) >> 4;
		printk("Link Speed: %d\n", command_operation.get_pci_dev_info.link_speed);
		printk("Link Width: x%d\n", command_operation.get_pci_dev_info.link_width);
		
		pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &valw);
		command_operation.get_pci_dev_info.mps = 128 << ((valw & PCI_EXP_DEVCTL_PAYLOAD) >> 5);
		command_operation.get_pci_dev_info.mrrs = 128 << ((valw & PCI_EXP_DEVCTL_READRQ) >> 12);
		printk("MPS: %d\n", command_operation.get_pci_dev_info.mps);
		printk("MRRS: %d\n", command_operation.get_pci_dev_info.mrrs);
		
	}
	else
	{
		printk("Cannot find PCI Express Capabilities\n");
		command_operation.get_pci_dev_info.link_speed = 0;
		command_operation.get_pci_dev_info.link_width = 0;
		command_operation.get_pci_dev_info.mps = 0;
		command_operation.get_pci_dev_info.mrrs = 0;
	}

	pci_read_config_word(pdev, PCI_STATUS, &valw);
	command_operation.cap_info.cap_error = 0;
	if (!(valw & PCI_STATUS_CAP_LIST))
	{
		command_operation.cap_info.cap_status = 0;
	}
	else
	{
		command_operation.cap_info.cap_status = 1;
		for(i = 0; i < 256; i++)
		{
			command_operation.cap_info.cap_buf[i].flag = 0;
		}
		pci_read_config_byte(pdev, PCI_CAPABILITY_LIST, &valb);
		valb &= ~3;
		while(valb)
		{

			pci_read_config_byte(pdev, valb + PCI_CAP_LIST_ID, &id);		
			pci_read_config_byte(pdev, valb + PCI_CAP_LIST_NEXT, &next);
			next &= ~3;
			command_operation.cap_info.cap_buf[valb].flag = 1;
			command_operation.cap_info.cap_buf[valb].id = id;
			command_operation.cap_info.cap_buf[valb].addr_offset = valb;
			command_operation.cap_info.cap_buf[valb].next_offset = next;
			printk("cap id = %x; addr_offset = %x; next_offset = %x\n", id, valb, next);
			if(id == 0xff)
			{
				command_operation.cap_info.cap_error = 1;
				break;
			}
			valb = next;
		}
	}

}

ssize_t pango_cdev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	if (copy_to_user(buf, &command_operation, sizeof(COMMAND_OPERATION)))
		return -EFAULT;
	return 1;
}

ssize_t pango_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	return 1;
}

static void set_dma_w_r(unsigned int value, struct PciPango *pci_pango)
{
	iowrite32(value, pci_pango->_pango_pci_driver._pci_io + CMD_REG_OFFSET);
}

static void set_dma_addr(DMA_ADDR* dma_addr, struct PciPango *pci_pango)
{
	iowrite32(dma_addr->addr & 0x00000000ffffffff, pci_pango->_pango_pci_driver._pci_io + RW_ADDR_LO_OFFSET);
	if(dma_addr->addr_size)
		iowrite32(dma_addr->addr >> 32, pci_pango->_pango_pci_driver._pci_io + RW_ADDR_HI_OFFSET);
}

#define PANGO_MAX_FRAME_BYTES (16U * 1024U * 1024U)

static void pango_free_frame_buffers(void)
{
	if (!frame_buffers.cpu_addr || !op_dev)
		return;

	dma_free_coherent(&op_dev->dev, frame_buffers.mmap_bytes,
			  frame_buffers.cpu_addr, frame_buffers.dma_addr);
	frame_buffers.cpu_addr = NULL;
	frame_buffers.dma_addr = (dma_addr_t)0;
	frame_buffers.frame_bytes = 0;
	frame_buffers.slot_bytes = 0;
	frame_buffers.mmap_bytes = 0;
	frame_buffers.frame_sequence = 0;
}

static int pango_configure_frame_buffers(struct pango_frame_config *cfg)
{
	u64 frame_bytes64;
	size_t frame_bytes;
	size_t slot_bytes;
	size_t mmap_bytes;
	void *cpu_addr;
	dma_addr_t dma_addr;

	if (!op_dev)
		return -ENODEV;
	if (cfg->abi_version != PANGO_FRAME_ABI_VERSION ||
	    !cfg->width || !cfg->height || !cfg->pixel_bytes)
		return -EINVAL;
	if (cfg->packet_bytes != PANGO_DMA_PACKET_BYTES ||
	    cfg->buffer_count != PANGO_FRAME_BUFFER_COUNT)
		return -EINVAL;

	frame_bytes64 = (u64)cfg->width * cfg->height * cfg->pixel_bytes;
	if (!frame_bytes64 || frame_bytes64 > PANGO_MAX_FRAME_BYTES)
		return -E2BIG;
	frame_bytes = (size_t)frame_bytes64;
	/* The FPGA frame counter advances once per complete 2048-byte request. */
	if (frame_bytes % cfg->packet_bytes)
		return -EINVAL;

	slot_bytes = PAGE_ALIGN(frame_bytes);
	mmap_bytes = slot_bytes * cfg->buffer_count;

	if (frame_buffers.cpu_addr &&
	    frame_buffers.frame_bytes == frame_bytes &&
	    frame_buffers.width == cfg->width &&
	    frame_buffers.height == cfg->height &&
	    frame_buffers.pixel_bytes == cfg->pixel_bytes &&
	    frame_buffers.packet_bytes == cfg->packet_bytes &&
	    frame_buffers.buffer_count == cfg->buffer_count)
		return 0;
	if (atomic_read(&frame_buffers.mmap_count))
		return -EBUSY;

	pango_free_frame_buffers();
	cpu_addr = dma_alloc_coherent(&op_dev->dev, mmap_bytes, &dma_addr,
				      GFP_KERNEL | __GFP_NOWARN);
	if (!cpu_addr)
		return -ENOMEM;

	memset(cpu_addr, 0, mmap_bytes);
	frame_buffers.cpu_addr = cpu_addr;
	frame_buffers.dma_addr = dma_addr;
	frame_buffers.frame_bytes = frame_bytes;
	frame_buffers.slot_bytes = slot_bytes;
	frame_buffers.mmap_bytes = mmap_bytes;
	frame_buffers.width = cfg->width;
	frame_buffers.height = cfg->height;
	frame_buffers.pixel_bytes = cfg->pixel_bytes;
	frame_buffers.packet_bytes = cfg->packet_bytes;
	frame_buffers.buffer_count = cfg->buffer_count;
	frame_buffers.frame_sequence = 0;

	LOG("frame mmap: %ux%u, frame=%zu, slot=%zu, total=%zu, dma=%pad\n",
	    cfg->width, cfg->height, frame_bytes, slot_bytes, mmap_bytes, &dma_addr);
	return 0;
}

static int pango_wait_dma_done(u32 before, u64 deadline_ns)
{
	u32 value;

	if (!pci_info._pango_pci_driver._pci_status_io)
		return -ENODEV;

	do {
		value = ioread32(pci_info._pango_pci_driver._pci_status_io +
				 PANGO_DMA_DONE_COUNT_OFFSET);
		if (value != before)
			return 0;
		cpu_relax();
	} while (ktime_get_ns() < deadline_ns);

	return -ETIMEDOUT;
}

static int pango_capture_frame(struct pango_frame_capture *capture)
{
	struct PciPango *pci_pango = &pci_info;
	u64 start_ns;
	u64 deadline_ns;
	dma_addr_t target;
	u32 offset;
	u32 before;
	u32 dma_cmd;
	int ret;

	if (!frame_buffers.cpu_addr)
		return -ENXIO;
	if (capture->buffer_index >= frame_buffers.buffer_count)
		return -EINVAL;
	if (!capture->timeout_ms)
		capture->timeout_ms = 1000;
	if (capture->timeout_ms > 10000)
		return -EINVAL;

	start_ns = ktime_get_ns();
	deadline_ns = start_ns + (u64)capture->timeout_ms * NSEC_PER_MSEC;
	target = frame_buffers.dma_addr +
		 (dma_addr_t)capture->buffer_index * frame_buffers.slot_bytes;

	/* op_type=1 (FPGA-to-host), address type is 32-bit by DMA-mask policy. */
	dma_cmd = ((frame_buffers.packet_bytes / 4U) - 1U) | BIT(24);
	for (offset = 0; offset < frame_buffers.frame_bytes;
	     offset += frame_buffers.packet_bytes) {
		/* A 2048-byte request from a page-aligned slot never crosses 4 KB. */
		before = ioread32(pci_pango->_pango_pci_driver._pci_status_io +
				  PANGO_DMA_DONE_COUNT_OFFSET);
		iowrite32(dma_cmd, pci_pango->_pango_pci_driver._pci_io + CMD_REG_OFFSET);
		iowrite32(lower_32_bits(target + offset),
			  pci_pango->_pango_pci_driver._pci_io + RW_ADDR_LO_OFFSET);
		ret = pango_wait_dma_done(before, deadline_ns);
		if (ret)
			return ret;
	}

	dma_rmb();
	frame_buffers.frame_sequence++;
	capture->frame_sequence = frame_buffers.frame_sequence;
	capture->bytes_transferred = frame_buffers.frame_bytes;
	capture->elapsed_ns = ktime_get_ns() - start_ns;
	return 0;
}

//static void StrToHex(char *pbDest, char *pbSrc, int nLen)
//{
//  char h1,h2;
//  char s1,s2;
//  int i;
// 
//    for (i=0; i<nLen/2; i++)
//    {
//        h1 = pbSrc[2*i];
//        h2 = pbSrc[2*i+1];
// 
//        s1 = toupper(h1) - 0x30; //toupper 转换为大写字母
//        if (s1 > 9)
//            s1 -= 7;
//        s2 = toupper(h2) - 0x30;
//        if (s2 > 9)
//            s2 -= 7;
// 
//        pbDest[i] = s1*16 + s2;
//    }
//}

/**************************************************************************
** 函数名称:    pango_cdev_ioctl
** 函数功能:    写数据，将应用层传递过来的数据先复制到内核中然后再对数据进行解析
** 输入参数:    *file：文件描述词
**          cmd：操作命令(cmd的最高位表示寄存器读写操作指令，0：读操作；1：写操作)
**          arg：读取数据宽度
** 输出参数:    无
** 返回参数:操作结果
****************************************************************************/
long pango_cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct PciPango *pci_pango = &pci_info;
	struct pango_frame_config frame_cfg;
	struct pango_frame_info frame_info;
	struct pango_frame_capture frame_capture;
	unsigned char dest_buf[100];
	unsigned int temp_data = 0;
	unsigned int i = 0;
	int ret = 0;
	if(down_interruptible(&pci_pango->_sem))
	{
		printk(KERN_ALERT "********* pango_cdev_read interruptible *********\n");
		return -ERESTARTSYS;
	}
	if (!op_dev || !pci_pango->_pango_pci_driver._pci_io ||
	    !pci_pango->_pango_pci_driver._pci_status_io) {
		ret = -ENODEV;
		goto ioctl_out;
	}
	switch(cmd)
	{
		case PANGO_FRAME_CONFIG_CMD:
			if (copy_from_user(&frame_cfg, (void __user *)arg, sizeof(frame_cfg))) {
				ret = -EFAULT;
				goto ioctl_out;
			}
			ret = pango_configure_frame_buffers(&frame_cfg);
			if (ret)
				goto ioctl_out;
			if (copy_to_user((void __user *)arg, &frame_cfg, sizeof(frame_cfg))) {
				ret = -EFAULT;
				goto ioctl_out;
			}
		break;

		case PANGO_FRAME_INFO_CMD:
			if (!frame_buffers.cpu_addr) {
				ret = -ENXIO;
				goto ioctl_out;
			}
			memset(&frame_info, 0, sizeof(frame_info));
			frame_info.abi_version = PANGO_FRAME_ABI_VERSION;
			frame_info.width = frame_buffers.width;
			frame_info.height = frame_buffers.height;
			frame_info.pixel_bytes = frame_buffers.pixel_bytes;
			frame_info.packet_bytes = frame_buffers.packet_bytes;
			frame_info.buffer_count = frame_buffers.buffer_count;
			frame_info.frame_bytes = frame_buffers.frame_bytes;
			frame_info.slot_bytes = frame_buffers.slot_bytes;
			frame_info.mmap_bytes = frame_buffers.mmap_bytes;
			frame_info.frames_captured = frame_buffers.frame_sequence;
			if (copy_to_user((void __user *)arg, &frame_info, sizeof(frame_info))) {
				ret = -EFAULT;
				goto ioctl_out;
			}
		break;

		case PANGO_FRAME_CAPTURE_CMD:
			if (copy_from_user(&frame_capture, (void __user *)arg,
					   sizeof(frame_capture))) {
				ret = -EFAULT;
				goto ioctl_out;
			}
			ret = pango_capture_frame(&frame_capture);
			if (ret)
				goto ioctl_out;
			if (copy_to_user((void __user *)arg, &frame_capture,
					 sizeof(frame_capture))) {
				ret = -EFAULT;
				goto ioctl_out;
			}
		break;

		case PCI_READ_DATA_CMD:	
			if (copy_from_user(&config_operation,(COMMAND_OPERATION*)arg, sizeof(COMMAND_OPERATION))){
				ret = -EFAULT;
				goto ioctl_out;
			}
			pci_read_config_dword(op_dev, config_operation.addr, &config_operation.data);
			if (copy_to_user((COMMAND_OPERATION*)arg, &config_operation, sizeof(COMMAND_OPERATION))){
				ret = -EFAULT;
				goto ioctl_out;
			}
		break;
		
		case PCI_WRITE_DATA_CMD:	
			if (copy_from_user(&config_operation,(COMMAND_OPERATION*)arg, sizeof(COMMAND_OPERATION))){
				ret = -EFAULT;
				goto ioctl_out;
			}
			pci_write_config_dword(op_dev, config_operation.addr, config_operation.data);
		break;
		
		case PCI_MAP_ADDR_CMD:
			spin_lock(&dma_info.addr_r.lock);
			if (copy_from_user(&dma_operation,(DMA_OPERATION*)arg, sizeof(DMA_OPERATION))){
				spin_unlock(&dma_info.addr_r.lock);
				ret = -EFAULT;
				goto ioctl_out;
			}
			//dma_info.addr_r.data_buf = pci_alloc_consistent(op_dev, dma_operation.current_len*4, &dma_info.addr_r.addr);
			//dma_info.addr_w.data_buf = pci_alloc_consistent(op_dev, dma_operation.current_len*4, &dma_info.addr_w.addr);  //修改前 可乐大盗
			dma_info.addr_r.data_buf = dma_alloc_coherent(&op_dev->dev, 
                                                 dma_operation.current_len*4, 
                                                 &dma_info.addr_r.addr, 
                                                 GFP_KERNEL);
    		dma_info.addr_w.data_buf = dma_alloc_coherent(&op_dev->dev,
                                                 dma_operation.current_len*4,
                                                 &dma_info.addr_w.addr,
                                                 GFP_KERNEL);

			dma_info.addr_r.addr_size = ((dma_info.addr_r.addr >> 32) > 0) ? 1 : 0;
			dma_info.addr_w.addr_size = ((dma_info.addr_w.addr >> 32) > 0) ? 1 : 0;
			dma_info.cmd.data.length = dma_operation.current_len - 1;
			dma_info.cmd.data.addr_type = dma_info.addr_r.addr_size;
			dma_info.addr_r.addr = dma_info.addr_r.addr + dma_operation.offset_addr;
			dma_info.addr_w.addr = dma_info.addr_w.addr + dma_operation.offset_addr;
			printk("addr_size = %d;     current_len = %d(dw)\n", (dma_info.addr_r.addr_size) ? 64 : 32, dma_operation.current_len);
			printk("dma_addr_r  = 0x%llx; offset_addr = 0x%x\n", dma_info.addr_r.addr - dma_operation.offset_addr, dma_operation.offset_addr);
			printk("dma_addr_w  = 0x%llx; offset_addr = 0x%x\n", dma_info.addr_w.addr - dma_operation.offset_addr, dma_operation.offset_addr);
			spin_unlock(&dma_info.addr_r.lock);
		break;
		
		case PCI_WRITE_TO_KERNEL_CMD:										
			spin_lock(&dma_info.addr_r.lock);
			if (copy_from_user(&dma_operation,(DMA_OPERATION*)arg, sizeof(DMA_OPERATION))){
				spin_unlock(&dma_info.addr_r.lock);
				ret = -EFAULT;
				goto ioctl_out;
			}
			memcpy(dma_info.addr_r.data_buf, dma_operation.data.write_buf, dma_operation.current_len*4);
			memset(dma_info.addr_w.data_buf, 0, dma_operation.current_len*4);
			spin_unlock(&dma_info.addr_r.lock);
		break;
		
		case PCI_DMA_READ_CMD:												/* CPU写数据 */
			spin_lock(&dma_info.addr_r.lock);
			dma_info.cmd.data.op_type = 0;									/* DMA读操作，DMA将数据读入到FPGA设备 */
			set_dma_w_r(dma_info.cmd.value, pci_pango);
			set_dma_addr(&dma_info.addr_r, pci_pango);
			spin_unlock(&dma_info.addr_r.lock);
		break;
		
		case PCI_DMA_WRITE_CMD:												/* CPU读数据 */
			spin_lock(&dma_info.addr_r.lock);
			dma_info.cmd.data.op_type = 1;									/* DMA写操作，DMA将数据写入到DDR */
			memset(dma_info.addr_w.data_buf, 0, dma_operation.current_len*4);
			set_dma_w_r(dma_info.cmd.value, pci_pango);
			set_dma_addr(&dma_info.addr_w, pci_pango);
			spin_unlock(&dma_info.addr_r.lock);
		break;
		
		case PCI_READ_FROM_KERNEL_CMD:										/* 将数据写入CPU缓存 */
			spin_lock(&dma_info.addr_r.lock);
			memcpy(dma_operation.data.read_buf, dma_info.addr_w.data_buf, dma_operation.current_len*4);
			if (copy_to_user((DMA_OPERATION*)arg, &dma_operation, sizeof(DMA_OPERATION))) {
				spin_unlock(&dma_info.addr_r.lock);
				ret = -EFAULT;
				goto ioctl_out;
			}
			spin_unlock(&dma_info.addr_r.lock);
		break;
		
		case PCI_UMAP_ADDR_CMD:
			spin_lock(&dma_info.addr_r.lock);
			//pci_free_consistent(op_dev, dma_operation.current_len*4, dma_info.addr_r.data_buf, dma_info.addr_r.addr);
			//pci_free_consistent(op_dev, dma_operation.current_len*4, dma_info.addr_w.data_buf, dma_info.addr_w.addr);  //修改前 可乐大盗
			dma_free_coherent(&op_dev->dev, dma_operation.current_len*4, dma_info.addr_r.data_buf, dma_info.addr_r.addr);
    		dma_free_coherent(&op_dev->dev, dma_operation.current_len*4, dma_info.addr_w.data_buf, dma_info.addr_w.addr);
			dma_info.addr_r.data_buf = NULL;
			dma_info.addr_w.data_buf = NULL;
			spin_unlock(&dma_info.addr_r.lock);
		break;

		case PCI_PERFORMANCE_START_CMD:
			spin_lock(&performance_config.addr.lock);
			if (copy_from_user(&performance_operation,(PERFORMANCE_OPERATION*)arg, sizeof(PERFORMANCE_OPERATION))){
				spin_unlock(&performance_config.addr.lock);
				ret = -EFAULT;
				goto ioctl_out;
			}
			//performance_config.addr.data_buf = pci_alloc_consistent(op_dev, DMA_MAX_PACKET_SIZE*10*2, &performance_config.addr.addr);  //修改前 可乐大盗
			performance_config.addr.data_buf = dma_alloc_coherent(&op_dev->dev,
                                                        DMA_MAX_PACKET_SIZE*10*2,
                                                        &performance_config.addr.addr,
                                                        GFP_KERNEL);
			for(i = 0; i < (DMA_MAX_PACKET_SIZE*10)/16; i++)
			{
				memset(dest_buf, 0, sizeof(dest_buf));
				temp_data = i;
				dest_buf[0] = (unsigned char)temp_data;
				dest_buf[1] = (unsigned char)(temp_data >> 8);
				dest_buf[2] = (unsigned char)(temp_data >> 16);
				dest_buf[3] = (unsigned char)(temp_data >> 24);
				memcpy(performance_config.addr.data_buf + i*16, dest_buf, 16);
			}

			performance_config.addr.addr_size = ((performance_config.addr.addr >> 32) > 0) ? 1 : 0;
			performance_config.cmd.data.length = performance_operation.current_len - 1;
			performance_config.cmd.data.addr_type = performance_config.addr.addr_size;
			performance_config.cmd.data.op_type = performance_operation.cmd;
			set_dma_w_r(performance_config.cmd.value, pci_pango);
			set_dma_addr(&performance_config.addr, pci_pango);
			printk("addr_size = %d;     current_len = %d(dw)\n", (performance_config.addr.addr_size) ? 64 : 32, performance_operation.current_len);
			printk("operation cmd = 0x%02x; dma_addr  = 0x%llx \n", performance_operation.cmd, performance_config.addr.addr);
			spin_unlock(&performance_config.addr.lock);
		break;

		case PCI_PERFORMANCE_END_CMD:
			spin_lock(&performance_config.addr.lock);
			if(!memcmp(performance_config.addr.data_buf, performance_config.addr.data_buf + DMA_MAX_PACKET_SIZE*10, DMA_MAX_PACKET_SIZE*10))
			{
				performance_operation.cmp_flag = 1;
			}
			else
			{
				performance_operation.cmp_flag = 0;
			}
			if (copy_to_user((PERFORMANCE_OPERATION*)arg, &performance_operation, sizeof(PERFORMANCE_OPERATION))){
				spin_unlock(&performance_config.addr.lock);
				ret = -EFAULT;
				goto ioctl_out;
			}
			//pci_free_consistent(op_dev, DMA_MAX_PACKET_SIZE*10*2, performance_config.addr.data_buf, performance_config.addr.addr);  //修改前 可乐大盗
			dma_free_coherent(&op_dev->dev, DMA_MAX_PACKET_SIZE*10*2, performance_config.addr.data_buf, performance_config.addr.addr);
			performance_config.addr.data_buf = NULL;
			spin_unlock(&performance_config.addr.lock);
		break;

		default:
			ret = -ENOTTY;
		break;
	}
	
ioctl_out:
	up (&pci_pango->_sem);
    return ret;
}


static void pango_frame_vma_open(struct vm_area_struct *vma)
{
	atomic_inc(&frame_buffers.mmap_count);
}

static void pango_frame_vma_close(struct vm_area_struct *vma)
{
	atomic_dec(&frame_buffers.mmap_count);
}

static const struct vm_operations_struct pango_frame_vm_ops = {
	.open = pango_frame_vma_open,
	.close = pango_frame_vma_close,
};

int pango_cdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	size_t requested = vma->vm_end - vma->vm_start;
	struct PciPango *pci_pango = &pci_info;
	int ret;

	if (down_interruptible(&pci_pango->_sem))
		return -ERESTARTSYS;
	if (!op_dev || !frame_buffers.cpu_addr) {
		ret = -ENXIO;
		goto mmap_out;
	}
	if (vma->vm_pgoff != 0 || requested != frame_buffers.mmap_bytes) {
		ret = -EINVAL;
		goto mmap_out;
	}
	if (!(vma->vm_flags & VM_SHARED)) {
		ret = -EINVAL;
		goto mmap_out;
	}

	ret = dma_mmap_coherent(&op_dev->dev, vma, frame_buffers.cpu_addr,
				frame_buffers.dma_addr, frame_buffers.mmap_bytes);
	if (ret)
		goto mmap_out;

	vma->vm_ops = &pango_frame_vm_ops;
	vma->vm_private_data = &frame_buffers;
	pango_frame_vma_open(vma);

mmap_out:
	up(&pci_pango->_sem);
	return ret;
}

int pango_cdev_open(struct inode *inode, struct file *filp)
{
	if (!op_dev || !pci_info._pango_pci_driver._pci_io ||
	    !pci_info._pango_pci_driver._pci_status_io)
		return -ENODEV;

	return 0;
}

int pango_cdev_release(struct inode *inode, struct file *filp)
{
	LOG("pango_cdev_release.\n");
	
	return 0;
}


//修改前 可乐大盗
// static int set_dma_mask(struct pci_dev *pdev)
// {
//     if(!pdev)
//     {
//         LOG("Invalid pdev\n");
//         return -EINVAL;
//     }

//     LOG("sizeof(dma_addr_t) == %ld\n", sizeof(dma_addr_t));

//     /* 64-bit addressing capability for XDMA? */
//     if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
//         /* query for DMA transfer */
//         /* @see Documentation/DMA-mapping.txt */
//         LOG("pci_set_dma_mask()\n");
//         /* use 64-bit DMA */
//         LOG("Using a 64-bit DMA mask.\n");
//         /* use 32-bit DMA for descriptors */
//         pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
//         /* use 64-bit DMA, 32-bit for consistent */
//     } else if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
//         LOG("Could not set 64-bit DMA mask.\n");
//         pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
//         /* use 32-bit DMA */
//         LOG("Using a 32-bit DMA mask.\n");
//     } else {
//         LOG("No suitable DMA possible.\n");
//         return -EINVAL;
//     }

//     return 0;
// }

static int set_dma_mask(struct pci_dev *pdev)
{
    // if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(64))) {
    //     dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64));
    //     printk("Using 64-bit DMA mask\n");
    // } 
	int ret;

	ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "No suitable 32-bit streaming DMA mask\n");
		return ret;
	}
	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "No suitable 32-bit coherent DMA mask\n");
		return ret;
	}
	dev_info(&pdev->dev, "Using 32-bit DMA mask\n");
	return 0;
}



static void pci_keep_intx_enabled(struct pci_dev *pdev)
{
    unsigned short pcmd, pcmd_new;

    pci_read_config_word(pdev, PCI_COMMAND, &pcmd);
    pcmd_new = pcmd & ~PCI_COMMAND_INTX_DISABLE;
    if (pcmd_new != pcmd) {
        LOG("%s: clear INTX_DISABLE, 0x%x -> 0x%x.\n",
            dev_name(&pdev->dev), pcmd, pcmd_new);
        pci_write_config_word(pdev, PCI_COMMAND, pcmd_new);
    }

    LOG("pci_keep_intx_enabled.\n");
}

///////////////////////////////////////////////////////////////////////

int pci_driver_probe(struct pci_dev *dev, const struct pci_device_id *device_id)
{
	int result = 0;
	unsigned long bar_address = 0;
	struct PangoPciDriver *driver = &pci_info._pango_pci_driver;
	LOG("pci_driver_probe.\n");
	
    LOG("dev vendor : 0x%x, device : 0x%x\n", dev->vendor, dev->device);
	
    result = pci_enable_device(dev);									//PCIE设备使能，初始化memory和IO类型的BAR
    LOG("pci_enable_device result : %d\n", result);
	
	if(result != 0)
	{
		goto fail_enable_device;
	}
	//辅助函数用于检查总线是否可以接收给定大小的总线地址(mask)，如果可以，则通知总线层给定的外围设备将使用该大小的总线地址。
	result = set_dma_mask(dev);
	
	if(result != 0)
	{
		goto fail_set_dma_mask;
	}
	
	ReadConfig(dev);
	pci_set_master(dev);												//设定设备工作在总线主设备模式
	// BAR1 carries DMA commands; BAR0 exposes the FPGA completion counter.
    result = pci_request_region(dev, driver->_pci_bar, PCI_DRIVER_DEV_NAME);
    LOG("pci_request_region result : %d\n", result);
	
	if(result != 0)
	{
		goto fail_request_region;
	}
	
	driver->_pci_io_size = pci_resource_len(dev, driver->_pci_bar);
	bar_address = pci_resource_start(dev, driver->_pci_bar);
	driver->_pci_io = ioremap(bar_address, driver->_pci_io_size);
	if (!driver->_pci_io) {
		result = -ENOMEM;
		goto fail_map_command_bar;
	}

	result = pci_request_region(dev, driver->_pci_status_bar, PCI_DRIVER_DEV_NAME "-status");
	if (result)
		goto fail_request_status_bar;
	driver->_pci_status_io_size = pci_resource_len(dev, driver->_pci_status_bar);
	if (driver->_pci_status_io_size < PANGO_DMA_STATUS_SIGNATURE_OFFSET + sizeof(u32)) {
		result = -ENODEV;
		goto fail_status_bar_size;
	}
	bar_address = pci_resource_start(dev, driver->_pci_status_bar);
	driver->_pci_status_io = ioremap(bar_address, driver->_pci_status_io_size);
	if (!driver->_pci_status_io) {
		result = -ENOMEM;
		goto fail_map_status_bar;
	}
	if (ioread32(driver->_pci_status_io + PANGO_DMA_STATUS_SIGNATURE_OFFSET) !=
	    PANGO_DMA_STATUS_SIGNATURE) {
		dev_err(&dev->dev, "FPGA image lacks completion-status ABI; program the new bitstream\n");
		result = -EPROTO;
		goto fail_status_signature;
	}
	op_dev = dev;

	atomic_set(&frame_buffers.mmap_count, 0);
	LOG("BAR%d command=%p, BAR%d status=%p, completion=0x%08x\n",
	    driver->_pci_bar, driver->_pci_io, driver->_pci_status_bar,
	    driver->_pci_status_io,
	    ioread32(driver->_pci_status_io + PANGO_DMA_DONE_COUNT_OFFSET));
    
	pci_keep_intx_enabled(dev);
	
	return 0;

fail_status_signature:
	iounmap(driver->_pci_status_io);
	driver->_pci_status_io = NULL;
fail_map_status_bar:
fail_status_bar_size:
	pci_release_region(dev, driver->_pci_status_bar);
fail_request_status_bar:
	iounmap(driver->_pci_io);
	driver->_pci_io = NULL;
fail_map_command_bar:
	pci_release_region(dev, driver->_pci_bar);
	
fail_request_region:
    pci_clear_master(dev);
    LOG("pci_clear_master\n");
	
fail_set_dma_mask:
	op_dev = NULL;
    pci_disable_device(dev);
    LOG("pci_disable_device\n");

fail_enable_device:
	
	return result;
}

void pci_driver_remove(struct pci_dev *dev)
{
	struct PangoPciDriver *driver = &pci_info._pango_pci_driver;

	pango_free_frame_buffers();
	if (driver->_pci_status_io) {
		iounmap(driver->_pci_status_io);
		driver->_pci_status_io = NULL;
	}
	pci_release_region(dev, driver->_pci_status_bar);
	if (driver->_pci_io) {
		iounmap(driver->_pci_io);
		driver->_pci_io = NULL;
	}
	pci_release_region(dev, driver->_pci_bar);
	op_dev = NULL;
	
    pci_clear_master(dev);
    LOG("pci_clear_master\n");
	
    pci_disable_device(dev);
    LOG("pci_disable_device\n");
	
	LOG("pci_driver_remove.\n");
}

///////////////////////////////////////////////////////////////////////

int init_pango_cdev(struct cdev *pango_cdev)
{
	int result = 0;
	
	LOG("init_pango_cdev.\n");
	
	result = alloc_chrdev_region(&pci_dev_info._dev, pci_dev_info._dev_firstminor, pci_dev_info._dev_count, pci_dev_info._dev_name);
	
	LOG("*********alloc_chrdev_region, result : %d\n", result);
	
	if(result < 0)
	{
		return result;
	}
	
	cdev_init(pango_cdev, &pango_cdev_fops);
	
	pango_cdev->owner = THIS_MODULE;
	pango_cdev->ops   = &pango_cdev_fops;
	
	result = cdev_add(pango_cdev, pci_dev_info._dev, pci_dev_info._dev_count);							//关联设备号
	
	LOG("init pango_cdev result : %d\n", result);
	
	return result;
}

void exit_pango_cdev(struct cdev *pango_cdev)
{
	cdev_del(pango_cdev);
	
	unregister_chrdev_region(pci_dev_info._dev, pci_dev_info._dev_count);
	
	LOG("exit cdev.\n");
}

int init_pango_pci_driver(struct pci_driver *pango_pci_driver)
{
	int result = 0;
	
	LOG("init_pango_pci_driver.\n");
	
	result = pci_register_driver(pango_pci_driver);						//注册PCI驱动
	
	LOG("*********pci_register_driver, result : %d\n", result);
	
	return result;
}

void exit_pango_pci_driver(struct pci_driver *pango_pci_driver)
{
	pci_unregister_driver(pango_pci_driver);
	
	LOG("exit pci pango driver.\n");
}

// class_create动态创建设备的逻辑类，并完成部分字段的初始化，然后将其添加到内核中。创建的逻辑类位于/sys/class/。
//修改前 可乐大盗
// int init_pango_cdev_class(struct class *pango_cdev_class)
// {
// 	int result = -1;
	
// 	LOG("init_pango_cdev_class.\n");
	
// 	pango_cdev_class = class_create(THIS_MODULE, pci_dev_info._dev_name);
	
// #ifndef NDEBUG
// 	(!!pango_cdev_class) ? LOG("class create success.\n") : LOG("class create failed.\n");
// #endif
	
// 	if(pango_cdev_class)
// 	{
// 		LOG("create device.\n");
// 		device_create(pango_cdev_class, NULL, pci_dev_info._dev, NULL, "%s", pci_dev_info._dev_name);
		
// 		result = 0;
// 	}
	
// 	return result;
// }

int init_pango_cdev_class(void)
{
    struct device *device;

    pci_info._cdev_class = class_create(THIS_MODULE, pci_dev_info._dev_name);
    if (IS_ERR(pci_info._cdev_class)) {
        int ret = PTR_ERR(pci_info._cdev_class);
        pci_info._cdev_class = NULL;
        return ret;
    }
    
    device = device_create(pci_info._cdev_class, NULL, pci_dev_info._dev,
                           NULL, "%s", pci_dev_info._dev_name);
    if (IS_ERR(device)) {
        int ret = PTR_ERR(device);
        class_destroy(pci_info._cdev_class);
        pci_info._cdev_class = NULL;
        return ret;
    }
    return 0;
}

void exit_pango_cdev_class(struct class *pango_cdev_class)
{
	if (!pango_cdev_class)
		return;
	LOG("device_destroy\n");
	device_destroy(pango_cdev_class, pci_dev_info._dev);
	
	LOG("class_destroy\n");
	class_destroy(pango_cdev_class);
	
	pango_cdev_class = NULL;
	
	LOG("exit cdev class.\n");
}

///////////////////////////////////////////////////////////////////////

int __init init_pci_pango(void)
{
	int result = 0;
	
	LOG("init_pci_pango.\n");
	sema_init(&pci_info._sem, 1);
	spin_lock_init(&dma_info.addr_r.lock);
	spin_lock_init(&dma_info.addr_w.lock);
	spin_lock_init(&performance_config.addr.lock);
	
	result = init_pango_cdev(&pci_info._cdev);						//申请主设备号
	LOG("init cdev result : %d\n", result);
	if (result)
		return result;
	
	result = init_pango_pci_driver(&pci_info._pango_pci_driver._pci_driver);
	LOG("init pci result : %d\n", result);
	if (result)
		goto fail_pci_driver;

	result = init_pango_cdev_class();
	LOG("init cdev class result : %d\n", result);
	if (result)
		goto fail_cdev_class;
	
	return 0;

fail_cdev_class:
	exit_pango_pci_driver(&pci_info._pango_pci_driver._pci_driver);
fail_pci_driver:
	exit_pango_cdev(&pci_info._cdev);
	return result;
}

void __exit exit_pci_pango(void)
{
	exit_pango_cdev_class(pci_info._cdev_class);
	
	exit_pango_pci_driver(&pci_info._pango_pci_driver._pci_driver);
	
	exit_pango_cdev(&pci_info._cdev);
	
	LOG("exit_pci_pango.\n");
}

module_init(init_pci_pango);
module_exit(exit_pci_pango);


/* 添加模块描述信息 */
/* 添加模块描述信息 */

MODULE_AUTHOR("Pango, lxg.");                                                          /* 作者信息 */
MODULE_DESCRIPTION("Pango pci driver");                                                /* 模块介绍信息 */
MODULE_LICENSE("GPL v2");                                                              /* 模块许可证 */
MODULE_ALIAS("pango pci driver");                                                      /* 模块的别名信息 */


