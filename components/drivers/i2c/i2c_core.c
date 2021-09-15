/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2012-04-25     weety         first version
 */

#include <rtdevice.h>

#define DBG_TAG               "I2C"
#ifdef RT_I2C_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

rt_err_t rt_i2c_bus_device_register(struct rt_i2c_bus_device *bus,
                                    const char               *bus_name)
{
    rt_err_t res = RT_EOK;

    rt_mutex_init(&bus->lock, "i2c_bus_lock", RT_IPC_FLAG_FIFO);

    if (bus->timeout == 0) bus->timeout = RT_TICK_PER_SECOND;

    res = rt_i2c_bus_device_device_init(bus, bus_name);

    LOG_I("I2C bus [%s] registered", bus_name);

    return res;
}

struct rt_i2c_bus_device *rt_i2c_bus_device_find(const char *bus_name)
{
    struct rt_i2c_bus_device *bus;
    rt_device_t dev = rt_device_find(bus_name);
    if (dev == RT_NULL || dev->type != RT_Device_Class_I2CBUS)
    {
        LOG_E("I2C bus %s not exist", bus_name);

        return RT_NULL;
    }

    bus = (struct rt_i2c_bus_device *)dev->user_data;

    return bus;
}

rt_size_t rt_i2c_transfer(struct rt_i2c_bus_device *bus,
                          struct rt_i2c_msg         msgs[],
                          rt_uint32_t               num)
{
    rt_size_t ret;

    if (bus->ops->master_xfer)
    {
#ifdef RT_I2C_DEBUG
        for (ret = 0; ret < num; ret++)
        {
            LOG_D("msgs[%d] %c, addr=0x%02x, len=%d", ret,
                  (msgs[ret].flags & RT_I2C_RD) ? 'R' : 'W',
                  msgs[ret].addr, msgs[ret].len);
        }
#endif

        rt_mutex_take(&bus->lock, RT_WAITING_FOREVER);
        ret = bus->ops->master_xfer(bus, msgs, num);
        rt_mutex_release(&bus->lock);

        return ret;
    }
    else
    {
        LOG_E("I2C bus operation not supported");

        return 0;
    }
}

rt_size_t rt_i2c_master_send(struct rt_i2c_bus_device *bus,
                             rt_uint16_t               addr,
                             rt_uint16_t               flags,
                             const rt_uint8_t         *buf,
                             rt_uint32_t               count)
{
    rt_err_t ret;
    struct rt_i2c_msg msg;

    msg.addr  = addr;
    msg.flags = flags;
    msg.len   = count;
    msg.buf   = (rt_uint8_t *)buf;

    ret = rt_i2c_transfer(bus, &msg, 1);

    return (ret > 0) ? count : ret;
}

rt_size_t rt_i2c_master_recv(struct rt_i2c_bus_device *bus,
                             rt_uint16_t               addr,
                             rt_uint16_t               flags,
                             rt_uint8_t               *buf,
                             rt_uint32_t               count)
{
    rt_err_t ret;
    struct rt_i2c_msg msg;
    RT_ASSERT(bus != RT_NULL);

    msg.addr   = addr;
    msg.flags  = flags | RT_I2C_RD;
    msg.len    = count;
    msg.buf    = buf;

    ret = rt_i2c_transfer(bus, &msg, 1);

    return (ret > 0) ? count : ret;
}

int rt_i2c_core_init(void)
{
    return 0;
}
INIT_COMPONENT_EXPORT(rt_i2c_core_init);


void i2c_scan(int argc, void **argv)
{
	struct rt_i2c_bus_device *i2c_bus;
	
	if (argc != 2)
		return;
	
	i2c_bus = rt_i2c_bus_device_find(argv[1]);
	if(i2c_bus == RT_NULL)
	{
		rt_kprintf("can not open i2c bus:%s\r\n",argv[1]);
		return;
	}
	
	struct rt_i2c_msg msgs;
	rt_uint8_t buf[3] = {0};
	for (uint8_t addr=0x08;addr<=0x77;addr++)
	{
		buf[0] = 0x0; //cmd
		msgs.addr = addr;
		msgs.flags = RT_I2C_WR;
		msgs.buf = buf;
		msgs.len = 1;
		
		if (rt_i2c_transfer(i2c_bus, &msgs, 1) != 0)
		{
			rt_kprintf("Found: 0x%x on %s\r\n",addr,argv[1]);
		}
	}
	
}
MSH_CMD_EXPORT(i2c_scan, i2c scan);

void i2c_recv(int argc, void **argv)
{
	struct rt_i2c_bus_device *i2c_bus;
	
	if (argc != 4)
		return;
	
	i2c_bus = rt_i2c_bus_device_find(argv[1]);
	if(i2c_bus == RT_NULL)
	{
		rt_kprintf("can not open i2c bus:%s\r\n",argv[1]);
		return;
	}
	
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[3] = {0};
	rt_uint8_t recv = 0;
	uint8_t addr = atoi(argv[2]);
	uint8_t reg_addr = atoi(argv[3]);
	{
		buf[0] = reg_addr; //cmd
		msgs[0].addr = addr;
		msgs[0].flags = RT_I2C_WR;
		msgs[0].buf = buf;
		msgs[0].len = 1;
	
		msgs[1].addr = addr;
		msgs[1].flags = RT_I2C_RD|RT_I2C_NO_START;
		msgs[1].buf = &recv;
		msgs[1].len = 1;
		
		if (rt_i2c_transfer(i2c_bus, msgs, 2) != 0)
		{
			rt_kprintf("Read: 0x%x from 0x%x\r\n",recv,addr);
		}
		else
		{
			rt_kprintf("No ack from 0x%x\r\n",addr);
		}
	}
	
}
MSH_CMD_EXPORT(i2c_recv, i2c recieve);

void i2c_send(int argc, void **argv)
{
	struct rt_i2c_bus_device *i2c_bus;
	
	if (argc != 5)
		return;
	
	i2c_bus = rt_i2c_bus_device_find(argv[1]);
	if(i2c_bus == RT_NULL)
	{
		rt_kprintf("can not open i2c bus:%s\r\n",argv[1]);
		return;
	}
	
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[3] = {0};

	uint8_t addr = atoi(argv[2]);
	uint8_t reg_addr = atoi(argv[3]);
	uint8_t value = atoi(argv[4]);
	{
		buf[0] = reg_addr; //cmd
		buf[1] = value;
		msgs[0].addr = addr;
		msgs[0].flags = RT_I2C_WR;
		msgs[0].buf = buf;
		msgs[0].len = 2;
	
		if (rt_i2c_transfer(i2c_bus, msgs, 1) != 0)
		{
			rt_kprintf("Send: 0x%x to 0x%x:0x%x\r\n",value,reg_addr,addr);
		}
		else
		{
			rt_kprintf("No ack from 0x%x\r\n",addr);
		}
	}
	
}
MSH_CMD_EXPORT(i2c_send, i2c send);
