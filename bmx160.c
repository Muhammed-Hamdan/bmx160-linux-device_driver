#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h> 
#include <linux/sysfs.h>

#include <asm/delay.h>

#define BMX160_IRQ_LOG         0

#define BMX160_ADDRESS      0x68
#define BMX160_CHIP_ID      0xD8
#define BMX160_FRAME_SIZE   32

#define REG_CHIP_ID             0x00
#define REG_PMU_STAT            0x03
#define REG_MAG_DATA_START      0x04
#define REG_GYRO_DATA_START     0x0C
#define REG_ACC_DATA_START      0x12
#define REG_ACC_CONF            0x40
#define REG_ACC_RANGE           0x41
#define GYRO_CONF               0x42
#define GYRO_RANGE              0x43
#define REG_INT_STATUS          0x1C
#define REG_INT_EN              0x50
#define REG_CMD                 0x7E

#define CMD_SET_ACC_NORMAL_MODE     0x11
#define CMD_SET_GYRO_NORMAL_MODE    0x15
#define CMD_SET_GYRO_FAST_MODE      0x17

#define DELAY_MS_GYRO_MODE_SHIFT        85
#define DELAY_MS_ACC_MODE_SHIFT         5
#define DELAY_US_REG_WRITE              5

#define bmx160_reg_read(client, address, count, data) i2c_smbus_read_i2c_block_data(client, address, count, data) 
#define bmx160_reg_write(client, address, count, data) i2c_smbus_write_i2c_block_data(client, address, count, data) 

static inline int bmx160_issue_cmd(struct i2c_client *client, uint8_t cmd){
    
    return bmx160_reg_write(client, REG_CMD, 1, &cmd);
}

struct interrupts_t {
    uint32_t pin_value, pin_trigger;
};

struct bmx160_t {
    struct i2c_client *client;
    uint64_t irq_counter;
    struct interrupts_t intr1_pin;
    uint8_t frame[BMX160_FRAME_SIZE];
};

irqreturn_t bmx160_hard_irq(int irq_no, void *dev_id){
    ((struct bmx160_t*)dev_id)->irq_counter += 1;
    return IRQ_WAKE_THREAD;
}

irqreturn_t bmx160_threaded_irq(int irq_no, void *dev_id){
    struct bmx160_t *bmx160 = (struct bmx160_t *)dev_id;
    uint32_t time;
    int err;

    err = bmx160_reg_read(bmx160->client, REG_GYRO_DATA_START, 15, bmx160->frame);
    if(err < 0){
        pr_info("bmx160: I2C Error %d at %lld", err, bmx160->irq_counter--);
    }
    else{
        time  = bmx160->frame[14];
        time  = (time << 8) | bmx160->frame[13];
        time  = (time << 8) | bmx160->frame[12];
        #if BMX160_IRQ_LOG
        pr_info("bmx160: Frame number %lld is read at time 0x%08x", bmx160->irq_counter, time);
        #endif
    }

    return IRQ_HANDLED;
}

static int bmx160_irq_init(struct i2c_client *client){

    int lenp;
    struct bmx160_t *bmx160 = (struct bmx160_t *)i2c_get_clientdata(client);
    struct interrupts_t *interrupt = &bmx160->intr1_pin;

    pr_info("bmx160: Inside %s...", __func__);

    lenp = of_property_read_variable_u32_array(client->dev.of_node, "interrupts", (uint32_t *)interrupt, 0, 2);
    pr_info("bmx160: value of lenp: %d", lenp);
    if(lenp == 2){
        pr_info("bmx160: value of interrupts: %d %d", interrupt->pin_value, interrupt->pin_trigger);
    }

    if(gpio_is_valid(interrupt->pin_value) == false){
        pr_err("GPIO %d is not valid\n", interrupt->pin_value);
        goto r_irq_init;
    }

    //Requesting the GPIO
    if(gpio_request(interrupt->pin_value,"INT1_PIN") < 0){
        pr_err("ERROR: GPIO %d request\n", interrupt->pin_value);
        goto r_irq_init_gpio;
    }
    
    //configure the GPIO as input
    gpio_direction_input(interrupt->pin_value);

    pr_info("bmx160: gpio_to_irq(pin) value - %d and client->irq value - %d", gpio_to_irq(interrupt->pin_value), client->irq);
    
    if(client->irq != gpio_to_irq(interrupt->pin_value))
        client->irq = gpio_to_irq(interrupt->pin_value);

    if(devm_request_threaded_irq(&client->dev, client->irq, bmx160_hard_irq, bmx160_threaded_irq, IRQF_ONESHOT | IRQF_TRIGGER_RISING, "bmx160", (void *)bmx160)) {
        pr_info("bmx160: Unable to register IRQ: %d", client->irq);
        goto r_irq_init_gpio;
    }

    pr_info("bmx160: Exit %s successfully!", __func__);

    return 0;

    r_irq_init_gpio:
        gpio_free(interrupt->pin_value);
    r_irq_init:
        return -1;
}

static int bmx160_init(struct i2c_client *client){
    
    int ret;
    uint8_t chip_id;
    uint8_t buffer[16];
    
    /* Check slave address given in device tree*/
    pr_info("bmx160: Inside %s...", __func__);
    if(client->addr != BMX160_ADDRESS){
        pr_info("bmx160: Device address specified is incorrect");
        return -1;
    }

    /* Read chip ID as sanity check for I2C communication*/
    ret = bmx160_reg_read(client, REG_CHIP_ID, 1, &chip_id);
    if(ret < 0){
        pr_info("bmx160: Unable to read chip ID");
        goto r_i2c_err;
    }
    pr_info("bmx160: Chip ID value - Expected %.2x Received %.2x", BMX160_CHIP_ID, chip_id);

    /* Set Accel and Gyro to normal mode*/
    ret = bmx160_issue_cmd(client, CMD_SET_GYRO_NORMAL_MODE);
    if(ret < 0)
        goto r_i2c_err;
    msleep(DELAY_MS_GYRO_MODE_SHIFT);
    ret = bmx160_issue_cmd(client, CMD_SET_ACC_NORMAL_MODE);
    if(ret < 0)
        goto r_i2c_err;
    msleep(DELAY_MS_ACC_MODE_SHIFT);

    /* Check if Accel and Gyro set to normal mode*/
    ret = bmx160_reg_read(client, REG_PMU_STAT, 1, buffer);
    if(ret < 0){
        pr_info("bmx160: Unable to Read PMU_STAT!");
        goto r_i2c_err;
    }
    pr_info("bmx:160: PMU Status: Target:%2x Actual:%2x", 0x14, buffer[0]);

    /* 
    Set Accelerometer and Gyrometer configuration
    Accel:
        ODR = 200 Hz
        Range = +-2g
    Gyro:
        ODR = 200 Hz
        Range = +-2000 deg/sec
    */
   ret = bmx160_reg_read(client, REG_ACC_CONF, 4, buffer);
   if(ret < 0){
        pr_info("bmx160: Unable to read ACC_CONF!");
        goto r_i2c_err;
   }
   buffer[0] = (buffer[0] & (~0x0F)) | 0x09;
   buffer[2] = (buffer[2] & (~0x0F)) | 0x09;
   ret = bmx160_reg_write(client, REG_ACC_CONF, 4, buffer);
   if(ret < 0){
        pr_info("bmx160: Unable to write ACC_CONF!");
        goto r_i2c_err;
   }
   udelay(DELAY_US_REG_WRITE);

   /* Configuring to generate interrupts */
   ret = bmx160_reg_read(client, REG_INT_EN, 8, buffer);
   if(ret < 0){
        pr_info("bmx160: Unable to read INT_EN!");
        goto r_i2c_err;
   }
   buffer[1] = (buffer[1] & (~0x10)) | 0x10;
   buffer[3] = (buffer[3] & (~0x0F)) | 0x0A;
   buffer[4] = (buffer[4] & (~0x0F)) | 0x00;
   buffer[6] = (buffer[6] & (~0x80)) | 0x80;
   ret = bmx160_reg_write(client, REG_INT_EN, 8, buffer);
   if(ret < 0){
        pr_info("bmx160: Unable to write INT_EN!");
        goto r_i2c_err;
   }
   udelay(DELAY_US_REG_WRITE);

    pr_info("bmx160: Exit %s successfully!", __func__);

    return 0;

    r_i2c_err:
        return -1;
}

ssize_t bmx160_show_raw_frame(struct device *dev, struct device_attribute *attr, char *buf) {
    
    struct bmx160_t *bmx160 = (struct bmx160_t *)dev->driver_data;

    /* 
        Don't use copy_to_user() in procfs/sysfs functions:
        copy_to_user() is done internally by these file systems (check fs/procfs/generic.c::proc_file_read())
        So buf is not a user virtual-address but a kernel virtual-address
     */
    memcpy(buf, bmx160->frame, BMX160_FRAME_SIZE);
        
    return BMX160_FRAME_SIZE;
}

static DEVICE_ATTR(raw_frame, S_IRUGO, bmx160_show_raw_frame, NULL);

static int bmx160_probe(struct i2c_client *client, const struct i2c_device_id *id){

    struct bmx160_t *bmx160;
    int ret;

    pr_info("bmx160: Inside %s...", __func__);

    bmx160 = devm_kzalloc(&client->dev, sizeof(struct bmx160_t), GFP_KERNEL);
    if(!bmx160)
        return -ENOMEM;
    bmx160->client = client;

    i2c_set_clientdata(client, (void *)bmx160);
    
    ret = bmx160_init(client);
    if(ret < 0){
        pr_info("bmx160: Init Failed!");
        goto r_probe_init;
    }

    ret = bmx160_irq_init(client);
    if(ret < 0){
        pr_info("bmx160: IRQ Init Failed");
        goto r_probe_irq;
    }

    /* Configure sysfs interface */
    ret = device_create_file(&client->dev, &dev_attr_raw_frame);
    if(ret < 0){
        pr_info("Unable to create sysfs file: dev_attr_raw_frame");
        goto r_probe_sysfs;
    }

    pr_info("bmx160: Exit %s successfully!", __func__);

    return 0;

    r_probe_sysfs:
    r_probe_irq:
    r_probe_init:  
        return -1;
}

static void bmx160_remove(struct i2c_client *client){
    
    struct bmx160_t *bmx160 = (struct bmx160_t *)i2c_get_clientdata(client);

    pr_info("bmx160: Inside %s...", __func__);

    device_remove_file(&client->dev, &dev_attr_raw_frame);
    devm_free_irq(&client->dev, client->irq, NULL);
    gpio_free(bmx160->intr1_pin.pin_value);
    devm_kfree(&client->dev, (void *)bmx160);

    pr_info("bmx160: Exit %s successfully!", __func__);
}

static const struct i2c_device_id bmx160_id[] = {
    { "bmx160", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bmx160_id);

static const __maybe_unused struct of_device_id bmx160_of_match[] = {
    {.compatible = "bosch,bmx160"},
    { }
};
MODULE_DEVICE_TABLE(of, bmx160_of_match);

struct i2c_driver bmx160_driver = {
    .driver = {
        .name = "bmx160",
        .of_match_table = bmx160_of_match
    },
    .probe = bmx160_probe,
    .remove = bmx160_remove,
    .id_table = bmx160_id
};

module_i2c_driver(bmx160_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hamdan");
MODULE_DESCRIPTION("Driver to communicate with bmx160");
MODULE_VERSION("1.00");
