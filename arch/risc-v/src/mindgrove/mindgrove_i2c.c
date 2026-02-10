
#include <nuttx/i2c/i2c_master.h>
#include "mindgrove_i2c.h"
#include "riscv_internal.h"
#include <stdio.h>
struct i2c_msg_s __i2c_msg_probe;
/* I2C ops callbacks */

static int mg_i2c_transfer(FAR struct i2c_master_s *dev,
                           FAR struct i2c_msg_s *msgs,
                           int count);

static int mg_i2c_setup(FAR struct i2c_master_s *dev);

static int mg_i2c_shutdown(FAR struct i2c_master_s *dev);

#ifdef CONFIG_I2C_RESET
static int mg_i2c_reset(FAR struct i2c_master_s *dev);
#endif

struct mg_i2c_priv_s
{
    struct i2c_master_s dev; /* must be first */
    uintptr_t base;
    uint32_t frequency;
    bool initialized;
};

static const struct i2c_ops_s g_mg_i2c_ops =
    {
        .transfer = mg_i2c_transfer,
        .setup = mg_i2c_setup,
        .shutdown = mg_i2c_shutdown,
#ifdef CONFIG_I2C_RESET
        .reset = mg_i2c_reset,
#endif
};

#ifdef CONFIG_MINDGROVE_I2C0
static struct mg_i2c_priv_s g_i2c0_priv;
#endif

#ifdef CONFIG_MINDGROVE_I2C1
static struct mg_i2c_priv_s g_i2c1_priv;
#endif

FAR struct i2c_master_s *mg_i2c_initialize(int bus)
{
    FAR struct mg_i2c_priv_s *priv;

    /* Select I2C instance */
    switch (bus)
    {
#ifdef CONFIG_MINDGROVE_I2C0
    case 0:
        priv = &g_i2c0_priv;
        priv->base = MG_I2C0_BASE;
        break;
#endif

#ifdef CONFIG_MINDGROVE_I2C1
    case 1:
        priv = &g_i2c1_priv;
        priv->base = MG_I2C1_BASE;
        break;
#endif

    default:
        return NULL;
    }

    /* Initialize private structure */
    priv->dev.ops = &g_mg_i2c_ops;
    priv->frequency = 100000; /* Default: 100 kHz */
    priv->initialized = false;

    return &priv->dev;
}

static int mg_i2c_setup(FAR struct i2c_master_s *dev)
{
    FAR struct mg_i2c_priv_s *priv = (FAR struct mg_i2c_priv_s *)dev;
    uint32_t freq = priv->frequency; /* default bus speed */

    /* Reset I2C status (CTRL_PIN) */
    putreg8(MG_I2C_CTRL_PIN,
            priv->base + MG_I2C_CTRL_OFFSET);

    /* Program prescaler */
    putreg8(MG_I2C_PRESCALER,
            priv->base + MG_I2C_S2_OFFSET);

    /* Compute SCL divider */
    uint64_t scl_div =
        CLOCK_FREQUENCY_BASE /
        (2U * (MG_I2C_PRESCALER + 1U) * (uint64_t)freq);

    putreg32((uint8_t)(scl_div & MG_I2C_SCL_MASK),
             priv->base + MG_I2C_SCL_OFFSET);

    /* Enable serial interface (idle state) */
    putreg8(MG_I2C_IDLE,
            priv->base + MG_I2C_CTRL_OFFSET);

    priv->initialized = true;
    return 0;
}

static int mg_i2c_transfer(struct i2c_master_s *dev,
                           struct i2c_msg_s *msgs,
                           int count)
{
    struct mg_i2c_priv_s *priv = (struct mg_i2c_priv_s *)dev;
   
    for (int j = 0; j < count; j++)
    {
        struct i2c_msg_s *msgi = &msgs[j];
        bool read = (msgi->flags & I2C_M_READ);

        if (!read)
        {
            if ((msgi->flags & I2C_M_NOSTART) == 0U)
            {
                // wait till bus is free
                while ((getreg8(priv->base + MG_I2C_STATUS_OFFSET) &
                        MG_I2C_STATUS_BB) == 0)
                {
                }
            }
            // write data in data register
            putreg8(msgi->addr << 1U, priv->base + MG_I2C_S0_OFFSET);
            if ((msgi->flags & I2C_M_NOSTART) == 0U)
            {
                /* as soon as start is initiated after start bit is
                given slave address along with r/~w is transmitted*/
                putreg8(MG_I2C_START, priv->base + MG_I2C_CTRL_OFFSET);
            }
            // wait till the eight bits completely get transmitted 
            while ((getreg8(priv->base + MG_I2C_CTRL_OFFSET) & MG_I2C_CTRL_PIN) != 0U)
            {
                /* busy wait */
            }
            // check whether ack is receieved from slave
            if (getreg8(priv->base + MG_I2C_STATUS_OFFSET) &
                MG_I2C_STATUS_AD0_LRB)
            {
                putreg8(MG_I2C_STOP, priv->base + MG_I2C_CTRL_OFFSET);
                return 1;
            }

            for (uint32_t i = 0; (i < msgi->length); i++)
            {
                // write the data in data register
                putreg8(msgi->buffer[i], priv->base + MG_I2C_S0_OFFSET);
                // wait till the eight bits completely get transmitted
                while ((getreg8(priv->base + MG_I2C_CTRL_OFFSET) & MG_I2C_CTRL_PIN) != 0U)
                {
                }
                // check whether ack is receieved from slave
                if (getreg8(priv->base + MG_I2C_STATUS_OFFSET) &
                    MG_I2C_STATUS_AD0_LRB)
                {
                    putreg8(MG_I2C_STOP, priv->base + MG_I2C_CTRL_OFFSET);
                    return 1;
                }
            }
            if ((msgi->flags & I2C_M_NOSTOP) == 0U)
            {
                putreg8(MG_I2C_STOP, priv->base + MG_I2C_CTRL_OFFSET);
            }
            else
            {
                putreg8(MG_I2C_REPSTART, priv->base + MG_I2C_CTRL_OFFSET);
                // wait till the eight bits completely get transmitted
                while ((getreg8(priv->base + MG_I2C_CTRL_OFFSET) & MG_I2C_CTRL_PIN) != 0U)
                {
                    /* busy wait */
                }
            }
        }

        else
        {
            uint8_t first_read;
            if ((msgi->flags & I2C_M_NOSTART) == 0U)
            {
                // wait till bus is free
                while ((getreg8(priv->base + MG_I2C_STATUS_OFFSET) &
                        MG_I2C_STATUS_BB) == 0)
                {
                }
            }
            // write data in data register
            putreg8((msgi->addr << 1U) | 1U, priv->base + MG_I2C_S0_OFFSET);
            if ((msgi->flags & I2C_M_NOSTART) == 0U)
            {
                /* as soon as start is initiated after start bit is
                given slave address along with r/~w is transmitted*/
                putreg8(MG_I2C_START, priv->base + MG_I2C_CTRL_OFFSET);
            }
            // wait till the eight bits completely get transmitted
            while ((getreg8(priv->base + MG_I2C_CTRL_OFFSET) & MG_I2C_CTRL_PIN) != 0U)
            {
                /* busy wait */
            }
            // check whether ack is receieved from slave
            if (getreg8(priv->base + MG_I2C_STATUS_OFFSET) &
                MG_I2C_STATUS_AD0_LRB)
            {
                putreg8(priv->base + MG_I2C_CTRL_OFFSET, MG_I2C_STOP);
                return 1;
            }
         

            for (uint8_t i = 0; (i <= msgi->length); i++)
            {
                if (i == 0U)
                {
                    if (msgi->length == 1U)
                    {
                        putreg8(MG_I2C_NACK, priv->base + MG_I2C_CTRL_OFFSET);
                    }
                    first_read = getreg8(priv->base + MG_I2C_S0_OFFSET);
                 
                    continue;
                }
                // wait till the eight bits completely get transmitted
                while ((getreg8(priv->base + MG_I2C_CTRL_OFFSET) & MG_I2C_CTRL_PIN) != 0U)
                {
                    /* busy wait */
                }
                if (i == (msgi->length - 1))
                {
                    putreg8(MG_I2C_NACK, priv->base + MG_I2C_CTRL_OFFSET);
                }
                msgi->buffer[i - 1u] = getreg8(priv->base + MG_I2C_S0_OFFSET);
            }

            //

            if ((msgi->flags & I2C_M_NOSTOP) == 0U)
            {

                putreg8(MG_I2C_STOP, priv->base + MG_I2C_CTRL_OFFSET);
            }
            else
            {

                putreg8(MG_I2C_REPSTART, priv->base + MG_I2C_CTRL_OFFSET);
                
                while ((getreg8(priv->base + MG_I2C_CTRL_OFFSET) & MG_I2C_CTRL_PIN) == 0)
                {
                    /* busy wait */
                }
            }
        }
    }
    return 0;
}

static int mg_i2c_shutdown(FAR struct i2c_master_s *dev)
{
    return 0;
}