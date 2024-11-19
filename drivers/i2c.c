#include "i2c.h"
#include <msp430.h>

#define DEFAULT_SLAVE_ADDRESS (0x29)

typedef enum
{
    ADDR_SIZE_8BIT,
    ADDR_SIZE_16BIT
} addr_size_t;

typedef enum
{
    REG_SIZE_8BIT,
    REG_SIZE_16BIT,
    REG_SIZE_32BIT
} reg_size_t;

static bool start_transfer(addr_size_t addr_size, uint16_t addr)
{
    bool success = false;
    UCB1CTL1 |= UCTXSTT + UCTR; /* Set up master as TX and send start condition */

    /* Note, when master is TX, we must write to TXBUF before waiting for UCTXSTT */
    switch (addr_size) {
    case ADDR_SIZE_8BIT:
        UCB1TXBUF = addr & 0xFF;
        break;
    case ADDR_SIZE_16BIT:
        UCB1TXBUF = (addr >> 8) & 0xFF; /* Send the most significant byte of the 16-bit address */
        break;
    }

    while (UCB1CTL1 & UCTXSTT); /* Wait for start condition to be sent */
    success = !(UCB1STAT & UCNACKIFG);
    if (success) {
        while (!(UCB1IFG & UCTXIFG)) {}; /* Wait for byte to be sent */
        success = !(UCB1STAT & UCNACKIFG);
    }

    if (success) {
        switch (addr_size) {
        case ADDR_SIZE_8BIT:
            break;
        case ADDR_SIZE_16BIT:
            UCB1TXBUF = addr & 0xFF; /* Send the least significant byte of the 16-bit address */
            while (!(UCB1IFG & UCTXIFG)) {}; /* Wait for byte to be sent */
            success = !(UCB1STAT & UCNACKIFG);
            break;
        }
    }
    return success;
}

static void stop_transfer()
{
    UCB1CTL1 |= UCTXSTP; /* Send stop condition */
    while (UCB1CTL1 & UCTXSTP); /* Wait for stop condition to be sent */
}

/* Read a register of size reg_size at address addr.
 * NOTE: The bytes are read from MSB to LSB. */
static bool read_reg(addr_size_t addr_size, uint16_t addr, reg_size_t reg_size, uint8_t *data)
{
    bool success = false;

    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    /* Address sent, now configure as receiver and get the data */
    UCB1CTL1 &= ~UCTR; /* Set as a receiver */
    UCB1CTL1 |= UCTXSTT; /* Send (repeating) start condition (including address of slave) */
    while (UCB1CTL1 & UCTXSTT); /* Wait for start condition to be sent */
    success = !(UCB1STAT & UCNACKIFG);
    if (success) {
        switch (reg_size) {
        case REG_SIZE_8BIT:
            break;
        case REG_SIZE_16BIT:
            /* Bytes are read from most to least significant */
            while ((UCB1IFG & UCRXIFG) == 0); /* Wait for byte before reading the buffer */
            data[1] = UCB1RXBUF; /* RX interrupt is cleared automatically afterwards */
            break;
        case REG_SIZE_32BIT:
            /* Bytes are read from most to least significant */
            while ((UCB1IFG & UCRXIFG) == 0);
            data[3] = UCB1RXBUF;
            while ((UCB1IFG & UCRXIFG) == 0);
            data[2] = UCB1RXBUF;
            while ((UCB1IFG & UCRXIFG) == 0);
            data[1] = UCB1RXBUF;
            break;
        }
        stop_transfer();
        while ((UCB1IFG & UCRXIFG) == 0); /* Wait for byte before reading the buffer */
        data[0] = UCB1RXBUF; /* RX interrupt is cleared automatically afterwards */
    }

    return success;
}

static bool read_reg_bytes(addr_size_t addr_size, uint16_t addr, uint8_t *bytes, uint16_t byte_count)
{
    bool success = false;
    bool transfer_stopped = false;
    int i;

    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    /* Address sent, now configure as receiver and get the value */
    UCB1CTL1 &= ~UCTR; /* Set as a receiver */
    UCB1CTL1 |= UCTXSTT; /* Send (repeating) start condition (including address of slave) */
    while (UCB1CTL1 & UCTXSTT); /* Wait for start condition to be sent */
    success = !(UCB1STAT & UCNACKIFG);
    if (success) {
        for (i = 0; i < byte_count; i++) {
            if (i + 1 == byte_count) {
                stop_transfer();
                transfer_stopped = true;
            }
            success = !(UCB1STAT & UCNACKIFG);
            if (success) {
                while ((UCB1IFG & UCRXIFG) == 0);/* Wait for byte before reading the buffer */
                bytes[i] = UCB1RXBUF; /* RX interrupt is cleared automatically afterwards */
            } else {
                break;
            }
        }
    }
    if (!transfer_stopped) {
        stop_transfer();
    }

    return success;
}

bool i2c_read_addr8_data8(uint8_t addr, uint8_t *data)
{
    return read_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_8BIT, data);
}

bool i2c_read_addr8_data16(uint8_t addr, uint16_t *data)
{
    return read_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_16BIT, (uint8_t *)data);
}

bool i2c_read_addr16_data8(uint16_t addr, uint8_t *data)
{
    return read_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_8BIT, data);
}

bool i2c_read_addr16_data16(uint16_t addr, uint16_t *data)
{
    return read_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_16BIT, (uint8_t *)data);
}

bool i2c_read_addr8_data32(uint16_t addr, uint32_t *data)
{
    return read_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_32BIT, (uint8_t *)data);
}

bool i2c_read_addr16_data32(uint16_t addr, uint32_t *data)
{
    return read_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_32BIT, (uint8_t *)data);
}

bool i2c_read_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count)
{
    return read_reg_bytes(ADDR_SIZE_8BIT, start_addr, bytes, byte_count);
}

/* Write data to a register of size reg_size at address addr.
 * NOTE: Writes the most significant byte (MSB) first. */
static bool write_reg(addr_size_t addr_size, uint16_t addr, reg_size_t reg_size, uint16_t data)
{
    bool success = false;

    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    switch (reg_size) {
    case REG_SIZE_8BIT:
        success = true;
        break;
    case REG_SIZE_16BIT:
        UCB1TXBUF = (data >> 8) & 0xFF; /* Start with the most significant byte */
        while (!(UCB1IFG & UCTXIFG)) {}; /* Wait for byte to be sent */
        success = !(UCB1STAT & UCNACKIFG);
        break;
    case REG_SIZE_32BIT:
        /* Not supported */
        return false;
    }

    if (success) {
        UCB1TXBUF = 0xFF & data; /* Send the least significant byte */

        while (!(UCB1IFG & UCTXIFG)) {}; /* Wait for byte to be sent */
        success = !(UCB1STAT & UCNACKIFG);
    }

    stop_transfer();
    return success;
}

static bool write_reg_bytes(addr_size_t addr_size, uint16_t addr, uint8_t *bytes, uint16_t byte_count)
{
    bool success = false;
    uint16_t i;
    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    for (i = 0; i < byte_count; i++) {
        UCB1TXBUF = bytes[i];
        while (!(UCB1IFG & UCTXIFG)) {}; /* Wait for byte to be sent */
        success = !(UCB1STAT & UCNACKIFG);
        if (!success) {
            break;
        }
    }

    stop_transfer();
    return success;
}

bool i2c_write_addr8_data8(uint8_t addr, uint8_t value)
{
    return write_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_8BIT, value);
}

bool i2c_write_addr8_data16(uint8_t addr, uint16_t value)
{
    return write_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_16BIT, value);
}

bool i2c_write_addr16_data8(uint16_t addr, uint8_t value)
{
    return write_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_8BIT, value);
}

bool i2c_write_addr16_data16(uint16_t addr, uint16_t value)
{
    return write_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_16BIT, value);
}
bool i2c_write_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count)
{
    return write_reg_bytes(ADDR_SIZE_8BIT, start_addr, bytes, byte_count);
}

void i2c_set_slave_address(uint8_t addr)
{
    UCB1I2CSA = addr;
}

void i2c_init()
{
    P4SEL |= 0x06;                            // Assign I2C pins to USCI_B0
    UCB1CTL1 |= UCSWRST;  // reset I2C peripheral
    UCB1CTL0 = UCMST + UCMODE_3 + UCSYNC;  // I2C master, I2C mode, sync mode
    UCB1CTL1 = UCSSEL_2 + UCSWRST;  // SMCLK clock source, keep reset
    UCB1BR0 = 12;  // set SCL clock frequency
    UCB1BR1 = 0;
    UCB1CTL1 &= ~UCSWRST;  // release I2C peripheral from reset
}
