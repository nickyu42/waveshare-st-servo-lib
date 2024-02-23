#include "servo.h"

#include <stdarg.h>
#include <stdbool.h>

#include "logging.h"

static uint8_t tx_buffer[9] = {
    // Fill tx_buffer with preamble
    0xff,
    0xff,
};
static uint8_t rx_buffer[16];

osEventFlagsId_t servo_received_flag;

// Used for storing the amount of bytes received last
// WARNING: NOT THREAD-SAFE
uint16_t received_bytes;

/**
 * Verifies if the checksum received checksum matches the calculated checksum
 * @param total_msg_len length of message in bytes
 * @return true if checksum in rx_buffer matches
 */
static bool rx_checksum_ok(const uint8_t total_msg_len)
{
    uint8_t cc = 0;
    for (size_t i = 2; i < total_msg_len - 1; i++)
    {
        cc += rx_buffer[i];
    }
    return (cc ^ 0xff) == rx_buffer[total_msg_len - 1];
}

/**
 * Creates a servo command in the given buffer.
 *
 * Message protocol, the protocol is in principle I2C
 *
 * Host -> Servo
 * cmd   [0xff 0xff ID MSG_LEN 0x01 CMD      CHECKSUM          ]
 * read  [0xff 0xff ID MSG_LEN 0x02 MEMADDR  LEN      CHECKSUM ]
 * write [0xff 0xff ID MSG_LEN 0x03 MEMADDR  <DATA>   CHECKSUM ]
 *
 * Servo -> Host
 * ack   [0xff 0xff ID MSG_LEN ERR CHECKSUM          ]
 * resp  [0xff 0xff ID MSG_LEN ERR <DATA>   CHECKSUM ]
 *
 * Checksum is calculated as: ~(ID + MSG_LEN + <DATA>)
 *
 * @param buffer the buffer in which the packet will be stored
 * @param s handle servo that the message will be sent to
 * @param data_bytes the number of data bytes
 * @param ...
 * @return size of command in bytes
 */
static size_t construct_message(uint8_t *buffer, const struct servo_handle_t *s, size_t data_bytes, ...)
{
    va_list arg_ptr;
    uint8_t current_byte;

    // id + data + checksum
    uint8_t cc = s->servo_id + data_bytes + 1;

    buffer[2] = s->servo_id;
    buffer[3] = data_bytes + 1;

    va_start(arg_ptr, data_bytes);
    for (size_t j = 0; j < data_bytes; j++)
    {
        current_byte = va_arg(arg_ptr, int);
        buffer[4 + j] = current_byte;
        cc += current_byte;
    }
    va_end(arg_ptr);

    buffer[4 + data_bytes] = cc ^ 0xff;

    // 5 bytes of metadata + n bytes of data
    // - preamble: 2
    // - servo id: 1
    // - message length: 1
    // - data bytes: n
    // - checksum: 1
    return 5 + data_bytes;
}

/**
 * Wrapper around HAL transmit, so servo library is hardware-agnostic.
 */
static enum servo_result_t read_write_blocking(
    const struct servo_handle_t *s,
    const uint8_t *tx, uint16_t tx_size,
    uint8_t *rx, uint16_t rx_size)
{
    uint32_t flags;
    HAL_StatusTypeDef res;
    res = HAL_UARTEx_ReceiveToIdle_DMA(s->uart, rx, 16);
    if (res != HAL_OK)
    {
        LOG_ERR("Could not start DMA RX on servo UART line: %d", res);
        return SERVO_HAL_ERR;
    }

    // Disable interrupt on half-transfer
    __HAL_DMA_DISABLE_IT(s->uart->hdmarx, DMA_IT_HT);

    HAL_StatusTypeDef err;
    err = HAL_UART_Transmit(s->uart, tx, tx_size, SERVO_UART_TIMOUT_ms);
    if (err != HAL_OK)
    {
        if (err == HAL_TIMEOUT)
        {
            return SERVO_TX_TIMOUT;
        }

        return SERVO_HAL_ERR;
    }

    osEventFlagsClear(servo_received_flag, SERVO_DMA_RECEIVED_MSK);
    flags = osEventFlagsWait(servo_received_flag, SERVO_DMA_RECEIVED_MSK, osFlagsWaitAny, 100);

    // Timeout occurred
    if (flags & osFlagsErrorTimeout)
    {
        return SERVO_RX_TIMEOUT;
    }

    // FIX: don't use a global for this
    if (received_bytes != rx_size)
    {
        return SERVO_MISSING_DATA;
    }

    return SERVO_OK;
}

enum servo_result_t servo_command(const struct servo_handle_t *s, const uint8_t cmd)
{
    enum servo_result_t res;

    size_t n_bytes = construct_message(tx_buffer, s, 1, cmd);

    res = read_write_blocking(s, tx_buffer, n_bytes, rx_buffer, n_bytes);
    if (res != SERVO_OK)
    {
        return res;
    }

    if (!rx_checksum_ok(n_bytes))
    {
        return SERVO_INVALID_CHECKSUM;
    }

    return SERVO_OK;
}

enum servo_result_t servo_write_mem_addr(struct servo_handle_t *s, uint8_t addr, uint8_t value)
{
    enum servo_result_t res;

    size_t n_bytes = construct_message(tx_buffer, s, 3, SERVO_MEM_WRITE_CMD, addr, value);

    res = read_write_blocking(s, tx_buffer, n_bytes, rx_buffer, 6);
    if (res != SERVO_OK)
    {
        return res;
    }

    if (!rx_checksum_ok(6))
    {
        return SERVO_INVALID_CHECKSUM;
    }

    return res;
}

enum servo_result_t servo_write_mem_addr_u16(struct servo_handle_t *s, uint8_t addr, uint16_t value)
{
    enum servo_result_t res;

    size_t n_bytes = construct_message(tx_buffer, s, 4, SERVO_MEM_WRITE_CMD, addr, (value & 0xff), (value >> 8));

    res = read_write_blocking(s, tx_buffer, n_bytes, rx_buffer, 6);
    if (res != SERVO_OK)
    {
        return res;
    }

    if (!rx_checksum_ok(6))
    {
        return SERVO_INVALID_CHECKSUM;
    }

    return res;
}

enum servo_result_t servo_read_mem_addr(struct servo_handle_t *s, uint8_t addr, uint8_t *buf, size_t mem_bytes)
{
    enum servo_result_t res;

    size_t n_bytes = construct_message(tx_buffer, s, 3, SERVO_MEM_READ_CMD, addr, mem_bytes);

    res = read_write_blocking(s, tx_buffer, n_bytes, rx_buffer, 6 + mem_bytes);
    if (res != SERVO_OK)
    {
        return res;
    }

    if (!rx_checksum_ok(6 + mem_bytes))
    {
        return SERVO_INVALID_CHECKSUM;
    }

    for (size_t i = 0; i < mem_bytes; i++)
    {
        buf[i] = rx_buffer[5 + i];
    }

    return SERVO_OK;
}

enum servo_result_t servo_enable_wheel_mode(struct servo_handle_t *s)
{
    enum servo_result_t res;

    res = servo_write_mem_addr(s, SERVO_SRAM_LOCK, 0);
    if (res != SERVO_OK)
    {
        LOG_ERR("Failed to unlock SRAM lock for servo=%u: %d", s->servo_id, res);
        return res;
    }

    res = servo_write_mem_addr(s, SERVO_EEPROM_MODE, 1);
    if (res != SERVO_OK)
    {
        LOG_ERR("Failed to set MODE=1 servo=%u: %d", s->servo_id, res);
        return res;
    }

    res = servo_write_mem_addr(s, SERVO_SRAM_LOCK, 1);
    if (res != SERVO_OK)
    {
        LOG_ERR("Failed to lock SRAM for servo=%u: %d", s->servo_id, res);
        return res;
    }

    return SERVO_OK;
}

enum servo_result_t servo_ping(struct servo_handle_t *s)
{
    return servo_command(s, SERVO_PING_CMD);
}

enum servo_result_t servo_set_servo_position(struct servo_handle_t *s, uint16_t angle_u16)
{
    return servo_write_mem_addr_u16(s, SERVO_SRAM_GOAL_POSITION_L, angle_u16);
}

enum servo_result_t servo_set_speed(struct servo_handle_t *s, uint16_t speed_u16)
{
    return servo_write_mem_addr_u16(s, SERVO_SRAM_GOAL_SPEED_L, speed_u16);
}

enum servo_result_t servo_init(struct servo_handle_t *s, uint8_t acc, uint16_t goal_speed)
{
    enum servo_result_t res;
    res = servo_write_mem_addr(s, SERVO_SRAM_ACC, acc);
    if (res != SERVO_OK)
    {
        LOG_ERR("Failed to set servo acceleration for servo=%u: %d", s->servo_id, res);
        return res;
    }

    res = servo_write_mem_addr_u16(s, SERVO_SRAM_GOAL_SPEED_L, goal_speed);
    if (res != SERVO_OK)
    {
        LOG_ERR("Failed to set servo goal speed for servo=%u: %d", s->servo_id, res);
        return res;
    }

    return servo_enable_wheel_mode(s);
}

// This entire read function costs 1.04 ms at a baud rate of 1M
enum servo_result_t servo_read_state(struct servo_handle_t *s)
{
    enum servo_result_t res;
    uint8_t mem_buff[2];

    res = servo_read_mem_addr(s, SERVO_SRAM_PRESENT_VOLTAGE, mem_buff, 1);
    if (res != SERVO_OK)
    {
        LOG_ERR("Could not read servo %u voltage: %d", s->servo_id, res);
        return res;
    }
    s->state.voltage_V = mem_buff[0];

    res = servo_read_mem_addr(s, SERVO_SRAM_PRESENT_CURRENT_L, mem_buff, 2);
    if (res != SERVO_OK)
    {
        LOG_ERR("Could not read servo %u current: %d", s->servo_id, res);
        return res;
    }
    s->state.current_mA = ((uint16_t)mem_buff[1]) << 8 | ((uint16_t)mem_buff[0]);

    res = servo_read_mem_addr(s, SERVO_SRAM_PRESENT_POSITION_L, mem_buff, 2);
    if (res != SERVO_OK)
    {
        LOG_ERR("Could not read servo %u position: %d", s->servo_id, res);
        return res;
    }
    s->state.position = ((uint16_t)mem_buff[1]) << 8 | ((uint16_t)mem_buff[0]);

    res = servo_read_mem_addr(s, SERVO_SRAM_PRESENT_SPEED_L, mem_buff, 2);
    if (res != SERVO_OK)
    {
        LOG_ERR("Could not read servo %u speed: %d", s->servo_id, res);
        return res;
    }
    s->state.speed = ((uint16_t)mem_buff[1]) << 8 | ((uint16_t)mem_buff[0]);

    res = servo_read_mem_addr(s, SERVO_SRAM_PRESENT_LOAD_L, mem_buff, 2);
    if (res != SERVO_OK)
    {
        LOG_ERR("Could not read servo %u load: %d", s->servo_id, res);
        return res;
    }
    s->state.load = ((uint16_t)mem_buff[1]) << 8 | ((uint16_t)mem_buff[0]);

    return SERVO_OK;
}

float servo_get_position_degrees(const struct servo_handle_t *s)
{
    // In the Waveshare servo reference library, the position is represented as 16-bit
    // sign-magnitude, but in reality, it's a 12 bit unsigned value, which is
    // usually what magnetic encoders spew out.

    return (((float)s->state.position) / 4095.0f) * 360.0f;
}

void servo_HAL_UART_RxCpltCallback(uint16_t size)
{
    osEventFlagsSet(servo_received_flag, SERVO_DMA_RECEIVED_MSK);
    received_bytes = size;
}