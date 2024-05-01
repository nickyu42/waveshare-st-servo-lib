# Message protocol
# --- Host -> Servo ---
# write [0xff 0xff ID MSG_LEN 0x03 MEMADDR  <DATA> CHECKSUM ]
# read  [0xff 0xff ID MSG_LEN 0x02 MEMADDR  LEN    CHECKSUM ]
# ping  [0xff 0xff ID MSG_LEN 0x01 CHECKSUM                 ]

# --- Servo -> Host ---
# ack   [0xff 0xff ID MSG_LEN ERR CHECKSUM          ]
# data  [0xff 0xff ID MSG_LEN ERR <DATA>   CHECKSUM ]

# Checksum is calculates as: ~(servo_id + msg_len + data)

# -------Commands-------
PING_CMD = 0x01
MEM_READ_CMD = 0x02
MEM_WRITE_CMD = 0x03

# -------EPROM(read only)--------
EEPROM_MODEL_L = 3
EEPROM_MODEL_H = 4

# -------EPROM(read & write)--------
EEPROM_ID = 5
EEPROM_BAUD_RATE = 6
EEPROM_MIN_ANGLE_LIMIT_L = 9
EEPROM_MIN_ANGLE_LIMIT_H = 10
EEPROM_MAX_ANGLE_LIMIT_L = 11
EEPROM_MAX_ANGLE_LIMIT_H = 12
EEPROM_CW_DEAD = 26
EEPROM_CCW_DEAD = 27
EEPROM_OFS_L = 31
EEPROM_OFS_H = 32
EEPROM_MODE = 33

# -------SRAM(read & write)--------
SRAM_TORQUE_ENABLE = 40
SRAM_ACC = 41
SRAM_GOAL_POSITION_L = 42
SRAM_GOAL_POSITION_H = 43
SRAM_GOAL_TIME_L = 44
SRAM_GOAL_TIME_H = 45
SRAM_GOAL_SPEED_L = 46
SRAM_GOAL_SPEED_H = 47
SRAM_TORQUE_LIMIT_L = 48
SRAM_TORQUE_LIMIT_H = 49
SRAM_LOCK = 55

# -------SRAM(read only)--------
SRAM_PRESENT_POSITION_L = 56  # 15-bit signed (sign-magnitude)
SRAM_PRESENT_POSITION_H = 57
SRAM_PRESENT_SPEED_L = 58  # 15-bit signed (sign-magnitude)
SRAM_PRESENT_SPEED_H = 59
SRAM_PRESENT_LOAD_L = (
    60  # 10-bit signed (sign-magnitude) (0~1000, 1000 = 100% max load)
)
SRAM_PRESENT_LOAD_H = 61
SRAM_PRESENT_VOLTAGE = 62  # 8 bit unsigned
SRAM_PRESENT_TEMPERATURE = 63  # 8 bit unsigned, in celcius
SRAM_MOVING = 66  # bool?
SRAM_PRESENT_CURRENT_L = 69  # 15-bit signed (sign-magnitude)
SRAM_PRESENT_CURRENT_H = 70


# // === ST Servo === TypeNum:9
# SMS_STS st;
# float ServoDigitalRange_ST  = 4095.0;
# float ServoAngleRange_ST    = 360.0;
# float ServoDigitalMiddle_ST = 2047.0;
# #define ServoInitACC_ST      100
# #define ServoMaxSpeed_ST     4000
# #define MaxSpeed_X_ST        4000
# #define ServoInitSpeed_ST    2000


def send_command(uart, servo_id, cmd):
    # XOR to get binary inverse in 1 byte
    checksum = (servo_id + 2 + cmd) ^ 0xFF
    buf = bytearray((0xFF, 0xFF, servo_id, 2, cmd, checksum))
    uart.write(buf)
    return uart.read(12)


def write_mem_addr(uart, servo_id, mem_addr, value):
    checksum = 0xFF & (servo_id + 4 + MEM_WRITE_CMD + mem_addr + value) ^ 0xFF
    buf = bytearray((0xFF, 0xFF, servo_id, 4, MEM_WRITE_CMD, mem_addr, value, checksum))
    uart.write(buf)


def write_mem_addr_u16(uart, servo_id, mem_addr, value):
    checksum = (
        0xFF & (servo_id + 5 + MEM_WRITE_CMD + mem_addr + (value & 0xFF) + (value >> 8))
        ^ 0xFF
    )
    buf = bytearray(
        (
            0xFF,
            0xFF,
            servo_id,
            5,
            MEM_WRITE_CMD,
            mem_addr,
            value & 0xFF,
            (value >> 8) & 0xFF,
            checksum,
        )
    )
    uart.write(buf)


def read_mem_addr(uart, servo_id, mem_addr, n_bytes):
    checksum = 0xFF & (servo_id + 4 + MEM_READ_CMD + mem_addr + n_bytes) ^ 0xFF
    buf = bytearray(
        (0xFF, 0xFF, servo_id, 4, MEM_READ_CMD, mem_addr, n_bytes, checksum)
    )
    uart.write(buf)


def ping(uart, servo_id):
    return send_command(uart, servo_id, PING_CMD)


def read_position(uart, servo_id):
    read_mem_addr(uart, servo_id, SRAM_PRESENT_POSITION_L, 2)
    data = uart.read(14 + 2)
    pos = data[14] << 8 | data[13]
    return pos


def read_speed(uart, servo_id):
    read_mem_addr(uart, servo_id, SRAM_PRESENT_SPEED_L, 2)
    data = uart.read(14 + 2)
    pos = data[14] << 8 | data[13]
    return pos


def read_load(uart, servo_id):
    read_mem_addr(uart, servo_id, SRAM_PRESENT_LOAD_L, 2)
    data = uart.read(14 + 2)
    pos = data[14] << 8 | data[13]
    return pos


def read_temp(uart, servo_id):
    read_mem_addr(uart, servo_id, SRAM_PRESENT_TEMPERATURE, 1)
    data = uart.read(14 + 1)
    temperature = data[13]
    return temperature


def read_voltage(uart, servo_id):
    read_mem_addr(uart, servo_id, SRAM_PRESENT_VOLTAGE, 1)
    data = uart.read(14 + 1)
    return data[13]


def read_mode(uart, servo_id):
    read_mem_addr(uart, servo_id, EEPROM_MODE, 1)
    data = uart.read(14 + 1)
    return data[13]


def read_current(uart, servo_id):
    read_mem_addr(uart, servo_id, SRAM_PRESENT_CURRENT_L, 2)
    data = uart.read(14 + 2)
    c = data[14] << 8 | data[13]
    return c


def set_servo_id(uart, from_id, to_id):
    print(write_mem_addr(uart, from_id, SRAM_LOCK, 0))
    print(write_mem_addr(uart, from_id, EEPROM_ID, to_id))
    print(write_mem_addr(uart, to_id, SRAM_LOCK, 1))


def enable_wheel_mode(uart, servo_id):
    print(write_mem_addr(uart, servo_id, SRAM_LOCK, 0))
    print(write_mem_addr(uart, servo_id, EEPROM_MODE, 1))
    print(write_mem_addr(uart, servo_id, SRAM_LOCK, 1))
