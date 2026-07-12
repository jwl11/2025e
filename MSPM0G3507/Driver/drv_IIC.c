#include "drv_IIC.h"

/**
 * @brief  等待 I2C 控制器空闲
 * @return true 完成, false 超时或错误
 */
static bool drv_iic_wait_idle(void)
{
    uint32_t timeout = 100000;

    while (timeout--) {
        uint32_t status = DL_I2C_getControllerStatus(as5600_INST);

        if (status & DL_I2C_CONTROLLER_STATUS_ERROR) {
            DL_I2C_flushControllerTXFIFO(as5600_INST);
            DL_I2C_resetControllerTransfer(as5600_INST);
            return false;
        }

        if (status & DL_I2C_CONTROLLER_STATUS_IDLE) {
            return true;
        }
    }
    return false;
}

/**
 * @brief  向 I2C 从机指定寄存器写入数据
 */
bool drv_iic_write(uint8_t devAddr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t txBuf[8];
    uint8_t totalLen;

    if (len > 7) return false;
    totalLen = 1 + len;

    txBuf[0] = reg;
    for (uint8_t i = 0; i < len; i++) {
        txBuf[1 + i] = data[i];
    }

    DL_I2C_resetControllerTransfer(as5600_INST);

    /* 先填 FIFO，再启动传输（顺序修复） */
    DL_I2C_fillControllerTXFIFO(as5600_INST, txBuf, totalLen);
    DL_I2C_startControllerTransfer(as5600_INST, devAddr,
        DL_I2C_CONTROLLER_DIRECTION_TX, totalLen);

    return drv_iic_wait_idle();
}

bool drv_iic_write_byte(uint8_t devAddr, uint8_t reg, uint8_t data)
{
    return drv_iic_write(devAddr, reg, &data, 1);
}

/**
 * @brief  从 I2C 从机指定寄存器读取数据
 *
 *         分两步: 1) 写寄存器地址  2) 读数据
 *         两步之间 DL_I2C_startControllerTransfer() 会自动加 STOP，
 *         AS5600 已验证可接受此格式。
 */
bool drv_iic_read(uint8_t devAddr, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (len > 8) return false;

    /* --- 第一步: 发寄存器地址 --- */
    DL_I2C_resetControllerTransfer(as5600_INST);
    DL_I2C_fillControllerTXFIFO(as5600_INST, &reg, 1);       /* 先填 FIFO  */
    DL_I2C_startControllerTransfer(as5600_INST, devAddr,       /* 再启动    */
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    if (!drv_iic_wait_idle()) {
        return false;
    }

    /* --- 第二步: 读数据 --- */
    DL_I2C_startControllerTransfer(as5600_INST, devAddr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    if (!drv_iic_wait_idle()) {
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        data[i] = DL_I2C_receiveControllerData(as5600_INST);
    }

    return true;
}

bool drv_iic_read_byte(uint8_t devAddr, uint8_t reg, uint8_t *data)
{
    return drv_iic_read(devAddr, reg, data, 1);
}
