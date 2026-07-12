#include "drv_IIC.h"

/**
 * @brief  等待 I2C 控制器空闲
 * @return true 空闲, false 超时（总线错误）
 */
static bool drv_iic_wait_idle(void)
{
    uint32_t timeout = 100000; /* 超时计数器 */
    uint32_t status;

    while (timeout--) {
        status = DL_I2C_getControllerStatus(as5600_INST);

        /* 检测到错误（NACK/仲裁丢失） */
        if (status & DL_I2C_CONTROLLER_STATUS_ERROR) {
            DL_I2C_flushControllerTXFIFO(as5600_INST);
            DL_I2C_resetControllerTransfer(as5600_INST);
            return false;
        }

        /* 传输完成，控制器空闲 */
        if (status & DL_I2C_CONTROLLER_STATUS_IDLE) {
            return true;
        }
    }
    return false; /* 超时 */
}

/**
 * @brief  向 I2C 从机指定寄存器写入数据
 * @param  devAddr  7位从机地址
 * @param  reg      寄存器地址
 * @param  data     数据缓冲区指针
 * @param  len      写入字节数
 * @return true 成功, false 失败
 */
bool drv_iic_write(uint8_t devAddr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t txBuf[8]; /* MSPM0 I2C TX FIFO 最大 8 字节 */
    uint8_t totalLen;

    if (len > 7) return false; /* 超出单次传输能力 */
    totalLen = 1 + len;

    /* 组装发送缓冲区：寄存器地址 + 数据 */
    txBuf[0] = reg;
    for (uint8_t i = 0; i < len; i++) {
        txBuf[1 + i] = data[i];
    }

    /* 重置并启动控制器发送 */
    DL_I2C_resetControllerTransfer(as5600_INST);
    DL_I2C_startControllerTransfer(as5600_INST, devAddr,
        DL_I2C_CONTROLLER_DIRECTION_TX, totalLen);
    DL_I2C_fillControllerTXFIFO(as5600_INST, txBuf, totalLen);

    return drv_iic_wait_idle();
}

/**
 * @brief  向 I2C 从机指定寄存器写入单字节
 */
bool drv_iic_write_byte(uint8_t devAddr, uint8_t reg, uint8_t data)
{
    return drv_iic_write(devAddr, reg, &data, 1);
}

/**
 * @brief  从 I2C 从机指定寄存器读取数据
 *
 *         采用两段式 I2C 读操作：
 *         1. 先发送寄存器地址（TX 模式）
 *         2. 再接收数据（RX 模式）
 *
 * @param  devAddr  7位从机地址
 * @param  reg      起始寄存器地址
 * @param  data     接收缓冲区指针
 * @param  len      读取字节数
 * @return true 成功, false 失败
 */
bool drv_iic_read(uint8_t devAddr, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (len > 8) return false; /* 超出硬件 FIFO 接收能力 */

    /* --- 第一步：发送寄存器地址 --- */
    DL_I2C_resetControllerTransfer(as5600_INST);
    DL_I2C_startControllerTransfer(as5600_INST, devAddr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    DL_I2C_fillControllerTXFIFO(as5600_INST, &reg, 1);

    if (!drv_iic_wait_idle()) {
        return false;
    }

    /* --- 第二步：接收数据 --- */
    DL_I2C_startControllerTransfer(as5600_INST, devAddr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    /* 等待接收完成 */
    if (!drv_iic_wait_idle()) {
        return false;
    }

    /* 从 RX FIFO 读取数据 */
    for (uint8_t i = 0; i < len; i++) {
        data[i] = DL_I2C_receiveControllerData(as5600_INST);
    }

    return true;
}

/**
 * @brief  从 I2C 从机指定寄存器读取单字节
 */
bool drv_iic_read_byte(uint8_t devAddr, uint8_t reg, uint8_t *data)
{
    return drv_iic_read(devAddr, reg, data, 1);
}
