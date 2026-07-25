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

//软件IIC

static i2cbus_struct *P_this; // 全局指针

/////////////////////////以下是为方便移植///////////////////////////////
/*
 * I2C 位操作宏:
 * - SCL: 推挽输出 (始终由主机驱动)
 * - SDA: SDA=1 切输入释放总线(靠上拉), SDA=0 切输出拉低
 */
#define I2C_SCL_H()     DL_GPIO_setPins(MPU6050_SCL_PORT, MPU6050_SCL_PIN)
#define I2C_SCL_L()     DL_GPIO_clearPins(MPU6050_SCL_PORT, MPU6050_SCL_PIN)
#define I2C_SDA_H()     DL_GPIO_disableOutput(MPU6050_SDA_PORT, MPU6050_SDA_PIN)
#define I2C_SDA_L()     do { DL_GPIO_enableOutput(MPU6050_SDA_PORT, MPU6050_SDA_PIN); \
                             DL_GPIO_clearPins(MPU6050_SDA_PORT, MPU6050_SDA_PIN); } while(0)
#define I2C_SDA_READ()  DL_GPIO_readPins(MPU6050_SDA_PORT, MPU6050_SDA_PIN)

/* 短别名, 内部使用 */
#define SCL_H()         I2C_SCL_H()
#define SCL_L()         I2C_SCL_L()
#define SDA_H()         I2C_SDA_H()
#define SDA_L()         I2C_SDA_L()
#define SDA_RD()        I2C_SDA_READ()
/////////////////////////以上是为方便移植///////////////////////////////

#define I2C_DLY  delay_us(P_this->delay_time)

/*
 * 以下 I2C 底层函数使用已验证通过的位操作时序
 * (与 app_MPU6050_test.c 中的手动测试代码一致)
 */
static void _SI2C_Start(void)
{
    SDA_H(); I2C_DLY;
    SCL_H(); I2C_DLY;
    SDA_L(); I2C_DLY;
    SCL_L(); I2C_DLY;
}

static void _SI2C_Stop(void)
{
    SDA_L(); I2C_DLY;
    SCL_H(); I2C_DLY;
    SDA_H(); I2C_DLY;
}

static void _SI2C_WriteByte(uint8_t Byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (Byte & 0x80) SDA_H(); else SDA_L();
        Byte <<= 1;
        I2C_DLY;
        SCL_H(); I2C_DLY;
        SCL_L(); I2C_DLY;
    }
}

static uint8_t _SI2C_ReceiveByte(void)
{
    uint8_t i, Byte = 0x00;
    SDA_H(); I2C_DLY;
    for (i = 0; i < 8; i++) {
        SCL_H(); I2C_DLY;
        Byte <<= 1;
        if (I2C_SDA_READ()) Byte |= 1;
        SCL_L(); I2C_DLY;
    }
    return Byte;
}

static void _SI2C_WriteAck(uint8_t AckBit)
{
    if (AckBit) SDA_H(); else SDA_L();
    I2C_DLY;
    SCL_H(); I2C_DLY;
    SCL_L(); I2C_DLY;
    SDA_H(); I2C_DLY;
}

static uint8_t _SI2C_ReceiveAck(void)
{
    uint8_t AckBit;
    SDA_H(); I2C_DLY;
    SCL_H(); I2C_DLY;
    AckBit = I2C_SDA_READ();
    SCL_L(); I2C_DLY;
    return AckBit;  /* 0=ACK, 非0=NACK */
}

/////////////////////////////////你应该看以下的函数，上面是iic底层通信协议///////////////////////////////////////
/*i2c_bus i2cbus_struct对象
 *scl_gpio scl的IO口
 *scl_pin scl引脚号
 *sda_gpio sda的IO口
 *sda_pin sda引脚号
 *Address 设备地址
 *delay_time 延时时间
 */
void MyI2C_Init(i2cbus_struct *i2c_bus, uint8_t Address, uint16_t delay_time)
{
    P_this              = i2c_bus; // 获取指针对象
    i2c_bus->mode_16bit = 0;            /// 默认使用8位操作模式
    i2c_bus->address    = Address << 1; // 获取7位设备地址
    i2c_bus->delay_time = delay_time;   // 延时时间(us)

    /* 为 SCL/SDA 使能内部上拉 (I2C 总线必须有上拉才能工作) */
    DL_GPIO_initDigitalInputFeatures(MPU6050_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MPU6050_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

//(0:8bit模式  1:16bit模式)
void MYI2C_Set_Mode(i2cbus_struct *i2c_bus, uint8_t status)
{
    P_this              = i2c_bus;
    i2c_bus->mode_16bit = status;
}

void MYI2C_Write_Reg(i2cbus_struct *i2c_bus, uint8_t RegAddress, uint16_t Data)
{
    P_this = i2c_bus;
    _SI2C_Start();
    _SI2C_WriteByte(i2c_bus->address);
    _SI2C_ReceiveAck();
    _SI2C_WriteByte(RegAddress);
    _SI2C_ReceiveAck();

    if (i2c_bus->mode_16bit == 1) {            // 如果是16位操作模式
        _SI2C_WriteByte((uint8_t)(Data >> 8)); // 发送高位
        _SI2C_ReceiveAck();
        _SI2C_WriteByte((uint8_t)Data); // 发送低位
    } else {
        _SI2C_WriteByte((uint8_t)Data);
    }
    _SI2C_ReceiveAck();
    _SI2C_Stop();
}
uint16_t MYI2C_Read_Reg(i2cbus_struct *i2c_bus, uint8_t RegAddress)
{
    P_this        = i2c_bus;
    uint16_t Data = 0;
    _SI2C_Start();
    _SI2C_WriteByte(i2c_bus->address);
    if (_SI2C_ReceiveAck() == 1) return 0xFFFF;
    _SI2C_WriteByte(RegAddress);
    if (_SI2C_ReceiveAck() == 1) return 0xFFFF;

    _SI2C_Start();
    _SI2C_WriteByte(i2c_bus->address | 0x01); //|0x01读命令
    if (_SI2C_ReceiveAck() == 1) return 0xFFFF;
    if (i2c_bus->mode_16bit == 1) { // 如果是16位操作模式
        Data = (uint16_t)_SI2C_ReceiveByte() << 8;
        _SI2C_WriteAck(0); // 接收应答
        Data |= _SI2C_ReceiveByte();
    } else {
        Data = _SI2C_ReceiveByte();
    }
    _SI2C_WriteAck(1); // 直接写1结束这次通信
    _SI2C_Stop();
    return Data;
}

void MYI2C_Write_Reg_Continue(i2cbus_struct *i2c_bus, uint8_t RegAddress, uint8_t *data_array, uint16_t array_size)
{    
    P_this = i2c_bus;
    _SI2C_Start();
    
    // 发送设备地址（写模式）
    _SI2C_WriteByte(i2c_bus->address);
    if (_SI2C_ReceiveAck() != 0) {
        _SI2C_Stop();
        return; // 未收到应答，退出
    }
    
    // 发送寄存器地址
    _SI2C_WriteByte(RegAddress);
    if (_SI2C_ReceiveAck() != 0) {
        _SI2C_Stop();
        return; // 未收到应答，退出
    }
    
    // 连续写入数据
    for (uint16_t i = 0; i < array_size;) {
        if (i2c_bus->mode_16bit == 1 && i + 1 < array_size) {
            // 16位模式发送高字节
            _SI2C_WriteByte(data_array[i]);
            if (_SI2C_ReceiveAck() != 0) {
                _SI2C_Stop();
                return;
            }
            
            // 16位模式发送低字节
            _SI2C_WriteByte(data_array[i + 1]);
            if (_SI2C_ReceiveAck() != 0) {
                _SI2C_Stop();
                return;
            }
            
            i += 2;
        } else {    
            // 8位模式发送
            _SI2C_WriteByte(data_array[i]);
            if (_SI2C_ReceiveAck() != 0) {
                _SI2C_Stop();
                return;
            }
            i++;
        }
    }
    
    _SI2C_Stop();
}

void MYI2C_Read_Reg_Continue(i2cbus_struct *i2c_bus, uint8_t RegAddress, uint16_t read_len, uint8_t *data_buf) // IIC 读起始地址Address里的Data[]
{
    P_this = i2c_bus; // 全局指针赋值（与原函数完全一致）
    _SI2C_Start();
    _SI2C_WriteByte(i2c_bus->address);
    if (_SI2C_ReceiveAck() == 1){
        _SI2C_Stop();
        return;
    }
    _SI2C_WriteByte(RegAddress);
    if (_SI2C_ReceiveAck() == 1){
        _SI2C_Stop();
        return;
    }

    _SI2C_Start();
    _SI2C_WriteByte(i2c_bus->address | 0x01); //|0x01读命令
    if (_SI2C_ReceiveAck() == 1){
        _SI2C_Stop();
        return;
    }

    for (uint16_t i = 0; i < read_len; i++) {
        data_buf[i] = _SI2C_ReceiveByte();
        if (i < read_len - 1)
            _SI2C_WriteAck(0); // ACK
        else
            _SI2C_WriteAck(1); // NACK
    }
    _SI2C_Stop();
}

uint8_t MYI2C_Add_Scan(i2cbus_struct *i2c_bus)
{
    P_this = i2c_bus;
    // 遍历7位I2C地址范围 (0x08-0x77)
    for (uint8_t address = 0x08; address <= 0x7F; address++) {

        _SI2C_Start();                    // 起始信号
        _SI2C_WriteByte(address << 1);    // 地址
        uint8_t ack = _SI2C_ReceiveAck(); // 读取ACK信号
        _SI2C_Stop();                     // 停止

        if (ack == 0) // 设备存在返回地址
            return address;
    }
    // 未找到设备
    return 0xFF;
}



