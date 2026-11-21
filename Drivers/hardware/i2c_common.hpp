#pragma once

enum I2C_Result
{
    I2C_Result_Success = 0,

    // 超时
    I2C_Result_timeout,
    // 地址无应答
    I2C_Result_nAddrHit,
    // 被NAK
    I2C_Result_NAK,
    // 失去仲裁
    I2C_Result_ARBLOSE,
    // 其他错误
    I2C_Result_ERROR,
};

enum I2C_SPEED
{
    I2C_SPEED_50k = 0,
    I2C_SPEED_100k,
    I2C_SPEED_400k,
};
