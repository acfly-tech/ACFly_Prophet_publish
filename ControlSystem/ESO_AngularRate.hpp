#pragma once

// 角速度带模型ESO
// 朱文杰 20180102
// 请勿用于商业用途
// ！！抄袭必究！！

#include "AC_Math.hpp"
#include "Basic.h"
#include "Filters_BS.hpp"
#include "Filters_LP.hpp"
#include "complex.hpp"
#include <stdbool.h>

// FT采样点数目(必须为偶数)
#define FT_N 80

class ESO_AngularRate_Base
{
  public:
    virtual ~ESO_AngularRate_Base() = default;

    virtual double get_T() const = 0;
    virtual double get_b() const = 0;
    virtual double get_u() const = 0;

    virtual void update_u(double u) = 0;
    virtual double run(double rate, bool inFlight) = 0;

    // 获取傅里叶变换结果
    virtual const complex<double> *get_ftFreqs() const = 0;

    // 获取估计角速度
    virtual double get_EsAngularRate() const = 0;
    // 获取估计扰动
    virtual double get_EsDisturbance() const = 0;
    // 获取估计滤波扰动
    virtual double get_EsDisturbanceFilted() const = 0;
    // 获取估计扰动导数
    virtual double get_EsDisturbanceD() const = 0;
    // 获取估计角加速度
    virtual double get_EsAngularAcceleration() const = 0;
    // 获取估计主动力
    virtual double get_EsMainPower() const = 0;
};

class ESO_AngularRate : public ESO_AngularRate_Base
{
  private:
    double invT;
    double z_inertia;
    double z1;
    double z2;

    double last_err;
    double last_rate;
    double last_disturbanceFilted;
    double disturbanceD_Filted;

    double Hz;
    double h;

    // 陷波频点
    double bsFreq1;
    double bsRange1;
    uint16_t bs1Counter;
    double bsFreq2;
    double bsRange2;
    uint16_t bs2Counter;
    double bsFreq3; // 滤波器3频点(仅手动设置)
    double bsRange3;
    // 手动陷波频点
    double manualBSFreq2; // 滤波器2手动陷波频点
    double manualBSRange2;
    double manualBSFreq3; // 滤波器3手动陷波频点(未设置时滤波器3关闭)
    double manualBSRange3;

    // 角速度滤波器
    Filter_Butter_LP rate_filter;
    Filter_Butter2_BS rate_BSfilter1;
    Filter_Butter2_BS rate_BSfilter2;
    Filter_Butter2_BS rate_BSfilter3;

    // 角加速度滤波器
    Filter_Butter_LP acc_filter;
    Filter_Butter2_BS acc_BSfilter1;
    Filter_Butter2_BS acc_BSfilter2;
    Filter_Butter2_BS acc_BSfilter3;

    // 扰动滤波器
    Filter_Butter_LP disturbanceZ_filter;

    // 傅里叶变换
    uint16_t ft_ind;
    double ft_in[FT_N];
    complex<double> ft_freqs[FT_N / 2];
    // 傅里叶变换参数
    static bool ft_coeffs_initialized;
    static complex<double> ft_coeffs[FT_N];
    // 初始化傅里叶变换
    void init_ft();

  public:
    double beta1;
    double beta2;

    double T;
    double get_T() const
    {
        return this->T;
    };
    double b;
    double get_b() const
    {
        return this->b;
    };
    double u;
    double get_u() const
    {
        return this->u;
    };

    const complex<double> *get_ftFreqs() const
    {
        return ft_freqs;
    }

    double get_bsFreq1() const
    {
        return this->bsFreq1;
    };
    double get_bsRange1() const
    {
        return this->bsRange1;
    };
    double get_bsFreq2() const
    {
        return this->bsFreq2;
    };
    double get_bsRange2() const
    {
        return this->bsRange2;
    };
    double get_bsFreq3() const
    {
        return this->bsFreq3;
    };
    double get_bsRange3() const
    {
        return this->bsRange3;
    };

    ESO_AngularRate() {}

    // manualBSFreq/manualBSRange: 滤波器2手动陷波频点/频宽(BS/BSBW参数)
    // manualBSFreq2/manualBSRange2: 滤波器3手动陷波频点/频宽(BS2/BSBW2参数, 无效时滤波器3不开启)
    // 滤波器1固定自动分析 不接受手动设置
    inline void init(uint8_t order, double T, double b, double beta1, double beta2, uint8_t orderD, double betaD, double manualBSFreq, double manualBSRange, double manualBSFreq2,
                     double manualBSRange2, double Hz)
    {
        init(order, T, b, beta1, beta2, orderD, betaD, manualBSFreq, manualBSRange, manualBSFreq2, manualBSRange2, Hz, 0);
    }
    // a_max: 已废弃不再使用(原ESO内部z_inertia斜率限幅移除: a_max偏小时ESO模型失配
    // 会引起高频震颤, 限幅改由Ctrl_Attitude对滤波扰动以AC_AMaxRP/AC_AMaxYaw斜率限幅实现, 对齐仿真CtrlAtt31), 保留形参兼容既有调用
    inline void init(uint8_t order, double T, double b, double beta1, double beta2, uint8_t orderD, double betaD, double manualBSFreq, double manualBSRange, double manualBSFreq2,
                     double manualBSRange2, double Hz, double a_max)
    {
        (void) a_max;
        this->Hz = Hz;
        this->h = 1.0 / Hz;
        this->beta1 = beta1;
        this->beta2 = beta2;
        this->manualBSFreq2 = manualBSFreq;
        this->manualBSRange2 = manualBSRange;
        this->manualBSFreq3 = manualBSFreq2;
        this->manualBSRange3 = manualBSRange2;

        // 重置角速度滤波器
        rate_filter.set_cutoff_frequency(order, Hz, beta1);
        rate_filter.reset(0);
        rate_BSfilter1.set_inavailable();
        rate_BSfilter1.reset(0);
        rate_BSfilter2.set_inavailable();
        rate_BSfilter2.reset(0);
        rate_BSfilter3.set_inavailable();
        rate_BSfilter3.reset(0);
        // 重置角加速度滤波器
        acc_filter.set_cutoff_frequency(order, Hz, beta2);
        acc_filter.reset(0);
        acc_BSfilter1.set_inavailable();
        acc_BSfilter1.reset(0);
        acc_BSfilter2.set_inavailable();
        acc_BSfilter2.reset(0);
        acc_BSfilter3.set_inavailable();
        acc_BSfilter3.reset(0);

        // 滤波器3: 仅手动(BS2/BSBW2参数) init时一次性配置 未设置或无效则保持关闭
        bsFreq3 = bsRange3 = 0;
        if (manualBSFreq3 > 5 && manualBSFreq3 < Hz / 3 - 5)
        {
            bsFreq3 = manualBSFreq3;
            if (manualBSRange3 > 0)
                bsRange3 = manualBSRange3;
            else
                bsRange3 = manualBSFreq3 * 1.2;
            if (bsRange3 > 60)
                bsRange3 = 60;
            acc_BSfilter3.set_cutoff_frequency(Hz, bsFreq3, bsRange3);
            rate_BSfilter3.set_cutoff_frequency_from(acc_BSfilter3);
        }
        else
        {
            acc_BSfilter3.set_inavailable();
            rate_BSfilter3.set_inavailable();
        }
        // 重置扰动滤波器
        disturbanceZ_filter.set_cutoff_frequency(orderD, Hz, betaD);
        disturbanceZ_filter.reset(0);
        disturbanceD_Filted = 0;

        bsFreq1 = bsRange1 = bs1Counter = 0;
        bsFreq2 = bsRange2 = bs2Counter = 0;
        bsFreq3 = bsRange3 = 0;

        this->z1 = this->z2 = this->z_inertia = 0;
        this->last_err = this->last_rate = this->last_disturbanceFilted = 0;
        this->u = 0;

        this->T = T;
        this->invT = 1.0 / T;
        this->b = b;
        init_ft();
    }

    inline void update_u(double u)
    {
        this->u = u;

        // 1. ESO 推演状态 z1
        this->z1 += this->h * (this->z_inertia + this->z2);

        // 2. 更新 z_inertia
        // 原电调限流斜率限幅(a_max)已移除: 限幅过小时ESO模型与实际响应失配会引起高频震颤,
        // 扰动限幅改由Ctrl_Attitude对滤波扰动以AC_AMaxRP/AC_AMaxYaw斜率限幅实现(对齐仿真CtrlAtt31)
        this->z_inertia += this->h * this->invT * (this->b * this->u - this->z_inertia);
    }

    double run(double rate, bool inFlight);

    inline double get_EsAngularRate() const
    {
        return this->z1;
    }
    inline double get_EsDisturbance() const
    {
        return this->z2;
    }
    inline double get_EsDisturbanceFilted() const
    {
        return disturbanceZ_filter.get_result();
    }
    inline double get_EsDisturbanceD() const
    {
        return disturbanceD_Filted;
    }
    inline double get_EsAngularAcceleration() const
    {
        return this->z2 + this->z_inertia;
    }
    inline double get_EsMainPower() const
    {
        return this->z_inertia;
    }
};

class ESO_AngularRateHeli : public ESO_AngularRate_Base
{
  private:
    double invT;
    double z_inertia;
    double z1;

    double last_v;
    double acc;

    double Hz;
    double h;

    double vibeFreq1;
    double vibeFreq2;

    Filter_Butter_LP rate_filter;
    Filter_Butter2_BS rate_BSfilter1;
    Filter_Butter2_BS rate_BSfilter2;

    Filter_Butter_LP disturbance_filter;

    // 傅里叶变换
    uint16_t ft_ind;
    double ft_in[FT_N];
    complex<double> ft_freqs[FT_N / 2];
    // 傅里叶变换参数
    static bool ft_coeffs_initialized;
    static complex<double> ft_coeffs[FT_N];

  public:
    double beta1;

    double T;
    double get_T() const
    {
        return this->T;
    };
    double b;
    double get_b() const
    {
        return this->b;
    };
    double u;
    double get_u() const
    {
        return this->u;
    };

    const complex<double> *get_ftFreqs() const
    {
        return ft_freqs;
    }

    void init_ft();

    inline void init(uint8_t order, double T, double b, double beta1, uint8_t orderD, double betaD, double Hz)
    {
        this->Hz = Hz;
        this->h = 1.0 / Hz;
        this->beta1 = beta1;
        rate_filter.set_cutoff_frequency(order, Hz, beta1);
        rate_BSfilter1.set_inavailable();
        rate_BSfilter2.set_inavailable();
        disturbance_filter.set_cutoff_frequency(orderD, Hz, betaD);

        this->z1 = this->z_inertia = 0;

        this->T = T;
        this->invT = 1.0f / T;
        this->b = b;

        vibeFreq1 = vibeFreq2 = 0;
        init_ft();
    }

    inline void update_u(double u)
    {
        this->u = u;
        this->z_inertia += this->h * this->invT * (this->b * this->u - this->z_inertia);
    }

    double run(double rate, bool inFlight);

    inline double get_EsAngularRate() const
    {
        return this->z_inertia + this->z1;
    }
    inline double get_EsDisturbance() const
    {
        return this->z1;
    }
    inline double get_EsDisturbanceFilted() const
    {
        return disturbance_filter.get_result();
    }
    inline double get_EsDisturbanceD() const
    {
        return 0;
    }
    inline double get_EsAngularAcceleration() const
    {
        return this->acc;
    }
    inline double get_EsMainPower() const
    {
        return this->z_inertia;
    }
};