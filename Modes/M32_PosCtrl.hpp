#pragma once

#include "Modes.hpp"

class M32_PosCtrl : public Mode_Base
{
  private:
    // 低电量状态
    enum class LPW_STATE
    {
        // 正常不需要返航
        NORMAL = 0,
        // 低电量返航
        ND_RTL,
        // 低电量降落
        ND_LAND,
    };
    friend constexpr bool operator<(LPW_STATE a, LPW_STATE b)
    {
        return static_cast<int>(a) < static_cast<int>(b);
    }
    friend constexpr bool operator>(LPW_STATE a, LPW_STATE b)
    {
        return static_cast<int>(a) > static_cast<int>(b);
    }

  public:
    M32_PosCtrl();
    virtual void get_MavlinkMode(ModeFuncCfg cfg, Receiver rc, uint8_t btn_zones[4], AFunc *mode);
    virtual ModeResult main_func(void *param1, uint32_t param2);
};