#pragma once

#include "AC_Math.hpp"
#include "smooth_kp.hpp"
#include <utility>
class TD4
{
  private:
  public:
    unsigned char tracking_mode;
    double x1;
    double x2;
    double x3;
    double x4;

    double P1;
    double P2;
    double P3;
    double P4;

    double r2p, r2n, r3p, r3n, r4p, r4n;

    inline unsigned char get_tracking_mode()
    {
        return tracking_mode;
    }
    inline double get_x1()
    {
        return x1;
    }
    inline double get_x2()
    {
        return x2;
    }
    inline double get_x3()
    {
        return x3;
    }
    inline double get_x4()
    {
        return x4;
    }

    inline void reset()
    {
        this->x1 = this->x2 = this->x3 = this->x4 = 0;
        tracking_mode = 0;
    }

    inline TD4()
    {
        this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e12;
        this->reset();
    }
    inline TD4(double P1, double P2, double P3, double P4)
    {
        this->P1 = P1;
        this->P2 = P2;
        this->P3 = P3;
        this->P4 = P4;
        this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e12;

        this->reset();
    }

    inline double track4(double expect, double h)
    {
        this->tracking_mode = 4;

        double e1 = expect - this->x1;
        double e1_1 = -this->x2;
        double e1_2 = -this->x3;
        double e1_3 = -this->x4;
        double T2 = this->P1 * e1;
        double P1 = 0;
        if (T2 > this->r2p)
            T2 = this->r2p;
        else if (T2 < -this->r2n)
            T2 = -this->r2n;
        else
            P1 = this->P1;
        double T2_1 = P1 * e1_1;
        double T2_2 = P1 * e1_2;
        double T2_3 = P1 * e1_3;

        double e2 = T2 - this->x2;
        double e2_1 = T2_1 - this->x3;
        double e2_2 = T2_2 - this->x4;
        double T3 = this->P2 * e2;
        double P2 = 0;
        if (T3 > this->r3p)
            T3 = this->r3p;
        else if (T3 < -this->r3n)
            T3 = -this->r3n;
        else
            P2 = this->P2;
        T3 += T2_1;
        double T3_1 = P2 * e2_1 + T2_2;
        double T3_2 = P2 * e2_2 + T2_3;

        double e3 = T3 - this->x3;
        double e3_1 = T3_1 - this->x4;
        double T4 = this->P3 * e3;
        double P3 = 0;
        if (T4 > this->r4p)
            T4 = this->r4p;
        else if (T4 < -this->r4n)
            T4 = -this->r4n;
        else
            P3 = this->P3;
        T4 += T3_1;
        double T4_1 = P3 * e3_1 + T3_2;

        double e4 = T4 - this->x4;
        double T5 = this->P4 * e4 + T4_1;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * this->x4;
        this->x4 += h * T5;

        return this->x1;
    }
    inline double track4(double expect, double expect_1, double expect_2, double expect_3, double expect_4, double h)
    {
        this->tracking_mode = 4;

        double e1 = expect - this->x1;
        double e1_1 = expect_1 - this->x2;
        double e1_2 = expect_2 - this->x3;
        double e1_3 = expect_3 - this->x4;
        double T2 = this->P1 * e1;
        double P1 = 0;
        if (T2 > this->r2p)
            T2 = this->r2p;
        else if (T2 < -this->r2n)
            T2 = -this->r2n;
        else
            P1 = this->P1;
        T2 += expect_1;
        double T2_1 = P1 * e1_1 + expect_2;
        double T2_2 = P1 * e1_2 + expect_3;
        double T2_3 = P1 * e1_3 + expect_4;

        double e2 = T2 - this->x2;
        double e2_1 = T2_1 - this->x3;
        double e2_2 = T2_2 - this->x4;
        double T3 = this->P2 * e2;
        double P2 = 0;
        if (T3 > this->r3p)
            T3 = this->r3p;
        else if (T3 < -this->r3n)
            T3 = -this->r3n;
        else
            P2 = this->P2;
        T3 += T2_1;
        double T3_1 = P2 * e2_1 + T2_2;
        double T3_2 = P2 * e2_2 + T2_3;

        double e3 = T3 - this->x3;
        double e3_1 = T3_1 - this->x4;
        double T4 = this->P3 * e3;
        double P3 = 0;
        if (T4 > this->r4p)
            T4 = this->r4p;
        else if (T4 < -this->r4n)
            T4 = -this->r4n;
        else
            P3 = this->P3;
        T4 += T3_1;
        double T4_1 = P3 * e3_1 + T3_2;

        double e4 = T4 - this->x4;
        double T5 = this->P4 * e4 + T4_1;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * this->x4;
        this->x4 += h * T5;

        return this->x1;
    }

    inline double track3(double expect, double h)
    {
        this->tracking_mode = 3;

        double e2 = expect - this->x2;
        double e2_1 = -this->x3;
        double e2_2 = -this->x4;
        double T3 = this->P2 * e2;
        double P2 = 0;
        if (T3 > this->r3p)
            T3 = this->r3p;
        else if (T3 < -this->r3n)
            T3 = -this->r3n;
        else
            P2 = this->P2;
        double T3_1 = P2 * e2_1;
        double T3_2 = P2 * e2_2;

        double e3 = T3 - this->x3;
        double e3_1 = T3_1 - this->x4;
        double T4 = this->P3 * e3;
        double P3 = 0;
        if (T4 > this->r4p)
            T4 = this->r4p;
        else if (T4 < -this->r4n)
            T4 = -this->r4n;
        else
            P3 = this->P3;
        T4 += T3_1;
        double T4_1 = P3 * e3_1 + T3_2;

        double e4 = T4 - this->x4;
        double T5 = this->P4 * e4 + T4_1;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * this->x4;
        this->x4 += h * T5;

        return this->x2;
    }
};

class TD4_Lite
{
  private:
  public:
    unsigned char tracking_mode;
    double x1;
    double x2;
    double x3;
    double x4;

    inline unsigned char get_tracking_mode()
    {
        return tracking_mode;
    }
    inline double get_x1()
    {
        return x1;
    }
    inline double get_x2()
    {
        return x2;
    }
    inline double get_x3()
    {
        return x3;
    }
    inline double get_x4()
    {
        return x4;
    }

    inline void reset()
    {
        this->x1 = this->x2 = this->x3 = this->x4 = 0;
        tracking_mode = 0;
    }

    inline TD4_Lite()
    {
        this->reset();
    }

    inline double track4(double expect, double h, double P1, double P2, double P3, double P4)
    {
        this->tracking_mode = 4;

        double e1 = expect - this->x1;
        double e1_1 = -this->x2;
        double e1_2 = -this->x3;
        double e1_3 = -this->x4;
        double T2 = P1 * e1;
        double T2_1 = P1 * e1_1;
        double T2_2 = P1 * e1_2;
        double T2_3 = P1 * e1_3;

        double e2 = T2 - this->x2;
        double e2_1 = T2_1 - this->x3;
        double e2_2 = T2_2 - this->x4;
        double T3 = P2 * e2;
        T3 += T2_1;
        double T3_1 = P2 * e2_1 + T2_2;
        double T3_2 = P2 * e2_2 + T2_3;

        double e3 = T3 - this->x3;
        double e3_1 = T3_1 - this->x4;
        double T4 = P3 * e3;
        T4 += T3_1;
        double T4_1 = P3 * e3_1 + T3_2;

        double e4 = T4 - this->x4;
        double T5 = P4 * e4 + T4_1;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * this->x4;
        this->x4 += h * T5;

        return this->x1;
    }

    inline double track3(double expect, double h, double P2, double P3, double P4)
    {
        this->tracking_mode = 3;

        double e2 = expect - this->x2;
        double e2_1 = -this->x3;
        double e2_2 = -this->x4;
        double T3 = P2 * e2;
        double T3_1 = P2 * e2_1;
        double T3_2 = P2 * e2_2;

        double e3 = T3 - this->x3;
        double e3_1 = T3_1 - this->x4;
        double T4 = P3 * e3;
        T4 += T3_1;
        double T4_1 = P3 * e3_1 + T3_2;

        double e4 = T4 - this->x4;
        double T5 = P4 * e4 + T4_1;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * this->x4;
        this->x4 += h * T5;

        return this->x2;
    }
};

class TD3_Lite
{
  private:
  public:
    unsigned char tracking_mode;
    double x1;
    double x2;
    double x3;

    inline unsigned char get_tracking_mode()
    {
        return tracking_mode;
    }
    inline double get_x1()
    {
        return x1;
    }
    inline double get_x2()
    {
        return x2;
    }
    inline double get_x3()
    {
        return x3;
    }

    inline void reset()
    {
        this->x1 = this->x2 = this->x3 = 0;
        tracking_mode = 0;
    }

    inline TD3_Lite()
    {
        this->reset();
    }

    inline double track3(double expect, double h, double P1, double P2, double P3)
    {
        this->tracking_mode = 3;

        double e1 = expect - this->x1;
        double e1_1 = -this->x2;
        double e1_2 = -this->x3;
        double T2 = P1 * e1;
        double T2_1 = P1 * e1_1;
        double T2_2 = P1 * e1_2;

        double e2 = T2 - this->x2;
        double e2_1 = T2_1 - this->x3;
        double T3 = P2 * e2;
        T3 += T2_1;
        double T3_1 = P2 * e2_1 + T2_2;

        double e3 = T3 - this->x3;
        double T4 = P3 * e3;
        T4 += T3_1;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * T4;

        return this->x1;
    }
};

class TD4_SL
{
  private:
  public:
    unsigned char tracking_mode;
    double x1;
    double x2;
    double x3;
    double x4;
    double T5;

    double P1;
    double P2;
    double P3;
    double P4;

    double r2p, r2n, r3p, r3n, r4p, r4n;

    inline unsigned char get_tracking_mode()
    {
        return tracking_mode;
    }
    inline double get_x1()
    {
        return x1;
    }
    inline double get_x2()
    {
        return x2;
    }
    inline double get_x3()
    {
        return x3;
    }
    inline double get_x4()
    {
        return x4;
    }

    inline void reset()
    {
        this->x1 = this->x2 = this->x3 = this->x4 = 0;
        tracking_mode = 0;
    }

    inline TD4_SL()
    {
        this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e5;
        this->reset();
    }
    inline TD4_SL(double P1, double P2, double P3, double P4)
    {
        this->P1 = P1;
        this->P2 = P2;
        this->P3 = P3;
        this->P4 = P4;
        this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e5;

        this->reset();
    }

    inline double track4(double expect, double h)
    {
        this->tracking_mode = 4;

        double r;

        double e1 = expect - this->x1;
        double e1_1 = -this->x2;
        double e1_2 = -this->x3;
        double e1_3 = -this->x4;
        if (e1 > 0)
            r = r2p;
        else
            r = r2n;
        smooth_kp_d3 d1 = smooth_kp_3(e1, e1_1, e1_2, e1_3, this->P1, r);
        double T2 = d1.d0;
        double T2_1 = d1.d1;
        double T2_2 = d1.d2;
        double T2_3 = d1.d3;

        double e2 = T2 - this->x2;
        double e2_1 = T2_1 - this->x3;
        double e2_2 = T2_2 - this->x4;
        if (e2 > 0)
            r = r3p;
        else
            r = r3n;
        smooth_kp_d2 d2 = smooth_kp_2(e2, e2_1, e2_2, this->P2, r);
        double T3 = d2.d0 + T2_1;
        double T3_1 = d2.d1 + T2_2;
        double T3_2 = d2.d2 + T2_3;

        double e3 = T3 - this->x3;
        double e3_1 = T3_1 - this->x4;
        if (e3 > 0)
            r = r4p;
        else
            r = r4n;
        smooth_kp_d1 d3 = smooth_kp_1(e3, e3_1, this->P3, r);
        double T4 = d3.d0 + T3_1;
        double T4_1 = d3.d1 + T3_2;

        double e4 = T4 - this->x4;
        T5 = this->P4 * e4 + T4_1;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * this->x4;
        this->x4 += h * T5;

        return this->x1;
    }
    inline double track4(double expect, double expect_1, double expect_2, double expect_3, double expect_4, double h)
    {
        this->tracking_mode = 4;

        double r;

        double e1 = expect - this->x1;
        double e1_1 = expect_1 - this->x2;
        double e1_2 = expect_2 - this->x3;
        double e1_3 = expect_3 - this->x4;
        if (e1 > 0)
            r = r2p;
        else
            r = r2n;
        smooth_kp_d3 d1 = smooth_kp_3(e1, e1_1, e1_2, e1_3, this->P1, r);
        double T2 = d1.d0 + expect_1;
        double T2_1 = d1.d1 + expect_2;
        double T2_2 = d1.d2 + expect_3;
        double T2_3 = d1.d3 + expect_4;

        double e2 = T2 - this->x2;
        double e2_1 = T2_1 - this->x3;
        double e2_2 = T2_2 - this->x4;
        if (e2 > 0)
            r = r3p;
        else
            r = r3n;
        smooth_kp_d2 d2 = smooth_kp_2(e2, e2_1, e2_2, this->P2, r);
        double T3 = d2.d0 + T2_1;
        double T3_1 = d2.d1 + T2_2;
        double T3_2 = d2.d2 + T2_3;

        double e3 = T3 - this->x3;
        double e3_1 = T3_1 - this->x4;
        if (e3 > 0)
            r = r4p;
        else
            r = r4n;
        smooth_kp_d1 d3 = smooth_kp_1(e3, e3_1, this->P3, r);
        double T4 = d3.d0 + T3_1;
        double T4_1 = d3.d1 + T3_2;

        double e4 = T4 - this->x4;
        T5 = this->P4 * e4 + T4_1;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * this->x4;
        this->x4 += h * T5;

        return this->x1;
    }

    inline double track3(double expect, double h)
    {
        this->tracking_mode = 3;
        double r;

        double e2 = expect - this->x2;
        double e2_1 = -this->x3;
        double e2_2 = -this->x4;
        if (e2 > 0)
            r = r3p;
        else
            r = r3n;
        smooth_kp_d2 d2 = smooth_kp_2(e2, e2_1, e2_2, this->P2, r);
        double T3 = d2.d0;
        double T3_1 = d2.d1;
        double T3_2 = d2.d2;

        double e3 = T3 - this->x3;
        double e3_1 = T3_1 - this->x4;
        if (e3 > 0)
            r = r4p;
        else
            r = r4n;
        smooth_kp_d1 d3 = smooth_kp_1(e3, e3_1, this->P3, r);
        double T4 = d3.d0 + T3_1;
        double T4_1 = d3.d1 + T3_2;

        double e4 = T4 - this->x4;
        T5 = this->P4 * e4 + T4_1;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * this->x4;
        this->x4 += h * T5;

        return this->x2;
    }
    inline double track3(double expect, double expect_1, double expect_2, double expect_3, double h)
    {
        this->tracking_mode = 3;
        double r;

        double e2 = expect - this->x2;
        double e2_1 = expect_1 - this->x3;
        double e2_2 = expect_2 - this->x4;
        if (e2 > 0)
            r = r3p;
        else
            r = r3n;
        smooth_kp_d2 d2 = smooth_kp_2(e2, e2_1, e2_2, this->P2, r);
        double T3 = d2.d0 + expect_1;
        double T3_1 = d2.d1 + expect_2;
        double T3_2 = d2.d2 + expect_3;

        double e3 = T3 - this->x3;
        double e3_1 = T3_1 - this->x4;
        if (e3 > 0)
            r = r4p;
        else
            r = r4n;
        smooth_kp_d1 d3 = smooth_kp_1(e3, e3_1, this->P3, r);
        double T4 = d3.d0 + T3_1;
        double T4_1 = d3.d1 + T3_2;

        double e4 = T4 - this->x4;
        T5 = this->P4 * e4 + T4_1;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * this->x4;
        this->x4 += h * T5;

        return this->x2;
    }

    inline double track4_order3(double expect, double h)
    {
        this->tracking_mode = 4;

        double r;

        double e1 = expect - this->x1;
        double e1_1 = -this->x2;
        double e1_2 = -this->x3;
        if (e1 > 0)
            r = r2p;
        else
            r = r2n;
        smooth_kp_d2 d1 = smooth_kp_2(e1, e1_1, e1_2, this->P1, r);
        double T2 = d1.d0;
        double T2_1 = d1.d1;
        double T2_2 = d1.d2;

        double e2 = T2 - this->x2;
        double e2_1 = T2_1 - this->x3;
        if (e2 > 0)
            r = r3p;
        else
            r = r3n;
        smooth_kp_d1 d2 = smooth_kp_1(e2, e2_1, this->P2, r);
        double T3 = d2.d0 + T2_1;
        double T3_1 = d2.d1 + T2_2;

        double e3 = T3 - this->x3;
        double T4 = this->P3 * e3 + T3_1;
        this->x4 = T4;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * T4;

        return this->x1;
    }
    inline double track4_order3(double expect, double expect_1, double expect_2, double expect_3, double h)
    {
        this->tracking_mode = 4;

        double r;

        double e1 = expect - this->x1;
        double e1_1 = expect_1 - this->x2;
        double e1_2 = expect_2 - this->x3;
        if (e1 > 0)
            r = r2p;
        else
            r = r2n;
        smooth_kp_d2 d1 = smooth_kp_2(e1, e1_1, e1_2, this->P1, r);
        double T2 = d1.d0 + expect_1;
        double T2_1 = d1.d1 + expect_2;
        double T2_2 = d1.d2 + expect_3;

        double e2 = T2 - this->x2;
        double e2_1 = T2_1 - this->x3;
        if (e2 > 0)
            r = r3p;
        else
            r = r3n;
        smooth_kp_d1 d2 = smooth_kp_1(e2, e2_1, this->P2, r);
        double T3 = d2.d0 + T2_1;
        double T3_1 = d2.d1 + T2_2;

        double e3 = T3 - this->x3;
        double T4 = this->P3 * e3 + T3_1;
        this->x4 = T4;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * T4;

        return this->x1;
    }

    inline double track3_order3(double expect, double h)
    {
        this->tracking_mode = 4;

        double r;

        double e2 = expect - this->x2;
        double e2_1 = -this->x3;
        if (e2 > 0)
            r = r3p;
        else
            r = r3n;
        smooth_kp_d1 d2 = smooth_kp_1(e2, e2_1, this->P2, r);
        double T3 = d2.d0;
        double T3_1 = d2.d1;

        double e3 = T3 - this->x3;
        double T4 = this->P3 * e3 + T3_1;
        this->x4 = T4;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * T4;

        return this->x1;
    }
    inline double track3_order3(double expect, double expect_1, double expect_2, double h)
    {
        this->tracking_mode = 4;

        double r;

        double e2 = expect - this->x2;
        double e2_1 = expect_1 - this->x3;
        if (e2 > 0)
            r = r3p;
        else
            r = r3n;
        smooth_kp_d1 d2 = smooth_kp_1(e2, e2_1, this->P2, r);
        double T3 = d2.d0 + expect_1;
        double T3_1 = d2.d1 + expect_2;

        double e3 = T3 - this->x3;
        double T4 = this->P3 * e3 + T3_1;
        this->x4 = T4;

        this->x1 += h * this->x2;
        this->x2 += h * this->x3;
        this->x3 += h * T4;

        return this->x1;
    }
};

// 小窗口最小二乘 TD4 反演控制器（模板参数决定窗口大小）
template <int M> class RealTimeTD4Inverter
{
  private:
    // 单步周期
    double h;
    // TD4仿真周期
    const double rh = 1.0 / 200;

    // 是否完成初始化
    bool inited;
    // TD4系数
    double P1, P2, P3, P4;
    // 输出大小惩罚率
    double lambda;
    // 输出变化率惩罚率
    double lambda_du;
    // 各个点权重
    double w[M];

    // 脉冲响应（单位输入后的 x1 序列）
    double impulse_response[M];
    // 下三角 Toeplitz 矩阵（固定）
    double H[M][M];
    double v[M];

    // 计算脉冲响应（单位输入在第0步）
    // 并求解最优h
    void compute_impulse_response()
    {
        // 模拟TD
        TD4_Lite td;
        // 当前模拟时间
        double cur_h = 0;
        // 顶点时间
        double vertex_h = 0;
        // 顶点高度
        double vertex = 0;
        // 上一个点高度(用于判断顶点)
        double last_x1;
        // 顶点已找到
        bool vertex_found = false;
        // h已找到
        bool h_found = false;
        while (cur_h < 10)
        {
            // 积分计算当前模拟时间
            cur_h += rh;
            // 模拟脉冲响应
            if (cur_h < 0.077)
            {
                // 模拟短时间高脉冲
                double td4_x1 = td.track4(1.0, rh, P1, P2, P3, P4);
                last_x1 = td4_x1;
            }
            else
            {
                // 找顶点
                double td4_x1 = td.track4(0.0, rh, P1, P2, P3, P4);
                if (!vertex_found)
                { // 记录顶点
                    if (td4_x1 < last_x1)
                    {
                        vertex_found = true;
                        vertex_h = cur_h - rh;
                        vertex = last_x1;
                    }
                }
                else
                { // 顶点已记录
                    // 找末端点 计算周期h
                    if (td4_x1 < vertex * 0.15)
                    {
                        h = cur_h / (double) (M);
                        h_found = true;
                        break;
                    }
                }
                last_x1 = td4_x1;
            }
        }

        if (!h_found)
        {
            inited = false;
            return;
        }

        // 根据周期h模拟脉冲响应
        cur_h = 0;
        uint16_t cur_M = 0;
        td.reset();
        while (cur_M < M)
        {
            double td4_rs = td.track4((cur_M == 0) ? 1.0 : 0.0, rh, P1, P2, P3, P4);
            cur_h += rh;
            if (cur_h >= h)
            {
                cur_h -= h;
                impulse_response[cur_M] = td4_rs;
                ++cur_M;
            }
        }
        inited = true;
    }

    // 构建 H 矩阵（下三角 Toeplitz）
    void build_H()
    {
        memset(H, 0, sizeof(H));
        for (int i = 0; i < M; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                H[i][j] = impulse_response[i - j];
            }
        }
    }

    // 从当前状态开始，模拟未来 M 步（输入全为0），得到自由响应
    void predict_free_response(const TD4_Lite &start_td, double x1_free[M])
    {
        TD4_Lite sim = start_td; // 拷贝当前状态
        double cur_h = 0;
        uint16_t cur_M = 0;
        while (cur_M < M)
        {
            double td4_rs = sim.track4(0, rh, P1, P2, P3, P4);
            cur_h += rh;
            if (cur_h >= h)
            {
                cur_h -= h;
                x1_free[cur_M] = td4_rs;
                ++cur_M;
            }
        }
    }

    // 重写后的 solve_linear 函数（更稳定版本）
    bool solve_linear(double A_in[M][M], const double c_in[M], double x[M])
    {
        double mat[M][M + 1];
        int pivot[M]; // 新增：跟踪行置换，提高稳定

        // 初始化 pivot 和拷贝矩阵
        for (int i = 0; i < M; ++i)
        {
            pivot[i] = i; // 初始行顺序
            for (int j = 0; j < M; ++j)
            {
                mat[i][j] = A_in[i][j];
            }
            mat[i][M] = c_in[i];
        }

        const double epsilon = 1e-12; // 小主元阈值

        // 高斯消元 + 部分主元
        for (int col = 0; col < M; ++col)
        {
            // 找主元行（从 col 开始向下搜索最大绝对值）
            int max_row = col;
            for (int row = col + 1; row < M; ++row)
            {
                if (fabs(mat[pivot[row]][col]) > fabs(mat[pivot[max_row]][col]))
                {
                    max_row = row;
                }
            }

            // 交换 pivot（行索引交换，而不是直接交换矩阵，提高效率）
            std::swap(pivot[col], pivot[max_row]);

            // 如果主元太小，加小正则到对角（稳定处理，计算量稍增）
            if (fabs(mat[pivot[col]][col]) < epsilon)
            {
                mat[pivot[col]][col] += 1e-10; // 小扰动，避免除零/不稳
                if (fabs(mat[pivot[col]][col]) < epsilon)
                {
                    // 严重病态，返回 false（可加日志：printf("Matrix singular!\n");）
                    memset(x, 0, sizeof(double) * M);
                    return false;
                }
            }

            // 消元（对后续行）
            for (int row = col + 1; row < M; ++row)
            {
                double factor = mat[pivot[row]][col] / mat[pivot[col]][col];
                for (int j = col; j <= M; ++j)
                { // 从 col 开始（已知前部为0，但安全起见）
                    mat[pivot[row]][j] -= factor * mat[pivot[col]][j];
                }
            }
        }

        // 回代（从后向前）
        for (int i = M - 1; i >= 0; --i)
        {
            double sum = 0.0;
            for (int j = i + 1; j < M; ++j)
            {
                sum += mat[pivot[i]][j] * x[j];
            }
            double denom = mat[pivot[i]][i];
            if (fabs(denom) < epsilon)
            {
                // 回代中发现小分母，返回 false
                memset(x, 0, sizeof(double) * M);
                return false;
            }
            x[i] = (mat[pivot[i]][M] - sum) / denom;
        }

        return true;
    }

  public:
    // 构造函数
    explicit RealTimeTD4Inverter()
    {
        reset();
    }
    // 获取单步h
    double get_h()
    {
        return h;
    }

    // 重置（如果需要重置内部状态，可调用）
    void reset()
    {
        for (uint8_t i = 0; i < M; ++i)
            w[i] = 1;
        h = 0;
        inited = false;
    }
    void reset_params(double p1, double p2, double p3, double p4, double lambda_du = 0.1, double lambda = 0)
    {
        reset();

        this->P1 = p1;
        this->P2 = p2;
        this->P3 = p3;
        this->P4 = p4;
        this->lambda_du = lambda_du;
        this->lambda = lambda;

        compute_impulse_response();
        build_H();
    }

    // 核心函数：计算当前应该给出的 expect 值
    // y_future：长度至少 M 的未来参考轨迹数组
    //   若传入 nullptr，则全部使用 current_td.x1 作为参考（保守方案）
    double compute_expect(const TD4_Lite &current_td, const double y_future[M] = nullptr)
    {
        // 自由响应（未来不加输入时的 x1）
        double x1_free[M];
        // Hessian 矩阵
        double A[M][M];
        // 梯度向量（H^T (y - x_free)）
        double c[M];

        // 计算自由相应
        predict_free_response(current_td, x1_free);

        // 计算目标偏差向量
        double b[M];
        if (y_future)
        {
            for (int i = 0; i < M; ++i)
            {
                b[i] = y_future[i] - x1_free[i];
            }
        }
        else
        {
            // 没有未来轨迹，用当前 TD4 的 x1 作为所有未来目标（保守）
            for (int i = 0; i < M; ++i)
            {
                b[i] = current_td.x1 - x1_free[i];
            }
        }

        // 计算权重 w_i = exp(-i / tau)，并应用到 A 和 c
        // double weight_tau = (double)M / 2;
        // double w[M];
        // for (int i = 0; i < M; ++i) {
        //     w[i] = std::exp(-static_cast<double>(i) / weight_tau);  // 前大后小，w[0]=1, w[M-1]小
        // }

        // 构建 A = H^T (W^2) H + λ I
        //       c = H^T (W^2) b
        // 注意：W 是对角，W^2[i] = w[i]^2
        memset(A, 0, sizeof(A));
        memset(c, 0, sizeof(c));
        for (int k = 0; k < M; ++k)
        {
            for (int i = 0; i < M; ++i)
            {
                double Hik = H[i][k];
                double wi2 = w[i] * w[i]; // w_i^2
                c[k] += Hik * (wi2 * b[i]);
                for (int l = 0; l < M; ++l)
                {
                    A[k][l] += Hik * (wi2 * H[i][l]);
                }
            }
            A[k][k] += lambda;
        }

        // 加变化率惩罚
        if (lambda_du > 0.0)
        {
            for (int k = 0; k < M; ++k)
            {
                A[k][k] += lambda_du * 2.0;
                if (k - 1 >= 0)
                    A[k][k - 1] -= lambda_du;
                if (k + 1 < M)
                    A[k][k + 1] -= lambda_du;
            }
        }

        // double v[M] = {0};
        memset(v, 0, sizeof(double) * M);
        if (!solve_linear(A, c, v))
        {
            // 矩阵奇异或数值问题，返回 0 或上一次的值（这里简单返回0）
            return 0.0;
        }

        return v[0]; // 只返回第一个控制量（receding horizon 原则）
    }

    void run_td4(TD4_Lite &start_td, double expect) const
    {
        double cur_h = 0;
        while (cur_h < h)
        {
            start_td.track4(expect, rh, P1, P2, P3, P4);
            cur_h += rh;
        }
    }
};