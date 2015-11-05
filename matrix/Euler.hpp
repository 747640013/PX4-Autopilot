/**
 * @file Euler.hpp
 *
 * Euler angle tait-bryan body 3-2-1
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once
#include <Vector.hpp>
#include <Dcm.hpp>
#include <Quaternion.hpp>

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Quaternion;

template<typename Type>
class Euler : public Vector<Type, 3>
{
public:
    virtual ~Euler() {};

    Euler() : Vector<Type, 3>()
    {
    }

    Euler(Type phi_, Type theta_, Type psi_) : Vector<Type, 3>()
    {
        phi() = phi_;
        theta() = theta_;
        psi() = psi_;
    }

    Euler(const Dcm<Type> & dcm) {
        theta() = Type(asin(-dcm(2, 0)));

        if (fabs(theta() - M_PI_2) < 1.0e-3) {
            psi() = Type(atan2(dcm(1, 2) - dcm(0, 1),
                               dcm(0, 2) + dcm(1, 1)));

        } else if (fabs(theta() + M_PI_2) < 1.0e-3) {
            psi() = Type(atan2(dcm(1, 2) - dcm(0, 1),
                               dcm(0, 2) + dcm(1, 1)));

        } else {
            phi() = Type(atan2f(dcm(2, 1), dcm(2, 2)));
            psi() = Type(atan2f(dcm(1, 0), dcm(0, 0)));
        }
    }

    Euler(const Quaternion<Type> & q) {
        *this = Euler(Dcm<Type>(q));
    }

    inline Type phi() const {
        return (*this)(0);
    }
    inline Type theta() const {
        return (*this)(1);
    }
    inline Type psi() const {
        return (*this)(2);
    }

    inline Type & phi() {
        return (*this)(0);
    }
    inline Type & theta() {
        return (*this)(1);
    }
    inline Type & psi() {
        return (*this)(2);
    }

};

typedef Euler<float> Eulerf;

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
