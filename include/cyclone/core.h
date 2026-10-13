#pragma once

#include <cyclone/precision.h>

namespace cyclone {
    class Vector3
    {
    public:
        real x;
        real y;
        real z;

    private:
        real pad;

    public:
        Vector3() : x(0), y(0), z(0), pad(0) {}

        Vector3(const real x, const real y, const real z)
            : x(x), y(y), z(z), pad(0) {}

        void invert()
        {
            x = -x;
            y = -y;
            z = -z;
        }

        real magnitude() const
        {
            return real_sqrt(x * x + y * y + z * z);
        }

        real squareMagnitude() const
        {
            return x * x + y * y + z * z;
        }

        void normalize()
        {
            real l = magnitude();
            if (l > 0.0f)
            {
                real inverse = 1.0f / l;
                x *= inverse;
                y *= inverse;
                z *= inverse;
            }
        }

        // --- NOVOS MÉTODOS DE MULTIPLICAÇÃO POR ESCALAR ---

        /** Multiplica este vetor por um escalar (atribuição multiplicativa). */
        void operator*=(const real value)
        {
            x *= value;
            y *= value;
            z *= value;
        }

        /** Retorna uma nova cópia do vetor multiplicada por um escalar. */
        Vector3 operator*(const real value) const
        {
            return Vector3(x * value, y * value, z * value);
        }
    };
}
