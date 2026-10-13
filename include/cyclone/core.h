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

        void operator*=(const real value)
        {
            x *= value;
            y *= value;
            z *= value;
        }

        Vector3 operator*(const real value) const
        {
            return Vector3(x * value, y * value, z * value);
        }

        void operator+=(const Vector3& v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
        }

        Vector3 operator+(const Vector3& v) const
        {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        void operator-=(const Vector3& v)
        {
            x -= v.x;
            y -= v.y;
            z -= v.z;
        }

        Vector3 operator-(const Vector3& v) const
        {
            return Vector3(x - v.x, y - v.y, z - v.z);
        }

        void addScaledVector(const Vector3& vector, real scale)
        {
            x += vector.x * scale;
            y += vector.y * scale;
            z += vector.z * scale;
        }

        // --- NOVOS MÉTODOS: PRODUTO POR COMPONENTES ---

        /** Calcula e retorna o produto componente a componente com outro vetor. */
        Vector3 componentProduct(const Vector3& vector) const
        {
            return Vector3(x * vector.x, y * vector.y, z * vector.z);
        }

        /** Atualiza este vetor multiplicando seus componentes pelos do vetor fornecido. */
        void componentProductUpdate(const Vector3& vector)
        {
            x *= vector.x;
            y *= vector.y;
            z *= vector.z;
        }

        // --- NOVOS MÉTODOS: PRODUTO ESCALAR (DOT PRODUCT) ---

        /** Calcula e retorna o produto escalar entre este vetor e outro. */
        real scalarProduct(const Vector3& vector) const
        {
            return x * vector.x + y * vector.y + z * vector.z;
        }

        /** Sobrecarga do operador * para realizar o produto escalar de forma intuitiva. */
        real operator*(const Vector3& vector) const
        {
            return x * vector.x + y * vector.y + z * vector.z;
        }
    };
}
