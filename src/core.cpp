#include <cyclone/core.h>

namespace cyclone {

    // ==========================================
    // IMPLEMENTAÇÕES DA CLASSE MATRIX3
    // ==========================================

    void Matrix3::setInverse(const Matrix3 &m)
    {
        real t4 = m.data[0] * m.data[4];
        real t6 = m.data[0] * m.data[5];
        real t8 = m.data[1] * m.data[3];
        real t10 = m.data[2] * m.data[3];
        real t12 = m.data[1] * m.data[6];
        real t14 = m.data[2] * m.data[6];
        
        // Calcula o determinante
        real t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] +
                   t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

        // Garante que o determinante não seja zero (evita divisão por zero)
        if (t16 == (real)0.0f) return;
        real t17 = (real)1.0f / t16;
        
        data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
        data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
        data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
        data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
        data[4] = (m.data[0] * m.data[8] - t14) * t17;
        data[5] = -(t6 - t10) * t17;
        data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
        data[7] = -(m.data[0] * m.data[7] - t12) * t17;
        data[8] = (t4 - t8) * t17;
    }

    Matrix3 Matrix3::inverse() const
    {
        Matrix3 result;
        result.setInverse(*this);
        return result;
    }

    void Matrix3::invert()
    {
        setInverse(*this);
    }


    // ==========================================
    // IMPLEMENTAÇÕES DA CLASSE MATRIX4
    // ==========================================

    real Matrix4::getDeterminant() const
    {
        return data[8] * data[5] * data[2] +
               data[4] * data[9] * data[2] +
               data[8] * data[1] * data[6] -
               data[0] * data[9] * data[6] -
               data[4] * data[1] * data[10] +
               data[0] * data[5] * data[10];
    }

    void Matrix4::setInverse(const Matrix4 &m)
    {
        real det = getDeterminant();
        if (det == (real)0.0f) return;
        det = ((real)1.0f) / det;

        data[0] = (-m.data[9] * m.data[6] + m.data[5] * m.data[10]) * det;
        data[4] = (m.data[8] * m.data[6] - m.data[4] * m.data[10]) * det;
        data[8] = (-m.data[8] * m.data[5] + m.data[4] * m.data[9]) * det;

        data[1] = (m.data[9] * m.data[2] - m.data[1] * m.data[10]) * det;
        data[5] = (-m.data[8] * m.data[2] + m.data[0] * m.data[10]) * det;
        data[9] = (m.data[8] * m.data[1] - m.data[0] * m.data[9]) * det;

        data[2] = (-m.data[5] * m.data[2] + m.data[1] * m.data[6]) * det;
        data[6] = (m.data[4] * m.data[2] - m.data[0] * m.data[6]) * det;
        data[10] = (-m.data[4] * m.data[1] + m.data[0] * m.data[5]) * det;

        data[3] = (m.data[9] * m.data[6] * m.data[3]
                 - m.data[5] * m.data[10] * m.data[3]
                 - m.data[9] * m.data[2] * m.data[7]
                 + m.data[1] * m.data[10] * m.data[7]
                 + m.data[5] * m.data[2] * m.data[11]
                 - m.data[1] * m.data[6] * m.data[11]) * det;

        data[7] = (-m.data[8] * m.data[6] * m.data[3]
                 + m.data[4] * m.data[10] * m.data[3]
                 + m.data[8] * m.data[2] * m.data[7]
                 - m.data[0] * m.data[10] * m.data[7]
                 - m.data[4] * m.data[2] * m.data[11]
                 + m.data[0] * m.data[6] * m.data[11]) * det;

        data[11] = (m.data[8] * m.data[5] * m.data[3]
                  - m.data[4] * m.data[9] * m.data[3]
                  - m.data[8] * m.data[1] * m.data[7]
                  + m.data[0] * m.data[9] * m.data[7]
                  + m.data[4] * m.data[1] * m.data[11]
                  - m.data[0] * m.data[5] * m.data[11]) * det;
    }

    Matrix4 Matrix4::inverse() const
    {
        Matrix4 result;
        result.setInverse(*this);
        return result;
    }

    void Matrix4::invert()
    {
        setInverse(*this);
    }

}
