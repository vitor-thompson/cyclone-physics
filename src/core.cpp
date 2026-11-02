/*
 * core.cpp
 * 
 * Este arquivo contém a implementação dos métodos das classes matemáticas fundamentais
 * do motor físico Cyclone (Vector3, Matrix3, Matrix4 e Quaternion).
 * Inclui transformações de coordenadas, inversão de matrizes e produtos vetoriais.
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/core.h>

namespace cyclone {

/*
 * =========================================================
 * CONSTANTES DE EIXOS PADRÃO
 * =========================================================
 */
const Vector3 Vector3::GRAVITY = Vector3(0, -9.81, 0);
const Vector3 Vector3::HIGH_GRAVITY = Vector3(0, -19.62, 0);
const Vector3 Vector3::UP = Vector3(0, 1, 0);
const Vector3 Vector3::RIGHT = Vector3(1, 0, 0);
const Vector3 Vector3::FORWARD = Vector3(0, 0, 1);
const Vector3 Vector3::X = Vector3(1, 0, 0);
const Vector3 Vector3::Y = Vector3(0, 1, 0);
const Vector3 Vector3::Z = Vector3(0, 0, 1);

/*
 * =========================================================
 * IMPLEMENTAÇÃO DA CLASSE MATRIX3
 * =========================================================
 */

void Matrix3::setInverse(const Matrix3 &m) {
    real t4 = m.data[0] * m.data[4];
    real t6 = m.data[0] * m.data[5];
    real t8 = m.data[1] * m.data[3];
    real t10 = m.data[2] * m.data[3];
    real t12 = m.data[1] * m.data[6];
    real t14 = m.data[2] * m.data[6];

    // Calcula o determinante da matriz 3x3
    real det = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] +
                t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

    // Se o determinante for zero, a matriz é singular e não possui inversa
    if (det == (real)0.0) return;

    real invd = ((real)1.0) / det;

    data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * invd;
    data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * invd;
    data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * invd;
    data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * invd;
    data[4] = (m.data[0] * m.data[8] - m.data[2] * m.data[6]) * invd;
    data[5] = -(m.data[0] * m.data[5] - m.data[2] * m.data[3]) * invd;
    data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * invd;
    data[7] = -(m.data[0] * m.data[7] - m.data[1] * m.data[6]) * invd;
    data[8] = (m.data[0] * m.data[4] - m.data[1] * m.data[3]) * invd;
}

Matrix3 Matrix3::inverse() const {
    Matrix3 result;
    result.setInverse(*this);
    return result;
}

void Matrix3::setTranspose(const Matrix3 &m) {
    data[0] = m.data[0];
    data[1] = m.data[3];
    data[2] = m.data[6];
    data[3] = m.data[1];
    data[4] = m.data[4];
    data[5] = m.data[7];
    data[6] = m.data[2];
    data[7] = m.data[5];
    data[8] = m.data[8];
}

Matrix3 Matrix3::transpose() const {
    Matrix3 result;
    result.setTranspose(*this);
    return result;
}

void Matrix3::setOrientation(const Quaternion &q) {
    data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
    data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
    data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
    data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
    data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
    data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
    data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
    data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
    data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DA CLASSE MATRIX4
 * =========================================================
 */

Matrix4 Matrix4::operator*(const Matrix4 &o) const {
    Matrix4 result;
    result.data[0] = o.data[0]*data[0] + o.data[4]*data[1] + o.data[8]*data[2];
    result.data[4] = o.data[0]*data[4] + o.data[4]*data[5] + o.data[8]*data[6];
    result.data[8] = o.data[0]*data[8] + o.data[4]*data[9] + o.data[8]*data[10];

    result.data[1] = o.data[1]*data[0] + o.data[5]*data[1] + o.data[9]*data[2];
    result.data[5] = o.data[1]*data[4] + o.data[5]*data[5] + o.data[9]*data[6];
    result.data[9] = o.data[1]*data[8] + o.data[5]*data[9] + o.data[9]*data[10];

    result.data[2] = o.data[2]*data[0] + o.data[6]*data[1] + o.data[10]*data[2];
    result.data[6] = o.data[2]*data[4] + o.data[6]*data[5] + o.data[10]*data[6];
    result.data[10] = o.data[2]*data[8] + o.data[6]*data[9] + o.data[10]*data[10];

    result.data[3] = o.data[3]*data[0] + o.data[7]*data[1] + o.data[11]*data[2] + data[3];
    result.data[7] = o.data[3]*data[4] + o.data[7]*data[5] + o.data[11]*data[6] + data[7];
    result.data[11] = o.data[3]*data[8] + o.data[7]*data[9] + o.data[11]*data[10] + data[11];

    return result;
}

void Matrix4::setInverse(const Matrix4 &m) {
    // Certifica-se de tratar a matriz de transformação afim (3x4)
    real det = m.getDeterminant();
    if (det == (real)0.0) return;
    real invDet = ((real)1.0) / det;

    data[0] = (-m.data[9] * m.data[6] + m.data[5] * m.data[10]) * invDet;
    data[4] = (m.data[8] * m.data[6] - m.data[4] * m.data[10]) * invDet;
    data[8] = (-m.data[8] * m.data[5] + m.data[4] * m.data[9]) * invDet;

    data[1] = (m.data[9] * m.data[2] - m.data[1] * m.data[10]) * invDet;
    data[5] = (-m.data[8] * m.data[2] + m.data[0] * m.data[10]) * invDet;
    data[9] = (m.data[8] * m.data[1] - m.data[0] * m.data[9]) * invDet;

    data[2] = (-m.data[5] * m.data[2] + m.data[1] * m.data[6]) * invDet;
    data[6] = (m.data[4] * m.data[2] - m.data[0] * m.data[6]) * invDet;
    data[10] = (-m.data[4] * m.data[1] + m.data[0] * m.data[5]) * invDet;

    data[3] = (m.data[9] * m.data[6] * m.data[3]
               - m.data[5] * m.data[10] * m.data[3]
               - m.data[9] * m.data[2] * m.data[7]
               + m.data[1] * m.data[10] * m.data[7]
               + m.data[5] * m.data[2] * m.data[11]
               - m.data[1] * m.data[6] * m.data[11]) * invDet;

    data[7] = (-m.data[8] * m.data[6] * m.data[3]
               + m.data[4] * m.data[10] * m.data[3]
               + m.data[8] * m.data[2] * m.data[7]
               - m.data[0] * m.data[10] * m.data[7]
               - m.data[4] * m.data[2] * m.data[11]
               + m.data[0] * m.data[6] * m.data[11]) * invDet;

    data[11] = (m.data[8] * m.data[5] * m.data[3]
                - m.data[4] * m.data[9] * m.data[3]
                - m.data[8] * m.data[1] * m.data[7]
                + m.data[0] * m.data[9] * m.data[7]
                + m.data[4] * m.data[1] * m.data[11]
                - m.data[0] * m.data[5] * m.data[11]) * invDet;
}

void Matrix4::setOrientationAndPos(const Quaternion &q, const Vector3 &pos) {
    data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
    data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
    data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
    data[3] = pos.x;

    data[4] = 2 * q.i * q.j - 2 * q.k * q.r;
    data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
    data[6] = 2 * q.j * q.k + 2 * q.i * q.r;
    data[7] = pos.y;

    data[8] = 2 * q.i * q.k + 2 * q.j * q.r;
    data[9] = 2 * q.j * q.k - 2 * q.i * q.r;
    data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
    data[11] = pos.z;
}

} // namespace cyclone
