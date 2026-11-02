/*
 * Núcleo matemático do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#ifndef CYCLONE_CORE_H
#define CYCLONE_CORE_H

#include "precision.h"
#include <cmath>
#include <algorithm>

namespace cyclone {

/**
 * @brief Representa um vetor de 3 dimensões (x, y, z) utilizado para posições, velocidades e forças.
 */
class Vector3 {
public:
    real x;
    real y;
    real z;

private:
    real pad; // Padding para alinhamento de memória

public:
    /**
     * @brief Construtor padrão. Inicializa o vetor como (0, 0, 0).
     */
    Vector3() : x(0), y(0), z(0) {}

    /**
     * @brief Construtor com coordenadas específicas.
     */
    Vector3(real x, real y, real z) : x(x), y(y), z(z) {}

    /**
     * @brief Inverte todas as componentes do vetor.
     */
    void invert() {
        x = -x;
        y = -y;
        z = -z;
    }

    /**
     * @brief Retorna o comprimento (magnitude) do vetor.
     */
    real magnitude() const {
        return real_sqrt(x * x + y * y + z * z);
    }

    /**
     * @brief Retorna o quadrado do comprimento do vetor (evita raiz quadrada, sendo mais rápido).
     */
    real squareMagnitude() const {
        return x * x + y * y + z * z;
    }

    /**
     * @brief Normaliza o vetor para que sua magnitude seja 1 (vetor unitário).
     */
    void normalize() {
        real l = magnitude();
        if (l > 0) {
            operator*=(((real)1) / l);
        }
    }

    /**
     * @brief Multiplica este vetor por um valor escalar.
     */
    void operator*=(const real value) {
        x *= value;
        y *= value;
        z *= value;
    }

    /**
     * @brief Retorna um novo vetor multiplicado por um escalar.
     */
    Vector3 operator*(const real value) const {
        return Vector3(x * value, y * value, z * value);
    }

    /**
     * @brief Adiciona outro vetor a este vetor.
     */
    void operator+=(const Vector3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    /**
     * @brief Retorna a soma deste vetor com outro.
     */
    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    /**
     * @brief Subtrai outro vetor deste vetor.
     */
    void operator-=(const Vector3& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    /**
     * @brief Retorna a subtração deste vetor por outro.
     */
    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    /**
     * @brief Adiciona um vetor escalado a este vetor (este + vector * scale).
     */
    void addScaledVector(const Vector3& vector, real scale) {
        x += vector.x * scale;
        y += vector.y * scale;
        z += vector.z * scale;
    }

    /**
     * @brief Calcula o produto escalar (dot product) entre dois vetores.
     */
    real operator*(const Vector3& vector) const {
        return x * vector.x + y * vector.y + z * vector.z;
    }

    /**
     * @brief Calcula o produto vetorial (cross product).
     */
    Vector3 vectorProduct(const Vector3& vector) const {
        return Vector3(
            y * vector.z - z * vector.y,
            z * vector.x - x * vector.z,
            x * vector.y - y * vector.x
        );
    }

    /**
     * @brief Operador de produto vetorial (atalho %).
     */
    Vector3 operator%(const Vector3& vector) const {
        return Vector3(
            y * vector.z - z * vector.y,
            z * vector.x - x * vector.z,
            x * vector.y - y * vector.x
        );
    }

    /**
     * @brief Atribuição do produto vetorial.
     */
    void operator%=(const Vector3& vector) {
        *this = vectorProduct(vector);
    }

    /**
     * @brief Calcula o produto componente a componente (Hadamard).
     */
    Vector3 componentProduct(const Vector3& vector) const {
        return Vector3(x * vector.x, y * vector.y, z * vector.z);
    }

    /**
     * @brief Multiplica componente a componente in-place.
     */
    void componentProductUpdate(const Vector3& vector) {
        x *= vector.x;
        y *= vector.y;
        z *= vector.z;
    }

    /**
     * @brief Calcula a distância euclidiana até outro vetor.
     */
    real distance(const Vector3& v) const {
        return (*this - v).magnitude();
    }

    /**
     * @brief Calcula o quadrado da distância euclidiana até outro vetor.
     */
    real squareDistance(const Vector3& v) const {
        return (*this - v).squareMagnitude();
    }
};

/**
 * @brief Matriz 3x3 para transformações lineares e tensores de inércia.
 */
class Matrix3 {
public:
    real data[9];

    Matrix3() {
        for (int i = 0; i < 9; i++) data[i] = 0;
    }

    Matrix3(real c0, real c1, real c2, real c3, real c4, real c5, real c6, real c7, real c8) {
        data[0] = c0; data[1] = c1; data[2] = c2;
        data[3] = c3; data[4] = c4; data[5] = c5;
        data[6] = c6; data[7] = c7; data[8] = c8;
    }

    Vector3 operator*(const Vector3& vector) const {
        return Vector3(
            data[0] * vector.x + data[1] * vector.y + data[2] * vector.z,
            data[3] * vector.x + data[4] * vector.y + data[5] * vector.z,
            data[6] * vector.x + data[7] * vector.y + data[8] * vector.z
        );
    }

    Matrix3 operator*(const Matrix3& o) const {
        return Matrix3(
            data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6],
            data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7],
            data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8],

            data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6],
            data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7],
            data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8],

            data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6],
            data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7],
            data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8]
        );
    }

    void operator*=(const Matrix3& o) {
        Matrix3 t = *this;
        *this = t * o;
    }

    void setInverse(const Matrix3& m);
    Matrix3 inverse() const;
    void setTranspose(const Matrix3& m);
    Matrix3 transpose() const;
};

/**
 * @brief Matriz 3x4 para transformações afins (rotação e translação) de corpos rígidos.
 */
class Matrix4 {
public:
    real data[12];

    Matrix4() {
        data[0] = 1; data[1] = 0; data[2] = 0; data[3] = 0;
        data[4] = 0; data[5] = 1; data[6] = 0; data[7] = 0;
        data[8] = 0; data[9] = 0; data[10] = 1; data[11] = 0;
    }

    Vector3 operator*(const Vector3& vector) const {
        return Vector3(
            data[0] * vector.x + data[1] * vector.y + data[2] * vector.z + data[3],
            data[4] * vector.x + data[5] * vector.y + data[6] * vector.z + data[7],
            data[8] * vector.x + data[9] * vector.y + data[10] * vector.z + data[11]
        );
    }

    Matrix4 operator*(const Matrix4& o) const;
    void setInverse(const Matrix4& m);
    void setOrientationAndPos(const class Quaternion& q, const Vector3& pos);
};

/**
 * @brief Quatérnion para representação de orientações e rotações 3D sem Gimbal Lock.
 */
class Quaternion {
public:
    union {
        struct {
            real r;
            real i;
            real j;
            real k;
        };
        real data[4];
    };

    Quaternion() : r(1), i(0), j(0), k(0) {}
    Quaternion(real r, real i, real j, real k) : r(r), i(i), j(j), k(k) {}

    void normalize() {
        real d = r*r + i*i + j*j + k*k;
        if (d == 0) {
            r = 1;
            return;
        }
        d = ((real)1.0) / real_sqrt(d);
        r *= d;
        i *= d;
        j *= d;
        k *= d;
    }

    void rotateByVector(const Vector3& vector) {
        Quaternion q(0, vector.x, vector.y, vector.z);
        (*this) = (*this) * q;
    }

    void operator*=(const Quaternion& multiplier) {
        Quaternion q = *this;
        r = q.r * multiplier.r - q.i * multiplier.i - q.j * multiplier.j - q.k * multiplier.k;
        i = q.r * multiplier.i + q.i * multiplier.r + q.j * multiplier.k - q.k * multiplier.j;
        j = q.r * multiplier.j + q.j * multiplier.r + q.k * multiplier.i - q.i * multiplier.k;
        k = q.r * multiplier.k + q.k * multiplier.r + q.i * multiplier.j - q.j * multiplier.i;
    }

    Quaternion operator*(const Quaternion& multiplier) const {
        Quaternion result = *this;
        result *= multiplier;
        return result;
    }
};

} // namespace cyclone

#endif // CYCLONE_CORE_H
