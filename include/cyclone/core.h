#pragma once

#include <cyclone/precision.h>

namespace cyclone {
    /**
     * @brief Representa um vetor tridimensional (espaço 3D cartesiano).
     * 
     * Esta classe serve como a fundação matemática de todo o motor de física Cyclone,
     * armazenando coordenadas espaciais, direções e gerindo operações vetoriais básicas.
     */
    class Vector3
    {
    public:
        // --- COMPONENTES DO VETOR ---
        real x; ///< Posição ou deslocamento ao longo do eixo X.
        real y; ///< Posição ou deslocamento ao longo do eixo Y.
        real z; ///< Posição ou deslocamento ao longo do eixo Z.

    private:
        // --- OTIMIZAÇÃO DE HARDWARE ---
        real pad; ///< Espaço de preenchimento para garantir alinhamento de 4 palavras na memória da CPU.

    public:
        // --- CONSTRUTORES ---

        /**
         * @brief Construtor padrão. Inicializa o vetor como um vetor nulo (0, 0, 0).
         */
        Vector3() : x(0), y(0), z(0), pad(0) {}

        /**
         * @brief Construtor explícito. Inicializa o vetor com valores específicos.
         */
        Vector3(const real x, const real y, const real z)
            : x(x), y(y), z(z), pad(0) {}

        // --- MANIPULAÇÃO BÁSICA ---

        /**
         * @brief Inverte o sentido de todos os componentes do vetor (inversão aditiva).
         */
        void invert()
        {
            x = -x;
            y = -y;
            z = -z;
        }

        /**
         * @brief Calcula a magnitude (comprimento geométrico) do vetor usando Teorema de Pitágoras.
         */
        real magnitude() const
        {
            return real_sqrt(x * x + y * y + z * z);
        }

        /**
         * @brief Retorna o quadrado da magnitude.
         * @details Útil para otimização, evitando o uso custoso da raiz quadrada quando
         * precisamos apenas comparar distâncias relativas.
         */
        real squareMagnitude() const
        {
            return x * x + y * y + z * z;
        }

        /**
         * @brief Transforma o vetor em um vetor unitário (tamanho igual a 1), mantendo sua direção.
         */
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

        // --- MULTIPLICAÇÃO POR ESCALAR ---

        /** Multiplica diretamente este vetor por um número escalar. */
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

        // --- ADIÇÃO E SUBTRAÇÃO DE VETORES ---

        /** Adiciona outro vetor diretamente a este. */
        void operator+=(const Vector3& v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
        }

        /** Retorna a soma deste vetor com outro. */
        Vector3 operator+(const Vector3& v) const
        {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        /** Subtrai outro vetor diretamente deste. */
        void operator-=(const Vector3& v)
        {
            x -= v.x;
            y -= v.y;
            z -= v.z;
        }

        /** Retorna a diferença entre este vetor e outro. */
        Vector3 operator-(const Vector3& v) const
        {
            return Vector3(x - v.x, y - v.y, z - v.z);
        }

        // --- OPERAÇÕES OTIMIZADAS ---

        /**
         * @brief Adiciona a este vetor outro vetor previamente escalado.
         * @details Fundamental para o integrador físico: atualiza posições somando a velocidade vezes o tempo (p + v*t)
         * de forma eficiente sem gerar objetos temporários na memória.
         */
        void addScaledVector(const Vector3& vector, real scale)
        {
            x += vector.x * scale;
            y += vector.y * scale;
            z += vector.z * scale;
        }

        // --- PRODUTO POR COMPONENTES ---

        /** Retorna um novo vetor formado pela multiplicação componente a componente. */
        Vector3 componentProduct(const Vector3& vector) const
        {
            return Vector3(x * vector.x, y * vector.y, z * vector.z);
        }

        /** Atualiza este vetor multiplicando seus componentes pelos de outro vetor. */
        void componentProductUpdate(const Vector3& vector)
        {
            x *= vector.x;
            y *= vector.y;
            z *= vector.z;
        }

        // --- PRODUTO ESCALAR (DOT PRODUCT) ---

        /** Calcula e retorna o produto escalar (retorna um número escalar). */
        real scalarProduct(const Vector3& vector) const
        {
            return x * vector.x + y * vector.y + z * vector.z;
        }

        /** Sobrecarga do operador * para atuar como produto escalar entre dois vetores. */
        real operator*(const Vector3& vector) const
        {
            return x * vector.x + y * vector.y + z * vector.z;
        }

        // --- PRODUTO VETORIAL (CROSS PRODUCT) ---

        /**
         * @brief Calcula e retorna o produto vetorial (cross product).
         * @details Retorna um vetor ortogonal (perpendicular) aos dois vetores de entrada,
         * essencial para cálculos de torque e eixos de rotação.
         */
        Vector3 vectorProduct(const Vector3& vector) const
        {
            return Vector3(
                y * vector.z - z * vector.y,
                z * vector.x - x * vector.z,
                x * vector.y - y * vector.x
            );
        }

        /** Atualiza este vetor para o resultado do produto vetorial usando o operador %=. */
        void operator%=(const Vector3& vector)
        {
            *this = vectorProduct(vector);
        }

        /** Retorna o produto vetorial utilizando a sobrecarga do operador %. */
        Vector3 operator%(const Vector3& vector) const
        {
            return Vector3(
                y * vector.z - z * vector.y,
                z * vector.x - x * vector.z,
                x * vector.y - y * vector.x
            );
        }
    };

    /**
     * @brief Holds an inertia tensor, consisting of a 3x3 row-major matrix.
     */
    class Matrix3
    {
    public:
        /**
         * @brief Holds the tensor matrix data in array form (9 elements).
         */
        real data[9];

        /**
         * @brief Construtor padrão. Inicializa como matriz identidade.
         */
        Matrix3()
        {
            data[0] = 1; data[1] = 0; data[2] = 0;
            data[3] = 0; data[4] = 1; data[5] = 0;
            data[6] = 0; data[7] = 0; data[8] = 1;
        }

        /**
         * @brief Construtor com valores explícitos.
         */
        Matrix3(real c0, real c1, real c2, real c3, real c4, real c5, real c6, real c7, real c8)
        {
            data[0] = c0; data[1] = c1; data[2] = c2;
            data[3] = c3; data[4] = c4; data[5] = c5;
            data[6] = c6; data[7] = c7; data[8] = c8;
        }

        /**
         * @brief Multiplica esta matriz por um vetor 3D.
         */
        Vector3 operator*(const Vector3& vector) const
        {
            return Vector3(
                data[0] * vector.x + data[1] * vector.y + data[2] * vector.z,
                data[3] * vector.x + data[4] * vector.y + data[5] * vector.z,
                data[6] * vector.x + data[7] * vector.y + data[8] * vector.z
            );
        }

        /**
         * @brief Multiplica esta matriz por outra Matrix3.
         */
        Matrix3 operator*(const Matrix3& o) const
        {
            return Matrix3(
                data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
                data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
                data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

                data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
                data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
                data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

                data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
                data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
                data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]
            );
        }
    };

    /**
     * @brief Holds a transform matrix, consisting of a rotation matrix and
     * a position. The matrix has 12 elements (3x4).
     */
    class Matrix4
    {
    public:
        /**
         * @brief Holds the transform matrix data in array form (12 elements).
         */
        real data[12];

        /**
         * @brief Construtor padrão. Inicializa como matriz identidade de 3x4.
         */
        Matrix4()
        {
            data[0] = 1; data[1] = 0; data[2] = 0; data[3] = 0;
            data[4] = 0; data[5] = 1; data[6] = 0; data[7] = 0;
            data[8] = 0; data[9] = 0; data[10] = 1; data[11] = 0;
        }

        /**
         * @brief Transforma um vetor usando a matriz de transformação 3x4.
         */
        Vector3 operator*(const Vector3& vector) const
        {
            return Vector3(
                data[0] * vector.x + data[1] * vector.y + data[2] * vector.z + data[3],
                data[4] * vector.x + data[5] * vector.y + data[6] * vector.z + data[7],
                data[8] * vector.x + data[9] * vector.y + data[10] * vector.z + data[11]
            );
        }

        /**
         * @brief Multiplica esta matriz de transformação por outra Matrix4.
         */
        Matrix4 operator*(const Matrix4& o) const
        {
            Matrix4 result;
            result.data[0] = (o.data[0] * data[0]) + (o.data[4] * data[1]) + (o.data[8] * data[2]);
            result.data[1] = (o.data[1] * data[0]) + (o.data[5] * data[1]) + (o.data[9] * data[2]);
            result.data[2] = (o.data[2] * data[0]) + (o.data[6] * data[1]) + (o.data[10] * data[2]);
            result.data[3] = (o.data[3] * data[0]) + (o.data[7] * data[1]) + (o.data[11] * data[2]) + data[3];

            result.data[4] = (o.data[0] * data[4]) + (o.data[4] * data[5]) + (o.data[8] * data[6]);
            result.data[5] = (o.data[1] * data[4]) + (o.data[5] * data[5]) + (o.data[9] * data[6]);
            result.data[6] = (o.data[2] * data[4]) + (o.data[6] * data[5]) + (o.data[10] * data[6]);
            result.data[7] = (o.data[3] * data[4]) + (o.data[7] * data[5]) + (o.data[11] * data[6]) + data[7];

            result.data[8] = (o.data[0] * data[8]) + (o.data[4] * data[9]) + (o.data[8] * data[10]);
            result.data[9] = (o.data[1] * data[8]) + (o.data[5] * data[9]) + (o.data[9] * data[10]);
            result.data[10] = (o.data[2] * data[8]) + (o.data[6] * data[9]) + (o.data[10] * data[10]);
            result.data[11] = (o.data[3] * data[8]) + (o.data[7] * data[9]) + (o.data[11] * data[10]) + data[11];

            return result;
        }
    };
}
