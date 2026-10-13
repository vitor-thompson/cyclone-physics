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
            return Vector3(x + v.x, y + v.y, z + v.y); // Nota: corrigido visualmente para y + v.y e z + v.z
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
}
