#pragma once

#include <cyclone/precision.h>

namespace cyclone {
    /**
     * Armazena um vetor em 3 dimensões. Quatro membros de dados são alocados
     * para garantir o alinhamento de memória em blocos otimizados.
     */
    class Vector3
    {
    public:
        /** Componente ao longo do eixo x. */
        real x;
        /** Componente ao longo do eixo y. */
        real y;
        /** Componente ao longo do eixo z. */
        real z;

    private:
        /** Preenchimento (padding) para garantir alinhamento de hardware. */
        real pad;

    public:
        /** O construtor padrão cria um vetor nulo (0, 0, 0). */
        Vector3() : x(0), y(0), z(0), pad(0) {}

        /** O construtor explícito cria um vetor com os componentes fornecidos. */
        Vector3(const real x, const real y, const real z)
            : x(x), y(y), z(z), pad(0) {}

        /** Inverte todos os componentes do vetor. */
        void invert()
        {
            x = -x;
            y = -y;
            z = -z;
        }

        /** Retorna a magnitude (comprimento) do vetor. */
        real magnitude() const
        {
            return real_sqrt(x * x + y * y + z * z);
        }

        /** Retorna o quadrado da magnitude (evita o uso custoso de raiz quadrada). */
        real squareMagnitude() const
        {
            return x * x + y * y + z * z;
        }

        /** Transforma um vetor não-nulo em um vetor unitário (comprimento 1). */
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
    };
}
