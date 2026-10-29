/*
 * Núcleo matemático do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#ifndef CYCLONE_CORE_H
#define CYCLONE_CORE_H

#include <math.h>

namespace cyclone {

    // Definição do tipo real (pode ser float ou double)
    typedef float real;

    // ... (demais definições de Vector3, Matrix4, etc. mantêm-se iguais) ...

    /**
     * Armazena um tensor de inércia ou matriz de rotação 3x3 em ordem row-major.
     */
    class Matrix3
    {
    public:
        real data[9];

        Matrix3()
        {
            data[0] = data[1] = data[2] = data[3] = data[4] =
            data[5] = data[6] = data[7] = data[8] = 0;
        }

        Matrix3(real c0, real c1, real c2, real c3, real c4, real c5, real c6, real c7, real c8)
        {
            data[0] = c0; data[1] = c1; data[2] = c2;
            data[3] = c3; data[4] = c4; data[5] = c5;
            data[6] = c6; data[7] = c7; data[8] = c8;
        }

        /**
         * Define os valores da matriz a partir de três vetores componentes.
         * Cada vetor é organizado como uma coluna da matriz (espaço de contato para o mundo).
         */
        void setComponents(const Vector3 &compOne, const Vector3 &compTwo,
                           const Vector3 &compThree)
        {
            data[0] = compOne.x;
            data[1] = compTwo.x;
            data[2] = compThree.x;
            data[3] = compOne.y;
            data[4] = compTwo.y;
            data[5] = compThree.y;
            data[6] = compOne.z;
            data[7] = compTwo.z;
            data[8] = compThree.z;
        }

        // ... (outros métodos da Matrix3) ...
    };

} // namespace cyclone

#endif // CYCLONE_CORE_H
