/*
 * Arquivo de precisão e tipos do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#ifndef CYCLONE_PRECISION_H
#define CYCLONE_PRECISION_H

#include <math.h>
#include <float.h>

namespace cyclone {

    /**
     * Define o tipo real usado em todo o motor. 
     * Pode ser alterado para 'double' se maior precisão numérica for necessária.
     */
    typedef float real;

    #define REAL_MAX FLT_MAX

    /**
     * Funções matemáticas em precisão real para garantir portabilidade
     * e facilitar a alternância entre float e double.
     */
    inline real real_sqrt(real x)
    {
        return sqrtf(x);
    }

    inline real real_abs(real x)
    {
        return fabsf(x);
    }

    inline real real_sin(real x)
    {
        return sinf(x);
    }

    inline real real_cos(real x)
    {
        return cosf(x);
    }

    inline real real_exp(real x)
    {
        return expf(x);
    }

    inline real real_pow(real x, real y)
    {
        return powf(x, y);
    }

    // Constante matemática Pi em precisão real
    #define R_PI 3.14159265f

} // namespace cyclone

#endif // CYCLONE_PRECISION_H
