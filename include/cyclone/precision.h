/*
 * Arquivo de precisão e tipos do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 * Suporta alternância entre precisão simples (float) e dupla (double).
 */

#ifndef CYCLONE_PRECISION_H
#define CYCLONE_PRECISION_H

#include <math.h>
#include <float.h>

namespace cyclone {

    // Definição do tipo real (pode ser alterado para double se necessário)
    typedef float real;

    #define REAL_MAX FLT_MAX

    inline real real_sqrt(real x) { return sqrtf(x); }
    inline real real_abs(real x) { return fabsf(x); }
    inline real real_sin(real x) { return sinf(x); }
    inline real real_cos(real x) { return cosf(x); }
    inline real real_exp(real x) { return expf(x); }
    inline real real_pow(real x, real y) { return powf(x, y); }

    #define R_PI 3.14159265f

} // namespace cyclone

#endif // CYCLONE_PRECISION_H
