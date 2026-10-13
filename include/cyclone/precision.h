#pragma once

#include <cmath>

namespace cyclone {
    /**
     * Define a precisão dos números reais. O Cyclone pode ser compilado
     * em versões de precisão simples ou dupla. Por padrão, usamos precisão simples.
     */
    typedef float real;

    /** Define a precisão do operador de raiz quadrada com base no tipo real. */
    #define real_sqrt sqrtf
}
