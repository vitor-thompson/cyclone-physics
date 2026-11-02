/**
 * @file precision.h
 * @brief Definicao da precisao numerica de ponto flutuante utilizada pelo motor Cyclone.
 *
 * Este arquivo permite alternar a precisao do motor entre precisao simples (float)
 * e precisao dupla (double) alterando as definicoes de macros de pre-processamento.
 */

#ifndef CYCLONE_PRECISION_H
#define CYCLONE_PRECISION_H

#include <cmath>

namespace cyclone {

/**
 * Define o tipo de ponto flutuante padrao utilizado na simulacao fisica.
 * Por padrao, utiliza-se precisao simples ('float') para otimizacao de desempenho.
 * Se a macro SINGLE_PRECISION nao estiver definida, o motor utilizara 'double'.
 */
#if 1
    #define SINGLE_PRECISION
#endif

#ifdef SINGLE_PRECISION
    /**
     * @brief Tipo de dado escalar real do motor (float).
     */
    typedef float real;

    /**
     * @brief Valor maximo representavel para o tipo real.
     */
    #define REAL_MAX FLT_MAX

    /**
     * @brief Funcao wrapper para a raiz quadrada em precisao simples.
     */
    #define real_sqrt sqrtf

    /**
     * @brief Funcao wrapper para o valor absoluto em precisao simples.
     */
    #define real_abs fabsf

    /**
     * @brief Funcao wrapper para o cosseno em precisao simples.
     */
    #define real_cos cosf

    /**
     * @brief Funcao wrapper para o seno em precisao simples.
     */
    #define real_sin sinf

    /**
     * @brief Funcao wrapper para a arcocosseno em precisao simples.
     */
    #define real_acos acosf

    /**
     * @brief Funcao wrapper para a potencia em precisao simples.
     */
    #define real_pow powf

    /**
     * @brief Funcao wrapper para a exponenciacao em precisao simples.
     */
    #define real_exp expf

    /**
     * @brief Define a tolerancia para comparacoes numericas em precisao simples.
     */
    #define REAL_EPSILON 1.192092896e-07F
#else
    /**
     * @brief Tipo de dado escalar real do motor (double).
     */
    typedef double real;

    /**
     * @brief Valor maximo representavel para o tipo real.
     */
    #define REAL_MAX DBL_MAX

    /**
     * @brief Funcao wrapper para a raiz quadrada em precisao dupla.
     */
    #define real_sqrt sqrt

    /**
     * @brief Funcao wrapper para o valor absoluto em precisao dupla.
     */
    #define real_abs fabs

    /**
     * @brief Funcao wrapper para o cosseno em precisao dupla.
     */
    #define real_cos cos

    /**
     * @brief Funcao wrapper para o seno em precisao dupla.
     */
    #define real_sin sin

    /**
     * @brief Funcao wrapper para a arcocosseno em precisao dupla.
     */
    #define real_acos acos

    /**
     * @brief Funcao wrapper para a potencia em precisao dupla.
     */
    #define real_pow pow

    /**
     * @brief Funcao wrapper para a exponenciacao em precisao dupla.
     */
    #define real_exp exp

    /**
     * @brief Define a tolerancia para comparacoes numericas em precisao dupla.
     */
    #define REAL_EPSILON 2.2204460492503131e-016
#endif

} // namespace cyclone

#endif // CYCLONE_PRECISION_H
