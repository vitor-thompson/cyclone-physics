/*
 * pcontacts.h
 * 
 * Este arquivo define o sistema de detecção e resolução de contatos para partículas
 * no motor físico Cyclone. Inclui a estrutura 'ParticleContact' para representar colisões
 * e o 'ParticleContactResolver' responsável por processar velocidades e interpenetração.
 * 
 * Parte da biblioteca física Cyclone.
 */

#ifndef CYCLONE_PCONTACTS_H
#define CYCLONE_PCONTACTS_H

#include "particle.h"

namespace cyclone {

// Declaração antecipada para evitar dependências circulares
class ParticleContactResolver;

/**
 * @brief Representa um contato físico entre duas partículas (ou uma partícula e um obstáculo fixo).
 */
class ParticleContact {
public:
    // As duas partículas envolvidas no contato. Se a segunda for nullptr, o contato é com um cenário fixo.
    Particle *particle[2];

    // Restitution (coeficiente de restituição / elasticidade do choque)
    real restitution;

    // Direção e sentido do contato no espaço do mundo
    Vector3 contactNormal;

    // Profundidade da interpenetração entre os corpos
    real penetration;

    // Movimentação acumulada aplicada para resolver a posição
    Vector3 particleMovement[2];

    /**
     * @brief Resolve o contato, lidando com velocidades (impulso) e interpenetração.
     * @param duration Intervalo de tempo do quadro.
     */
    void resolve(real duration);

    /**
     * @brief Calcula a velocidade separadora desejada para o contato.
     */
    real calculateSeparatingVelocity() const;

private:
    /**
     * @brief Resolve os efeitos de impulso devidos à velocidade da colisão.
     */
    void resolveVelocity(real duration);

    /**
     * @brief Resolve os efeitos geométricos devidos à interpenetração (afundamento).
     */
    void resolveInterpenetration(real duration);

    // Permite que o resolvedor acesse detalhes internos de contato
    friend class ParticleContactResolver;
};

/**
 * @brief Processa uma lista de contatos de partículas, resolvendo-os em ordem de severidade.
 */
class ParticleContactResolver {
protected:
    unsigned iterations;           // Número máximo de iterações permitidas por quadro
    unsigned iterationsUsed;       // Contador de iterações realmente utilizadas

public:
    /**
     * @brief Construtor que define o número de iterações do resolvedor.
     */
    ParticleContactResolver(unsigned iterations);

    /**
     * @brief Define o número máximo de iterações de resolução.
     */
    void setIterations(unsigned iterations);

    /**
     * @brief Resolve um conjunto de contatos fornecido.
     * @param contactArray Ponteiro para o array de contatos.
     * @param numContacts Número total de contatos no array.
     * @param duration Intervalo de tempo do quadro.
     */
    void resolveContacts(ParticleContact *contactArray, unsigned numContacts, real duration);
};

} // namespace cyclone

#endif // CYCLONE_PCONTACTS_H
