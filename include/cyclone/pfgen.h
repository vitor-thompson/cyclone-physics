/*
 * pfgen.h
 * 
 * Este arquivo define o sistema de geradores de força para partículas no motor físico Cyclone.
 * Ele inclui a interface base 'ParticleForceGenerator' e a classe 'ParticleForceRegistry',
 * responsável por mapear quais forças devem ser aplicadas a quais partículas a cada quadro (frame).
 * 
 * Parte da biblioteca física Cyclone.
 */

#ifndef CYCLONE_PFGEN_H
#define CYCLONE_PFGEN_H

#include "particle.h"
#include <vector>

namespace cyclone {

/**
 * @brief Interface para geradores de força que atuam sobre partículas.
 */
class ParticleForceGenerator {
public:
    /**
     * @brief Sobrescreva este método para calcular e aplicar a força à partícula informada.
     * 
     * @param particle Ponteiro para a partícula que receberá a força.
     * @param duration Intervalo de tempo (delta time) do quadro atual.
     */
    virtual void updateForce(Particle *particle, real duration) = 0;
};

/**
 * @brief Gerador de força da gravidade simples.
 */
class ParticleGravity : public ParticleForceGenerator {
    Vector3 gravity; // Aceleração da gravidade

public:
    ParticleGravity(const Vector3 &gravity);
    virtual void updateForce(Particle *particle, real duration);
};

/**
 * @brief Gerador de força de atrito com o ar/arrasto (Drag).
 */
class ParticleDrag : public ParticleForceGenerator {
    real k1; // Coeficiente de arrasto de velocidade linear
    real k2; // Coeficiente de arrasto de velocidade quadrática

public:
    ParticleDrag(real k1, real k2);
    virtual void updateForce(Particle *particle, real duration);
};

/**
 * @brief Gerador de força de mola ancorada a um ponto fixo no espaço.
 */
class ParticleAnchoredSpring : public ParticleForceGenerator {
protected:
    Vector3 *anchor; // Ponto de ancoragem da mola
    real springConstant; // Constante elástica da mola (k)
    real restLength; // Comprimento de repouso da mola

public:
    ParticleAnchoredSpring(Vector3 *anchor, real springConstant, real restLength);
    virtual void updateForce(Particle *particle, real duration);
};

/**
 * @brief Registro central para gerenciar e aplicar múltiplos geradores de força a partículas.
 */
class ParticleForceRegistry {
protected:
    /**
     * @brief Estrutura interna que associa uma partícula a um gerador de força.
     */
    struct ParticleForceRegistration {
        Particle *particle;
        ParticleForceGenerator *fg;
    };

    typedef std::vector<ParticleForceRegistration> Registry;
    Registry registrations;

public:
    /**
     * @brief Registra um par de partícula e gerador de força.
     */
    void add(Particle *particle, ParticleForceGenerator *fg);

    /**
     * @brief Remove o registro entre uma partícula e um gerador de força.
     */
    void remove(Particle *particle, ParticleForceGenerator *fg);

    /**
     * @brief Limpa todos os registros cadastrados.
     */
    void clear();

    /**
     * @brief Atualiza e aplica todas as forças registradas às suas respectivas partículas.
     * 
     * @param duration Intervalo de tempo (delta time) do quadro.
     */
    void updateForces(real duration);
};

} // namespace cyclone

#endif // CYCLONE_PFGEN_H
