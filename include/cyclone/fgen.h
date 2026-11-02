/*
 * fgen.h
 * 
 * Este arquivo define os geradores de força e torque aplicáveis a corpos rígidos
 * no motor físico Cyclone. Inclui a interface base 'ForceGenerator' e o registro
 * 'ForceRegistry' para associar forças a corpos rígidos específicos a cada quadro.
 * 
 * Parte da biblioteca física Cyclone.
 */

#ifndef CYCLONE_FGEN_H
#define CYCLONE_FGEN_H

#include "body.h"
#include <vector>

namespace cyclone {

/**
 * @brief Interface base para geradores de força que atuam sobre corpos rígidos.
 */
class ForceGenerator {
public:
    /**
     * @brief Sobrescreva este método para calcular e aplicar a força/torque ao corpo rígido.
     * 
     * @param body Ponteiro para o corpo rígido que receberá a força.
     * @param duration Intervalo de tempo (delta time) do quadro atual.
     */
    virtual void updateForce(RigidBody *body, real duration) = 0;
};

/**
 * @brief Gerador de força da gravidade para corpos rígidos.
 */
class Gravity : public ForceGenerator {
    Vector3 gravity; // Aceleração da gravidade no mundo

public:
    /**
     * @brief Construtor da força de gravidade.
     * @param gravity Vetor de aceleração da gravidade.
     */
    Gravity(const Vector3 &gravity);

    virtual void updateForce(RigidBody *body, real duration);
};

/**
 * @brief Gerador de força de mola que conecta dois corpos rígidos em pontos de conexão específicos.
 */
class Spring : public ForceGenerator {
    Vector3 connectionPoint;     // Ponto de conexão no corpo local
    RigidBody *other;            // O outro corpo rígido na extremidade da mola
    Vector3 otherConnectionPoint;// Ponto de conexão no outro corpo (local)
    real springConstant;         // Constante elástica da mola (k)
    real restLength;             // Comprimento de repouso da mola

public:
    Spring(const Vector3 &localConnectionPt,
           RigidBody *other,
           const Vector3 &otherConnectionPt,
           real springConstant,
           real restLength);

    virtual void updateForce(RigidBody *body, real duration);
};

/**
 * @brief Gerador de força de mola conectando um corpo rígido a um ponto fixo no mundo.
 */
class AnchoredSpring : public ForceGenerator {
protected:
    Vector3 connectionPoint; // Ponto de conexão no corpo local
    Vector3 anchor;          // Ponto de ancoragem fixo no espaço do mundo
    real springConstant;     // Constante elástica da mola (k)
    real restLength;         // Comprimento de repouso da mola

public:
    AnchoredSpring(const Vector3 &connectionPoint,
                   const Vector3 &anchor,
                   real springConstant,
                   real restLength);

    virtual void updateForce(RigidBody *body, real duration);
};

/**
 * @brief Registro central para gerenciar e aplicar múltiplos geradores de força a corpos rígidos.
 */
class ForceRegistry {
protected:
    /**
     * @brief Estrutura interna que mapeia um corpo rígido a um gerador de força.
     */
    struct ForceRegistration {
        RigidBody *body;
        ForceGenerator *fg;
    };

    typedef std::vector<ForceRegistration> Registry;
    Registry registrations;

public:
    /**
     * @brief Registra a associação entre um corpo rígido e um gerador de força.
     */
    void add(RigidBody *body, ForceGenerator *fg);

    /**
     * @brief Remove uma associação registrada.
     */
    void remove(RigidBody *body, ForceGenerator *fg);

    /**
     * @brief Limpa todos os registros do sistema.
     */
    void clear();

    /**
     * @brief Atualiza e aplica todas as forças registradas a seus respectivos corpos rígidos.
     * @param duration Intervalo de tempo (delta time) do quadro.
     */
    void updateForces(real duration);
};

} // namespace cyclone

#endif // CYCLONE_FGEN_H
