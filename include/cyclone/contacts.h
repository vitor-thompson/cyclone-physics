/*
 * contacts.h
 * 
 * Este arquivo define o sistema de contatos e resolução de colisões para corpos rígidos
 * no motor físico Cyclone. Inclui a estrutura 'RigidBodyContact' para gerenciar impulsos,
 * atrito e restituição, e o 'ContactResolver' para processar múltiplos contatos simultâneos.
 * 
 * Parte da biblioteca física Cyclone.
 */

#ifndef CYCLONE_CONTACTS_H
#define CYCLONE_CONTACTS_H

import cyclone.body;

namespace cyclone {

// Declaração antecipada
class ContactResolver;

/**
 * @brief Representa um ponto de contato físico entre dois corpos rígidos.
 */
class RigidBodyContact {
public:
    // Os dois corpos rígidos em contato (o segundo pode ser nullptr para objetos fixos do cenário)
    RigidBody *body[2];

    // Coeficiente de restituição (elasticidade) da colisão
    real restitution;

    // Coeficiente de atrito entre os corpos
    real friction;

    // Normal do contato no espaço do mundo (aponta do corpo 1 para o corpo 2)
    Vector3 contactNormal;

    // Ponto de contato exato no espaço do mundo
    Vector3 contactPoint;

    // Profundidade da interpenetração
    real penetration;

    // Matriz que mapeia impulsos para alterações de velocidade no ponto de contato
    Matrix3 contactToWorld;

    /**
     * @brief Configura dados internos do contato antes da resolução.
     */
    void calculateInternals(real duration);

    /**
     * @brief Troca a ordem dos corpos no contato.
     */
    void swapBodies();

    /**
     * @brief Resolve a velocidade (impulso) para o contato.
     */
    void applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);

    /**
     * @brief Resolve a interpenetração geométrica.
     */
    void applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration);

    /**
     * @brief Calcula a velocidade separadora desejada.
     */
    real calculateSeparatingVelocity() const;

private:
    /**
     * @brief Resolve os impulsos de atrito e restituição.
     */
    void matchVelocity(Vector3 *linearVelocity, Vector3 *angularVelocity);

    friend class ContactResolver;
};

/**
 * @brief Gerencia e resolve uma lista de contatos de corpos rígidos de forma iterativa.
 */
class ContactResolver {
protected:
    unsigned velocityIterations;
    unsigned positionIterations;
    real velocityEpsilon;
    real positionEpsilon;

public:
    unsigned velocityIterationsUsed;
    unsigned positionIterationsUsed;

    /**
     * @brief Construtor do resolvedor de contatos.
     */
    ContactResolver(unsigned iterations, real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);
    
    ContactResolver(unsigned velocityIterations, unsigned positionIterations,
                    real velocityEpsilon, real positionEpsilon);

    /**
     * @brief Define o número de iterações para velocidade e posição.
     */
    void setIterations(unsigned velocityIterations, unsigned positionIterations);
    void setEpsilon(real velocityEpsilon, real positionEpsilon);

    /**
     * @brief Resolve o conjunto de contatos fornecido.
     */
    void resolveContacts(RigidBodyContact *contactArray, unsigned numContacts, real duration);
};

} // namespace cyclone

#endif // CYCLONE_CONTACTS_H
