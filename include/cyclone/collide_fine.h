/*
 * collide_fine.h
 * 
 * Este arquivo define o sistema de detecção fina de colisão para corpos rígidos
 * no motor físico Cyclone. Inclui primitivas geométricas (esferas, planos, caixas OBB)
 * e algoritmos de teste de intersecção e geração de contatos de corpo rígido.
 * 
 * Parte da biblioteca física Cyclone.
 */

#ifndef CYCLONE_COLLIDE_FINE_H
#define CYCLONE_COLLIDE_FINE_H

#include "core.h"
#include "body.h"
#include "contacts.h"

namespace cyclone {

/**
 * @brief Representa uma primitiva geométrica base associada a um corpo rígido.
 */
class CollisionPrimitive {
public:
    RigidBody *body; // Corpo rígido ao qual esta primitiva está acoplada
    Matrix4 offset;  // Transformação offset relativa ao centro de massa do corpo

    /**
     * @brief Retorna a matriz de transformação no espaço do mundo para esta primitiva.
     */
    Matrix4 getTransform() const;
};

/**
 * @brief Primitiva geométrica de Esfera de Colisão.
 */
class CollisionSphere : public CollisionPrimitive {
public:
    real radius; // Raio da esfera
};

/**
 * @brief Primitiva geométrica de Plano (ex: solo infinito).
 */
class CollisionPlane {
public:
    Vector3 direction; // Vetor normal do plano
    real offset;       // Distância do plano até a origem do mundo ao longo da normal
};

/**
 * @brief Primitiva geométrica de Caixa Orientada (OBB - Oriented Bounding Box).
 */
class CollisionBox : public CollisionPrimitive {
public:
    Vector3 halfSize; // Metade das dimensões da caixa (extensões ao longo dos eixos locais X, Y, Z)
};

/**
 * @brief Contém algoritmos estáticos para testes rápidos de intersecção geométrica.
 */
class IntersectionTests {
public:
    static bool sphereAndSphere(const CollisionSphere &one, const CollisionSphere &two);
    static bool sphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane);
    static bool boxAndBox(const CollisionBox &one, const CollisionBox &two);
    static bool boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane);
};

/**
 * @brief Contém algoritmos para gerar dados detalhados de contato de corpos rígidos durante colisões.
 */
class CollisionDetector {
public:
    static unsigned sphereAndSphere(const CollisionSphere &one,
                                    const CollisionSphere &two,
                                    CollisionData *data);

    static unsigned sphereAndHalfSpace(const CollisionSphere &sphere,
                                       const CollisionPlane &plane,
                                       CollisionData *data);

    static unsigned sphereAndBox(const CollisionSphere &sphere,
                                 const CollisionBox &box,
                                 CollisionData *data);

    static unsigned boxAndHalfSpace(const CollisionBox &box,
                                    const CollisionPlane &plane,
                                    CollisionData *data);

    static unsigned boxAndBox(const CollisionBox &one,
                              const CollisionBox &box,
                              CollisionData *data);
};

} // namespace cyclone

#endif // CYCLONE_COLLIDE_FINE_H
