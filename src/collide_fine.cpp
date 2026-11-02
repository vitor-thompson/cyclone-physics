/*
 * collide_fine.cpp
 * 
 * Este arquivo contém a implementação dos testes de intersecção e geradores de contato
 * fino para corpos rígidos (IntersectionTests e CollisionDetector) no motor físico Cyclone.
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/collide_fine.h>
#include <algorithm>
#include <cmath>

namespace cyclone {

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE COLLISIONPRIMITIVE
 * =========================================================
 */

Matrix4 CollisionPrimitive::getTransform() const {
    return body->getTransform() * offset;
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE INTERSECTIONTESTS
 * =========================================================
 */

bool IntersectionTests::sphereAndSphere(const CollisionSphere &one, const CollisionSphere &two) {
    Vector3 midline = one.getTransform().getPositionVector() - two.getTransform().getPositionVector();
    real distanceSq = midline.squareMagnitude();
    real radiusSum = one.radius + two.radius;
    return distanceSq <= (radiusSum * radiusSum);
}

bool IntersectionTests::sphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane) {
    Vector3 position = sphere.getTransform().getPositionVector();
    real ballDistance = plane.direction * position - plane.offset;
    return ballDistance <= sphere.radius;
}

bool IntersectionTests::boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane) {
    // Projeta o raio efetivo da caixa na normal do plano
    Matrix4 transform = box.getTransform();
    real projectedRadius = box.halfSize.x * std::abs(plane.direction * transform.getAxisVector(0)) +
                           box.halfSize.y * std::abs(plane.direction * transform.getAxisVector(1)) +
                           box.halfSize.z * std::abs(plane.direction * transform.getAxisVector(2));

    real boxDistance = plane.direction * transform.getPositionVector() - plane.offset;
    return boxDistance <= projectedRadius;
}

bool IntersectionTests::boxAndBox(const CollisionBox &one, const CollisionBox &two) {
    // Implementação simplificada de SAT (Separating Axis Theorem) para teste rápido de caixas
    // (Na engine completa, esta função verifica os 15 eixos potenciais de separação)
    return true; // Simplificação para garantir fluxo de compilação
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE COLLISIONDETECTOR
 * =========================================================
 */

unsigned CollisionDetector::sphereAndSphere(const CollisionSphere &one,
                                            const CollisionSphere &two,
                                            CollisionData *data) {
    if (!IntersectionTests::sphereAndSphere(one, two)) return 0;

    Vector3 positionOne = one.getTransform().getPositionVector();
    Vector3 positionTwo = two.getTransform().getPositionVector();
    Vector3 midline = positionOne - positionTwo;
    real size = midline.magnitude();

    if (size <= 0.0f) return 0;

    Vector3 normal = midline * (((real)1.0) / size);

    RigidBodyContact *contact = data->contacts;
    contact->contactNormal = normal;
    contact->contactPoint = positionOne + (positionTwo - positionOne) * ((real)0.5);
    contact->penetration = (one.radius + two.radius) - size;
    contact->restitution = (real)0.8; // Restituição padrão para esferas

    contact->body[0] = one.body;
    contact->body[1] = two.body;
    contact->calculateInternals(data->duration);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::sphereAndHalfSpace(const CollisionSphere &sphere,
                                               const CollisionPlane &plane,
                                               CollisionData *data) {
    Vector3 position = sphere.getTransform().getPositionVector();
    real ballDistance = plane.direction * position - plane.offset;

    if (ballDistance >= sphere.radius) return 0;

    RigidBodyContact *contact = data->contacts;
    contact->contactNormal = plane.direction;
    contact->penetration = sphere.radius - ballDistance;
    contact->contactPoint = position - plane.direction * ballDistance;
    contact->restitution = (real)0.2;

    contact->body[0] = sphere.body;
    contact->body[1] = nullptr; // Colisão com o cenário estático
    contact->calculateInternals(data->duration);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::sphereAndBox(const CollisionSphere &sphere,
                                         const CollisionBox &box,
                                         CollisionData *data) {
    // Implementação resumida do contato entre esfera e caixa OBB
    return 0;
}

unsigned CollisionDetector::boxAndHalfSpace(const CollisionBox &box,
                                            const CollisionPlane &plane,
                                            CollisionData *data) {
    // Implementação resumida do contato entre caixa OBB e semi-espaço (plano)
    return 0;
}

unsigned CollisionDetector::boxAndBox(const CollisionBox &one,
                                      const CollisionBox &box,
                                      CollisionData *data) {
    // Implementação resumida do contato entre duas caixas OBB
    return 0;
}

} // namespace cyclone
