/*
 * fgen.cpp
 * 
 * Este arquivo contém a implementação dos geradores de força e torque para corpos rígidos
 * (Gravity, Spring, AnchoredSpring e ForceRegistry) no motor físico Cyclone.
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/fgen.h>

namespace cyclone {

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE GRAVITY
 * =========================================================
 */

Gravity::Gravity(const Vector3 &gravity)
    : gravity(gravity) {}

void Gravity::updateForce(RigidBody *body, real duration) {
    // Se o corpo tiver massa infinita (inverso da massa <= 0), não aplica gravidade
    if (body->getInverseMass() <= 0.0f) return;

    // Aplica a força da gravidade multiplicada pela massa do corpo (F = m * g)
    body->addForce(gravity * body->getMass());
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE SPRING
 * =========================================================
 */

Spring::Spring(const Vector3 &localConnectionPt,
               RigidBody *other,
               const Vector3 &otherConnectionPt,
               real springConstant,
               real restLength)
    : connectionPoint(localConnectionPt),
      other(other),
      otherConnectionPoint(otherConnectionPt),
      springConstant(springConstant),
      restLength(restLength) {}

void Spring::updateForce(RigidBody *body, real duration) {
    // Calcula a posição dos pontos de conexão no espaço do mundo
    Vector3 lpp = body->getPointInWorldSpace(connectionPoint);
    Vector3 opp = other->getPointInWorldSpace(otherConnectionPoint);

    // Calcula o vetor diferença e o comprimento atual da mola
    Vector3 force = lpp - opp;
    real magnitude = force.magnitude();

    // Calcula a magnitude da força elástica usando a Lei de Hooke
    real displacement = std::abs(magnitude - restLength);
    real forceMagnitude = displacement * springConstant;

    // Normaliza e aplica a força no ponto de conexão do corpo
    force.normalize();
    force *= -forceMagnitude;
    body->addForceAtPoint(force, lpp);
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE ANCHOREDSPRING
 * =========================================================
 */

AnchoredSpring::AnchoredSpring(const Vector3 &connectionPoint,
                               const Vector3 &anchor,
                               real springConstant,
                               real restLength)
    : connectionPoint(connectionPoint),
      anchor(anchor),
      springConstant(springConstant),
      restLength(restLength) {}

void AnchoredSpring::updateForce(RigidBody *body, real duration) {
    // Calcula o ponto de conexão no espaço do mundo
    Vector3 worldPt = body->getPointInWorldSpace(connectionPoint);

    // Calcula o vetor entre o ponto de conexão e a âncora fixa
    Vector3 force = worldPt - anchor;
    real magnitude = force.magnitude();

    // Aplica a Lei de Hooke
    real displacement = std::abs(magnitude - restLength);
    real forceMagnitude = displacement * springConstant;

    force.normalize();
    force *= -forceMagnitude;
    body->addForceAtPoint(force, worldPt);
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE FORCEREGISTRY
 * =========================================================
 */

void ForceRegistry::add(RigidBody *body, ForceGenerator *fg) {
    ForceRegistration registration;
    registration.body = body;
    registration.fg = fg;
    registrations.push_back(registration);
}

void ForceRegistry::remove(RigidBody *body, ForceGenerator *fg) {
    for (auto it = registrations.begin(); it != registrations.end(); ++it) {
        if (it->body == body && it->fg == fg) {
            registrations.erase(it);
            break;
        }
    }
}

void ForceRegistry::clear() {
    registrations.clear();
}

void ForceRegistry::updateForces(real duration) {
    for (auto i = registrations.begin(); i != registrations.end(); ++i) {
        i->fg->updateForce(i->body, duration);
    }
}

} // namespace cyclone
