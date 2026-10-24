#include <cyclone/fgen.h>
#include <cyclone/precision.h>

using namespace cyclone;

// --- GRAVIDADE ---

Gravity::Gravity(const Vector3 &gravity) : gravity(gravity) {}

void Gravity::updateForce(RigidBody* body, real duration)
{
    // Verifica se o corpo tem massa finita (não é imóvel)
    if (!body->hasFiniteMass()) return;

    // Aplica a força dimensionada pela massa direto no centro de massa
    body->addForce(gravity * body->getMass());
}

// --- MOLA ---

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

void Spring::updateForce(RigidBody* body, real duration)
{
    // Calcula as duas pontas da mola no espaço global (world space)
    Vector3 lws = body->getPointInWorldSpace(connectionPoint);
    Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);

    // Calcula o vetor de deslocamento da mola
    Vector3 force = lws - ows;

    // Calcula a magnitude da força com base na Lei de Hooke
    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    // Normaliza, aplica a direção oposta (puxar/empurrar) e aplica no ponto
    force.normalize();
    force *= -magnitude;
    body->addForceAtPoint(force, lws);
}
