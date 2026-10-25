#include <cyclone/fgen.h>
#include <cyclone/precision.h>

using namespace cyclone;

// ============================================================================
// --- GRAVIDADE ---
// ============================================================================

Gravity::Gravity(const Vector3 &gravity) : gravity(gravity) {}

void Gravity::updateForce(RigidBody* body, real duration)
{
    if (!body->hasFiniteMass()) return;
    body->addForce(gravity * body->getMass());
}

// ============================================================================
// --- MOLA ---
// ============================================================================

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
    Vector3 lws = body->getPointInWorldSpace(connectionPoint);
    Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);

    Vector3 force = lws - ows;

    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    force.normalize();
    force *= -magnitude;
    body->addForceAtPoint(force, lws);
}

// ============================================================================
// --- AERODINÂMICA BASE (`Aero`) ---
// ============================================================================

Aero::Aero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed)
    : tensor(tensor), position(position), windspeed(windspeed) {}

void Aero::updateForce(RigidBody* body, real duration)
{
    updateForceFromTensor(body, duration, tensor);
}

void Aero::updateForceFromTensor(RigidBody* body, real duration, const Matrix3 &tensorToUse)
{
    // 1. Velocidade do corpo na coordenada global
    Vector3 velocity = body->velocity;
    if (windspeed) {
        velocity -= *windspeed;
    }

    // 2. Transforma a velocidade do ar para o espaço do corpo (body space)
    Vector3 bVel = body->transformMatrix.transformInverseDirection(velocity);

    // 3. Aplica o tensor aerodinâmico no espaço do corpo
    Vector3 bForce = tensorToUse * bVel;

    // 4. Converte a força resultante de volta para o espaço do mundo (world space)
    Vector3 force = body->transformMatrix.transformDirection(bForce);

    // 5. Aplica a força no ponto de conexão local da superfície
    body->addForceAtBodyPoint(force, position);
}

// ============================================================================
// --- CONTROLE AERODINÂMICO (`AeroControl`) ---
// ============================================================================

AeroControl::AeroControl(const Matrix3 &base, const Matrix3 &min, const Matrix3 &max,
                           const Vector3 &position, const Vector3 *windspeed)
    : Aero(base, position, windspeed), minTensor(min), maxTensor(max), controlSetting(0.0f) {}

void AeroControl::setControl(real value)
{
    controlSetting = value;
}

Matrix3 AeroControl::getTensor()
{
    if (controlSetting <= -1.0f) {
        return minTensor;
    } else if (controlSetting >= 1.0f) {
        return maxTensor;
    } else if (controlSetting < 0.0f) {
        Matrix3 result;
        for (int i = 0; i < 9; i++) {
            result.data[i] = tensor.data[i] + (-controlSetting) * (minTensor.data[i] - tensor.data[i]);
        }
        return result;
    } else {
        Matrix3 result;
        for (int i = 0; i < 9; i++) {
            result.data[i] = tensor.data[i] + controlSetting * (maxTensor.data[i] - tensor.data[i]);
        }
        return result;
    }
}

void AeroControl::updateForce(RigidBody* body, real duration)
{
    Matrix3 derivedTensor = getTensor();
    updateForceFromTensor(body, duration, derivedTensor);
}

// ============================================================================
// --- AERODINÂMICA ANGULAR/ROTATORIÁ (`AngledAero`) ---
// ============================================================================

AngledAero::AngledAero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed)
    : Aero(tensor, position, windspeed), orientation(Quaternion(1, 0, 0, 0)) {}

void AngledAero::setOrientation(const Quaternion &quat)
{
    orientation = quat;
}

void AngledAero::updateForce(RigidBody* body, real duration)
{
    // Combina a orientação do corpo com a orientação da própria superfície (vela/leme)
    Quaternion worldOrientation = body->orientation * orientation;

    // Matriz de rotação temporária no espaço global
    Matrix4 transform;
    transform.setOrientationAndPosition(worldOrientation, body->getPointInWorldSpace(position));

    // Velocidade relativa do ar
    Vector3 velocity = body->velocity;
    if (windspeed) {
        velocity -= *windspeed;
    }

    // Transforma a velocidade para o espaço local da superfície
    Vector3 bVel = transform.transformInverseDirection(velocity);

    // Calcula a força com o tensor local
    Vector3 bForce = tensor * bVel;

    // Transforma a força para o mundo
    Vector3 force = transform.transformDirection(bForce);

    // Aplica no corpo no ponto de montagem
    body->addForceAtBodyPoint(force, position);
}

// ============================================================================
// --- EMPUXO / FLUTUAÇÃO (`Buoyancy`) ---
// ============================================================================

Buoyancy::Buoyancy(const Vector3 &cOfB,
                   real maxDepth, real volume, real waterHeight,
                   real liquidDensity)
    : centerOfBuoyancy(cOfB),
      maxDepth(maxDepth),
      volume(volume),
      waterHeight(waterHeight),
      liquidDensity(liquidDensity) {}

void Buoyancy::updateForce(RigidBody* body, real duration)
{
    // Posição do centro de flutuação no espaço mundial
    Vector3 pointInWorld = body->getPointInWorldSpace(centerOfBuoyancy);
    real depth = pointInWorld.y;

    // 1. Fora d'água
    if (depth >= waterHeight + maxDepth) return;

    Vector3 force(0, 0, 0);

    // 2. Submersão total
    if (depth <= waterHeight - maxDepth) {
        force.y = liquidDensity * volume;
        body->addForceAtBodyPoint(force, centerOfBuoyancy);
        return;
    }

    // 3. Submersão parcial
    force.y = liquidDensity * volume * (waterHeight + maxDepth - depth) / (2.0f * maxDepth);
    body->addForceAtBodyPoint(force, centerOfBuoyancy);
}
