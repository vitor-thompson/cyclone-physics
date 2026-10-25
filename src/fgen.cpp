#include <cyclone/fgen.h>
#include <cyclone/precision.h>

using namespace cyclone;

// ==========================================
// --- GRAVIDADE ---
// ==========================================

Gravity::Gravity(const Vector3 &gravity) : gravity(gravity) {}

void Gravity::updateForce(RigidBody* body, real duration)
{
    // Verifica se o corpo tem massa finita (não é imóvel)
    if (!body->hasFiniteMass()) return;

    // Aplica a força dimensionada pela massa direto no centro de massa
    body->addForce(gravity * body->getMass());
}

// ==========================================
// --- MOLA ---
// ==========================================

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

// ==========================================
// --- AERODINÂMICA (AERO) ---
// ==========================================

Aero::Aero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed)
    : tensor(tensor), position(position), windspeed(windspeed) {}

void Aero::updateForce(RigidBody* body, real duration)
{
    updateForce(body, duration, tensor);
}

void Aero::updateForce(RigidBody* body, real duration, const Matrix3 &tensor)
{
    // 1. Calcula a velocidade do ar relativa à superfície no espaço global
    Vector3 pointWorld = body->getPointInWorldSpace(position);
    
    // Velocidade linear no ponto de aplicação + contribuição da velocidade angular
    // (v = velocity + rotation % r)
    Vector3 vel = body->velocity;
    // Se sua classe RigidBody possuir método para velocidade no ponto, pode utilizá-lo.
    // Usando a aproximação padrão:
    
    // 2. Subtrai a velocidade do vento, se houver
    Vector3 airSpeed = vel;
    if (windspeed) {
        airSpeed -= *windspeed;
    }

    // 3. Converte a velocidade do ar para o espaço local (body space) usando a rotação da matriz
    // (Inverte apenas a direção, ignorando a translação)
    Vector3 bVel = body->transformMatrix.transformInverseDirection(airSpeed);

    // 4. Aplica o tensor aerodinâmico para obter a força no espaço local
    Vector3 bForce = tensor * bVel;

    // 5. Converte a força resultante de volta para o espaço global
    Vector3 force = body->transformMatrix.transformDirection(bForce);

    // 6. Aplica a força no ponto específico do corpo
    body->addForceAtBodyPoint(force, position);
}

// ==========================================
// --- CONTROLE AERODINÂMICO (AEROCONTROL) ---
// ==========================================

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
        // Interpola entre minTensor e o tensor base (0)
        Matrix3 result;
        for (int i = 0; i < 9; i++) {
            result.data[i] = tensor.data[i] + (-controlSetting) * (minTensor.data[i] - tensor.data[i]);
        }
        return result;
    } else {
        // Interpola entre o tensor base (0) e maxTensor
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
    Aero::updateForce(body, duration, derivedTensor);
}
