/*
 * Geradores de Força do motor Cyclone.
 * Atualizado com a classe Explosion de três componentes (Capítulo 17.2.3).
 */

#ifndef CYCLONE_FGEN_H
#define CYCLONE_FGEN_H

#include "precision.h"
#include "core.h"
#include "body.h"
#include "particle.h"

namespace cyclone {

    /**
     * Interface base para geradores de força aplicados a corpos rígidos.
     */
    class ForceGenerator
    {
    public:
        virtual void updateForce(RigidBody *body, real duration) = 0;
    };

    /**
     * Interface base para geradores de força aplicados a partículas.
     */
    class ParticleForceGenerator
    {
    public:
        virtual void updateForce(Particle *particle, real duration) = 0;
    };

    /**
     * Gerador de força para explosões cinematográficas em três fases:
     * Implosão, Onda de Concussão e Chaminé de Convecção (Capítulo 17.2.3).
     */
    class Explosion : public ForceGenerator, public ParticleForceGenerator
    {
    protected:
        real timePassed;

    public:
        // Posição da detonação
        Vector3 detonation;

        // Propriedades da Implosão
        real implosionMaxRadius;
        real implosionMinRadius;
        real implosionDuration;
        real implosionForce;

        // Propriedades da Onda de Concussão (Shockwave)
        real concussionWaveSpeed;
        real peakConcussionForce;
        real concussionWidth;
        real concussionTail;

        // Propriedades da Chaminé de Convecção
        real peakConvectionForce;
        real chimneyRadius;
        real chimneyHeight;
        real convectionDuration;

        Explosion()
            : timePassed(0.0f),
              implosionMaxRadius(10.0f), implosionMinRadius(1.0f), implosionDuration(0.2f), implosionForce(5.0f),
              concussionWaveSpeed(15.0f), peakConcussionForce(50.0f), concussionWidth(2.0f), concussionTail(1.0f),
              peakConvectionForce(2.0f), chimneyRadius(5.0f), chimneyHeight(20.0f), convectionDuration(5.0f) {}

        virtual void updateForce(RigidBody *body, real duration)
        {
            timePassed += duration;
            Vector3 position = body->getPosition();
            Vector3 blastToBody = position - detonation;
            real distance = blastToBody.magnitude();

            // 1. Fase de Implosão
            if (timePassed < implosionDuration && distance > implosionMinRadius && distance < implosionMaxRadius)
            {
                Vector3 implosionDir = blastToBody.unit();
                body->addForce(implosionDir * -implosionForce);
            }

            // 2. Fase da Onda de Concussão (Shockwave)
            real waveFront = concussionWaveSpeed * timePassed;
            if (distance >= waveFront - concussionWidth && distance <= waveFront + (concussionTail + 1.0f) * concussionWidth)
            {
                Vector3 waveDir = blastToBody.unit();
                real forceMag = peakConcussionForce; // Aplicando simplificação do perfil de força
                
                // Aplica a força descentralizada para gerar torque/rotação
                Vector3 offsetPoint = position + Vector3(0.5f, 0.5f, 0.5f); // Ponto descentralizado
                body->addForceAtPoint(waveDir * forceMag, offsetPoint);
            }

            // 3. Fase da Chaminé de Convecção
            if (timePassed < convectionDuration)
            {
                real dxz = real_sqrt(blastToBody.x * blastToBody.x + blastToBody.z * blastToBody.z);
                real dy = blastToBody.y;
                if (dxz < chimneyRadius && dy > 0 && dy < chimneyHeight)
                {
                    Vector3 convectionDir(0.1f, 1.0f, 0.1f); // Leve inclinação para moldar o cogumelo
                    body->addForce(convectionDir * peakConvectionForce);
                }
            }
        }

        virtual void updateForce(Particle *particle, real duration)
        {
            // Implementação análoga para partículas, se necessário
        }
    };

} // namespace cyclone

#endif // CYCLONE_FGEN_H
