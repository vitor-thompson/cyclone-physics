#include <cyclone/pfgen.h>

namespace cyclone {
    // --- Implementação do Gerador de Gravidade ---
    ParticleGravity::ParticleGravity(const Vector3 &gravity)
        : gravity(gravity) {}

    void ParticleGravity::updateForce(Particle* particle, real duration)
    {
        // Verifica se a partícula possui massa finita (objetos estáticos ignoram gravidade)
        if (!particle->hasFiniteMass()) return;

        // Aplica a força escalada pela massa: F = m * g
        particle->addForce(gravity * particle->getMass());
    }

    // --- Arrasto (Drag) ---
    ParticleDrag::ParticleDrag(real k1, real k2)
        : k1(k1), k2(k2) {}

    void ParticleDrag::updateForce(Particle* particle, real duration)
    {
        Vector3 force = particle->getVelocity();
        
        // Calcula o coeficiente de arrasto total com base na magnitude da velocidade
        real dragCoeff = force.magnitude();
        dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

        // Calcula a força final no sentido oposto à velocidade (-v)
        force.normalize();
        force *= -dragCoeff;
        
        particle->addForce(force);
    }

    // --- Mola Básica (ParticleSpring) ---
    ParticleSpring::ParticleSpring(Particle *other, real springConstant, real restLength)
        : other(other), springConstant(springConstant), restLength(restLength) {}

    void ParticleSpring::updateForce(Particle* particle, real duration)
    {
        // 1. Calcula o vetor entre as posições das duas partículas
        Vector3 force = particle->getPosition();
        force -= other->getPosition();

        // 2. Calcula a magnitude da força elástica usando a Lei de Hooke
        real magnitude = force.magnitude();
        magnitude = real_abs(magnitude - restLength);
        magnitude *= springConstant;

        // 3. Normaliza o vetor e aplica a força no sentido oposto (atração/repulsão)
        force.normalize();
        force *= -magnitude;
        particle->addForce(force);
    }

    // --- Mola Ancorada (ParticleAnchoredSpring) ---
    ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3 *anchor, real springConstant, real restLength)
        : anchor(anchor), springConstant(springConstant), restLength(restLength) {}

    void ParticleAnchoredSpring::updateForce(Particle* particle, real duration)
    {
        // 1. Calcula o vetor entre a posição da partícula e o ponto de ancoragem fixo
        Vector3 force = particle->getPosition();
        force -= *anchor;

        // 2. Calcula a magnitude da força elástica usando a Lei de Hooke
        real magnitude = force.magnitude();
        magnitude = real_abs(magnitude - restLength);
        magnitude *= springConstant;

        // 3. Normaliza o vetor e aplica a força no sentido oposto
        force.normalize();
        force *= -magnitude;
        particle->addForce(force);
    }

    // --- Cabo Elástico / Bungee (ParticleBungee) ---
    ParticleBungee::ParticleBungee(Particle *other, real springConstant, real restLength)
        : other(other), springConstant(springConstant), restLength(restLength) {}

    void ParticleBungee::updateForce(Particle* particle, real duration)
    {
        // 1. Calcula o vetor entre as posições das duas partículas
        Vector3 force = particle->getPosition();
        force -= other->getPosition();

        // 2. Verifica se o bungee está comprimido ou dentro do comprimento de repouso
        real magnitude = force.magnitude();
        if (magnitude <= restLength) return; // Não gera força se não estiver esticado

        // 3. Calcula a magnitude da força elástica de tração
        magnitude = springConstant * (magnitude - restLength);

        // 4. Normaliza o vetor e aplica a força de puxão
        force.normalize();
        force *= -magnitude;
        particle->addForce(force);
    }

    // --- Empuxo / Flutuação (ParticleBuoyancy) ---
    ParticleBuoyancy::ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity)
        : maxDepth(maxDepth), volume(volume), waterHeight(waterHeight), liquidDensity(liquidDensity) {}

    void ParticleBuoyancy::updateForce(Particle* particle, real duration)
    {
        // 1. Obtém a profundidade (coordenada Y do objeto)
        real depth = particle->getPosition().y;

        // 2. Verifica se o objeto está totalmente fora da água (acima da superfície + maxDepth)
        if (depth >= waterHeight + maxDepth) return;

        Vector3 force(0, 0, 0);

        // 3. Verifica se o objeto está totalmente submerso (abaixo da superfície - maxDepth)
        if (depth <= waterHeight - maxDepth)
        {
            force.y = liquidDensity * volume;
            particle->addForce(force);
            return;
        }

        // 4. Caso contrário, o objeto está parcialmente submerso
        force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / (2.0f * maxDepth);
        particle->addForce(force);
    }

    // --- Mola Rígida Simulada / Fake Implicit Spring (ParticleFakeSpring) ---
    ParticleFakeSpring::ParticleFakeSpring(Vector3 *anchor, real springConstant, real damping)
        : anchor(anchor), springConstant(springConstant), damping(damping) {}

    void ParticleFakeSpring::updateForce(Particle* particle, real duration)
    {
        if (!particle->hasFiniteMass()) return;

        // 1. Calcula a posição relativa da partícula em relação à âncora
        Vector3 position = particle->getPosition();
        position -= *anchor;

        // 2. Calcula as constantes do oscilador e verifica limites
        real gamma = 0.5f * real_sqrt(4 * springConstant - damping * damping);
        if (gamma == 0.0f) return;

        Vector3 c = position * (damping / (2.0f * gamma)) + particle->getVelocity() * (1.0f / gamma);

        // 3. Calcula a posição prevista (alvo) ao fim do quadro
        Vector3 target = position * real_cos(gamma * duration) + c * real_sin(gamma * duration);
        target *= real_exp(-0.5f * duration * damping);

        // 4. Calcula a aceleração resultante necessária e converte em força
        Vector3 accel = (target - position) * (1.0f / (duration * duration)) - particle->getVelocity() * duration;
        particle->addForce(accel * particle->getMass());
    }

    // --- Implementação do Registro de Forças ---
    void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator *fg)
    {
        ParticleForceRegistration registration;
        registration.particle = particle;
        registration.fg = fg;
        registrations.push_back(registration);
    }

    void ParticleForceRegistry::remove(Particle* particle, ParticleForceGenerator *fg)
    {
        Registry::iterator i = registrations.begin();
        for (; i != registrations.end(); ++i)
        {
            if (i->particle == particle && i->fg == fg)
            {
                registrations.erase(i);
                break;
            }
        }
    }

    void ParticleForceRegistry::clear()
    {
        registrations.clear();
    }

    void ParticleForceRegistry::updateForces(real duration)
    {
        Registry::iterator i = registrations.begin();
        for (; i != registrations.end(); ++i)
        {
            i->fg->updateForce(i->particle, duration);
        }
    }
}
