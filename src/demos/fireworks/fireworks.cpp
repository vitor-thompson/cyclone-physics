#include <cyclone/particle.h>
#include <cstdlib>

namespace cyclone {

    // Gerador de números aleatórios simples para as variações de velocidade e idade
    class Random {
    public:
        real randomReal(real low, real high) {
            return low + static_cast<real>(rand()) / (static_cast<real>(RAND_MAX / (high - low)));
        }

        Vector3 randomVector(const Vector3 &low, const Vector3 &high) {
            return Vector3(
                randomReal(low.x, high.x),
                randomReal(low.y, high.y),
                randomReal(low.z, high.z)
            );
        }
    };

    /**
     * @brief Fogos de artifício são partículas com dados adicionais de idade e tipo.
     */
    class Firework : public Particle
    {
    public:
        unsigned type;
        real age;

        /**
         * @brief Atualiza o fogo de artifício e reduz o tempo de rastilho (fuse).
         * @return Retorna true se o foguete chegou ao fim da vida e precisa explodir.
         */
        bool update(real duration)
        {
            integrate(duration);
            age -= duration;
            return (age < 0.0f);
        }
    };

    /**
     * @brief Regras que controlam o comportamento, tempo de vida e filhos gerados.
     */
    struct FireworkRule
    {
        unsigned type;
        real minAge;
        real maxAge;
        Vector3 minVelocity;
        Vector3 maxVelocity;
        real damping;

        struct Payload {
            unsigned type;
            unsigned count;
        };

        unsigned payloadCount;
        Payload *payloads;

        void setParameters(unsigned type, real minAge, real maxAge,
                           const Vector3 &minVelocity, const Vector3 &maxVelocity,
                           real damping)
        {
            this->type = type;
            this->minAge = minAge;
            this->maxAge = maxAge;
            this->minVelocity = minVelocity;
            this->maxVelocity = maxVelocity;
            this->damping = damping;
            payloadCount = 0;
            payloads = nullptr;
        }

        /**
         * @brief Cria um novo fogo de artifício com base nesta regra.
         */
        void create(Firework *firework, const Firework *parent = nullptr) const
        {
            Random r;
            firework->type = type;
            firework->age = r.randomReal(minAge, maxAge);
            
            if (parent) {
                firework->setPosition(parent->getPosition());
                Vector3 vel = parent->getVelocity();
                vel += r.randomVector(minVelocity, maxVelocity);
                firework->setVelocity(vel);
            } else {
                firework->setPosition(0.0f, 0.0f, 0.0f);
                firework->setVelocity(r.randomVector(minVelocity, maxVelocity));
            }

            firework->setMass(1.0f);
            firework->setDamping(damping);
            firework->setAcceleration(Vector3(0.0f, -10.0f, 0.0f)); // Gravidade padrão
            firework->clearAccumulator();
        }
    };

} // namespace cyclone
