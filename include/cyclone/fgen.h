#pragma once

#include <cyclone/body.h>
#include <vector>

namespace cyclone {

    /**
     * @brief Interface base para todos os geradores de força em corpos rígidos.
     */
    class ForceGenerator
    {
    public:
        virtual ~ForceGenerator() {}

        /**
         * @brief Atualiza e aplica a força ao corpo rígido fornecido.
         */
        virtual void updateForce(RigidBody *body, real duration) = 0;
    };

    /**
     * @brief Aplica força gravitacional a corpos rígidos.
     */
    class Gravity : public ForceGenerator
    {
        Vector3 gravity;
    public:
        Gravity(const Vector3 &gravity);
        virtual void updateForce(RigidBody *body, real duration) override;
    };

    /**
     * @brief Aplica força de mola entre pontos de corpos rígidos.
     */
    class Spring : public ForceGenerator
    {
        Vector3 connectionPoint;
        Vector3 otherConnectionPoint;
        RigidBody *other;
        real springConstant;
        real restLength;
    public:
        Spring(const Vector3 &localConnectionPt,
               RigidBody *other,
               const Vector3 &otherConnectionPt,
               real springConstant,
               real restLength);

        virtual void updateForce(RigidBody *body, real duration) override;
    };

}
