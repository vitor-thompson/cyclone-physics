#pragma once

#include <cyclone/body.h>

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
     * @brief Aplica força de mola entre pontos específicos de corpos rígidos.
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

    /**
     * @brief Gerador de força que aplica forças aerodinâmicas baseadas em tensores.
     */
    class Aero : public ForceGenerator
    {
    protected:
        /** Tensor aerodinâmico da superfície no espaço local (body space). */
        Matrix3 tensor;

        /** Posição relativa da superfície aerodinâmica no corpo. */
        Vector3 position;

        /** Ponteiro para o vetor de velocidade do vento no ambiente. */
        const Vector3* windspeed;

    public:
        Aero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed);

        virtual void updateForce(RigidBody *body, real duration) override;

    protected:
        /** Atualiza a força usando um tensor específico (usado por AeroControl). */
        void updateForce(RigidBody *body, real duration, const Matrix3 &tensor);
    };

    /**
     * @brief Gerador de força aerodinâmica com superfícies de controle (asas móveis, lemes).
     */
    class AeroControl : public Aero
    {
        Matrix3 maxTensor;
        Matrix3 minTensor;
        real controlSetting; // Varia de -1 a +1

        Matrix3 getTensor();

    public:
        AeroControl(const Matrix3 &base, const Matrix3 &min, const Matrix3 &max,
                    const Vector3 &position, const Vector3 *windspeed);

        void setControl(real value);

        virtual void updateForce(RigidBody *body, real duration) override;
    };

}
