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
         * @brief Aplica a força ao corpo rígido fornecido para o intervalo de tempo dado.
         */
        virtual void updateForce(RigidBody *body, real duration) = 0;
    };

    /**
     * @brief Aplica força gravitacional ao centro de massa.
     */
    class Gravity : public ForceGenerator
    {
        Vector3 gravity;

    public:
        Gravity(const Vector3 &gravity);
        virtual void updateForce(RigidBody *body, real duration) override;
    };

    /**
     * @brief Aplica força de mola entre dois pontos específicos de corpos rígidos.
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
     * @brief Gerador de força aerodinâmica baseado em um tensor de sustentação/arrasto.
     */
    class Aero : public ForceGenerator
    {
    protected:
        /** Tensor aerodinâmico da superfície no espaço local (body space). */
        Matrix3 tensor;

        /** Posição relativa da superfície aerodinâmica no corpo rígido. */
        Vector3 position;

        /** Ponteiro para o vetor de velocidade do vento no ambiente. */
        const Vector3* windspeed;

    public:
        Aero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed);

        virtual void updateForce(RigidBody *body, real duration) override;

    protected:
        /** Método protegido para aplicar força usando um tensor específico. */
        void updateForceFromTensor(RigidBody *body, real duration, const Matrix3 &tensorToUse);
    };

    /**
     * @brief Gerador aerodinâmico com superfícies de controle interpoladas (ex: ailerons, profundor).
     */
    class AeroControl : public Aero
    {
        Matrix3 maxTensor;
        Matrix3 minTensor;
        real controlSetting; // Varia de -1.0 (min) a +1.0 (max)

        Matrix3 getTensor();

    public:
        AeroControl(const Matrix3 &base, const Matrix3 &min, const Matrix3 &max,
                    const Vector3 &position, const Vector3 *windspeed);

        /** Define a posição da superfície de controle entre -1.0 e +1.0. */
        void setControl(real value);

        virtual void updateForce(RigidBody *body, real duration) override;
    };

    /**
     * @brief Gerador aerodinâmico com orientação rotacional ajustável (ex: vela de barco, leme náutico).
     */
    class AngledAero : public Aero
    {
        /** Orientação relativa da superfície em relação ao corpo rígido. */
        Quaternion orientation;

    public:
        AngledAero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed);

        /** Define a rotação/orientação relativa da superfície aerodinâmica. */
        void setOrientation(const Quaternion &quat);

        virtual void updateForce(RigidBody *body, real duration) override;
    };

    /**
     * @brief Gerador de empuxo/flutuação para corpos rígidos em superfícies líquidas.
     */
    class Buoyancy : public ForceGenerator
    {
        real maxDepth;
        real volume;
        real waterHeight;
        real liquidDensity;
        Vector3 centerOfBuoyancy;

    public:
        Buoyancy(const Vector3 &cOfB,
                 real maxDepth, real volume, real waterHeight,
                 real liquidDensity = 1000.0f);

        virtual void updateForce(RigidBody *body, real duration) override;
    };

}
