/*
 * Definição do corpo rígido (RigidBody) do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#ifndef CYCLONE_BODIES_H
#define CYCLONE_BODIES_H

#include "cyclone/core.h"

namespace cyclone {

    class RigidBody
    {
    public:
        // Posição linear do corpo no espaço do mundo
        Vector3 position;

        // Orientação do corpo no espaço do mundo (armazenada como Quatérnio)
        Quaternion orientation;

        // Velocidade linear do corpo no espaço do mundo
        Vector3 velocity;

        // Velocidade angular (rotação) do corpo no espaço do mundo
        Vector3 rotation;

    protected:
        // Aceleração linear acumulada no quadro atual
        Vector3 acceleration;

        // Aceleração linear registrada no último quadro (usada para contatos de repouso)
        Vector3 accelerationAtUpdate;

        // Força acumulada a ser aplicada no próximo INTEGRATE
        Vector3 forceAccum;

        // Torque acumulado a ser aplicado no próximo INTEGRATE
        Vector3 torqueAccum;

        // Inversão da massa (1 / massa). Se for 0, o corpo é imóvel (infinito)
        real inverseMass;

        // Tensor de inércia inverso no espaço local do corpo
        Matrix3 inverseInertiaTensor;

        // Tensor de inércia inverso transformado para o espaço do mundo (atualizado a cada quadro)
        Matrix3 inverseInertiaTensorWorld;

        // Coeficiente de amortecimento linear (arrasto)
        real linearDamping;

        // Coeficiente de amortecimento angular
        real angularDamping;

        // Matriz de transformação que mapeia o espaço local para o espaço do mundo
        Matrix4 transformMatrix;

        // Indica se o corpo está dormindo (otimização de repouso)
        bool isAwake;

        // Indica se o corpo pode dormir
        bool canSleep;

    public:
        // Construtor e Destrutor
        RigidBody();
        ~RigidBody();

        // Integração de movimento por quadro
        void integrate(real duration);

        // Atualização de dados derivados (matriz de transformação, tensor de inércia no mundo)
        void calculateDerivedData();

        // Gerenciamento de forças e torques
        void addForce(const Vector3 &force);
        void addForceAtPoint(const Vector3 &force, const Vector3 &point);
        void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
        void clearAccumulators();

        // Getters e Setters de Massa e Inércia
        void setMass(real mass);
        real getMass() const;
        void setInverseMass(real inverseMass);
        real getInverseMass() const;
        bool hasFiniteMass() const;

        void setInertiaTensor(const Matrix3 &inertiaTensor);
        void getInertiaTensor(Matrix3 *inertiaTensor) const;
        Matrix3 getInertiaTensor() const;
        void getInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const;
        Matrix3 getInverseInertiaTensor() const;
        void getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const;
        Matrix3 getInverseInertiaTensorWorld() const;

        // Getters e Setters de Posição e Orientação
        void setPosition(const Vector3 &position);
        void getPosition(Vector3 *position) const;
        Vector3 getPosition() const;

        void setOrientation(const Quaternion &orientation);
        void getOrientation(Quaternion *orientation) const;
        Quaternion getOrientation() const;

        void setTransform(const Matrix4 &matrix);
        void getTransform(Matrix4 *matrix) const;

        // Getters e Setters de Velocidade
        void setVelocity(const Vector3 &velocity);
        void getVelocity(Vector3 *velocity) const;
        Vector3 getVelocity() const;

        void setRotation(const Vector3 &rotation);
        void getRotation(Vector3 *rotation) const;
        Vector3 getRotation() const;

        // Estado de Sono (Awake / Sleep)
        bool getAwake() const { return isAwake; }
        void setAwake(bool awake = true);
        void setAsleep(bool asleep) { setAwake(!asleep); }

        // Aceleração do último quadro (para o Capítulo 15)
        Vector3 getLastFrameAcceleration() const { return accelerationAtUpdate; }
    };

} // namespace cyclone

#endif // CYCLONE_BODIES_H
