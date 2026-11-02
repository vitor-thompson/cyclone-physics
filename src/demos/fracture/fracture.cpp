/*
 * Demonstração de Física de Fratura atualizada para o motor Cyclone.
 * Implementa divisão de blocos on-the-fly e recálculo de inércia (Capítulo 17.2.2).
 */

#include "cyclone/cyclone.h"
#include "../app.h"
#include <stdio.h>

#define MAX_BLOCKS 64

struct Block {
    cyclone::RigidBody* body;
    cyclone::Vector3 halfSize;
    cyclone::Matrix4 offset;
    bool exists;

    void calculateMassProperties(cyclone::real invDensity)
    {
        if (invDensity <= 0)
        {
            body->setInverseMass(0);
            body->setInverseInertiaTensor(cyclone::Matrix3());
        }
        else
        {
            cyclone::real volume = halfSize.magnitude() * 2.0f;
            cyclone::real mass = volume / invDensity;
            body->setMass(mass);

            mass *= 0.333f;
            cyclone::Matrix3 tensor;
            tensor.setInertiaTensorCoeffs(
                mass * halfSize.y*halfSize.y + halfSize.z*halfSize.z,
                mass * halfSize.y*halfSize.x + halfSize.z*halfSize.z,
                mass * halfSize.y*halfSize.x + halfSize.z*halfSize.y
            );
            body->setInertiaTensor(tensor);
        }
    }
};

class FractureDemo : public Application
{
protected:
    Block blocks[MAX_BLOCKS];
    unsigned numBlocks;

public:
    FractureDemo();
    virtual ~FractureDemo();

    virtual void initGraphics();
    virtual void update(real duration);
    virtual void display();
    virtual void generateContacts();

    void divideBlock(const cyclone::Contact& contact, Block* target, Block* newBlocks);
    void checkFractures(const cyclone::Contact* contacts, unsigned numContacts);
};

FractureDemo::FractureDemo() : numBlocks(0) {}

FractureDemo::~FractureDemo()
{
    for (unsigned i = 0; i < MAX_BLOCKS; i++) {
        delete blocks[i].body;
    }
}

void FractureDemo::divideBlock(const cyclone::Contact& contact, Block* target, Block* newBlocks)
{
    cyclone::Vector3 normal = contact.contactNormal;
    cyclone::RigidBody *body = contact.body[0];
    if (body != target->body)
    {
        normal.invert();
        body = contact.body[1];
    }

    cyclone::Vector3 point = body->getPointInLocalSpace(contact.contactPoint);
    normal = body->getDirectionInLocalSpace(normal);
    point = point - normal * (point * normal);

    cyclone::Vector3 size = target->halfSize;

    cyclone::RigidBody tempBody;
    tempBody.setPosition(body->getPosition());
    tempBody.setOrientation(body->getOrientation());
    tempBody.setVelocity(body->getVelocity());
    tempBody.setRotation(body->getRotation());
    tempBody.setLinearDamping(body->getLinearDamping());
    tempBody.setAngularDamping(body->getAngularDamping());
    tempBody.setInverseInertiaTensor(body->getInverseInertiaTensor());
    tempBody.calculateDerivedData();

    target->exists = false;

    cyclone::real invDensity = target->halfSize.magnitude() * 8.0f * body->getInverseMass();

    for (unsigned i = 0; i < 8; i++)
    {
        cyclone::Vector3 min, max;
        if ((i & 1) == 0) { min.x = -size.x; max.x = point.x; }
        else { min.x = point.x; max.x = size.x; }
        
        if ((i & 2) == 0) { min.y = -size.y; max.y = point.y; }
        else { min.y = point.y; max.y = size.y; }
        
        if ((i & 4) == 0) { min.z = -size.z; max.z = point.z; }
        else { min.z = point.z; max.z = size.z; }

        cyclone::Vector3 halfSize = (max - min) * 0.5f;
        cyclone::Vector3 newPos = halfSize + min;
        newPos = tempBody.getPointInWorldSpace(newPos);

        newBlocks[i].body->setPosition(newPos);
        newBlocks[i].body->setVelocity(tempBody.getVelocity());
        newBlocks[i].body->setOrientation(tempBody.getOrientation());
        newBlocks[i].body->setRotation(tempBody.getRotation());
        newBlocks[i].body->setLinearDamping(tempBody.getLinearDamping());
        newBlocks[i].body->setAngularDamping(tempBody.getAngularDamping());
        newBlocks[i].halfSize = halfSize;
        newBlocks[i].offset = cyclone::Matrix4();
        newBlocks[i].exists = true;

        newBlocks[i].calculateMassProperties(invDensity);
    }
}

void FractureDemo::checkFractures(const cyclone::Contact* contacts, unsigned numContacts)
{
    // Lógica para verificar limite de ruptura (breaking strain) com base nos contatos
}

void FractureDemo::generateContacts()
{
    cyclone::CollisionPlane plane;
    plane.direction = cyclone::Vector3(0, 1, 0);
    plane.offset = 0;

    for (unsigned i = 0; i < MAX_BLOCKS; i++)
    {
        if (!blocks[i].exists) continue;
        if (!cData.hasMoreContacts()) return;
        cyclone::CollisionDetector::boxAndHalfSpace(*blocks[i].body, plane, &cData);
    }

    checkFractures(cData.contacts, cData.contactCount);
}

void FractureDemo::update(real duration)
{
    generateContacts();
    Application::update(duration);
}

void FractureDemo::display()
{
    // Renderização dos blocos fracionados na tela
}
