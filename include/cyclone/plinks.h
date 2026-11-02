/*
 * plinks.h
 * 
 * Este arquivo define as classes de vinculos e restricoes entre particulas no motor
 * fisico Cyclone. Inclui a interface base 'ParticleLink' e suas especializacoes
 * 'ParticleCable' (cabos flexiveis) e 'ParticleRod' (hastes rigidas), alem de ancoras
 * associadas a pontos fixos no espaço.
 * 
 * Parte da biblioteca fisica Cyclone.
 */

#ifndef CYCLONE_PLINKS_H
#define CYCLONE_PLINKS_H

#include "pcontacts.h"

namespace cyclone {

/**
 * @brief Classe base para vinculos que conectam duas particulas gerando contatos sinteticos.
 */
class ParticleLink {
public:
    /**
     * @brief As duas particulas vinculadas pela restricao.
     */
    Particle* particle[2];

protected:
    /**
     * @brief Retorna o comprimento atual da conexao entre as duas particulas.
     */
    real currentLength() const {
        Vector3 relativePos = particle[0]->getPosition() - particle[1]->getPosition();
        return relativePos.magnitude();
    }

public:
    /**
     * @brief Preenche a estrutura de contato se a restricao for violada.
     * 
     * @param contact Ponteiro para a estrutura de contato a ser preenchida.
     * @param limit Numero maximo de contatos que podem ser gerados.
     * @return Quantidade de contatos gerados (0 ou 1).
     */
    virtual unsigned addContact(ParticleContact *contact, unsigned limit) const = 0;
};

/**
 * @brief Vinculo do tipo Cabo: impede que duas particulas se afastem alem de um comprimento maximo.
 */
class ParticleCable : public ParticleLink {
public:
    real maxLength;   /**< Comprimento maximo permitido para o cabo. */
    real restitution; /**< Coeficiente de restituicao do impacto ao esticar o cabo. */

public:
    /**
     * @brief Gera um contato de restricao se o cabo for esticado alem de maxLength.
     */
    virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
};

/**
 * @brief Vinculo do tipo Haste: mantem a distancia entre duas particulas estritamente constante.
 */
class ParticleRod : public ParticleLink {
public:
    real length; /**< Comprimento fixo da haste rigida. */

public:
    /**
     * @brief Gera um contato de restricao para evitar compressao ou extensao da haste.
     */
    virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
};

/**
 * @brief Vinculo base conectando uma particula a um ponto fixo de ancoragem no mundo.
 */
class ParticleConstraint {
public:
    Particle* particle; /**< A particula vinculada a restricao. */
    Vector3 anchor;     /**< A posicao da ancora no espaco do mundo. */

protected:
    /**
     * @brief Retorna a distancia atual entre a particula e o ponto de ancoragem.
     */
    real currentLength() const {
        Vector3 relativePos = particle->getPosition() - anchor;
        return relativePos.magnitude();
    }

public:
    /**
     * @brief Preenche a estrutura de contato se a restricao com a ancora for violada.
     */
    virtual unsigned addContact(ParticleContact *contact, unsigned limit) const = 0;
};

/**
 * @brief Restricao do tipo Cabo Ancorado: impede que a particula se afaste do ponto fixo alem de maxLength.
 */
class ParticleCableConstraint : public ParticleConstraint {
public:
    real maxLength;   /**< Comprimento maximo permitido ate a ancora. */
    real restitution; /**< Coeficiente de restituicao do impacto ao esticar. */

public:
    virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
};

/**
 * @brief Restricao do tipo Haste Ancorada: mantem a distancia da particula em relacao a ancora estritamente constante.
 */
class ParticleRodConstraint : public ParticleConstraint {
public:
    real length; /**< Distancia fixa em relacao a ancora. */

public:
    virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
};

} // namespace cyclone

#endif // CYCLONE_PLINKS_H
