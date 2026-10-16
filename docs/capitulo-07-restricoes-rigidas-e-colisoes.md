# Capítulo 7: Restrições Rígidas, Colisões e Links

Este documento sintetiza os conceitos físicos, modelos matemáticos e a implementação prática desenvolvida no Capítulo 7 do motor de física *Cyclone*, marcando a transição de geradores de força contínua para resolução de impactos instantâneos e contatos geométricos.

---

## 1. Fundamentos Físicos: Velocidade de Separação e Coeficiente de Restituição

Diferente das molas (que atuam suavemente ao longo do tempo), as colisões e restrições rígidas exigem correções instantâneas de velocidade através de **impulsos**.

### Velocidade de Separação ($v_s$)
A taxa na qual dois objetos se afastam (ou se aproximam, se o valor for negativo) ao longo da normal de contato $\vec{n}$ é dada por:
$$v_s = (\dot{p}_a - \dot{p}_b) \cdot \vec{n}$$

### Coeficiente de Restituição ($e$)
A elasticidade do choque relaciona a velocidade de separação antes ($\nu_s$) e depois ($\nu'_s$) do impacto:
$$\nu'_s = -e \cdot \nu_s$$

* Se $e = 1$, o choque é perfeitamente elástico (ricochete total).
* Se $e = 0$, o choque é perfeitamente inelástico (os corpos grudam ou param a aproximação).

---

## 2. O Conceito de Impulso ($g$)

Um impulso é uma variação instantânea de quantidade de movimento que altera diretamente a velocidade de um corpo, sem passar pela aceleração contínua:
$$g = m \Delta \dot{p} \implies \dot{p}' = \dot{p} + \frac{1}{m} g$$

---

## 3. Resolução de Interpenetração (Correção Geométrica)

Quando os objetos se sobrepõem devido à discretização do tempo, aplicamos uma correção de posição proporcional à massa inversa de cada corpo:
$$\Delta x = \frac{-d}{\sum m^{-1}} \cdot \frac{1}{m} \cdot \vec{n}$$
Onde $d$ é a profundidade de interpenetração e $\vec{n}$ é a normal de contato.

---

## 4. Implementação Prática: Resolução de Velocidade de Contato

Abaixo está o trecho principal que calcula e aplica o impulso corretivo no motor *Cyclone*:

```cpp
void ParticleContact::resolveVelocity(real duration)
{
    real separatingVelocity = calculateSeparatingVelocity();

    // Se já estão se afastando, nenhum impulso é necessário
    if (separatingVelocity > 0) return;

    real newSepVelocity = -separatingVelocity * restitution;

    // Correção para contatos de repouso (evita vibração por gravidade acumulada)
    Vector3 accCausedVelocity = particle[0]->getAcceleration();
    if (particle[1]) accCausedVelocity -= particle[1]->getAcceleration();
    real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

    if (accCausedSepVelocity < 0) {
        newSepVelocity += restitution * accCausedSepVelocity;
        if (newSepVelocity < 0) newSepVelocity = 0;
    }

    real deltaVelocity = newSepVelocity - separatingVelocity;
    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1]) totalInverseMass += particle[1]->getInverseMass();

    if (totalInverseMass <= 0) return;

    real impulse = deltaVelocity / totalInverseMass;
    Vector3 impulsePerIMass = contactNormal * impulse;

    // Aplicação do impulso proporcional à massa inversa
    particle[0]->setVelocity(
        particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass()
    );

    if (particle[1]) {
        particle[1]->setVelocity(
            particle[1]->getVelocity() + impulsePerIMass * -particle[1]->getInverseMass()
        );
    }
}

```

## 5. Modelagem de Links (Cabos e Hastes)
   
   Reutilizamos a infraestrutura de contatos para simular restrições estruturais:
   
   Cabos (ParticleCable): Geram um contato com normal invertida (tração) apenas quando o comprimento atual excede o limite máximo ($l > l_{max}$).
   
   Hastes (ParticleRod): Geram dois contatos automáticos (com restituição zero) para impedir tanto o estiramento quanto a compressão, mantendo uma distância estritamente fixa entre as partículas ($l = l_{fixo}$).
  
