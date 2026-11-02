# Resumo da Arquitetura e Componentes do Motor Físico Cyclone

Este documento apresenta uma visão geral detalhada de todos os arquivos de cabeçalho (`.h`) e implementação (`.cpp`) que compõem o motor físico 3D **Cyclone**, divididos por suas camadas de responsabilidade e com explicações sobre as principais funções dentro de cada módulo.

---

## 1. Módulo Core e Matemática Base

### `precision.h`
* **Tipo:** Cabeçalho
* **Descrição:** Define o nível de precisão de ponto flutuante utilizado por toda a engine (`float` para precisão simples ou `double` para alta precisão), além de funções matemáticas universais (como `real_pow`, `real_abs`, `real_sqrt`).

### `core.h` & `core.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Trata da matemática vetorial e matricial fundamental em 3D.
* **Principais Funções e Classes:**
  * `Vector3`: Representa posições, velocidades e forças no espaço tridimensional.
    * `magnitude()`, `squareMagnitude()`: Calculam o comprimento do vetor.
    * `normalize()`: Converte o vetor para magnitude unitária.
    * Operadores `%` (Produto Vetorial / Cross Product) e `*` (Produto Escalar / Dot Product).
  * `Matrix3`: Matriz 3x3 para tensores de inércia e transformações de rotação puro.
    * `setInverse()`, `inverse()`: Inversão de matrizes 3x3.
    * `setOrientation()`: Converte um quatérnion em matriz de rotação 3x3.
  * `Matrix4`: Matriz de transformação afim 3x4 (posição e orientação combinadas).
    * `setOrientationAndPos()`: Monta a matriz de transformação afim.
    * `transform()`, `transformInverse()`: Transforma pontos entre espaço local e espaço do mundo.
  * `Quaternion`: Representa a orientação 3D sem o problema de *Gimbal Lock*.
    * `addScaledVector()`: Aplica rotação com base na velocidade angular durante um intervalo de tempo.

---

## 2. Subsistema de Partículas (Pontos de Massa)

### `particle.h` & `particle.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Modela objetos pontuais sem dimensão angular (sem rotação), apenas com translacão.
* **Principais Funções:**
  * `Particle::integrate(real duration)`: Executa a integração semi-implícita de Euler. Atualiza a posição com base na velocidade e a velocidade com base na aceleração e acumulador de forças.
  * `Particle::addForce(const Vector3 &force)`: Adiciona uma força ao acumulador do quadro.

### `pfgen.h` & `pfgen.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Geradores de força dedicados a partículas e o registro central de forças.
* **Principais Funções:**
  * `ParticleGravity::updateForce()`: Aplica a força da gravidade ($F = m \cdot g$).
  * `ParticleDrag::updateForce()`: Aplica resistência do ar/fluido (arrasto linear e quadrático).
  * `ParticleAnchoredSpring::updateForce()`: Aplica a Lei de Hooke para molas presas a um ponto fixo.
  * `ParticleForceRegistry::updateForces()`: Percorre todas as associações entre partículas e geradores e aplica as forças correspondentes.

### `pcontacts.h` & `pcontacts.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Resolução de colisões e contatos pontuais entre partículas.
* **Principais Funções:**
  * `ParticleContact::resolve(real duration)`: Executa a resolução de velocidade e a correção de interpenetração.
  * `ParticleContact::calculateSeparatingVelocity()`: Retorna a velocidade com que as duas partículas estão se afastando ao longo da normal do contato.
  * `ParticleContactResolver::resolveContacts()`: Processa iterativamente os contatos mais críticos para evitar colapsos físicos.

### `plinks.h` & `plinks.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Restrições geométricas entre partículas (cabos e hastes).
* **Principais Funções:**
  * `ParticleCable::addContact()`: Gera um contato sintético de tensão quando a distância entre duas partículas excede o limite do cabo.
  * `ParticleRod::addContact()`: Gera um contato tanto para compressão quanto para extensão para manter uma distância estritamente fixa.

### `pworld.h` & `pworld.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Gerenciador global do subsistema de partículas.
* **Principais Funções:**
  * `ParticleWorld::runPhysics(real duration)`: Executa o ciclo de quadro: limpa acumuladores, atualiza forças, integra movimento, gera contatos sintéticos e os resolve.

---

## 3. Subsistema de Corpos Rígidos (Corpos com Volume e Rotação)

### `body.h` & `body.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Modela corpos rígidos tridimensionais que possuem massa, tensor de inércia, rotação e translação.
* **Principais Funções:**
  * `RigidBody::integrate(real duration)`: Atualiza a posição linear e a orientação (quatérnion) com base em forças e torques acumulados.
  * `RigidBody::calculateDerivedData()`: Recalcula a matriz de transformação no mundo (`transformMatrix`) e o tensor de inércia inverso no espaço global (`inverseInertiaTensorWorld`).
  * `RigidBody::addForceAtPoint()`: Aplica uma força em um ponto do espaço global, gerando tanto aceleração linear quanto torque ($\tau = r \times F$).

### `fgen.h` & `fgen.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Geradores de força e torque para corpos rígidos.
* **Principais Funções:**
  * `Gravity::updateForce()`: Aplica força gravitacional sobre o centro de massa.
  * `Spring::updateForce()`: Aplica força elástica entre dois corpos rígidos em pontos de conexão arbitrários.
  * `ForceRegistry::updateForces()`: Atualiza as forças de todos os corpos rígidos cadastrados.

---

## 4. Detecção de Colisão e Resolução de Contatos de Corpos Rígidos

### `collide_coarse.h` & `collide_coarse.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Detecção ampla de colisões (*Broad Phase*) via Árvores de Hierarquia de Volumes Envolventes (BVH).
* **Principais Funções:**
  * `BoundingSphere::overlaps()`: Testa se duas esferas envolventes se sobrepõem.
  * `BVHNode::insert()`: Insere dinamicamente um novo corpo rígido na árvore espacial.
  * `generatePotentialContacts()`: Percorre a árvore BVH descartando pares distantes e retornando apenas pares que potencialmente colidiram.

### `collide_fine.h` & `collide_fine.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Detecção fina de colisões (*Narrow Phase*) entre primitivas geométricas (Esferas, Planos e Caixas OBB).
* **Principais Funções:**
  * `IntersectionTests::sphereAndSphere()`: Teste exato entre duas esferas.
  * `IntersectionTests::boxAndHalfSpace()`: Testa intersecção entre uma caixa OBB e um plano.
  * `CollisionDetector::sphereAndSphere()` / `sphereAndHalfSpace()`: Detectam a colisão e preenchem a estrutura `RigidBodyContact` com a normal, profundidade e ponto de contato.

### `contacts.h` & `contacts.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** Resolução de impulsos e interpenetrações para corpos rígidos com atrito e restituição.
* **Principais Funções:**
  * `RigidBodyContact::calculateInternals()`: Prepara as matrizes de base de contato, posições relativas e a velocidade de separação inicial.
  * `ContactResolver::resolveContacts()`: Algoritmo iterativo que resolve alternadamente impulsos de velocidade e deslocamentos de posição para prevenir afundamentos e calcular impactos/atritos.

---

## 5. Gerenciamento Global do Mundo Rígido

### `world.h` & `world.cpp`
* **Tipo:** Cabeçalho e Implementação
* **Descrição:** O motor principal que coordena a simulação unificada do mundo de corpos rígidos.
* **Principais Funções:**
  * `World::runPhysics(real duration)`: Coordenador mestre. Atualiza forças, integra corpos, executa fases ampla e fina de colisão e chama o resolvedor de contatos.
  * `World::generateContacts()`: Agrupa contatos vindos de geradores customizados e do detector de colisões finas.

---

## Fluxo de Execução de um Quadro Físico (*Physics Frame*)

```
 +-------------------------------------------------------+
 |                 World::runPhysics()                   |
 +-------------------------------------------------------+
                             |
                             v
   1. REGISTRO DE FORÇAS (ForceRegistry::updateForces)
      Calcula Gravidade, Molas, Drag e Torques.
                             |
                             v
   2. INTEGRAÇÃO (RigidBody::integrate)
      Atualiza Velocidades, Posições e Quatérnions de Rotação.
                             |
                             v
   3. DETECÇÃO AMPLA (collide_coarse - BVH Tree)
      Filtra e descarta pares de corpos distantes.
                             |
                             v
   4. DETECÇÃO FINA (collide_fine - CollisionDetector)
      Calcula Normais, Pontos de Contato e Interpenetrações.
                             |
                             v
   5. RESOLUÇÃO DE CONTATOS (ContactResolver::resolveContacts)
      Aplica impulsos de velocidade, restituição, atrito 
      e desfaz o afundamento dos corpos.
```