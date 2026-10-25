# Capítulo 10: Leis de Movimento para Corpos Rígidos (Motor Cyclone)

Esta documentação resume os conceitos físicos e a implementação do Capítulo 10 do motor de física *Cyclone* (baseado no livro *Game Physics Engine Development* de Ian Millington).

---

## 1. Visão Geral da Física

O Capítulo 10 estende o motor de partículas para **corpos rígidos tridimensionais (3D)**, combinando movimento **linear** e **rotacional**:

1. **Movimento Linear:** Mantido a partir do modelo de partículas, tratando o centro de massa do objeto como a origem.
2. **Movimento Rotacional:** Introduz o torque ($\tau$), o tensor de inércia ($I$) e a aceleração angular ($\ddot{\theta}$).
3. **Efeito Misto:** Forças aplicadas fora do centro de massa geram aceleração linear e torque simultaneamente.

---

## 2. Equações Fundamentais

| Conceito Físico | Relação Linear | Relação Rotacional |
| :--- | :--- | :--- |
| **Segunda Lei de Newton** | $a = m^{-1} \cdot F$ | $\ddot{\theta} = I^{-1} \cdot \tau$ |
| **Geração de Torque** | — | $\tau = p_f \times f$ |
| **Amortecimento (Drag)** | $v = v \cdot d_{linear}^{dt}$ | $\omega = \omega \cdot d_{angular}^{dt}$ |

* **Torque ($\tau = p_f \times f$):** Vetor resultante do produto vetorial entre a força ($f$) e a sua posição de aplicação ($p_f$) em relação ao centro de massa.
* **Tensor de Inércia no Mundo ($I_{world}^{-1}$):** O tensor de inércia local ($I_{body}^{-1}$) é convertido a cada quadro para o espaço do mundo via matriz de rotação, pois o torque e a aceleração angular atuam em coordenadas globais.

---

## 3. Estrutura das Classes Principais

### **RigidBody (`include/cyclone/body.h` / `src/body.cpp`)**
* **Atributos Lineares:** `inverseMass`, `position`, `velocity`, `acceleration`, `forceAccum`, `linearDamping`.
* **Atributos Rotacionais:** `orientation` (Quatérnion), `rotation` (Velocidade Angular), `torqueAccum`, `angularDamping`.
* **Tensores de Inércia:** `inverseInertiaTensor` (local) e `inverseInertiaTensorWorld` (derivado no mundo).
* **Dados Derivados (`calculateDerivedData()`):** Mantém a `transformMatrix` e o `inverseInertiaTensorWorld` sincronizados com a orientação e a posição atuais.

### **Acumulação de Forças e Torques**
* `addForce(f)`: Aplica força no centro de massa (apenas deslocamento linear, $\tau = 0$).
* `addForceAtPoint(f, p)`: Aplica força em um ponto do mundo, gerando deslocamento linear e torque.
* `addForceAtBodyPoint(f, p)`: Converte um ponto do espaço local (*body space*) para o mundo e aplica a força.

### **Integrador (`integrate(duration)`)**
1. Calcula aceleração linear ($a = m^{-1} \cdot F$) e angular ($\ddot{\theta} = I_{world}^{-1} \cdot \tau$).
2. Atualiza velocidades linear (`velocity`) e angular (`rotation`).
3. Aplica amortecimento linear e angular (`linearDamping` / `angularDamping`).
4. Atualiza posição (`position`) e orientação (`orientation`).
5. Atualiza os dados derivados e limpa os acumuladores (`clearAccumulators()`).

---

## 4. Exemplo Prático de Uso

```cpp
#include <cyclone/body.h>
#include <cyclone/fgen.h>

using namespace cyclone;

// 1. Instanciação e configuração do Corpo Rígido
RigidBody box;
box.position = Vector3(0.0f, 5.0f, 0.0f);
box.orientation = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
box.inverseMass = 0.5f; // Massa = 2.0 kg
box.linearDamping = 0.98f;
box.angularDamping = 0.95f;

// Set do Tensor de Inércia para um bloco de dimensão (1, 1, 1)
Matrix3 inertiaTensor;
inertiaTensor.setInertiaTensorCoeffs(1.0f, 1.0f, 1.0f);
box.setInertiaTensor(inertiaTensor);
box.calculateDerivedData();

// 2. Aplicação de uma força fora do centro de massa
// Força empurrando na direção X, aplicada no topo do objeto (+Y local)
Vector3 force(10.0f, 0.0f, 0.0f);
Vector3 applicationPointBodySpace(0.0f, 1.0f, 0.0f);

box.addForceAtBodyPoint(force, applicationPointBodySpace);

// 3. Passo de Simulação (Delta Time = 0.016s / ~60 FPS)
box.integrate(0.016f);
