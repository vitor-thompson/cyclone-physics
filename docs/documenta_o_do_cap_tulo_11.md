# Capítulo 11: O Motor de Física de Corpos Rígidos em Ação (Aplicações)

Esta documentação resume a implementação e a teoria física apresentadas no Capítulo 11 do motor de física *Cyclone* (baseado no livro *Game Physics Engine Development* de Ian Millington).

---

## 1. Visão Geral

No Capítulo 11, o motor de física 3D para corpos rígidos é aplicado em dois cenários práticos sem a necessidade de colisões rígidas:
1. **Simulador de Voo (Aerodinâmica):** Modelagem de sustentação (*lift*), arrasto (*drag*) e superfícies de controle (ailerons, leme, profundor).
2. **Simulador Náutico (Barco a Vela e Empuxo):** Modelagem de flutuação para corpos rígidos usando o **Centro de Flutuação** e controle de vela/leme com orientação angular.

---

## 2. Conceitos Físicos e Matemáticos

### 2.1 O Tensor Aerodinâmico ($A$)
Em vez de resolver equações de mecânica dos fluidos, a força gerada pelo ar em uma superfície (como uma asa) é aproximada multiplicando a matriz $3 \times 3$ do tensor aerodinâmico pela velocidade do ar:

$$f_a = A \cdot v_w$$

* **Transformação de Espaço:** A velocidade do vento ($v_w$) é transformada do espaço global (*world space*) para o espaço do corpo rígido (*body space*). A força resultante ($f_a$) é calculada e depois transformada de volta para coordenadas globais.
* **Superfícies de Controle (`AeroControl`):** Utiliza interpolação (*blend*) entre três tensores matrizes ($A_{-1}, A_0, A_1$) de acordo com o comando do jogador (de $-1$ a $+1$).
* **Superfícies Orientáveis (`AngledAero`):** Mantém uma rotação relativa via quatérnion para girar a superfície (ex: velas e lemes) sem mudar a estrutura base da asa.

### 2.2 Flutuação e Torque de Auto-Estabilização (`Buoyancy`)
Diferente das partículas, o empuxo em corpos rígidos atua no **Centro de Flutuação** ($p_{buoyancy}$), que pode ser diferente do centro de massa ($p_{cm}$).

* **Torque de Restauração:** A distância entre o centro de empuxo e o centro de massa gera um torque que desvira o barco quando ele inclina nas ondas.
* **Múltiplos Pontos de Flutuação:** Utilizar múltiplos geradores de empuxo (ex: 4 pontos em um catamarã) simula o comportamento dinâmico de inclinação (*pitching* e *rolling*) de maneira realista.

---

## 3. Geradores de Força Implementados

### `Aero`
Mapeia a velocidade do ar em força de sustentação e arrasto através de um tensor local.

### `AeroControl`
Herda de `Aero`. Permite variar dinamicamente o tensor com base na entrada de controle (posição neutra, máxima e mínima).

### `AngledAero`
Herda de `Aero`. Adiciona uma rotação relativa à superfície aerodinâmica em relação ao corpo rígido.

### `Buoyancy`
Calcula a força ascendente proporcional ao volume imerso e aplica-a no centro de flutuação do corpo rígido.

---

## 4. Exemplo de Código Prático

```cpp
#include <cyclone/body.h>
#include <cyclone/fgen.h>

using namespace cyclone;

// 1. Criando o Corpo Rígido do Barco/Avião
RigidBody boat;
boat.position = Vector3(0.0f, 0.0f, 0.0f);
boat.orientation = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
boat.inverseMass = 0.001f; // 1000 kg

// 2. Configurando o Vetor de Vento Ambiental
Vector3 wind(10.0f, 0.0f, 5.0f);

// 3. Tensor Aerodinâmico para a Vela
Matrix3 sailTensor(
    -0.1f,  0.0f, 0.0f,
     1.0f, -0.5f, 0.0f,
     0.0f,  0.0f, -0.1f
);

// 4. Instanciando Geradores de Força
AngledAero mainSail(sailTensor, Vector3(0.0f, 2.0f, 0.0f), &wind);
Buoyancy aftLeftHull(Vector3(-1.0f, -0.5f, -2.0f), 1.0f, 2.0f, 0.0f);

// 5. Ajustando a Vela e Atualizando Forças na Simulação
mainSail.setOrientation(Quaternion(Vector3(0, 1, 0), 0.5f)); // Girando a vela 0.5 rad
mainSail.updateForce(&boat, 0.016f);
aftLeftHull.updateForce(&boat, 0.016f);

// 6. Integração do Corpo
boat.integrate(0.016f);
```