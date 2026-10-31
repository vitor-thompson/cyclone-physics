# Capítulo 16: Estabilidade e Otimização (Stability and Optimization)

Este documento registra a análise teórica, arquitetural e a implementação prática dos recursos descritos no **Capítulo 16** do livro *Game Physics Engine Development* (Ian Millington). 

Neste capítulo, o motor **Cyclone** foi aprimorado para corrigir falhas de estabilidade numérica e receber otimizações globais de desempenho essenciais para simulações em tempo real.

---

## 🛠️ 1. Estabilidade (16.1)

Durante simulações contínuas de longa duração, pequenos erros de precisão de ponto flutuante podem se acumular e causar comportamentos estranhos, como deformações visuais, oscilações involuntárias e deslizamentos pontuais.

### 1.1 Quaternion Drift (16.1.1)
* **Problema:** A integração de rotação acumulada quadro a quadro faz com que o quatérnio perca sua propriedade unitária (comprimento $\ne 1.0$), deformando a matriz de transformação e causando esmagamento visual no modelo 3D.
* **Solução:** Adicionou-se uma etapa explícita de normalização do quatérnio (`orientation.normalize()`) na rotina de integração de `RigidBody` e na reconstrução de dados derivados (`calculateDerivedData`).

### 1.2 Interpenetração em Rampas (16.1.2)
* **Problema:** Um objeto parado em um plano inclinado afunda levemente a cada quadro antes de a colisão ser resolvida. Ao ser empurrado de volta para fora na direção da normal da rampa, o vetor resultante gera um desvio imperceptível ladeira abaixo. Com o tempo, o objeto desliza mesmo com alto atrito.
* **Solução:** Em `Contact::calculateLocalVelocity`, incorporou-se a antecipação da velocidade gerada pela aceleração do último quadro (`lastFrameAcceleration * duration`) projetada no plano tangencial da rampa. Com isso, o resolvedor calcula a força de fricção necessária para anular o movimento tangencial preventivamente.

### 1.3 Estabilidade de Integração (16.1.3)
* **Análise:** O motor utiliza **Newton–Euler de 1ª Ordem** (rápido e robusto). Métodos de ordem superior como **Newton–Euler 2** (que inclui $\frac{1}{2}at^2$) oferecem ganhos marginais em altas taxas de quadros. Já o **Runge–Kutta 4 (RK4)** é desaconselhado para motores de colisão baseados em impulsos discretos, pois exige reavaliações de forças no meio do passo (*mid-step*), gerando alto custo computacional.

### 1.4 Detecção Pessimista de Colisão (16.1.4)
* **Conceito:** Contatos que ocorrem por apenas um quadro podem causar micro-vibrações (*jitter*). A solução é usar uma margem pessimista: detectar contatos levemente antes do toque real e subtrair essa margem da penetração gerada.

### 1.5 Precisão Matemática (16.1.5)
* **Arquitetura (`precision.h`):** O motor usa o tipo genérico `real` e wrappers de funções matemáticas (`real_sqrt`, `real_abs`, etc.). Isso permite alternar facilmente entre precisão simples (`float`) e dupla (`double`) via `#define DOUBLE_PRECISION`.

---

## ⚡ 2. Otimizações (16.2)

### 2.1 Sistema de Sono / Descanso (*Sleep System*) (16.2.1)
O sistema de sono evita o processamento inútil de objetos estáveis e parados, reduzindo drasticamente a carga de trabalho do motor.

* **Atributos Principais em `RigidBody`:**
  * `isAwake`: Booleano indicando se o corpo está ativo.
  * `canSleep`: Booleano definindo se o corpo tem permissão para dormir.
  * `motion`: Métrica escalar do nível recente de movimento.
* **Métrica de Movimento e RWA:**
  * Calcula-se o movimento sem viés de massa: $\text{currentMotion} = |\dot{\mathbf{p}}|^2 + |\boldsymbol{\dot{\theta}}|^2$.
  * Aplica-se uma **Média Ponderada Recente (RWA)** com viés ajustado pelo tempo do quadro:
    $$\text{motion} = \text{bias} \cdot \text{motion} + (1 - \text{bias}) \cdot \text{currentMotion}$$
  * Se `motion < sleepEpsilon`, o objeto entra em repouso (`setAwake(false)`).
* **Propagação de Estado (`matchAwakeState`):**
  * Quando um objeto ativo colide com um dormente, `Contact::matchAwakeState()` acorda o objeto dormente. O efeito propaga-se por cadeias/pilhas de objetos.
  * Aplicações de forças ou torques externos despertam o corpo automaticamente.

### 2.2 Margens de Erro no Resolvedor (16.2.2)
* **Conceito:** A resolução iterativa costuma gastar repetições preciosas realizando micro-ajustes sub-visuais.
* **Implementação:** Definiram-se limites de tolerância `positionEpsilon` e `velocityEpsilon` (padrão $\approx 0.01$). O resolvedor ignora contatos cujos erros de penetração ou velocidade estejam abaixo dessas margens, proporcionando um ganho de velocidade de até 5 vezes.

### 2.3 Agrupamento de Contatos / Batching (16.2.3)
* **Conceito:** Dividir o grafo global de contatos em ilhas/lotes (*batches*) independentes que não interferem entre si, permitindo resolver subconjuntos de colisões isoladamente.

### 2.4 Otimizações de Código e Memória (16.2.4)
* **Data Locality (Localidade de Cache):** Evitar estruturas do tipo *Array of Structures* (AoS) dispersas e priorizar arrays contíguos de dados rígidos (`RigidBody physics[256]`) para evitar falhas de cache da CPU (*cache misses*).
* **Vetorização (SIMD/SSE):** Uso de registradores vetoriais para acelerar multiplicações matriciais e produtos vetoriais em plataformas suportadas.

---

## 📁 3. Resumo das Modificações na Codebase

| Arquivo | Principais Alterações |
| :--- | :--- |
| **`include/cyclone/precision.h`** | Abstração de `real` (float/double) e encapsulamento de rotinas matemáticas. |
| **`include/cyclone/body.h`** | Membros do sistema de sono (`isAwake`, `canSleep`, `motion`), normalização de quatérnios e aceleradores de aplicação de força com despertar automático. |
| **`src/body.cpp`** | Cálculo do RWA de movimento em `clearAccumulators`, normalização de quatérnio em `integrate`, e controle de desativação. |
| **`src/contacts.cpp`** | Compensação de aceleração planar em `calculateLocalVelocity`, rotina `matchAwakeState`, e adição de `positionEpsilon` / `velocityEpsilon` em `ContactResolver`. |

---
*Status: Capítulo 16 concluído com sucesso. Pronto para a aplicação de efeitos físicos avançados no Capítulo 17.*