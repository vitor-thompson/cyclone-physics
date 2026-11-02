# Capítulo 17: Juntando Tudo (Putting It All Together)

Este documento registra a análise de arquitetura e as aplicações práticas desenvolvidas no **Capítulo 17** do livro *Game Physics Engine Development* (Ian Millington). 

Neste capítulo, o motor **Cyclone** é consolidado como um sistema completo de física de corpos rígidos 3D e aplicado na resolução de problemas práticos comuns em jogos modernos: **Ragdolls**, **Fraturas de Objetos** e **Explosões Cinematográficas**.

---

## 🏗️ 1. Arquitetura Geral do Motor (17.1)

O pipeline do *Cyclone* é estruturado em **quatro componentes de processamento** e **três tipos essenciais de dados**, garantindo modularidade e integração com o loop do jogo.

### 4 Componentes do Pipeline
1. **Geradores de Forças e Torques:** Analisam o estado da simulação e aplicam forças diretamente nos corpos rígidos afetados.
2. **Simulador de Corpos Rígidos:** Atualiza posições, orientações, velocidades lineares e angulares aplicando a integração numérica.
3. **Detector de Colisões:** Identifica interpenetrações e gera uma lista de estruturas `Contact`.
4. **Resolvedor de Colisões (`ContactResolver`):** Ajusta velocidades e posições dos corpos rígidos para satisfazer as restrições físicas dos contatos.

### 3 Tipos de Dados dos Objetos
* **Geometria de Renderização:** Malha visual e materiais (não utilizados diretamente no processamento físico).
* **Geometria de Colisão:** Formas primitivas simplificadas (caixas, esferas, planos) utilizadas no cálculo de interseção.
* **Dados do Corpo Rígido (`RigidBody`):** Massa, tensor de inércia, posição, orientação, velocidades, atrito e restituição.

---

## 🎮 2. Aplicações Práticas (17.2)

### 2.1 Ragdolls e Juntas (`joints.h`)
* **Conceito:** Personagens ou esqueletos simulados proceduralmente por meio de corpos rígidos ("ossos") conectados por restrições ("juntas").
* **Implementação de Juntas:**
  * Definiu-se a classe `cyclone::Joint`, herdada de `ContactGenerator`.
  * As juntas operam como **ball-joints** (juntas esféricas), mantendo dois pontos locais ancorados.
  * Quando ocorre um afastamento acima de um limiar (`error`), um contato especial é gerado dinamicamente com **restituição $0$** e **fricção infinita** para impedir desalinhar.

### 2.2 Física de Fratura (`fracture.cpp`)
* **Conceito:** Destruição de objetos frágeis em tempo real com base no ponto de impacto da colisão (*on-the-fly*).
* **Decomposição de Blocos:**
  * O ponto de contato exato determina o centro dos planos de corte.
  * O método `divideBlock` fragmenta o bloco atingido em 8 novos sub-blocos menores.
* **Recálculo de Propriedades Físicas (`calculateMassProperties`):**
  * Para cada fragmento gerado, calcula-se o novo volume e massa proporcional à densidade do bloco original.
  * O novo tensor de inércia é recalculado analiticamente tratando cada pedaço como um paralelepípedo retângulo retangular.

### 2.3 Explosões Cinematográficas (`fgen.h`)
Explosões em jogos são modeladas com três componentes através de uma subclasse customizada de `ForceGenerator`:
1. **Implosão:** Sucção inicial do ar em direção ao centro de detonação em um intervalo curto de tempo (`implosionDuration`).
2. **Onda de Concussão (Shockwave):** Frente de alta pressão em expansão radial rápida (`concussionWaveSpeed`). Para forçar rotação/torque nos objetos arremessados, a força é aplicada em um ponto descentralizado (`addForceAtPoint`).
3. **Chaminé de Convecção:** Corrente térmica ascendente aplicada em formato cilíndrico sobre a explosão, simulando o efeito de nuvem de cogumelo.

---

## ⚠️ 3. Limitações Estruturais do Motor (17.3)

Devido à escolha de um resolvedor iterativo e sequencial de contatos em vez de um sistema matricial simultâneo, o motor apresenta gargalos conhecidos em certos cenários:

1. **Pilhas Instáveis (*Stacking*):** Acúmulo de erros de penetração ao longo da pilha pode fazer objetos no topo vibrarem.
2. **Atrito em Forças de Reação:** O atrito não é aplicado durante a resolução da penetração posicional, permitindo deslizamentos não-físicos entre corpos em repouso.
3. **Correntes Longas de Juntas:** Cadeias extensas de articulações sofrem com estiramento ou divergência computacional.
4. **Molas Rígidas (*Stiff Springs*):** Constantes elásticas elevadas exigem integração implícita ou passos de tempo extremamente reduzidos para evitar explosões numéricas.

---
*Status: Capítulo 17 concluído com sucesso. Projeto pronto para a análise final de arquitetura e resolvedores simultâneos no Capítulo 18.*