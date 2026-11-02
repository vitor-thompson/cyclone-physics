# Capitulo 15: Estabilidade e Otimizacao (Stability and Optimization)

Este documento registra as tecnicas de otimizacao de desempenho e estabilizacao numerica implementadas no motor **Cyclone**, correspondente ao **Capitulo 15** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 15 aborda estrategias cruciais para garantir que a simulacao permaneca fluida, previsivel e computacionalmente viavel quando lidando com cenarios complexos e grandes quantidades de corpos rigidos.

## 💤 1. Estado de Repouso / Desativacao (*Sleeping / Deactivation*)

Processar atualizacoes de integracao e testes de colisao para objetos que estao parados consomem ciclos desnecessarios de CPU. O motor implementa um sistema de **desativacao dinamica** (*sleep mode*).

### 1.1 Criterio de Adormecimento

Um `RigidBody` e colocado em estado de repouso (`isAwake = false`) quando sua **energia cinetica ponderada** permanece abaixo de um determinado limiar de tolerancia (*motion threshold*) por um determinado periodo contínuo de tempo.

$$
E_{\text{motion}} = \alpha \cdot E_{\text{motion\_anterior}} + (1 - \alpha) \cdot \left( \vert{}\mathbf{v}\vert{}^2 + \vert{}\boldsymbol{\omega}\vert{}^2 \right)
$$

* Se $E_{\text{motion}} < \epsilon_{\text{sleep}}$ durante um numero limite de quadros, o corpo adormece.

* **Reativacao (*Waking Up*):** O corpo e forçado a acordar imediatamente se receber uma nova forca/torque externa ou se sofrer um impacto de outro objeto que esteja acordado.

## 🧮 2. Estabilizacao Numerica

Integracoes numericas aproximadas acumulam erros de arredondamento ao longo do tempo. O motor aplica correcoes continuas para evitar instabilidades:

### 2.1 Re-normalizacao de Quaternions

Multiplicacoes sucessivas do quaternion de orientacao por matrizes ou velocidades angulares fazem com que sua norma se desvie de $1$. A cada passo de tempo, o quaternion e explicitamente normalizado:

$$
\mathbf{q}_{\text{norm}} = \frac{\mathbf{q}}{\vert{}\mathbf{q}\vert{}}
$$

### 2.2 Limites de Velocidade (*Speed Clamping*)

Para evitar que erros de integracao gerem velocidades infinitas ou fisicamente impossiveis (gerando o efeito conhecido como "explosao numerica"), as velocidades linear e angular maximas sao truncadas se excederem um teto de seguranca.

### 2.3 Amortecimento (*Damping*)

Pequenos coeficientes de amortecimento linear (`linearDamping`) e angular (`angularDamping`) sao aplicados continuamente no integrador para dissipar micro-energias parasitas originadas por imprecisoes de ponto flutuante.

## ⚙️ 3. Otimizacoes no Resolvedor de Contatos (`ContactResolver`)

O resolvedor iterativo de contatos e um dos maiores gargalos do motor. Para otimizar a execucao:

1. **Limite de Iteracoes Dinamico:** O numero maximo de iteracoes do resolvedor e configurado em relacao direta ao numero de contatos ativos ($N_{\text{iter}} = k \cdot N_{\text{contatos}}$).

2. **Ordenacao de Prioridades:** O resolvedor foca suas iteracoes primeiro nos contatos com maior velocidade de penetracao ou maior sobreposicao posicional, garantindo que os problemas visivelmente mais graves sejam corrigidos primeiro caso o limite de iteracoes seja atingido.

3. **Atualizacao Seletiva:** Apos resolver um contato, apenas os contatos associados aos dois corpos envolvidos sao recalculados no buffer, evitando o custo de atualizar contatos de corpos nao afetados.