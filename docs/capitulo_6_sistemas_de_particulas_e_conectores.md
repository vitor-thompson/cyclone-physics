# Capitulo 6: Sistemas de Particulas e Conectores (Particle-Based Simulators and Links)

Este documento registra os conceitos e a estrutura para construcao de conectores e simuladores baseados em particulas no motor **Cyclone**, correspondente ao **Capitulo 6** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 6 expande o uso de particulas criando restricoes entre elas (como cabos e hastes rigidas) e organizando a atualizacao da simulacao em um framework unificado.

## 🔗 1. Conectores de Particulas (`ParticleLink`)

Um conector e uma abstracao que gera contatos entre duas particulas (ou entre uma particula e um ponto fixo) sempre que uma determinada condicao geometrica e violada.

A classe base abstrata `ParticleLink` herda de `ParticleContactGenerator` e serve como base para implementar cabos e hastes.

### 1.1 Cabos (`ParticleCable`)

Um cabo impede que duas particulas se afastem alem de uma distancia maxima limite ($l_{\text{max}}$), mas permite que elas se aproximem livremente.

* **Condicao de Violacao:** Distancia atual $d > l_{\text{max}}$.
* **Normal de Contato:** Aponta da segunda particula para a primeira.
* **Penetracao (Separacao em atraso):** $g = d - l_{\text{max}}$.
* **Restituicao:** Coeficiente de restituicao ajustavel (geralmente baixo para evitar quiques em cabos esticados).

### 1.2 Hastes Rigidas (`ParticleRod`)

Uma haste rigida mantem a distancia entre duas particulas estritamente constante em relacao ao seu comprimento inicial ($l$).

* **Condicao de Violacao:** Distancia atual $d \neq l$.
* **Se $d > l$ (Extensao):** Gera um contato com normal apontando da segunda particula para a primeira e penetracao $g = d - l$.
* **Se $d < l$ (Compressao):** Gera um contato com normal invertida (apontando da primeira particula para a segunda) e penetracao $g = l - d$.
* **Restituicao:** Zerada ($e = 0$) para impedir oscilacoes elaticas na haste.

## ⚙️ 2. Simuladores de Particulas e Gerenciamento

Para facilitar a atualizacao de multiplos objetos e conexoes no loop do jogo, o capitulo introduz o gerenciamento unificado de estado:

* **Geracao Dinamica de Contatos:** A lista de geradores de contato (`ParticleContactGenerator`) e percorrida a cada quadro para preencher o buffer global de contatos.
* **Integracao em Lote:** As particulas cadastradas passam pela etapa de integracao de forcas e atualizacao de posicao/velocidade de forma centralizada.
* **Resolucao de Restricoes:** O resolvedor de contatos trata simultaneamente colisoes com o ambiente e violacoes de cabos/hastes de forma iterativa.