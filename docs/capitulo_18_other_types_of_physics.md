# Capítulo 18: Outros Tipos de Física (Other Types of Physics)

## 1. Visão Geral

O Capítulo 18 conclui a jornada teórica e prática do desenvolvimento do motor físico **Cyclone**. Diferente dos capítulos anteriores — focados no desenvolvimento incremental de código —, este capítulo apresenta um panorama comparativo sobre decisões de arquitetura e abordagens alternativas de física utilizadas na indústria de jogos e simulações avançadas.

O motor **Cyclone** adota uma resolução de contatos baseada em **impulsos sequenciais**, o que proporciona alta performance, clareza didática e bom comportamento para uma ampla variedade de aplicações. Contudo, este capítulo explora outras arquiteturas de solução (como resolvedores simultâneos baseados em LCP/Jacobiano e abordagens de coordenadas reduzidas), mapeando suas vantagens, desvantagens e limites operacionais.

---

## 2. Resolução Simultânea de Contatos (Simultaneous Contact Resolution)

Enquanto o Cyclone trata colisões e restrições uma a uma iterativamente (o que pode gerar pequenos desvios, deslizamentos ou tremores em empilhamentos complexos de objetos), motores de nível de produção frequentemente resolvem o conjunto completo de contatos ao mesmo tempo.

* **Física Baseada em Forças:** Em vez de aplicar impulsos rápidos de quadro único para afastar objetos após a interpenetração, esses motores calculam **forças de repouso contínuas** que mantêm os corpos separados de forma estável.
* **O Jacobiano:** É a matriz matemática que atua como um "mapa de causa e efeito", traduzindo como forças ou torques aplicados em um ponto afetam a aceleração de outros componentes do mundo. 
  * O vetor de estado global agrupa todas as forças/torques e acelerações de todos os corpos rígidos em um vetor de tamanho $6n$ (onde $n$ é o número de corpos).
  * A matriz é majoritariamente preenchida por zeros, sendo classificada como uma **matriz esparsa (*sparse matrix*)**.
* **O Problema de Complementaridade Linear (LCP):**
  A equação fundamental do sistema combina forças conhecidas e desconhecidas através do Jacobiano:
  $$\mathbf{J}\mathbf{f}_{\text{contacts}} + \ddot{\mathbf{p}}_{\text{known}} = \ddot{\mathbf{p}}$$
  Para encontrar o vetor $\mathbf{f}$ que satisfaz essa equação respeitando limites físicos (como o limite máximo da força de atrito em comparação com a reação normal), o motor formula um **LCP (Linear Complementary Problem)** ou um problema de programação quadrática (QP).
* **O Algoritmo Pivô:** Soluções baseadas no método de *Lemke* fazem estimativas e corrigem erros iterativamente para convergir na solução correta. Embora estáveis quando funcionam, exigem meses de engenharia robusta para evitar loops infinitos ou falhas numéricas.

---

## 3. Abordagens de Coordenadas Reduzidas (Reduced Coordinate Approaches)

Uma alternativa teórica que busca eliminar graus de liberdade redundantes desde a concepção matemática:

* **Graus de Liberdade (DoF):** No Cyclone, cada corpo possui 12 graus de liberdade livres (posição, orientação, velocidade e rotação) que são corrigidos posteriormente por restrições. Nas coordenadas reduzidas, calcula-se antecipadamente apenas o que é **fisicamente permitido** mover.
* **Vantagens:** Para casos estáticos ou árvores específicas sem ciclos fechados (como esqueletos de *ragdolls* em formato de árvore), as equações de movimento podem ser resolvidas de forma extremamente rápida.
* **Limitações:** Torna-se impraticável para motores de uso geral porque contatos e colisões em jogos aparecem e desaparecem dinamicamente a cada quadro, tornando o recálculo constante das coordenadas reduzidas inviável na prática.

---

## 4. Resumo e Leituras Recomendadas

O capítulo encerra destacando referências fundamentais para quem deseja aprofundar os estudos em física para jogos além do escopo educacional do Cyclone:

1. **Open Dynamics Engine (ODE):** Uma biblioteca de código aberto popular em projetos independentes que utiliza arquitetura baseada em força e Jacobiano.
2. **David Eberly (*Game Physics*, 2004):** Excelente referência para levantamento matemático aprofundado e algoritmos isolados.
3. **David Baraff e Andrew Witkin (1997):** Artigos clássicos essenciais para o entendimento prático de motores baseados no Jacobiano e algoritmos de resolução LCP.