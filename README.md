# Projeto Embarcados

Desenvolvendo uma luva que controla jogos eletrônicos.

## Entrega 1

### Integrantes

- João Gabriel Valentim
- Hudson Monteiro Araújo

### Ideia

<!--  Descreva aqui em poucas palavras qual a ideia do seu controle. Se ele vai ser de jogo ou de aplicação -->
A ideia central do projeto é controlar (outros softwares/aplicações) por meio de processamento de imagem, áudio, sensores contráteis e botões de membrana.
Então por meio da luva será possível identificar com gestos, tanto com o reconhecimento da imagem como um movimento específico, ou por meio de comando de voz a realização de uma tarefa.

### Nome

<!-- De um nome ao seu controle! -->
Manopla do infito

### Usuários 

<!-- Descreva aqui quem seriam os possíveis usuários deste controle. -->
A luva é especialmente útil para pessoas com mobilidade reduzida ou que precisam de uma forma mais intuitiva e precisa de controle de dispositivos eletrônicos.


### Software/Jogo 

<!-- Qual software que seu controle vai controlar? -->
O software que queremos controlar para música é o spotify

### Jornada do usuários (3 pts)

<!-- Descreva ao menos duas jornadas de usuários distintos, é para caprichar! -->
jornada 1:
O usuário liga o sistema de som e o controle, e a matriz de botões acende indicando que está pronto para uso.

O usuário decide utilizar o comando de voz para começar a tocar música. Ele segura o botão do comando de voz e diz "tocar música". O sensor de áudio detecta a voz do usuário e envia um sinal para a caixa de som, que começa a tocar a música.

O usuário quer aumentar o volume. Ele usa a matriz de botões para selecionar o controle de volume. A caixa de som recebe o sinal da matriz e aumenta o volume de acordo.

A música está tocando, mas o usuário quer mudar de música. Ele seleciona o controle de avançar ou voltar na matriz de botões e faz um gesto com a mão para a direita, indicando que quer avançar para a próxima música. O sensor de movimento detecta o gesto e envia um sinal para a caixa de som, que passa para a próxima música.

O usuário decide mudar a música novamente, mas desta vez ele prefere usar o comando de voz. Ele segura o botão do comando de voz novamente e diz "tocar próxima música". O sensor de áudio detecta a voz do usuário e envia um sinal para a caixa de som, que passa para a próxima música.

Finalmente, o usuário decide parar de usar o controle e desliga a caixa de som e o controle. Ele pressiona o botão de desligar na matriz de botões e o controle se desliga.

jornada 2:



### Comandos/ Feedbacks (2 pts)

<!-- 
Quais são os comandos/ operacões possíveis do seu controle?

Quais os feedbacks que seu controle vai fornecer ao usuário?
-->
Comandos de gestos:
Mostrar o dedo para a esquerda ou direita para mudar de música.
Fazer um movimento para pausar ou retomar a reprodução de música.


## In/OUT (3 pts)

<!--
Para cada Comando/ Feedback do seu controle, associe qual sensores/ atuadores pretende utilizar? Faca em formato de lista, exemplo:

- Avanca música: Push button amarelo
- Volume da música: Fita de LED indicando potência do som
-->
Reconhecer gestos, Comandos de avançar ou voltar : Camêra OpenMV M7
Comando de voz : Sensor de áudio LMV324
Volume da música: Matriz de botões
Pause/Play : Sensor contrátil

### Design (2 pts)

<!--
Faca um esboco de como seria esse controle (vai ter uma etapa que terão que detalhar melhor isso).
-->
