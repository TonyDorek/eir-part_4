## Analysis and Control of Multi-Robot Systems - Exam project

### Changes for the five_rob_n_obs
n_robots è una variabile ma lasciatela a 5, non ho implementato la gestione del cambiamento di questa variabile quindi se non è 5 rompe tutto
Cambiamenti importanti:
- initialize_randomized.m è quella da chiamare per inizializzare i parametri, cose importanti:
  - random=true; se volete randomizzare, random=false setta 5 robot e 2 ostacoli come la vecchia simulazione con quelle posizioni
  - n_obj per scegliere quanti ostacoli generare
  - spawnrange=40 è un parametro che dice x e y vengono randomicamente generate da -40 a 40, cambiabile quanto ve pare
  - per adesso è più basso del range di connettività quindi so quasi sempre tutti connessi ma basta cambiare i parametri
  - utilizza le funzioni ausiliare randompos e randomobj per generare posizioni iniziali/finali di robot e la seconda posizioni degli ostacoli
- Funzione cbf e cbf_grad cambiate per fare in modo che vengano aggiornate dinamicamente (almeno sugli ostacoli) hanno n_robots, n_obj come parametri ma non vengono presi dall' input ma dal workspace (li ho settati come Parameters e non Input, inoltre ho spuntato via "tunable" altrimenti si incazza)
- "obj1" "obj2" non esistono più ora ci sta "obstacles" anche su simulink
- Barrier Certificate Enforcement prende parametri dal workspace per configurare quanti stati, azioni e certificati

 






### Decentralized safety certificates for multiagent
List of files:

1) init.m -> MATLAB script with initialization parameters

2) model.slx -> Simulink model of the system


3) visualize.m -> MATLAB script for visualization purposes
