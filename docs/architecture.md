\# Architecture logicielle ROS – TIAGo Receptionist



Ce document décrit l’architecture logicielle du projet \*\*TIAGo Receptionist\*\*, basée sur le middleware ROS.

L’architecture repose sur une organisation modulaire où chaque fonction du robot est implémentée sous la forme

d’un \*\*nœud ROS autonome\*\*, communiquant avec les autres via des \*\*topics\*\*.



---



\## Principe général



\- Chaque \*\*nœud\*\* correspond à un script Python exécutant une tâche précise (ASR, TTS, perception, dialogue, etc.).

\- Les \*\*topics\*\* assurent la communication asynchrone entre les nœuds.

\- Le nœud `reception\_core` joue le rôle d’orchestrateur et coordonne l’ensemble du scénario \*Receptionist\*.



---



\## Schéma d’architecture ROS



```mermaid

flowchart LR

&nbsp; %% ===== NODES (scripts) =====

&nbsp; ASR((asr\_listen\\nlisten\_asr))

&nbsp; EMO((emotion\_perception\\nemotion\_detector))

&nbsp; IA((ia\_dialog\\nia\_node))

&nbsp; CORE((reception\_core\\nreception\_manager))

&nbsp; FSM((reception\_core\\nsession\_state))

&nbsp; NAME((user\_name\\nname\_node))

&nbsp; FACE((face\_detector\\nface\_detector))

&nbsp; MEM((memory\_manager\\nmemory\_node))

&nbsp; TTS((tts\_speak\\nspeak\_tts))



&nbsp; %% ===== TOPICS =====

&nbsp; T\_ASR\_EN\[/Topic\\n/asr/enable/]

&nbsp; T\_ASR\_OUT\[/Topic\\n/asr/text\_out/]



&nbsp; T\_CAM\[/Topic\\ncamera\_topic/]

&nbsp; T\_XTION\[/Topic\\n/xtion/rgb/image\_color/]

&nbsp; T\_FACE\[/Topic\\n/face\_event/]

&nbsp; T\_EMO\[/Topic\\n/emotion/state/]



&nbsp; T\_IA\_IN\[/Topic\\n/ia/input\_text/]

&nbsp; T\_IA\_UN\[/Topic\\n/ia/user\_name/]

&nbsp; T\_IA\_OUT\[/Topic\\n/ia/text\_out/]



&nbsp; T\_TTS\_TXT\[/Topic\\n/tts/text/]

&nbsp; T\_TTS\_SPK\[/Topic\\n/tts/is\_speaking/]



&nbsp; T\_NAME\_START\[/Topic\\n/name/start/]

&nbsp; T\_NAME\_DONE\[/Topic\\n/name/done/]



&nbsp; %% ===== FLOWS =====

&nbsp; CORE --> T\_ASR\_EN --> ASR

&nbsp; ASR --> T\_ASR\_OUT --> CORE



&nbsp; %% Camera / Face detection

&nbsp; T\_CAM --> FACE

&nbsp; FACE --> T\_FACE --> CORE

&nbsp; FACE --> T\_FACE --> IA



&nbsp; %% Emotion detection

&nbsp; T\_XTION --> EMO

&nbsp; EMO --> T\_EMO --> CORE

&nbsp; EMO --> T\_EMO --> IA



&nbsp; %% IA dialog

&nbsp; CORE --> T\_IA\_IN --> IA

&nbsp; IA --> T\_IA\_OUT --> CORE

&nbsp; IA --> T\_TTS\_TXT --> TTS



&nbsp; %% TTS feedback

&nbsp; TTS --> T\_TTS\_SPK --> CORE



&nbsp; %% User name acquisition

&nbsp; CORE --> T\_NAME\_START --> NAME

&nbsp; NAME --> T\_ASR\_EN --> ASR

&nbsp; ASR --> T\_ASR\_OUT --> NAME

&nbsp; NAME --> T\_IA\_UN --> CORE

&nbsp; NAME --> T\_NAME\_DONE --> CORE

&nbsp; NAME --> T\_TTS\_TXT --> TTS



&nbsp; %% State machine and memory

&nbsp; CORE <--> FSM

&nbsp; CORE <--> MEM



