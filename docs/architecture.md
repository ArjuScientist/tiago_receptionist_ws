```mermaid

flowchart LR



%% NODES (scripts)

ASR((asr\_listen | listen\_asr))

EMO((emotion\_perception | emotion\_detector))

IA((ia\_dialog | ia\_node))

CORE((reception\_core | reception\_manager))

FSM((reception\_core | session\_state))

NAME((user\_name | name\_node))

FACE((face\_detector | face\_detector))

MEM((memory\_manager | memory\_node))

TTS((tts\_speak | speak\_tts))



%% TOPICS

T\_ASR\_EN\[/asr/enable/]

T\_ASR\_OUT\[/asr/text\_out/]



T\_CAM\[/camera\_topic/]

T\_XTION\[/xtion/rgb/image\_color/]

T\_FACE\[/face\_event/]

T\_EMO\[/emotion/state/]



T\_IA\_IN\[/ia/input\_text/]

T\_IA\_UN\[/ia/user\_name/]

T\_IA\_OUT\[/ia/text\_out/]



T\_TTS\_TXT\[/tts/text/]

T\_TTS\_SPK\[/tts/is\_speaking/]



T\_NAME\_START\[/name/start/]

T\_NAME\_DONE\[/name/done/]



%% FLOWS

CORE --> T\_ASR\_EN --> ASR

ASR --> T\_ASR\_OUT --> CORE



T\_CAM --> FACE

FACE --> T\_FACE --> CORE

FACE --> T\_FACE --> IA



T\_XTION --> EMO

EMO --> T\_EMO --> CORE

EMO --> T\_EMO --> IA



CORE --> T\_IA\_IN --> IA

IA --> T\_IA\_OUT --> CORE

IA --> T\_TTS\_TXT --> TTS



TTS --> T\_TTS\_SPK --> CORE



CORE --> T\_NAME\_START --> NAME

NAME --> T\_ASR\_EN --> ASR

ASR --> T\_ASR\_OUT --> NAME

NAME --> T\_IA\_UN --> CORE

NAME --> T\_NAME\_DONE --> CORE

NAME --> T\_TTS\_TXT --> TTS



CORE <--> FSM

CORE <--> MEM



