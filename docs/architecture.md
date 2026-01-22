flowchart LR

%% ===== NODES (blue) =====
ASR((asr_listen<br/>listen_asr))
EMO((emotion_perception<br/>emotion_detector))
IA((ia_dialog<br/>ia_node))
CORE((reception_core<br/>reception_manager))
FSM((reception_core<br/>session_state))
NAME((user_name<br/>name_node))
FACE((face_detector<br/>face_detector))
MEM((memory_manager<br/>memory_node))
TTS((tts_speak<br/>speak_tts))

%% ===== TOPICS (red) =====
T_ASR_EN[/asr/enable/]
T_ASR_OUT[/asr/text_out/]
T_CAM[/camera_topic/]
T_XTION[/xtion/rgb/image_color/]
T_FACE[/face_event/]
T_EMO[/emotion/state/]
T_IA_IN[/ia/input_text/]
T_IA_UN[/ia/user_name/]
T_IA_OUT[/ia/text_out/]
T_TTS_TXT[/tts/text/]
T_TTS_SPK[/tts/is_speaking/]
T_NAME_START[/name/start/]
T_NAME_DONE[/name/done/]

%% ===== FLOWS =====
CORE --> T_ASR_EN --> ASR
ASR --> T_ASR_OUT --> CORE

T_CAM --> FACE
FACE --> T_FACE --> CORE
FACE --> T_FACE --> IA

T_XTION --> EMO
EMO --> T_EMO --> CORE
EMO --> T_EMO --> IA

CORE --> T_IA_IN --> IA
IA --> T_IA_OUT --> CORE
IA --> T_TTS_TXT --> TTS

TTS --> T_TTS_SPK --> CORE

CORE --> T_NAME_START --> NAME
NAME --> T_ASR_EN --> ASR
ASR --> T_ASR_OUT --> NAME
NAME --> T_IA_UN --> CORE
NAME --> T_NAME_DONE --> CORE
NAME --> T_TTS_TXT --> TTS

CORE <--> FSM
CORE <--> MEM

%% ===== STYLES =====
classDef node fill:#1f77b4,color:#ffffff,stroke:#0b3c5d,stroke-width:2px;
classDef topic fill:#d62728,color:#ffffff,stroke:#7f0000,stroke-width:2px;

%% Apply styles
class ASR,EMO,IA,CORE,FSM,NAME,FACE,MEM,TTS node;
class T_ASR_EN,T_ASR_OUT,T_CAM,T_XTION,T_FACE,T_EMO,T_IA_IN,T_IA_UN,T_IA_OUT,T_TTS_TXT,T_TTS_SPK,T_NAME_START,T_NAME_DONE topic;
