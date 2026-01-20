flowchart LR

%% NODES (scripts)
ASR((asr_listen<br/>listen_asr))
EMO((emotion_perception<br/>emotion_detector))
IA((ia_dialog<br/>ia_node))
CORE((reception_core<br/>reception_manager))
FSM((reception_core<br/>session_state))
NAME((user_name<br/>name_node))
FACE((face_detector<br/>face_detector))
MEM((memory_manager<br/>memory_node))
TTS((tts_speak<br/>speak_tts))

%% TOPICS
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

%% FLOWS
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
NAME --> T_NAME_DONE --> COR
