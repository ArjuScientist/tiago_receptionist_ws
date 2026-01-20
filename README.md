# tiago_receptionist_ws
FISA A5 S3E Promo 2023-2026
Lili, Kylian and Arjuna

## Modules Overview

| Package              | Script               | Function                                                                 | Subscribed Topics                                  | Published Topics            |
|----------------------|----------------------|--------------------------------------------------------------------------|---------------------------------------------------|-----------------------------|
| `asr_listen`         | `listen_asr`         | Speech recognition, retrieves user utterances                            | `/asr/enable`                                     | `/asr/text_out`             |
| `emotion_perception` | `emotion_detector`   | Detects user emotions from face image                                    | `/xtion/rgb/image_color`                          | `/emotion/state`            |
| `ia_dialog`          | `ia_node`             | AI dialogue model and intent processing                                  | `/emotion/state`<br>`/ia/user_name`<br>`/face_event` | `/ia/text_out`<br>`/tts/text` |
| `reception_core`     | `reception_manager`  | Main orchestrator, manages receptionist interaction flow                 | `/emotion/state`<br>`/asr/text_out`<br>`/ia/text_out` | `/asr/enable`<br>`/ia/input_text`<br>`/ia/user_name` |
| `reception_core`     | `session_state`      | Manages interaction and dialogue states                                  | —                                                 | —                           |
| `user_name`          | `name_node`          | Retrieves and confirms the user's name                                   | `/name/start`<br>`/asr/text_out`                  | `/tts/text`<br>`/asr/enable`<br>`/ia/user_name`<br>`/name/done` |
| `face_detector`      | `face_detector`      | Detects the presence of a face using the robot camera                    | `camera_topic`                                   | `/face_event`               |
| `memory_manager`     | `memory_node`        | Stores interaction history and user data                                 | —                                                 | —                           |
| `tts_speak`          | `speak_tts`          | Handles robot speech output                                              | `/tts/text`                                      | `/tts/is_speaking`          |

flowchart LR
  %% Nodes as big circles
  CORE((NODE\nreception_manager)):::node
  IA((NODE\nia_node)):::node
  ASR((NODE\nlisten_asr)):::node
  EMO((NODE\nemotion_detector)):::node
  FACE((NODE\nface_detector)):::node
  NAME((NODE\nname_node)):::node
  TTS((NODE\nspeak_tts)):::node
  MEM((NODE\nmemory_node)):::node

  %% Topics as rectangles
  AEN[Topic\n/asr/enable]:::topic
  AOUT[Topic\n/asr/text_out]:::topic
  XTION[Topic\n/xtion/rgb/image_color]:::topic
  EMO_T[Topic\n/emotion/state]:::topic
  FACE_T[Topic\n/face_event]:::topic
  IA_IN[Topic\n/ia/input_text]:::topic
  IA_UN[Topic\n/ia/user_name]:::topic
  IA_OUT[Topic\n/ia/text_out]:::topic
  TTS_T[Topic\n/tts/text]:::topic
  SPK[Topic\n/tts/is_speaking]:::topic
  NST[Topic\n/name/start]:::topic
  NDN[Topic\n/name/done]:::topic
  CAM[Topic\ncamera_topic]:::topic

  %% Flows
  CORE --> AEN --> ASR
  ASR --> AOUT --> CORE

  CAM --> FACE
  FACE --> FACE_T --> CORE
  FACE --> FACE_T --> IA

  XTION --> EMO
  EMO --> EMO_T --> CORE
  EMO --> EMO_T --> IA

  CORE --> IA_IN --> IA
  IA --> IA_OUT --> CORE

  IA --> TTS_T --> TTS
  TTS --> SPK --> CORE

  CORE --> NST --> NAME
  NAME --> AEN --> ASR
  ASR --> AOUT --> NAME
  NAME --> IA_UN --> CORE
  NAME --> NDN --> CORE
  NAME --> TTS_T --> TTS

  CORE <--> MEM

  classDef node fill:#2b6cb0,stroke:#1a365d,color:#fff,stroke-width:2px;
  classDef topic fill:#cbd5e0,stroke:#4a5568,color:#111,stroke-width:1px;

