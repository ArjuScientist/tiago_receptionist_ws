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
  %% ===== NODES (scripts) =====
  ASR((asr_listen\nlisten_asr))
  EMO((emotion_perception\nemotion_detector))
  IA((ia_dialog\nia_node))
  CORE((reception_core\nreception_manager))
  FSM((reception_core\nsession_state))
  NAME((user_name\nname_node))
  FACE((face_detector\nface_detector))
  MEM((memory_manager\nmemory_node))
  TTS((tts_speak\nspeak_tts))

  %% ===== TOPICS =====
  T_ASR_EN[/Topic\n/asr/enable/]
  T_ASR_OUT[/Topic\n/asr/text_out/]

  T_CAM[/Topic\ncamera_topic/]
  T_XTION[/Topic\n/xtion/rgb/image_color/]
  T_FACE[/Topic\n/face_event/]
  T_EMO[/Topic\n/emotion/state/]

  T_IA_IN[/Topic\n/ia/input_text/]
  T_IA_UN[/Topic\n/ia/user_name/]
  T_IA_OUT[/Topic\n/ia/text_out/]

  T_TTS_TXT[/Topic\n/tts/text/]
  T_TTS_SPK[/Topic\n/tts/is_speaking/]

  T_NAME_START[/Topic\n/name/start/]
  T_NAME_DONE[/Topic\n/name/done/]

  %% ===== FLOWS (from your table) =====
  CORE --> T_ASR_EN --> ASR
  ASR --> T_ASR_OUT --> CORE

  %% Camera / Face
  T_CAM --> FACE
  FACE --> T_FACE --> CORE
  FACE --> T_FACE --> IA

  %% Emotion pipeline
  T_XTION --> EMO
  EMO --> T_EMO --> CORE
  EMO --> T_EMO --> IA

  %% IA dialog
  CORE --> T_IA_IN --> IA
  IA --> T_IA_OUT --> CORE
  IA --> T_TTS_TXT --> TTS

  %% TTS speaking feedback
  TTS --> T_TTS_SPK --> CORE

  %% Name flow
  CORE --> T_NAME_START --> NAME
  NAME --> T_ASR_EN --> ASR
  ASR --> T_ASR_OUT --> NAME
  NAME --> T_IA_UN --> CORE
  NAME --> T_NAME_DONE --> CORE
  NAME --> T_TTS_TXT --> TTS

  %% FSM + Memory (no topics provided -> logical links)
  CORE <--> FSM
  CORE <--> MEM
