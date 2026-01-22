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

