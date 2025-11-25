# tiago_receptionist_ws
FISA A5 S3E Promo 2023-2026
Lili, Kylian and Arjuna

## Modules Overview

| Package             | Script               | Function                                             | Subscribed Topics          | Published Topics      |
|--------------------|----------------------|-------------------------------------------------------|----------------------------|------------------------|
| reception_core     | reception_manager    | Main orchestrator, manages the full receptionist flow | /face_event, /emotion/state, /asr/text_out, /tts/is_speaking, /ia/output | /asr/enable, /tts/text, /ia/input |
|                    | session_state        | Handles the dialogue and interaction state machine    |                            |                        |
| face_perception    | face_detector        | Face detection using TIAGo camera                     | —                          | /face_event           |
| emotion_perception | emotion_detector     | Emotion detection from face                           | —                          | /emotion/state        |
| asr_listen         | listen_asr           | Speech recognition (ASR)                              | /asr/enable                | /asr/text_out         |
| tts_speak          | speak_tts            | Text-to-speech, robot vocal output                    | /tts/text                  | /tts/is_speaking      |
| ia_dialog          | ia_node              | AI model that generates answers                       | /ia/input                  | /ia/output            |
| user_name          | name_node            | Extract the user's name                               | /asr/text_out              | /user/name (optional) |
| memory_manager     | memory_node          | Stores user data, encounter history, memory           | /user/name, /emotion/state, /face_event | /memory/state (optional) |
