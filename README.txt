PROJET TIAGO – ROBOT RECEPTIONNISTE ROS

Ce dépôt contient le workspace complet tiago_ws du projet
de robot réceptionniste basé sur le robot Tiago (PAL Robotics).

Le projet repose sur ROS Noetic et une architecture modulaire
de nœuds ROS pour la perception, l’interaction vocale
et le dialogue intelligent.

------------------------------------------------------------
RECUPERATION DU PROJET
------------------------------------------------------------

Deux possibilités :

1) Clonage via GitHub :
git clone https://github.com/ArjuScientist/tiago_receptionist_ws

2) Téléchargement de l’archive ZIP depuis GitHub
puis extraction sur le bureau du robot Tiago.

Le dossier tiago_ws doit se trouver par exemple ici :
~/Desktop/tiago_ws

------------------------------------------------------------
STRUCTURE DU WORKSPACE
------------------------------------------------------------

tiago_ws/
├── src/
│   ├── asr_listen/
│   ├── emotion_perception/
│   ├── face_detector/
│   ├── ia_dialog/
│   ├── memory_manager/
│   ├── reception_core/
│   ├── tts_speak/
│   └── user_name/
├── build/
├── devel/
├── env_S3E_LAK/
└── README.txt

Le workspace est déjà structuré.
Aucune création manuelle de catkin workspace n’est nécessaire.

------------------------------------------------------------
PREREQUIS
------------------------------------------------------------

- Ubuntu 20.04
- ROS Noetic
- Python 3
- Accès réseau au robot Tiago

Connexion au robot Tiago (être sur le même réseau) :
ssh pal@10.68.0.1
Mot de passe : pal

------------------------------------------------------------
ENVIRONNEMENT PYTHON
------------------------------------------------------------

Le projet utilise un environnement Python virtuel local
placé dans le workspace.

Création de l’environnement (si non présent) :

cd ~/Desktop/tiago_ws
python3 -m venv env_S3E_LAK

Activation de l’environnement :

source ~/Desktop/tiago_ws/env_S3E_LAK/bin/activate

------------------------------------------------------------
COMPILATION DU WORKSPACE
------------------------------------------------------------

Se placer dans le workspace :

cd ~/Desktop/tiago_ws

Compiler :

catkin_make

Sourcer l’environnement ROS :

source devel/setup.bash

Cette commande doit être exécutée dans chaque nouveau terminal.

------------------------------------------------------------
LANCEMENT DU PROJET
------------------------------------------------------------

Le lancement principal se fait via le package reception_core.

Exemple :

roslaunch reception_core tiago_reception.launch

Ce lancement initialise :
- la détection de visage
- la reconnaissance vocale
- la gestion du dialogue
- la synthèse vocale
- le gestionnaire d’état central

------------------------------------------------------------
ARCHITECTURE LOGICIELLE
------------------------------------------------------------

Le nœud central reception_core agit comme orchestrateur :
- il reçoit les événements des autres nœuds
- il décide des actions à effectuer
- il pilote le comportement global du robot

Tous les échanges passent par des topics ROS
afin de garantir modularité et évolutivité.

------------------------------------------------------------
TIPS ET COMMANDES UTILES
------------------------------------------------------------

Créer un package ROS :

cd ~/Desktop/tiago_ws/src
catkin_create_pkg <nom_du_package>

Recompiler après modification :

cd ~/Desktop/tiago_ws
catkin_make
source devel/setup.bash

Créer un script Python dans un package :

cd <nom_du_package>
vim nom_du_fichier.py

Rendre un script exécutable :

chmod +x nom_du_fichier.py

Lancer un script :

rosrun <nom_du_package> <nom_du_fichier.py>

Modifier un fichier avec vim :
- aller au début : gg
- supprimer tout : dG
- sauvegarder et quitter : :wq

Supprimer un package :

rm -r <nom_du_package>

Supprimer un fichier :

rm <nom_du_fichier>

------------------------------------------------------------
OUTILS DE DEBUG ROS
------------------------------------------------------------

Lister les topics :

rostopic list

Afficher les messages d’un topic :

rostopic echo /nom_du_topic

Lister les nœuds :

rosnode list

------------------------------------------------------------
NOTES IMPORTANTES
------------------------------------------------------------

- Toujours vérifier que l’environnement Python est activé
  avant de lancer les scripts.
- Toujours sourcer devel/setup.bash après un catkin_make.
- Les dossiers build/ et devel/ peuvent être supprimés
  et régénérés sans perte de code.

------------------------------------------------------------
AUTEURS
------------------------------------------------------------

Projet développé dans le cadre d’un projet robotique
sur le robot Tiago.
