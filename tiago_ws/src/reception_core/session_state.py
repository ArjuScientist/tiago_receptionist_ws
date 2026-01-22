#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SessionState
-------------
Ce module définit l'ensemble des états utilisés par le robot dans la
séquence d'accueil. Il est centralisé ici afin d'être importé facilement
dans tous les scripts du package reception_core.

Les états décrivent :
 - l'attente d'une personne
 - la salutation
 - la demande du nom
 - l'initialisation de l'IA
 - la conversation
 - les erreurs (appel humain)
 - la fin de session
"""

from enum import Enum

class SessionState(Enum):
    """État global du robot dans le scénario d'accueil"""

    # 🔵 Phase initiale
    INIT = 0                 # Initialisation du système, chargement modules

    # 👤 Détection & accueil
    WAIT_FOR_PERSON = 1      # Le robot attend qu'un visage soit détecté
    GREET = 2                # Le robot salue la personne

    # 📛 Identification utilisateur
    ASK_NAME = 3             # Le robot demande le nom
    VALIDATE_NAME = 4        # Option : validation / reformulation du nom

    # 🧠 IA / Contexte
    LAUNCH_IA = 5            # Initialisation / création du contexte IA

    # 💬 Interaction principale
    DIALOG = 6               # Dialogue continu IA ↔ utilisateur

    # 🔥 Gestion des problèmes
    CALL_HOST = 7            # Si incompréhensions ou échec ASR : appel humain

    # 🏁 Fin de session
    END_SESSION = 8          # Au revoir, reset variables puis retour en attente

    # 💤 État optionnel
    SLEEP = 9                # Mode veille longue (non utilisé pour l'instant)
