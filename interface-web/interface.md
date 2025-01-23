Documentation de l'Application Tiago

## Table des Matières
- [Introduction](#introduction)
- [Fonctions Principales](#fonctions-principales)
  - [initializeApp](#initializeapp)
  - [importRoslib](#importroslib)
  - [importstyles](#importstyles)
  - [connectToRos](#connecttoros)
  - [subscribeToTopics](#subscribetotopics)
  - [loadAnswersAndDisplay](#loadanswersanddisplay)
  - [callAnswerService](#callanswerservice)
  - [updateStatusDisplay](#updatestatusdisplay)
  - [updateListenerDisplay](#updatelistenerdisplay)
  - [handlePartialMessage](#handlepartialmessage)
  - [handleFullMessage](#handlefullmessage)
  - [onEndspeak](#onendspeak)
  - [startup](#startup)
- [Typage des Variables](#typage-des-variables)
- [Structures des Objets](#structures-des-objets)

## Introduction
Cette application permet d'interagir avec le robot Tiago via ROS. Elle gère la transcription en temps réel et les réponses aux questions posées par le robot.

## Structures des Objets
Voici les structures des objets utilisés dans le code :

### `elements`
Un objet contenant des références aux éléments HTML de l'interface utilisateur.

```javascript
const elements = {
    transcriptionDiv: HTMLDivElement, // Div pour afficher la transcription
    answersDiv: HTMLDivElement, // Div pour afficher les réponses
    statusDiv: HTMLDivElement, // Div pour afficher le statut
    reloadBtn: HTMLButtonElement, // Bouton pour recharger la connexion
    statusCircle: HTMLDivElement, // Indicateur de statut
    toggleMic: HTMLButtonElement, // Bouton pour activer/désactiver le microphone
    settingsBtn: HTMLButtonElement, // Bouton pour afficher les paramètres
    optionsDiv: HTMLDivElement, // Div pour les options
    loadingAnimator: HTMLDivElement, // Indicateur de chargement
    listenerMsg: HTMLDivElement, // Div pour afficher le message d'écoute
    listenTrue: HTMLDivElement, // Indicateur d'écoute active
    listenFalse: HTMLDivElement, // Indicateur d'écoute inactive
    questionData: HTMLDivElement, // Div pour afficher les questions
};
```

## Fonctions Principales

### `initializeApp()`
Initialise l'application, charge les scripts nécessaires et établit la connexion avec ROS.

```javascript
function startup() {
  updateStatusDisplay("Chargement des fichiers...", false);
  Promise.all([importRoslib(), importstyles()])
  .then(() => {
    connectToRos();
  })
  .catch((error) => {
    console.error("Erreur lors du chargement des fichiers :", error);
    updateStatusDisplay("Erreur de chargement", false);
  });
}
```



### `importRoslib()`
Charge la bibliothèque Roslib.js pour permettre la communication avec ROS.

**Retourne** : `Promise` qui se résout lorsque le script est chargé avec succès.

```javascript
function importRoslib() {
  return new Promise((resolve, reject) => {
    const scriptUrl = `http://${hostname}:8000/roslib`;
    const script = document.createElement("script");
    script.src = scriptUrl;
    script.type = "text/javascript";

    script.onload = () => {
      console.log("Roslib chargé avec succès !");
      resolve(true);
    };

    script.onerror = () => {
      console.error("Erreur lors du chargement de Roslib.");
      reject(false);
    };

    document.head.appendChild(script);
  });
}
```

### `importstyles()`
Charge les styles CSS nécessaires pour l'application.

**Retourne** : `Promise` qui se résout lorsque le style est chargé avec succès.

```javascript
function importstyles() {
  return new Promise((resolve, reject) => {
    const scriptUrl = `http://${hostname}:8000/styles`;
    const link = document.createElement("link");
    link.href = scriptUrl;
    link.rel = "stylesheet";
    link.type = "text/css";

    link.onload = () => {
      console.log("Styles chargés avec succès !");
      resolve(true);
    };

    link.onerror = () => {
      console.error("Erreur lors du chargement des styles.");
      reject(false);
    };

    document.head.appendChild(link);
  });
}
```

### `connectToRos()`
Établit la connexion avec le serveur ROS et gère les événements de connexion, d'erreur et de fermeture.

```javascript
const connectToRos = () => {
  ros = new ROSLIB.Ros({ url: `ws://${hostname}:9090` });
  updateStatusDisplay("Connexion en cours...", false);

  ros.on("connection", () => {
    console.log("Connexion ROS réussie !");
    updateReloadButtonVisibility(true);
    updateStatusDisplay("Connecté", true);
    reconnectionAttempts = 0;
  });

  ros.on("error", () => {
    console.error("Erreur de connexion ROS.");
    updateReloadButtonVisibility(false);
    updateStatusDisplay("Erreur de connexion", false);
    attemptReconnection();
  });

  ros.on("close", () => {
    console.warn("Connexion ROS fermée.");
    updateReloadButtonVisibility(false);
    updateStatusDisplay("Déconnecté...", false);
    attemptReconnection();
  });

  // Subscribe to topics
  subscribeToTopics();
};
```

### `subscribeToTopics()`
S'abonne aux différents topics ROS pour recevoir des messages, y compris les réponses aux questions.

```javascript
const subscribeToTopics = () => {
  const topics = {
    stt_partial: new ROSLIB.Topic({ ros, name: "/stt/partial", messageType: "std_msgs/String" }),
    stt_full: new ROSLIB.Topic({ ros, name: "/stt/full", messageType: "std_msgs/String" }),
    tiago_interact_stt_status: new ROSLIB.Topic({ ros, name: "/tiago_interact/stt/status", messageType: "std_msgs/Bool" }),
    tiago_asker_question_possible_answers: new ROSLIB.Topic({ ros, name: "/tiago_asker/question/possible_answers", messageType: "std_msgs/String" }),
    tts_goal: new ROSLIB.Topic({ ros, name: "/tts/goal", messageType: "pal_interaction_msgs/TtsActionGoal" }),
    tts_result: new ROSLIB.Topic({ ros, name: "/tts/result", messageType: "pal_interaction_msgs/TtsActionResult" }),
  };

  topics.tiago_interact_stt_status.subscribe((message) => {
    isSttEnabled = message.data;
    elements.transcriptionDiv.style.display = isSttEnabled ? "block" : "none";
    elements.answersDiv.style.display = isSttEnabled ? "none" : "block";
    updateListenerDisplay(isSttEnabled);
  });

  topics.stt_partial.subscribe(handlePartialMessage);
  topics.stt_full.subscribe(handleFullMessage);
  topics.tiago_asker_question_possible_answers.subscribe(loadAnswersAndDisplay);
  topics.tts_goal.subscribe(loadAskTiagoRealTime);
  topics.tts_result.subscribe(onEndspeak);
};
```

### `loadAnswersAndDisplay(answers: string)`
Affiche les réponses possibles sous forme de boutons dynamiques.

**Paramètres**:
- `answers` : Une chaîne contenant les réponses possibles, séparées par des virgules.

```javascript
const loadAnswersAndDisplay = (answers) => {
    elements.answersDiv.innerHTML = ""; // Vider le conteneur des réponses
    if (answers && answers.data) {
        const answerList = answers.data.split(",");
        answerList.forEach((answer) => {
            const answerElement = document.createElement("button");
            answerElement.className = "answer-element";
            answerElement.textContent = answer.trim();
            answerElement.addEventListener("click", () => {
                callAnswerService(answer.trim()); // Appeler le service avec la réponse
            });
            elements.answersDiv.appendChild(answerElement);
        });
    } else {
        elements.answersDiv.textContent = "Aucune réponse disponible.";
    }
};
```

### `callAnswerService(answer: string)`
Appelle le service ROS correspondant à la réponse sélectionnée.

**Paramètres**:
- `answer` : La réponse sélectionnée par l'utilisateur.

```javascript
const callAnswerService = (answer) => {
  const answerService = new ROSLIB.Service({
      ros: ros,
      name: `/your_service_name/${answer}`, // Remplacez par le nom de votre service
      serviceType: "std_srvs/Empty", // Remplacez par le type de votre service
  });

  answerService.callService(new ROSLIB.ServiceRequest(), (result) => {
      console.log(`Service '${answer}' appelé avec succès`, result);
  }, (error) => {
      console.error(`Erreur lors de l'appel du service '${answer}'`, error);
  });
};
```

### `updateStatusDisplay(status: string, isConnected: boolean)`
Met à jour l'affichage de l'état de connexion dans l'interface utilisateur.

**Paramètres**:
- `status` : Le message d'état à afficher.
- `isConnected` : Booléen indiquant si la connexion est active.

```javascript
const attemptReconnection = () => {
  if (reconnectionAttempts >= maxReconnectionAttempts) {
    updateStatusDisplay("Échec de reconnexion après plusieurs tentatives.", false);
    stopConnectionCheck();
    return;
  }

  reconnectionAttempts++;
  console.log(`Tentative de reconnexion : ${reconnectionAttempts} / ${maxReconnectionAttempts}`);
  setTimeout(connectToRos, reconnectionDelay);
};
```

### `updateListenerDisplay(isListening: boolean)`
Met à jour l'affichage de l'état d'écoute du robot.

**Paramètres**:
- `isListening` : Booléen indiquant si le robot écoute ou non.

```javascript
const updateListenerDisplay = (isListening) => {
  console.log("isListening : " + isListening);
  if (typeof isListening !== "boolean") {
    console.error("Erreur : La donnée passée à 'updateListenerDisplay' n'est pas un booléen.");
    elements.listenerMsg.textContent = "Erreur : État inconnu.";
    elements.listenTrue.style.display = "none";
    elements.listenFalse.style.display = "none";
    return;
  }
  elements.listenerMsg.textContent = isListening ? "Je t'écoute :D" : "Je ne t'écoute pas ;)";
  elements.listenTrue.style.display = isListening ? "grid" : "none";
  elements.listenFalse.style.display = isListening ? "none" : "grid";
};
```

### `handlePartialMessage(message: object)`
Gère les messages partiels reçus du service de reconnaissance vocale.

**Paramètres**:
- `message` : Un objet contenant le message partiel reçu.

```javascript
const handlePartialMessage = (message) => {
  if (!isSttEnabled) return;
  if (!partialSpan) {
    partialSpan = document.createElement("span");
    partialSpan.className = "partial";
    elements.transcriptionDiv.appendChild(partialSpan);
  }
  partialSpan.textContent = message.data;
};
```

### `handleFullMessage(message: object)`
Gère les messages complets reçus du service de reconnaissance vocale.

**Paramètres**:
- `message` : Un objet contenant le message complet reçu.

```javascript
const handleFullMessage = (message) => {
  if (!isSttEnabled) return;
  if (partialSpan) {
    partialSpan.remove();
    partialSpan = null;
  }
  if (currentFinalSpan) {
    if (previousFinalSpan) previousFinalSpan.remove();
    previousFinalSpan = currentFinalSpan;
    previousFinalSpan.className = "partial";
  }

  currentFinalSpan = document.createElement("span");
  currentFinalSpan.className = "final";
  currentFinalSpan.textContent = message.data;
  elements.transcriptionDiv.appendChild(currentFinalSpan);
};
```

### `Event Listeners`
Tous les éléments qui sont sur des évènement.

```javascript
document.addEventListener("DOMContentLoaded", () => {
  elements.reloadBtn.addEventListener("click", connectToRos);
  elements.toggleMic.addEventListener("click", () => {
    elements.toggleMic.disabled = true;
    setTimeout(() => (elements.toggleMic.disabled = false), 3000);
    toggleMicValue = !toggleMicValue;
    console.log("toggleMicValue : " + toggleMicValue);
    elements.toggleMic.style.backgroundColor = toggleMicValue ? "lime" : "red";
    elements.toggleMic.style.color = toggleMicValue ? "black" : "white";

    elements.retranscriptionDiv.style.display = toggleMicValue ? "block" : "none";

    const micService = new ROSLIB.Service({
      ros: ros,
      name: "/tiago_interact/stt/toggle",
      serviceType: "std_srvs/Empty",
    });

    micService.callService(
      new ROSLIB.ServiceRequest(), () => console.log("Service appelé avec succès")
    );
  });

  elements.settingsBtn.addEventListener("click", () => {
    elements.optionsDiv.style.display = elements.optionsDiv.style.display === "none" ? "block" : "none";
  });
});
```

### `onEndspeak()`
Gère l'événement lorsque le robot a terminé de parler, en retirant le message affiché.

```javascript
```

### `startup()`
Démarre l'application en chargeant les fichiers nécessaires et en établissant la connexion avec ROS.

```javascript
```

## Typage des Variables
Voici un aperçu des types de variables utilisés dans le code :

- `const hostname: string` : Le nom d'hôte de la fenêtre actuelle.
- `let ros: ROSLIB.Ros` : Instance de la classe ROSLIB.Ros pour la connexion à ROS.
- `let toggleMicValue: boolean` : Indique si le microphone est activé ou non.
- `let partialSpan: HTMLSpanElement | null` : Élément HTML pour afficher le texte partiel.
- `let previousFinalSpan: HTMLSpanElement | null` : Élément HTML pour afficher le texte final précédent.
- `let currentFinalSpan: HTMLSpanElement | null` : Élément HTML pour afficher le texte final actuel.
- `let isSttEnabled: boolean` : Indique si la reconnaissance vocale est activée.
- `let isInteractStatus: boolean` : Indique si l'interaction est active.
- `let tiagoSay: HTMLSpanElement | null` : Élément HTML pour afficher le texte que Tiago dit.
- `let reconnectionAttempts: number` : Compteur pour les tentatives de reconnexion.
- `const maxReconnectionAttempts: number` : Nombre maximum de tentatives de reconnexion.
- `const reconnectionDelay: number` : Délai entre les tentatives de reconnexion.
- `let connectionCheckInterval: number` : Intervalle pour vérifier la connexion.

