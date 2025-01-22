# Erreurs connues

Lors de notre projet, nous avons identifié certains bugs et erreurs.  
Dans cette partie de la documentation, nous allons évoquer les erreurs et bugs trouvés et fournir une solution pour celles-ci (si possible).

## move_group unreachable

Parfois, lorsque vous tentez de faire bouger Tiago via MoveIt, sans raison apparente, vous pouvez obtenir des erreurs disant que le `move_group` est injoignable.

La cause de cette erreur n'a pas encore été identifiée.

La solution pour cette erreur n'a toujours pas été trouvée à ce jour.  
- Redémarrer le robot ne change rien.  
- Redémarrer le conteneur Docker ne change rien non plus.  
- Se brancher et se débrancher (Ethernet) ne change rien non plus.

La seule solution trouvée à ce jour : attendre ou changer d'ordinateur.

## "Home"

Lors de notre formation, nous avons appris que faire effectuer le mouvement "Home" à Tiago permet de le remettre dans son état initial.

Cette action ne doit pas être effectuée.

Elle a pour effet de faire bugger l'utilisation de MoveIt (surtout en utilisant la fonction 'go' d'un `move_group`).

Nous avons recodé une fonction "Home" qui ne pose pas de problème.

Il faut cependant savoir que cette fonction "Home" est plutôt rapide.  
Il faut donc uniquement l'appeler en faisant attention à l'environnement autour de Tiago : il faut qu'il ait de la place autour de lui. (Nous avons cassé certaines pinces imprimées en 3D avec cette action).

## Utilisation de la manette

### Reprendre le contrôle

Lors de notre formation, nous avons également appris qu'utiliser la manette (en appuyant sur start) permet de reprendre le contrôle pendant que le robot effectue une action (afin d'éviter un accident).

Il s'avère que cela ne fonctionne pas ! La seule manière de faire en sorte que Tiago arrête son mouvement est de tuer le terminal qui exécute l'action.

### Bouger le robot avec la manette

Lors du développement de fonctions pour le mouvement du robot, nous pensons qu'il faut éviter de bouger le robot avec la manette. Ce n'est pas certain, mais il semble que cela pose des problèmes avec le code.

Bouger la tête avec la manette a également tendance à faire crasher la tête.

## Tablette qui crash

La tablette a tendance à planter avec l'utilisation du 'CESI Publish Sound'. La tablette semble rencontrer moins ce problème lorsqu'elle est branchée.

L'apparition de ce problème est plutôt aléatoire, et nous ne savons pas vraiment comment l'éviter.

## Main qui va trop loin par rapport aux flyers

Lorsque Tiago va chercher les flyers, parfois il va trop loin et abîme les flyers.  
Si cela arrive, il faut refaire une calibration de ses bras (voir la documentation pour la calibration).

---

J'ai corrigé les fautes d'orthographe, les accords et les formulations pour améliorer la clarté.