# ProjetDeRecherche: Commande d'un turtlebot à partir des données de la caméra RealSense D435

[IMT Lille Douai majeur OAPI] Projet de Recherche de 2nde année

## Table des matières
Introduction
1) Librairies, langages et logiciels
2) Détection d'images à partir de la librealsense2
3) Détection du visage grâce à OpenCV
4) Déplacement du robot TurtleBot2 
5) Déplacement du robot en fonction de la distance à un visage
6) Détection du visage humain avec OpenPose
7) Détection du squelette humain grâce à OpenPose

## Introduction 

L'objectif de ce projet de recherche est d'analyser le flux d'images délivré par une caméra RealSense D435 afin de détecter la présence d'un être humain dans son champ de vision. 

Selon le comportement de cet humain, le robot doit modifier son comportement et agir en fonction de la gestuelle de l'être humain face à lui. Il réalise alors des actions simples telles que du tracking, des mouvements longitudinaux ou rotationnels, etc.

## 1) Librairies, langages et logiciels

Afin de mener à bien le projet, nous avons travaillé sur une machine Ubuntu. La programmation futréalisée à partir de Python 2.7 et des différentes librairies python associés aux TurtleBot2 (rospy), à la caméra RealSense D435 (librealsense2), à la détection de visages sur des images (OpenCV) et à la détection de squelettes humains (pyopenpose). La majorité des commandes ROS furent incorporées dans le corps du programme python grace à rospy. Pour   lancer   le   programme   sur   le   robot,   nous   effectuons   les   commandes   suivantes. 

Dans un premier terminal, on lance le robot:
```
roslaunch turtlebot_bringup minimal.launch
```

Dans un second terminal, on lance le programme python de la manière suivante:
```
Python nomDuProgramme.py
```

## 2) Détection d'images à partir de la librealsense2

Avec   la   caméra   Intel   Realsense   Depth   camera   D435,   nous   avons   les   caractéristiques suivantes:
-Depth technology: Active IR Stereo
-Minimum Depth Distance (Min-Z): 0.105 m
-Depth Field of View (FOV): 87°+-3° x 58°+-1° x 95°+-3°
-Depth   Output   Resolution   &   frame   rate:   Up   to   1280X720   active   stereo   depth resolution. Up to 90 fps (frames per second).

La  récupération  des  données  de  la  caméra  RealSense  D435  dans  un  programme  python  est réalisée grâce à la librairie librealsense2. Elle permet notamment d’afficher les flux d’images couleur  et de  profondeur  grâce  à  un  stream  et  de  renvoyer  le  tableau  des  profondeurs associéesaux différentes images.

```python
%Configure depth and color streams
pipeline = rs.pipeline()
bonjour = True
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

%Start streaming
profile=pipeline.start(config)
```

--- Image du flux de la caméra ---

## 3) Détection du visage grâce à OpenCV

Le premier modèle implémenté fut basé sur une solution OpenCV de détection des visages pour obtenir l’information de présence d’un être humain.

A l’aide de la bibliothèque OpenCV, nous détectons les visages  des personnes devant la caméra  et  calculons  la  distance  entre  la  caméra  et  le  visage.  Lorsque  la  distance  est supérieure à la distance de consigne, le robot avance pour arriver à la distance de consigne. Si  la  distance  est  inférieure,  le  robot  recule.  On  souhaite  que  le  robot  effectue  un  tracking d’une personne humaine.

--- Image de Samah et Arthur avec visages détectés---

La  distance  du  visage  par  rapport  à  la  caméra  est  obtenue  en  regardant  la  valeur  de profondeur au centre du rectangle formé autour du visage détecté. 

```python
cv2.rectangle(img, (face[0], face[1]), (face[0] + face[2], face[1] + face[3]), (255, 0, 0), 3)
x1=face[0]
y1=face[1]
x2=face[0]+face[2]
y2=face[1]+face[3]
print(x1,y1,x2,y2)
print(depth_sensor.get_option(rs.option.depth_units)) %La profondeur est en mm
depth_crop = depth_image[face[0]:face[0]+face[2],face[1]:face[1]+face[3]]
if ((x1 & x2) < 640) &  ((y1 & y2) < 480):
	depth_centre=depth_image[face[1]+face[3]/2,face[0]+face[2]/2]
	cv2.circle(img,(face[0] + face[2]/2, face[1] + face[3]/2),10,(0,255,255),-1)
	print(depth_centre)
```
Ce modele a cependant des limites:
-Le visage doit etre de face par rapport a la camera (Les visages de profil sont mal détectés).
-La luminosité environnante doit être suffisante

Une seconde méthode que nous avons essayée fut de calculer la moyenne des profondeurs au sein du rectangle entourant le visage. On obtient une valeur de profondeur proche de la profondeur obtenue en prenant le centre du rectangle.

```python
%Calcul de la moyenne des profondeurs des pixels delimites par le rectangle:
somme=0
n=0
if ((x1 & x2) < 640) &  ((y1 & y2) < 480 ) :
	for i in range (x1,x2):
		for j in range (y1,y2):
			somme=somme+depth_image[j,i]
			n=n+1
	moyenne=somme/n
	print(moyenne)
```

Les limites de cette méthode sont le temps de calcul et une erreur importante lorsque le rectangle n’encadre pas précisément le visage. Nous avons choisi de partir sur la méthodede prise de la profondeur au centre du rectangle encadrant le visage, cette méthode s’étant révéléla plus fiable.Le +:Pour avoir une meilleure compréhension du tableau des profondeurs, nous l’avons écritdans un fichier Excelpour récupérer l’ensembledes données de l’image:




