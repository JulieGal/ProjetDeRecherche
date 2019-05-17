# Projet de Recherche: Commande d'un turtlebot à partir des données de la caméra RealSense D435

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

Afin de mener à bien le projet, nous avons travaillé sur une machine Ubuntu. La programmation fut réalisée à partir de Python 2.7 et des différentes librairies python associés aux TurtleBot2 (rospy), à la caméra RealSense D435 (librealsense2), à la détection de visages sur des images (OpenCV) et à la détection de squelettes humains (pyopenpose). La majorité des commandes ROS furent incorporées dans le corps du programme python grâce à rospy. Pour   lancer   le   programme   sur   le   robot,   nous   effectuons   les   commandes   suivantes. 

Dans un premier terminal, on lance le robot:
```
roslaunch turtlebot_bringup minimal.launch
```

Dans un second terminal, on lance le programme python de la manière suivante:
```
Python nomDuProgramme.py
```

## 2) Détection d'images à partir de la librealsense2

Avec   la   caméra   Intel   Realsense   Depth   camera   D435, nous avons les caractéristiques suivantes:
-Depth technology: Active IR Stereo
-Minimum Depth Distance (Min-Z): 0.105 m
-Depth Field of View (FOV): 87°+-3° x 58°+-1° x 95°+-3°
-Depth Output Resolution & frame rate: Up to 1280x720 active stereo depth resolution. Up to 90 fps (frames per second).

La  récupération  des  données  de  la  caméra  RealSense  D435  dans  un  programme  python  est réalisée grâce à la librairie librealsense2. Elle permet notamment d’afficher les flux d’images couleur  et de  profondeur  grâce  à  un  stream  et  de  renvoyer  le  tableau des profondeurs associées aux différentes images.

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

![Optional Text](../JulieGal/ProjetDeRecherche/Assets/Rapport_stream_cam (1).png)

## 3) Détection du visage grâce à OpenCV

Le premier modèle implémenté fut basé sur une solution OpenCV de détection des visages pour obtenir l’information de présence d’un être humain.

A l’aide de la bibliothèque OpenCV, nous détectons les visages  des personnes devant la caméra  et  calculons  la  distance  entre  la  caméra  et  le  visage.  Lorsque  la  distance  est supérieure à la distance de consigne, le robot avance pour arriver à la distance de consigne. Si  la  distance  est  inférieure,  le  robot  recule.  On  souhaite  que  le  robot  effectue  un  tracking d’une personne humaine.

--- Image de Samah et Arthur avec visages détectés---

### Méthode 1:

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
Ce modèle a cependant des limites:
-Le visage doit être de face par rapport a la caméra (Les visages de profil sont mal détectés).
-La luminosité environnante doit être suffisante.

### Méthode 2:

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

Les limites de cette méthode sont le temps de calcul et une erreur importante lorsque le rectangle n’encadre pas précisément le visage. Nous avons choisi de partir sur la méthode de prise de la profondeur au centre du rectangle encadrant le visage, cette méthode s’étant révélé la plus fiable.

Le +:
Pour avoir une meilleure compréhension du tableau des profondeurs, nous l’avons écrit dans un fichier Excel pour récupérer l’ensemble des données de l’image:

``` python
workbook = xlsxwriter.Workbook('arrays.xlsx')    
worksheet = workbook.add_worksheet()    
row = 0    
for col, data in enumerate(depth_image):    
	worksheet.write_column(row, col, data)    
workbook.close()
```

## 4) Déplacement du robot TurtleBot2 

Dans un premier temps, nous avons défini la fonction « GoForward » qui ordonne au robot de bouger suivant un axe (ici on a choisi l’axe des x) et avec un certain angle. On définit la variable vitesse qui permet de contrôler la vitesse du robot. Après, la fonction move_cmd nous permet de faire avancer le robot.

``` python
class GoForward():
	def __init__(self):
        # initiliaze
		rospy.init_node('GoForward', anonymous=False)
		# tell user how to stop TurtleBot
		rospy.loginfo("To stop TurtleBot CTRL + C")
		# What function to call when you ctrl + c    
		rospy.on_shutdown(self.shutdown)
			
		# Create a publisher which can "talk" to TurtleBot and tell it to move
		# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		 
		#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
		r = rospy.Rate(10);

		# Twist is a datatype for velocity
		move_cmd = Twist()
		move_cmd_rec=Twist()
		# let's go forward at 0.2 m/s
		vitesse = 0.1 #CE PARAMETRE EST MODIFIABLE
		move_cmd.linear.x = vitesse
		move_cmd_rec.linear.x = -vitesse
		# let's turn at 0 radians/s
		move_cmd.angular.z = 0
```
On définit la variable distance qui prend la valeur de la distance entre le robot et le visage, distance_voulue qui correspond à la distance à laquelle on souhaite que le robot s’arrête par rapport à la personne et la difdist qui représente la différence entre ces deux distances.

``` python
distance = depth_center*0.001

distance_voulue = 0.8

difdist=distance_voulue - distance
```

## 5) Déplacement du robot en fonction de la distance à un visage

--- Schema distance ---

Nous avons mis la condition sur la différence de distance pour que le robot puisse avancer ou reculer en fonction du signe. Si la distance entre la position du robot et la position qu’on souhaite est positive c’est que le robot doit avancer vers la personne, sinon, il recule.

``` python
while not rospy.is_shutdown() and  (difdist < 0) and (bil==True):
	bil=True
	t1=rospy.Time.now().to_sec()
			
	distance=distance-abs(vitesse)*(t1-t0)
	difdist=distance_voulue - distance
	print(distance)
	# publish the velocity
	self.cmd_vel.publish(move_cmd)
	# wait for 0.1 seconds (10 HZ) and publish again
	t0=t1
	r.sleep()
	bul=False
				
while not rospy.is_shutdown() and  (difdist > 0) and (bul==True):
	t1=rospy.Time.now().to_sec()
	distance=distance+abs(vitesse)*(t1-t0)
	difdist=distance_voulue - distance
	print(difdist)
	self.cmd_vel.publish(move_cmd_rec)
	t0=t1
	r.sleep()
	bil=False
```

Et enfin, on définit la fonction qui permet d’arrêter le robot:

``` python
def shutdown(self):
		# stop turtlebot
		rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
		self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
		rospy.sleep(1)

if __name__ == '__main__':
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")
```


## 6) Détection du visage humain avec OpenPose
## 7) Détection du squelette humain grâce à OpenPose




