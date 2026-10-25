# Glossaire

> Tous les termes techniques employés dans la documentation, expliqués simplement.
> Classés par ordre alphabétique. Les termes en *italique* dans une définition ont leur propre entrée.

---

**Aliasing perceptif** — Confusion entre deux endroits différents qui se ressemblent ; source
de fausses *fermetures de boucle*.

**Alignement (de trajectoires)** — Transformation rigide (rotation + translation) qui
superpose au mieux une trajectoire estimée sur la vérité terrain avant de mesurer l'écart
(méthode de Umeyama / Horn). Évite de pénaliser un simple choix de repère différent.

**ament / ament_cmake / ament_python** — Système de construction des paquets ROS 2.
`ament_cmake` sert aux paquets C++ ou de configuration, `ament_python` aux paquets Python.

**ArUco (marqueur)** — Carré noir et blanc imprimé, codant un numéro, conçu pour être détecté
facilement par une caméra. Comme sa taille réelle est connue, on peut calculer sa *position* et
son *orientation* par rapport à la caméra.

**Asservissement visuel** — Commande d'un robot calculée directement à partir de ce que voit la
caméra (ex. : tourner tant que la cible n'est pas au centre de l'image).

**ATE (Absolute Trajectory Error)** — Erreur de trajectoire absolue : distance moyenne entre la
trajectoire estimée par le *SLAM* et la vraie trajectoire, après les avoir superposées au mieux.
Mesure la cohérence globale.

**Biais (d'un capteur)** — Erreur systématique : écart moyen entre la mesure et la vraie
valeur, qui ne disparaît pas en moyennant. À distinguer du *bruit*.

**Boucle fermée / boucle ouverte** — En boucle ouverte, on envoie une commande sans vérifier son
effet. En boucle fermée, on mesure le résultat, on le compare à l'objectif et on corrige.

**Bridge (pont) ros_gz** — Programme qui traduit les messages de *Gazebo* en messages *ROS 2*
et inversement.

**Bruit (d'un capteur)** — Erreur aléatoire qui change à chaque mesure ; on la caractérise par
son écart-type. Elle diminue quand on moyenne plusieurs mesures.

**Caméra RGB-D** — Caméra qui fournit, pour chaque pixel, la couleur (Rouge, Vert, Bleu) et la
**profondeur** (D, *depth*) : la distance de l'objet vu. Exemples : Intel RealSense, Orbbec Astra.

**Carte d'occupation (occupancy grid)** — Carte 2D découpée en cases, chaque case étant marquée
libre, occupée ou inconnue.

**Carte de coûts (costmap)** — Carte d'occupation enrichie utilisée par *Nav2* : chaque case a un
« coût » d'autant plus élevé qu'elle est proche d'un obstacle. Le planificateur cherche le chemin
de coût minimal.

**colcon** — Outil en ligne de commande qui compile tous les paquets d'un *workspace*.

**Contour (OpenCV)** — Courbe qui suit le bord d'une zone d'un *masque* binaire. Permet de mesurer
l'aire et la position d'une zone détectée.

**Covariance** — Mesure de l'incertitude d'une estimation (variance = écart-type au carré).
Un SLAM s'en sert pour pondérer chaque mesure : plus la covariance est petite, plus la mesure
est jugée fiable.

**cv_bridge** — Bibliothèque qui convertit une image ROS (`sensor_msgs/Image`) en tableau
*OpenCV* et inversement.

**DDS (Data Distribution Service)** — Norme de communication réseau sur laquelle repose ROS 2 pour
transporter les messages entre programmes et entre machines.

**Dérive (drift)** — Erreur qui s'accumule avec le temps dans une estimation de position par
*odométrie* : chaque petite erreur s'ajoute aux précédentes.

**Facteur temps réel (RTF, real-time factor)** — Rapport entre le temps écoulé dans la
simulation et le temps réel. RTF = 0,9 : la simulation avance à 90 % de la vitesse réelle.

**Fermeture de boucle (loop closure)** — Moment où un *SLAM* reconnaît un endroit déjà visité. Il
peut alors corriger toute la trajectoire et la carte accumulées depuis le premier passage.

**Gazebo** — Simulateur physique 3D qui reproduit un robot, ses capteurs et son environnement.
Version utilisée : Gazebo Jetty.

**HSV** — Représentation d'une couleur en Teinte (*Hue*), Saturation et Valeur (luminosité). Plus
adaptée que RGB pour détecter une couleur, car la teinte varie peu avec l'éclairage.

**Hystérésis** — Utilisation de deux seuils différents pour entrer dans un état et pour en sortir,
afin d'éviter de basculer sans cesse quand la mesure est proche du seuil.

**ICP (Iterative Closest Point)** — Algorithme qui recale deux nuages de points (ou deux scans
LiDAR) en cherchant le déplacement qui superpose au mieux leurs points les plus proches.

**IMU (centrale inertielle)** — Capteur qui mesure les accélérations et les vitesses de rotation
du robot.

**Launch (fichier)** — Script qui démarre en une seule commande plusieurs *nœuds* avec leurs
*paramètres*.

**LiDAR** — Télémètre laser tournant : il mesure la distance aux obstacles tout autour du robot, dans
un plan. Le TurtleBot3 porte un LDS (LiDAR 2D, 360°).

**Localisation** — Estimation de la position et de l'orientation du robot dans une carte.

**Masque (binaire)** — Image en noir et blanc où les pixels blancs sont ceux qui satisfont une
condition (ex. : « la couleur est rouge »).

**MCAP** — Format de fichier utilisé pour les *rosbags*, qui permet la compression (zstd).

**Mermaid** — Langage texte qui permet de dessiner des schémas dans un fichier Markdown ; GitHub
les affiche directement.

**Message** — Donnée structurée échangée entre *nœuds* ROS 2 (ex. `sensor_msgs/LaserScan` pour un
scan LiDAR).

**Middleware** — Couche logicielle intermédiaire qui fait communiquer des programmes ; pour ROS 2,
c'est l'implémentation *DDS* (Fast DDS par défaut).

**Nav2 (Navigation 2)** — Ensemble de paquets ROS 2 pour la navigation autonome : localisation,
*cartes de coûts*, planification de chemin, suivi de trajectoire, comportements de secours.

**Nuage de points (point cloud)** — Ensemble de points 3D, obtenu par exemple en projetant chaque
pixel d'une image de profondeur dans l'espace.

**Nœud (d'un graphe de SLAM)** — Pose du robot mémorisée par le SLAM avec les données des
capteurs à cet instant (on parle aussi d'image clé, *keyframe*).

**Nœud (node)** — Programme ROS 2 qui remplit une fonction (lire un capteur, détecter une cible,
commander les moteurs…) et communique avec les autres par *topics*.

**Odométrie visuelle** — Estimation du déplacement du robot à partir des images successives
de la caméra, sans les roues.

**Odométrie** — Estimation du déplacement du robot à partir de la rotation de ses roues (odométrie
des roues) ou des images (odométrie visuelle). Précise à court terme, mais elle *dérive*.

**OpenCV** — Bibliothèque de référence pour le traitement d'image.

**Paramètre (ROS)** — Valeur de réglage d'un nœud (seuil, gain, vitesse…) modifiable sans changer
le code, généralement depuis un fichier *YAML*.

**Paramètres intrinsèques (calibration)** — Caractéristiques internes d'une caméra : focales
$f_x, f_y$ et centre optique $(c_x, c_y)$, en pixels. Publiés dans `CameraInfo` (matrice K).

**Profondeur (image de)** — Image où chaque pixel contient une distance au lieu d'une couleur.

**Profondeur recalée (registered depth)** — Image de profondeur dont chaque pixel correspond
exactement au même point que le pixel de l'image couleur.

**Régulateur proportionnel (P)** — Loi de commande où la correction est proportionnelle à l'erreur :
commande = gain × erreur.

**Rendu hors écran (headless rendering)** — Calcul des images des caméras simulées sans ouvrir
de fenêtre, utile pour les essais automatisés.

**Repère (frame)** — Système d'axes attaché à un objet (robot, capteur, carte). Les positions sont
toujours exprimées dans un repère donné. Voir *TF*.

**Repère optique** — Repère d'une caméra selon la convention de la vision : z vers l'avant, x vers
la droite, y vers le bas.

**ROS 2 (Robot Operating System 2)** — Ensemble d'outils et de bibliothèques qui facilitent
l'écriture de logiciels robotiques. Ce n'est pas un système d'exploitation mais un *middleware*
avec ses outils. Version utilisée : Lyrical Luth.

**rosbag** — Enregistrement de messages ROS 2 dans un fichier, que l'on peut rejouer plus tard
comme si les capteurs étaient actifs.

**RPE (Relative Pose Error)** — Erreur de pose relative : erreur sur le déplacement entre deux
instants proches. Mesure la *dérive* locale.

**RTAB-Map** — Logiciel de *SLAM* (Real-Time Appearance-Based Mapping) qui utilise caméra RGB-D,
stéréo ou LiDAR, et détecte les *fermetures de boucle* par l'apparence des images.

**RViz** — Outil de visualisation 3D des données ROS 2 (scans, images, cartes, repères, trajectoires).

**SDF / URDF / xacro** — Formats de description d'un robot ou d'un monde : URDF (format ROS),
SDF (format Gazebo, plus complet), xacro (URDF avec macros).

**SLAM (Simultaneous Localization And Mapping)** — Construire la carte d'un lieu inconnu tout en
s'y localisant. **V-SLAM** : SLAM qui utilise principalement une ou plusieurs caméras.

**Temps réel (souple)** — Traitement assez rapide pour suivre le rythme des capteurs, sans garantie
stricte d'échéance.

**Temps simulé (sim time)** — Horloge fournie par *Gazebo*. Les nœuds réglés avec
`use_sim_time: true` l'utilisent à la place de l'horloge de l'ordinateur.

**Texture** — Image plaquée sur la surface d'un objet 3D pour lui donner son apparence (motifs,
couleurs). Dans Gazebo, `albedo_map`.

**TF (transformations)** — Système ROS 2 qui tient à jour la position relative de tous les *repères*
(carte, robot, capteurs) et permet de convertir une mesure d'un repère à un autre.

**Topic** — Canal de communication nommé (ex. `/scan`) sur lequel des nœuds publient des messages
et d'autres s'y abonnent.

**TurtleBot3** — Robot mobile pédagogique à deux roues motrices (ROBOTIS), en versions Burger
(petite) et Waffle / Waffle Pi (plus grande, avec caméra).

**TwistStamped** — Message de commande de vitesse (vitesse linéaire et angulaire) accompagné d'un
horodatage. Utilisé sur `/cmd_vel` par le TurtleBot3 sous ROS 2 Lyrical.

**Vérité terrain (ground truth)** — Valeur exacte, connue indépendamment de l'estimation à évaluer
(en simulation : la vraie position du robot donnée par Gazebo).

**Workspace (colcon)** — Dossier qui regroupe des paquets ROS 2 et leurs résultats de compilation
(`build/`, `install/`, `log/`).

**YAML** — Format de fichier texte lisible utilisé pour écrire les *paramètres* ROS 2.

**Zone morte** — Plage d'erreur autour de zéro dans laquelle on ne corrige pas, pour éviter que le
robot oscille autour de la consigne.
