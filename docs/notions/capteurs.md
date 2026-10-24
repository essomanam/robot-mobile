# Notion — Les capteurs du TurtleBot3

> **Résumé en 3 lignes**
> 1. Le robot perçoit le monde avec un **LiDAR** (distances dans un plan, 360°), une **caméra** (couleur, et profondeur si RGB-D) et estime son propre mouvement avec l'**odométrie des roues** et une **IMU**.
> 2. Aucun capteur n'est parfait : chacun a une portée, un bruit et des cas d'échec ; c'est leur **complémentarité** qui rend le robot fiable.
> 3. Les valeurs « simulation » ci-dessous sont lues dans le modèle Gazebo `turtlebot3_waffle` de Lyrical ; celles du robot réel seront mesurées à l'étape E5.

---

## 1. La plateforme TurtleBot3

Robot à **entraînement différentiel** : deux roues motrices indépendantes et des roulettes libres.
Il avance quand les deux roues tournent à la même vitesse et pivote quand elles tournent en sens
opposé.

| | Burger | Waffle / Waffle Pi |
|---|---|---|
| Dimensions (L × l × h) | 138 × 178 × 192 mm | 281 × 306 × 141 mm |
| Vitesse linéaire max. | 0,22 m/s | 0,26 m/s |
| Vitesse angulaire max. | 2,84 rad/s | 1,82 rad/s |
| Caméra d'origine | aucune | caméra Raspberry Pi (Waffle Pi) |

Source : [TB3] dans les [références](../references.md).

## 2. LiDAR

**Ce qu'il mesure.** Un faisceau laser tourne sur lui-même et mesure la distance au premier
obstacle dans chaque direction, dans un **plan horizontal**. Résultat : un message `LaserScan`,
c'est-à-dire une liste de distances, une par angle.

**Passage en coordonnées cartésiennes.** La mesure *i* donne une distance $r_i$ à l'angle
$\theta_i = \theta_{min} + i \cdot \Delta\theta$ ; le point correspondant, dans le repère du LiDAR, est :

$$
x_i = r_i \cos\theta_i \qquad y_i = r_i \sin\theta_i
$$

| Caractéristique | Simulation (Waffle) |
|---|---|
| Nombre de mesures par tour | 360 (une par degré) |
| Fréquence | 10 Hz |
| Portée | 0,12 à 3,5 m |
| Bruit simulé | gaussien, écart-type 0,01 m |

Sur le robot réel, la portée dépend du modèle monté (LDS-01 : 0,12 à 3,5 m ; LDS-02 : 0,16 à 8 m).

**Limites.**
- Ne voit **que dans son plan** : un plateau de table ou un objet bas lui échappe.
- Surfaces vitrées, miroirs, matériaux très sombres : mesures absentes ou fausses.
- Ne donne aucune information de couleur ou d'identité (un mur et une cible se ressemblent).

## 3. Caméra couleur (RGB)

**Ce qu'elle mesure.** Une image : pour chaque pixel, trois intensités (rouge, vert, bleu).

**Modèle sténopé (pinhole).** Un point 3D $(X, Y, Z)$ exprimé dans le **repère optique** de la
caméra (z vers l'avant) se projette au pixel $(u, v)$ :

$$
u = f_x \frac{X}{Z} + c_x \qquad v = f_y \frac{Y}{Z} + c_y
$$

où $f_x, f_y$ sont les focales en pixels et $(c_x, c_y)$ le centre optique. Ces valeurs sont publiées
dans le message `CameraInfo` (matrice **K**). Sur le robot réel, on les obtient par **calibration**.

| Caractéristique | Simulation (Waffle) |
|---|---|
| Résolution | 1920 × 1080 pixels |
| Fréquence | 30 Hz |
| Champ de vision horizontal | 1,03 rad (≈ 59°) |
| Profondeur | **aucune** |

La caméra du modèle `burger_cam`, utilisée par les prototypes, est très différente : 320 × 240
pixels et objectif **grand-angle** (≈ 182°), qui ne suit pas le modèle sténopé ci-dessus.

**Limites.**
- **Aucune distance** : une petite cible proche et une grande cible lointaine donnent la même image.
- Sensible à l'éclairage (ombres, reflets, contre-jour) et au flou de mouvement.
- Une haute résolution coûte du temps de calcul : on réduit souvent l'image avant traitement.

## 4. Caméra RGB-D

**Ce qu'elle mesure.** En plus de l'image couleur, une **image de profondeur** : pour chaque pixel,
la distance $Z$ de l'objet vu. Plusieurs techniques existent : lumière structurée (motif infrarouge
projeté), stéréo active (deux caméras + projecteur), temps de vol.

**Rétroprojection.** Connaissant $Z$, on retrouve le point 3D de chaque pixel en inversant le modèle sténopé :

$$
X = \frac{(u - c_x)\,Z}{f_x} \qquad Y = \frac{(v - c_y)\,Z}{f_y}
$$

Appliqué à tous les pixels, cela donne un **nuage de points** 3D.

**Limites typiques.** Portée utile d'environ 0,3 à 3-5 m ; le **bruit augmente avec la distance** ;
trous sur les surfaces brillantes, transparentes ou très sombres ; la lumière du soleil perturbe les
capteurs infrarouges.

**Pourquoi c'est central dans ce projet.** La profondeur donne l'**échelle réelle** au SLAM visuel,
la **distance de la cible** pour l'approche, et une vue en 3D des **obstacles hors du plan du LiDAR**.
Aucun modèle TurtleBot3 fourni avec Gazebo n'en possède : il faudra en ajouter une (étape E1).

## 5. Centrale inertielle (IMU)

**Ce qu'elle mesure.** Les **accélérations** (accéléromètre) et les **vitesses de rotation**
(gyromètre), à haute fréquence (200 Hz en simulation).

**Limites.** Chaque mesure a un petit biais ; intégrer ces mesures pour obtenir une orientation
ou une position fait croître l'erreur avec le temps. Utile sur des durées courtes, en complément.

## 6. Odométrie des roues

**Principe.** Des codeurs mesurent la rotation de chaque roue. Avec le rayon des roues $r$ et
l'écart entre elles $L$, on en déduit les vitesses du robot :

$$
v = \frac{r\,(\omega_d + \omega_g)}{2} \qquad \omega = \frac{r\,(\omega_d - \omega_g)}{L}
$$

où $\omega_d, \omega_g$ sont les vitesses de rotation des roues droite et gauche. On intègre ensuite
ces vitesses pour obtenir la position. En simulation : $r$ = 0,033 m, $L$ = 0,287 m, publication à 30 Hz.

**Limites.** Glissement des roues, rayon mal connu, sol irrégulier : de petites erreurs qui
s'**accumulent** (la *dérive*). Après quelques mètres, la position estimée peut être fausse de
plusieurs centimètres, puis de plus en plus. C'est précisément ce que le SLAM corrige.

## 7. Complémentarité

| Capteur | Mesure | Forces | Faiblesses | Usage dans le projet |
|---|---|---|---|---|
| LiDAR | distances dans un plan, 360° | précis, insensible à l'éclairage | 2D seulement, pas de couleur | SLAM (complément), sécurité, comparaison slam_toolbox |
| Caméra RGB | couleur, texture | riche, identifie cibles et marqueurs | pas de distance, sensible à la lumière | détection de cible, marqueurs, V-SLAM |
| Profondeur (RGB-D) | distance par pixel, 3D | échelle réelle, obstacles hors plan | portée courte, bruit, trous | V-SLAM, distance cible, obstacles non cartographiés |
| IMU | accélérations, rotations | rapide, indépendante de l'environnement | dérive si intégrée | complément d'odométrie (option) |
| Odométrie roues | déplacement | continue, simple | dérive | base de l'estimation de position |
