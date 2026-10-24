# Étape EX — Titre de l'étape

> **Résumé en 3 lignes**
> 1. Ce que l'étape cherche à obtenir.
> 2. Comment on s'y prend (méthode principale).
> 3. Ce qu'on a obtenu (ou : ce qu'on attend, si l'étape n'est pas finie).

| Champ | Valeur |
|---|---|
| Statut | ⚪ à faire / 🟡 en cours / 🟢 terminée / 🔴 bloquée |
| Dates | début AAAA-MM-JJ — fin AAAA-MM-JJ |
| Exigences couvertes | EX-… (voir [analyse du sujet](../00_analyse_du_sujet.md)) |
| Décisions liées | D-… (voir [registre des décisions](../decisions.md)) |
| Entrées de journal | [AAAA-MM-JJ](../journal/) |

---

## 1. Objectif

Une phrase qui dit ce que l'on veut obtenir, puis des **critères de réussite mesurables**
(on doit pouvoir répondre « oui » ou « non » à chacun) :

- [ ] Critère 1 (ex. : « la caméra publie des images à ≥ 15 Hz »)
- [ ] Critère 2

## 2. Pourquoi cette étape ?

Le lien avec le sujet et avec les autres étapes : qu'est-ce qu'elle débloque, qu'est-ce qui
en dépend ? Un lecteur qui ne connaît pas le domaine doit comprendre pourquoi on la fait.

## 3. Notions nécessaires

Les quelques notions à connaître pour suivre la suite, expliquées en deux ou trois phrases
chacune, avec un lien vers le document détaillé dans [`notions/`](../notions/) et vers le
[glossaire](../glossaire.md).

## 4. Choix techniques et justification

Pour chaque choix important : quelles options existaient, comment on les a comparées, ce
qu'on a retenu et pourquoi. Le choix est ensuite consigné dans le
[registre des décisions](../decisions.md).

| Critère | Option A | Option B |
|---|---|---|
| … | … | … |

**Retenu :** … — **Raison :** …

## 5. Méthodologie

La démarche, pas à pas. Pour chaque traitement, décrire la **transformation des données** :

```
entrée (topic, type, fréquence) → traitement (algorithme, paramètres) → sortie (topic, type)
```

Inclure les équations utiles (avec la signification de chaque symbole) et un schéma
(Mermaid ou image dans [`figures/`](../figures/)).

## 6. Implémentation

- Fichiers créés ou modifiés (liens relatifs vers le code)
- Nœuds, topics publiés et souscrits, types de messages
- Paramètres et leurs valeurs (avec l'unité et la raison de la valeur)
- Repères (TF) concernés

## 7. Protocole expérimental

- Conditions (monde simulé, robot, éclairage, vitesse…)
- Scénarios testés
- Métriques mesurées et comment elles sont calculées
- Nombre d'essais par scénario

## 8. Résultats

Tableaux, courbes, captures d'écran, liens vers les données (rosbags archivés hors Git).
Seulement des faits mesurés ici ; l'interprétation vient à la section suivante.

## 9. Analyse et discussion

Interprétation des résultats, comparaison aux critères de la section 1, limites,
écart attendu entre simulation et réalité, causes des échecs.

## 10. Reproduire

Les commandes exactes, dans l'ordre, pour refaire l'expérience depuis un clone propre du dépôt.

```bash
# terminal 1
...
```

## 11. Bilan et suites

Ce qui est acquis, ce qui reste à faire, ce qui est transmis à l'étape suivante.

## 12. Messages clés pour la présentation

Trois à cinq phrases à dire au superviseur, et les figures à montrer.
