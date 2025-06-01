# Pathfinding

This implements the pathfinding algorithm Jump Point Search (JPS) as outlined in the 2011 paper by Daniel Harabor and Alban Grastien titled "Online Graph Pruning for Pathfinding on Grid Maps" which can be found at https://harabor.net/data/papers/harabor-grastien-aaai11.pdf.

Compared to classic Dijkstra's JPS is roughly 41 times faster with a size of 500 x 500. A graph showing this difference in speed can be found at https://docs.google.com/spreadsheets/d/1OVB1Yl6vTghZtO38zINSolpbtuzkI1TvzcwxwPO7F-k/edit?usp=sharing

JPS is fast due to its ability to ignore a large majority of tiles. 
