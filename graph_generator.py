import math

import copy
from abc import abstractmethod

import networkx as nx
import matplotlib.pyplot as plt
from random import random, randint
import astar
from numpy import sort
from plotly.utils import numpy
from simanneal import Annealer


#
# #README
# Algorytmy w nastepujacej kolejnosci: Wyżarzanie, bladzenie, A*
# Kod jest troche poplatany bo nie ogarniam obiektowosci a nawet metod w pythonie
# Nie polecam ziom zmieniac za duzo bo sa zmienne ktore ida przez caly plik i niektore nie uzywaja wartosci
# z poprzednich algorytmow (czyli uzylem tylko tej samej nazwy zmiennej) a niektore jak na przyklad
# zbior wszystkich najkrotszych sciezek sa tworzone raz i uzywane z tej samej zmiennej we wszystkich algorytmach
# Algorytm dostaje na wejscie wektor wierzcholkow ktore musi odwiedzic (nie moze wystapic 0)
# na wyjsciu wyrzuca kolejnosc w jakiej odwiedzi wierzcholki (da sie z tego zrobic pelna sciezke) i
# wartosc funkcji celu (czyli miks wszystkiego)
# Mozliwe, ze znajde czas zeby uporzadkowac to bardziej, ale licze ze ni bedzie takiej potrzby

number_of_nodes = 80
number_of_edges = 4
scale = 10000
normalize = 1.0
traffic_base = 1.0
velocity = 10.0  # m/s
alfa = 1.0
beta = 1.0
g = nx.connected_watts_strogatz_graph(number_of_nodes, number_of_edges, .5, tries=100, seed=None)
# print(g.edges)
position = nx.spring_layout(g)
for i in range(number_of_nodes):
    g.add_node(i, x=scale * numpy.float(position[i][0]), y=scale * numpy.float(position[i][1]))


for edge in g.edges:
    distance = math.sqrt((g.nodes[edge[0]]['x'] - g.nodes[edge[1]]['x']) ** 2
                         + (g.nodes[edge[0]]['y'] - g.nodes[edge[1]]['y']) ** 2) * (random() + 1.0)
    traffic = random() * normalize + traffic_base
    time = distance / velocity * traffic
    g.add_edge(edge[0], edge[1],
               distance=distance,
               traffic=traffic,
               time=distance / velocity * traffic,
               objective_frag=alfa * distance + beta * time)

nx.write_graphml_xml(g, 'graph')

nx.draw(g, position, with_labels=True, font_weight='bold')
plt.show()