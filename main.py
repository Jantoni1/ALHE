import math

import copy
from abc import abstractmethod

import networkx as nx
import matplotlib.pyplot as plt
from random import random, randint
import astar
from numpy import sort
from simanneal import Annealer

number_of_nodes = 10
number_of_edges = 4
max_distance = 1000
g = nx.connected_watts_strogatz_graph(number_of_nodes, number_of_edges, .5, tries=100, seed=None)
# print(g.edges)
for i in range(number_of_nodes):
    g.add_node(i, x=random() * max_distance, y=random() * max_distance)

#
# print(g.nodes[1])
# print(g.nodes.data()[1]['x'])
# print(g.edges)
x = []
for i in range(number_of_nodes - 1):
    if random() < 0.25:
        x = x + [i + 1]
x2 = copy.deepcopy(x)
x = [0] + x + [0]
x1 = copy.deepcopy(x)
print(x)

normalize = 1.0
traffic_base = 1.0
velocity = 10.0  # m/s
alfa = 1.0
beta = 1.0
gamma = 1.0
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

shortest_paths = dict(nx.shortest_path(g, source=None, target=None, weight='objective_frag'))
shortest_paths_length = dict(nx.shortest_path_length(g, source=None, target=None, weight='objective_frag'))
print(shortest_paths_length)


# for i in range(len(g.nodes)):
#     print(shortest_paths_length[i])


class PizzaDeliveryProblemAnnealing(Annealer):
    def move(self):
        """Swaps two cities in the route."""
        a = randint(1, len(self.state) - 2)
        b = randint(1, len(self.state) - 2)
        self.state[a], self.state[b] = self.state[b], self.state[a]

    def energy(self):
        e = 0
        for i in range(len(self.state) - 1):
            e += shortest_paths_length[self.state[i]][self.state[i + 1]]
        return e


initial_state = x
pizza = tsp = PizzaDeliveryProblemAnnealing(initial_state)
itinerary, miles = tsp.anneal()
print("Dla symulowanego wyzarzania:")
print(itinerary)
# for i in range(len(itinerary) - 1):
#     print(shortest_paths_length[itinerary[i]][itinerary[i+1]])
print(miles)
# print(x)
# nx.write_gml(g, "graph")
# sp = dict(nx.all_pairs_shortest_path(g))
# print(sp[1])
labels = nx.get_edge_attributes(g, 'objective_frag')

number_of_iterations = 50000
e_max = float("inf")
itinerary_max = []
for i in range(number_of_iterations):
    a = randint(1, len(x1) - 2)
    b = randint(1, len(x1) - 2)
    x1[a], x1[b] = x1[b], x1[a]
    e = 0
    for k in range(len(x1) - 1):
        e += shortest_paths_length[x1[k]][x1[k + 1]]
    if e < e_max:
        e_max = e
        itinerary_max = copy.deepcopy(x1)

print("Dla bladzenia przypadkowego:")
print(itinerary_max)
print(e_max)

heuristic_set = []
heuristic_dictionary = {}
x2.append(0)
for i in x2:
    heuristic_dictionary[i] = {}
for i in x2:
    for k in x2:
        if i != k:
            heuristic_set.append(math.sqrt((g.nodes[i]['x'] - g.nodes[k]['x']) ** 2
                                           + (g.nodes[i]['y'] - g.nodes[k]['y']) ** 2))
            heuristic_dictionary[i].update({k: math.sqrt((g.nodes[i]['x'] - g.nodes[k]['x']) ** 2
                                                         + (g.nodes[i]['y'] - g.nodes[k]['y']) ** 2)})
            heuristic_dictionary[k].update({i: math.sqrt((g.nodes[i]['x'] - g.nodes[k]['x']) ** 2
                                                         + (g.nodes[i]['y'] - g.nodes[k]['y']) ** 2)})
x2.remove(0)

heuristic_set = sorted(heuristic_set)
# print(heuristic_set)
shortest_distances_heuristic = heuristic_set[0:len(x2)]
heuristic_set = heuristic_set[len(x2):len(heuristic_set)]
# print(heuristic_set)
# print(shortest_distances_heuristic)

path = [0]
cost_sum = 0
cost_min = float("inf")
new_el = 0
last_el = 0
cost = 0
current_heuristic_value = 0
heuristic = 0
length = len(x2)
for k in range(length):
    cost_min = float("inf")
    for i in x2:
        cost = shortest_paths_length[last_el][i]
        if heuristic_dictionary[last_el][i] in shortest_distances_heuristic:
            heuristic += sum(shortest_distances_heuristic) - heuristic_dictionary[last_el][i]
        else:
            heuristic += sum(shortest_distances_heuristic) - \
                         shortest_distances_heuristic[len(shortest_distances_heuristic) - 1]
        if cost + heuristic < cost_min:
            cost_min = cost
            new_el = i
            current_heuristic_value = heuristic_dictionary[last_el][new_el]
    path.append(new_el)
    last_el = new_el
    cost_sum += cost_min
    x2.remove(new_el)
    if current_heuristic_value in shortest_distances_heuristic:
        shortest_distances_heuristic.remove(current_heuristic_value)
    else:
        shortest_distances_heuristic.remove(shortest_distances_heuristic[len(shortest_distances_heuristic) - 1])
cost_sum += shortest_paths_length[last_el][0]
path.append(0)
print("Dla A*: ")
print(path)
print(cost_sum)


# plt.subplot(111)
# edge_labels=nx.draw_networkx_edge_labels(g,pos=nx.spring_layout(g), edge_labels = labels)
# nx.draw(g, with_labels=True, font_weight='bold')
# plt.show()

