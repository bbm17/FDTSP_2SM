import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
from operator import add
import math, time
import operator
import csv
rnd = np.random
import time
import os
import sys, getopt
import copy

import testeo_factibilidad

def main(name_file, instances_path, soluciones_folder):

    instancia = instances_path + '/' + name_file

    indices = []
    n = ''
    endurance = ''
    speed_drone = ''
    for i in range(len(name_file)):
        if name_file[i] == '_':
            indices.append(i)
        elif name_file[i] == '.':
            indices.append(i)

    square = name_file[0]+name_file[1]

    for j in range(indices[0] + 1, indices[1]):
        n += name_file[j]

    for j in range(indices[1] + 1, indices[2]):
        endurance += name_file[j]

    for j in range(indices[2] + 1, indices[3]):
        speed_drone += name_file[j]

    'PARAMETERS' 
    n = int(n)

    if n <= 40:
        tiempoLimite = 60 
    elif n == 60 or n == 80:
        tiempoLimite = 300
    elif n == 100 or n == 150:
        tiempoLimite = 600

    square = int(square)                                                            
    speed_drone = int(speed_drone)                                                         
    endurance1 = int(int(endurance)/60)                                                      
    endurance2 = int(endurance)/60

    epochs = 10                                                              
    nrSeeds = 10
    iter_proposed_model = 1                                                            
    dr = 1                                                                   
    delta = 0.7

    'FUNCTIONS'
    'Bisecting K-means Module'
    def convert_to_2d_array(points):
        points = np.array(points)
        if len(points.shape) == 1:
            points = np.expand_dims(points, -1)
        return points
    def SSE(points):
        points = convert_to_2d_array(points)
        centroid = np.mean(points, 0)
        errors = np.linalg.norm(points - centroid, ord=2, axis=1)
        return np.sum(errors)
    def kmeans(points, k=2, epochs=epochs, max_iter=100, verbose=False):
        points = convert_to_2d_array(points)
        assert len(points) >= k
        best_sse = np.inf
        last_sse = np.inf
        for ep in range(epochs):
            if ep == 0:
                random_idx = np.random.permutation(points.shape[0])
                centroids = points[random_idx[:k], :]
            for it in range(max_iter):
                clusters = [None] * k

                indexes = [False for i in range(len(clusters))]
                usados = []
                for p in points:
                    index = np.argmin(np.linalg.norm(centroids-p, 2, 1))
                    indexes[index] = True
                    if clusters[index] is None:
                        clusters[index] = np.expand_dims(p, 0)
                        usados.append(p.tolist())
                    else:
                        clusters[index] = np.vstack((clusters[index], p))
                if False in indexes:
                    index2 = indexes.index(False)
                    min_value = math.inf
                    min_p = math.inf
                    for p in points:
                        valor = np.linalg.norm(centroids[index2]-p)
                        valor2 = float(valor)
                        if valor2 < min_value:
                            if p.tolist() not in usados:
                                min_p = p
                                min_value = valor2
                    clusters[index2] = np.expand_dims(min_p, 0)

                centroids = [np.mean(c, 0) for c in clusters]
                sse = np.sum([SSE(c) for c in clusters])
                gain = last_sse - sse
                if verbose:
                    print((f'Epoch: {ep:3d}, Iter: {it:4d}, '
                        f'SSE: {sse:12.4f}, Gain: {gain:12.4f}'))
                if sse < best_sse:
                    best_clusters, best_sse = clusters, sse
                if np.isclose(gain, 0, atol=0.00001):
                    break
                last_sse = sse
        return best_clusters, centroids 
    def bisecting_kmeans(points, kk=2, epochs=epochs, max_iter=100, verbose=False):
        points = convert_to_2d_array(points)
        clusters = [points]
        while len(clusters) < kk:
            max_sse_i = np.argmax([SSE(c) for c in clusters])
            cluster = clusters.pop(max_sse_i)
            kmeans_clusters, centroids = kmeans(cluster, k=kk, epochs=epochs, max_iter=max_iter, verbose=verbose)
            clusters.extend(kmeans_clusters)
        return clusters, centroids
    def visualize(new_clusters, new_centroids, trucks, show_coordinates=True):
        plt.figure(figsize=(10, 8))
        for i, new_cluster in enumerate(new_clusters):
            points = convert_to_2d_array(new_cluster)
            if points.shape[1] < 2:
                points = np.hstack([points, np.zeros_like(points)])
            plt.plot(points[:,0], points[:,1], 'o', label='Drone Node')
            plt.scatter(new_centroids[i][0], new_centroids[i][1], marker='*',label='Centroid', s=300)
            plt.scatter(np.array(trucks)[:, 0], np.array(trucks)[:, 1], marker='s', label='Truck Node', color='red', s=100)
            plt.title('Bisecting K-means result',fontsize=16)
            plt.xlabel('service area (mile)',fontsize=16)
            plt.ylabel('service area (mile)',fontsize=16)
            if show_coordinates==True:
                for point in points:
                    plt.text(point[0], point[1], '({}, {})'.format(round(point[0], 2), round(point[1], 2)))
                plt.text(new_centroids[i][0], new_centroids[i][1], '({}, {})'.format(round(new_centroids[i][0], 2), round(new_centroids[i][1], 2)))
        resolution_value = 1200
        plt.legend(['Drone Node', 'Centroid', 'Truck Node'])
        plt.savefig("bisecting.png", format="png", dpi=resolution_value)
        plt.show()
    'Simulated Annealing module'
    class SimAnneal(object):
        def __init__(self, coords, tpo_inicio, tpo_lim, T=-1, alpha=-1, stopping_T=-1, stopping_iter=-1, idx_deposito = 0):
            self.coords = coords
            self.N = len(coords)
            self.T = math.sqrt(self.N) if T == -1 else T
            self.T_save = self.T 
            self.alpha = 0.995 if alpha == -1 else alpha
            self.stopping_temperature = 1e-10000 if stopping_T == -1 else stopping_T
            self.stopping_iter = 100000 if stopping_iter == -1 else stopping_iter
            self.iteration = 1
            self.nodes = [i for i in range(self.N)]
            self.best_solution = None
            self.best_fitness = float("Inf")
            self.fitness_list = []

            self.indice_depot = idx_deposito
            self.tpo_ini = tpo_inicio
            self.tpo_lim = tpo_lim

        def initial_solution(self):
            cur_node = self.indice_depot

            solution = [cur_node]
            free_nodes = set(self.nodes)
            free_nodes.remove(cur_node)
            while free_nodes:
                next_node = min(free_nodes, key=lambda x: self.dist(cur_node, x)) 
                free_nodes.remove(next_node)
                solution.append(next_node)
                cur_node = next_node
            cur_fit = self.fitness(solution)
            if cur_fit < self.best_fitness:
                self.best_fitness = cur_fit
                self.best_solution = solution
            self.fitness_list.append(cur_fit)
            return solution, cur_fit
        def dist(self, node_0, node_1):
            coord_0, coord_1 = self.coords[node_0], self.coords[node_1]
            return math.sqrt((coord_0[0] - coord_1[0]) ** 2 + (coord_0[1] - coord_1[1]) ** 2)
        def fitness(self, solution):
            cur_fit = 0
            for i in range(self.N):
                cur_fit += self.dist(solution[i % self.N], solution[(i + 1) % self.N])
            return cur_fit
        def p_accept(self, candidate_fitness):
            if self.T == 0.0:
                self.T = 1.0E-300
            return math.exp(-abs(candidate_fitness - self.cur_fitness) / self.T)
        def accept(self, candidate):
            candidate_fitness = self.fitness(candidate)
            if candidate_fitness < self.cur_fitness:
                self.cur_fitness, self.cur_solution = candidate_fitness, candidate
                if candidate_fitness < self.best_fitness:
                    self.best_fitness, self.best_solution = candidate_fitness, candidate
            else:
                if random.random() < self.p_accept(candidate_fitness):
                    self.cur_fitness, self.cur_solution = candidate_fitness, candidate
        def anneal(self):
            self.cur_solution, self.cur_fitness = self.initial_solution()
            while self.iteration <= self.stopping_iter and self.T >= self.stopping_temperature:
                candidate = list(self.cur_solution)
                l = random.randint(2, self.N - 1)
                i = random.randint(1, self.N - l)

                candidate[i : (i + l)] = reversed(candidate[i : (i + l)])
                self.accept(candidate)
                self.T *= self.alpha
                self.iteration += 1
                self.fitness_list.append(self.cur_fitness)

                if time.time() - self.tpo_ini >= self.tpo_lim:
                    break 
            improvement = 100 * (self.fitness_list[0] - self.best_fitness) / (self.fitness_list[0])
        def batch_anneal(self, times=10):
            for i in range(1, times + 1):
                self.T = self.T_save
                self.iteration = 1
                self.cur_solution, self.cur_fitness = self.initial_solution()
                self.anneal()
        def visualize_routes(self):
            plotTSP([self.best_solution], self.coords) 
        def visualize_routes2(self):
            plotTSP2([self.best_solution], self.coords)
        def get_routes(self):
            return self.best_solution
    def plotTSP(paths, points, num_iters=1):
        plt.figure(figsize=(10, 8))
        for i, point_id in enumerate(points):
            plt.scatter(point_id[0], point_id[1], color='g',marker='s',label="Truck Node",s=120)       
        x = []; y = []
        for i in paths[0]:
            x.append(points[i][0])
            y.append(points[i][1])
        a_scale = float(max(x))/float(100)
        if num_iters > 1:
            for i in range(1, num_iters):
                xi = []; yi = [];
                for j in paths[i]:
                    xi.append(points[j][0])
                    yi.append(points[j][1])
                plt.arrow(xi[-1], yi[-1], (xi[0] - xi[-1]), (yi[0] - yi[-1]),
                        head_width = a_scale, color = 'r',
                        length_includes_head = True, ls = 'dashed',
                        width = 0.001/float(num_iters))
                for i in range(0, len(x) - 1):
                    plt.arrow(xi[i], yi[i], (xi[i+1] - xi[i]), (yi[i+1] - yi[i]),
                            head_width = a_scale, color = 'r', length_includes_head = True,
                            ls = 'dashed', width = 0.001/float(num_iters))
        plt.arrow(x[-1], y[-1], (x[0] - x[-1]), (y[0] - y[-1]), head_width = a_scale,
                color ='red', length_includes_head=True)
        for i in range(0,len(x)-1):
            plt.arrow(x[i], y[i], (x[i+1] - x[i]), (y[i+1] - y[i]), head_width = a_scale,
                    color = 'red', length_includes_head = True)
        plt.xlim(min(x), max(x)*1.1)
        plt.ylim(min(y), max(y)*1.1)
        plt.title('Shortest Truck Route after Clustering',fontsize=16)
        plt.xlabel('service area (mile)',fontsize=16)
        plt.ylabel('service area (mile)',fontsize=16)
        plt.legend(['Truck Node'])
        resolution_value = 1200
        plt.savefig("truck nodes.png", format="png", dpi=resolution_value)
        plt.show()     
    def plotTSP2(paths, points, num_iters=1):
        plt.figure(figsize=(10, 8))
        for i, point_id in enumerate(points):
            plt.scatter(point_id[0], point_id[1], color='blue',marker='o',label="Customer Node",s=120)      
        x = []; y = []
        for i in paths[0]:
            x.append(points[i][0])
            y.append(points[i][1])
        a_scale = float(max(x))/float(100)
        if num_iters > 1:
            for i in range(1, num_iters):
                xi = []; yi = [];
                for j in paths[i]:
                    xi.append(points[j][0])
                    yi.append(points[j][1])
                plt.arrow(xi[-1], yi[-1], (xi[0] - xi[-1]), (yi[0] - yi[-1]),
                        head_width = a_scale, color = 'r',
                        length_includes_head = True, ls = 'dashed',
                        width = 0.001/float(num_iters))
                for i in range(0, len(x) - 1):
                    plt.arrow(xi[i], yi[i], (xi[i+1] - xi[i]), (yi[i+1] - yi[i]),
                            head_width = a_scale, color = 'r', length_includes_head = True,
                            ls = 'dashed', width = 0.001/float(num_iters))
        plt.arrow(x[-1], y[-1], (x[0] - x[-1]), (y[0] - y[-1]), head_width = a_scale,
                color ='red', length_includes_head=True)
        for i in range(0,len(x)-1):
            plt.arrow(x[i], y[i], (x[i+1] - x[i]), (y[i+1] - y[i]), head_width = a_scale,
                    color = 'red', length_includes_head = True)
        plt.xlim(min(x), max(x)*1.1)
        plt.ylim(min(y), max(y)*1.1)
        plt.title('Shortest Truck Route for all customer nodes (TSP-0D)',fontsize=16)
        plt.xlabel('service area (mile)',fontsize=16)
        plt.ylabel('service area (mile)',fontsize=16)
        plt.legend(['Customer Node'])
        resolution_value = 1200
        plt.savefig("Customer Node.png", format="png", dpi=resolution_value)
        plt.show() 
        
    'FDTSP ALGORITHM BEGINS'
    dr = 1                  
    costos = []
    tiempos = []
    w_times = []
    d_deliveries = []
    s_levels = []
    for seed in range(1, nrSeeds + 1):
        random.seed(seed)
        np.random.seed(seed)
        rnd = np.random

        inicio = time.time()
        mejor_costo = math.inf
        mejor_waiting_time = math.inf
        mejor_drone_delivery = 0
        mejor_service_level = 0

        for it in range(0,iter_proposed_model):

            if time.time() - inicio >= tiempoLimite:
                break
            
            df = pd.read_csv(instancia)

            df = df[['X','Y']]
            points = np.array(df.values.tolist())
            algorithm = bisecting_kmeans
            nd=2
            K=math.ceil(n/(nd+1))
            verbose = False
            max_iter = 1000
            clusters, centroids = algorithm(points=points, kk=K, verbose=verbose, max_iter=max_iter, epochs=epochs)
            
            c = 300                   
            
            d = dr + 1                         
            for j in range(c):
                new_clusters = []
                new_centroids = []
                for i in range(len(clusters)):
                    if len(clusters[i])>d:
                        K = 2
                        verbose = False
                        max_iter = 1000
                        epochs = epochs
                        _clusters, _centroids = bisecting_kmeans(points=clusters[i], kk=K, verbose=verbose, max_iter=max_iter, epochs=epochs)
                        new_clusters.extend(_clusters)
                        new_centroids.extend(_centroids)
                    else:
                        new_clusters.append(clusters[i])
                        new_centroids.append(centroids[i])
                clusters = new_clusters
                centroids = new_centroids

                if time.time() - inicio >= tiempoLimite:
                    break

            trucks = []
            for j in range(len(clusters)):
                error = []
                for i in range(len(clusters[j])):
                    
                    if clusters[j][i].tolist() == points[0].tolist():
                        k = -9999999
                    else:
                        k = np.linalg.norm(clusters[j][i]-centroids[j], ord=2, axis=0)
                    error.append(k)
                
                index_min = np.argmin(error)
                truck = clusters[j][index_min]
                trucks.append(truck)
            trucks = np.array(trucks)
            list_trucks = trucks.copy().tolist()
            sparse_clusters = clusters.copy()
            list_clusters = [cluster.tolist() for cluster in sparse_clusters]
            for i in range(len(list_clusters)):
                list_clusters[i].remove(list_trucks[i])
            list_drones = list_clusters.copy()
            del list_clusters
            list_dr1 = []
            for k in list_drones:
                if len(k)==0:
                    list_dr1.append([])
                elif len(k)==1:
                    list_dr1.append(k[0]) 
            d1_tr_test = []
            for i in range(len(list_dr1)):
                if list_dr1[i]!=[]:
                    dr1_test = math.sqrt((list_trucks[i][0]-list_dr1[i][0])**2+(list_trucks[i][1]-list_dr1[i][1])**2)
                    d1_tr_test.append(dr1_test)
                else:
                    d1_tr_test.append(0)
            d1_tr_test_note = [random.randrange(1, 1000, 1) for i in range(len(d1_tr_test))]
            for k in range(len(d1_tr_test)):
                if d1_tr_test[k] > (endurance2*delta*speed_drone)/2: 
                    d1_tr_test_note[k] = 'NO LAUNCHED'
                else:
                    d1_tr_test_note[k] = d1_tr_test[k]
            noLaunched_indices_d1 = [index for index, element in enumerate(d1_tr_test_note) if element == "NO LAUNCHED"]
            count1 = len(noLaunched_indices_d1)
            list_dr1_new = list_dr1.copy()
            k = 0
            for i, idx in enumerate(noLaunched_indices_d1):
                list_trucks.insert(i, list_dr1_new[idx + k])
                list_dr1_new.insert(i, [])
                k +=1
            for i, idx in enumerate(noLaunched_indices_d1):
                list_dr1_new[idx+count1] = []
                idx +=1
            list_trucks = [i for i in list_trucks if i!=[]]
            trucks_new = list_trucks.copy() 
            trucks_new = np.array(trucks_new)                                             

  
            pd.DataFrame(trucks_new).to_csv(soluciones_folder + '/' + "trucks_new"+ '_' + name_file[:4] + '_' + str(n) + '_' + str(endurance) + '_' + str(speed_drone) + '_' + str(dr) + '_' + str(seed) +".csv", index=False)                
          
            def load_csv():                                     
                data = pd.read_csv(soluciones_folder + '/' + "trucks_new"+ '_' + name_file[:4] + '_' + str(n) + '_' + str(endurance) + '_' + str(speed_drone) + '_' + str(dr) + '_' + str(seed) +".csv").values
                x, y = list(data[:, 0]), list(data[:, 1])
                return x, y
            x, y = load_csv()
            coords1 = [[x[i], y[i]] for i in range(len(x))]
            
            for l in range(len(coords1)):
                if round(coords1[l][0],3) == round(points[0].tolist()[0],3) and round(coords1[l][1],3) == round(points[0].tolist()[1],3):
                    indice_deposito = l
                    break

            if len(coords1) <= 2:
                road_map_sa_truck_nodes = [coords1[indice_deposito]]
                for i in coords1:
                    if i != indice_deposito:
                        road_map_sa_truck_nodes.append(coords1[i])
            else:
                sa = SimAnneal(coords1, inicio, tiempoLimite, stopping_iter=100000000, idx_deposito = indice_deposito)
                sa.anneal()
                road_map_sa_truck_nodes = sa.get_routes() 

            list_trucks_new = trucks_new.copy().tolist()

            vuelos = {}
            for i in list_dr1_new:
                if i != []:
                    vuelos[tuple(i)] = []

            d1_tr = []
            for i in range(len(road_map_sa_truck_nodes)):
                if list_dr1_new[road_map_sa_truck_nodes[i]]!=[]:
                    tempo = math.sqrt((list_trucks_new[road_map_sa_truck_nodes[i]][0]-list_dr1_new[road_map_sa_truck_nodes[i]][0])**2+\
                                    (list_trucks_new[road_map_sa_truck_nodes[i]][1]-list_dr1_new[road_map_sa_truck_nodes[i]][1])**2)
                    d1_tr.append(tempo)

                    for j in vuelos:
                        if j[0] == list_dr1_new[road_map_sa_truck_nodes[i]][0] and j[1] == list_dr1_new[road_map_sa_truck_nodes[i]][1]:
                            vuelos[j].append(list_trucks_new[road_map_sa_truck_nodes[i]])
                            break
                else:
                    d1_tr.append(0)
            d1_ntr = []
            for i in range(len(road_map_sa_truck_nodes)-1):
                if list_dr1_new[road_map_sa_truck_nodes[i]]!=[]:
                    tempo11 = math.sqrt((list_dr1_new[road_map_sa_truck_nodes[i]][0]-list_trucks_new[road_map_sa_truck_nodes[i+1]][0])**2+\
                                        (list_dr1_new[road_map_sa_truck_nodes[i]][1]-list_trucks_new[road_map_sa_truck_nodes[i+1]][1])**2)
                    d1_ntr.append(tempo11) 

                    for j in vuelos:
                        if j[0] == list_dr1_new[road_map_sa_truck_nodes[i]][0] and j[1] == list_dr1_new[road_map_sa_truck_nodes[i]][1]:
                            vuelos[j].append(list_trucks_new[road_map_sa_truck_nodes[i+1]])
                            break
                else:
                    d1_ntr.append(0)
            for i in range(len(road_map_sa_truck_nodes)-1,len(road_map_sa_truck_nodes)):
                if list_dr1_new[road_map_sa_truck_nodes[i]]!=[]:
                    tempo101 = math.sqrt((list_dr1_new[road_map_sa_truck_nodes[i]][0]-list_trucks_new[road_map_sa_truck_nodes[0]][0])**2+\
                                        (list_dr1_new[road_map_sa_truck_nodes[i]][1]-list_trucks_new[road_map_sa_truck_nodes[0]][1])**2)
                    d1_ntr.append(tempo101)

                    for j in vuelos:
                        if j[0] == list_dr1_new[road_map_sa_truck_nodes[i]][0] and j[1] == list_dr1_new[road_map_sa_truck_nodes[i]][1]:
                            vuelos[j].append(list_trucks_new[road_map_sa_truck_nodes[0]])
                            break
                else:
                    d1_ntr.append(0)        
            d1_travel = list(map(add,d1_tr,d1_ntr))
            d1_tr_new = [random.randrange(1, 1000, 1) for i in range(len(d1_tr))]
            d1_ntr_new = [random.randrange(1, 1000, 1) for i in range(len(d1_ntr))]
            for k in range(len(d1_tr)):
                if d1_travel[k] > endurance2*delta*speed_drone and 2*d1_tr[k] < (endurance2-(0.5/60))*speed_drone:
                    d1_tr_new[k] = 'COMEBACK'
                    d1_ntr_new[k] = 'COMEBACK'
                elif d1_travel[k] > endurance2*delta*speed_drone and 2*d1_tr[k] > (endurance2-(0.5/60))*speed_drone:
                    d1_tr_new[k] = 'NO LAUNCHED 2'
                    d1_ntr_new[k] = 'NO LAUNCHED 2'
                else:
                    d1_tr_new[k] = d1_tr[k]
                    d1_ntr_new[k] =  d1_ntr[k]

            for i in range(len(road_map_sa_truck_nodes)):
                if list_dr1_new[road_map_sa_truck_nodes[i]]!=[] and d1_tr_new[i] == 'COMEBACK':
        
                    for j in vuelos: 
                        if j[0] == list_dr1_new[road_map_sa_truck_nodes[i]][0] and j[1] == list_dr1_new[road_map_sa_truck_nodes[i]][1]:
                            vuelos[j][1] = list_trucks_new[road_map_sa_truck_nodes[i]]
                            break

            d1_tr_new_temp = [random.randrange(1, 1000, 1) for i in range(len(d1_tr_new))]
            d1_ntr_new_temp = [random.randrange(1, 1000, 1) for i in range(len(d1_ntr_new))]
            for k in range(len(d1_tr_new)):
                if d1_tr_new[k] =='COMEBACK' or d1_tr_new[k]=='NO LAUNCHED 2' :
                    d1_tr_new_temp[k] = 0
                    d1_ntr_new_temp[k] = 0 
                else:
                    d1_tr_new_temp[k] = d1_tr_new[k]
                    d1_ntr_new_temp[k] = d1_ntr_new[k]

            travel_d1_tr = [60*within_distance1/speed_drone for within_distance1 in d1_tr_new_temp]

            setup_d1 = [random.randrange(1, 1000, 1) for i in range(len(d1_tr_new))]
            for k in range(len(d1_tr_new)):
                if d1_tr_new[k] =='NO LAUNCHED 2' or d1_tr_new[k]== 0:
                    setup_d1[k] = 0
                else:
                    setup_d1[k] = 1 

            service_d1 = [random.randrange(1, 1000, 1) for i in range(len(d1_tr_new))]
        
            for k in range(len(d1_tr_new)):
                if d1_tr_new[k] =='NO LAUNCHED 2' or d1_tr_new[k]== 0:
                    service_d1[k] = 0
                else:
                    service_d1[k] = 0.5 
            within_d1 = [sum(x) for x in zip(travel_d1_tr,setup_d1,service_d1)]
            travel_d1_ntr = [60*within_distance1/speed_drone for within_distance1 in d1_ntr_new_temp]

            time_d1  = [sum(x) for x in zip(within_d1,travel_d1_ntr)]
            time_drone = time_d1

            waiting_distance_dr1 = [random.randrange(1, 1000, 1) for i in range(len(d1_tr_new))]
            for k in range(len(d1_tr_new)):
                if d1_tr_new[k] == 'COMEBACK':
                    waiting_distance_dr1[k] = 2*d1_tr[k]
                else:
                    waiting_distance_dr1[k] = 0
            waiting_time_dr1 = [60*within_distance1/speed_drone for within_distance1 in waiting_distance_dr1]
            waiting_drone = waiting_time_dr1

            if d1_tr_new.count('NO LAUNCHED 2') == 0:
                service_truck = [0.5 if i==0 else 0 for i in waiting_drone] 
                service_truck[0] = 0 

                distance_trucks = []
                for i in range(len(road_map_sa_truck_nodes)-1):
                    temp = abs((list_trucks_new[road_map_sa_truck_nodes[i]][0]-list_trucks_new[road_map_sa_truck_nodes[i+1]][0])) + abs(list_trucks_new[road_map_sa_truck_nodes[i]][1]-list_trucks_new[road_map_sa_truck_nodes[i+1]][1])
                    
                    distance_trucks.append(temp)
                    if i+1 == len(road_map_sa_truck_nodes)-1:
                        k = abs(list_trucks_new[road_map_sa_truck_nodes[i+1]][0]-list_trucks_new[road_map_sa_truck_nodes[0]][0]) + abs(list_trucks_new[road_map_sa_truck_nodes[i+1]][1]-list_trucks_new[road_map_sa_truck_nodes[0]][1])
                        
                        distance_trucks.append(k)
            else:
                indices_1 = []
                nueva_list_trucks_new = copy.deepcopy(list_trucks_new)
                nueva_road_map_sa_truck_nodes = road_map_sa_truck_nodes.copy()
                nueva_waiting_time_dr1 = waiting_time_dr1.copy()
                nueva_time_d1 = time_d1.copy()

                for i in range(len(road_map_sa_truck_nodes)):
                    if d1_tr_new[i] == 'NO LAUNCHED 2':
                        indices_1.append(i)
                for i in indices_1:
                    nueva_list_trucks_new.append(list_dr1_new[road_map_sa_truck_nodes[i]])
                    indice = nueva_list_trucks_new.index(list_dr1_new[road_map_sa_truck_nodes[i]])
                    nueva_road_map_sa_truck_nodes.append(indice)
                    list_dr1_new[road_map_sa_truck_nodes[i]] = []
                    nueva_waiting_time_dr1.append(0)
                    nueva_time_d1.append(0)
                
                waiting_drone = waiting_time_dr1 
                service_truck = [0.5 if i==0 else 0 for i in waiting_drone] 
                service_truck[0] = 0
                
                distance_trucks = []
                for i in range(len(nueva_road_map_sa_truck_nodes)-1):
                    temp = abs(nueva_list_trucks_new[nueva_road_map_sa_truck_nodes[i]][0]-nueva_list_trucks_new[nueva_road_map_sa_truck_nodes[i+1]][0]) + abs(nueva_list_trucks_new[nueva_road_map_sa_truck_nodes[i]][1]-nueva_list_trucks_new[nueva_road_map_sa_truck_nodes[i+1]][1])

                    distance_trucks.append(temp)
                    if i+1 == len(nueva_road_map_sa_truck_nodes)-1:
                        k = abs(nueva_list_trucks_new[nueva_road_map_sa_truck_nodes[i+1]][0]-nueva_list_trucks_new[nueva_road_map_sa_truck_nodes[0]][0]) + abs(nueva_list_trucks_new[nueva_road_map_sa_truck_nodes[i+1]][1]-nueva_list_trucks_new[nueva_road_map_sa_truck_nodes[0]][1])
                        
                        distance_trucks.append(k)

                road_map_sa_truck_nodes = nueva_road_map_sa_truck_nodes.copy()
                list_trucks_new = copy.deepcopy(nueva_list_trucks_new)

                del (indices_1,
                nueva_list_trucks_new,nueva_road_map_sa_truck_nodes,
                nueva_waiting_time_dr1,
                nueva_time_d1)


            tiempos.append(time.time() - inicio) 
                
            fact,t, tiemposArribo, truckRoute, droneRoute, espera = testeo_factibilidad.ordenar(list_trucks_new, road_map_sa_truck_nodes, vuelos, dr, name_file, instances_path, metrica = 'SI')

            if fact > 0:   
                coords2 = pd.read_csv(instancia).values.tolist()     
                indice_deposito2 = 0
                sa = SimAnneal(coords2, inicio, tiempoLimite, stopping_iter=100000000, idx_deposito = indice_deposito2) 
                sa.anneal()

                road_map_sa_only_trucks = sa.get_routes()
                distance_trucks2 = []
                for i in range(len(road_map_sa_only_trucks)-1):
                    temp2 = abs((points[road_map_sa_only_trucks[i]][0]-points[road_map_sa_only_trucks[i+1]][0])) + abs(points[road_map_sa_only_trucks[i]][1]-points[road_map_sa_only_trucks[i+1]][1]) 
                    distance_trucks2.append(temp2)
                    if i+1 == len(road_map_sa_only_trucks)-1:
                        k2 = abs((points[road_map_sa_only_trucks[i+1]][0]-points[road_map_sa_only_trucks[0]][0])) + abs(points[road_map_sa_only_trucks[i+1]][1]-points[road_map_sa_only_trucks[0]][1])
                        distance_trucks2.append(k2)
                speed_truck2 = 35
                travel_truck2  = [60*each_distance/speed_truck2 for each_distance in distance_trucks2]
                service_truck2 = [0.5 if i>0 else 0 for i in travel_truck2] 
                time_truck2 = [sum(y) for y in zip(travel_truck2,service_truck2)]
                tiemposArribo = np.cumsum(time_truck2)
                t = sum(time_truck2)

            if fact == 0:
                waiting_time = espera/ t
                drone_delivery = len(droneRoute) / (n - 1)
                conteo = 0
                for i in tiemposArribo:
                    if i != 0 and i != n and tiemposArribo[i] > 240: 
                        conteo += 1
                service_level = conteo / (n - 1)
            
            else:
                waiting_time = 0
                drone_delivery = 0
                conteo = 0
                for i in range(len(tiemposArribo)):
                    if i != 0 and i != len(tiemposArribo) - 1 and tiemposArribo[i] > 240:
                        conteo += 1
                service_level = conteo / (n - 1)

            if t < mejor_costo:
                mejor_costo = t
                mejor_drone_delivery = drone_delivery
                mejor_service_level = service_level
                mejor_waiting_time = waiting_time
            
        if os.path.exists(soluciones_folder + '/' +"trucks_new"+ '_' + name_file[:4] + '_' + str(n) + '_' + str(endurance) + '_' + str(speed_drone) + '_' + str(dr) + '_' + str(seed) +".csv"):
            os.remove(soluciones_folder + '/' +"trucks_new"+ '_' + name_file[:4] + '_' + str(n) + '_' + str(endurance) + '_' + str(speed_drone) + '_' + str(dr) + '_' + str(seed) +".csv")
    
        costos.append(mejor_costo)
        d_deliveries.append(mejor_drone_delivery)
        s_levels.append(mejor_service_level)
        w_times.append(mejor_waiting_time)

    return costos,tiempos, d_deliveries, s_levels, w_times
      

if __name__=="__main__":
    argv = sys.argv[1:]
    try:
        opts, args = getopt.getopt(argv,"a:")
        if len (opts) < 1:
            print('.\FDTSP_SA.py -a')
            sys.exit(2)
    except getopt.GetoptError:
        print('.\FDTSP_SA.py -a')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-a':
            inst = str(arg)

    instances_path = '../instancias'
    soluciones_folder = './Soluciones'

    file = open('.' + inst, "r")
    for i in file:
        instancia = i[:-1]
        costos,tiempos, d_deliveries, s_levels, w_times = main(instancia,instances_path,soluciones_folder)
        promedioCostos = sum(costos)/(len(costos))
        mininmoCostos = min(costos)
        promedioTiempos = sum(tiempos)/(len(tiempos))
        minimoTiempos = min(tiempos)

        prom_d_deliveries = sum(d_deliveries)/len(d_deliveries)
        min_d_deliveries = d_deliveries[costos.index(min(costos))]
        prom_s_levels = sum(s_levels)/len(s_levels)
        min_s_levels = s_levels[costos.index(min(costos))]
        prom_w_times = sum(w_times)/len(w_times)
        min_w_times = w_times[costos.index(min(costos))]

        print(inst, promedioCostos, mininmoCostos, promedioTiempos, prom_d_deliveries, min_d_deliveries, prom_s_levels, min_s_levels, prom_w_times, min_w_times)

    file.close()


