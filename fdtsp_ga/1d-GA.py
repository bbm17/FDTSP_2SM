import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
from operator import add
import math, time
import operator
import csv

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
    speed_truck = 35
    endurance1 = int(endurance)/60
    endurance2 = int(endurance)/60 

    epochs = 10
    
    nrSeeds = 10
    
    iter_proposed_model = 10000



    dr = 1





    epochs_ga = 1000
    chromosome = 50                                                           
    terminal_condition = 20                                                   
    mutation_miltiplier = 5  

    delta = 0.7
                                                     
    'FUNCTIONS'
    'BISECTING K-MEANS MODULE'
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
    def kmeans(points, k=2, epochs=10, max_iter=100, verbose=False):
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
                    valores = []
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
    def bisecting_kmeans(points, kk=2, epochs=10, max_iter=100, verbose=False):
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
    def get_road_map_from_a_point(road_map, start_index):
        index_of_start_point = road_map.index(start_index)
        corr_road_map = road_map[index_of_start_point:]
        corr_road_map.extend(road_map[:index_of_start_point])
        return corr_road_map
    def get_road_map_from_a_point(road_map2, start_index2):
        index_of_start_point2 = road_map2.index(start_index2)
        corr_road_map2 = road_map2[index_of_start_point2:]
        corr_road_map2.extend(road_map2[:index_of_start_point2])
        return corr_road_map2

    def visualize_function(data, corr_road_map):
        plt.figure(figsize=(10, 8))
        for i in range(len(corr_road_map)):
            plt.scatter(data.values[i][0], data.values[i][1],marker='s',label ='Truck Node',color='green')

        dist = 0
        for i in range(len(corr_road_map)-1):
            start, end = corr_road_map[i], corr_road_map[i+1]
            point1, point2 = data.values[start], data.values[end]
            distance = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
            dist += distance
            plt.arrow(point1[0], point1[1], 
                point2[0] - point1[0], point2[1] - point1[1],
                head_width=0.2,
                head_length=0.2,
                length_includes_head=True,
                color='red')
            if i+1 == len(corr_road_map)-1:
                start, end = corr_road_map[-1], corr_road_map[0]
                point1, point2 = data.values[start], data.values[end]
                distance = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
                dist += distance
                plt.arrow(point1[0], point1[1], 
                point2[0] - point1[0], point2[1] - point1[1],
                head_width=0.2,
                head_length=0.2,
                length_includes_head=True,
                color='red')
        plt.title('Shortest Truck Route after Clustering',fontsize=18)
        plt.xlabel('service area (mile)',fontsize=16)
        plt.ylabel('service area (mile)',fontsize=16)
        plt.legend(['Truck Node'])
        resolution_value = 1200
        plt.savefig("truck nodes.png", format="png", dpi=resolution_value)
        plt.show()  
    def visualize_function2(data2, corr_road_map2):
        plt.figure(figsize=(10, 8))
        for i in range(len(corr_road_map2)):
            plt.scatter(data2.values[i][0], data2.values[i][1],marker='o',label ='Customer Node',color='blue')

        dist = 0
        for i in range(len(corr_road_map2)-1):
            start, end = corr_road_map2[i], corr_road_map2[i+1]
            point1, point2 = data2.values[start], data2.values[end]
            distance = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
            dist += distance
            plt.arrow(point1[0], point1[1], 
                point2[0] - point1[0], point2[1] - point1[1],
                head_width=0.2,
                head_length=0.2,
                length_includes_head=True,
                color='red')
            if i+1 == len(corr_road_map2)-1:
                start, end = corr_road_map2[-1], corr_road_map2[0]
                point1, point2 = data2.values[start], data2.values[end]
                distance = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
                dist += distance
                plt.arrow(point1[0], point1[1], 
                point2[0] - point1[0], point2[1] - point1[1],
                head_width=0.2,
                head_length=0.2,
                length_includes_head=True,
                color='red')
        plt.title('Shortest Truck Route for all customer nodes (TSP-0D)',fontsize=18)
        plt.xlabel('service area (mile)',fontsize=16)
        plt.ylabel('service area (mile)',fontsize=16)
        plt.legend(['Customer Node'])
        resolution_value = 1200
        plt.savefig("Customer nodes.png", format="png", dpi=resolution_value)
        plt.show()  
    'FDTSP ALGORITHM BEGINS'

    costos = []
    tiempos = []
    w_times = []
    d_deliveries = []
    s_levels = []
    for seed in range(1, nrSeeds + 1):
        random.seed(seed)
        np.random.seed(seed)


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
                        epochs = 50
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
                else:
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
            data = pd.read_csv(soluciones_folder + '/' + "trucks_new"+ '_' + name_file[:4] + '_' + str(n) + '_' + str(endurance) + '_' + str(speed_drone) + '_' + str(dr) + '_' + str(seed) +".csv")                 
                                                


            def load_csv():                                                           

                data = pd.read_csv(soluciones_folder + '/' + "trucks_new"+ '_' + name_file[:4] + '_' + str(n) + '_' + str(endurance) + '_' + str(speed_drone) + '_' + str(dr) + '_' + str(seed) +".csv").values
                x, y = list(data[:, 0]), list(data[:, 1])
                return x, y
            def initialization(x,y, indice):
                ini_solution=[]    
                for i in range(chromosome):
                    t = random.sample(range(0, numberofcustomer), numberofcustomer) 
                    t.remove(indice)
                    t.insert(0,indice)
                    ini_solution.append(t)  
                return ini_solution
            def distance(solution):   
                totaldis = []
                for j in range(len(solution)):
                    totaldistance = 0
                    for i in range(numberofcustomer - 1):
                        k = solution[j][i]
                        w = solution[j][i+1]
                        temp = math.sqrt((((x[k]-x[w])**2) + ((y[k]-y[w])**2)))
                        totaldistance += temp
                        if i+1 == numberofcustomer-1:
                            g = solution[j][0]
                            totaldistance += math.sqrt((((x[k]-x[w])**2) + ((y[k]-y[w])**2)))
                    totaldis.append(totaldistance)
                return totaldis
            def crossover(parent):  
                offspring=[]
                for i in range(0, chromosome, 2):
                    alpha = random.random()
                    if alpha < crossover_rate:
                        temp_1, temp_2, temp_3, temp_4 = [], [], [], []
                        point1 = random.randint(0, numberofcustomer - 1)
                        point2 = random.randint(0, numberofcustomer - 1)
                        while (point2 == point1):
                            point2 = random.randint(0, numberofcustomer - 1)
                        if point1 > point2:
                            temp = point1
                            point1 = point2
                            point2 = temp
                        for k in range(point1,point2):
                            temp_1.append(parent[i][k])
                            temp_2.append(parent[i+1][k])
                        for j in range(numberofcustomer):                
                            if j in range(point1,point2):
                                temp_3.append(parent[i][j])
                            else:
                                for item in parent[i+1]:
                                    if item not in (temp_1 + temp_3):
                                        temp_3.append(item)
                                        break 
                        for j in range(numberofcustomer):                
                            if j in range(point1,point2):
                                temp_4.append(parent[i+1][j])
                            else:
                                for item in parent[i]:
                                    if item not in (temp_2 + temp_4):
                                        temp_4.append(item) 
                                        break                              
                        offspring.append(temp_3)
                        offspring.append(temp_4)                    
                    else:
                        offspring.append(parent[i])
                        offspring.append(parent[i+1])
                return offspring
            def mutation(parent):
                random.shuffle(parent)
                child = []
                for x in range(mutation_miltiplier):
                    for i in range(chromosome):
                        alpha = random.random() 
                        if alpha < mutation_rate:
                            draw1 = random.randint(1, numberofcustomer - 1)
                            draw2 = random.randint(1, numberofcustomer - 1)
                            while (draw2==draw1):
                                draw2 = random.randint(1, numberofcustomer - 1)
                            temp_7 = []
                            for j in range(numberofcustomer):
                                if j == draw1:
                                    temp_7.append(parent[i][draw2])
                                elif j == draw2:
                                    temp_7.append(parent[i][draw1])
                                else:
                                    temp_7.append(parent[i][j])
                            child.append(temp_7)
                        else:
                            child.append(parent[i])
                return child
            def comparsion(parent, child):
                mixparent = []
                for i in range(len(parent)):
                    mixparent.append(parent[i])
                for i in range(len(child)):    
                    mixparent.append(child[i])
                total_dis1 = distance(parent)
                total_dis2 = distance(child)
                total_dis=[]
                for i in range(len(parent)):
                    total_dis.append(total_dis1[i])
                for i in range(len(child)):  
                    total_dis.append(total_dis2[i])
                for i in range(len(parent) + len(child)):
                    for j in range(len(parent) + len(child) - 1):
                        if total_dis[j] >= total_dis[j+ 1]:
                            tem = total_dis[j]
                            total_dis[j] = total_dis[j+ 1]
                            total_dis[j+ 1] = tem
                            tem = mixparent[j]
                            mixparent[j] = mixparent[j+ 1]
                            mixparent[j + 1] = tem
                del mixparent[chromosome:]   
                return mixparent
            def ga_tsp(x, y, parent):
                total_iteration = 1000
                count = 0
                objective=[]
                answer=2000
                for iteration in range(total_iteration):
                    offspring = crossover(parent)
                    child = mutation(offspring)        
                    parent = comparsion(parent, child)
                    totaldis = distance(parent)
                    answer = min(totaldis)
                    z = totaldis.index(answer)
                    objective.append(answer)
                    if iteration != 0:       
                        if objective[iteration-1] == objective[iteration]:
                            count +=1
                        else:
                            count = 0
                    if count == terminal_condition:         
                        break
                return answer, parent[z]
            numberofcustomer = len(trucks_new) 

            x, y = load_csv()
            coords1 = [[x[i], y[i]] for i in range(len(x))]
            for l in range(len(coords1)):
                if round(coords1[l][0],3) == round(points[0].tolist()[0],3) and round(coords1[l][1],3) == round(points[0].tolist()[1],3):
                    indice_deposito = l
                    break

            best = 6000
            tracing_list = []
            for i in range(epochs_ga):
                crossover_rate = 0.9
                mutation_rate = 1

                exparent = initialization(x, y, indice_deposito)

                answer, parent = ga_tsp(x, y, exparent)
                tracing_list.append(parent)

                if answer < best: 
                    best = answer
                    best_round = i+1 

                if time.time() - inicio >= tiempoLimite:
                    break

            road_map = tracing_list[best_round-1]



            list_trucks_new = trucks_new.copy().tolist()

            vuelos = {}
            for i in list_dr1_new:
                if i != []:
                    vuelos[tuple(i)] = []

            d1_tr = []
            for i in range(len(road_map)):
                if list_dr1_new[road_map[i]]!=[]:
                    tempo = math.sqrt((list_trucks_new[road_map[i]][0]-list_dr1_new[road_map[i]][0])**2+(list_trucks_new[road_map[i]][1]-list_dr1_new[road_map[i]][1])**2)
                    d1_tr.append(tempo)

                    for j in vuelos:
                        if j[0] == list_dr1_new[road_map[i]][0] and j[1] == list_dr1_new[road_map[i]][1]:
                            vuelos[j].append(list_trucks_new[road_map[i]])
                            break
                else:
                    d1_tr.append(0)

            d1_ntr = []
            for i in range(len(road_map)-1):
                if list_dr1_new[road_map[i]]!=[]:
                    tempo11 = math.sqrt((list_dr1_new[road_map[i]][0]-list_trucks_new[road_map[i+1]][0])**2+(list_dr1_new[road_map[i]][1]-list_trucks_new[road_map[i+1]][1])**2)
                    d1_ntr.append(tempo11) 

                    for j in vuelos:
                        if j[0] == list_dr1_new[road_map[i]][0] and j[1] == list_dr1_new[road_map[i]][1]:
                            vuelos[j].append(list_trucks_new[road_map[i+1]])
                            break
                else:
                    d1_ntr.append(0)
            for i in range(len(road_map)-1,len(road_map)):
                if list_dr1_new[road_map[i]]!=[]:
                    tempo101 = math.sqrt((list_dr1_new[road_map[i]][0]-list_trucks_new[road_map[0]][0])**2+(list_dr1_new[road_map[i]][1]-list_trucks_new[road_map[0]][1])**2)
                    d1_ntr.append(tempo101)

                    for j in vuelos:
                        if j[0] == list_dr1_new[road_map[i]][0] and j[1] == list_dr1_new[road_map[i]][1]:
                            vuelos[j].append(list_trucks_new[road_map[0]])
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
            
            for i in range(len(road_map)):
                if list_dr1_new[road_map[i]]!=[] and d1_tr_new[i] == 'COMEBACK':

                    for j in vuelos: 
                        if j[0] == list_dr1_new[road_map[i]][0] and j[1] == list_dr1_new[road_map[i]][1]:

                            vuelos[j][1] = list_trucks_new[road_map[i]]

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
                for i in range(len(road_map)-1):

                    temp = abs(list_trucks_new[road_map[i]][0]-list_trucks_new[road_map[i+1]][0]) + abs(list_trucks_new[road_map[i]][1]-list_trucks_new[road_map[i+1]][1])
                    temp = abs((list_trucks_new[road_map[i]][0]-list_trucks_new[road_map[i+1]][0])) + abs(list_trucks_new[road_map[i]][1]-list_trucks_new[road_map[i+1]][1])

                    distance_trucks.append(temp)
                    if i+1 == len(road_map)-1:

                        k = abs(list_trucks_new[road_map[i+1]][0]-list_trucks_new[road_map[0]][0]) + abs(list_trucks_new[road_map[i+1]][1]-list_trucks_new[road_map[0]][1])

                        distance_trucks.append(k)
            else:
                indices_1 = []
                nueva_list_trucks_new = copy.deepcopy(list_trucks_new)
                nueva_road_map = road_map.copy()
                nueva_waiting_time_dr1 = waiting_time_dr1.copy()
                nueva_time_d1 = time_d1.copy()

                for i in range(len(road_map)):
                    if d1_tr_new[i] == 'NO LAUNCHED 2':
                        indices_1.append(i)
                for i in indices_1:
                    nueva_list_trucks_new.append(list_dr1_new[road_map[i]])
                    indice = nueva_list_trucks_new.index(list_dr1_new[road_map[i]])
                    nueva_road_map.append(indice)
                    list_dr1_new[road_map[i]] = []
                    nueva_waiting_time_dr1.append(0)
                    nueva_time_d1.append(0)
                
                waiting_drone = [max(nueva_waiting_time_dr1[i]) for i in range(len(nueva_waiting_time_dr1))]

                service_truck = [0.5 if i==0 else 0 for i in waiting_drone]
                service_truck[0] = 0
                

                distance_trucks = []
                for i in range(len(nueva_road_map)-1):

                    temp = abs(nueva_list_trucks_new[nueva_road_map[i]][0]-nueva_list_trucks_new[nueva_road_map[i+1]][0]) + abs(nueva_list_trucks_new[nueva_road_map[i]][1]-nueva_list_trucks_new[nueva_road_map[i+1]][1])

                    distance_trucks.append(temp)
                    if i+1 == len(nueva_road_map)-1:

                        k = abs(nueva_list_trucks_new[nueva_road_map[i+1]][0]-nueva_list_trucks_new[nueva_road_map[0]][0]) + abs(nueva_list_trucks_new[nueva_road_map[i+1]][1]-nueva_list_trucks_new[nueva_road_map[0]][1])

                        distance_trucks.append(k)
                
                road_map = nueva_road_map.copy()
                list_trucks_new = copy.deepcopy(nueva_list_trucks_new)

                del (indices_1,
                nueva_list_trucks_new,nueva_road_map,
                nueva_waiting_time_dr1,
                nueva_time_d1)








            tiempos.append(time.time() - inicio)
                

            fact,t, tiemposArribo, truckRoute, droneRoute, espera = testeo.ordenar(list_trucks_new, road_map, vuelos, dr, name_file, instances_path, metrica = 'SI')

            if fact > 0:





                x,y = [],[]
                for nodo in points:
                    x.append(nodo[0]), y.append(nodo[1])
                
                indice_deposito = 0
                numberofcustomer = len(x)
                best = 6000
                tracing_list = []
                epochs_ga_2 = 10
                for i in range(epochs_ga_2):
                    crossover_rate = 0.9
                    mutation_rate = 1
                    exparent = initialization(x, y, indice_deposito)
                    answer, parent = ga_tsp(x, y, exparent)
                    tracing_list.append(parent)
                    if answer < best: 
                        best = answer
                        best_round = i+1                     

                road_map = tracing_list[best_round-1]
            
                distance_trucks2 = []
                for i in range(len(road_map)-1):
                    temp2 = abs((points[road_map[i]][0]-points[road_map[i+1]][0])) + abs(points[road_map[i]][1]-points[road_map[i+1]][1])
                    distance_trucks2.append(temp2)
                    if i+1 == len(road_map)-1:
                        k2 = abs((points[road_map[i+1]][0]-points[road_map[0]][0])) + abs(points[road_map[i+1]][1]-points[road_map[0]][1])
                        distance_trucks2.append(k2)
                speed_truck2 = 35
                travel_truck2  = [60*each_distance/speed_truck2 for each_distance in distance_trucks2]
                service_truck2 = [0.5 if i>0 else 0 for i in travel_truck2] 
                time_truck2 = [sum(y) for y in zip(travel_truck2,service_truck2)]
                tiemposArribo = np.cumsum(time_truck2)

                totaltime_only_truck = sum(time_truck2)


                t = totaltime_only_truck





            
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

        if os.path.exists(soluciones_folder + '/' + "trucks_new"+ '_' + name_file[:4] + '_' + str(n) + '_' + str(endurance) + '_' + str(speed_drone) + '_' + str(dr) + '_' + str(seed) +".csv"):
            os.remove(soluciones_folder + '/' + "trucks_new"+ '_' + name_file[:4] + '_' + str(n) + '_' + str(endurance) + '_' + str(speed_drone) + '_' + str(dr) + '_' + str(seed) +".csv")
        
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
            print('.\FDTSP_GA.py -a')
            sys.exit(2)
    except getopt.GetoptError:
        print('.\FDTSP_GA.py -a')
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