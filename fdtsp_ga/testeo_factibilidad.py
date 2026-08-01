import math
import pandas as pd

global nrNodos, endurance, droneSpeed, truckSpeed, coords, se, serv_t, serv_d, nrDrones, time_t, time_d, grid, seed

def ordenar(t, t_r, vuelos, ndrones, file, instances_path, metrica = ''):

    def dataReading(name_file, instances_path):
        instance = instances_path + '/' + name_file

        file = pd.read_csv(instance)

        string = ''
        param =[]
        for l in name_file:
            if l == '_':
                param.append(string)
                string = ''
            elif l == '.':
                param.append(string)
                string = ''
            else:
                string += l

        n = int(param[1]) # Nr de nodos
        endurance = int(param[2])  # Battery endurance of a drone
        droneSpeed = int(param[3]) 
        truckSpeed = 35 #km/hour

        positions = []
        for i in range(n):
            aux =[]
            x = file['X'][i]
            y = file['Y'][i]
            aux.append(x), aux.append(y)
            positions.append(aux)
        coords= {}
        for i in range(len(positions)):
            coords[i] = positions[i]
        
        coords[n] = coords[0] # Duplicación del deposito

        return n, endurance, droneSpeed, truckSpeed, coords

    def tiemposViaje():
        time_t = list()
        time_d = list()
        nodos = [i for i in coords]

        for i in nodos:
            lista_aux_t = list()
            lista_aux_d = list()
            for j in nodos:
                if i != nodos[-1] and j != 0 and i != j:
                    if i == 0 and j == nodos[-1]:
                        lista_aux_t.append(False)
                        lista_aux_d.append(False)
                    else:
                        lista_aux_t.append( (( abs(float(coords[i][0]) - float(coords[j][0])) 
                                            + abs(float(coords[i][1]) - float(coords[j][1])) )/truckSpeed)*60 ) # min
                        lista_aux_d.append( ((math.sqrt((float(coords[i][0]) - float(coords[j][0]))**2 
                                                    + (float(coords[i][1]) - float(coords[j][1]))**2))/droneSpeed)*60 ) # min
                else: 
                    lista_aux_t.append(False)
                    lista_aux_d.append(False)

            time_t.append(lista_aux_t)
            time_d.append(lista_aux_d)
            
        return time_t, time_d

    def factibilidad_func(truckRoute, droneRoute='', tiemposArribo = ''): # Se evaluan distintos tipos de infactibilidad. Se pueden identificar cada uno, pero se penaliza de la misma forma para cada tipo.
        if type(droneRoute) != str:
                
            conteoL = {i:0 for i in truckRoute} # Conteo de lanzamientos por nodo
            conteoR = {i:0 for i in truckRoute} # Conteo de recepciones por nodo

            indices = {truckRoute[i]: i for i in range(len(truckRoute))}

            infactibilidades = 0
            for route in droneRoute:
                conteoL[route[0]] += 1
                conteoR[route[2]] += 1

                if indices[route[2]] < indices[route[0]]: # La ruta del dron debe seguir el sentido de la ruta del camión.
                    infactibilidades += 1
                    # print('tipo 1')
                
                # Verificando duranción de los vuelos de los drones, considerando tiempos de espera.
                if route[2] != truckRoute[-1]:
                    
                    espera = 0 # Tiempo de espera de dron
                    se = 0
                    for vuelo in droneRoute:
                        if vuelo[0] == route[0]:
                            se +=1
                    if route[0] == 0:
                        if tiemposArribo[route[0]] + se + time_d[route[0]][route[1]] + serv_d + time_d[route[1]][route[2]] < tiemposArribo[route[2]]: # El camión llega después
                            espera += tiemposArribo[route[2]] - (tiemposArribo[route[0]] + se + time_d[route[0]][route[1]] + serv_d + time_d[route[1]][route[2]])

                    else:
                        if tiemposArribo[route[0]] + se + serv_t + time_d[route[0]][route[1]] + serv_d + time_d[route[1]][route[2]] < tiemposArribo[route[2]]: # El camión llega después
                            espera += tiemposArribo[route[2]] - (tiemposArribo[route[0]] + se + serv_t + time_d[route[0]][route[1]] + serv_d + time_d[route[1]][route[2]])

                    if time_d[route[0]][route[1]] + serv_d + time_d[route[1]][route[2]] + espera > endurance:  #El vuelo del dron es factible por el tiempo de la distancia recorrida, sin considerar espera
                            infactibilidades += 1

                    elif time_d[route[0]][route[1]] + serv_d + time_d[route[1]][route[2]] > endurance: # El vuelo del dron, sin considerar la espera, es infactible.
                        infactibilidades += 1
                 
            free_drones = nrDrones
            for i in truckRoute:

                if nrDrones < conteoL[i]: # Existen más lanzamientos en un nodo que la cantidad de drones.
                    infactibilidades += 1
                    
                if nrDrones < conteoR[i]: # Existen más recepciones en un nodo que la cantidad de drones.
                    infactibilidades += 1

                # Verificando balance de drones volando.
                if conteoL[i] > conteoR[i]:
                    cont = 0 # En caso de que exista un vuelo <i,j,i> y el balance (lanzaminetos-recepciones) es (+), se consideran estos vuelos solo como lanzamientos.
                    for j in droneRoute:
                        if j[0] == i and j[2] == i:
                            cont += 1
                    free_drones -= (conteoL[i] - conteoR[i] + cont)
                
                elif conteoL[i] < conteoR[i]:
                    free_drones +=  conteoR[i] - conteoL[i]
                
                if free_drones < 0: # Hay un desbalance en la cantidad de drones volando.
                    infactibilidades += 1
                        
        else:
            infactibilidades = 0

        # infactibilidades 0 => Es factible
        return infactibilidades 

    def tiemposArribo_func(truckRoute, droneRoute='', tpoArribo = '', metrica_t = ''):
        if metrica_t == 'SI':
            waiting_time = 0
        # Tiempos de arribo solo para ruta del camion (TSP)
        t = 0
        tpoArribo[0] = t
        for i in range(len(truckRoute) - 1):
            t = time_t[truckRoute[i]][truckRoute[i + 1]]
            
            if i == 0:
                tpoArribo[truckRoute[i + 1]] = tpoArribo[truckRoute[i]] + t
            else: 
                tpoArribo[truckRoute[i + 1]] = tpoArribo[truckRoute[i]] + t + serv_t


        if type(droneRoute) != str and len(droneRoute) != 0:

            for i in truckRoute:
                contador_lanzamientos = 0
                tpoRetorno = -1

                for j in droneRoute:
                    if i == j[0]:
                        contador_lanzamientos += 1
                        if i == j[2]:
                            if tpoRetorno < time_d[j[0]][j[1]] + serv_d + time_d[j[1]][j[2]]:
                                tpoRetorno = time_d[j[0]][ j[1]] + serv_d + time_d[j[1]][j[2]] # Finalmente se almacena el viaje loop más largo.
                
                # Actualización del tiempo de arribo de los siguientes nodos visitados por el camióm, si es que existe un vuelo tipo "loop"
                if tpoRetorno != -1:
                    if metrica_t == 'SI':
                        waiting_time += tpoRetorno
                    index_l = truckRoute.index(i) # Índice del nodo de lanzamiento 
                    for k in range(int(index_l) + 1, len(truckRoute)):
                        tpoArribo[truckRoute[k]] += tpoRetorno

                # Actualización del tiempo de arribo de los nodos visitados por los drones y nodos siguientes del camión
                for j in droneRoute:
                    if i == j[0]:
                        # Actualizacion del arribo de los nodos visitados por el camión
                        index_l = truckRoute.index(i) # índice del lanzamiento

                        for k in range(int(index_l) + 1, len(truckRoute)):
                            tpoArribo[truckRoute[k]] += se # Se suma el tiempo de set up a los arribo de los nodos que son proximamente visitados

                        # Actualizacion del arribo del nodo visitado por el dron    
                        tpoActual = tpoArribo[i]
                        if j[0] == 0:
                            tpoActual += time_d[j[0]][j[1]] + (se * contador_lanzamientos)
                        elif j[0] != 0:
                            tpoActual += time_d[j[0]][j[1]] + (se * contador_lanzamientos) + serv_t 
                        tpoArribo[j[1]] = tpoActual
                    
                    elif i != j[0] and i == j[2] and i != truckRoute[-1]:
                            tpoActual = tpoArribo[j[1]] + serv_d + time_d[j[1]][j[2]]

                            if tpoActual > tpoArribo[j[2]]: # El camión debe esperar al dron
                                # Actualizacion de los arribos del camión, según la espera al dron. 
                                espera = tpoActual - tpoArribo[j[2]]
                                if metrica_t == 'SI':
                                    waiting_time += espera
                                index_r = truckRoute.index(j[2]) #Índice del nodo de recepción.
                                for k in range(int(index_r), len(truckRoute)):
                                    tpoArribo[truckRoute[k]] += espera
                                    for n in droneRoute:
                                        if n[0] == truckRoute[k] and n[1] in tpoArribo:
                                            tpoArribo[n[1]] += espera
        # return tpoArribo
        if metrica_t == "SI":
            return waiting_time
    
    ####
    nrNodos, endurance, droneSpeed, truckSpeed, coords = dataReading(file, instances_path)
    time_t, time_d = tiemposViaje()

    se = 1 # min
    serv_t = 0.5 # min
    serv_d = 0.5 # min
    nrDrones = ndrones

    ruta_ordenada = [t[i] for i in t_r]

    truckRoute = []
    for j in ruta_ordenada:
        for i in coords:
            if round(coords[i][0],3) == round(j[0],3) and round(coords[i][1],3) == round(j[1],3):
                truckRoute.append(i)
                break

    truckRoute.append(nrNodos)

    droneRoute = []
    for j in vuelos:
        r = []
        for i in coords:
            if round(coords[i][0],3) == round(j[0],3) and round(coords[i][1],3) == round(j[1],3):
                r.append(i)
                break
        c = 0
        for k in vuelos[j]:
            for i in coords:
                if round(coords[i][0],3) == round(k[0],3) and round(coords[i][1],3) == round(k[1],3):
                    if c == 0:
                        r.insert(0,i)
                    elif c == 1:
                        if i != 0:
                            r.append(i)
                        else:
                            r.append(nrNodos)
                    break
            c += 1
        droneRoute.append(r)

    tiempos = {}
    if metrica == 'SI':
        espera = tiemposArribo_func(truckRoute, droneRoute, tpoArribo= tiempos, metrica_t = metrica)
    else:
        tiemposArribo_func(truckRoute, droneRoute, tpoArribo= tiempos)
    fact = factibilidad_func(truckRoute, droneRoute, tiemposArribo= tiempos)

    if metrica == '':
        return fact, tiempos[truckRoute[-1]]
    else:
        return fact, tiempos[truckRoute[-1]], tiempos, truckRoute, droneRoute, espera
