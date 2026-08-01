import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import math
import sys, getopt
import time

import gurobipy as gp
from gurobipy import GRB

def param(instances_path, name_file):
    instancia = instances_path + '/' + name_file
    
    cadena = ''
    datos =[]
    for letra in name_file:
        if letra == '_':
            datos.append(cadena)
            cadena = ''
        elif letra == '.':
            datos.append(cadena)
            cadena = ''
        else:
            cadena += letra

    nrNodes = int(datos[1])

    battery = int(datos[2]) 
    droneSpeed = int(datos[3]) 

    archivo = pd.read_csv(instancia)
    positions = []
    for i in range(nrNodes): 
        aux = []
        x = archivo['X'][i]
        y = archivo['Y'][i]
        aux.append(x), aux.append(y)
        positions.append(aux) 
    positions = np.array(positions)

    return positions, nrNodes, battery, droneSpeed

def lecturaDatos(name_file, instances_path):
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

def modelo_FDTSP(tiempoLimite, seed, m):
    np.random.seed(seed)

    V = [i for i in coords]

    N = V.copy() #Lista con solo los clientes
    N.pop(0)
    N.pop(-1)

    N_0 = V.copy() #Lista con solo los clientes y deposito inicial
    N_0.pop(-1)
    
    N_N = V.copy() #Lista con solo los clientes y deposito final
    N_N.pop(0)

    Arcs = [(i,j) for i in N_0 for j in N_N if j!=i]
    Arcs.remove((0 , V[-1]))

    L = [(i,j,k) for i in N_0 for j in N for k in N_N if i != j and k != j]
    for j in N:
        L.remove((0,j,V[-1]))

    se = 1          # Setup for launching/retreival a drone
    serv_t = 0.5    # Truck Service Time
    serv_d = 0.5    # Dron Service Time

    # Big-M
    M = 0
    for i in N_0: 
        max_x = 0
        max_y = 0
        for j in N_N:
            if j!=i:
                if i == 0 and j ==  V[-1]:
                    continue
                else:
                    if time_t[i][j] > max_x: 
                        max_x = time_t[i][j]
                    if time_d[i][j] > max_y:
                        max_y = time_d[i][j]
        M += max_x + max_y
    
    ###### Modelo matemático ######
    with gp.Env(empty=True) as env:
        env.setParam('OutputFlag', 0)
        env.start()
        with gp.Model("FDTSP",env=env) as mdl:

            mdl.setParam("TimeLimit", tiempoLimite)

            mdl.Params.Threads = 1

            ###### Variables ######
            x = mdl.addVars([(i,j) for i,j in Arcs], vtype= GRB.BINARY, name = 'x')
            y = mdl.addVars([i for i in N], vtype= GRB.BINARY, name = 'y')
            z = mdl.addVars([(i,j,k) for i,j,k in L], vtype= GRB.BINARY, name = 'z')
            w = mdl.addVars([i for i in V], vtype = GRB.INTEGER, name = 'w')
            a = mdl.addVars([i for i in V], vtype = GRB.CONTINUOUS, name = 'a')
    
            ###### Función objetivo ###### 
            mdl.setObjective(a[V[-1]], GRB.MINIMIZE)
            
            ###### Restricciones de ruteo  ###### 
            # Sujeto a 
            exprs_x_1 = {}
            for i in N:
                expr = gp.LinExpr()
                for j in N_N:
                    if j != i:
                        expr.addTerms(1, x[i,j])
                exprs_x_1[i] = expr

            # (1) Flow conservation constraints for the truck 
            for i in N:
                mdl.addConstr(gp.quicksum(x[j,i] for j in N_0 if j != i) == exprs_x_1[i], 'Ruteo_1_%i' %i)

            # (2) Link the arc varibales with the y-variables 
            for i in N:
                mdl.addConstr(exprs_x_1[i] == y[i], 'Ruteo_2')
            
            # (3) Ensure that the truck leaves from and returns to the depot 
            mdl.addConstr(gp.quicksum(x[0, j] for j in N ) == 1, 'Ruteo_3_1')

            # (4) 
            mdl.addConstr(gp.quicksum(x[i, V[-1]] for i in N) == 1, 'Ruteo_3_2')
            
            # (5) Guarantee that each customer is served exactly once 
            for j in N:
                mdl.addConstr(y[j] + gp.quicksum(z[i,j,k] for i in N_0 for k in N if i != j and k != j and k != i) + gp.quicksum(z[i,j,V[-1]] for i in N if i != j) + gp.quicksum(z[i,j,i] for i in N if i != j) == 1, 'Ruteo_4')
            
            # (6) Limita el numero de drones lanzados desde un nodo visitado por el camión
            for i in N:
                mdl.addConstr(y[i]*m >= gp.quicksum(z[i,j,k] for j in N for k in N_N if i != j and k != j and k != i) + gp.quicksum(z[i,j,i] for j in N if i != j), 'Ruteo_5') #Lanzamineto

            expr_w_1 = gp.LinExpr()
            for j in N:
                for k in N:
                    if k != j:
                        expr_w_1.addTerms(1, z[0,j,k])

            # (7) Limita el numero de drones lanzados desde el depósito
            mdl.addConstr(m >= expr_w_1, 'Ruteo_6')

            # (8) Limita la existencia de lanzamientos y recepciones, según la presencia del camión
            for i in N:
                mdl.addConstr(y[i]*M >= gp.quicksum(z[i,j,k] for j in N for k in N_N if i != j and k != j) + gp.quicksum(z[k,j,i] for k in N_0 for j in N if k != j and i != j), 'Ruteo_7')
                
            # (9) Sets w_0 equal to the number of airborne drones upon leaving 0. # w_0 = lanzaminetos desde 0, no se consideran recepciones en N 
            mdl.addConstr(w[0] == expr_w_1, 'Ruteo_8')

            # (10) Sets w_0' equal to zero to ensure that all drones rejoins the truck upon returning to the depot
            mdl.addConstr(w[V[-1]] == 0, 'Ruteo_9') # Drones volando en N = 0

            # (11) Guarantee that the number of airborne drones upon leaving each location visited by the truck does not exceed the number of available drones 
            mdl.addConstr(w[0] <= m*gp.quicksum(x[0,j] for j in N), 'Ruteo_11') # Explicacion a presencia de x[i,j], es que el nodo i servido por el dron no es visitado por el camion, por ende no pueden despegar drones desde i

            # (12) 
            for i in N:
                mdl.addConstr(w[i] <= m*gp.quicksum(x[i,j] for j in N_N if j != i), 'Ruteo_12')

            exprs_w_2 = {}
            for j in N:
                expr = gp.LinExpr()
                for r in N:
                    for s in N_N:
                        if j != r and s != r and s != j:
                            expr.addTerms(1, z[j,r,s])
                exprs_w_2[j] = expr

            # (13) Allow to correctly set the values of the w-variables # volando en 0 - lanzados en 0 + lanzados en j <= volando en j  
            for j in N:
                mdl.addConstr(w[0] - gp.quicksum(z[0,r,j] for r in N if j != r) + exprs_w_2[j]
                            <= w[j] + M*(1 - x[0,j]), 'Ruteo_13')

            # (14) Allow to correctly set the values of the w-variables # volando en i - recibidos en j + lanzados en j <= volando en j ###
            for i in N:
                for j in N:
                    if j != i:
                        mdl.addConstr(w[i] - gp.quicksum(z[s,r,j] for s in N_0 for r in N if s != r and j != r and s != j) + exprs_w_2[j]
                            <= w[j] + M*(1 - x[i,j]), 'Ruteo_14')
                        
            # (15) # volando en i - recibidos en N <= 0 
            for i in N:
                mdl.addConstr(w[i] - gp.quicksum(z[s,r,V[-1]] for s in N for r in N if s != r)
                            <= w[V[-1]] + M*(1 - x[i,V[-1]]), 'Ruteo_15')
                
            # (16) Si lo que está volando saliendo de i - lo que aterriza en j es es menor que 0 (excede el nr de drones), no se pude lanzar en j. Esta restriccion limita tambien los viajes <i,j,i> ###
            for i in N_0:
                for j in N:
                    if i != j:
                        mdl.addConstr(gp.quicksum(z[(j,r,s)] for r in N for s in N if j != r and s != r ) <= m - (w[i] - gp.quicksum(z[(s,r,j)] for s in N_0 for r in N if s != r and j != r and s != j)) + M*(1 - x[i,j]), 'Ruteo_16')
                        
            # # Time restrictions    
            # (17) Arrival at the depot 0 
            mdl.addConstr(a[0] == 0, 'Tiempo_17')

            expr_a_1 = gp.LinExpr()
            for r in N:
                for s in N:
                    if s != r:
                        expr_a_1.addTerms(se, z[0,r,s])

            # (18)    
            for j in N:
                mdl.addConstr(a[0] 
                + (M + time_t[0][j])*x[0,j] #+ service_t[0]
                + expr_a_1
                <= a[j] + M, 'Tiempo_18')
            # (19)
            for j in N:
                mdl.addConstr(a[0]
                + gp.quicksum( (M + time_d[0][j])*z[0,j,k] for k in N if k != j) # k not in N_N 
                + expr_a_1
                <= a[j] + M, 'Tiempo_19')
            # (20)
            for j in N:
                for l in N:
                    if l != j:
                        mdl.addConstr(a[0]
                        + expr_a_1
                        + (M + time_d[0][l] + serv_d + time_d[l][j])*z[0,l,j]
                        <= a[j] + M, 'Tiempo_20')

            exprs_a_2 = {}
            for i in N:
                expr = gp.LinExpr()
                for r in N:
                    for s in N_N:
                        if i != r and s != r:
                            expr.addTerms(se, z[i,r,s])
                exprs_a_2[i] = expr

            # (21) 
            for i in N:
                for j in N:
                    if j != i:
                        for l in N:
                            if l != i:
                                mdl.addConstr(a[i] 
                                + (M + time_t[i][j] + serv_t)*x[i,j]
                                + exprs_a_2[i]
                                + (time_d[i][l] + serv_d + time_d[l][i])*z[i,l,i] # VUELO <i,k,i>: No puede ser una sumatoria, se utiliza el viaje con el tiempo más largo
                                <= a[j] + M, 'Tiempo_21')

            # (22)
            for i in N:
                for j in N:
                    if j != i:
                        mdl.addConstr(a[i] 
                        + exprs_a_2[i]
                        + gp.quicksum( (M + serv_t + time_d[i][j])*z[i,j,k] for k in N_N if k != j) 
                        <= a[j] + M, 'Tiempo_22')

            # (23)
            for i in N:
                for j in N:
                    for l in N:
                        if j != i and i != l and j != l:
                            mdl.addConstr(a[i]
                            + serv_t
                            + exprs_a_2[i]
                            + (M + time_d[i][l] + serv_d + time_d[l][j] )*z[i,l,j]
                            <= a[j] + M, 'Tiempo_23')

            # (24) 
            for i in N: 
                for j in N:
                    if j != i:
                        mdl.addConstr(a[i] 
                        + (M + time_t[i][V[-1]] + serv_t)*x[i,V[-1]] 
                        + exprs_a_2[i]
                        + (time_d[i][j] + serv_d + time_d[j][i])*z[i,j,i] # VUELO <i,k,i>: No puede ser una sumatoria, se utiliza el viaje con el tiempo más largo
                        <= a[V[-1]] + M, 'Tiempo_24')    

            # Drone Battery Endurance 
            # (25)
            for j in N:
                for k in N:
                    if k != j:
                        mdl.addConstr(endurance >= a[k] - a[0] - expr_a_1 - (1 - z[0,j,k])*M, 'Bateria_26')

            # (26)
            for i in N:
                for j in N:
                    for k in N:
                        if i != j and k != j and k != i:
                            mdl.addConstr(endurance >= a[k] - a[i] - exprs_a_2[i] - serv_t - (1 - z[i,j,k])*M, 'Bateria_25')
                            
            # (27)
            for i in N:
                for j in N:
                    if i != j:
                        mdl.addConstr(endurance >= time_d[i][j] + serv_d + time_d[j][V[-1]] - (1 - z[i,j,V[-1]])*M, 'Bateria_27')

            # (28)
            for i in N:
                for j in N:
                    if i != j:
                        mdl.addConstr(endurance >= time_d[i][j] + serv_d + time_d[j][i] - (1 - z[i,j,i])*M, 'Bateria_28')

            mdl.optimize()
            try:
                obj = mdl.ObjVal
            except AttributeError:
                obj = math.inf

            runTime = mdl.Runtime
            best_Bd = mdl.ObjBoundC
            gap = mdl.MIPGap

            # print('ObjVal mdl= ', obj)
            # print('BEST BOUND =',best_Bd)
            # print('MIP GAP =', gap)
            # print('Runtime mdl= ', runTime)

            if obj != math.inf: 
                fact = 0
            
            else:
                fact = -1

    
    # plot_solution(V, coords, time_t, time_d, sol_x, sol_z)

    return obj, best_Bd, gap, runTime, fact

def modelo_FDTSP_reducido(tiempoLimite, seed, coords = '' ,time_t = '', time_d = '', truckRoute = '', m = '', endurance = '', se = '', serv_t = '', serv_d = '', instances_path = '', name_file = '', tiemposTSP = '', droneRoute = ''):
    np.random.seed(seed)
    ini_carga = time.time()

    if type(coords) == str:
        positions, nrNodes, battery, droneSpeed = param(instances_path, name_file)

        coords= {}
        for i in range(len(positions)):
            coords[i] = positions[i]

        V = [i for i in coords]

        V= V.copy()
        V.append(int(V[-1])+1)

        N = V.copy() #Lista con solo los clientes
        N.pop(0)
        N.pop(-1)

        N_0 = V.copy() #Lista con solo los clientes y deposito inicial
        N_0.pop(-1)
        
        N_N = V.copy() #Lista con solo los clientes y deposito final
        N_N.pop(0)
        
        coords=coords.copy()
        coords[V[-1]] = positions[0]

        Arcs = [(i,j) for i in N_0 for j in N_N if j!=i]
        Arcs.remove((0 , V[-1]))

        L = [(i,j,k) for i in N_0 for j in N for k in N_N if i != j and k != j]
        for j in N:
            L.remove((0,j,V[-1]))

        truck_speed = 35 #mph
        drone_speed= droneSpeed #mph

        #Tiempo viaje del camion, minutos
        time_t = {(i,j): ((math.sqrt( (float(coords[i][0]) - float(coords[j][0]))**2 +
                                            (float(coords[i][1]) - float(coords[j][1]))**2))/truck_speed)*60 for i,j in Arcs if i!= V[-1] and j != V[-1]}
        for i in V:
            if i != V[0] and i != V[-1]:
                time_t[i][ V[-1]] = time_t[0][i]
        
        #Tiempo viaje del dron, minutos
        time_d = {(i,j): ((math.sqrt( (float(coords[i][0]) - float(coords[j][0]))**2 +
                                            (float(coords[i][1]) - float(coords[j][1]))**2))/ drone_speed)*60 for i,j in Arcs if i!= V[-1] and j != V[-1]}
        for i in V:
            if i != V[0] and i != V[-1]:
                time_d[i][ V[-1]] = time_d[0][i] # Por simetria

        se = 1 # Setup for launching/retreival a drone
        endurance = battery  # 30 , 60 # min # Battery endurance of a drone
        serv_t = 0.5
        serv_d = 0.5

    else: # Para usar dentro del VNS (Matheuristic). En caso de que se ingresen parametros desde el algoritmo.
        V = [i for i in coords]
        N = V.copy() #Lista con solo los clientes
        N.pop(0)
        N.pop(-1)

        N_0 = V.copy() #Lista con solo los clientes y deposito inicial
        N_0.pop(-1)
        
        N_N = V.copy() #Lista con solo los clientes y deposito final
        N_N.pop(0)

        Arcs = [(i,j) for i in N_0 for j in N_N if j!=i]
        Arcs.remove((0 , V[-1]))

        L = [(i,j,k) for i in N_0 for j in N for k in N_N if i != j and k != j]
        for j in N:
            L.remove((0,j,V[-1]))               

    # Big-M
    M = 0
    for i in N_0: 
        max_x = 0
        max_y = 0
        for j in N_N:
            if j!=i:
                if i == 0 and j ==  V[-1]:
                    continue
                else:
                    if time_t[i][j] > max_x: 
                        max_x = time_t[i][j]
                    if time_d[i][j] > max_y:
                        max_y = time_d[i][j]
        M += max_x + max_y

    if type(truckRoute) != str: 
        x = { (i,j): 0 for (i,j) in Arcs}
        y = { i: 0 for i in N}
        for i in range(len(truckRoute) - 1):
            x[truckRoute[i],truckRoute[i+1]] = 1
            if truckRoute[i] != 0 and truckRoute[i] != truckRoute[-1]:
                y[truckRoute[i]] = 1
            
        N_dron = [i for i in V if i not in truckRoute]
        N_camion = [i for i in N if i not in N_dron]
        N_0_camion = [i for i in N_0 if i not in N_dron]
        N_N_camion = [i for i in N_N if i not in N_dron]
        L_dron = [(i,j,k) for i in N_0_camion for j in N_dron for k in N_N_camion if i != j and k != j]
        for j in N_dron:
            L_dron.remove((0,j,V[-1]))

    ##### Modelo matemático ######
    with gp.Env(empty=True) as env:
        env.setParam('OutputFlag', 0)
        env.start()
        with gp.Model("FDTSP",env=env) as mdl:

            mdl.Params.Threads = 1

            ###### Variables ######
            z = mdl.addVars([(i,j,k) for i,j,k in L_dron], vtype= GRB.BINARY, name = 'z')
            w = mdl.addVars([i for i in V], vtype = GRB.INTEGER, name = 'w')
            a = mdl.addVars([i for i in V], vtype = GRB.CONTINUOUS, name = 'a')
            
            for i in N_N_camion:
                if tiemposTSP[i] - 1 > 0: 
                    a[i].lb = tiemposTSP[i] - 1
                else:
                    a[i].lb = tiemposTSP[i]

            a[0].lb = 0
            a[0].ub = 0
            for i in N_dron:
                a[i].lb = 0

            if type(droneRoute) != str:
                for r in droneRoute:
                    z[r[0],r[1],r[2]].start = 1
    
            ###### Función objetivo ###### 
            # (1)
            mdl.setObjective(a[V[-1]], GRB.MINIMIZE)
            
            ### Sujeto a 
            ######################## Forzar z == 0 para j que no circulan por recorrido del camión y exceden batería (sin considerar tiempos de espera) ######################## 
            flag = False
            if time.time() - ini_carga <= tiempoLimite:
                flag = True
                for i in N_0_camion:
                    for j in N_dron:
                        for k in N_N_camion:
                            if i == 0 and k == V[-1]:
                                continue
                            else:
                                if truckRoute.index(i) <= truckRoute.index(k): # Se limitan a los viajes que van en la dirección del camión
                                    if time_d[i][j] + serv_d + time_d[j][k] <= endurance: # Se limita a los viajes que 
                                        continue
                                    else:
                                        mdl.addConstr(z[i,j,k] == 0)
                                else: 
                                    mdl.addConstr(z[i,j,k] == 0)

            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # ###### Restricciones de ruteo  ###### 

            # (5) Guarantee that each customer is served exactly once 
            
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for j in N_dron:
                    mdl.addConstr(gp.quicksum(z[i,j,k] for i in N_0_camion for k in N_camion if i != j and k != j and k != i) + gp.quicksum(z[i,j,V[-1]] for i in N_camion if i != j) + gp.quicksum(z[i,j,i] for i in N_camion if i != j) == 1, 'Ruteo_5')
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (6) Limita el numero de drones lanzados desde un nodo visitado por el camión
            
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    mdl.addConstr(y[i]*m >= gp.quicksum(z[i,j,k] for j in N_dron for k in N_N_camion if i != j and k != j and k != i) + gp.quicksum(z[i,j,i] for j in N_dron if i != j), 'Ruteo_6') #Lanzamineto
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False
            
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                exprs_w1 = gp.LinExpr()
                for j in N_dron:
                    for k in N_camion:
                        if k != j:
                            exprs_w1.addTerms(1,z[0,j,k])
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (7) Limita el numero de drones lanzados desde el depósito
            
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                mdl.addConstr(m >= exprs_w1, 'Ruteo_7')
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (8) Limita la existencia de lanzamientos y recepciones, según la presencia del camión
            
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    mdl.addConstr(y[i]*M >= gp.quicksum(z[i,j,k] for j in N_dron for k in N_N_camion if i != j and k != j) + gp.quicksum(z[k,j,i] for k in N_0_camion for j in N_dron if k != j and i != j), 'Ruteo_8')

            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (9) Sets w_0 equal to the number of airborne drones upon leaving 0. # w_0 = lanzaminetos desde 0, no se consideran recepciones en N 
            
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                mdl.addConstr(w[0] == exprs_w1, 'Ruteo_9')

            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (10) Sets w_0' equal to zero to ensure that all drones rejoins the truck upon returning to the depot
            
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                mdl.addConstr(w[V[-1]] == 0, 'Ruteo_10') # Drones volando en N = 0
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (11) Guarantee that the number of airborne drones upon leaving each location visited by the truck does not exceed the number of available drones 
            
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                mdl.addConstr(w[0] <= m*gp.quicksum(x[0,j] for j in N_camion), 'Ruteo_11')
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (12) 
            
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    mdl.addConstr(w[i] <= m*gp.quicksum(x[i,j] for j in N_N_camion if j != i), 'Ruteo_12')
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                exprs_w2 = {}
                for j in N_camion: 
                    expr = gp.LinExpr()
                    for r in N_dron:
                        for s in N_N_camion:
                            if j != r and s != r and s != j:
                                expr.addTerms(1,z[j,r,s])
                    exprs_w2[j] = expr
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (13) Allow to correctly set the values of the w-variables # volando en 0 - lanzados en 0 + lanzados en j <= volando en j  
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for j in N_camion:
                    mdl.addConstr(w[0] - gp.quicksum(z[0,r,j] for r in N_dron if j != r) + exprs_w2[j]
                                <= w[j] + M*(1 - x[0,j]), 'Ruteo_13')

            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (14) Allow to correctly set the values of the w-variables # volando en i - recibidos en j + lanzados en j <= volando en j ###
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    for j in N_camion:
                        if j != i:
                            mdl.addConstr(w[i] - gp.quicksum(z[s,r,j] for s in N_0_camion for r in N_dron if s != r and j != r and s != j) + exprs_w2[j]
                                <= w[j] + M*(1 - x[i,j]), 'Ruteo_14')

            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (15) # volando en i - recibidos en N <= 0 
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    mdl.addConstr(w[i] - gp.quicksum(z[s,r,V[-1]] for s in N_camion for r in N_dron if s != r)
                                <= w[V[-1]] + M*(1 - x[i,V[-1]]), 'Ruteo_15')
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (16) Si lo que está volando saliendo de i - lo que aterriza en j es es menor que 0 (excede el nr de drones), no se pude lanzar en j. Esta restriccion limita tambien los viajes <i,j,i> ###
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_0_camion:
                    for j in N_camion:
                        if i != j:
                            mdl.addConstr(gp.quicksum(z[(j,r,s)] for r in N_dron for s in N_N_camion if j != r and s != r ) <= m - (w[i] - gp.quicksum(z[(s,r,j)] for s in N_0_camion for r in N_dron if s != r and j != r and s != j)) + M*(1 - x[i,j]), 'Ruteo_16')
            elif time.time() - ini_carga > tiempoLimite:
                flag = False
            # Time restrictions  

            # (17) Arrival at the depot 0 
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                mdl.addConstr(a[0] == 0, 'Tiempo_17')

            elif time.time() - ini_carga > tiempoLimite:
                flag = False
   
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                expr_a1 = gp.LinExpr()
                for r in N_dron:
                    for s in N_camion:
                        if s != r:
                            expr_a1.addTerms(se,z[0,r,s])
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False
            
            # (18) 
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for j in N_camion:  
                    mdl.addConstr(a[0] 
                    + (M + time_t[0][j])*x[0,j]
                    + expr_a1
                    <= a[j] + M, 'Tiempo_18')
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False
            
            # (19)
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for j in N_dron:
                    mdl.addConstr(a[0]
                    + gp.quicksum( (M + time_d[0][j])*z[0,j,k] for k in N_camion if k != j) # k not in N_N 
                    + expr_a1
                    <= a[j] + M, 'Tiempo_19')
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False
            
            # (20)
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for j in N_camion:
                    for l in N_dron:
                        if l != j:
                            mdl.addConstr(a[0]
                            + expr_a1
                            + (M + time_d[0][l] + serv_d + time_d[l][j])*z[0,l,j]
                            <= a[j] + M, 'Tiempo_20')
                # print('Restricción 20') #

            elif time.time() - ini_carga > tiempoLimite:
                flag = False
            
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                exprs_a2 = {}
                for i in N_camion:
                    expr2 = gp.LinExpr()
                    for r in N_dron:
                        for s in N_N_camion:
                            if i != r and s != r:
                                expr2.addTerms([se], [z[i,r,s]])
                    exprs_a2[i] = expr2
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False
            
            # (21)
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    for j in N_camion:
                        if j != i:
                            for l in N_dron:
                                if l != i:
                                    mdl.addConstr(a[i] 
                                    + (M + time_t[i][j] + serv_t)*x[i,j]
                                    + exprs_a2[i]
                                    + (time_d[i][l] + serv_d + time_d[l][i])*z[i,l,i] # VUELO <i,k,i>: No puede ser una sumatoria, se utiliza el viaje con el tiempo más largo
                                    <= a[j] + M, 'Tiempo_21')
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False
            
            # (22)
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    for j in N_dron:
                        if j != i:
                            mdl.addConstr(a[i] 
                            + exprs_a2[i]
                            + gp.quicksum( (M + serv_t + time_d[i][j])*z[i,j,k] for k in N_N_camion if k != j) # Equivalente utilizar un for j in N_N + (M + service_t[i] + time_d[i,j])*z[i,j,k]
                            <= a[j] + M, 'Tiempo_22')
            
            elif time.time() - ini_carga > tiempoLimite:
                flag = False
            
            # (23)
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    for j in N_camion:
                        for l in N_dron:
                            if j != i and i != l and j != l:
                                mdl.addConstr(a[i]
                                + serv_t
                                + exprs_a2[i]
                                + (M + time_d[i][l] + serv_d + time_d[l][j] )*z[i,l,j]
                                <= a[j] + M, 'Tiempo_23')
                # print('Restricción 23')

            elif time.time() - ini_carga > tiempoLimite:
                flag = False
                
            # (24) 
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion: 
                    for j in N_dron:
                        if j != i:
                            mdl.addConstr(a[i] 
                            + (M + time_t[i][V[-1]] + serv_t)*x[i,V[-1]] 
                            + exprs_a2[i]
                            + (time_d[i][j] + serv_d + time_d[j][i])*z[i,j,i] 
                            <= a[V[-1]] + M, 'Tiempo_24') 
                # print('Restricción 24')    

            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # # Drone Battery Endurance
            # (25)
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for j in N_dron:
                    for k in N_camion:
                        if k != j:
                            mdl.addConstr(endurance >= a[k] - a[0] - expr_a1 - (1 - z[0,j,k])*M, 'Bateria_25')
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (26)
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    for j in N_dron:
                        for k in N_camion:
                            if i != j and k != j and k != i:
                                mdl.addConstr(endurance >= a[k] - a[i] - exprs_a2[i] - serv_t - (1 - z[i,j,k])*M, 'Bateria_26') # Considera tiempo de espera # gp.quicksum( se*z[i,r,s] for r in N_dron for s in N_N_camion if i != r and s != r)
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (27)
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    for j in N_dron:
                        if i != j:
                            mdl.addConstr(endurance >= time_d[i][j] + serv_d + time_d[j][i] - (1 - z[i,j,i])*M, 'Bateria_27')
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            # (28)
            if time.time() - ini_carga <= tiempoLimite and flag == True:
                flag = True
                for i in N_camion:
                    for j in N_dron:
                            if i != j:
                                mdl.addConstr(endurance >= time_d[i][j] + serv_d + time_d[j][V[-1]] - (1 - z[i,j,V[-1]])*M, 'Bateria_28')
            elif time.time() - ini_carga > tiempoLimite:
                flag = False

            if time.time() - ini_carga <= tiempoLimite and flag == True:
                mdl.setParam("TimeLimit", tiempoLimite - (time.time() - ini_carga))
                mdl.optimize()

            try:
                obj = mdl.ObjVal
            except AttributeError:
                obj = math.inf

            # print('ObjVal mdl reducido= ', obj)
            # print('BEST BOUND =',best_Bd)
            # print('MIP GAP =', gap)
            # print('Runtime mdl reducido= ', t)

            lista_z = []
            nodos_z = []
            dict_a = {}
            if obj != math.inf: 

                for var in mdl.getVars():
                        if var.x > 0.9 and var.VarName[0] == 'z':
                            contador = 0
                            numero1 = ''
                            numero2 = ''
                            numero3 = ''
                            for letra in var.VarName:
                                if letra != 'z' and letra != '[' and letra != ',' and contador == 0:
                                    numero1 += letra
                                if letra != 'z' and letra != ',' and contador == 1:
                                    numero2 += letra
                                if letra != 'z' and letra != ']' and letra != ',' and contador == 2:
                                    numero3 += letra
                                if letra == ',':
                                    contador += 1
                            lista_z.append([int(numero1), int(numero2),  int(numero3)])
                            nodos_z.append(int(numero2))
                        if var.VarName[0] == "a":
                            numero = ''
                            for letra in var.VarName:
                                if letra != 'a' and letra != '[' and letra != ']':
                                    numero += letra
                            dict_a[int(numero)] = var.x
                fact = 0
            else:
                fact = -1

    return lista_z, nodos_z, dict_a, fact #, t

def main(inst):
    global nrNodos, endurance, droneSpeed, truckSpeed, coords, nrDrones, time_t, time_d

    instances_path = './instancias_modelo'

    nrNodos, endurance, droneSpeed, truckSpeed, coords = lecturaDatos(inst, instances_path)
    time_t, time_d = tiemposViaje()

    tiempoLimite = 3600 # segundos
    seed = 1
    nrDrones = 5

    for i in range(1, nrDrones + 1):
        obj, best_Bd, gap, runTime, fact = modelo_FDTSP(tiempoLimite, seed, m = i)
        print(inst, nrDrones, seed, runTime, obj, best_Bd, gap, fact)


if __name__=="__main__":
    argv = sys.argv[1:]
    try:
        opts, args = getopt.getopt(argv,"a:")
        if len (opts) < 1:
            print('.\modelo.py -a')
            sys.exit(2)
    except getopt.GetoptError:
        print('.\modelo.py -a')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-a':
            inst = arg

    file = open(inst, "r")
    for i in file:
        instancia = i[:-1]
        main(instancia)
    file.close()


    



    

