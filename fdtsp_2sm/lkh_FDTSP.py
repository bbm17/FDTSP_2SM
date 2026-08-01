import lkh
import numpy as np
import os

def lkh_solution(nrNodes, truckNodes, time_t, semilla, grid, endurance, droneSpeed, nrDrones, solver_path = ''):
    tsp_file = '.\lkh_' + grid + '_' + str(nrNodes) + '_' + str(endurance) + '_' + str(droneSpeed) + '_' + str(nrDrones) + '_' + str(semilla) + '.tsp'

    archivo = open(tsp_file, 'w')
    archivo.write('NAME: solverLKH' + '\n')
    archivo.write('TYPE: TSP\n')
    archivo.write(f"COMMENT: FD-STP\n")
    archivo.write(f"DIMENSION: {len(truckNodes) - 1}\n")
    archivo.write(f"EDGE_WEIGHT_TYPE: EXPLICIT\nEDGE_WEIGHT_FORMAT: FULL_MATRIX\nEDGE_WEIGHT_SECTION\n") # DISPLAY_DATA_TYPE: TWOD_DISPLAY\n

    a =[]
    for i in range(len(truckNodes) - 1):
        linea = ''
        for j in range(len(truckNodes) - 1):
            if i == j:
                if j != len(truckNodes) - 2:
                    linea += str(0) + ' '
                else:
                    linea += str(0)

            else:
                if j != len(truckNodes) - 2:
                    if j == 0:
                        linea += str( time_t[truckNodes[j]][truckNodes[i]] ) + ' '
                    else:
                        linea += str( time_t[truckNodes[i]][truckNodes[j]] ) + ' '
                else:
                    linea += str( time_t[truckNodes[i]][truckNodes[j]] )
        archivo.write(linea + '\n')

    archivo.write('DEPOT_SECTION\n')
    archivo.write('1\n-1\n')

    archivo.close()

    # Especificar ruta LKH.exe
    if solver_path == '':
        print('Ingrese la ruta de LKH.exe')

    ciudad = lkh.solve(solver_path, problem_file= tsp_file, max_trials=10000, runs=1, seed = semilla)
    
    ruta = []
    ruta.append(truckNodes[0])
    for c in ciudad[0]:
        ruta.append(truckNodes[c - 1])
    ruta.append(nrNodes)
    
    os.remove(tsp_file)
    return ruta # ruta[1:]

