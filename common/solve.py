from input import Input
import numpy as np
import random

cross_over = 0.5
mutation = 0.05

def initialize(size, num_of_sensors, Y):
    population = np.zeros((size, num_of_sensors))
    for sn in range(size):
        i = Y
        while i > 0:
            rd = random.randint(0, num_of_sensors-1)
            if population[sn][rd] == 0:
                population[sn][rd] = 1
                i -= 1
    return population

def cal_k_coverage_cost(size, k, num_of_relays, num_of_sensors, sensor_coverage, population):
    cov = np.zeros((size, num_of_relays))
    m = num_of_relays
    cov_matrix = sensor_coverage
    for s in range(size):
        for i in range(num_of_relays):
            for j in range(num_of_sensors):
                if population[s][j] == 1 and cov_matrix[i][j] == 1:
                    cov[s][i] += 1
    sum = [0] * size
    for s in range(size):
        for i in range(num_of_relays):
            if cov[s][i] >= k:
                sum[s] += 1.0/m
            else:
                sum[s] += float(cov[s][i]) / (k*m)
    return sum


def cal_max_communicate_loss(size, comm_loss_matrix, num_of_sensors, population):
    max = [-99999] * size
    loss_matrix = comm_loss_matrix
    for s in range(size):
        for i in range(num_of_sensors):
            for j in range(i+1, num_of_sensors):
                if population[s][i] == 1 and population[s][j] == 1 and loss_matrix[i][j] > max[s]:
                    max[s] = loss_matrix[i][j]

    return max


def cal_min_sensors(size, population):
    min = [99999] * size
    for s in range(size):
        min[s] = np.count_nonzero(population[s] == 1.0)
    return min


def fast_non_dominated_sort(size, k_coverage_cost, max_communicate_loss, min_sensors):
    Sp = np.empty(size, dtype=np.object)
    F = np.empty(size + 1, dtype=np.object)
    for i in range(size):
        Sp[i] = []
        F[i] = []
    Np = [0] * size
    rank = [0] * size

    for i in range(size):
        for j in range(i+1, size):
            if(k_coverage_cost[i] >= k_coverage_cost[j] and max_communicate_loss[i] <= max_communicate_loss[j]) and min_sensors[i] <= min_sensors[j]:
                Sp[i].append(j)
            else:
                if(k_coverage_cost[i] < k_coverage_cost[j] and max_communicate_loss[i] > max_communicate_loss[j]) and min_sensors[i] > min_sensors[j]:
                    Sp[j].append(i)
                    Np[i] += 1
    for i in range(size):
        if Np[i] == 0:
            rank[i] = 1
            F[1].append(i)

    i = 1
    while F[i] != None and F[i] != []:
        Q = []
        for x in F[i]:
            for z in Sp[x]:
                Np[z] -= 1
                if Np[z] == 0:
                    rank[z] = i+1
                    Q.append(z)
        i += 1
        F[i] = Q
    return rank


def mode_reproduction(size, population, num_of_sensors, p_best, pg, pm, pp):
    new_population = np.zeros((size, num_of_sensors))
    a = pg + pm
    b = pg + pm + pp
    for i in range(size):
        for j in range(num_of_sensors):
            rand = random.random()
            if rand <= pg:
                new_population[i][j] = population[i][j]
            elif rand > pg and rand <= a:
                r = random.randint(0, len(p_best)-1)
                new_population[i][j] = population[p_best[r]][j]
            elif rand > a and rand <= b:
                r = random.randint(0, size-1)
                new_population[i][j] = population[r][j]
            elif rand > b:
                r = -1
                while 1:
                    r = random.randint(0, size-1)
                    if r != i:
                        break
                new_population[i][j] = population[r][j]
    return new_population

def generate_offspring(size, num_of_sensors, rank, population):
    q_population = np.zeros((size, num_of_sensors))
    # binary selection
    for i in range(size):
        candicate_1 = random.randint(0, size - 1)
        candicate_2 = random.randint(0, size - 1)
        if rank[candicate_1] < rank[candicate_2]:
            q_population[i] = population[candicate_2]
        else:
            q_population[i] = population[candicate_1]
    # uniform cross over
    for i in range(int(size / 2)):
        for j in range(num_of_sensors):
            if(random.random() < cross_over):
                temp = q_population[i][j]
                q_population[i][j] = q_population[size-1][j]
                q_population[size-1][j] = temp
    # random mutate
    for i in range(size):
        for j in range(num_of_sensors):
            if(random.random() < mutation):
                q_population[i][j] = (q_population[i][j] + 1) % 2
    
    return q_population

def crowding_distance_assigment(size):
    I = [0] * size

def sortSecond(val): 
    return val[1]  
class Solve:
    def __init__(self, filename, size, loop):
        self.inp = Input.from_file(filename)
        # num of sensors will be set
        self.Y = int(self.inp.num_of_sensors * 3 / 4)
        # k coverage
        self.k = 3
        self.size = size
        self.loop = loop
        self.pg = 0.3
        self.pm = 0.075
        self.pp = 0.05
        self.k_coverage_cost = []
        self.max_communicate_loss = []
        self.min_sensors = []
        self.rank = []

    def find_bests(self):
        self.p_best = []
        for i in range(self.size):
            if self.rank[i] == 1:
                self.p_best.append(i)

    '''Multi Objective Different Evolution algorithm'''

    '''Nondominated sorting genetic algorithm II (NSGA-II)'''

    '''Multi objective different evolution _ decomposition'''

if __name__ == "__main__":
    s = Solve('../data/small_data/no-dem1_r25_1.in', 50, 50)
    population = initialize(s.size, s.inp.num_of_sensors, s.Y)
    s.k_coverage_cost = cal_k_coverage_cost(
        s.size, s.k, s.inp.num_of_relays, s.inp.num_of_sensors, s.inp.sensor_coverage, population)
    s.max_communicate_loss = cal_max_communicate_loss(
        s.size, s.inp.comm_loss_matrix, s.inp.num_of_sensors, population)
    s.min_sensors = cal_min_sensors(s.size, population)
    s.rank = fast_non_dominated_sort(
        s.size, s.k_coverage_cost, s.max_communicate_loss, s.min_sensors)
    q_population = generate_offspring(s.size, s.inp.num_of_sensors, s.rank, population)

    population = np.concatenate((population, q_population))

    s.k_coverage_cost = cal_k_coverage_cost(
        2*s.size, s.k, s.inp.num_of_relays, s.inp.num_of_sensors, s.inp.sensor_coverage, population)
    s.max_communicate_loss = cal_max_communicate_loss(
        2*s.size, s.inp.comm_loss_matrix, s.inp.num_of_sensors, population)
    s.min_sensors = cal_min_sensors(2*s.size, population)
    s.rank = fast_non_dominated_sort(
        2 * s.size, s.k_coverage_cost, s.max_communicate_loss, s.min_sensors)
    s.min_sensors.sort()

    sort_min_sensors = []
    for i in range(2 * s.size):
        sort_min_sensors.append((i, s.min_sensors[i]))
    sort_min_sensors.sort(key = sortSecond)
    print(sort_min_sensors)
    ## mode
    # s.k_coverage_cost = cal_k_coverage_cost(
    #     s.size, s.k, s.inp.num_of_relays, s.inp.num_of_sensors, s.inp.sensor_coverage, population)
    # s.max_communicate_loss = cal_max_communicate_loss(
    #     s.size, s.inp.comm_loss_matrix, s.inp.num_of_sensors, population)
    # s.min_sensors = cal_min_sensors(s.size, population)
    # s.rank = fast_non_dominated_sort(
    #     s.size, s.k_coverage_cost, s.max_communicate_loss, s.min_sensors)
    # s.find_bests()
    # best = s.p_best[random.randint(0, len(s.p_best)-1)]
    # print("Gen 0:")
    # print("k_coverage: {},  comm_loss: {}, min_sensors: {}".format(
    #     s.k_coverage_cost[best], s.max_communicate_loss[best], s.min_sensors[best]))
    # i = 1
    # while i < s.loop:
    #     population = mode_reproduction(
    #         s.size, population, s.inp.num_of_sensors, s.p_best, s.pg, s.pm, s.pp)
    #     s.k_coverage_cost = cal_k_coverage_cost(
    #         s.size, s.k, s.inp.num_of_relays, s.inp.num_of_sensors, s.inp.sensor_coverage, population)
    #     s.max_communicate_loss = cal_max_communicate_loss(
    #         s.size, s.inp.comm_loss_matrix, s.inp.num_of_sensors, population)
    #     s.rank = fast_non_dominated_sort(
    #         s.size, s.k_coverage_cost, s.max_communicate_loss, s.min_sensors)
    #     s.min_sensors = cal_min_sensors(s.size, population)
    #     s.find_bests()
    #     best = s.p_best[random.randint(0, len(s.p_best)-1)]
    #     print("Gen {}:".format(i))
    #     print("k_coverage: {},  comm_loss: {}, min_sensors: {}".format(
    #         s.k_coverage_cost[best], s.max_communicate_loss[best], s.min_sensors[best]))
    #     i += 1
