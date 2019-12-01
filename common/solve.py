from input import Input
import numpy as np
import random


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
        self.population = np.zeros((self.size, self.inp.num_of_sensors))

    # init population

    def initialize(self):
        for sn in range(self.size):
            i = self.Y
            while i > 0:
                rd = random.randint(0, self.inp.num_of_sensors-1)
                if self.population[sn][rd] == 0:
                    self.population[sn][rd] = 1
                    i -= 1

    # cost function
    def cal_k_coverage_cost(self):
        cov = np.zeros((self.size, self.inp.num_of_relays))
        m = self.inp.num_of_relays
        cov_matrix = self.inp.sensor_coverage
        for s in range(self.size):
            for i in range(self.inp.num_of_relays):
                for j in range(self.inp.num_of_sensors):
                    if self.population[s][j] == 1 and cov_matrix[i][j] == 1:
                        cov[s][i] += 1
        sum = [0] * self.size
        for s in range(self.size):
            for i in range(self.inp.num_of_relays):
                if cov[s][i] >= self.k:
                    sum[s] += 1.0/m
                else:
                    sum[s] += float(cov[s][i]) / (self.k*m)
        self.k_coverage_cost = sum

    def cal_max_communicate_loss(self):
        max = [-99999] * self.size
        loss_matrix = self.inp.comm_loss_matrix
        for s in range(self.size):
            for i in range(self.inp.num_of_sensors):
                for j in range(i+1, self.inp.num_of_sensors):
                    if self.population[s][i] == 1 and self.population[s][j] == 1 and loss_matrix[i][j] > max[s]:
                        max[s] = loss_matrix[i][j]

        self.max_communicate_loss = max
    
    def cal_min_sensors(self):
        min = [99999] * self.size
        for s in range(self.size):
            min[s] = np.count_nonzero(self.population[s] == 1.0)
        self.min_sensors = min
    
    # pareto rank
    def fast_non_dominated_sort(self):
        k_coverage_cost = self.k_coverage_cost
        max_communicate_loss = self.max_communicate_loss
        min_sensors = self.min_sensors
        Sp = np.empty(self.size, dtype=np.object)
        F = np.empty(self.size + 1, dtype=np.object)
        for i in range(self.size):
            Sp[i] = []
            F[i] = []
        Np = [0] * self.size
        rank = [0] * self.size

        for i in range(self.size):
            for j in range(i+1, self.size):
                if(k_coverage_cost[i] >= k_coverage_cost[j] and max_communicate_loss[i] <= max_communicate_loss[j]) and min_sensors[i] <= min_sensors[j]:
                    Sp[i].append(j)
                else:
                    if(k_coverage_cost[i] < k_coverage_cost[j] and max_communicate_loss[i] > max_communicate_loss[j]) and min_sensors[i] > min_sensors[j]:
                        Sp[j].append(i)
                        Np[i] += 1
        for i in range(self.size):
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
        self.rank = rank

    def find_bests(self):
        self.p_best = []
        for i in range(self.size):
            if self.rank[i] == 1:
                self.p_best.append(i)

    '''Multi Objective Different Evolution algorithm'''

    def mode_reproduction(self):
        new_population = np.zeros((self.size, self.inp.num_of_sensors))
        a = self.pg + self.pm
        b = self.pg + self.pm + self.pp
        for i in range(self.size):
            for j in range(self.inp.num_of_sensors):
                rand = random.random()
                if rand <= self.pg:
                    new_population[i][j] = self.population[i][j]
                elif rand > self.pg and rand <= a:
                    r = random.randint(0, len(self.p_best)-1)
                    new_population[i][j] = self.population[self.p_best[r]][j]
                elif rand > a and rand <= b:
                    r = random.randint(0, self.size-1)
                    new_population[i][j] = self.population[r][j]
                elif rand > b:
                    r = -1
                    while 1:
                        r = random.randint(0, self.size-1)
                        if r != i:
                            break
                    new_population[i][j] = self.population[r][j]
        self.population = new_population

    '''Nondominated sorting genetic algorithm II (NSGA-II)'''
    

if __name__ == "__main__":
    s = Solve('../data/small_data/no-dem1_r25_1.in', 50, 50)
    s.initialize()
    s.cal_k_coverage_cost()
    s.cal_max_communicate_loss()
    s.cal_min_sensors()
    s.fast_non_dominated_sort()
    s.cal_min_sensors()
    s.find_bests()
    best = s.p_best[random.randint(0, len(s.p_best)-1)]
    print("Gen 0:")
    print("k_coverage: {},  comm_loss: {}, min_sensors: {}".format(s.k_coverage_cost[best], s.max_communicate_loss[best], s.min_sensors[best]))
    i = 1
    while i < s.loop:
        s.mode_reproduction()
        s.cal_k_coverage_cost()
        s.cal_max_communicate_loss()
        s.fast_non_dominated_sort()
        s.cal_min_sensors()
        s.find_bests()
        best = s.p_best[random.randint(0, len(s.p_best)-1)]
        print("Gen {}:".format(i))
        print("k_coverage: {},  comm_loss: {}, min_sensors: {}".format(s.k_coverage_cost[best], s.max_communicate_loss[best], s.min_sensors[best]))
        i += 1