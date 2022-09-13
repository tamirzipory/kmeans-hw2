import sys
import numpy as np
import spkmean as cd1


#HELPFER FUNCTIONS:

 # is_input_valid is an helpfer func that checks user input's validity:"
def is_input_valid(k, goal):
    if len(user_input) != 4: #checks number of arguments from user is correct
        print("Invalid Input!")
        return False
    for i in range(len(k)): # checks k validity
        if k[i].isdigit()==False: # check that k is a non-negative integer
            print("Invalid Input!")
            return False
    k = int(k); #initialize k
    valid_goals = {'wam','ddg','lnorm','jacobi', 'spk'}
    if not (goal in valid_goals):
        print("Invalid Input!")
        return False
    return True

def get_min_distances(points, centroids):
    num_of_centroids = len(centroids)
    n = len(points)
    min_distances = [0 for i in range(n)]
    for i in range(n):
        min_dis = float('inf')
        for j in range(num_of_centroids):
            currdis = distance(points[i], centroids[j])
            if currdis<min_dis:
                min_dis = currdis
        min_distances[i] = min_dis
    return min_distances
    

def distance(x, y):
    length = len(x)
    temp =0
    for i in range(length):
        temp+=(float(x[i])-float(y[i]))**2
    return temp

def get_odds(distances):
    n = len(distances)
    dis_sum = sum(distances)
    odds = [0 for i in range(n)]
    for i in range(n):
        odds[i] = distances[i]/dis_sum
    return odds

def random_select_centroid_index(points, odds): # selects a point's index as new centroid 
    ##need to make sure that its the correct syntax
    indexes = [i for i in range(n)]
    index_selected = (np.random.choice(indexes, 1, p=odds))[0]

    return index_selected

def get_first_centroids(points, k, n, d): #points is matrix of points
    centroids = []
    np.random.seed(0)# random selection from all points
    random_index = int(np.random.randint(0, n))
    centroids.append(points[random_index])
    used_indexes = [random_index]
    odds = [0.0 for i in range(n)]
    min_distances = [0.0 for i in range(n)]
    i=1
    while i<k:
        min_distances = get_min_distances(points, centroids)
        odds = get_odds(min_distances)
        i += 1
        new_centroid_index = random_select_centroid_index(points, odds)
        new_centroid = points[new_centroid_index]
        centroids.append(new_centroid)
        used_indexes.append(new_centroid_index)
    return [centroids, used_indexes]

def is_n_valid(n,k):
    return n>k or n==k

def cast_to_floats(points):
    for i in range(n):
        d_dim_point = user_input_points[i].split(",")
        for j in range(len(d_dim_point)):
            d_dim_point[j] = float(d_dim_point[j])
        user_input_points[i] = d_dim_point

def get_routing_input(n,points):
    ans = []
    d = len(points[0])
    for i in range(n):
        for j in range(d):
                ans.append(points[i][j])
    return ans

def mat_to_array(mat,r,c):
    ans = [];
    for i in range (r):
        for j in range (c):
            ans.append(mat[i][j])

    return ans

#helping functions to print matrix in python
def spk_print(final_centroids,indexes):    
    for i in range(len(indexes)):
        indexes[i] = str(indexes[i])

    indexes = ",".join(indexes)
    print(indexes)
    print_mat(final_centroids, len(final_centroids), len(final_centroids[0]));


def print_mat(mat, r, c):
    for i in range(r):
        for j in range(c):
            mat[i][j] = '%.4f'%mat[i][j]
        result = []
    for i in range(r):
        result.append(",".join(mat[i]))
    for i in range(r):
        if i!=r-1:
            print(result[i])
        else:
            print(result[i])


#___ MAIN CODE OF PYTHON: ___  

# recieving args from cmd
user_input = sys.argv
if len(user_input) != 4: #checks number of arguments from user is correct
    print("Invalid Input!")
    sys.exit();
k = user_input[1]
goal = user_input[2]
valid = True
valid = is_input_valid(k, goal)
if valid == False:
    sys.exit()
k = int(k)
if(k==1 and goal=="spk"):
    print("Invalid Input!")
    sys.exit()

filename = user_input[3]
f = open(filename, "r")
reading_file = f.read()
user_input_points = reading_file.splitlines()
n = len(user_input_points)
if not is_n_valid(n,k):
    print("Invalid Input!")
    sys.exit()

cast_to_floats(user_input_points)
f.close()
d = len(user_input_points[0])
routing_input = get_routing_input(n,user_input_points)
T = cd1.goalRoutingFunc(k, n, d, routing_input, goal)

r = len(T)
c = len(T[0])

#printing goal matrix we got from C's goal router here

#if goal is not spk we will print the matrix we got from goal router here
if (goal=="wam" or goal == "ddg" or goal =="lnorm"):
    print_mat(T,r,c)

if (goal == "jacobi"):
    for i in range(n):
        if abs(T[0][i])<0.00005:
            T[0][i] = 0.0000
    print_mat(T,r,c)

#if goal is spk we will send T (that we got from goal router) to Kmeans algorythm in C
if goal == "spk":
    k = len(T[0])
    centroids_and_indexes = get_first_centroids(T, k, n, k) # these are the first centroids and their indexes
    centroids= centroids_and_indexes[0] #first centroids
    indexes = centroids_and_indexes[1] #first centroids indexes
    initial_centroids_fit_input = mat_to_array(centroids,k,k)
    T_fit_input = mat_to_array(T,n,k)
    # getting final centroids out of  T as input points :
    final_centroids= cd1.fit(k, n, k,initial_centroids_fit_input, T_fit_input)
    # printing the initial indexes and the final centroids (as did in HW2):
    spk_print(final_centroids,indexes)

   