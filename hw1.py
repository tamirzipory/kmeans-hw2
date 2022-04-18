import sys
import json

class Env:
    k = "default"
    input_file = "default"
    output_file = "default"
    maxiter = "default"


def print_err():
    print("An Error Has Occurred")
    sys.exit(1)


def read_args():
    if len(sys.argv) < 4 or len(sys.argv) > 5:
        print("invalid input")
        sys.exit(1)

    Env.k = int(sys.argv[1])

    if len(sys.argv) == 4:
        Env.max_iter = 200
        Env.input_file = sys.argv[2]
        Env.output_file = sys.argv[3]
    else:
        Env.max_iter = int(sys.argv[2])
        Env.input_file = sys.argv[3]
        Env.output_file = sys.argv[4]


def write_output():

    try:
        output = open(Env.output_file, "w+")
        for cent in centroid:
            for i in range(len(cent)):
                output.write('%.4f' % cent[i])
                if i < len(cent)-1:
                    output.write(", ")
            output.write('\n')
    except Exception as ex:
        print(ex)
        output.close()
        print_err()

    finally:
        output.close()




read_args()

try:
    input = open(Env.input_file, "r")
    mat = []
    mat = [[float(num) for num in line.split(',')] for line in input]
    # print (mat)
    centroid = mat[0:Env.k]
    # centroid = json.dumps(centroid)
    # mat = json.dumps(mat)
except:
    print_err()
finally:
    input.close()


#Matan Part



# here, the centro matrix is the correct

write_output()

