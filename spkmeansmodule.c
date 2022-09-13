#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <math.h>
#include <ctype.h>
#include "spkmeans.h"
#include <stdbool.h>

/* sends user input from python to goalRouter in C to get correct matrix and print in Python */
static PyObject *goalRoutingFunc(PyObject *self, PyObject *args)
{

    PyObject *inputPointsPy, *pyT, *pyTRow;
    int n, k, d, i, j;
    char *goal;
    double **inputPointsC, **T, ***targetMatrixPointer, **vectors, **values;
    if (!PyArg_ParseTuple(args, "iiiOs", &k, &n, &d, &inputPointsPy, &goal))
    {
        RAEInput();
    }
    targetMatrixPointer = (double ***)calloc(2, sizeof(double **));
    if (targetMatrixPointer == NULL)
    {
        RAEGen();
    }

    inputPointsC = allocateMem(n, d);
    if (inputPointsC == NULL)
    {
        free(targetMatrixPointer);
        RAEGen();
    }

    for (i = 0; i < n; ++i)
    {
        for (j = 0; j < d; ++j)
        {
            inputPointsC[i][j] = PyFloat_AsDouble(PyList_GetItem(inputPointsPy, i * d + j));
        }
    }
    /*sending ready input for C to goal router*/
    goalRouter(1, inputPointsC, goal, n, d, k, targetMatrixPointer);

    pyT = PyList_New(0);

    if (strcmp("wam", goal) == 0 || strcmp("ddg", goal) == 0 || strcmp("lnorm", goal) == 0)
    {
        T = targetMatrixPointer[0];
        for (i = 0; i < n; i++)
        {
            pyTRow = PyList_New(0);
            for (j = 0; j < n; j++)
            {
                PyList_Append(pyTRow, PyFloat_FromDouble(T[i][j]));
            }
            PyList_Append(pyT, pyTRow);
        }

        freeLoop(T, n);
        freeLoop(inputPointsC, n);
        free(inputPointsC);
    }

    if (strcmp("jacobi", goal) == 0){

        vectors = targetMatrixPointer[0];
        values = targetMatrixPointer[1];
        pyTRow = PyList_New(0);
        for (j = 0; j < n; j++)
        {
            PyList_Append(pyTRow, PyFloat_FromDouble(values[j][j]));
        }
        PyList_Append(pyT, pyTRow);
        for (i = 0; i < n; i++)
        {
            pyTRow = PyList_New(0);
            for (j = 0; j < n; j++)
            {
                PyList_Append(pyTRow, PyFloat_FromDouble(vectors[i][j]));
            }
            PyList_Append(pyT, pyTRow);
        }

        freeLoop(vectors, n);
        free(vectors);
        freeLoop(values, n);
        free(values);
    }

    if (strcmp("spk", goal) == 0)
    {
        T = targetMatrixPointer[0];
        k = (int)targetMatrixPointer[1][0][0];
        for (i = 0; i < n; i++)
        {
            pyTRow = PyList_New(0);
            for (j = 0; j < k; j++)
            {
                PyList_Append(pyTRow, PyFloat_FromDouble(T[i][j]));
            }
            PyList_Append(pyT, pyTRow);
        }

        freeLoop(T, n);
        free(targetMatrixPointer[1][0]);
        free(targetMatrixPointer[1]);
        freeLoop(inputPointsC, n);
        free(inputPointsC);
    }

    free(targetMatrixPointer);
    /* returns output from C's goalrouter to python*/
    return Py_BuildValue("O", pyT);
}

/*sending the centroids we got from python to Kmeans in C*/
static PyObject *fit(PyObject *self, PyObject *args)
{
    PyObject *cents_py, *points_py;
    int n, k, d, i, j;
    double *centroids = NULL;
    double *all_points = NULL;
    double *our_final_centroids;
    PyObject *pyCent;
    PyObject *pyCentArr;
    double cordinate;
    PyObject *pyCor;

    if (!PyArg_ParseTuple(args, "iiiOO", &k, &n, &d, &cents_py, &points_py))
    {
        RAEInput();
    }
    all_points = (double *)(malloc(n * d * sizeof(double)));
    if (all_points == NULL)
    {
        RAEGen();
    }
    i = 0;
    for (i = 0; i < n * d; i++)
    {
        pyCor = PyList_GetItem(points_py, i);
        cordinate = PyFloat_AsDouble(pyCor);
        all_points[i] = cordinate;
    }

    centroids = (double *)malloc(d * k * sizeof(double));
    if (centroids == NULL)
    {
        free(all_points);
        RAEGen();
    }

    /* init centroids */

    for (i = 0; i < d * k; i++)
    {
        pyCor = PyList_GetItem(cents_py, i);
        cordinate = PyFloat_AsDouble(pyCor);
        centroids[i] = cordinate;
    }
    /*now we have first centroids from python in C array */

    our_final_centroids = getFinalCentroids(centroids, all_points, k, d, n, 300, 0);
    /* creating final centroids array for python in matrix form */
    pyCentArr = PyList_New(0);

    for (i = 0; i < k; i++)
    {
        pyCent = PyList_New(0);
        for (j = 0; j < k; j++)
        {
            PyList_Append(pyCent, PyFloat_FromDouble(our_final_centroids[i * k + j]));
        }
        PyList_Append(pyCentArr, pyCent);
    }
    return Py_BuildValue("O", pyCentArr);
}

static PyMethodDef KmeansMethods[] = {
    {"goalRoutingFunc",
     (PyCFunction)goalRoutingFunc,
     METH_VARARGS,
     PyDoc_STR("given a legal goal, exectuing it s steps")},
    {"fit",
     (PyCFunction)fit,
     METH_VARARGS,
     PyDoc_STR("given points and initial centroids, finding the final cenroids")},
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef moduledef = {
    PyModuleDef_HEAD_INIT,
    "spkmean",    /* name of module */
    NULL,         /* module documentation, may be NULL */
    -1,           /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
    KmeansMethods /* the PyMethodDef array from before containing the methods of the extension */
};

/*
 * The PyModuleDef structure, in turn, must be passed to the interpreter in the moduleג€™s initialization function.
 * The initialization function must be named PyInit_name(), where name is the name of the module and should match
 * what we wrote in struct PyModuleDef.
 * This should be the only non-static item defined in the module file
 */
PyMODINIT_FUNC
PyInit_spkmean(void)
{
    {
        return PyModule_Create(&moduledef);
    }
}