    
#ifndef SPKMEANS_H
#define SPKMEANS_H
#include <math.h>
#include <stdio.h>

struct eigVA
{
    double value;
    double *vector;
};

void goalRouter(int, double **, char *, int , int , int , double ***);
int isSymetric(double **, int);
void printMat(double **, int, int);
void sortInplace(struct eigVA *, int);
double calcW(double *, double *, int);
double **initPoints(FILE * , int, int);
double **getAdgMat(double **, int, int);
double **diagMat(double **, int);
double rowSum(double **, int, int);
double **getSqrt(double **, int);
double **getIdentityMat(int n);
double getVectorsPoduct(double **, double **, int, int, int);
double ** getmultipiedMats(double **, double **, int, double**);
double **findLnorm(double **, double **, int);
void maxIJ(double **, int, int *);
double getC(double);
double get_t(double);
int signOf(double);
void updateP(int, int, int, double, double, double **);
void printValuesAndVectors(double **, double **, int);
double** normalize(double **, int, int);
double sumOfSquares(double*, int);
int replaceK(double *, int);
double *getFinalCentroids(double *, double *, int, int, int, int, double);
void initClusters(int, int *);
void resetClusters(int *, int, int);
int updateCentroids(int *, double *, double *, int *, int, int, int, double);
void sumPoints(double *, double *, int, int);
void updateClusters(double *, double *, int *, int, int, int *, int);
void addPointToCluster(int *, int , int, int);
int getCentroid(double *, double *, int, int, int);
double distance(double *, double *, int, int, int);
int checkMovement(int *, int);
void findDims(FILE *, int *);
double*** jacobiAlg(double** , int ,double***);
void RAEInput(void); /*reports and exits for invalid inputs*/
void RAEGen(void); /*reports and exits in general cases*/ 
void freeLoop(double**,int);
double** allocateMem(int ,int); /*returns memory or null if failed*/
void printMat_withtxt(char*, double **, int , int ); /*debug only*/
int is_diagonal(double **, double **, int, double);
double off(double **,int);
#endif 