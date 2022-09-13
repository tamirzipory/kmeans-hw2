
#define PY_SSIZE_T_CLEAN
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <ctype.h>
#include <string.h>
#include <float.h>
#include "spkmeans.h"

int main(int argc, char *argv[])
{
    /**setting variables**/
    int k, d, n, dimensions[2] = {0, 0};
    double **allPoints, ***inputHlp;
    char *goal, *fileName;
    FILE *f;
    k = 0;
    /*INPUT CHECKING*/
    if (argc != 3)
    { /*checking if K is an int*/
        RAEInput();
    }
    goal = argv[1];
    if (((strcmp(goal, "wam") != 0)) && ((strcmp(goal, "ddg") != 0)) && ((strcmp(goal, "lnorm") != 0)) && ((strcmp(goal, "jacobi") != 0)))
    {
        RAEInput();
    }
    fileName = argv[2];
    f = fopen(fileName, "r");
    if (f == NULL)
    {
        RAEGen();
    }
    
    findDims(f, dimensions);
    n = dimensions[0];
    d = dimensions[1];
    rewind(f);

    allPoints = initPoints(f, n, d);
    if (allPoints == NULL)
    {
        RAEGen();
    }
    
    inputHlp = (double ***)calloc(2, sizeof(double **));
    if (inputHlp == NULL)
    {
        freeLoop(allPoints, n);
        free(allPoints);
        RAEGen();
    }
    goalRouter(0, allPoints, goal, n, d, k, inputHlp); /*since we are in main func of c we just print and return */
    fclose(f);
    free(inputHlp);
    return 0;
}

/* helping function to report error and exit*/
void RAEInput(void)
{
    printf("Invalid Input!");
    exit(1);
}
void RAEGen(void)
{
    printf("An Error Has Occurred");
    exit(1);
}

/*helping function that frees matrix from memory*/
void freeLoop(double **mat, int k)
{
    int j = 0;
    for (j = 0; j < k; j++)
    {
        free(mat[j]);
    }
    return;
}

/*gets input points and returns the correct matrix accoring to given goal*/
void goalRouter(int CFP, double **allPoints, char *goal, int n, int d, int k, double ***TargMatAndK)
{
    /*CFP = call from python */
    /*if we called the algorithm from C we will print here, otherwise will send to python and print there*/
    int i, j;
    double **W, **sqrtD, **D, **lnorm, ***VandA,
        **V, **A, **U, **kMat, *eigenvalues, **T;
    struct eigVA *structArr;

    /* **** case A- goal is not JACOBI, means it's from- wam,ddg,lnorm,spk **** */

    if (strcmp(goal, "jacobi") != 0)
    {
        W = getAdgMat(allPoints, n, d);
        if (W == NULL)
        {
            freeLoop(allPoints, n);
            free(allPoints);
            RAEGen();
        }

        /*goal =  wam*/

        if (strcmp(goal, "wam") == 0)
        {
            if (CFP == 0)
            {
                printMat(W, n, n);
                freeLoop(W, n);
                free(W);
                freeLoop(allPoints, n);
                free(allPoints);
            }
            if (CFP == 1)
            {
                TargMatAndK[0] = W;
            }
            return;
        }

        D = diagMat(W, n);
        if (D == NULL)
        {
            freeLoop(allPoints, n);
            freeLoop(W, n);
            free(allPoints);
            free(W);
            RAEGen();
        }
        /*goal = ddg:*/
        if (strcmp(goal, "ddg") == 0)
        {
            if (CFP == 0)
            {
                printMat(D, n, n);
                freeLoop(W, n);
                freeLoop(D, n);
                free(W);
                free(D);
                freeLoop(allPoints, n);
                free(allPoints);
            }
            else
            {
                freeLoop(W, n); 
                free(W);   
                TargMatAndK[0] = D;
            }
            return;
        }

        sqrtD = getSqrt(D, n);
        if (sqrtD == NULL)
        {
            freeLoop(allPoints, n);
            freeLoop(W, n);
            freeLoop(D, n);
            free(allPoints);
            free(W);
            free(D);
            RAEGen();
        }

        lnorm = findLnorm(W, sqrtD, n);
        if (lnorm == NULL)
        {
            freeLoop(allPoints, n);
            freeLoop(W, n);
            freeLoop(D, n);
            freeLoop(sqrtD, n);
            free(allPoints);
            free(W);
            free(D);
            free(sqrtD);
            RAEGen();
        }

        /*goal =  lnorm*/
        if (strcmp(goal, "lnorm") == 0)
        {
            if (CFP == 0){
                printMat(lnorm, n, n);
                freeLoop(W, n);
                freeLoop(D, n);
                freeLoop(sqrtD, n);
                freeLoop(lnorm, n);
                free(W);
                free(D);
                free(sqrtD);
                free(lnorm);
                freeLoop(allPoints, n);
                free(allPoints);
            }
            else
            {
                freeLoop(W, n);
                freeLoop(D, n);
                freeLoop(sqrtD, n);
                free(W);
                free(D);
                free(sqrtD);
                TargMatAndK[0] = lnorm;
            }

            return;
        }
        /*goal = spk:*/

        /*freeing old memory:*/
        freeLoop(W, n);
        freeLoop(D, n);
        freeLoop(sqrtD, n);
        free(D);
        free(sqrtD);
        free(W);

        V = allocateMem(n, n);
        if (V == NULL)
        {
            freeLoop(allPoints, n);
            freeLoop(lnorm, n);
            free(lnorm);
            free(allPoints);
            RAEGen();
        }
        A = allocateMem(1, n);
        if (A == NULL)
        {
            freeLoop(allPoints, n);
            freeLoop(lnorm, n);
            freeLoop(V, n);
            free(lnorm);
            free(allPoints);
            free(V);
            RAEGen();
        }
        VandA = (double ***)calloc(2, sizeof(double **));
        if (VandA == NULL)
        {
            freeLoop(allPoints, n);
            freeLoop(lnorm, n);
            freeLoop(V, n);
            free(lnorm);
            free(V);
            free(A);
            free(allPoints);
            RAEGen();
        }
        VandA = jacobiAlg(lnorm, n, VandA); /* VandA is an empty input to write to*/
        V = VandA[0];
        A = VandA[1];
    
        if (strcmp(goal, "spk") == 0)
        {
            eigenvalues = (double *)calloc(n, sizeof(double));
            if (eigenvalues == NULL)
            {
                freeLoop(allPoints, n);
                freeLoop(V, n);
                free(allPoints);
                free(A);
                free(VandA);
                RAEGen();
            }
            /* an array containing objects named "struct" which contains two fields: an eigenvector and in eigenvalue each*/
            structArr = (struct eigVA *)calloc(n, sizeof(struct eigVA));
            if (structArr == NULL)
            {
                freeLoop(allPoints, n);
                freeLoop(V, n);
                free(allPoints);
                free(A);
                free(VandA);
                free(eigenvalues);
                RAEGen();
            }

            for (i = 0; i < n; i++)
            {
                structArr[i].value = A[i][i];
                structArr[i].vector = (double *)calloc(n, sizeof(double));

                if (structArr[i].vector == NULL)
                {
                    printf("An Error Has Occurred");
                    for (j = 0; j < i; j++)
                    {
                        free(structArr[j].vector);
                    }

                    for (j = 0; j < n; j++)
                    {
                        free(allPoints[i]);
                        free(structArr[j].vector);
                    }
                    free(V);
                    free(structArr);
                    exit(1);
                }
                for (j = 0; j < n; j++)
                {
                    structArr[i].vector[j] = V[j][i];
                }
            }
            sortInplace(structArr, n);
            for (i = 0; i < n; i++)
            {
                V[i] = structArr[i].vector;
                eigenvalues[i] = structArr[i].value;
            }

            /*finding k in case it's ungiven (k is ungiven iff k==0)*/

            if (k == 0)
            {
                k = replaceK(eigenvalues, n);
            }
            kMat = (double **)calloc(1, sizeof(double *));
            kMat[0] = (double *)calloc(1, sizeof(double));
            kMat[0][0] = (double)k;
            TargMatAndK[1] = kMat;
            U = allocateMem(n, k);
            if (U == NULL)
            {
                freeLoop(V, n);
                free(V);
                free(kMat);
                free(kMat[0]);
                RAEGen();
            }
            for (i = 0; i < n; i++)
            {
                for (j = 0; j < k; j++)
                {
                    U[i][j] = V[j][i];
                }
            }

            T = normalize(U, n, k); /*Turns U into T*/

            TargMatAndK[0] = T;
            freeLoop(V, n);
            freeLoop(A, n);
            free(V);
            free(A);
            return;
        }
    }
    /*  ****case B = goal is JACOBI: **** */
    else if (strcmp(goal, "jacobi") == 0){        
        VandA = (double ***)calloc(2, sizeof(double **));
        if (VandA == NULL)
        {
            freeLoop(allPoints, n);
            free(allPoints);
            RAEGen();
        }
        VandA = jacobiAlg(allPoints, n, VandA);
        V = VandA[0];
        A = VandA[1];
        if (CFP == 0){
            printValuesAndVectors(A, V, n);
            freeLoop(V, n);
            freeLoop(A,n);
            free(V);
            free(A);
            free(VandA);
        }
        else{
            TargMatAndK[0] = V;
            TargMatAndK[1] = A;
            
        }
        return;
    }
    return;
    /*end of the goal router */
}

/* HELING FUNCTIONS*/
int isSymetric(double **allPoints, int n)
{
    int i, j;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            if (allPoints[i][j] != allPoints[j][i])
            {
                return 0;
            }
        }
    }
    return 1;
}

void printMat(double **Mat, int n, int d)
{
    int i, j;
    double cell, remain;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < d; j++)
        {
            cell = 10000 * Mat[i][j];
            remain = cell - floor(cell);
            if (remain < 0.5)
            {   
                cell = floor(cell);
            }
            else
            {
                cell = ceil(cell);
            }
            cell = cell / 10000;
            if (j < d - 1)
            {
                printf("%.4f,", cell);
            }
            else if (i != n - 1)
            {
                printf("%.4f\n", cell);
            }
            else
            {
                printf("%.4f", cell);
            }
        }
    }
    
    printf("\n");
    return;
}

double calcCellW(double *rowI, double *rowJ, int d)
{
    double cell;
    int i;
    double dis = 0;
    for (i = 0; i < d; i++)
    {
        dis += pow((rowI[i] - rowJ[i]), 2);
    }
    dis = sqrt(dis);
    cell = -(dis / 2);
    cell = exp(cell);
    return cell;
}

double **initPoints(FILE *f, int n, int d)
{
    int row, col;
    char c;
    double cell, **pointsArr;

    pointsArr = allocateMem(n, d);
    if (pointsArr == NULL)
    {
        return NULL;
    }
    for (row = 0; row < n; row++)
    {
        for (col = 0; col < d; col++)
        {
            fscanf(f, "%lf", &cell);
            pointsArr[row][col] = cell;
            fscanf(f, "%c", &c);
        }
    }
    return pointsArr;
}
/*allocates mem iff succeeded. else returns NULL*/
double **allocateMem(int r, int c)
{ 
    int i;
    double **ans;
    ans = (double **)calloc(r, sizeof(double *));
    if (ans == NULL)
    {
        return NULL;
    }
    for (i = 0; i < r; i++)
    {
        ans[i] = (double *)calloc(c, sizeof(double));
        if (ans[i] == NULL)
        {
            freeLoop(ans, i);
            free(ans);
            return NULL;
        }
    }
    return ans;
}

double **getAdgMat(double **pointsArr, int n, int d)
{
    int i, j;
    double **W;
    double tmp;
    W = allocateMem(n, n);
    if (W == NULL)
    {
        return NULL;
    }
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            if (i == j)
            {
                W[i][j] = 0;
            }
            else
            {
                tmp = calcCellW(pointsArr[i], pointsArr[j], d);
                if (tmp > 0)
                {
                    W[i][j] = tmp;
                    W[j][i] = W[i][j];
                }
            }
        }
    }
    return W;
}

double **diagMat(double **W, int n)
{
    int i, j;
    double **diagMat;
    diagMat = allocateMem(n, n);
    if (diagMat == NULL)
    {
        return NULL;
    }
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            if (i == j)
            {
                diagMat[i][j] = rowSum(W, n, i);
            }
            else
            {
                diagMat[i][j] = 0;
            }
        }
    }
    return diagMat;
}

/** sum of all the i'th row **/
double rowSum(double **W, int n, int i)
{
    int j;
    double sum = 0;
    for (j = 0; j < n; j++)
    {
        sum = sum + W[i][j];
    }
    return sum;
}

/*mat^-1/2*/
double **getSqrt(double **mat, int n)
{
    int i, j;
    double **sqrtMat;
    sqrtMat = allocateMem(n, n);
    if (sqrtMat == NULL)
    {
        return NULL;
    }

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            if (i != j)
            {
                sqrtMat[i][j] = 0;
            }
            else
            {
                sqrtMat[i][i] = 1 / sqrt(mat[i][j]);
            }
        }
    }
    return sqrtMat;
}

double **getIdentityMat(int n)
{
    int i, j;
    double **I;
    I = allocateMem(n, n);
    if (I == NULL)
    {
        return NULL;
    }
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            if (i == j)
            {
                I[i][i] = 1;
            }
            else
            {
                I[i][j] = 0;
            }
        }
    }
    return I;
}
/*dot product of M1 ith row with M2 jth col*/
double dotProduct(double **M1, double **M2, int n, int i, int j)
{
    int k;
    double sum = 0;
    for (k = 0; k < n; k++)
    {
        sum += M1[i][k] * M2[k][j];
    }
    return sum;
}

double **getMultipiedMats(double **M1, double **M2, int n)
{
    int i, j;
    double **ans;
    ans = allocateMem(n, n);
    if (ans == NULL)
    {
        return NULL;
    }
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            ans[i][j] = dotProduct(M1, M2, n, i, j);
        }
    }
    return ans;
}

double **findLnorm(double **W, double **sqrtD, int n)
{
    int i, j;
    double tmp;
    double **lnorm;
    double **tmpM1, **tmpM2;
    lnorm = getIdentityMat(n);
    if (lnorm == NULL)
    {
        return NULL;
    }
    tmpM1 = getMultipiedMats(sqrtD, W, n);
    if (tmpM1 == NULL)
    {
        freeLoop(lnorm, n);
        free(lnorm);
        return NULL;
    }
    tmpM2 = getMultipiedMats(tmpM1, sqrtD, n);
    if (tmpM2 == NULL)
    {
        freeLoop(lnorm, n);
        free(lnorm);
        return NULL;
    }
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            tmp = lnorm[i][j];
            lnorm[i][j] = tmp - tmpM2[i][j];
        }
    }
    freeLoop(tmpM1, n);
    free(tmpM1);
    freeLoop(tmpM2, n);
    free(tmpM2);
    return lnorm;
}

void maxIJ(double **Mat, int n, int *ans)
{
    int i, j;
    double tmpMax;
    tmpMax = -1;
    for (i = 0; i < n; i++)
    {
        for (j = i + 1; j < n; j++)
        {
            if (fabs(Mat[i][j]) > tmpMax)
            {
                ans[0] = i;
                ans[1] = j;
                tmpMax = fabs(Mat[i][j]);
            }
        }
    }
    return;
}

double getC(double t)
{
    double tmp;
    tmp = 1.0 + t * t;
    return 1.0 / sqrt(tmp);
}

double get_t(double theta)
{
    return ((double)signOf(theta)) / (fabs(theta) + sqrt(theta * theta + 1));
}

int signOf(double num)
{
    if (num < 0)
    {
        return -1;
    }
    return 1;
}

void updateP(int n, int iMax, int jMax, double c, double s, double **P)
{
    int i, j;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            if (i == j)
            {
                P[i][j] = 1;
            }
            else
            {
                P[i][j] = 0;
            }
        }
    }
    P[iMax][jMax] = s;
    P[jMax][iMax] = -s;
    P[iMax][iMax] = c;
    P[jMax][jMax] = c;

    return;
}

void printValuesAndVectors(double **A, double **V, int n){
    int i;
    double **eigenvalues = (double **)calloc(1, sizeof(double *));
    eigenvalues[0] = (double *)calloc(n, sizeof(double));
    for (i = 0; i < n; i++){
        eigenvalues[0][i] = A[i][i];
        if(fabs(eigenvalues[0][i])<0.00005){
            eigenvalues[0][i] = 0.0000;
        }
    }
    printMat(eigenvalues, 1, n);
    printMat(V, n, n);
    free(eigenvalues[0]);
    free(eigenvalues);
}

double ***jacobiAlg(double **A, int n, double ***VandA)
{
    int iMax, jMax, cnt = 0, IJMax[2] = {0, 0}, i, j;
    double **P, **V, **tmp, **PTrans, **oldA, **oldV;
    double theta, t, s, c, epsilon;
    int is_diag =0;
    epsilon = pow(10, -5);
    P = getIdentityMat(n);
    if (P == NULL)
    {
        RAEGen();
    }
    V = getIdentityMat(n);
    if (V == NULL)
    {
        freeLoop(P, n);
        free(P);
        RAEGen();
    }

    while (is_diag==0 && (cnt < 100)){
        maxIJ(A, n, IJMax); /*setting the indexes of tha max to IJMax*/
        iMax = IJMax[0];
        jMax = IJMax[1];
        if(!(A[iMax][jMax] == 0)){
            theta = (A[jMax][jMax] - A[iMax][iMax]) / (2.0 * A[iMax][jMax]);
            t = get_t(theta);
        }
        else{ /*Aijmax -> thetha is infinity-> theth'as sign is 1 as a convension*/
            t = 0; 
        }  
        c = getC(t);
        s = t * c;
        updateP(n, iMax, jMax, c, s, P);
        PTrans = allocateMem(n, n);
        if (PTrans == NULL){
            RAEGen();
        }

        for (i = 0; i < n; i++){
            for (j = 0; j < n; j++)
            {
                PTrans[i][j] = P[j][i];
            }
        }
        oldA = A;
        tmp = getMultipiedMats(PTrans, A, n);
        A = getMultipiedMats(tmp, P, n); /*A became A'*/
        is_diag = is_diagonal(oldA, A, n, epsilon);
        oldV = V;
        V = getMultipiedMats(V, P, n);
        freeLoop(oldA,n);
        free(oldA);
        freeLoop(oldV, n);
        free(oldV);
        freeLoop(tmp, n);
        free(tmp);
        freeLoop(PTrans, n);
        free(PTrans);
        cnt++;
    }

    VandA[0] = V;
    VandA[1] = A;
    freeLoop(P, n);
    free(P);
    return VandA;
}

int is_diagonal(double ** A, double ** A_tag, int n, double epsilon ){
    if(off(A,n)-off(A_tag,n) > epsilon){
        return 0;
    }
    return 1;
}
double off(double ** matrix,int n){
	int i, j;
    double num1, num2, sum_of_squares=0;

    for( i=0; i<n; ++i){
        for(j = 0; j < i; ++j){
            num1=matrix[i][j];
            num2=matrix[j][i];
            sum_of_squares+=(num1*num1) + (num2*num2);
        }
    }
    return sum_of_squares;
}


/** Sorting  an array of eigan values and eigen vectorrs structs by key eigenval  **/
void sortInplace(struct eigVA *arr, int n)
{
    int i, j;
    struct eigVA tmp;

    for (i = 0; i < n - 1; i++)
    {
        for (j = i + 1; j < n; j++)
        {
            if (arr[j].value < arr[i].value)
            {
                tmp = arr[i];
                arr[i] = arr[j];
                arr[j] = tmp;
            }
        }
    }
    return;
}
/*normalizing U to build T*/
double** normalize(double **U, int n, int k)
{
    int i, j;
    double ** T;
    double divisor=0;
    T = allocateMem(n,k);
    for (i = 0; i < n; ++i)
    {

        divisor = sqrt(sumOfSquares(U[i], k));
        for (j = 0; j < k; ++j)
        {
            if(divisor!=0){
                T[i][j] = U[i][j] / divisor;
            }
            else{
                T[i][j] = U[i][j];
            }     
        }
    }
    freeLoop(U,n);
    free(U);
    return T;
}

double sumOfSquares(double *arr, int len)
{
    int i;
    double ans = 0;
    for (i = 0; i < len; ++i)
    {
        ans += pow(arr[i], 2);
    }
    return ans;
}

int replaceK(double *eigenvalues, int n)
{
    int i;
    double curGap;
    double tmpGap = -1;
    int maxIndex = 1;
    for (i = 0; i < n / 2; ++i)
    {
        curGap = eigenvalues[i + 1] - eigenvalues[i];
        if (curGap > tmpGap)
        {
            tmpGap = curGap;
            maxIndex = i;
        }
    }
    return maxIndex + 1;
}

/**** KMEANS ****/
double *getFinalCentroids(double *centroids, double *allPoints, int k, int d, int points_cnt, int max_iter, double epsilone)
{
    int iter = 0;
    int *clusters = NULL;
    int stop = 0;
    int *movements = NULL;
    int *centroidofpoints = NULL; /*centroidofpoints[i] = cluster index of point i */
    int i;

    movements = (int *)malloc(k * sizeof(int)); /*init movements */
    if (movements == NULL)
    {
        free(allPoints);
        free(centroids);
        printf("An Error Has Occurred");
        /* somhow stop program -> was "return 1 before */
    }
    for (i = 0; i < k; i++)
    {
        movements[i] = 0;
    }

    centroidofpoints = (int *)malloc(points_cnt * sizeof(int)); /*init centroids of points*/
    if (centroidofpoints == NULL)
    {
        free(allPoints);
        free(centroids);
        free(movements);
        printf("An Error Has Occurred");
        /* somhow stop program -> was "return 1 before */
    }

    for (i = 0; i < points_cnt; i++)
    {
        centroidofpoints[i] = -1;
    }

    clusters = (int *)malloc(k * points_cnt * sizeof(int));
    if (clusters == NULL)
    {
        free(movements);
        free(centroidofpoints);
        free(allPoints);
        free(centroids);
        printf("An Error Has Occurred");
        /* somhow stop program -> was "return 1 before */
    }
    initClusters(points_cnt * k, clusters);

    /*
     * MAIN WHILE
     */

    while (iter < max_iter && stop == 0)
    {
        updateClusters(centroids, allPoints, centroidofpoints, d, points_cnt, clusters, k); /* update the centroid index to allpoints*/
        if (updateCentroids(clusters, allPoints, centroids, movements, k, d, points_cnt, epsilone) == 1)
        {
            free(movements);
            free(centroidofpoints);
            free(allPoints);
            free(centroids);
            printf("An Error Has Occurred");
            break;
        } /* finds the average of each cluster and updates center*/
        resetClusters(clusters, points_cnt, k);
        /*makes sure we've updated clusters back to k*n zeros)*/
        stop = checkMovement(movements, k);
        iter++;
    }

    /*free all memory*/
    free(movements);
    free(centroidofpoints);
    free(allPoints);
    free(clusters);
    return centroids;
}

/*HELPING FUNCTIONS IN K MEANS*/

void initClusters(int size, int *ans)
{
    int i;
    for (i = 0; i < size; i++)
    {
        ans[i] = -1;
    }
}

void resetClusters(int *clusters, int k, int n)
{
    int i;
    for (i = 0; i < n * k; i++)
    {
        clusters[i] = -1;
    }
}

int updateCentroids(int *clusters, double *allpoints, double *centroids, int *movements, int k, int d, int n, double epsilone)
{
    int points_in_cluster;
    double move;
    int curr_point;
    int curr_clust;
    int i;
    double *temp_cent = (double *)malloc(d * sizeof(double));
    if (temp_cent == NULL)
    {
        /* somhow stop program -> was "return 1 before */
    }
    for (i = 0; i < d; i++)
    {
        temp_cent[i] = 0;
    }

    for (curr_clust = 0; curr_clust < k; curr_clust++)
    { /*for each cluster*/
        points_in_cluster = 0;
        curr_point = clusters[curr_clust * n + points_in_cluster];
        while (curr_point != -1 && points_in_cluster < n)
        {
            sumPoints(allpoints, temp_cent, curr_point, d);
            points_in_cluster++;
            curr_point = clusters[curr_clust * n + points_in_cluster];
        }

        for (i = 0; i < d; i++)
        {
            temp_cent[i] = temp_cent[i] / points_in_cluster;
        }
        move = distance(temp_cent, centroids, 0, curr_clust * d, d);
        if (move > epsilone)
        {
            movements[curr_clust] = 0;
        }
        else
        {
            movements[curr_clust] = 1;
        }
        for (i = 0; i < d; i++)
        {
            centroids[curr_clust * d + i] = temp_cent[i];
            temp_cent[i] = 0;
        }
    }

    free(temp_cent);
    return 0;
}

void sumPoints(double *allpoints, double *temp_cent, int point, int d)
{
    int i;
    for (i = 0; i < d; i++)
    {
        temp_cent[i] += allpoints[point * d + i];
    }
}

void updateClusters(double *centroids, double *allpoints, int *CofP, int d, int n, int *clusters, int k)
{
    int cluster;
    int point;
    for (point = 0; point < n; point++)
    {
        cluster = getCentroid(allpoints, centroids, point * d, d, k);
        CofP[point] = cluster;
        addPointToCluster(clusters, point, n, cluster);
    }
}

void addPointToCluster(int *clusters, int point, int n, int cluster)
{
    int i;
    int curr_clust;
    curr_clust = cluster * n;
    for (i = 0; i < n; i++)
    {
        if (clusters[i + curr_clust] == -1)
        {
            clusters[i + curr_clust] = point;
            return;
        }
    }
}

int getCentroid(double *allpoints, double *centroids, int curr_point, int d, int k)
{
    int i;
    int tempClust = 0;
    double tempDis = -1;
    double dis = -1;
    for (i = 0; i < k; i++)
    {
        tempDis = distance(allpoints, centroids, curr_point, i * d, d);
        if (i == 0)
        {
            dis = tempDis;
        }
        if (dis > tempDis)
        {
            dis = tempDis;
            tempClust = i;
        }
    }
    return tempClust;
}

double distance(double *allpoints, double *centroids, int curr_point, int currCent, int d)
{
    double dis = 0;
    int i;
    for (i = 0; i < d; i++)
    {
        dis += pow((allpoints[curr_point + i] - centroids[currCent + i]), 2);
    }
    dis = pow(dis, 0.5);
    return dis;
}

int checkMovement(int *movements, int k)
{
    int i;
    for (i = 0; i < k; i++)
    {
        if (movements[i] == 0)
        {
            return 0;
        }
    }
    return 1;
}

/**finding n: based on the number of lines and finding d: based on the number of commas in a line **/
void findDims(FILE *f, int *ans)
{
    int n = 1, d = 1;
    char c;

    while ((c = getc(f)) != EOF)
    {
        if (n == 1)
        {
            if (c == ',')
            {
                d++;
            }
        }
        if (c == '\n')
        {
            n++;
        }
    }
    n -= 1;
    ans[0] = n;
    ans[1] = d;
    return;
}
