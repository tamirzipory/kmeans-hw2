#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#define MAX_FILE_NAME 100
int main(void)
{
    FILE *fp;
    int count = 0;
    int dim = 1;
    char filename[MAX_FILE_NAME];
    double *matrix;
    double *row;
    double *all;
    double point;
    char ch;
    char c;
    char w;
    printf("Enter file name: \n");
    scanf("%s", filename);
    fp = fopen(filename, "r");
    assert(fp!=NULL);

    for (c = getc(fp); c != EOF; c = getc(fp))
    {
        if(c == '\n')
            count+=1;   
    }
        
    rewind(fp);    
    for (w = getc(fp);  w != '\n'; w = getc(fp)){
        if(w == ',')
           dim+=1;
    }

    int i, j;
    rewind(fp); 

    all = (double *) calloc(count*dim, sizeof(double));
    matrix = (double *) calloc(count*dim, sizeof(double)); // this is the matrix - every element point to arr of nums

    row = (double *)calloc(dim, sizeof(double)); // the arr that save cluster

    for(int i = 0; i < dim; i++)
    {
        row[i] = i*dim; // initialize 
    }

 i = 0, j = 0;

    while(fscanf(fp, "%lf%c", &point, &ch)!=EOF)
    {  
        if(j == dim-1)
        {    
            matrix[i] = *row;
            i++;
            j = 0;
        }
        else
        {
            row[j] = point;
            j++;
        }
    }
    i = 0;
    rewind(fp);
    while(fscanf(fp, "%lf%c", &point, &ch)!=EOF)
    {  
       all[j] = point;
       j++;
    }
    /*

   for(int i = 0; i < count; i++)
   {
       for(int j = 0; j < dim; j++)
       {
           printf("%.4f ,", matrix[i]);
       }
       printf("end of line \n");
   }
*/
   fclose(fp);
   printf("The file %s has %d lines and %d dim\n ", filename, count, dim);
/*
     for(int i = 0; i < count*dim; i++)
   {
           
           printf("%.4f ,", all[i]);
       
   }
*/
double * my_mat = (double *) malloc(dim*count * sizeof(double *));
for(int index = 0; index < dim *count; index++)
{
    my_mat[index] = all[index];
}
printf("here\n");
for(int i = 0; i < count; i++)
{
    for(int j = 0; j < dim; j++)
    {
        printf("%.4f ,", my_mat[i*dim+j]);
    }
       printf("\n");
}

}
