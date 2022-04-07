#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#define MAX_FILE_NAME 100
int main(void)
{
    FILE *fp, *fp2;
    int count = 0;
    int dim = 1;
    char filename[MAX_FILE_NAME];
    char c;
    char w;
    printf("Enter file name: \n");
    scanf("%s", filename);
    fp = fopen(filename, "r");
    fp2 = fopen(filename, "r");
    assert(fp!=NULL);

    for (c = getc(fp); c != EOF; c = getc(fp)){
        if(c == '\n')
            count+=1;
        
    }
        
    rewind(fp);    
    for (w = getc(fp);  w != '\n'; w = getc(fp)){
        if(w == ',')
           dim+=1;
    }
  rewind(fp);
    int i, j;
    double *matrix;
    double **cell;
    double point;
    char ch;
    matrix = (double *) calloc(count*dim, sizeof(double));

    cell = (double **)calloc(dim, sizeof(double));

    for(int i = 0; i < dim; i++)
    {
        cell[i] = matrix+i*dim;
   
    }

 i = 0, j = 0;

    while(fscanf(fp, "%lf%c", &point, &ch)!=EOF)
    {
    
        if(j == dim-1){
            matrix[i] = **cell;
            
            i++;
            j = 0;
        }
        
        else{
            cell[j] = &point;
            j++;
        }
    }

   for(int i = 0; i < count; i++)
   {
       for(int j = 0; j < dim; j++){
           printf("%.4f ,", matrix[i]);
       }
       printf("end of \n");
   }

   fclose(fp);
   fclose(fp2);
   printf("The file %s has %d lines and %d dim\n ", filename, count, dim);
}