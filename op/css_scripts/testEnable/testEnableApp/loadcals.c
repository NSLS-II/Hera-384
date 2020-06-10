#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cadef.h>
#include <ezca.h>
int main(int argc, char *argv[]) 
{
FILE *stream;
int i, nelm, nfile,nvals; 
char fname[128], c[100];
float tempslo, tempoffs;
float *slo, *offs;

    ezcaGet("det1.CALF",ezcaString, 1, &fname);
    printf("File name: %s\n",fname);
    ezcaGet("det1.NELM",ezcaLong, 1, &nelm);
    printf("nelm = %i\n",nelm);

    stream = fopen(fname, "r");
    if (stream == NULL) {
    	perror("fopen");
    	exit(EXIT_FAILURE);
    }
    for(i=0;i<4;i++){
       fgets(c, 100, stream);
       printf("Data from the file:\n%s", c);
       }



       char* token = strtok(c, " ");
       printf("token = %s\n",token);
       token=strtok(NULL," ");
       printf("token = %s\n",token);
       token=strtok(NULL," ");
       printf("token = %s\n",token);

    nvals=sscanf(token,"%i",&nfile);
    if(nvals!=1){
      printf("File does not give number of elements\n");
      fclose(stream);
      exit(0);
      }
    if(nfile!=nelm){
      printf("File has wrong number of elements for this detector!\n");
      fclose(stream);
      exit(0);
      }
    fscanf(stream, "%[^\n]", c); /* read number of columns line */
    slo = (float *)calloc(nelm,sizeof(float));
    offs = (float *)calloc(nelm,sizeof(float));
    
    for(i=0;i<nelm;i++){
      nvals=fscanf(stream,"%f %f",&tempslo, &tempoffs);
      slo[i]=tempslo;
      offs[i]=tempoffs;
      }  
    fclose(stream);
    printf("\n\n");

    ezcaPut("det1.SLP", ezcaFloat, nelm, slo);
    ezcaPut("det1.OFFS", ezcaFloat, nelm, offs);

    free(slo);
    free(offs);

  return(0);
} /* end main() */
