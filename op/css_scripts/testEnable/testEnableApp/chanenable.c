#include <cadef.h>
#include <ezca.h>
int printf();

int main() 
{
int d, i, nelm; 
char s[384];
   ezcaGet("det1.NELM",ezcaLong,1,&nelm);
   ezcaGet("det1.CHEN", ezcaByte, 384, &s);
   ezcaGet("det1.MONCH",ezcaByte,1,&d);
   printf("d=%i\n",d);
   for(i=0;i<nelm;i++){
      printf("%i ",s[i]);
      }
   printf("\n\n");
   s[d]=0;
   for(i=0;i<nelm;i++){
      printf("%i ",s[i]);
      }
   printf("\n\n");  
  ezcaPut("det1.CHEN", ezcaByte, nelm, s);
  return(0);
} /* end main() */
