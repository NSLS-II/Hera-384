#include <cadef.h>
#include <ezca.h>
int main() 
{
int d, i, nelm; 
char s[384];
   ezcaGet("det1.NELM", ezcaLong, 1, &nelm);
   ezcaGet("det1.TSEN", ezcaByte, nelm, &s);
      for(i=0;i<nelm;i++){
      printf("%i ",s[i]);
      }
   printf("\n\n");
   for(d=0;d<nelm;d++){
     s[d]=1;
     }
   for(i=0;i<nelm;i++){
      printf("%i ",s[i]);
      }
   printf("\n\n");
  ezcaPut("det1.TSEN", ezcaByte, nelm, s);
  return(0);
} /* end main() */
