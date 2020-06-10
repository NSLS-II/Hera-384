#include <cadef.h>
#include <ezca.h>
int main() 
{
int i, d, nelm; 
char s[384];
   ezcaGet("det1.NELM", ezcaLong, 1, &nelm);
   ezcaGet("det1.TSEN", ezcaByte, nelm, &s);
   ezcaGet("det1.MONCH",ezcaLong,1,&d);
   printf("Channel: %i\n",d);
      for(i=0;i<nelm;i++){
      printf("%i ",s[i]);
      }
   printf("\n\n");
   s[d]=0;
   for(i=0;i<nelm;i++){
      printf("%i ",s[i]);
      }
  ezcaPut("det1.TSEN", ezcaByte, nelm, s);
  return(0);
} /* end main() */
