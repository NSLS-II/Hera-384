#include <cadef.h>
#include <ezca.h>
int main() 
{
int d, nelm; 
char s[384];
   ezcaGet("det1.NELM", ezcaLong, 1, &nelm);
   ezcaGet("det1.TSEN", ezcaByte, nelm, &s);
   for(d=0;d<nelm;d++){
     s[d]=0;
     }
  ezcaPut("det1.TSEN", ezcaByte, nelm, s);
  return(0);
} /* end main() */
