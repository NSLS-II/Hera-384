#include <cadef.h>
#include <ezca.h>
int main() 
{
int d, nelm; 
int s[384];
   ezcaGet("det1.NELM", ezcaLong, 1, &nelm);
   ezcaGet("det1.CHEN", ezcaByte, 384, &s);
   for(d=0;d<nelm;d++){
     s[d]=0;
     }
  ezcaPut("det1.CHEN", ezcaByte, nelm, s);
  return(0);
} /* end main() */
