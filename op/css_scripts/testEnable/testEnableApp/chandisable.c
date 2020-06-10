#include <cadef.h>
#include <ezca.h>
int main() 
{
int d, nelm; 
char s[384];
   ezcaGet("det1.NELM",ezcaLong,1,&nelm);
   ezcaGet("det1.CHEN", ezcaByte, nelm, &s);
   ezcaGet("det1.MONCH",ezcaLong,1,&d);
   s[d]=1;
  ezcaPut("det1.CHEN", ezcaByte, nelm, s);
  return(0);
} /* end main() */
