function thresh(val)
thr=zeros(1,12);
for i=1:12;
   thr(i)=val;
  endfor;
  caput("det1.THRSH",thr);
#  caput("det1.CNT",1)
#  a=caget("det1.INTENS");
#  plot(a);
endfunction
