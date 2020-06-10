th=zeros(384,20);
for i=1:20
   thresh(140+2*i);
   caput("det1.CNT",1)
   th(:,i)=caget("det1.INTENS");
endfor
