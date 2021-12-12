%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xnew = rk4(diffeq,xold,input,dt)

h = dt;
hh = h/2;
h6 = h/6;

y = xold;
dydx = feval(diffeq,y,input);
yt = y + hh*dydx;

dyt = feval(diffeq,yt,input);
yt = y +hh*dyt;

dym = feval(diffeq,yt,input);
yt = y +h*dym;
dym = dyt + dym;

dyt = feval(diffeq,yt,input);
yout = y + h6*(dydx+dyt+2*dym);
xnew = yout;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end