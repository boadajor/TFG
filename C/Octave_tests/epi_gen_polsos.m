%
% May 2017

clear,clc, close 'all',

for test=[0];

  switch test
    case 0,tone=[' 1 '];
	 t_p=750;t_t=750;leng=zeros(size(tone));leng(:)=t_t;leng(tone==" ")=t_p;leng=leng*1e-3;An=0;Ab=0;Ar=0;Ad=0;amp=(1-An/2-Ab-Ar-Ad)/2;
    case 1,tone=['    111    1111111    111     '];
	 t_p=100;t_t=100;leng=zeros(size(tone));leng(:)=t_t;leng(tone==" ")=t_p;leng=leng*1e-3;An=0;Ab=0;Ar=0;Ad=0;amp=(1-An/2-Ab-Ar-Ad)/2;
  end

% show parameters
test,amp,An,others=Ab+Ar+Ad

fs=48e3;
t=0:1/fs:sum(leng)-1/fs;

% DTMF synthesis
f_col=[697  770  852  941 425]';f_row=[1209 1336 1477 1633];

% Matrices
M=['1','2','3','A';
   '4','5','6','B';
   '7','8','9','C';
   '*','0','#','D';
   'b','r','d',' '];
F1=repmat(f_col,1,4);
F2=repmat(f_row,5,1);
A1=amp*ones(5,4);A1(5,4)=0;
A2=amp*ones(5,4);A2(5,:)=0;
leng2=[0 leng];

%% signal synthesis
x=0*t;
for i=1:length(tone)
    f1=F1(M==tone(i));
    f2=F2(M==tone(i));
    a1=A1(M==tone(i));
    a2=A2(M==tone(i));
    tin=sum(leng2(1:i));
    tfin=tin+leng(i);
    ti=t(t>=tin&t<tfin);
    x(t>=tin&t<tfin)=a1*cos(2*pi*f1*ti);%+a2*cos(2*pi*f2*ti);
end

%% noise synthesis
n=rand(size(x));n=n-mean(n);n=An*n;

%% others
% ring invitation to call (Ab)
b=Ab*cos(2*pi*F1(M=='b')*t);

% error (Ar)
d_r=200e-3;
d_r2=2*d_r;
T_r=1200e-3;N_r=round(T_r*fs);

ir=t<d_r|(t>d_r2&t<d_r2+d_r);in=1:length(t);ir=in(ir);
ir2=0:N_r:length(t)-N_r;Ir=repmat(ir,1,length(ir2))';
Ir2=repmat(ir2,length(ir),1);Ir2=Ir2(:);in=Ir+Ir2;
r=0*t;r(in)=Ar*cos(2*pi*F1(M=='b')*t(in));

% occupied (Ad)
d_r=200e-3;
d_r2=2*d_r;
T_r=4*d_r;N_r=round(T_r*fs);

ir=t<d_r|(t>d_r2&t<d_r2+d_r);in=1:length(t);ir=in(ir);
ir2=0:N_r:length(t)-N_r;Ir=repmat(ir,1,length(ir2))';
Ir2=repmat(ir2,length(ir),1);Ir2=Ir2(:);in=Ir+Ir2;
d=0*t;d(in)=Ad*cos(2*pi*F1(M=='b')*t(in));


%% addition of signal, noise and others
y=x+n+b+r+d;
y=y*1;
play_so(y,fs),pause

%name=['test',num2str(test),'.wav'];
%wavwrite(y,fs,name)

end
