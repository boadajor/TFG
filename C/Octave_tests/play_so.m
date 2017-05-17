% funcio equivalent a sound.m de Matlab
% play_so(x,Fm), x és vector mostres, Fm és freqüència de mostreig
function play_so(x,Fm)
    if nargin==1, Fm=48e3, elseif (nargin !=2), print_usage (), end
    file=[tmpnam(),'.wav'];% genera un arxiu en el directori temporal
    wavwrite(x,Fm,file);% a partir d'x genera un arxiu .wav
    system(['play ',file]);% ordena al sistema reproduir el .wav
    system(['rm ',file]); % suprimeix l'arxiu temporal
end