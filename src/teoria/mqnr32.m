%Gz = mqnr(y, u, T) nao recursivo
function Gz = mqnr32(y, u, T)   
    n = size(y, 1);
    for i = 1:size(y, 2)
        Y=[];
        fi=[];
        for k=4:n 
            y1 = y(k-1, i);
            y2 = y(k-2, i);
            u2 = at(u, k-2, i); 
            u3 = at(u, k-3, i);  
            Y  = [Y; y(k, i)];
            fi = [fi; -y1 -y2 u2 u3];   
        end
        teta(:, i) = (fi'*fi)\fi'*Y;
    end
    
    uxi = (1:size(u, 2))';
    
    for i = 1:size(y, 2)
        a1=teta(1, i); 
        a2=teta(2, i); 
        b2=teta(3, i); 
        b3=teta(4, i);

        z = tf('z', T, 'variable', 'z^-1');
        j = at(uxi, i);
        Gz(i, j) = (b2 + b3*z^-1)/(1 + a1*z^-1 + a2*z^-2);
        Gz(i, j).ioDelay = 2;
    end
end

