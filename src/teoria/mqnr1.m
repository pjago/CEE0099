%Gz = mqnr(y, u, T) nao recursivo
function Gz = mqnr1(y, u, T)    
    n = size(y, 1);
    for i = 1:size(y, 2)
        Y=[];
        fi=[];
        for k=2:n 
            y1 = y(k-1, i);
            u1 = at(u, k-1, i);
            Y  = [Y; y(k, i)];
            fi = [fi; -y1 u1];   
        end
        teta(:, i) = (fi'*fi)\fi'*Y;
    end
    
    uxi = (1:size(u, 2))';
    
    for i = 1:size(y, 2)
        a1=teta(1, i); 
        b1=teta(2, i); 
        
        z = tf('z', T, 'variable', 'z^-1');
        j = at(uxi, i);
        Gz(i, j) = b1/(1 + a1*z^-1);
        Gz(i, j).ioDelay = 1;
    end
end
