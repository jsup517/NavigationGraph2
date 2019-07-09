function P = RtoP(Rn, Rd, k, T, M, availableSet, current)
% Derive Prediction Matrix
testRd = (Rd>0).*Rd+(Rd==0);
testRn = (Rd>0).*Rn;%+(Rd==0);
testRn(M,M) = 1;
R = testRn./testRd;
P = zeros(k,T);
for ii=1:k
    if ii==1
        Ri{1} = R;
    else
        Ri{ii} = R*Ri{ii-1};
        sumRi = sum(Ri{ii});
%         for ri = 1:length(sumRi)
%             Ri{ii}(:,ri) = Ri{ii}(:,ri)/sumRi(ri);
%         end
        %Ri{ii} = Ri{ii}/norm(Ri{ii});
    end
    allView = hexToBinaryVector(availableSet,T);
    currentView = allView(current,:);
    distance = sum(currentView==allView,2);
    [val ind] = sort(distance,'descend');
    ind2 = min(M,3);
    if M>3 & M==currentView
        D(ii,1:M) = sum(Ri{ii}(:,ind(2:ind2)),2);
    else
        D(ii,1:M) = sum(Ri{ii}(:,ind(1)),2);
    end
    sum(D(ii,:));
    for mm=1:M
        tempM = allView(mm,:);
        for tt=1:T
            if tempM(tt) == 1
            %if 1
                P(ii,tt) = P(ii,tt) + D(ii,mm);
                %P(ii,tt) = 1;
            end
        end
    end
    P(ii,:) = P(ii,:)/sum(P(ii,:));
end
end