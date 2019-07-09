clc; clear all; close all;

load('objectStates.mat');

num = length(objectState);
[users segments]=size(objectState{1});

for seg=1:segments
    for oo=1:num
        for uu=1:users
            Ob{uu}(oo,seg)=(objectState{oo}(uu,seg)>0);
        end
    end
end

% initialize Naviagation Graph
V = 0; numV = 1;
E = 0;

% update Navigation Graph
for uu=1:users
    priorStatus = 0;
    for seg=1:segments
        % check status
        currentStatus=bi2de(Ob{uu}(:,seg)');
        % update Navigation Graph
        if find(currentStatus==V)
            % found
            ind = find(currentStatus==V);
            ind2 = find(priorStatus==V);
            E(ind, ind2) = E(ind, ind2) + 1;
        else
            % not found
            V = [V; currentStatus];
            tempE = E;
            numV = numV + 1;
            E = zeros(numV,numV);
            E(1:end-1,1:end-1) = tempE;
            ind2 = find(priorStatus==V);
            E(end,ind2) = 1; 
        end
        priorStatus = currentStatus;
    end
end

% status analysis
for seg=1:segments
    for uu=1:users
        statusRecord(uu,seg)=bi2de(Ob{uu}(:,seg)');
    end
    a(seg,:) = hist(statusRecord(:,seg),sort(V,'ascend')); 
end

bar(a,'stacked')

% initialize Naviagation Graph
V2{1} = 0; 
E2{1} = 0;

% update Navigation Graph
for uu=1:users
    priorStatus2 = 0;
    ind4 = 1;
    V2{1} = bi2de(Ob{uu}(:,1)');
    E2{1} = 0;
    V2{2} = 0;
    E2{2} = 0;
    for seg=2:segments
        % check status
        currentStatus2{seg}=bi2de(Ob{uu}(:,seg)');
        % update Navigation Graph
        if find(currentStatus2{seg}==V2{seg})
            if uu==1
                V2{seg+1} = 0;
                E2{seg+1} = 0;
            end
            % found
            ind3 = find(currentStatus2{seg}==V2{seg});
            E2{seg}(ind3, ind4) = E2{seg}(ind3, ind4) + 1;
        else
            % not found
            V2{seg} = [V2{seg}; currentStatus2{seg}];
            tempE2 = E2{seg};
            numV2 = length(V2{seg});
            E2{seg} = zeros(numV2,length(V2{seg-1}));
            E2{seg}(1:end-1,1:end) = tempE2;
            E2{seg}(end,ind4) = 1; 
            if seg<segments
                if uu==1
                    V2{seg+1}=0;
                    E2{seg+1}=0;
                end
                tempE2 = E2{seg+1};
                numV2 = length(V2{seg+1});
                E2{seg+1} = zeros(numV2,length(V2{seg}));
                E2{seg+1}(1:end,1:end-1) = tempE2;
                %E2{seg+1}(end) = 0; 
            end
        end
        priorStatus2 = currentStatus2{seg};
        ind4 = find(priorStatus2==V2{seg});
    end
end