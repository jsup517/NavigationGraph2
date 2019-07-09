function P = NGprediction(Node, Edge, k, T, currentView, segment)
% Derive Prediction Matrix
P = zeros(k,T);
currentNode = find(sum((Node{segment}==currentView),2)==length(currentView));

for ii=1:k
    if ii==1
        if ii+segment-1>length(Edge)
            Ri{1} = Edge{end};
        else
            Ri{1} = Edge{ii+segment-1};
        end
    else
        if ii+segment-1>length(Edge)
            Ri{ii} = Edge{end}*Ri{ii-1};
        else
            Ri{ii} = Edge{ii+segment-1}*Ri{ii-1};
        end
        sumRi = sum(Ri{ii});
%         for ri = 1:length(sumRi)
%             Ri{ii}(:,ri) = Ri{ii}(:,ri)/sumRi(ri);
%         end
        %Ri{ii} = Ri{ii}/norm(Ri{ii});
    end
    if size(Edge{segment},2)>=length(Node{segment})
        D = Ri{ii}(:,currentNode);
    else
        allNode = hexToBinaryVector(Node{segment},T);
        biNode = allNode(1:end-1,:);
        for tt=1:size(biNode,1)
            hdistance(tt) = sum(biNode(tt,:).*hexToBinaryVector(currentView,T));
        end
        %tempNodes = find(hdistance>0);
        [val tempNodes]=sort(hdistance,'descend');
        ra = max([1 floor(0.2*length(tempNodes))]);
        ra = min([k ra]);
        D = sum(Ri{ii}(:,tempNodes(1:ra)),2);
    end
    for tt=1:T
        for mm=1:length(D)
            if ii+segment>length(Node)
                allNode = hexToBinaryVector(Node{end},T);
                tempM = allNode(mm,:);
            else
                allNode = hexToBinaryVector(Node{ii+segment},T);
                tempM = allNode(mm,:);
            end
            if tempM(tt) == 1
            %if 1
                P(ii,tt) = P(ii,tt) + D(mm);
                %P(ii,tt) = 1;
            end
        end
    end
    P(ii,:) = P(ii,:)/sum(P(ii,:));
end
end