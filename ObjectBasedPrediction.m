clear all; close all; clc;

% Performance Metric
predictionError= 0;
falsePositve = 0;

resX = 2560; % size of the video
resY = 1440;

bndBox

% Markov model for tile transitions
W = 12; H = 6; % number of tiles W=width, H=heigth
T = W*H; % total number of tiles
M = 1; % initial size of transition matrix
Puser=1; % meaningless
datasetSize = 48; % view trace size
testData = 5; % test data (training data : 43)
segmentDuration = 1; % sec
k = floor(5/segmentDuration); % client buffer length
userCount = 1; 

for video=0:2
    %figure(video+1); hold;
for user=1:datasetSize
    % Update the matrix while playing the video
    %[timeStamp frameID occupiedTiles pitch yaw]=readHeadMotion(['Rollercoaster-8lsB-P8nGSM_0_' num2str(user) '.txt'],W,H);
    [timeStamp occupiedTiles pitch yaw]=readHeadMotion3(['C:\Users\polar\OneDrive\PostDocUIUC\NavigationGraph\Infocom2020\Experiment_1\' num2str(user) '\video_' num2str(video) '.csv'],W,H);
    counter = 1; 
    currentView = binaryVectorToHex(occupiedTiles(1,:)); % read first segment's view
    
    %load('RollercoasterR.mat');
    counter3 = 1;
    for tran = 1:floor(max(timeStamp)/segmentDuration)
        priorView = currentView;
        tempView = zeros(1,T);
        counter2 = 0;
        while timeStamp(counter) < tran*segmentDuration % read all view within one segment duration
            counter2 = counter2 + 1;
            tempView = tempView | occupiedTiles(counter,:);
            counter = counter + 1;
        end
%         reshape(tempView, [H W])
%         reshape(hexToBinaryVector(binaryVectorToHex(tempView),T),[H W])
        % groungTruth : view traces from real dataset, not predicted
        % result, it is compared with prediction result
        groundTruth{video+1}{user}(tran,:) = tempView;
        currentView = binaryVectorToHex(tempView);
        %currentView = viewTransition(tran);
        tempView2 = reshape(tempView,[H W]);
        % dataset was upside down, so I changed it back
        for ll=1:H
            tempView3(ll,:) = tempView2(H-ll+1,:);
        end
        
        % Navigation Graph for Object Tracking
        % states : Combinations of Object index + 1
        % recording which objects are in the viewport in current segment
        numObjects = length(objects);
        for oo=1:numObjects
            if sum(objects{oo}(:,5)==tran)
                bndInd = find(objects{oo}(:,5)==tran);
                boundingBox = objects{oo}(bndInd,1:4);
                objectTiles = boxMap(boundingBox,W,H,resX,resY);
                objectState{oo}(user,tran) = sum(objectTiles.*occupiedTiles(tran,:));
            else
                objectState{oo}(user,tran) = 0;
            end
        end
        
        % Navigation Graph at the Server (cross-user view prediction)
        if user==1 
            Node{tran} = currentView;
            nNode(tran) = 1;
            Edge{tran} = 1;
        else
            currentNode = find(sum((Node{tran}==currentView),2)==length(currentView));
            if tran==1
                if currentNode
                    priorNode = currentNode;
                else
                    nNode(tran) = nNode(tran)+1;
                    Node{tran} = [Node{tran}; currentView];
                    priorNode = nNode(tran);
                end            
            else
                if currentNode
                    if size(Edge{tran-1},2)<nNode(tran-1)
                        Edge{tran-1}(currentNode,nNode(tran-1)) = 1;
                    else
                        Edge{tran-1}(currentNode,priorNode) = Edge{tran-1}(currentNode,priorNode)+1;
                    end
                    priorNode = currentNode;
                else
                    nNode(tran) = nNode(tran)+1;
                    Node{tran} = [Node{tran}; currentView];
                    if size(Edge{tran-1},2)<nNode(tran-1)
                        Edge{tran-1}(nNode(tran),nNode(tran-1)) = 1;
                    else
                        Edge{tran-1}(nNode(tran),priorNode) = 1;
                    end
                    priorNode = nNode(tran);
                end
            end
        end
        
        % Navigation Graph at the Clients (single-user prediction)
        if (tran == 1)% & (M<2)
            availableSet = currentView;
            M = 1;
            Rn = 1; Rd = 1;
        end
        tempCurrent = find(sum(currentView==availableSet,2)==length(currentView));
        if tempCurrent
            current = tempCurrent;
        else
            M = M + 1;
            tempRd = zeros(M); 
            for mm=1:M
                tempRd(mm,1:M-1) = Rd(1,:); 
            end
            Rd = tempRd;
            tempRn = diag(zeros(1,M)); tempRn(1:M-1,1:M-1) = Rn; Rn = tempRn;
            availableSet = [availableSet; currentView];
            current = M;
        end
        prior = find(sum(priorView==availableSet,2)==length(priorView));
        Rn(current,prior) = Rn(current,prior)+1;
        Rd(:,prior) = Rd(:,prior) + 1;
        R = Rn./Rd;
        %R = R/norm(R);

        % Translate R to Prediction Matrix (P)
        P = RtoP(Rn, Rd, k, T, M, availableSet, current);
        PSU{video+1}{user}(tran,:) = reshape(P, [1 T*k]);
        % Cross user prediction
        if user>datasetSize-testData && tran<floor(max(timeStamp)/segmentDuration)-k% && tran>1
            P2 = NGprediction(Node, Edge, k, T, currentView, tran);
            P3 = zeros(k,T);
            for latencyT=1:k
                tempInd = zeros(1,T);
                counterInd = counter;
                while timeStamp(counterInd) < (tran+latencyT)*segmentDuration
                    if timeStamp(counterInd) > (tran+latencyT-1)*segmentDuration
                        tempCounterInd = min([counterInd length(occupiedTiles)]);
                        tempInd = tempInd | occupiedTiles(tempCounterInd,:);
                    end
                    counterInd = counterInd + 1;
                    if counterInd>length(timeStamp)
                        break;
                    end
                end
                %ind = find(occupiedTiles(min([counter+latencyT length(occupiedTiles)]),:));
                ind = find(tempInd);
                tempPrecision = zeros(1,T); tempPrecision(ind) = 1/length(ind);
                precision2(tran,latencyT) = sum(min(P2(latencyT,:)/sum(P2(latencyT,:)),tempPrecision));
                predictionError2(tran,latencyT) = sum(0==P2(latencyT,ind))/sum(ind); % falseNegative

                ent1 = entropy(P(latencyT,:));
                ent2 = entropy(P2(latencyT,:));
                tempEnt1 = find(P(latencyT,:)>0);
                tempEnt2 = find(P2(latencyT,:)>0);
                ent1 = ent(P(latencyT,:));
                ent2 = ent(P2(latencyT,:));
                if ent1>ent2
                    P3(latencyT,:) = P2(latencyT,:);
                else
                    P3(latencyT,:) = P(latencyT,:);
                end
                precision3(tran,latencyT) = sum(min(P3(latencyT,:)/sum(P3(latencyT,:)),tempPrecision));
                predictionError3(tran,latencyT) = sum(0==P3(latencyT,ind))/sum(ind); % falseNegative                
                % Draw Ground truth and predictions
%                 indDraw = reshape(tempInd, [H W]);
%                 pDraw1 = reshape(P(latencyT,:), [H W]);
%                 pDraw2 = reshape(P1(latencyT,:), [H W]);
%                 pDraw3 = reshape(P3(latencyT,:), [H W]);
%                 subplot(4,5,1+(latencyT-1)*4); image(1000*indDraw); subplot(4,5,2+(latencyT-1)*4); image(1000*pDraw1); 
%                 subplot(4,5,3+(latencyT-1)*4); image(1000*pDraw2); subplot(4,5,4+(latencyT-1)*4); image(1000*pDraw3);
            end
            %drawnow;
            if counter3 == 1
                meanPredictionError2 = predictionError2(1:tran,:);    
                meanPrecision2 = precision2(1:tran,:);
                meanPredictionError3 = predictionError3(1:tran,:);    
                meanPrecision3 = precision3(1:tran,:);               
                %plot3(1:length(pitch),pitch,yaw); drawnow;
            else
                meanPredictionError2 = [meanPredictionError2; predictionError2(1:tran,:)];    
                meanPrecision2 = [meanPrecision2; precision2(1:tran,:)];
                mean(meanPrecision2)
                meanPredictionError3 = [meanPredictionError3; predictionError3(1:tran,:)];    
                meanPrecision3 = [meanPrecision3; precision3(1:tran,:)];                
            end
%             meanPrecision2 = [meanPrecision2 precision2(tran,1:k)];
%             meanPredictionError2 = [meanPredictionError2 predictionError2(tran,1:k)];
            counter3 = counter3 + 1;
            %figure(3); fig3 = bar3([hexToBinaryVector(availableSet(current),T); tempP]'); axis([0 k+1 0 T+1 0 1]);
            PCU{video+1}{user}(tran,:) = reshape(P2, [1 T*k]);
            PSUCU{video+1}{user}(tran,:) = reshape(P3, [1 T*k]);
        end
        tempP = zeros(k,T);        
        %latencyT = floor(latency/segmentDuration)+1;
        %image(100*tempView3); scatter(0.5*W*(yaw(counter-counter2:counter)+1)+0.5,0.5*H*(pitch(counter-counter2:counter)+1)+0.5,'ro');
        %drawnow;     
        
        % Single user prediction
        for latencyT=1:k
            tempInd = zeros(1,T);
            counterInd = counter;
            while timeStamp(counterInd) < (tran+latencyT)*segmentDuration
                if timeStamp(counterInd) > (tran+latencyT-1)*segmentDuration
                    tempCounterInd = min([counterInd length(occupiedTiles)]);
                    tempInd = tempInd | occupiedTiles(tempCounterInd,:);
                end
                counterInd = counterInd + 1;
                if counterInd>length(timeStamp)
                    break;
                end
            end
            %ind = find(occupiedTiles(min([counter+latencyT length(occupiedTiles)]),:));
            ind = find(tempInd);     
            tempP(latencyT,ind) = 1;
            ind2 = find(occupiedTiles(min([counter+latencyT length(occupiedTiles)]),:)==0);
            predictionError(tran,latencyT) = sum(0==P(latencyT,ind))/sum(ind); % falseNegative
            %predictionError2(tran,latencyT) = sum(0==P2(latencyT,ind))/sum(ind); % falseNegative
            falsePositive(tran,latencyT) = sum(P(latencyT,ind2));
            truePositive(tran,latencyT) = sum(P(latencyT,ind));
            trueNegative(tran,latencyT) = sum(0==P(latencyT,ind2));
           
            %precision(tran,latencyT) = truePositive(tran,latencyT)/(truePositive(tran,latencyT)+trueNegative(tran,latencyT));
            tempPrecision = zeros(1,T); tempPrecision(ind) = 1/length(ind);
            precision(tran,latencyT) = sum(min(P(latencyT,:)/sum(P(latencyT,:)),tempPrecision));
            recall(tran,latencyT) = truePositive(tran,latencyT)/(truePositive(tran,latencyT)+predictionError(tran,latencyT));
            f1(tran,latencyT) = 2*precision(tran,latencyT)*recall(tran,latencyT)/(precision(tran,latencyT)+recall(tran,latencyT));
        end
%         figure(1); fig1 = bar3([hexToBinaryVector(availableSet(current),T); P]'); axis([0 k+1 0 T+1 0 1]);
%         set(fig1(1), 'facecolor', 'r'); 
%         figure(2); fig2 = bar3([hexToBinaryVector(availableSet(current),T); tempP]'); axis([0 k+1 0 T+1 0 1]);
%         drawnow;
        %figure(3); plot(1:tran,predictionError,1:tran,falsePositive,1:tran,truePositive); legend('Prediction Error','False Positive','True Positive');
    %     drawnow; 
        % pause(segmentDuration);
    end
%    figure(user); plot(segmentDuration*(1:k),mean(predictionError),segmentDuration*(1:k),mean(predictionError2),segmentDuration*(1:k),mean(precision),segmentDuration*(1:k),mean(precision2));
%    legend('Prediction Error','Prediction Error2','Precision','Precision2'); xlabel('latency (sec)');
    %save('RollercoasterR.mat','Rn','Rd','availableSet','M');
    if user==1 
        meanPredictionError = predictionError(1:tran,:);    
        meanPrecision = precision(1:tran,:);
    else
        meanPredictionError = [meanPredictionError; predictionError(1:tran,:)];    
        meanPrecision = [meanPrecision; precision(1:tran,:)];
    end
    mean(meanPrecision)
    %meanPredictionError2(1:tran,:,user) = (predictionError2(1:tran,:));
    meanFalsePositive(1:tran,:,userCount) = (falsePositive(1:tran,:));
    meanTruePositive(1:tran,:,userCount) = (truePositive(1:tran,:));

    %meanPrecision2(:,:,user) = (precision2(:,:));
    meanRecall(1:tran,:,userCount) = (recall(1:tran,:));
    meanF1(1:tran,:,userCount) = (f1(1:tran,:));
    if user>datasetSize-testData
        Puser = Puser + 1;
    end
    user
    userCount = userCount+1;
end
end
figure(user+1); grid on;
plot((1:k)*segmentDuration,mean(mean(meanPredictionError,3)),(1:k)*segmentDuration,mean(mean(meanPredictionError2,3)),...
    (1:k)*segmentDuration,mean(mean(meanPredictionError3,3)),(1:k)*segmentDuration,mean(mean(meanPrecision,3)),...
    (1:k)*segmentDuration,mean(mean(meanPrecision2,3)),(1:k)*segmentDuration,mean(mean(meanPrecision3,3)),'LineWidth',2);
legend('Prediction Error - SU','Prediction Error - CU','Prediction Error = SU+CU','Precision - SU','Precision - CU','Precision - SU+CU'); xlabel('latency (sec)');
% figure(user+2); plot(1:tran,mean(mean(meanPredictionError,3),2),1:tran,mean(mean(meanFalsePositive,3),2),1:tran,mean(mean(meanTruePositive,3),2), ...
%     1:tran,mean(mean(meanPrecision,3),2),1:tran,mean(mean(meanRecall,3),2),1:tran,mean(mean(meanF1,3),2));
% legend('Prediction Error','False Positive','True Positive','Precision','Recall','F1'); xlabel('Segments (sec)');

figure(user+3); hold; grid on; pp = 1;
for ll=1:k
    %precision1=reshape((meanPrecision(:,ll,:)),[1 tran*(userCount-1)]);
    precision1 = meanPrecision(:,ll);
    [a1 b1] = hist(precision1,1000);
    for ii=1:length(a1)
        dd{ll}(ii) =sum(a1(1:ii));
    end
    if ll*segmentDuration == pp
        plot(b1,dd{ll}/dd{ll}(end),'LineWidth',2)
        pp = pp + 1;
    end
end
legend('1 sec','2 sec','3 sec','4 sec','5 sec'); xlabel('Predict Precision'); ylabel('CDF');
figure(user+4); hold; grid on; pp = 1;
for ll=1:k
    %[x y z] = size(meanPrecision2);
    %precision3=reshape((meanPrecision2(:,ll,:)),[1 x*z]);
    precision22 = meanPrecision2(:,ll);
    [a2 b2] = hist(precision22,1000);
    for ii=1:length(a2)
        ee{ll}(ii) =sum(a2(1:ii));
    end
    if ll*segmentDuration == pp
        plot(b2,ee{ll}/ee{ll}(end),'LineWidth',2)
        pp = pp + 1;
    end
end
legend('1 sec','2 sec','3 sec','4 sec','5 sec'); xlabel('Predict Precision'); ylabel('CDF');
figure(user+5); hold; grid on; pp = 1;
for ll=1:k
    %[x y z] = size(meanPrecision2);
    %precision3=reshape((meanPrecision2(:,ll,:)),[1 x*z]);
    precision33 = meanPrecision3(:,ll);
    [a3 b3] = hist(precision33,1000);
    for ii=1:length(a3)
        ff{ll}(ii) =sum(a3(1:ii));
    end
    if ll*segmentDuration == pp
        plot(b3,ff{ll}/ff{ll}(end),'LineWidth',2)
        pp = pp + 1;
    end
end
legend('1 sec','2 sec','3 sec','4 sec','5 sec'); xlabel('Predict Precision'); ylabel('CDF');
