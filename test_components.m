function[ ] = test_components()
    

    % Drone 1, Customer 11, Leaving and Returning
%     anDrones(1).CustomerSet( 2).aanCust = [0 10; 0 9; 0 8; 0 7; 0 6; 0 5; 0 4; 0 3; 0 2; 0 1; 0 0; 10 9; 6 0];
%     anDrones(1).CustomerSet( 1).aanCust = [0 10; 0 9; 6 0]; 
%     anDrones(2).CustomerSet(12).aanCust = [0 10; 6 0]; 
% 
%     anDrones(2).CustomerSet(11).aanCust = [1 2 3 4];
%     anDrones(2).CustomerSet(11).nCust = 11; 
%     anDrones(2).CustomerSet( 4).aanCust = [12; 13; 14]; 


    solnIn.anPart1 = [0 10 9 8 7 3 5 6 0 ]; 
    solnIn.anPart2 = [11 1 -1 12 4 2]; 
    solnIn.anPart3 = [1 6 -1 2 3 6]; 
    solnIn.anPart4 = [3 7 -1 3 6 7]; 

    C0.x = [ 0 -4 -7 -6 4 6 2 9 8 7 6 3 0 ];
    C0.y = [ 0 1 -3 -8 -9 -3 10 9 -2 -8 -8 3 0];
    aafDistances = calculateDistances(C0.x, C0.y);
    
    
    % Count the number of drones
    if isempty(solnIn.anPart2)
        nDrones = 0; 
    else
        nDrones = 1; 
        i = 1; 
        while i < length(solnIn.anPart2) 
            if solnIn.anPart2(i) == -1 
                nDrones = nDrones + 1; 
            end
            i = i + 1; 
        end
    end

    solnNew = solnIn; 
    
    % Remove all UAV flights from S_new
%     solnNew.anPart2 = []; 
%     solnNew.anPart3 = [];
%     solnNew.anPart4 = []; 
%     for iDrone = 1 : nDrones
%         solnNew.anPart2 = -1; 
%         solnNew.anPart3 = -1; 
%         solnNew.anPart4 = -1; 
%     end
    iCustomer = 1; 
    iDrone = 1; 
    while iCustomer <= length(solnNew.anPart2) && iDrone <= nDrones
        if solnNew.anPart2(iCustomer) == -1
            iDrone = iDrone + 1; 
        else
            solnNew.anPart3(iCustomer) = 0;
            solnNew.anPart4(iCustomer) = 0;
        end
        iCustomer = iCustomer + 1; 
    end

    

    % Create the P_j structure
    n = length(solnIn.anPart1) - 1; 
    
    iDroneCustomer = 1; 
    for iDrone = 1 : nDrones
        while iDroneCustomer < length(solnIn.anPart2) && solnIn.anPart2(iDroneCustomer) ~= -1           
            iRow = 1; 
%             P_j(iDrone).Customer(solnIn.anPart2(iDroneCustomer)).aanCust = zeros(n*(n+1)/2, 2); % the number of possible permutations
            for iLeaving = 1 : length(solnIn.anPart1) - 1  % subtract 1 because it drone can't leave from last spot
                for sReturning = iLeaving + 1 : length(solnIn.anPart1)
                    % Only add solution if it is feasible
                    if check_flight_validity(aafDistances, solnIn.anPart1(iLeaving), solnIn.anPart2(iDroneCustomer), solnIn.anPart1(sReturning))
%                         P_j(iDrone).Customer(solnIn.anPart2(iDroneCustomer)).aanCust(iRow, :) = ...
%                         [solnIn.anPart1(iLeaving), solnIn.anPart1(sReturning)];
                        P_j(iDrone).Customer(solnIn.anPart2(iDroneCustomer)).aanCust(iRow, :) = ...
                        [iLeaving, sReturning];
                        iRow = iRow + 1;
                    end
                    
                end
                
            end
            iDroneCustomer = iDroneCustomer + 1; 
        end
        iDroneCustomer = iDroneCustomer + 1; 
    end
    



    
    % Run algorithm
    for iteration = 1 : 10
        P_jCopy = P_j; 
        iDrone = 1; 
        bDone = 0; 
        iDroneCustomer = 1; 
        while iDrone < nDrones && bDone ~= 1
            while iDroneCustomer < length(solnNew.anPart2) && solnNew.anPart2(iDroneCustomer) ~= -1 && bDone ~= -1
                % Randomly pick (i, s) from P_c (if possible)
%                 fprintf("iDrone: %d\n", iDrone)
%                 fprintf("Customer: %d\n", solnNew.anPart2(iDroneCustomer))
%                 P_jCopy(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust
                anDimensions = size(P_jCopy(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust);

%                 solnNew
                if anDimensions(1) == 0
                    bDone = 1; 
                else
                    nRows = anDimensions(1); 
 
                    nRandRow = randi(nRows);
                    nRandi = P_jCopy(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust(nRandRow, 1);
                    nRands = P_jCopy(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust(nRandRow, 2);

                    
                    % Assign launch i and reconvene s locations to customer j
                    solnNew.anPart3(iDroneCustomer) = nRandi; 
                    solnNew.anPart4(iDroneCustomer) = nRands;
                    
%                     solnNew

%                     P_jCopy(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust
%                     P_jCopy(iDrone).Customer(1).aanCust
                    % Update P_jCopy according to the previously assigned flights to UAV_u
                    iTempRow = 1; 
                    for iRow = 1 : nRows
                        i = P_jCopy(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust(iRow, 1);
                        s = P_jCopy(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust(iRow, 2); 

                        if i < nRandi && s <= nRandi
                            P_jCopy2(iDrone).Customer(solnIn.anPart2(iDroneCustomer)).aanCust(iTempRow, :) = ...
                            [i, s];
                            iTempRow = iTempRow + 1; 
                        elseif i >= nRands && s > nRandi
                            P_jCopy2(iDrone).Customer(solnIn.anPart2(iDroneCustomer)).aanCust(iTempRow, :) = ...
                            [i, s];
                            iTempRow = iTempRow + 1; 
                        else
                            bOk = 0; 
                        end
                        
                        anDimensions = size(P_jCopy(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust);
                        if iRow == nRows && anDimensions(1) == 0
                            P_jCopy2 = P_jCopy; 
                        end

                    end
                    
                    % Actually update P_jCopy
                    P_jCopy(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust = P_jCopy2(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust;
%                     P_jCopy(iDrone).Customer(solnNew.anPart2(iDroneCustomer)).aanCust
%                     P_jCopy(iDrone).Customer(1).aanCust
%                         

                    % for each customer that drone iDrone is delivering to
                        % for each possible set i, s values for that drone
                            % if i < nRandi && s <= nRandi: bOk = 1; 

                            % elseif i >= nRands && s > nRandi: bOk = 1; 

                            % else bOk = 0
                    % Iterate iDroneCustomer
                    iDroneCustomer = iDroneCustomer + 1; 
                end
            end
            iDrone = iDrone + 1; 
        end
    end


end

function[ bValid ] = check_flight_validity( aafDistances, nLeaving, nVisiting, nReturning) 
    
    maxDistance = 10; 
    alpha = 1.5;
    bValid = 1; 
    
    % Calculate the distance from departure to the customer
    a = aafDistances(nLeaving+1, nVisiting+1); 

    % Calculate the distance from the customer to the returning
    b = aafDistances(nVisiting+1, nReturning+1); 

    % Total drone distance 
    fDroneDistance = (a+b)/alpha; 

    if fDroneDistance > maxDistance
        bValid = 0; 
    end
    

end


function[ aafDistances ] = calculateDistances( x, y )
% calculateDistances will calculate the distances between all the points 
%   on the graph
% INPUT
%   x, y   the x and y coordinates of the customers
% OUTPUT
%   aafDistances   An array of distances where D_ij is the distance between 
%                    house i and house j; this matrix should be symmetric

    % Variables
    %
    
    % Initialize the distances array
    aafDistances = zeros(length(x));
    
    % Calculate the distances
    for i = 1 : length(x)
        for j = 1 : length(x)
            aafDistances( i, j ) = sqrt( (x(i) - x(j))^2 + (y(i) - y(j))^2 );
        end
    end
    
end

