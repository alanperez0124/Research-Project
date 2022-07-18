function[ ] = test_components()
    

    % Drone 1, Customer 11, Leaving and Returning
    anDrones(1).CustomerSet( 2).aanCust = [0 10; 0 9; 0 8; 0 7; 0 6; 0 5; 0 4; 0 3; 0 2; 0 1; 0 0; 10 9; 6 0];
    anDrones(1).CustomerSet( 1).aanCust = [0 10; 0 9; 6 0]; 
    anDrones(2).CustomerSet(12).aanCust = [0 10; 6 0]; 

%     anDrones(1).CustomerSet(1).aanCust
%     anDrones(1).CustomerSet(2).aanCust
%     
%     anDrones(2).CustomerSet(12).aanCust
    anDrones(2).CustomerSet(11).aanCust = [1 2 3 4];
    anDrones(2).CustomerSet(11).nCust = 11; 

    

    anDrones(2).CustomerSet( 4).aanCust = [12; 13; 14]; 
% 
%     anDrones(2).CustomerSet(7).aanCust
    anDrones(2).CustomerSet(12).aanCust

    solnIn.anPart1 = [0 10 9 8 7 3 5 6 0 ]; 
    solnIn.anPart2 = [11 1 -1 12 4 2]; 
    solnIn.anPart3 = [1 6 -1 2 3 6]; 
    solnIn.anPart4 = [3 7 -1 3 6 7]; 
    solnIn
    
    
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
    
    n = length(solnIn.anPart1) - 1; 
    
    iDroneCustomer = 1; 
    for iDrone = 1 : nDrones
        while iDroneCustomer < length(solnIn.anPart2) && solnIn.anPart2(iDroneCustomer) ~= -1           
            iRow = 1; 
            P_j(iDrone).Customer(solnIn.anPart2(iDroneCustomer)).aanCust = zeros(n*(n+1)/2, 2); % the number of possible permutations
            for iLeaving = 1 : length(solnIn.anPart1)
                for sReturning = iLeaving + 1 : length(solnIn.anPart1)
                    P_j(iDrone).Customer(solnIn.anPart2(iDroneCustomer)).aanCust(iRow, :) = ...
                        [solnIn.anPart1(iLeaving), solnIn.anPart1(sReturning)];
                    iRow = iRow + 1; 
                end
                
            end
            iDroneCustomer = iDroneCustomer + 1; 
        end
        iDroneCustomer = iDroneCustomer + 1; 
    end
    
    P_j(1).Customer(1).aanCust 
    P_j(2).Customer(12).aanCust 
    

    
     
    
end


