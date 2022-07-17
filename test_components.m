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

%     for i = 1 : 2 
%         anDrones(1).CustomerSet(i).aanCust
%     end

    
     
    
end


