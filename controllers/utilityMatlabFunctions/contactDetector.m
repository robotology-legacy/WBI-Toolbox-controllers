function legsInContact = contactDetector(icubChair,state)
%#codegen
legsInContact = 0;

if (state < 3) && (icubChair == 1)
    
    legsInContact = 1;
    
end

end