function[CM] = compositebody_cm(V,xbar,ybar,zbar)
%{
Self, Justin
California Polytechnic State University, SLO

The purpose of this function is to determine the location of the
centroid of a composite body. 

OUTPUTS: xcm, ycm = (location of the CM of the composite body)
INPUTS:  V = vector containing part volumes
         xbar = vector: x-distance from the origin of PART centroid to
         point of interest
         ybar = vector: y-distance from the origin of PART centroid to
         point of interest
         zbar = vector: z-distance from the origin of PART centroid to
         point of interest
%}

% Input parameters (** USING FOR MASS MOMENT; NOT GEOMETRIC)

% Vixi, Aiyi, Aizi
for i = 1:length(V)
    Vixi(i) = V(i)*xbar(i);
    Viyi(i) = V(i)*ybar(i);
    Vizi(i) = V(i)*zbar(i);
end

% Finally, solving for (xbar,ybar) of COMPOSITE body
CM(1) = sum(Vixi) / sum(V);
CM(2) = sum(Viyi) / sum(V);
CM(3) = sum(Vizi) / sum(V);

end % end function