

function n = EE_calculateNormal( e1, e2, QA )

   if QA > 0
       n = cross3(e1,e2);
       n = n / norm(n);
   elseif QA < 0
       n = cross3(e2,e1);
       n = n / norm(n);
   else
       error('Case of parallel edges not yet implemented.'); 
   end

end



