function [Gx ,Gy ,Gz] = Geo2ECEF(lat_d,lng_d,alt_m)
 % Function to convert Geodetic to ECEF coordinates
 
 
             aa = 6378137.0;
             bb = 6356752.314;             
             scl = 10^10;    % mm position accuracy reqs 10 decimal accuracy in radian lat and lng
             lat_r = round(scl*lat_d*pi/180)/scl;
             lng_r = round(scl*lng_d*pi/180)/scl;
             ff = (aa-bb)/aa;
             ee = sqrt((2-ff)*ff);
             
             R_M = (aa*(1-ee*ee))/((1-ee*ee*sin(lat_r)*sin(lat_r))^(3/2));
             R_N = aa/sqrt(1-ee*ee*sin(lat_r)*sin(lat_r));            
           
            
            lat_r = round(scl*lat_d*pi/180)/scl;
            lng_r = round(scl*lng_d*pi/180)/scl;
            
             Gx = (R_N + alt_m)*cos(lat_r)*cos(lng_r);
             Gy = (R_N + alt_m)*cos(lat_r)*sin(lng_r);
             Gz = (R_N*(1-ee*ee)+alt_m)*sin(lat_r);

            % round to nearest millimeter
            Gx = round(Gx*1000)/1000;
            Gy = round(Gy*1000)/1000;
            Gz = round(Gz*1000)/1000;
             


