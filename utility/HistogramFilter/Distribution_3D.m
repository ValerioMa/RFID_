classdef Distribution_3D < handle
    %DISTRIBUTION This class represents a discrete distribution.
    
    properties
        start_
        stop_
        values
        v_size
    end
    
    methods
        function [] = wrap_angle(obj,n_discr)
           
           while obj.start_(3)>(n_discr-1)
               obj.start_(3) = obj.start_(3) - n_discr;
           end
           while obj.start_(3)<0
               obj.start_(3) = obj.start_(3) + n_discr;
           end
           
           if obj.v_size(3)>n_discr
              values_new = zeros(obj.v_size(1),obj.v_size(2),n_discr);
              
              for i=1:obj.v_size(3)
                 idx = rem(obj.start_(3)+i-1,n_discr)+1;
                 values_new(:,:,idx) = values_new(:,:,idx) + obj.values(:,:,i); 
              end
              obj.start_(3) = 0;
              obj.v_size(3) = n_discr;
              obj.stop_(3)  = n_discr - 1;
              obj.values = values_new;
           end           
        end    
            
        function obj = Distribution_3D(start_,values)
            %DISTRIBUTION Construct an instance of this class
            %   Detailed explanation goes here
% %             if not(sum(size(start_)==[3,1])==2)
% %                 error('Dimensioni del vettore start_ non corrette')
% %             end
            %             if not(numel(size(values))==3)
            %                error('Values have not the correct shape')
            %             end                                    
            obj.start_ = start_;
            obj.values = values;
            obj.stop_ = zeros(3,1);
            obj.update_stop();
            obj.normalize();
            
        end
        
        function [] = normalize(obj)
            %NORMALIZE Normalize the distribution weights
            s = sum(obj.values(:));
            if s~=0
                tmp = obj.values/s;
                obj.values = tmp;
            end
        end
        
        function [] = cut(obj,start,stop)
            %CUT start and stop 
            delta_s = start - obj.start_;  
            obj.v_size = delta_s + 1;
            if(delta_s(1)>0)
                obj.values = obj.values((delta_s(1)+1):end,1:end,1:end);
                obj.start_(1) = start(1);                
            end
            if(delta_s(2)>0)
                obj.values = obj.values(1:end,(delta_s(2)+1):end,1:end);
                obj.start_(2) = start(2);                
            end
            if(delta_s(3)>0)
                obj.values = obj.values(1:end,1:end,(delta_s(3)+1):end);
                obj.start_(3) = start(3);                
            end
            
            delta_s = obj.stop() - stop;
            if(delta_s(1)>0)
                obj.values = obj.values(1:(end-delta_s(1)),1:end,1:end);               
                obj.stop_(1) = stop(1); 
            end
            if(delta_s(2)>0)
                obj.values = obj.values(1:end,1:(end-delta_s(2)),1:end);
                obj.stop_(2) = stop(2); 
            end
            if(delta_s(3)>0)
                obj.values = obj.values(1:end,1:end,1:(end-delta_s(3)));
                obj.stop_(3) = stop(3); 
            end
        end
            
        
        function v=value(obj,index)
%             if not(sum(size(index)==[3,1])==2)
%                 error('Dimensioni del vettore index non corrette')
%             end
                
                        
            if index(1)<obj.start_(1) || index(2)<obj.start_(2) || index(3)<obj.start_(3) || index(1) > obj.stop_(1) || index(2) > obj.stop_(2) || index(3) > obj.stop_(3)
                v = 0;
            else
                index_n = index - obj.start_ +1;
                v = obj.values(index_n(1),index_n(2),index_n(3));
            end
        end
        
        function s=start(obj)
            s = obj.start_;
        end
        
        function [] = update_stop(obj)
            % Return the stop point of the distribution, which is the first index
            % 'outside' the distribution.
                        
            obj.v_size = size(obj.values)';
            if numel(obj.v_size)<3
                obj.v_size(3,1) = 1;
            end            
                        
            obj.stop_(1)  = obj.start_(1) + obj.v_size(1) -1;
            obj.stop_(2)  = obj.start_(2) + obj.v_size(2) -1;
            obj.stop_(3)  = obj.start_(3) + obj.v_size(3) -1;
        end
        
        function s = stop(obj)
           s =  obj.stop_;
        end
        
        function [] = show(obj,fig,idx)
            figure(fig);
            hold on;
            dim = size(obj.values);
            x = obj.start_(1) + (0:(dim(1)-1));
            y = obj.start_(2) + (0:(dim(2)-1));
            
            if exist('idx')==0
                surf(x,y,sum(obj.values(:,:,:),3)');
            else                
                surf(x,y,obj.values(:,:,idx)');
            end
        end
    end
    
    
    methods(Static)
        function p = unit_pulse(center)
            % UNIT_PULSE Returns a unit pulse at center.
            values = zeros(3,3,3);
            values(2,2,2) = 1;
            p =  Distribution_3D(round(center-1),values);
        end
        
        function d = triangle(center, half_width)
            % TRIANGLE Returns a triangular distribution. The peak is at 'center' and it is
            %zero at center +/- half_width. center and half_width are integers.
            w = round(half_width);
            c = round(center);
            values = zeros(2*w(1)+1,2*w(2)+1,2*w(3)+1);
            for x = (-w(1):1:w(1))
                for y = (-w(2):1:w(2))
                    for z = (-w(3):1:w(3))
                        p1 = 1-abs(x)/w(1);
                        p2 = 1-abs(y)/w(2);
                        p3 = 1-abs(z)/w(3);
                        values(x+w(1)+1,y+w(2)+1,z+w(3)+1) =  min([p1,p2,p3]);
                    end
                end
            end
            
            d =  Distribution_3D(c-w,values);
        end
        
        function d = gaussian(mu, sigma2, cut)
            % GAUSSIAN Returns a gaussian distribution, centered at mu, with variance
            %sigma**2. For efficiency reasons, the tails are cut at
            %cut * sigma, so with cut=5, it will fill the array from -5 sigma to +5 sigma.
            sigma = chol(sigma2);
            delta = round(sigma*cut + 1e-4); 
            
            delta_x = -delta(1):1:delta(1);
            delta_y = -delta(2):1:delta(2);
            delta_z = -delta(3):1:delta(3);
            
            wx = round(mu(1) + delta_x); 
            wy = round(mu(2) + delta_y);
            wz = round(mu(3) + delta_z);

            values = zeros(numel(wx),numel(wy),numel(wz));
            for x = wx
                x_indx = round(x - wx(1) + 1);
                for y = wy
                    y_indx = round(y - wy(1) + 1);
                    for z = wz
                        z_indx = round(z - wz(1) + 1);
                        d_value = [x-mu(1);y-mu(2);z-mu(3)];
                        values(x_indx,y_indx,z_indx) =  exp(-0.5*(d_value )'/sigma2*(d_value));
                    end
                end
            end
            d =  Distribution_3D([wx(1);wy(1);wz(1)],values);         
        end
        
        
        function d = sum(array_dist)
            %Returns the sum of all distributions (which is a list of Distribution
            %objects). If weights (a list) is specified, it must specify one float
            %value for each distribution

            sum_element = numel(array_dist);
            start =   ones(3,1)*Inf;
            stop  = - ones(3,1)*Inf;
            
            for i=1:sum_element
               start = min(array_dist{i}.start_,start); 
               stop = max(array_dist{i}.stop_,stop); 
            end
            
            range = stop'-start'+1;
            sum_dist = zeros(range);

% %             % SLOW VERSION
% %             for i=1:sum_element
% %                 for x=start(1):stop(1)
% %                     for y=start(2):stop(2)
% %                         for z=start(3):stop(3)
% %                             p = [x;y;z];
% %                             pv = p-start+1;                       
% %                             sum_dist(pv(1),pv(2),pv(3)) = sum_dist(pv(1),pv(2),pv(3)) + array_dist(i).value(p);
% %                        end
% %                     end                
% %                 end
% %             end
% %                         
            % FASTER VERSION
            for i=1:sum_element
                start_point = array_dist{i}.start_ - start; % positive number                
                stop_point = start_point + array_dist{i}.v_size;
                start_point  = start_point  + 1;
                x_range = start_point(1):stop_point(1);
                y_range = start_point(2):stop_point(2);
                z_range = start_point(3):stop_point(3);
                sum_dist(x_range,y_range,z_range) = sum_dist(x_range,y_range,z_range) + array_dist{i}.values;                
            end
            
            d = Distribution_3D(start,sum_dist);            
        end
        
        function c = multilpy(a, b)
            % Multiply two distributions and return the resulting distribution.
            start = max(a.start(),b.start());
            stop  = min(a.stop(),b.stop());
            delta = stop-start+1;
            values = zeros(delta(1),delta(2),delta(3));
            for x=start(1):stop(1)
                for y=start(2):stop(2)
                    for z=start(3):stop(3)
                       p = [x;y;z];
                       pv = p-start+1;
                       values(pv(1),pv(2),pv(3)) = a.value(p)*b.value(p); 
                    end                
                end
            end

            c = Distribution_3D(start,values);
            c.normalize()
        end
        
        
        function c = convolve(a, b)
            % Convolve distribution a and b and return the resulting new distribution.
            min_indx = a.start()+ b.start();
            max_indx = a.stop()+ b.stop();
            delta_indx = (max_indx-min_indx+1);

            values = zeros(delta_indx(1),delta_indx(2),delta_indx(3));
            a_start = a.start();
            a_stop = a.stop();
            b_start = b.start();
            b_stop = b.stop();
            
% %             % SLOW VERSION more secure
% %             for x=a_start(1):a_stop(1)
% %                 for y = a_start(2):a_stop(2)
% %                     for z = a_start(3):a_stop(3)                        
% %                         va = a.value([x;y;z]); 
% %                         for xb=b_start(1):b_stop(1)
% %                             x_index = x+xb-min_indx(1)+1;
% %                             for yb = b_start(2):b_stop(2)
% %                                 y_index = y+yb-min_indx(2)+1;
% %                                 for zb = b_start(3):b_stop(3)
% %                                     vb = b.value([xb;yb;zb]);
% %                                     z_index = z+zb-min_indx(3) + 1;                                     
% %                                     values(x_index, y_index, z_index) = values(x_index ,y_index,z_index) + va*vb;
% %                                 end                                
% %                             end                            
% %                         end
% %                     end
% %                 end
% %             end
            
            % FAST VERSION NO CHECK
            for x=1:(a_stop(1)-a_start(1)+1)
                for y = 1:(a_stop(2)-a_start(2)+1)
                    for z = 1:(a_stop(3)-a_start(3)+1)
                        va = a.values(x,y,z);    % fast version no check, use only if you know that x,y,z are correct value                        
                        for xb=1:(b_stop(1)-b_start(1)+1)
                            x_index = x+xb - 1;
                            for yb = 1:(b_stop(2)-b_start(2)+1)
                                y_index = y+yb - 1;
                                for zb = 1:(b_stop(3)-b_start(3)+1)
                                    vb = b.values(xb,yb,zb);    % fast version no check                                    
                                    z_index = z+zb - 1;                                     
                                    values(x_index, y_index, z_index) = values(x_index ,y_index,z_index) + va*vb;
                                end                                
                            end                            
                        end
                    end
                end
            end
            
            
            c = Distribution_3D(min_indx,values);
        end
    end
end

