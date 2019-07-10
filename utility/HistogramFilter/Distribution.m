classdef Distribution < handle
    %DISTRIBUTION This class represents a discrete distribution.
    
    properties
        offset
        values
    end
    
    methods
        function obj = Distribution(offset,values)
            %DISTRIBUTION Construct an instance of this class
            %   Detailed explanation goes here
            obj.offset = offset;
            obj.values = values;
        end
        
        function [] = normalize(obj)
            %NORMALIZE Normalize the distribution weights
            s = sum(obj.values(:));
            if s~=0
                tmp = obj.values/s;
                obj.values = tmp;
            end
        end
        
        function v=value(obj,index)
            % Retrieve the value of the distribution at index
            index = index(:) - obj.offset(:) +1;            
            
            if sum(index<0)>0 || sum(index> size(obj.values))>0
                % Index is outside the bound
                v = 0;
            else
                cell_index = num2cell(index);
                v = obj.values(cell_index{:});
            end
        end
        
        function s=start(obj)
           s = obj.offset;
        end
        
        function s=stop(obj)
           % Return the stop point of the distribution, which is the first index
           % 'outside' the distribution. 
           s =  obj.offset + size(obj.values) -1;
        end
        
        function [] = show(obj,style,fig,tag)
           figure(fig);
           hold on;
           x = obj.offset + (-1:length(obj.values));
           plot(x,[0,obj.values,0],style,'Tag',tag);
        end
    end
    
    
    methods(Static)
        function p = unit_pulse(center)
            % UNIT_PULSE Returns a unit pulse at center
            cell_index = num2cell(ones(length(center)));
            p =  Distribution(round(center),ones(cell_index{:}));
        end
        
%         function d = triangle(center, half_width)
%             % TRIANGLE Returns a triangular distribution. The peak is at 'center' and it is
%             %zero at center +/- half_width. center and half_width are integers.
%             w = round(half_width);
%             c = round(center);
%             values = [];
%             for i = (-w:1:0)
%                 values(end+1) = w + i;
%             end
%             for i = (1:1:w)
%                 values(end+1) = w - i;
%             end
%             d =  Distribution(c-w,values);
%             d.normalize()
%         end
        
        function d = gaussian(mu, sigma2, cut)
            % GAUSSIAN Returns a gaussian distribution, centered at mu, with variance
            %sigma**2. For efficiency reasons, the tails are cut at
            %cut * sigma, so with cut=5, it will fill the array from -5 sigma to +5 sigma.
            sigma = chol(sigma2);
            extent = ceil(sigma*cut(:))';
            
            start = mu - extent;
            point = start;
            stop = mu + extent;
            
            matrix_size = num2cell(extent*2+1);
            values = zeros(matrix_size{:});
            while sum((stop - point)>0)>0
                position = num2cell(point-start+1);                
                values(position{:}) = exp(-0.5*(point-mu)/sigma2*(point-mu)');
                i = 0;                                
                while((point(end-i) + 1)>stop(end-i))
                    point(end-i) = start(end-i);                   
                    i = i +1;                    
                end                
                point(end-i) = point(end-i) + 1;                
            end
            d =  Distribution(mu-extent,values);
            d.normalize()
        end
        
        
        function d = sum(distributions, weights)
           %Returns the sum of all distributions (which is a list of Distribution
           %objects). If weights (a list) is specified, it must specify one float
           %value for each distribution   
           if isempty(weights)
               weights = ones(length(distributions),1);
           end
           
           start = Inf;
           stop = -Inf;
           for i=1:length(distributions)
               start = min(distributions(i).start,start);
               stop  = max(distributions(i).stop,stop);
           end
           sum_dist = zeros(stop-start,1);
           
           
           for i=1:length(distributions)
              dist = distributions(i);
              
              % Weight all values and att them to sum_dist
              for j = len(dist.values)
                 sum_dist(dist.start()-start+j) = sum_dist(dist.start()-start+j) + dist.values(j) * weights(i); 
              end               
           end
           
           d = Distribution(start,sum_dist);
           d.normalize()
        end
        
        function c = multilpy(a, b)
        % Multiply two distributions and return the resulting distribution.
            start = max(a.start(),b.start());
            stop  = min(a.stop(),b.stop());
            
            values = [];
            for i=start:stop
                values(end+1) = a.value(i)*b.value(i);                
            end
            
            c = Distribution(start,values);
            c.normalize()            
        end
        
        
        function c = convolve(a, b)
        % Convolve distribution a and b and return the resulting new distribution.
            min_indx = a.start()+ b.start();
            max_indx = a.stop()+ b.stop();
            delta_indx = (max_indx-min_indx+1);
            values = zeros(1,delta_indx);
            a.start()
            a.stop()
            for i=a.start():a.stop()
                va = a.value(i);
                for j=b.start():b.stop()
                    vb = b.value(j);
                    values(i+j-min_indx+1) = values(i+j-min_indx+1) + va*vb;
                end
            end
            
            c = Distribution(min_indx,values);
            c.normalize()            
        end
    end
end

