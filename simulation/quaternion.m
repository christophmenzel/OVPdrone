classdef quaternion
    properties
        a = 0;
        i = 0;
        j = 0;
        k = 0;
    end
    methods
        function obj = quaternion(varargin)
            if nargin == 1
                vals = varargin{1};
                obj.a = vals(1);
                obj.i = vals(2);
                obj.j = vals(3);
                obj.k = vals(4);
            elseif nargin == 2
                if length(varargin{1}) == 1
                    obj.a = varargin{1};
                    imagp = varargin{2};
                    obj.i = imagp(1);
                    obj.j = imagp(2);
                    obj.k = imagp(3);
                elseif length(varargin{1}) == 3
                    alpha = varargin{2};
                    imagp = varargin{1}*sin(alpha/2);
                    obj.i = imagp(1);
                    obj.j = imagp(2);
                    obj.k = imagp(3);
                    if norm(imagp) > 0
                        obj.a = cos(alpha/2);
                    else
                        obj.a = 1;
                    end
                end
            elseif nargin == 4
                obj.a = varargin{1};
                obj.i = varargin{2};
                obj.j = varargin{3};
                obj.k = varargin{4};
            end
        end
        function val = getVector(obj)
            val = [obj.a;obj.i;obj.j;obj.k];
        end
        function val = realPart(obj)
            val = obj.a;
        end
        function val = vectorPart(obj)
            val = [obj.i, obj.j, obj.k]';
        end
        function result = conjugated(obj)
            result = quaternion(obj.a,-obj.i,-obj.j,-obj.k);
        end
        function val = norm(obj)
            val = obj.a^2 + obj.i^2 + obj.j^2 + obj.k^2;
        end
        function val = abs(obj)
            val = sqrt(norm(obj));
        end
        function result = inv(obj)
            result = conjugated(obj);
            result.a = result.a / norm(obj);
            result.i = result.i / norm(obj);
            result.j = result.j / norm(obj);
            result.k = result.k / norm(obj);
        end
        function result = normalize(obj)
            result = quaternion( ...
                obj.a/abs(obj), ...
                obj.i/abs(obj), ...
                obj.j/abs(obj), ...
                obj.k/abs(obj));
        end
        function v2 = rotate(obj,v1)
            prod = qmultiply(obj,qmultiply(quaternion(0,v1),obj.conjugated()));
            v2 = prod.vectorPart();
        end
    end
end