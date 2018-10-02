classdef Nameable < matlab.mixin.Heterogeneous & matlab.mixin.Copyable
    properties (SetAccess = protected)
        Name
    end

    methods
        function set.Name(this, name)
            assert(ischar(name), 'MotorProto:Component:InvalidDataType', 'The name property must be a string');
            this.Name = name;
        end
    end
    
    methods (Sealed)          
        function I = isNamed(this,nameIn)
            %% Returns a logical array with a true value in the position where
            %   the element has the given name
            
            %% Input Validation
            assert(ischar(nameIn)|isCharacterCell(nameIn),...
                    'MotorProto:NamedElementArray:invalidType',...
                    'The Name property must be a string');
            
            %% Code Body
            if ischar(nameIn)
                I = strcmp({this.Name},nameIn);
            else
                nIn      = numel(nameIn);
                nThis    = numel(this);
                I        = false(nThis,nIn);
                thisName = {this.Name};
                for iThis = 1:nThis
                    I(iThis,:) = strcmp(nameIn.',thisName(iThis));
                end
            end
        end
        
        function I = hasName(this,namesIn)
            %% Returns a logical array indicating true if the element has one
            %   of the specified names
            
            if ischar(namesIn)
                I = strcmp({this.Name},namesIn);
            elseif isCharacterCell(namesIn)
                namesIn    = reshape(namesIn,1,[]);
                nNamesIn   = numel(namesIn);
                
                mapNames   = {this.Name};
                mapNames   = reshape(mapNames,[],1);
                nNamesThis = numel(mapNames);
                
                namesIn    = repmat(namesIn,nNamesThis,1);
                mapNames   = repmat(mapNames,1,nNamesIn);
                
                I          = strcmp(mapNames,namesIn);
                I          = any(I,2);
            else
                error('MotorProto:NamedElementArray',...
                        ['The input argument must be a character array or',...
                        'a cell array of strings']);
            end
        end
        
        function objectOut = getElementNamed(this,nameIn)
            %% Input Validation
            assert(ischar(nameIn),...
                    'MotorProto:NamedElementArray:invalidType',...
                    'The Name property must be a string');
            
            %% Code Body 
            objectOut = this(this.isNamed(nameIn));
        end
        
        function this = rename(this,oldName,newName)
            validateattributes(oldName,{'char'},{'row'});
            validateattributes(newName,{'char'},{'row'});
            
            hasOldName = this.isNamed(oldName);
            hasNewName = this.isNamed(newName);
            
            if ~any(hasNewName)
                if any(hasOldName)
                    this(hasOldName).Name = newName;
                else
                    warning('MotorProto:NamedElementArray',...
                            'No element named %s exists.',...
                            oldName);
                end
            else
                error('MotorProto:NamedElementArray',...
                        'An element named %s already exists.',...
                        newName);
            end
        end
        
        function [this,newNames] = add(this,elementIn)
            %% Input Validation
            assert(isa(elementIn,'Component'),...
                    'MotorProto:NamedElementArray:InvalidType',...
                    'The elementIn argument must be a NamedElementArray object');

            assert(~any(cellfun('isempty',{elementIn.Name})),...
                    'MotorProto:NamedElementArray:EmptyName',...
                    'The elementIn arguments must have a nonempty Name property');
            
            %%Test for handle equality
            nThis  = numel(this);
            isSame = false(size(elementIn));
            for iThis = 1:nThis
                isSame = isSame | (elementIn == this(iThis));
            end
            
            if any(isSame)
                warning('MotorProto:NamedElementArray:DupHandleAdd',...
                        'Duplicate elements will not be added to the array');
            end
            
            %%Rename elements with duplicate names
            this     = makeNamesUnique([this,elementIn(~isSame)]);
            newNames = {this.Name};
            newNames = newNames((nThis+1):end);
        end

        function [this,isRemove] = remove(this,nameIn)
            isRemove = this.hasName(nameIn);
            if all(isRemove);
                this = this.empty(0,1);
            else
                this(isRemove) = [];
%                 nThis         = sum(~isRemove);
%                 for iThis = 1:nThis
%                     this(iThis).Index = iThis;
%                 end
            end
        end

        function I = ne(this1,this2)
            I = ne@handle(this1,this2);
        end
        
        function I = eq(this1,this2)
            I = eq@handle(this1,this2);
        end
    end
    
   	methods (Sealed,Access = protected)
        function this = makeNamesUnique(this)
            [~,iUnique]    = unique(this);
            nThis          = numel(this);
            iCopy          = (1:nThis);
            iCopy(iUnique) = [];
            this(iCopy)    = copy(this(iCopy));
            
            names          = {this.Name};
            names          = unique(names);
            nNames         = numel(names);
            nameCount      = zeros(nNames,1);
            
            if nNames ~= nThis
                for iThis = 1:nThis
                    iName = strcmp(this(iThis).Name,names);
                    if nameCount(iName) ~= 0
                        newName = [names{iName},'_',num2str(nameCount(iName))];
                        while any(strcmp(newName,names))
                            nameCount(iName) = nameCount(iName) + 1;
                            newName = [names{iName},'_',num2str(nameCount(iName))];
                        end
                        nameCount(iName) = nameCount(iName) + 1;
                        this(iThis).Name = newName;
                    else
                        nameCount(iName) = 1;
                    end
                end
            end
        end
    end
    
    methods (Static)
        function componentOut = newComponent(componentType,varargin)
            componentOut = eval(componentType);
            if isa(componentOut,'Component')
                componentOut = componentOut.newComponent(varargin{:});
            else
                error(  'MotorProto:Component:invalidObjectType',...
                        '%s is not a recognized Component subclass',...
                            componentType);
            end
        end
    end
end